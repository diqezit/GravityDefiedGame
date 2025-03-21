using System;
using System.Collections.Generic;
using System.Threading;
using System.Windows;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Utilities.GameConstants;
using static GravityDefiedGame.Utilities.GameConstants.Debug;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;
using static System.Math;

namespace GravityDefiedGame.Models
{
    [Flags]
    public enum BikeState
    {
        None = 0,
        InAir = 1,
        InWheelie = 2,
        MovingBackward = 4,
        Crashed = 8,
        InStoppie = 16
    }

    public class ForcesComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public ForcesComponent(Motorcycle bike, BikePhysics physics) : base(string.Empty)
        {
            _bike = bike;
            _physics = physics;
        }

        public Vector CalculateThrustForce()
        {
            double engineForce = _bike.Throttle * _physics.EnginePower;
            var (cosAngle, sinAngle) = GetTrigsFromAngle(_bike.Angle);
            return new Vector(cosAngle * engineForce, sinAngle * engineForce);
        }

        public Vector CalculateBrakeForce()
        {
            if (_bike.Brake <= MinBrakeInput || _bike.Velocity.Length <= 0)
                return new Vector(0, 0);
            return -_bike.Velocity * (_bike.Brake * _physics.BrakeForce / _bike.Velocity.Length);
        }

        public Vector CalculateDragForce()
        {
            double speed = _bike.Velocity.Length;
            if (speed <= 0)
                return new Vector(0, 0);
            double currentDrag = _physics.DragCoefficient * (_bike.IsInAir ? AirFrictionMultiplier : 1.0);
            if (!_bike.IsInAir && _bike.Throttle == 0 && _bike.Brake == 0)
                currentDrag *= 2.0;
            return -_bike.Velocity * (currentDrag * speed);
        }

        public Vector CalculateSlopeForce(Level level)
        {
            if (_bike.IsInAir)
                return new Vector(0, 0);
            double slopeAngle = level.CalculateSlopeAngle(_bike.Position.X);
            double slopeForceMagnitude = _physics.Mass * _physics.Gravity * Sin(slopeAngle);
            return new Vector(
                slopeForceMagnitude * Cos(slopeAngle),
                -slopeForceMagnitude * Sin(slopeAngle)
            );
        }

        public Vector CalculateTotalForce(Level level) =>
            new Vector(0, _physics.Gravity * _physics.Mass) +
            CalculateThrustForce() +
            CalculateBrakeForce() +
            CalculateDragForce() +
            CalculateSlopeForce(level);
    }

    public class TorqueComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public TorqueComponent(Motorcycle bike, BikePhysics physics) : base(string.Empty)
        {
            _bike = bike;
            _physics = physics;
        }

        public double CalculateBaseTorque()
        {
            if (_bike.Throttle <= 0)
                return 0;

            double engineForce = _bike.Throttle * _physics.EnginePower;
            Vector thrustForce = new(Cos(_bike.Angle) * engineForce, Sin(_bike.Angle) * engineForce);
            Vector r_rear = _bike.WheelPositions.Rear - _bike.Position;
            return r_rear.X * thrustForce.Y - r_rear.Y * thrustForce.X;
        }

        public double CalculateWheelieTorque(double deltaTime) =>
            CalculateTrickTorque(
                deltaTime,
                true,
                _bike.Throttle,
                WheelieThrottleMinimum,
                WheelieThrottleMultiplier * _physics.EnginePower,
                WheelieForceBase,
                _physics.EnginePower * 2,
                CalculateWheelieSpeedEfficiency,
                CalculateWheelieAngleEfficiency,
                CalculateWheelieBalanceFactor,
                UpdateWheelieBalance);

        public double CalculateStoppieTorque(double deltaTime) =>
            CalculateTrickTorque(
                deltaTime,
                false,
                _bike.Brake,
                StoppieThresholdMinimum,
                StoppieBrakeMultiplier * _physics.BrakeForce,
                StoppieForceBase,
                _physics.BrakeForce * 2,
                CalculateStoppieSpeedEfficiency,
                CalculateStoppieAngleEfficiency,
                CalculateStoppieBalanceFactor,
                UpdateStoppieBalance);

        private double CalculateTrickTorque(
            double deltaTime,
            bool isWheelie,
            double inputValue,
            double minThreshold,
            double forceMultiplier,
            double baseForce,
            double maxForce,
            Func<double> speedEfficiencyFunc,
            Func<double> angleEfficiencyFunc,
            Func<double> balanceFactorFunc,
            Action<double> updateBalanceAction)
        {
            if (_bike.IsInAir || inputValue <= minThreshold)
                return 0;

            double torque = 0;
            double speed = _bike.Velocity.Length;
            if ((isWheelie && speed > 0) || (!isWheelie && speed > StoppieMinSpeed))
            {
                double efficiency = speedEfficiencyFunc() * angleEfficiencyFunc();
                double trickForce = (inputValue - minThreshold) * forceMultiplier * baseForce * efficiency;
                trickForce = SanitizeValue(trickForce, 0.0, "Invalid trick force calculated");

                if (trickForce > maxForce)
                {
                    Logger.Warning("TorqueComponent", $"Excessive {(isWheelie ? "wheelie" : "stoppie")} force: {trickForce:F1}, clamping to {maxForce:F1}");
                    trickForce = maxForce;
                }

                if ((isWheelie && _bike.IsInWheelie) || (!isWheelie && _bike.IsInStoppie))
                    trickForce *= balanceFactorFunc();

                torque += isWheelie ? trickForce : -trickForce;
            }

            if ((isWheelie && _bike.IsInWheelie) || (!isWheelie && _bike.IsInStoppie))
            {
                updateBalanceAction(deltaTime);
                double playerControl = _bike.LeanAmount * (isWheelie ? WheelieControlMultiplier : StoppieControlMultiplier);
                double balanceTorque = (isWheelie ? _physics.WheelieBalance : _physics.StoppieBalance) *
                                      (isWheelie ? WheelieBalanceStrength : StoppieBalanceStrength) +
                                      (isWheelie ? playerControl : -playerControl);

                balanceTorque = SanitizeValue(balanceTorque, 0.0, "Invalid balance torque calculated");
                if (isWheelie && _bike.Brake > 0)
                    balanceTorque += _bike.Brake * WheelieStabilizationFactor;
                else if (!isWheelie && _bike.Throttle > 0)
                    balanceTorque -= _bike.Throttle * StoppieStabilizationFactor;

                torque += balanceTorque;
            }

            return torque;
        }

        private double CalculateEfficiency(
            double value, double minOptimal,
            double maxOptimal, double minValue, double maxValue,
            double lowEfficiency, double highEfficiency)
        {
            if (value < minValue)
                return 0.0;
            else if (value < minOptimal)
                return Lerp(lowEfficiency, 1.0, (value - minValue) / (minOptimal - minValue));
            else if (value > maxOptimal)
                return Lerp(1.0, highEfficiency, (value - maxOptimal) / (maxValue - maxOptimal));
            else
                return 1.0;
        }

        private double CalculateWheelieSpeedEfficiency() =>
            CalculateEfficiency(_bike.Velocity.Length, WheelieOptimalMinSpeed, WheelieOptimalMaxSpeed, 0, WheelieMaxSpeed, 0.5, 0.1);

        private double CalculateStoppieSpeedEfficiency() =>
            CalculateEfficiency(_bike.Velocity.Length, StoppieOptimalMinSpeed, StoppieOptimalMaxSpeed, StoppieMinSpeed, StoppieMaxSpeed, 0.3, 0.2);

        private double CalculateWheelieAngleEfficiency() =>
            CalculateEfficiency(_bike.Angle, -WheelieOptimalAngle, 0, -WheelieOptimalAngle - PI / 6, PI / 4, 0.4, 0.2);

        private double CalculateStoppieAngleEfficiency() =>
            CalculateEfficiency(_bike.Angle, -StoppieOptimalAngle, 0, -StoppieOptimalAngle - PI / 4, PI / 6, 0.2, 0.4);

        private double CalculateWheelieBalanceFactor()
        {
            double angleDiff = Abs(_bike.Angle - WheelieBalanceAngle);
            return angleDiff < WheelieBalanceTolerance ? Lerp(0.2, 1.0, angleDiff / WheelieBalanceTolerance) : 1.0;
        }

        private double CalculateStoppieBalanceFactor()
        {
            double angleDiff = Abs(_bike.Angle - StoppieBalanceAngle);
            return angleDiff < StoppieBalanceTolerance ? Lerp(0.2, 1.0, angleDiff / StoppieBalanceTolerance) : 1.0;
        }

        private void UpdateWheelieBalance(double deltaTime)
        {
            double angleDiff = _bike.Angle - WheelieBalanceAngle;
            _physics.WheelieBalance = -angleDiff * WheelieBalanceResponseFactor;
            _physics.WheelieBalance = SanitizeValue(_physics.WheelieBalance, 0.0, "Invalid wheelie balance value calculated");
            _bike.WheelieTime += deltaTime;
            _bike.WheelieTime = SanitizeValue(_bike.WheelieTime, 0.0, "Invalid wheelie time value");

            if (_bike.WheelieTime > WheelieEasyTime)
            {
                double difficultyFactor = Min(1.0, (_bike.WheelieTime - WheelieEasyTime) / WheelieHardTimeDelta);
                _physics.WheelieBalance *= (1.0 - difficultyFactor * WheelieProgressiveDifficulty);
            }
        }

        private void UpdateStoppieBalance(double deltaTime)
        {
            double angleDiff = _bike.Angle - StoppieBalanceAngle;
            _physics.StoppieBalance = -angleDiff * StoppieBalanceResponseFactor;
            _physics.StoppieBalance = SanitizeValue(_physics.StoppieBalance, 0.0, "Invalid stoppie balance value calculated");
            _bike.StoppieTime += deltaTime;
            _bike.StoppieTime = SanitizeValue(_bike.StoppieTime, 0.0, "Invalid stoppie time value");

            if (_bike.StoppieTime > StoppieEasyTime)
            {
                double difficultyFactor = Min(1.0, (_bike.StoppieTime - StoppieEasyTime) / StoppieHardTimeDelta);
                _physics.StoppieBalance *= (1.0 - difficultyFactor * StoppieProgressiveDifficulty);
            }
        }

        public double CalculateTotalTorque(double deltaTime) =>
            CalculateBaseTorque() +
            CalculateWheelieTorque(deltaTime) +
            CalculateStoppieTorque(deltaTime);
    }

    public class TricksComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public TricksComponent(Motorcycle bike, BikePhysics physics)
        {
            _bike = bike;
            _physics = physics;
        }

        public void UpdateTrickStates(double deltaTime)
        {
            bool wasInWheelie = _bike.IsInWheelie;
            bool wasInStoppie = _bike.IsInStoppie;

            _bike.IsInWheelie = !_bike.IsInAir &&
                               _bike.WheelPositions.Front.Y < _bike.WheelPositions.Rear.Y - _physics.WheelRadius * WheelieHeightFactor &&
                               _bike.Angle > WheelieMinAngle;

            _bike.IsInStoppie = !_bike.IsInAir &&
                               _bike.WheelPositions.Rear.Y < _bike.WheelPositions.Front.Y - _physics.WheelRadius * StoppieHeightFactor &&
                               _bike.Angle < -StoppieMinAngle;

            double wheelieTime = _bike.WheelieTime;
            HandleTrickStateChange(wasInWheelie, _bike.IsInWheelie, ref wheelieTime, "Wheelie");
            _bike.WheelieTime = wheelieTime;

            double stoppieTime = _bike.StoppieTime;
            HandleTrickStateChange(wasInStoppie, _bike.IsInStoppie, ref stoppieTime, "Stoppie");
            _bike.StoppieTime = stoppieTime;
        }

        private void HandleTrickStateChange(bool wasActive, bool isActive, ref double trickTime, string trickName)
        {
            if (!isActive && wasActive)
            {
                trickTime = 0;
                Logger.Debug("TricksComponent", $"{trickName} ended after {trickTime:F1}s");
            }
            else if (isActive && !wasActive)
            {
                Logger.Debug("TricksComponent", $"{trickName} started");
            }
        }
    }

    public class SuspensionComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public SuspensionComponent(Motorcycle bike, BikePhysics physics) : base(string.Empty)
        {
            _bike = bike;
            _physics = physics;
        }

        public void HandleWheelCollisions(Level level, double deltaTime)
        {
            double frontGroundY = level.GetGroundYAtX(_bike.WheelPositions.Front.X);
            bool frontContact = HandleWheelCollision(true, frontGroundY, deltaTime);

            double rearGroundY = level.GetGroundYAtX(_bike.WheelPositions.Rear.X);
            bool rearContact = HandleWheelCollision(false, rearGroundY, deltaTime);

            _bike.State = frontContact || rearContact
                ? _bike.State & ~BikeState.InAir
                : _bike.State | BikeState.InAir;
        }

        public bool HandleWheelCollision(bool isFrontWheel, double groundY, double deltaTime)
        {
            Point wheelPosition = isFrontWheel ? _bike.WheelPositions.Front : _bike.WheelPositions.Rear;
            double penetration = wheelPosition.Y + _physics.WheelRadius - groundY;

            if (penetration <= 0)
            {
                SetSuspensionOffset(isFrontWheel, _physics.SuspensionRestLength);
                return false;
            }

            ProcessWheelPenetration(isFrontWheel, penetration, deltaTime);
            return true;
        }

        private void ProcessWheelPenetration(bool isFrontWheel, double penetration, double deltaTime)
        {
            penetration = Min(penetration, _physics.WheelRadius * MaxWheelPenetration);

            double baseCompression = penetration * PenetrationBaseMultiplier;
            double progressiveFactor = 1.0 + Pow(SafeDivide(penetration, _physics.WheelRadius * WheelRadiusHalfFactor), 2) * PenetrationProgressiveFactor;
            double desiredCompression = baseCompression * progressiveFactor;

            double currentOffset = isFrontWheel ? _bike.SuspensionOffsets.Front : _bike.SuspensionOffsets.Rear;
            double dampingFactor = SuspensionDampingFactor * deltaTime;
            double newOffset = Lerp(currentOffset, _physics.SuspensionRestLength - desiredCompression, dampingFactor);

            newOffset = ClampValue(newOffset, _physics.SuspensionRestLength * MinSuspensionCompression, _physics.SuspensionRestLength);
            Point attachmentPoint = isFrontWheel ? _bike.AttachmentPoints.Front : _bike.AttachmentPoints.Rear;
            Point wheelPosition = isFrontWheel ? _bike.WheelPositions.Front : _bike.WheelPositions.Rear;
            double compressionRatio = 1.0 - SafeDivide(newOffset, _physics.SuspensionRestLength);

            double progressiveFactorForce = 1.0 + Pow(compressionRatio, 2) * SuspensionProgressiveFactor;
            double adjustedSuspensionStrength = _physics.SuspensionStrength * progressiveFactorForce;

            Vector reactionForce = new(
                -_bike.Velocity.X * _physics.GroundFriction * FrictionMultiplier,
                -adjustedSuspensionStrength * penetration - _physics.SuspensionDamping * _bike.Velocity.Y
            );

            Vector r = wheelPosition - _bike.Position;
            double torque = r.X * reactionForce.Y - r.Y * reactionForce.X;

            _bike.Velocity += reactionForce / _physics.Mass * deltaTime;
            _bike.AngularVelocity += torque / _physics.MomentOfInertia * deltaTime;

            SetSuspensionOffset(isFrontWheel, newOffset);
        }

        private void SetSuspensionOffset(bool isFrontWheel, double offset)
        {
            var current = _bike.SuspensionOffsets;
            _bike.SuspensionOffsets = isFrontWheel
                ? (offset, current.Rear)
                : (current.Front, offset);
        }
    }

    public class CollisionComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public CollisionComponent(Motorcycle bike, BikePhysics physics) : base(string.Empty)
        {
            _bike = bike;
            _physics = physics;
        }

        public void CheckFrameCollision(Level level, double deltaTime)
        {
            if (_bike.IsInAir || _bike.IsCrashed || _bike.Velocity.Length <= FrameCollisionMinVelocity)
                return;

            var collisionInfo = DetectFrameCollision(level);
            if (collisionInfo.IsCollision && collisionInfo.MaxPenetration > _physics.WheelRadius * FrameCollisionMinPenetration)
                HandleFrameCollision(collisionInfo, deltaTime);
        }

        private void HandleFrameCollision((bool IsCollision, Point CollisionPoint, double MaxPenetration) collisionInfo, double deltaTime)
        {
            double crashThreshold = _physics.WheelRadius * FrameCrashThreshold;
            bool isBadAngle = Abs(_bike.Angle) > FrameCriticalBackwardTiltAngle;
            bool isHighSpeed = _bike.Velocity.Length > 200;

            if (collisionInfo.MaxPenetration > crashThreshold && (isHighSpeed || isBadAngle))
            {
                _bike.State |= BikeState.Crashed;
                Logger.Warning("CollisionComponent",
                    $"Frame collision detected at {collisionInfo.CollisionPoint}, " +
                    $"penetration: {collisionInfo.MaxPenetration:F1}, " +
                    $"speed: {_bike.Velocity.Length:F1}, " +
                    $"angle: {_bike.Angle * 180 / PI:F1}°");
            }
            else
            {
                ApplyFrameCollisionResponse(collisionInfo.CollisionPoint, collisionInfo.MaxPenetration, deltaTime);
            }
        }

        private void ApplyFrameCollisionResponse(Point collisionPoint, double penetration, double deltaTime)
        {
            double reactionForce = penetration * _physics.SuspensionStrength * FrameCollisionReactionForce;
            double impulse = reactionForce * deltaTime;
            double deltaVelocityY = -impulse / _physics.Mass;

            deltaVelocityY = Max(deltaVelocityY, -FrameCollisionMaxDeltaVelocity * deltaTime);
            _bike.Velocity = new Vector(_bike.Velocity.X, _bike.Velocity.Y + deltaVelocityY);

            double stabilizingFactor = Abs(_bike.Angle) > FrameStabilizingAngleThreshold
                ? FrameStabilizingFactorStrong
                : FrameStabilizingFactorBase;

            double stabilizingTorque = -_bike.Angle * stabilizingFactor;
            double deltaAngularVelocity = stabilizingTorque * deltaTime / _physics.MomentOfInertia;

            deltaAngularVelocity = ClampValue(deltaAngularVelocity,
                                             -FrameCollisionMaxDeltaAngular * deltaTime,
                                              FrameCollisionMaxDeltaAngular * deltaTime);

            _bike.AngularVelocity += deltaAngularVelocity;
        }

        private (bool IsCollision, Point CollisionPoint, double MaxPenetration) DetectFrameCollision(Level level)
        {
            var framePoints = _bike.GetFramePoints();
            bool frameCollision = false;
            Point collisionPoint = default;
            double maxPenetration = 0;

            foreach (var point in framePoints)
            {
                double groundY = level.GetGroundYAtX(point.X);
                double penetration = point.Y - groundY;

                if (penetration > 0 && penetration > maxPenetration)
                {
                    frameCollision = true;
                    maxPenetration = penetration;
                    collisionPoint = point;
                }
            }

            return (frameCollision, collisionPoint, maxPenetration);
        }
    }

    public class BikePhysics : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly ForcesComponent _forcesComponent;
        private readonly TorqueComponent _torqueComponent;
        private readonly TricksComponent _tricksComponent;
        private readonly SuspensionComponent _suspensionComponent;
        private readonly CollisionComponent _collisionComponent;

        private double _prevAngularVelocity = 0;
        private Vector _prevVelocity = new(0, 0);
        private int _updateCounter = 0;
        private const int StatusLogInterval = 100;

        public double BrakeForce { get; private set; }
        public double DragCoefficient { get; private set; }
        public double EnginePower { get; private set; }
        public double Gravity { get; private set; }
        public double GroundFriction { get; private set; }
        public double LeanSpeed { get; private set; }
        public double Mass { get; private set; }
        public double MaxLeanAngle { get; private set; }
        public double MaxWheelDistance { get; private set; }
        public double MinWheelDistance { get; private set; }
        public double MomentOfInertia { get; private set; }
        public double NominalWheelBase { get; private set; }
        public double SuspensionDamping { get; private set; }
        public double SuspensionRestLength { get; private set; }
        public double SuspensionStrength { get; private set; }
        public double WheelRadius { get; private set; } = DefaultWheelRadius;
        public double WheelieBalance { get; set; } = 0.0;
        public double StoppieBalance { get; set; } = 0.0;
        public double MaxSuspensionAngle { get; private set; }

        public BikePhysics(Motorcycle bike) : base(BikePhysicsTag)
        {
            _bike = bike;
            Gravity = DefaultGravity * GravityMultiplier;

            _forcesComponent = new ForcesComponent(bike, this);
            _torqueComponent = new TorqueComponent(bike, this);
            _tricksComponent = new TricksComponent(bike, this);
            _suspensionComponent = new SuspensionComponent(bike, this);
            _collisionComponent = new CollisionComponent(bike, this);
        }

        public void InitializeProperties(BikeType bikeType)
        {
            var props = GetBikeProperties(bikeType);
            var wheelProps = GetWheelProperties(bikeType);

            (Mass, EnginePower, BrakeForce, DragCoefficient) = (props.mass, props.power, props.brakeForce, props.drag);
            (MaxLeanAngle, LeanSpeed) = (props.maxLeanAngle, props.leanSpeed);
            GroundFriction = props.friction * wheelProps.friction;
            (SuspensionStrength, SuspensionDamping, SuspensionRestLength) =
                (props.suspensionStrength * wheelProps.suspensionStrength,
                 props.suspensionDamping * wheelProps.suspensionDamping,
                 props.suspensionRestLength);
            MaxSuspensionAngle = props.maxSuspensionAngle; 
            WheelRadius = wheelProps.radius;

            MomentOfInertia = Mass * Pow(_bike.WheelBase / 2, 2) * MomentOfInertiaMultiplier;

            NominalWheelBase = _bike.WheelBase;
            MinWheelDistance = NominalWheelBase * WheelDistanceMinRatio;
            MaxWheelDistance = NominalWheelBase * WheelDistanceMaxRatio;
        }

        public void UpdatePhysics(double deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            if (cancellationToken.IsCancellationRequested) return;

            Vector oldVelocity = _bike.Velocity;
            double oldAngularVelocity = _bike.AngularVelocity;
            SavePreviousState();

            _suspensionComponent.HandleWheelCollisions(level, deltaTime);

            Vector totalForce = _forcesComponent.CalculateTotalForce(level);
            if (!ValidateVectorParameter("TotalForce", totalForce, Mass * MaxSafeAcceleration))
            {
                TryLog(LogLevel.W, $"High total force: {totalForce.Length:F1} N");
            }

            double totalTorque = _torqueComponent.CalculateTotalTorque(deltaTime);
            double maxTorque = MomentOfInertia * MaxAngularVelocity / deltaTime;
            if (!ValidatePhysicalParameter("TotalTorque", Abs(totalTorque), 0, maxTorque))
            {
                TryLog(LogLevel.W, $"High torque: {totalTorque:F1} Nm (max safe: {maxTorque:F1})");
            }

            _bike.Velocity += totalForce / Mass * deltaTime;
            _bike.AngularVelocity += totalTorque / MomentOfInertia * deltaTime;

            Vector velocityDelta = _bike.Velocity - oldVelocity;
            double acceleration = velocityDelta.Length / deltaTime;
            if (!ValidatePhysicalParameter("Acceleration", acceleration, 0, MaxSafeAcceleration))
            {
                TryLog(LogLevel.W, $"High acceleration: {acceleration:F1} units/s² (max safe: {MaxSafeAcceleration:F1})");
            }

            double angularAcceleration = Abs(_bike.AngularVelocity - oldAngularVelocity) / deltaTime;
            if (!ValidatePhysicalParameter("AngularAcceleration", angularAcceleration, 0, MaxAngularVelocity * 2))
            {
                TryLog(LogLevel.W, $"High angular acceleration: {angularAcceleration:F1} rad/s² (max safe: {MaxAngularVelocity * 2:F1})");
            }

            UpdateLean(deltaTime, level);
            ApplyRotation(deltaTime);
            _collisionComponent.CheckFrameCollision(level, deltaTime);
            CheckCrashConditions();
            _tricksComponent.UpdateTrickStates(deltaTime);

            double frontCompression = 1.0 - SafeDivide(_bike.SuspensionOffsets.Front, SuspensionRestLength);
            double rearCompression = 1.0 - SafeDivide(_bike.SuspensionOffsets.Rear, SuspensionRestLength);

            if (!ValidatePhysicalParameter("FrontSuspensionCompression", frontCompression, 0, MaxSuspensionCompression))
            {
                TryLog(LogLevel.W, $"High front suspension compression: {frontCompression:P0}");
            }

            if (!ValidatePhysicalParameter("RearSuspensionCompression", rearCompression, 0, MaxSuspensionCompression))
            {
                TryLog(LogLevel.W, $"High rear suspension compression: {rearCompression:P0}");
            }

            double wheelDistance = CalculateWheelDistance();
            if (!ValidateConnectionStrain("WheelBase", wheelDistance, NominalWheelBase, WheelDistanceMinRatio, WheelDistanceMaxRatio))
            {
                TryLog(LogLevel.W, $"Abnormal wheel distance: {wheelDistance:F1} (nominal: {NominalWheelBase:F1})");
            }

            if (_bike.IsInWheelie)
            {
                ValidatePhysicalParameter("WheelieAngle", _bike.Angle, 0, PI / 1.5);
                ValidatePhysicalParameter("WheelieTime", _bike.WheelieTime, 0, 60);
                ValidatePhysicalParameter("WheelieBalance", WheelieBalance, -10, 10);

                if (_bike.WheelieTime % 1.0 < deltaTime)
                {
                    TryLog(LogLevel.D, $"Wheelie active for {_bike.WheelieTime:F1}s, angle: {_bike.Angle * 180 / PI:F1}°");
                }
            }

            if (_bike.IsInStoppie)
            {
                ValidatePhysicalParameter("StoppieAngle", _bike.Angle, -PI / 1.5, 0);
                ValidatePhysicalParameter("StoppieTime", _bike.StoppieTime, 0, 30);
                ValidatePhysicalParameter("StoppieBalance", StoppieBalance, -10, 10);

                if (_bike.StoppieTime % 1.0 < deltaTime)
                {
                    TryLog(LogLevel.D, $"Stoppie active for {_bike.StoppieTime:F1}s, angle: {_bike.Angle * 180 / PI:F1}°");
                }
            }

            if (_bike.IsInWheelie && _bike.IsInStoppie)
            {
                TryLog(LogLevel.E, "Impossible state: both wheelie and stoppie active simultaneously");
            }

            if (_bike.IsInAir)
            {
                double frontGroundY = level.GetGroundYAtX(_bike.WheelPositions.Front.X);
                double rearGroundY = level.GetGroundYAtX(_bike.WheelPositions.Rear.X);
                double frontClearance = frontGroundY - (_bike.WheelPositions.Front.Y + WheelRadius);
                double rearClearance = rearGroundY - (_bike.WheelPositions.Rear.Y + WheelRadius);

                if (frontClearance < 0 || rearClearance < 0)
                {
                    TryLog(LogLevel.W, $"Inconsistent air state: wheel clearance negative (front: {frontClearance:F1}, rear: {rearClearance:F1})");
                }
            }

            _updateCounter++;
            if (_updateCounter >= StatusLogInterval)
            {
                _updateCounter = 0;
                TryLog(LogLevel.D, $"Status: Pos=({_bike.Position.X:F0},{_bike.Position.Y:F0}), " +
                                   $"Speed={_bike.Velocity.Length:F1}, " +
                                   $"Angle={_bike.Angle * 180 / PI:F1}°, " +
                                   $"State={_bike.State}");
            }

            SanitizePhysicalState();
        }

        private void ApplyRotation(double deltaTime) =>
            _bike.Angle = NormalizeAngle(_bike.Angle + _bike.AngularVelocity * deltaTime);

        private void CheckCrashConditions()
        {
            if (_bike.IsInAir || _bike.IsCrashed)
                return;

            double absAngle = Abs(_bike.Angle);
            if (absAngle > CriticalLeanAngle)
            {
                _bike.State |= BikeState.Crashed;
                TryLog(LogLevel.W, $"Bike crashed due to excessive tilt: {_bike.Angle * 180 / PI:F1}°");
                return;
            }

            double currentDistance = CalculateWheelDistance();
            if (currentDistance < MinWheelDistance || currentDistance > MaxWheelDistance)
            {
                _bike.State |= BikeState.Crashed;
                double deformation = Abs(currentDistance - NominalWheelBase) / NominalWheelBase * 100;
                TryLog(LogLevel.W, $"Bike crashed due to frame deformation: {deformation:F1}% (distance: {currentDistance:F1})");
            }
        }

        public void CheckFrameCollision(Level level, double deltaTime) =>
            _collisionComponent.CheckFrameCollision(level, deltaTime);

        public void HandleWheelCollision(bool isFrontWheel, double groundY, double deltaTime) =>
            _suspensionComponent.HandleWheelCollision(isFrontWheel, groundY, deltaTime);

        private double CalculateWheelDistance() =>
            CalculateDistance(_bike.WheelPositions.Front, _bike.WheelPositions.Rear);

        private BikeProperties GetBikeProperties(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => Bike.Standard,
            BikeType.Sport => Bike.Sport,
            BikeType.OffRoad => Bike.OffRoad,
            _ => Bike.Standard
        };

        private WheelProperties GetWheelProperties(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => Wheels.Standard,
            BikeType.Sport => Wheels.Sport,
            BikeType.OffRoad => Wheels.OffRoad,
            _ => Wheels.Standard
        };

        private void SavePreviousState()
        {
            _prevVelocity = _bike.Velocity;
            _prevAngularVelocity = _bike.AngularVelocity;
        }

        private void UpdateAirLean(double deltaTime)
        {
            const double maxAirAngularVelocity = PI;
            double desiredAngularVelocity = _bike.LeanAmount * maxAirAngularVelocity;
            const double damping = 0.5;
            _bike.AngularVelocity += (desiredAngularVelocity - _bike.AngularVelocity) * damping * deltaTime;
        }

        private void UpdateGroundLean(double deltaTime, Level level)
        {
            double slopeAngle = _bike.IsInAir ? 0 : level.CalculateSlopeAngle(_bike.Position.X);

            double throttleEffect = _bike.Throttle * 0.2;
            double targetLean = _bike.LeanAmount * MaxLeanAngle + slopeAngle + throttleEffect;
            double leanError = targetLean - _bike.Angle;
            double leanErrorDerivative = -_bike.AngularVelocity;

            double controlTorque = 1000.0 * leanError * LeanSpeed + 200.0 * leanErrorDerivative;
            _bike.AngularVelocity += controlTorque / MomentOfInertia * deltaTime;

            double maxAngularVel = MaxAngularVelocity * GroundAngularVelocityFactor;
            _bike.AngularVelocity = ClampValue(_bike.AngularVelocity, -maxAngularVel, maxAngularVel);
        }

        private void UpdateLean(double deltaTime, Level level)
        {
            if (_bike.IsInAir)
                UpdateAirLean(deltaTime);
            else
                UpdateGroundLean(deltaTime, level);
        }

        private void SanitizePhysicalState()
        {
            _bike.Angle = SanitizeValue(_bike.Angle, 0, "Invalid angle value");
            _bike.AngularVelocity = SanitizeValue(_bike.AngularVelocity, 0, "Invalid angular velocity");
            _bike.Velocity = SanitizeVector(_bike.Velocity, new Vector(0, 0), "Invalid velocity");

            _bike.Angle = NormalizeAngle(_bike.Angle);
            _bike.AngularVelocity = ClampValue(_bike.AngularVelocity, -MaxAngularVelocity, MaxAngularVelocity);

            if (_bike.Velocity.Length > MaxSafeVelocity)
            {
                TryLog(LogLevel.W, $"Extreme velocity: {_bike.Velocity.Length:F1}, clamping");
                _bike.Velocity = _bike.Velocity * (MaxSafeVelocity / _bike.Velocity.Length);
            }
        }
    }
}