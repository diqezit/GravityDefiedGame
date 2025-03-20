using System;
using System.Collections.Generic;
using System.Threading;
using System.Windows;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Utilities.GameConstants;
using static GravityDefiedGame.Utilities.GameConstants.Debug;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;

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

    // Класс для расчета сил
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

            double effectiveBrakeForce = _bike.Brake * _physics.BrakeForce;
            Vector brakeVector = -_bike.Velocity;
            return brakeVector * (SafeDivide(effectiveBrakeForce, brakeVector.Length));
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
            double slopeForceMagnitude = _physics.Mass * _physics.Gravity * Math.Sin(slopeAngle);
            return new Vector(slopeForceMagnitude * Math.Cos(slopeAngle), -slopeForceMagnitude * Math.Sin(slopeAngle));
        }

        public Vector CalculateTotalForce(Level level)
        {
            Vector gravityForce = new Vector(0, _physics.Gravity * _physics.Mass);
            Vector thrustForce = CalculateThrustForce();
            Vector brakeForce = CalculateBrakeForce();
            Vector dragForce = CalculateDragForce();
            Vector slopeForce = CalculateSlopeForce(level);
            return gravityForce + thrustForce + brakeForce + dragForce + slopeForce;
        }
    }

    // Класс для расчета крутящего момента
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
            double torque = 0;
            if (_bike.Throttle > 0)
            {
                double engineForce = _bike.Throttle * _physics.EnginePower;
                Vector thrustForce = new Vector(Math.Cos(_bike.Angle) * engineForce, Math.Sin(_bike.Angle) * engineForce);
                Vector r_rear = _bike.WheelPositions.Rear - _bike.Position;
                torque += r_rear.X * thrustForce.Y - r_rear.Y * thrustForce.X;
            }
            return torque;
        }

        public double CalculateWheelieTorque(double deltaTime)
        {
            if (_bike.IsInAir) return 0;

            double torque = 0;

            if (_bike.Throttle > WheelieThrottleMinimum)
            {
                double speedEfficiency = CalculateWheelieSpeedEfficiency();
                double angleEfficiency = CalculateWheelieAngleEfficiency();

                double wheelieForce = (_bike.Throttle - WheelieThrottleMinimum) * WheelieThrottleMultiplier *
                                     _physics.EnginePower * WheelieForceBase * speedEfficiency * angleEfficiency;

                wheelieForce = SanitizeValue(wheelieForce, 0.0, "Invalid wheelie force calculated");
                double maxWheelieForce = _physics.EnginePower * 2;

                if (wheelieForce > maxWheelieForce)
                {
                    Logger.Warning("TorqueComponent", $"Excessive wheelie force: {wheelieForce:F1}, clamping to {maxWheelieForce:F1}");
                    wheelieForce = maxWheelieForce;
                }

                if (_bike.IsInWheelie)
                    wheelieForce *= CalculateWheelieBalanceFactor();

                torque += wheelieForce;
            }

            if (_bike.IsInWheelie)
            {
                UpdateWheelieBalance(deltaTime);

                double playerControl = _bike.LeanAmount * WheelieControlMultiplier;
                double balanceTorque = _physics.WheelieBalance * WheelieBalanceStrength + playerControl;

                balanceTorque = SanitizeValue(balanceTorque, 0.0, "Invalid wheelie balance torque calculated");

                if (_bike.Brake > 0)
                    balanceTorque += _bike.Brake * WheelieStabilizationFactor;

                torque += balanceTorque;
            }

            return torque;
        }

        public double CalculateStoppieTorque(double deltaTime)
        {
            if (_bike.IsInAir) return 0;

            double torque = 0;

            if (_bike.Brake > StoppieThresholdMinimum && _bike.Velocity.Length > StoppieMinSpeed)
            {
                double speedEfficiency = CalculateStoppieSpeedEfficiency();
                double angleEfficiency = CalculateStoppieAngleEfficiency();

                double stoppieForce = (_bike.Brake - StoppieThresholdMinimum) * StoppieBrakeMultiplier *
                                     _physics.BrakeForce * StoppieForceBase * speedEfficiency * angleEfficiency;

                stoppieForce = SanitizeValue(stoppieForce, 0.0, "Invalid stoppie force calculated");
                double maxStoppieForce = _physics.BrakeForce * 2;

                if (stoppieForce > maxStoppieForce)
                {
                    Logger.Warning("TorqueComponent", $"Excessive stoppie force: {stoppieForce:F1}, clamping to {maxStoppieForce:F1}");
                    stoppieForce = maxStoppieForce;
                }

                if (_bike.IsInStoppie)
                    stoppieForce *= CalculateStoppieBalanceFactor();

                torque -= stoppieForce;
            }

            if (_bike.IsInStoppie)
            {
                UpdateStoppieBalance(deltaTime);

                double playerControl = _bike.LeanAmount * StoppieControlMultiplier;
                double balanceTorque = _physics.StoppieBalance * StoppieBalanceStrength - playerControl;

                balanceTorque = SanitizeValue(balanceTorque, 0.0, "Invalid stoppie balance torque calculated");

                if (_bike.Throttle > 0)
                    balanceTorque -= _bike.Throttle * StoppieStabilizationFactor;

                torque += balanceTorque;
            }

            return torque;
        }

        private double CalculateWheelieSpeedEfficiency()
        {
            double speed = _bike.Velocity.Length;
            if (speed < WheelieOptimalMinSpeed)
                return Lerp(0.5, 1.0, speed / WheelieOptimalMinSpeed);
            if (speed > WheelieOptimalMaxSpeed)
                return Lerp(1.0, 0.1, (speed - WheelieOptimalMaxSpeed) / (WheelieMaxSpeed - WheelieOptimalMaxSpeed));
            return 1.0;
        }

        private double CalculateWheelieAngleEfficiency()
        {
            double angle = _bike.Angle;
            if (angle > 0)
                return Lerp(1.0, 0.2, angle / (Math.PI / 4));
            if (angle < -WheelieOptimalAngle)
                return Lerp(1.0, 0.4, (-angle - WheelieOptimalAngle) / (Math.PI / 6));
            return Lerp(1.0, 1.2, -angle / WheelieOptimalAngle);
        }

        private double CalculateWheelieBalanceFactor()
        {
            double wheelieAngle = _bike.Angle;
            double optimalBalanceAngle = WheelieBalanceAngle;
            double angleDiff = Math.Abs(wheelieAngle - optimalBalanceAngle);
            if (angleDiff < WheelieBalanceTolerance)
                return Lerp(0.2, 1.0, angleDiff / WheelieBalanceTolerance);
            return 1.0;
        }

        private double CalculateStoppieSpeedEfficiency()
        {
            double speed = _bike.Velocity.Length;
            if (speed < StoppieMinSpeed)
                return 0.0;
            if (speed < StoppieOptimalMinSpeed)
                return Lerp(0.3, 1.0, (speed - StoppieMinSpeed) / (StoppieOptimalMinSpeed - StoppieMinSpeed));
            if (speed > StoppieOptimalMaxSpeed)
                return Lerp(1.0, 0.2, (speed - StoppieOptimalMaxSpeed) / (StoppieMaxSpeed - StoppieOptimalMaxSpeed));
            return 1.0;
        }

        private double CalculateStoppieAngleEfficiency()
        {
            double angle = _bike.Angle;
            if (angle < -StoppieOptimalAngle)
                return Lerp(1.0, 0.2, (-angle - StoppieOptimalAngle) / (Math.PI / 4));
            if (angle > 0)
                return Lerp(1.0, 0.4, angle / (Math.PI / 6));
            return Lerp(1.0, 1.2, -angle / StoppieOptimalAngle);
        }

        private double CalculateStoppieBalanceFactor()
        {
            double stoppieAngle = _bike.Angle;
            double optimalBalanceAngle = StoppieBalanceAngle;
            double angleDiff = Math.Abs(stoppieAngle - optimalBalanceAngle);
            if (angleDiff < StoppieBalanceTolerance)
                return Lerp(0.2, 1.0, angleDiff / StoppieBalanceTolerance);
            return 1.0;
        }

        private void UpdateWheelieBalance(double deltaTime)
        {
            double optimalAngle = WheelieBalanceAngle;
            double currentAngle = _bike.Angle;
            double angleDiff = currentAngle - optimalAngle;

            _physics.WheelieBalance = -angleDiff * WheelieBalanceResponseFactor;
            _physics.WheelieBalance = SanitizeValue(_physics.WheelieBalance, 0.0, "Invalid wheelie balance value calculated");

            _bike.WheelieTime += deltaTime;
            _bike.WheelieTime = SanitizeValue(_bike.WheelieTime, 0.0, "Invalid wheelie time value");

            if (_bike.WheelieTime > WheelieEasyTime)
            {
                double difficultyFactor = Math.Min(1.0, (_bike.WheelieTime - WheelieEasyTime) / WheelieHardTimeDelta);
                _physics.WheelieBalance *= (1.0 - difficultyFactor * WheelieProgressiveDifficulty);
            }
        }

        private void UpdateStoppieBalance(double deltaTime)
        {
            double optimalAngle = StoppieBalanceAngle;
            double currentAngle = _bike.Angle;
            double angleDiff = currentAngle - optimalAngle;

            _physics.StoppieBalance = -angleDiff * StoppieBalanceResponseFactor;
            _physics.StoppieBalance = SanitizeValue(_physics.StoppieBalance, 0.0, "Invalid stoppie balance value calculated");

            _bike.StoppieTime += deltaTime;
            _bike.StoppieTime = SanitizeValue(_bike.StoppieTime, 0.0, "Invalid stoppie time value");

            if (_bike.StoppieTime > StoppieEasyTime)
            {
                double difficultyFactor = Math.Min(1.0, (_bike.StoppieTime - StoppieEasyTime) / StoppieHardTimeDelta);
                _physics.StoppieBalance *= (1.0 - difficultyFactor * StoppieProgressiveDifficulty);
            }
        }

        public double CalculateTotalTorque(double deltaTime)
        {
            double baseTorque = CalculateBaseTorque();
            double wheelieTorque = CalculateWheelieTorque(deltaTime);
            double stoppieTorque = CalculateStoppieTorque(deltaTime);
            return baseTorque + wheelieTorque + stoppieTorque;
        }
    }

    // Класс для управления трюками
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

            if (!_bike.IsInWheelie && wasInWheelie)
            {
                _bike.WheelieTime = 0;
                Logger.Debug("TricksComponent", $"Wheelie ended after {_bike.WheelieTime:F1}s");
            }
            else if (_bike.IsInWheelie && !wasInWheelie)
            {
                Logger.Debug("TricksComponent", "Wheelie started");
            }

            if (!_bike.IsInStoppie && wasInStoppie)
            {
                _bike.StoppieTime = 0;
                Logger.Debug("TricksComponent", $"Stoppie ended after {_bike.StoppieTime:F1}s");
            }
            else if (_bike.IsInStoppie && !wasInStoppie)
            {
                Logger.Debug("TricksComponent", "Stoppie started");
            }
        }
    }

    // Класс для управления подвеской
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

            if (!frontContact && !rearContact)
            {
                _bike.State |= BikeState.InAir;
            }
            else
            {
                _bike.State &= ~BikeState.InAir;
            }
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
            penetration = Math.Min(penetration, _physics.WheelRadius * MaxWheelPenetration);

            double baseCompression = penetration * PenetrationBaseMultiplier;
            double progressiveFactor = 1.0 + Math.Pow(SafeDivide(penetration, _physics.WheelRadius * WheelRadiusHalfFactor), 2) * PenetrationProgressiveFactor;
            double desiredCompression = baseCompression * progressiveFactor;

            double currentOffset = isFrontWheel ? _bike.SuspensionOffsets.Front : _bike.SuspensionOffsets.Rear;
            double dampingFactor = SuspensionDampingFactor * deltaTime;
            double newOffset = Lerp(currentOffset, _physics.SuspensionRestLength - desiredCompression, dampingFactor);

            newOffset = ClampValue(newOffset, _physics.SuspensionRestLength * MinSuspensionCompression, _physics.SuspensionRestLength);
            Point attachmentPoint = isFrontWheel ? _bike.AttachmentPoints.Front : _bike.AttachmentPoints.Rear;
            double compressionRatio = 1.0 - SafeDivide(newOffset, _physics.SuspensionRestLength);

            double progressiveFactorForce = 1.0 + Math.Pow(compressionRatio, 2) * SuspensionProgressiveFactor;
            double adjustedSuspensionStrength = _physics.SuspensionStrength * progressiveFactorForce;
            double normalForce = adjustedSuspensionStrength * penetration;
            double frictionCoefficient = _physics.GroundFriction * FrictionMultiplier;

            Vector reactionForce = new Vector(
                -_bike.Velocity.X * frictionCoefficient,
                -adjustedSuspensionStrength * penetration - _physics.SuspensionDamping * _bike.Velocity.Y
            );

            Vector r = attachmentPoint - _bike.Position;
            double torque = r.X * reactionForce.Y - r.Y * reactionForce.X;

            _bike.Velocity += reactionForce / _physics.Mass * deltaTime;
            _bike.AngularVelocity += torque / _physics.MomentOfInertia * deltaTime;

            SetSuspensionOffset(isFrontWheel, newOffset);
        }

        private void SetSuspensionOffset(bool isFrontWheel, double offset)
        {
            var current = _bike.SuspensionOffsets;
            _bike.SuspensionOffsets = isFrontWheel ? (offset, current.Rear) : (current.Front, offset);
        }
    }

    // Класс для обработки столкновений
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
            bool isBadAngle = Math.Abs(_bike.Angle) > FrameCriticalBackwardTiltAngle;
            bool isHighSpeed = _bike.Velocity.Length > 200;

            if (collisionInfo.MaxPenetration > crashThreshold && (isHighSpeed || isBadAngle))
            {
                _bike.State |= BikeState.Crashed;
                Logger.Warning("CollisionComponent",
                    $"Frame collision detected at {collisionInfo.CollisionPoint}, " +
                    $"penetration: {collisionInfo.MaxPenetration:F1}, " +
                    $"speed: {_bike.Velocity.Length:F1}, " +
                    $"angle: {_bike.Angle * 180 / Math.PI:F1}°");
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

            deltaVelocityY = Math.Max(deltaVelocityY, -FrameCollisionMaxDeltaVelocity * deltaTime);
            _bike.Velocity = new Vector(_bike.Velocity.X, _bike.Velocity.Y + deltaVelocityY);

            double stabilizingFactor = Math.Abs(_bike.Angle) > FrameStabilizingAngleThreshold ? FrameStabilizingFactorStrong : FrameStabilizingFactorBase;
            double stabilizingTorque = -_bike.Angle * stabilizingFactor;
            double deltaAngularVelocity = stabilizingTorque * deltaTime / _physics.MomentOfInertia;

            deltaAngularVelocity = ClampValue(deltaAngularVelocity, -FrameCollisionMaxDeltaAngular * deltaTime, FrameCollisionMaxDeltaAngular * deltaTime);
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
                if (penetration > 0)
                {
                    frameCollision = true;
                    if (penetration > maxPenetration)
                    {
                        maxPenetration = penetration;
                        collisionPoint = point;
                    }
                }
            }

            return (frameCollision, collisionPoint, maxPenetration);
        }
    }

    // Основной класс BikePhysics
    public class BikePhysics : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private ForcesComponent _forcesComponent;
        private TorqueComponent _torqueComponent;
        private TricksComponent _tricksComponent;
        private SuspensionComponent _suspensionComponent;
        private CollisionComponent _collisionComponent;

        private double _prevAngularVelocity = 0;
        private Vector _prevVelocity = new(0, 0);
        private int _updateCounter = 0;
        private const int StatusLogInterval = 100;

        private const string PHYSICS_COMPONENT = "Physics";
        private const string FORCES_COMPONENT = "Forces";
        private const string TORQUE_COMPONENT = "Torque";
        private const string TRICKS_COMPONENT = "Tricks";
        private const string SUSPENSION_COMPONENT = "Suspension";
        private const string COLLISION_COMPONENT = "Collision";
        private const string WHEEL_COMPONENT = "Wheels";
        private const string BALANCE_COMPONENT = "Balance";

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
        public double WheelieBalance { get; set; } = 0.0; // Перенесено из private поля
        public double StoppieBalance { get; set; } = 0.0;  // Перенесено из private поля

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

            Mass = props.mass;
            EnginePower = props.power;
            BrakeForce = props.brakeForce;
            DragCoefficient = props.drag;
            MaxLeanAngle = props.maxLeanAngle;
            LeanSpeed = props.leanSpeed;
            GroundFriction = props.friction * wheelProps.friction;
            SuspensionStrength = props.suspensionStrength * wheelProps.suspensionStrength;
            SuspensionDamping = props.suspensionDamping * wheelProps.suspensionDamping;
            SuspensionRestLength = props.suspensionRestLength;
            WheelRadius = wheelProps.radius;

            MomentOfInertia = Mass * Math.Pow(_bike.WheelBase / 2, 2) * MomentOfInertiaMultiplier;

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
            if (!ValidateVectorParameter("TotalForce", totalForce, Mass * MaxSafeAcceleration, FORCES_COMPONENT))
            {
                TryLog(LogLevel.W, $"High total force: {totalForce.Length:F1} N");
            }

            double totalTorque = _torqueComponent.CalculateTotalTorque(deltaTime);
            double maxTorque = MomentOfInertia * MaxAngularVelocity / deltaTime;
            if (!ValidatePhysicalParameter("TotalTorque", Math.Abs(totalTorque), 0, maxTorque, TORQUE_COMPONENT))
            {
                TryLog(LogLevel.W, $"High torque: {totalTorque:F1} Nm (max safe: {maxTorque:F1})");
            }

            _bike.Velocity += totalForce / Mass * deltaTime;
            _bike.AngularVelocity += totalTorque / MomentOfInertia * deltaTime;

            Vector velocityDelta = _bike.Velocity - oldVelocity;
            double acceleration = velocityDelta.Length / deltaTime;
            if (!ValidatePhysicalParameter("Acceleration", acceleration, 0, MaxSafeAcceleration, PHYSICS_COMPONENT))
            {
                TryLog(LogLevel.W, $"High acceleration: {acceleration:F1} units/s² (max safe: {MaxSafeAcceleration:F1})");
            }

            double angularAcceleration = Math.Abs(_bike.AngularVelocity - oldAngularVelocity) / deltaTime;
            if (!ValidatePhysicalParameter("AngularAcceleration", angularAcceleration, 0, MaxAngularVelocity * 2, TORQUE_COMPONENT))
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

            if (!ValidatePhysicalParameter("FrontSuspensionCompression", frontCompression, 0, MaxSuspensionCompression, SUSPENSION_COMPONENT))
            {
                TryLog(LogLevel.W, $"High front suspension compression: {frontCompression:P0}");
            }

            if (!ValidatePhysicalParameter("RearSuspensionCompression", rearCompression, 0, MaxSuspensionCompression, SUSPENSION_COMPONENT))
            {
                TryLog(LogLevel.W, $"High rear suspension compression: {rearCompression:P0}");
            }

            double wheelDistance = CalculateWheelDistance();
            if (!ValidateConnectionStrain("WheelBase", wheelDistance, NominalWheelBase, WheelDistanceMinRatio, WheelDistanceMaxRatio, WHEEL_COMPONENT))
            {
                TryLog(LogLevel.W, $"Abnormal wheel distance: {wheelDistance:F1} (nominal: {NominalWheelBase:F1})");
            }

            if (_bike.IsInWheelie)
            {
                ValidatePhysicalParameter("WheelieAngle", _bike.Angle, 0, Math.PI / 1.5, TRICKS_COMPONENT);
                ValidatePhysicalParameter("WheelieTime", _bike.WheelieTime, 0, 60, TRICKS_COMPONENT);
                ValidatePhysicalParameter("WheelieBalance", WheelieBalance, -10, 10, BALANCE_COMPONENT);

                if (_bike.WheelieTime % 1.0 < deltaTime)
                {
                    TryLog(LogLevel.D, $"Wheelie active for {_bike.WheelieTime:F1}s, angle: {_bike.Angle * 180 / Math.PI:F1}°");
                }
            }

            if (_bike.IsInStoppie)
            {
                ValidatePhysicalParameter("StoppieAngle", _bike.Angle, -Math.PI / 1.5, 0, TRICKS_COMPONENT);
                ValidatePhysicalParameter("StoppieTime", _bike.StoppieTime, 0, 30, TRICKS_COMPONENT);
                ValidatePhysicalParameter("StoppieBalance", StoppieBalance, -10, 10, BALANCE_COMPONENT);

                if (_bike.StoppieTime % 1.0 < deltaTime)
                {
                    TryLog(LogLevel.D, $"Stoppie active for {_bike.StoppieTime:F1}s, angle: {_bike.Angle * 180 / Math.PI:F1}°");
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
                                   $"Angle={_bike.Angle * 180 / Math.PI:F1}°, " +
                                   $"State={_bike.State}");
            }

            SanitizePhysicalState();
        }

        private void ApplyRotation(double deltaTime)
        {
            _bike.Angle += _bike.AngularVelocity * deltaTime;
            _bike.Angle = NormalizeAngle(_bike.Angle);
        }

        private void CheckCrashConditions()
        {
            if (_bike.IsInAir || _bike.IsCrashed)
                return;

            double absAngle = Math.Abs(_bike.Angle);
            if (absAngle > CriticalLeanAngle)
            {
                _bike.State |= BikeState.Crashed;
                TryLog(LogLevel.W, $"Bike crashed due to excessive tilt: {_bike.Angle * 180 / Math.PI:F1}°");
                return;
            }

            double currentDistance = CalculateWheelDistance();
            if (currentDistance < MinWheelDistance || currentDistance > MaxWheelDistance)
            {
                _bike.State |= BikeState.Crashed;
                double deformation = Math.Abs(currentDistance - NominalWheelBase) / NominalWheelBase * 100;
                TryLog(LogLevel.W, $"Bike crashed due to frame deformation: {deformation:F1}% (distance: {currentDistance:F1})");
            }
        }

        public void CheckFrameCollision(Level level, double deltaTime)
        {
            _collisionComponent.CheckFrameCollision(level, deltaTime);
        }

        public void HandleWheelCollision(bool isFrontWheel, double groundY, double deltaTime)
        {
            _suspensionComponent.HandleWheelCollision(isFrontWheel, groundY, deltaTime);
        }

        private double CalculateWheelDistance()
        {
            return CalculateDistance(_bike.WheelPositions.Front, _bike.WheelPositions.Rear);
        }

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
            double maxAirAngularVelocity = Math.PI;
            double desiredAngularVelocity = _bike.LeanAmount * maxAirAngularVelocity;
            double damping = 0.5;
            _bike.AngularVelocity += (desiredAngularVelocity - _bike.AngularVelocity) * damping * deltaTime;
        }

        private void UpdateGroundLean(double deltaTime, Level level)
        {
            double slopeAngle = 0;
            if (!_bike.IsInAir)
            {
                slopeAngle = level.CalculateSlopeAngle(_bike.Position.X);
            }

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