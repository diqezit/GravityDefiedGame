using GravityDefiedGame.Utilities;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Windows;
using static GravityDefiedGame.Utilities.GameConstants;
using static GravityDefiedGame.Utilities.GameConstants.Debug;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;
using static GravityDefiedGame.Utilities.GameConstants.Motorcycle;
using static GravityDefiedGame.Utilities.GameConstants.Wheels;
using static GravityDefiedGame.Utilities.GameConstants.Bike;
using static GravityDefiedGame.Utilities.LoggerCore;
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
        private double _prevThrottle, _prevBrake;

        public ForcesComponent(Motorcycle bike, BikePhysics physics) : base() =>
            (_bike, _physics, _prevThrottle, _prevBrake) = (bike, physics, 0, 0);

        public Vector CalculateThrustForce(double deltaTime)
        {
            _prevThrottle += (_bike.Throttle - _prevThrottle) * Min(1.0, ThrottleTransitionRate * deltaTime);
            double force = _prevThrottle * _physics.EnginePower;
            var (cos, sin) = _physics.GetBikeTrigs();
            return new Vector(cos * force, sin * force);
        }

        public Vector CalculateBrakeForce(double deltaTime)
        {
            if (_bike.Brake <= MinBrakeInput || _bike.Velocity.Length <= 0)
            {
                _prevBrake = Max(0, _prevBrake - BrakeTransitionRate * deltaTime);
                return _prevBrake <= BrakeTransitionThreshold ? new Vector(0, 0) :
                    -_bike.Velocity * (_prevBrake * _physics.BrakeForce / _bike.Velocity.Length);
            }

            _prevBrake += (_bike.Brake - _prevBrake) * Min(1.0, BrakeTransitionRate * deltaTime);
            return -_bike.Velocity * (_prevBrake * _physics.BrakeForce / _bike.Velocity.Length);
        }

        public Vector CalculateDragForce()
        {
            double speed = _bike.Velocity.Length;
            if (speed <= 0) return new Vector(0, 0);

            double drag = _physics.DragCoefficient * (_bike.IsInAir ? AirFrictionMultiplier : 1.0);
            if (!_bike.IsInAir && _bike.Throttle == 0 && _bike.Brake == 0)
                drag *= _prevThrottle > DragThrottleThreshold ? DragThrottleMultiplierBase + _prevThrottle * DragThrottleMultiplier : DragIdleMultiplier;

            return -_bike.Velocity * (drag * speed);
        }

        public Vector CalculateSlopeForce(Level level)
        {
            if (_bike.IsInAir) return new Vector(0, 0);

            double frontAngle = level.CalculateSlopeAngle(_bike.WheelPositions.Front.X);
            double rearAngle = level.CalculateSlopeAngle(_bike.WheelPositions.Rear.X);
            double slopeAngle = level.CalculateSlopeAngle(_bike.Position.X);

            if (Abs(frontAngle - rearAngle) > SlopeAngleDifferenceThreshold)
            {
                double speedFactor = Min(1.0, _bike.Velocity.Length / SpeedFactorDenominator);
                double blend = BlendBase + BlendSpeedFactor * speedFactor;
                slopeAngle = _bike.Velocity.X > 0 ?
                    frontAngle * blend + rearAngle * (1 - blend) :
                    rearAngle * blend + frontAngle * (1 - blend);
            }

            double forceMagnitude = _physics.Mass * _physics.Gravity * Sin(slopeAngle);
            return new Vector(
                forceMagnitude * Cos(slopeAngle),
                -forceMagnitude * Sin(slopeAngle)
            );
        }

        public Vector CalculateTotalForce(Level level, double deltaTime) =>
            new Vector(0, _physics.Gravity * _physics.Mass) +
            CalculateThrustForce(deltaTime) +
            CalculateBrakeForce(deltaTime) +
            CalculateDragForce() +
            CalculateSlopeForce(level);
    }

    public class TorqueComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;
        private double _prevTorque;

        public TorqueComponent(Motorcycle bike, BikePhysics physics) : base() =>
            (_bike, _physics, _prevTorque) = (bike, physics, 0);

        public double CalculateBaseTorque(double deltaTime)
        {
            if (_bike.Throttle <= 0 && _prevTorque <= TorqueIdleThreshold) return 0;

            double force = _bike.Throttle * _physics.EnginePower;
            var (cos, sin) = _physics.GetBikeTrigs();
            Vector thrust = new(cos * force, sin * force);
            Vector r = _bike.WheelPositions.Rear - _bike.Position;
            double torque = r.X * thrust.Y - r.Y * thrust.X;

            double rate = Min(1.0, TorqueSmoothingFactor * deltaTime);
            _prevTorque = _bike.Throttle <= TorqueFadeThreshold && _prevTorque > 0 ?
                _prevTorque * (1.0 - rate) :
                _prevTorque + (torque - _prevTorque) * rate;

            return _prevTorque;
        }

        public double CalculateWheelieTorque(double deltaTime) =>
            CalculateTrickTorque(
                deltaTime, true, _bike.Throttle, WheelieThrottleMinimum,
                WheelieThrottleMultiplier * _physics.EnginePower, WheelieForceBase,
                _physics.EnginePower * WheelieMaxForceMultiplier, CalculateWheelieSpeedEfficiency,
                CalculateWheelieAngleEfficiency, CalculateWheelieBalanceFactor,
                UpdateWheelieBalance);

        public double CalculateStoppieTorque(double deltaTime) =>
            CalculateTrickTorque(
                deltaTime, false, _bike.Brake, StoppieThresholdMinimum,
                StoppieBrakeMultiplier * _physics.BrakeForce, StoppieForceBase,
                _physics.BrakeForce * StoppieMaxForceMultiplier, CalculateStoppieSpeedEfficiency,
                CalculateStoppieAngleEfficiency, CalculateStoppieBalanceFactor,
                UpdateStoppieBalance);

        private double CalculateTrickTorque(
            double deltaTime, bool isWheelie, double inputValue, double minThreshold,
            double forceMultiplier, double baseForce, double maxForce,
            Func<double> speedFunc, Func<double> angleFunc, Func<double> balanceFunc,
            Action<double> updateBalance)
        {
            if (_bike.IsInAir || inputValue <= minThreshold) return 0;

            string trickName = isWheelie ? "wheelie" : "stoppie";
            bool isInTrickState = isWheelie ? _bike.IsInWheelie : _bike.IsInStoppie;
            double torque = 0;
            double speed = _bike.Velocity.Length;

            bool validSpeed = isWheelie ? speed > 0 : speed > StoppieMinSpeed;
            if (validSpeed)
            {
                double efficiency = speedFunc() * angleFunc();
                double force = (inputValue - minThreshold) * forceMultiplier * baseForce * efficiency;
                force = SanitizeValue(force, 0.0, "Invalid trick force calculated");

                if (force > maxForce)
                {
                    WriteLog(LogLevel.W, "BikePhysics", $"Excessive {trickName} force: {force:F1}, clamping to {maxForce:F1}");
                    force = maxForce;
                }

                if (isInTrickState) force *= balanceFunc();
                torque += isWheelie ? force : -force;
            }

            if (isInTrickState)
            {
                updateBalance(deltaTime);
                double controlMult = isWheelie ? WheelieControlMultiplier : StoppieControlMultiplier;
                double balanceStrength = isWheelie ? WheelieBalanceStrength : StoppieBalanceStrength;
                double balanceValue = isWheelie ? _physics.WheelieBalance : _physics.StoppieBalance;

                double control = _bike.LeanAmount * controlMult;
                double balance = balanceValue * balanceStrength + (isWheelie ? control : -control);
                balance = SanitizeValue(balance, 0.0, "Invalid balance torque calculated");

                if (isWheelie && _bike.Brake > 0)
                    balance += _bike.Brake * WheelieStabilizationFactor;
                else if (!isWheelie && _bike.Throttle > 0)
                    balance -= _bike.Throttle * StoppieStabilizationFactor;

                torque += balance;
            }

            return torque;
        }

        private double CalculateEfficiency(
            double value, double minOptimal, double maxOptimal,
            double minValue, double maxValue, double lowEff, double highEff)
        {
            if (value < minValue) return 0.0;
            if (value < minOptimal) return Lerp(lowEff, 1.0, (value - minValue) / (minOptimal - minValue));
            if (value > maxOptimal) return Lerp(1.0, highEff, (value - maxOptimal) / (maxValue - maxValue));
            return 1.0;
        }

        private double CalculateWheelieSpeedEfficiency() =>
            CalculateEfficiency(_bike.Velocity.Length, WheelieOptimalMinSpeed, WheelieOptimalMaxSpeed,
                               0, WheelieMaxSpeed, WheelieLowSpeedEfficiency, WheelieHighSpeedEfficiency);

        private double CalculateStoppieSpeedEfficiency() =>
            CalculateEfficiency(_bike.Velocity.Length, StoppieOptimalMinSpeed, StoppieOptimalMaxSpeed,
                               StoppieMinSpeed, StoppieMaxSpeed, StoppieLowSpeedEfficiency, StoppieHighSpeedEfficiency);

        private double CalculateWheelieAngleEfficiency() =>
            CalculateEfficiency(_bike.Angle, -WheelieOptimalAngle, 0,
                               -WheelieOptimalAngle - WheelieAngleMinOffset, WheelieAngleMaxOffset,
                               WheelieLowAngleEfficiency, WheelieHighAngleEfficiency);

        private double CalculateStoppieAngleEfficiency() =>
            CalculateEfficiency(_bike.Angle, -StoppieOptimalAngle, 0,
                               -StoppieOptimalAngle - StoppieAngleMinOffset, StoppieAngleMaxOffset,
                               StoppieLowAngleEfficiency, StoppieHighAngleEfficiency);

        private double CalculateWheelieBalanceFactor()
        {
            double diff = Abs(_bike.Angle - WheelieBalanceAngle);
            return diff < WheelieBalanceTolerance ? Lerp(WheelieBalanceMinFactor, 1.0, diff / WheelieBalanceTolerance) : 1.0;
        }

        private double CalculateStoppieBalanceFactor()
        {
            double diff = Abs(_bike.Angle - StoppieBalanceAngle);
            return diff < StoppieBalanceTolerance ? Lerp(StoppieBalanceMinFactor, 1.0, diff / StoppieBalanceTolerance) : 1.0;
        }

        private void UpdateWheelieBalance(double deltaTime)
        {
            double diff = _bike.Angle - WheelieBalanceAngle;
            _physics.WheelieBalance = SanitizeValue(-diff * WheelieBalanceResponseFactor,
                                                   0.0, "Invalid wheelie balance value calculated");
            _bike.WheelieTime = SanitizeValue(_bike.WheelieTime + deltaTime, 0.0, "Invalid wheelie time value");

            if (_bike.WheelieTime > WheelieEasyTime)
            {
                double factor = Min(1.0, (_bike.WheelieTime - WheelieEasyTime) / WheelieHardTimeDelta);
                _physics.WheelieBalance *= (1.0 - factor * WheelieProgressiveDifficulty);
            }
        }

        private void UpdateStoppieBalance(double deltaTime)
        {
            double diff = _bike.Angle - StoppieBalanceAngle;
            _physics.StoppieBalance = SanitizeValue(-diff * StoppieBalanceResponseFactor,
                                                   0.0, "Invalid stoppie balance value calculated");
            _bike.StoppieTime = SanitizeValue(_bike.StoppieTime + deltaTime, 0.0, "Invalid stoppie time value");

            if (_bike.StoppieTime > StoppieEasyTime)
            {
                double factor = Min(1.0, (_bike.StoppieTime - StoppieEasyTime) / StoppieHardTimeDelta);
                _physics.StoppieBalance *= (1.0 - factor * StoppieProgressiveDifficulty);
            }
        }

        public double CalculateTotalTorque(double deltaTime) =>
            CalculateBaseTorque(deltaTime) +
            CalculateWheelieTorque(deltaTime) +
            CalculateStoppieTorque(deltaTime);
    }

    public class TricksComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public TricksComponent(Motorcycle bike, BikePhysics physics) => (_bike, _physics) = (bike, physics);

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
            double stoppieTime = _bike.StoppieTime;

            UpdateTrickTime(wasInWheelie, _bike.IsInWheelie, ref wheelieTime, "Wheelie");
            UpdateTrickTime(wasInStoppie, _bike.IsInStoppie, ref stoppieTime, "Stoppie");

            _bike.WheelieTime = wheelieTime;
            _bike.StoppieTime = stoppieTime;
        }

        private void UpdateTrickTime(bool wasActive, bool isActive, ref double trickTime, string trickName)
        {
            if (!isActive && wasActive)
            {
                WriteLog(LogLevel.D, "TricksComponent", $"{trickName} ended after {trickTime:F1}s");
                trickTime = 0;
            }
            else if (isActive && !wasActive)
                WriteLog(LogLevel.D, "TricksComponent", $"{trickName} started");
        }
    }

    public class SuspensionComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;
        private double _prevFrontPenetration, _prevRearPenetration;
        private double _prevFrontCompression, _prevRearCompression;
        private Vector _prevFrontReactionForce, _prevRearReactionForce;

        private const double MinPenetrationThreshold = 0.01;

        public SuspensionComponent(Motorcycle bike, BikePhysics physics) : base()
        {
            _bike = bike;
            _physics = physics;
            _prevFrontReactionForce = _prevRearReactionForce = new Vector(0, 0);
        }

        public void HandleWheelCollisions(Level level, double deltaTime)
        {
            double frontGroundY = level.GetGroundYAtX(_bike.WheelPositions.Front.X);
            double rearGroundY = level.GetGroundYAtX(_bike.WheelPositions.Rear.X);
            double frontSlopeAngle = level.CalculateSlopeAngle(_bike.WheelPositions.Front.X);
            double rearSlopeAngle = level.CalculateSlopeAngle(_bike.WheelPositions.Rear.X);

            bool frontContact = HandleWheelCollision(true, frontGroundY, frontSlopeAngle, deltaTime);
            bool rearContact = HandleWheelCollision(false, rearGroundY, rearSlopeAngle, deltaTime);

            _bike.State = (frontContact || rearContact)
                ? _bike.State & ~BikeState.InAir
                : _bike.State | BikeState.InAir;
        }

        public bool HandleWheelCollision(bool isFrontWheel, double groundY, double slopeAngle, double deltaTime)
        {
            Point wheelPos = isFrontWheel ? _bike.WheelPositions.Front : _bike.WheelPositions.Rear;
            double penetration = wheelPos.Y + _physics.WheelRadius - groundY;

            if (penetration <= 0)
            {
                SetSuspensionOffset(isFrontWheel, _physics.SuspensionRestLength);
                return false;
            }

            ProcessWheelPenetration(isFrontWheel, wheelPos, penetration, slopeAngle, deltaTime);
            return true;
        }

        private void ProcessWheelPenetration(bool isFrontWheel, Point wheelPos, double penetration, double slopeAngle, double deltaTime)
        {
            double prevPenetration = isFrontWheel ? _prevFrontPenetration : _prevRearPenetration;
            double penetrationVelocity = (penetration - prevPenetration) / deltaTime;

            if (isFrontWheel)
                _prevFrontPenetration = penetration;
            else
                _prevRearPenetration = penetration;

            if (penetration < MinPenetrationThreshold)
            {
                SetSuspensionOffset(isFrontWheel, _physics.SuspensionRestLength);
                return;
            }

            penetration = Min(penetration, _physics.WheelRadius * MaxWheelPenetration);
            double baseCompression = penetration * BaseCompressionMultiplier;
            double progressiveFactor = ProgressiveFactorBase + Pow(penetration / (_physics.WheelRadius * WheelRadiusHalfFactor), 2) * ProgressiveFactorMultiplier;
            double desiredCompression = baseCompression * progressiveFactor;

            double prevCompression = isFrontWheel ? _prevFrontCompression : _prevRearCompression;
            double smoothedCompression = Lerp(prevCompression, desiredCompression, CompressionSmoothingFactor);

            if (isFrontWheel)
                _prevFrontCompression = smoothedCompression;
            else
                _prevRearCompression = smoothedCompression;

            double newOffset = _physics.SuspensionRestLength - smoothedCompression;
            double currentOffset = isFrontWheel ? _bike.SuspensionOffsets.Front : _bike.SuspensionOffsets.Rear;

            if (Abs(newOffset - currentOffset) > _physics.SuspensionRestLength * LargeSuspensionChangeThreshold)
                newOffset = Lerp(currentOffset, newOffset, LargeSuspensionChangeSmoothingFactor);

            newOffset = ClampValue(newOffset, _physics.SuspensionRestLength * MinSuspensionOffset, _physics.SuspensionRestLength);

            double compressionRatio = CompressionRatioBase - newOffset / _physics.SuspensionRestLength;
            double progressiveFactorForce = ProgressiveFactorBase + Pow(compressionRatio, 2) * ProgressiveFactorForceMultiplier;
            double adjustedSuspensionStrength = _physics.SuspensionStrength * progressiveFactorForce;

            double penetrationDamping = -_physics.SuspensionDamping * penetrationVelocity;
            double velocityDamping = -_physics.SuspensionDamping * _bike.Velocity.Y * VelocityDampingFactor;

            double normalAngle = slopeAngle + PI / 2;
            Vector normalDirection = new Vector(Cos(normalAngle), Sin(normalAngle));

            Vector suspensionForce = normalDirection * (-adjustedSuspensionStrength * penetration);
            Vector dampingForce = normalDirection * (penetrationDamping + velocityDamping);
            Vector frictionForce = new Vector(-_bike.Velocity.X * _physics.GroundFriction * FrictionForceMultiplier, 0);

            Vector reactionForce = suspensionForce + dampingForce + frictionForce;
            Vector prevReactionForce = isFrontWheel ? _prevFrontReactionForce : _prevRearReactionForce;
            reactionForce = LerpVector(prevReactionForce, reactionForce, ReactionForceSmoothingFactor);

            if (isFrontWheel)
                _prevFrontReactionForce = reactionForce;
            else
                _prevRearReactionForce = reactionForce;

            Vector r = wheelPos - _bike.Position;
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

    public class CollisionComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public CollisionComponent(Motorcycle bike, BikePhysics physics) : base() =>
            (_bike, _physics) = (bike, physics);

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
            bool isHighSpeed = _bike.Velocity.Length > FrameCollisionHighSpeedThreshold;

            if (collisionInfo.MaxPenetration > crashThreshold && (isHighSpeed || isBadAngle))
            {
                _bike.State |= BikeState.Crashed;
                WriteLog(LogLevel.W, "CollisionComponent", $"Frame collision detected at {collisionInfo.CollisionPoint}, " +
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
        private readonly StateComponent _stateComponent;
        private readonly InputComponent _inputComponent;
        private readonly KinematicsComponent _kinematicsComponent;
        private readonly ValidationComponent _validationComponent;
        private readonly BikeGeom _geometryComponent;
        private readonly TrigCache _trigCache = new();

        private double _prevAngularVelocity;
        private Vector _prevVelocity = new(0, 0);
        private int _updateCounter;
        private double _prevSlopeAngle;

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
        public double WheelieBalance { get; set; }
        public double StoppieBalance { get; set; }
        public double MaxSuspensionAngle { get; private set; }
        public (double cos, double sin) GetBikeTrigs() => (_trigCache.Cos, _trigCache.Sin);

        public BikePhysics(Motorcycle bike) : base()
        {
            _bike = bike;
            Gravity = DefaultGravity * GravityMultiplier;
            _prevVelocity = new Vector(0, 0);
            _prevAngularVelocity = 0;
            _updateCounter = 0;
            _prevSlopeAngle = 0;
            WheelieBalance = StoppieBalance = 0.0;

            _forcesComponent = new ForcesComponent(bike, this);
            _torqueComponent = new TorqueComponent(bike, this);
            _tricksComponent = new TricksComponent(bike, this);
            _suspensionComponent = new SuspensionComponent(bike, this);
            _collisionComponent = new CollisionComponent(bike, this);

            _stateComponent = new StateComponent(_bike, this);
            _inputComponent = new InputComponent(bike);
            _kinematicsComponent = new KinematicsComponent(bike, this);
            _validationComponent = new ValidationComponent(bike, this);
            _geometryComponent = new BikeGeom(bike);
        }

        public void InitializeProperties(BikeType bikeType)
        {
            var props = GetBikeProperties(bikeType);
            var wheelProps = GetWheelProperties(bikeType);

            (Mass, EnginePower, BrakeForce, DragCoefficient) = (props.mass, props.power, props.brakeForce, props.drag);
            (MaxLeanAngle, LeanSpeed) = (props.maxLeanAngle, props.leanSpeed);
            GroundFriction = props.friction * wheelProps.friction;
            SuspensionStrength = props.suspensionStrength * wheelProps.suspensionStrength;
            SuspensionDamping = props.suspensionDamping * wheelProps.suspensionDamping;
            SuspensionRestLength = props.suspensionRestLength;
            MaxSuspensionAngle = props.maxSuspensionAngle;
            WheelRadius = wheelProps.radius;
            MomentOfInertia = Mass * Pow(_bike.WheelBase / 2, 2) * MomentOfInertiaMultiplier;
            NominalWheelBase = _bike.WheelBase;
            MinWheelDistance = NominalWheelBase * WheelDistanceMinRatio;
            MaxWheelDistance = NominalWheelBase * WheelDistanceMaxRatio;
        }

        public void Reset()
        {
            _stateComponent.Reset();
            _kinematicsComponent.UpdateAttachmentPoints();
            _kinematicsComponent.UpdateWheelPositions();
        }

        public void SetPosition(Point position)
        {
            _bike.Position = position;
            _kinematicsComponent.UpdateAttachmentPoints();
            _kinematicsComponent.UpdateWheelPositions();
        }

        public void SetBikeType(BikeType bikeType)
        {
            InitializeProperties(bikeType);
            WriteLog(LogLevel.I, "BikePhysics", $"Bike type changed to {bikeType}");
        }

        public void ApplyThrottle(double amount) => _inputComponent.ApplyThrottle(amount);
        public void ApplyBrake(double amount) => _inputComponent.ApplyBrake(amount);
        public void Lean(double direction) => _inputComponent.Lean(direction);

        public List<Point> GetFramePoints() => _geometryComponent.GetFramePoints();
        public (List<BikeGeom.SkeletonPoint> Points, List<BikeGeom.SkeletonLine> Lines) GetSkeleton() =>
            _geometryComponent.GetSkeleton();

        public void Update(double deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            if (_stateComponent.ShouldSkipUpdate(cancellationToken))
                return;

            try
            {
                _stateComponent.SavePreviousState();
                UpdateCycle(deltaTime, level, cancellationToken);
                _stateComponent.LogStateChanges();
            }
            catch (Exception ex) when (!cancellationToken.IsCancellationRequested)
            {
                _stateComponent.HandleUpdateException(ex);
            }
        }

        private void UpdateCycle(double deltaTime, Level level, CancellationToken cancellationToken)
        {
            if (cancellationToken.IsCancellationRequested)
                return;

            UpdatePhysics(deltaTime, level, cancellationToken);
            _kinematicsComponent.UpdateKinematics(deltaTime);
            _validationComponent.ValidateAndSanitizeState(deltaTime);
        }

        public void UpdatePhysics(double deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            if (cancellationToken.IsCancellationRequested) return;

            _trigCache.Update(_bike.Angle);
            SavePreviousState();
            _suspensionComponent.HandleWheelCollisions(level, deltaTime);

            Vector totalForce = _forcesComponent.CalculateTotalForce(level, deltaTime);
            double totalTorque = _torqueComponent.CalculateTotalTorque(deltaTime);

            ValidateVectorParameter("TotalForce", totalForce, Mass * MaxSafeAcceleration, "BikePhysics");
            double maxTorque = MomentOfInertia * MaxAngularVelocity / deltaTime;
            ValidatePhysicalParameter("TotalTorque", Abs(totalTorque), 0, maxTorque, "BikePhysics");

            Vector velocityBefore = _bike.Velocity;
            double angularVelocityBefore = _bike.AngularVelocity;

            _bike.Velocity += totalForce / Mass * deltaTime;
            _bike.AngularVelocity += totalTorque / MomentOfInertia * deltaTime;

            UpdateLean(deltaTime, level);
            _bike.Angle = NormalizeAngle(_bike.Angle + _bike.AngularVelocity * deltaTime);

            _collisionComponent.CheckFrameCollision(level, deltaTime);
            CheckCrashConditions();
            _tricksComponent.UpdateTrickStates(deltaTime);

            if (++_updateCounter >= StatusLogInterval)
            {
                _updateCounter = 0;
                WriteLog(LogLevel.D, "BikePhysics", $"Status: Pos=({_bike.Position.X:F0},{_bike.Position.Y:F0}), " +
                               $"Speed={_bike.Velocity.Length:F1}, " +
                               $"Angle={_bike.Angle * 180 / PI:F1}°, " +
                               $"State={_bike.State}");
            }

            SanitizePhysicalState();
        }

        private void CheckCrashConditions()
        {
            if (_bike.IsInAir || _bike.IsCrashed)
                return;

            if (Abs(_bike.Angle) > CriticalLeanAngle)
            {
                _bike.State |= BikeState.Crashed;
                WriteLog(LogLevel.W, "BikePhysics", $"Bike crashed due to excessive tilt: {_bike.Angle * 180 / PI:F1}°");
                return;
            }

            double currentDistance = CalculateDistance(_bike.WheelPositions.Front, _bike.WheelPositions.Rear);
            if (currentDistance < MinWheelDistance || currentDistance > MaxWheelDistance)
            {
                _bike.State |= BikeState.Crashed;
                double deformation = Abs(currentDistance - NominalWheelBase) / NominalWheelBase * 100;
                WriteLog(LogLevel.W, "BikePhysics", $"Bike crashed due to frame deformation: {deformation:F1}% (distance: {currentDistance:F1})");
            }
        }

        private void UpdateLean(double deltaTime, Level level)
        {
            if (_bike.IsInAir)
            {
                double desiredAngularVelocity = _bike.LeanAmount * MaxAirAngularVelocity;
                _bike.AngularVelocity += (desiredAngularVelocity - _bike.AngularVelocity) * AirDampingFactor * deltaTime;
                return;
            }

            double currentSlopeAngle = level.CalculateSlopeAngle(_bike.Position.X);
            double smoothedSlopeAngle = Abs(currentSlopeAngle - _prevSlopeAngle) > SlopeAngleSmoothingThreshold
                ? _prevSlopeAngle + (currentSlopeAngle - _prevSlopeAngle) * Min(1.0, SlopeTransitionRate * deltaTime)
                : _prevSlopeAngle;
            _prevSlopeAngle = smoothedSlopeAngle;

            double speedFactor = Min(1.0, _bike.Velocity.Length / LeanSpeedFactorDenominator);
            double adaptiveLeanSpeed = LeanSpeed * (LeanSpeedBaseMultiplier + LeanSpeedFactorMultiplier * speedFactor);
            double terrainAdaptationFactor = Min(1.0, _bike.Velocity.Length / TerrainAdaptationSpeedThreshold);
            double slopeInfluence = smoothedSlopeAngle * terrainAdaptationFactor;

            double targetLean = _bike.LeanAmount * MaxLeanAngle + slopeInfluence + _bike.Throttle * ThrottleLeanInfluence;
            double leanError = targetLean - _bike.Angle;
            double angularVelocityDamping = AngularVelocityDampingBase + AngularVelocityDampingFactor * speedFactor;

            double controlTorque = LeanControlTorqueMultiplier * leanError * adaptiveLeanSpeed + angularVelocityDamping * (-_bike.AngularVelocity);

            if (_bike.Throttle < InputIdleThreshold && _bike.Brake < InputIdleThreshold && Abs(_bike.AngularVelocity) > AngularVelocityIdleThreshold)
            {
                double stabilizationFactor = StabilizationFactorBase + StabilizationFactorSpeedMultiplier * Min(1.0, _bike.Velocity.Length / StabilizationSpeedThreshold);
                controlTorque += -_bike.AngularVelocity * stabilizationFactor * StabilizationTorqueMultiplier;
            }

            _bike.AngularVelocity += controlTorque / MomentOfInertia * deltaTime;
            double maxAngularVel = MaxAngularVelocity * GroundAngularVelocityFactor;
            _bike.AngularVelocity = ClampValue(_bike.AngularVelocity, -maxAngularVel, maxAngularVel);
        }

        private void SavePreviousState()
        {
            _prevVelocity = _bike.Velocity;
            _prevAngularVelocity = _bike.AngularVelocity;
        }

        private void SanitizePhysicalState()
        {
            _bike.Angle = NormalizeAngle(SanitizeValue(_bike.Angle, 0, "Invalid angle value"));
            _bike.AngularVelocity = ClampValue(SanitizeValue(_bike.AngularVelocity, 0, "Invalid angular velocity"),
                                             -MaxAngularVelocity, MaxAngularVelocity);
            _bike.Velocity = SanitizeVector(_bike.Velocity, new Vector(0, 0), "Invalid velocity");

            if (_bike.Velocity.Length > MaxSafeVelocity)
            {
                WriteLog(LogLevel.W, "BikePhysics", $"Extreme velocity: {_bike.Velocity.Length:F1}, clamping");
                _bike.Velocity = _bike.Velocity * (MaxSafeVelocity / _bike.Velocity.Length);
            }
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

        private class StateComponent
        {
            private readonly Motorcycle _bike;
            private readonly BikePhysics _physics;
            private bool _wasInWheelie, _wasInStoppie;
            private double _airTime, _brakeHoldTime;

            public StateComponent(Motorcycle bike, BikePhysics physics)
            {
                _bike = bike;
                _physics = physics;
                _airTime = _brakeHoldTime = 0;
            }

            public void Reset()
            {
                _bike.Position = DefaultStartPosition;
                _bike.Velocity = new Vector(0, 0);
                _bike.Throttle = _bike.Brake = _bike.LeanAmount = 0;
                _bike.State = BikeState.None;
                _bike.WasInAir = _wasInWheelie = _wasInStoppie = false;
                _bike.Angle = _bike.AngularVelocity = 0;
                _bike.WheelRotations = (0, 0);
                _bike.SuspensionOffsets = (_physics.SuspensionRestLength, _physics.SuspensionRestLength);
                _brakeHoldTime = _airTime = _bike.WheelieTime = _bike.StoppieTime = 0;
            }

            public bool ShouldSkipUpdate(CancellationToken token) =>
                _bike.IsCrashed || token.IsCancellationRequested;

            public void SavePreviousState()
            {
                _bike.WasInAir = _bike.IsInAir;
                _wasInWheelie = _bike.IsInWheelie;
                _wasInStoppie = _bike.IsInStoppie;
            }

            public void HandleUpdateException(Exception ex)
            {
                WriteLog(LogLevel.E, "BikePhysics", $"Error updating motorcycle: {ex.Message}");
                if (ex.StackTrace != null)
                {
                    var stack = ex.StackTrace.Split(Environment.NewLine, StringSplitOptions.RemoveEmptyEntries);
                    if (stack.Length > 0) WriteLog(LogLevel.D, "BikePhysics", $"Stack: {stack[0]}");
                }
                if (ex.InnerException != null)
                    WriteLog(LogLevel.D, "BikePhysics", $"Inner: {ex.InnerException.Message}");

                _bike.IsCrashed = true;
                WriteLog(LogLevel.E, "BikePhysics", "Critical physics error detected, bike crashed");
            }

            public void LogStateChanges()
            {
                if (_bike.IsInWheelie != _wasInWheelie)
                    WriteLog(LogLevel.D, "BikePhysics", _bike.IsInWheelie ? "Wheelie started" : "Wheelie ended");

                if (_bike.IsInStoppie != _wasInStoppie)
                    WriteLog(LogLevel.D, "BikePhysics", _bike.IsInStoppie ? "Stoppie started" : "Stoppie ended");

                LogLandingState();
            }

            private void LogLandingState()
            {
                if (_bike.IsInAir || !_bike.WasInAir || _airTime <= SignificantAirTimeThreshold)
                    return;

                WriteLog(LogLevel.D, "BikePhysics", $"Landed after {_airTime:F1}s with speed: {_bike.Velocity.Length:F1}");

                if (_bike.Velocity.Length > DangerLandingVelocity)
                    WriteLog(LogLevel.W, "BikePhysics", $"Hard landing with velocity: {_bike.Velocity.Length:F1}");

                _airTime = 0;
            }

            public void UpdateAirTime(double deltaTime)
            {
                if (!_bike.IsInAir)
                    return;

                _airTime += deltaTime;
                if (_airTime > LongAirTimeThreshold && _airTime % 1.0 < deltaTime)
                    WriteLog(LogLevel.D, "BikePhysics", $"Long air time: {_airTime:F1}s");

                if (-_bike.Position.Y > MaxSafeHeight)
                    WriteLog(LogLevel.W, "BikePhysics", $"Extreme height detected: {-_bike.Position.Y:F1}");
            }
        }

        private class InputComponent
        {
            private readonly Motorcycle _bike;

            public InputComponent(Motorcycle bike) => _bike = bike;

            public void ApplyThrottle(double amount) =>
                _bike.Throttle = ClampValue(amount, 0, 1);

            public void ApplyBrake(double amount) =>
                _bike.Brake = ClampValue(amount, 0, 1);

            public void Lean(double direction) =>
                _bike.LeanAmount = ClampValue(direction, -1, 1);
        }

        private class KinematicsComponent
        {
            private readonly Motorcycle _bike;
            private readonly BikePhysics _physics;

            public KinematicsComponent(Motorcycle bike, BikePhysics physics) =>
                (_bike, _physics) = (bike, physics);

            public void UpdateKinematics(double deltaTime)
            {
                UpdateAttachmentPoints();
                UpdateWheelPositions();
                UpdateWheelRotations(deltaTime);
                _bike.Position += _bike.Velocity * deltaTime;
            }

            public void UpdateAttachmentPoints()
            {
                var (cosAngle, sinAngle) = GetTrigsFromAngle(_bike.Angle);
                double halfWheelBase = _bike.WheelBase / 2;

                _bike.AttachmentPoints = (
                    new Point(
                        _bike.Position.X + halfWheelBase * cosAngle,
                        _bike.Position.Y + halfWheelBase * sinAngle
                    ),
                    new Point(
                        _bike.Position.X - halfWheelBase * cosAngle,
                        _bike.Position.Y - halfWheelBase * sinAngle
                    )
                );
            }

            public void UpdateWheelPositions()
            {
                double maxAngle = _physics.MaxSuspensionAngle;
                double bikeAngle = _bike.Angle;
                double verticalAngle = PI / 2;
                double normalAngle = bikeAngle + PI / 2;
                double angleDiff = normalAngle - verticalAngle;

                while (angleDiff > PI) angleDiff -= 2 * PI;
                while (angleDiff < -PI) angleDiff += 2 * PI;

                double suspensionAngle = _bike.EnforceSuspensionAngleLimits
                    ? (Abs(angleDiff) <= maxAngle
                        ? verticalAngle
                        : normalAngle - Sign(angleDiff) * maxAngle)
                    : verticalAngle;

                double frontSuspOffset = _bike.SuspensionOffsets.Front;
                double rearSuspOffset = _bike.SuspensionOffsets.Rear;

                if (_bike.IsInWheelie && !_bike.IsInAir)
                {
                    frontSuspOffset = _physics.SuspensionRestLength -
                        (_physics.SuspensionRestLength - frontSuspOffset) * (1.0 - _bike.WheelieIntensity * WheelieIntensityDampingMultiplier);
                }
                else if (_bike.IsInStoppie && !_bike.IsInAir)
                {
                    rearSuspOffset = _physics.SuspensionRestLength -
                        (_physics.SuspensionRestLength - rearSuspOffset) * (1.0 - _bike.StoppieIntensity * WheelieIntensityDampingMultiplier);
                }

                var (cosSuspAngle, sinSuspAngle) = GetTrigsFromAngle(suspensionAngle);

                _bike.WheelPositions = (
                    new Point(
                        _bike.AttachmentPoints.Front.X + frontSuspOffset * cosSuspAngle,
                        _bike.AttachmentPoints.Front.Y + frontSuspOffset * sinSuspAngle
                    ),
                    new Point(
                        _bike.AttachmentPoints.Rear.X + rearSuspOffset * cosSuspAngle,
                        _bike.AttachmentPoints.Rear.Y + rearSuspOffset * sinSuspAngle
                    )
                );
            }

            private void UpdateWheelRotations(double deltaTime)
            {
                var (cosAngle, sinAngle) = _physics.GetBikeTrigs();
                double groundSpeed = Vector.Multiply(_bike.Velocity, new Vector(cosAngle, sinAngle));
                double wheelCircumference = WheelCircumferenceFactor * _physics.WheelRadius;

                double frontRotation = UpdateSingleWheelRotation(_bike.WheelRotations.Front, groundSpeed, deltaTime, true, wheelCircumference);
                double rearRotation = UpdateSingleWheelRotation(_bike.WheelRotations.Rear, groundSpeed, deltaTime, false, wheelCircumference);

                _bike.WheelRotations = (frontRotation % FullRotation, rearRotation % FullRotation);
            }

            private double UpdateSingleWheelRotation(double currentRotation, double groundSpeed, double deltaTime,
                                                  bool isFrontWheel, double wheelCircumference)
            {
                double rotationFactor = _bike.IsInAir ? AirRotationFactor : GroundRotationFactor;
                double rotationDelta = SafeDivide(groundSpeed, wheelCircumference);

                rotationDelta = _bike.IsInAir
                    ? isFrontWheel
                        ? rotationDelta
                        : rotationDelta + SafeDivide(_bike.Throttle * ThrottleRotationFactor, wheelCircumference)
                    : isFrontWheel
                        ? rotationDelta
                        : rotationDelta * (1 + CalculateWheelSlipFactor());

                return currentRotation + rotationDelta * deltaTime * rotationFactor;
            }

            private double CalculateWheelSlipFactor()
            {
                if (_bike.Throttle <= WheelSlipThreshold)
                    return 0;

                double slipFactor = (_bike.Throttle - WheelSlipThreshold) * SlipThrottleFactor;
                slipFactor *= Min(1.0, _physics.GroundFriction / SlipFrictionRatio);
                slipFactor *= Min(1.0, _bike.Velocity.Length / SlipSpeedThreshold);

                if (slipFactor > HighWheelSlipThreshold && _bike.Throttle > HighThrottleThreshold)
                    WriteLog(LogLevel.D, "BikePhysics", $"High wheel slip: {slipFactor:F2} at throttle {_bike.Throttle:F1}");

                return slipFactor;
            }
        }

        private class ValidationComponent
        {
            private readonly Motorcycle _bike;
            private readonly BikePhysics _physics;
            private double _lastHighSpeedTime, _lastExtremeTiltTime;

            public ValidationComponent(Motorcycle bike, BikePhysics physics)
            {
                _bike = bike;
                _physics = physics;
                _lastHighSpeedTime = _lastExtremeTiltTime = 0;
            }

            public void ValidateAndSanitizeState(double deltaTime)
            {
                SanitizePhysicalState();
                _physics._stateComponent.UpdateAirTime(deltaTime);
                ValidateState(deltaTime);
            }

            private void SanitizePhysicalState()
            {
                SanitizeSuspension();
                _bike.Position = SanitizePosition(_bike.Position, DefaultStartPosition, "Invalid position detected");
            }

            private void SanitizeSuspension()
            {
                double restLength = _physics.SuspensionRestLength;
                double minOffset = MinSuspensionOffset;
                var offsets = _bike.SuspensionOffsets;

                offsets.Front = GetValidOffset(offsets.Front, restLength, minOffset, "front");
                offsets.Rear = GetValidOffset(offsets.Rear, restLength, minOffset, "rear");

                _bike.SuspensionOffsets = offsets;
            }

            private double GetValidOffset(double offset, double restLength, double minOffset, string wheel)
            {
                if (double.IsNaN(offset) || double.IsInfinity(offset))
                {
                    WriteLog(LogLevel.E, "BikePhysics", $"Critical error: Invalid {wheel} suspension offset: {offset}");
                    return restLength;
                }

                if (offset < minOffset)
                {
                    WriteLog(LogLevel.W, "BikePhysics", $"Correcting invalid {wheel} suspension offset: {offset:F1} → {minOffset:F1}");
                    return minOffset;
                }

                return offset > restLength ? restLength : offset;
            }

            private void ValidateState(double deltaTime)
            {
                double speed = _bike.Velocity.Length;
                if (speed > MaxSafeSpeed * SpeedWarningThresholdMultiplier)
                {
                    if (speed > MaxSafeSpeed && _lastHighSpeedTime <= 0)
                    {
                        WriteLog(LogLevel.W, "BikePhysics", $"Extreme velocity detected: {speed:F1} units/s");
                        _lastHighSpeedTime = SpeedWarningCooldown;
                    }
                    else if (speed > MaxSafeSpeed * SpeedCriticalThresholdMultiplier)
                    {
                        WriteLog(LogLevel.E, "BikePhysics", $"Critically high velocity: {speed:F1} units/s");
                        _lastHighSpeedTime = SpeedWarningCooldown;
                    }
                }

                double absAngle = Abs(_bike.Angle);
                if (absAngle > ExtremeTiltAngleThreshold && _lastExtremeTiltTime <= 0)
                {
                    WriteLog(LogLevel.W, "BikePhysics", $"Extreme tilt angle: {_bike.Angle * 180 / PI:F1}°");
                    _lastExtremeTiltTime = TiltWarningCooldown;
                }

                CheckSuspensionCompression();
            }

            private void CheckSuspensionCompression()
            {
                double restLength = _physics.SuspensionRestLength;
                double threshold = HighSuspensionCompressionThreshold;
                var offsets = _bike.SuspensionOffsets;

                double frontCompression = 1.0 - SafeDivide(offsets.Front, restLength);
                double rearCompression = 1.0 - SafeDivide(offsets.Rear, restLength);

                if (frontCompression > threshold)
                    WriteLog(LogLevel.W, "BikePhysics", $"Front suspension compressed to {frontCompression:P0}");

                if (rearCompression > threshold)
                    WriteLog(LogLevel.W, "BikePhysics", $"Rear suspension compressed to {rearCompression:P0}");

                if (frontCompression > CriticalSuspensionCompressionThreshold || rearCompression > CriticalSuspensionCompressionThreshold)
                    WriteLog(LogLevel.E, "BikePhysics", $"Critical suspension compression: front={frontCompression:P0}, rear={rearCompression:P0}");
            }
        }
    }
}