using System;
using System.Windows;
using System.Threading;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;
using static GravityDefiedGame.Utilities.GameConstants.Debug;
using static GravityDefiedGame.Utilities.GameConstants;

namespace GravityDefiedGame.Models;

public class BikePhysics : PhysicsComponent
{
    #region Properties and Fields
    private readonly Motorcycle _bike;
    private readonly BikeConfig _config;
    private readonly LeanController _leanController;
    private readonly ForceController _forceController;
    private readonly SuspensionSystem _suspensionSystem;
    private readonly CollisionHandler _collisionHandler;
    private readonly CrashDetector _crashDetector;

    public double Mass => _config.Mass;
    public double EnginePower => _config.EnginePower;
    public double BrakeForce => _config.BrakeForce;
    public double WheelRadius => _config.WheelRadius;
    public double SuspensionRestLength => _config.SuspensionRestLength;
    public double GroundFriction => _config.GroundFriction;
    #endregion

    public BikePhysics(Motorcycle bike) : base(BikePhysicsTag)
    {
        _bike = bike;
        _config = new BikeConfig(this, _bike);
        _leanController = new LeanController(this, _bike, _config);
        _forceController = new ForceController(this, _bike, _config);
        _suspensionSystem = new SuspensionSystem(this, _bike, _config);
        _collisionHandler = new CollisionHandler(this, _bike, _config);
        _crashDetector = new CrashDetector(this, _bike, _config);
    }

    public void InitializeProperties(BikeType bikeType)
    {
        _config.Initialize(bikeType);
    }

    public void UpdatePhysics(double deltaTime, CancellationToken cancellationToken = default)
    {
        if (cancellationToken.IsCancellationRequested) return;

        _leanController.SavePreviousState();
        _leanController.UpdateTransitionFactors(deltaTime);
        _leanController.UpdateLeanControl(deltaTime);
        _leanController.UpdateLean(deltaTime);
        _forceController.ApplyForces(deltaTime);
        _crashDetector.CheckCrashConditions();
        ValidatePhysicalState();
    }

    public void HandleWheelCollision(bool isFrontWheel, double groundY, double deltaTime)
    {
        _suspensionSystem.HandleWheelCollision(isFrontWheel, groundY, deltaTime);
    }

    public void CheckFrameCollision(Level level, double deltaTime)
    {
        _collisionHandler.CheckFrameCollision(level, deltaTime);
    }

    private void ValidatePhysicalState()
    {
        _bike.Angle = SanitizeValue(_bike.Angle, 0, "Invalid angle value");
        _bike.AngularVelocity = SanitizeValue(_bike.AngularVelocity, 0, "Invalid angular velocity");
        _bike.Velocity = SanitizeVector(_bike.Velocity, new Vector(0, 0), "Invalid velocity");
    }

    #region Component Classes
    private class BikeConfig
    {
        private readonly BikePhysics _physics;
        private readonly Motorcycle _bike;

        public double Mass { get; private set; }
        public double EnginePower { get; private set; }
        public double BrakeForce { get; private set; }
        public double Gravity { get; private set; }
        public double DragCoefficient { get; private set; }
        public double WheelRadius { get; private set; } = DefaultWheelRadius;
        public double MaxLeanAngle { get; private set; }
        public double LeanSpeed { get; private set; }
        public double GroundFriction { get; private set; }
        public double SuspensionStrength { get; private set; }
        public double SuspensionDamping { get; private set; }
        public double SuspensionRestLength { get; private set; }
        public double MomentOfInertia { get; private set; }
        public double MinWheelDistance { get; private set; }
        public double MaxWheelDistance { get; private set; }
        public double NominalWheelBase { get; private set; }

        private record BikeConfigData(
            double Mass, double Power, double BrakeForce, double Drag,
            double MaxLeanAngle, double LeanSpeed, double Friction,
            double SuspensionStrength, double SuspensionDamping,
            double SuspensionRestLength, double WheelRadius
        );

        public BikeConfig(BikePhysics physics, Motorcycle bike)
        {
            _physics = physics;
            _bike = bike;
            Gravity = DefaultGravity * GravityMultiplier;
        }

        public void Initialize(BikeType bikeType)
        {
            var config = GetBikeConfig(bikeType);
            ApplyBikeConfig(config);
            CalculateDerivedProperties();
            InitializeWheelDistanceLimits();
        }

        private BikeConfigData GetBikeConfig(BikeType bikeType)
        {
            var props = GetBikeProperties(bikeType);
            var wheelProps = GetWheelProperties(bikeType);

            return new(
                props.mass,
                props.power,
                props.brakeForce,
                props.drag,
                props.maxLeanAngle,
                props.leanSpeed,
                props.friction * wheelProps.friction,
                props.suspensionStrength * wheelProps.suspensionStrength,
                props.suspensionDamping * wheelProps.suspensionDamping,
                props.suspensionRestLength,
                wheelProps.radius
            );
        }

        private BikeProperties GetBikeProperties(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => GameConstants.Bike.Standard,
            BikeType.Sport => GameConstants.Bike.Sport,
            BikeType.OffRoad => GameConstants.Bike.OffRoad,
            _ => GameConstants.Bike.Standard
        };

        private WheelProperties GetWheelProperties(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => GameConstants.Wheels.Standard,
            BikeType.Sport => GameConstants.Wheels.Sport,
            BikeType.OffRoad => GameConstants.Wheels.OffRoad,
            _ => GameConstants.Wheels.Standard
        };

        private void ApplyBikeConfig(BikeConfigData config)
        {
            Mass = config.Mass;
            EnginePower = config.Power;
            BrakeForce = config.BrakeForce;
            DragCoefficient = config.Drag;
            MaxLeanAngle = config.MaxLeanAngle;
            LeanSpeed = config.LeanSpeed;
            GroundFriction = config.Friction;
            SuspensionStrength = config.SuspensionStrength;
            SuspensionDamping = config.SuspensionDamping;
            SuspensionRestLength = config.SuspensionRestLength;
            WheelRadius = config.WheelRadius;
        }

        private void CalculateDerivedProperties() =>
            MomentOfInertia = Mass * Math.Pow(_bike.WheelBase / 2, 2) * MomentOfInertiaMultiplier;

        private void InitializeWheelDistanceLimits()
        {
            NominalWheelBase = _bike.WheelBase;
            MinWheelDistance = NominalWheelBase * WheelDistanceMinRatio;
            MaxWheelDistance = NominalWheelBase * WheelDistanceMaxRatio;
        }
    }

    private class LeanController
    {
        private readonly BikePhysics _physics;
        private readonly Motorcycle _bike;
        private readonly BikeConfig _config;

        private Vector _prevVelocity = new(0, 0);
        private double _prevAngularVelocity = 0;
        private double _airTransitionFactor = 0.0;
        private double _wheelieTransitionFactor = 0.0;
        private double _targetLeanAngle = 0.0;
        private double _currentLeanControl = 0.0;
        private double _landingTimer = 0.0;

        public double AirTransitionFactor => _airTransitionFactor;
        public double WheelieTransitionFactor => _wheelieTransitionFactor;
        public double CurrentLeanControl => _currentLeanControl;

        public LeanController(BikePhysics physics, Motorcycle bike, BikeConfig config)
        {
            _physics = physics;
            _bike = bike;
            _config = config;
        }

        public void SavePreviousState()
        {
            _prevVelocity = _bike.Velocity;
            _prevAngularVelocity = _bike.AngularVelocity;
        }

        public void UpdateTransitionFactors(double deltaTime)
        {
            UpdateAirTransition(deltaTime);
            UpdateWheelieTransition(deltaTime);
        }

        private void UpdateAirTransition(double deltaTime)
        {
            double targetAirFactor = _bike.IsInAir ? 1.0 : 0.0;

            if (_bike.WasInAir && !_bike.IsInAir)
                _landingTimer = LandingTransitionTime;

            if (_landingTimer > 0)
            {
                _landingTimer -= deltaTime;
                _airTransitionFactor = _physics.Lerp(_airTransitionFactor, targetAirFactor, deltaTime * AirToGroundTransitionSpeed);
            }
            else
            {
                double transitionSpeed = _bike.IsInAir ? GroundToAirTransitionSpeed : AirToGroundTransitionSpeed;
                _airTransitionFactor = _physics.Lerp(_airTransitionFactor, targetAirFactor, deltaTime * transitionSpeed);
            }
        }

        private void UpdateWheelieTransition(double deltaTime)
        {
            double targetWheelieFactor = _bike.IsInWheelie ? 1.0 : 0.0;
            _wheelieTransitionFactor = _physics.Lerp(_wheelieTransitionFactor, targetWheelieFactor, deltaTime * WheelieTransitionSpeed);
        }

        public void UpdateLeanControl(double deltaTime)
        {
            _targetLeanAngle = _bike.LeanAmount * _config.MaxLeanAngle;
            double leanControlSpeed = LeanControlSpeed * (1.0 - _airTransitionFactor * AirLeanControlReduction);
            _currentLeanControl = _physics.Lerp(_currentLeanControl, _targetLeanAngle, deltaTime * leanControlSpeed);
        }

        public void UpdateLean(double deltaTime)
        {
            if (_airTransitionFactor < AirTransitionThreshold)
                UpdateGroundLean(deltaTime);
            else
                UpdateAirLean(deltaTime);

            ApplyRotation(deltaTime);
        }

        private void UpdateAirLean(double deltaTime)
        {
            double controlForce = CalculateAirControlForce();
            ApplyAirControlForce(controlForce, deltaTime);
            ApplyGroundInfluence(deltaTime);
            LimitAirAngularVelocity();
        }

        private double CalculateAirControlForce()
        {
            double force = (_currentLeanControl - _bike.Angle) * AirLeanControlFactor;
            return _physics.ClampValue(force, -MaxAirControlForce, MaxAirControlForce);
        }

        private void ApplyAirControlForce(double force, double deltaTime)
        {
            _bike.AngularVelocity += force * deltaTime * AirTorqueMultiplier;
        }

        private void ApplyGroundInfluence(double deltaTime)
        {
            if (_airTransitionFactor < 0.9)
            {
                double groundInfluence = (1.0 - _airTransitionFactor) * 0.5;
                double stabilizationForce = -_bike.Angle * StabilizationFactor * groundInfluence;
                _bike.AngularVelocity += stabilizationForce * deltaTime;
            }
        }

        private void LimitAirAngularVelocity()
        {
            double maxAirVelocity = MaxAngularVelocity * AirRotationMultiplier;
            _bike.AngularVelocity = _physics.ClampValue(_bike.AngularVelocity, -maxAirVelocity, maxAirVelocity);
        }

        private void UpdateGroundLean(double deltaTime)
        {
            double currentTarget = CalculateGroundTargetLean();
            double controlTorque = CalculateGroundControlTorque(currentTarget, deltaTime);
            ApplyAirInfluence(controlTorque, deltaTime);
            LimitGroundAngularVelocity();
            LimitGroundAngle();
        }

        private double CalculateGroundTargetLean()
        {
            double adaptedSmoothingFactor = LeanSmoothingFactor * (_config.LeanSpeed / 5.0);
            double wheelieBoost = _wheelieTransitionFactor * WheelieBoostFactor *
                               (_bike.Throttle > StrongThrottleThreshold ? 1.0 : 0.0);
            double adjustedTarget = _currentLeanControl * (1.0 + wheelieBoost);

            return _physics.Lerp(_bike.Angle, adjustedTarget, adaptedSmoothingFactor);
        }

        private double CalculateGroundControlTorque(double targetLean, double deltaTime)
        {
            double leanError = targetLean - _bike.Angle;
            double leanErrorDerivative = -_bike.AngularVelocity;
            double stabilizationFactor = _bike.Angle < 0 ? StabilizationFactor * 2.0 : StabilizationFactor;
            double stabilizationForce = stabilizationFactor * _bike.Angle * 1.5;

            if (_bike.Angle < -Math.PI / 4)
            {
                stabilizationForce *= 1.5;
                leanErrorDerivative *= 1.5;
            }

            return LeanProportionalCoefficient * leanError * (_config.LeanSpeed / 5.0) +
                   LeanDifferentialCoefficient * leanErrorDerivative * 1.2 +
                   stabilizationForce;
        }

        private void ApplyAirInfluence(double controlTorque, double deltaTime)
        {
            if (_airTransitionFactor > 0.1)
            {
                double airInfluence = _airTransitionFactor * 0.3;
                double airControlForce = (_currentLeanControl - _bike.Angle) * AirLeanControlFactor * airInfluence;
                controlTorque += airControlForce;
            }

            _bike.AngularVelocity += controlTorque / _config.MomentOfInertia * deltaTime;
        }

        private void LimitGroundAngularVelocity()
        {
            double maxAngularVel = MaxAngularVelocity * GroundAngularVelocityFactor;
            _bike.AngularVelocity = _physics.ClampValue(_bike.AngularVelocity, -maxAngularVel, maxAngularVel);
        }

        private void LimitGroundAngle()
        {
            _bike.Angle = _physics.ClampValue(_bike.Angle, -_config.MaxLeanAngle, _config.MaxLeanAngle);
        }

        private void ApplyRotation(double deltaTime)
        {
            _bike.Angle += _bike.AngularVelocity * deltaTime;
            _bike.Angle = NormalizeAngle(_bike.Angle);
        }
    }

    private class ForceController
    {
        private readonly BikePhysics _physics;
        private readonly Motorcycle _bike;
        private readonly BikeConfig _config;
        private readonly LeanController _leanController;

        public ForceController(BikePhysics physics, Motorcycle bike, BikeConfig config)
        {
            _physics = physics;
            _bike = bike;
            _config = config;
            _leanController = physics._leanController;
        }

        public void ApplyForces(double deltaTime)
        {
            var totalForce = CalculateTotalForce();
            _bike.Velocity += totalForce / _config.Mass * deltaTime;
        }

        private Vector CalculateTotalForce()
        {
            return new Vector(0, _config.Gravity * _config.Mass) +
                   new Vector(0, -CalculateLiftForce()) +
                   new Vector(0, -CalculateWheelieForce()) +
                   CalculateThrustForce() +
                   CalculateBrakeForce() +
                   CalculateDragForce();
        }

        private double CalculateLiftForce() =>
            (1.0 - _leanController.AirTransitionFactor) * (_bike.Velocity.Length > LiftSpeedThreshold
                ? Math.Min((_bike.Velocity.Length - LiftSpeedThreshold) * LiftForceMultiplier, MaxLiftForce)
                : 0);

        private double CalculateWheelieForce()
        {
            if (_leanController.AirTransitionFactor > 0.1)
                return 0;

            bool hasWheelieConditions = _bike.Throttle > WheelieThrottleThreshold &&
                                     _bike.Angle > WheelieMinAngle;

            return hasWheelieConditions
                ? _config.EnginePower * _bike.Throttle * WheelieForceMultiplier *
                  (1.0 + _bike.Angle * WheelieAngleBoost) * _leanController.WheelieTransitionFactor
                : 0;
        }

        private Vector CalculateThrustForce()
        {
            double engineForce = CalculateEngineForce();
            double verticalMultiplier = CalculateVerticalMultiplier();

            return new Vector(
                Math.Cos(_bike.Angle) * engineForce,
                Math.Sin(_bike.Angle) * engineForce * verticalMultiplier
            );
        }

        private double CalculateEngineForce()
        {
            var speedBoostFactor = Math.Min(
                LowSpeedBoostMax,
                _physics.SafeDivide(LowSpeedBoostBase, 1.0 + Math.Pow(_bike.Velocity.Length / SpeedFactorThreshold, 2))
            );

            var engineForce = _bike.Throttle * _config.EnginePower * (1.0 + speedBoostFactor);
            engineForce = Math.Min(engineForce, _config.EnginePower * EnginePowerLimitMultiplier);

            if (_leanController.AirTransitionFactor < 0.5)
            {
                var stabilityFactor = 1.0 - StabilityAngleFactor * Math.Abs(_bike.Angle) / _config.MaxLeanAngle;
                engineForce *= stabilityFactor;
            }

            return engineForce;
        }

        private double CalculateVerticalMultiplier()
        {
            return (_leanController.AirTransitionFactor < 0.5 && _bike.Angle > WheelieMinAngle && _bike.Throttle > StrongThrottleThreshold)
                ? 1.0 + (_leanController.WheelieTransitionFactor * (WheelieVerticalMultiplier - 1.0))
                : 1.0;
        }

        private Vector CalculateBrakeForce()
        {
            if (ShouldSkipBrakeForce())
                return new Vector(0, 0);

            Vector bikeDirection = new Vector(Math.Cos(_bike.Angle), Math.Sin(_bike.Angle));
            double forwardSpeed = Vector.Multiply(_bike.Velocity, bikeDirection);

            if (forwardSpeed > 0)
            {
                double brakeEfficiency = CalculateBrakeEfficiency(forwardSpeed);
                var brakeForce = _bike.Brake * _config.BrakeForce * brakeEfficiency;
                var brakeVector = -_bike.Velocity;
                return brakeVector * (_physics.SafeDivide(brakeForce, brakeVector.Length));
            }

            return new Vector(0, 0);
        }

        private bool ShouldSkipBrakeForce() =>
            _bike.Brake <= MinBrakeInput || _bike.Velocity.Length <= 0 || _bike.IsMovingBackward;

        private double CalculateBrakeEfficiency(double forwardSpeed)
        {
            double efficiency = BrakeEfficiencyMultiplier;

            if (forwardSpeed < NearStopThreshold * 2)
                efficiency *= 1.5;

            double airBrakeReduction = 1.0 - (_leanController.AirTransitionFactor * AirBrakeReductionFactor);
            return efficiency * airBrakeReduction;
        }

        private Vector CalculateDragForce()
        {
            var speed = _bike.Velocity.Length;
            if (speed <= 0)
                return new Vector(0, 0);

            var dragCoefficient = _config.DragCoefficient * (1.0 - (_leanController.AirTransitionFactor * (1.0 - AirDragReductor)));
            return -_bike.Velocity * (dragCoefficient * speed);
        }
    }

    private class SuspensionSystem
    {
        private readonly BikePhysics _physics;
        private readonly Motorcycle _bike;
        private readonly BikeConfig _config;

        public SuspensionSystem(BikePhysics physics, Motorcycle bike, BikeConfig config)
        {
            _physics = physics;
            _bike = bike;
            _config = config;
        }

        public void HandleWheelCollision(bool isFrontWheel, double groundY, double deltaTime)
        {
            var wheelPosition = isFrontWheel ? _bike.FrontWheelPosition : _bike.RearWheelPosition;
            var penetration = wheelPosition.Y + _config.WheelRadius - groundY;

            if (penetration <= 0)
            {
                SetSuspensionOffset(isFrontWheel, _config.SuspensionRestLength);
                return;
            }

            ProcessWheelPenetration(isFrontWheel, penetration, deltaTime);
        }

        private void ProcessWheelPenetration(bool isFrontWheel, double penetration, double deltaTime)
        {
            penetration = Math.Min(penetration, _config.WheelRadius * MaxWheelPenetration);
            var desiredCompression = CalculateDesiredCompression(penetration);
            var currentOffset = isFrontWheel ? _bike.FrontSuspensionOffset : _bike.RearSuspensionOffset;
            var dampingFactor = SuspensionDampingFactor * deltaTime;
            var newOffset = _physics.Lerp(currentOffset, _config.SuspensionRestLength - desiredCompression, dampingFactor);

            newOffset = _physics.ClampValue(newOffset, _config.SuspensionRestLength * MinSuspensionCompression, _config.SuspensionRestLength);
            var attachmentPoint = isFrontWheel ? _bike.FrontAttachmentPoint : _bike.RearAttachmentPoint;
            var compressionRatio = 1.0 - _physics.SafeDivide(newOffset, _config.SuspensionRestLength);
            var reactionForce = CalculateCollisionReactionForce(compressionRatio, penetration);

            ApplyReactionForceAndTorque(attachmentPoint, reactionForce, deltaTime);
            SetSuspensionOffset(isFrontWheel, newOffset);
        }

        private double CalculateDesiredCompression(double penetration)
        {
            var baseCompression = penetration * PenetrationBaseMultiplier;
            var progressiveFactor = 1.0 + Math.Pow(
                _physics.SafeDivide(penetration, _config.WheelRadius * WheelRadiusHalfFactor), 2) * PenetrationProgressiveFactor;

            return baseCompression * progressiveFactor;
        }

        private void ApplyReactionForceAndTorque(Point attachmentPoint, Vector force, double deltaTime)
        {
            var r = attachmentPoint - _bike.Position;
            var torque = r.X * force.Y - r.Y * force.X;

            _bike.Velocity += force / _config.Mass * deltaTime;
            _bike.AngularVelocity += torque / _config.MomentOfInertia * deltaTime;
        }

        private void SetSuspensionOffset(bool isFrontWheel, double offset)
        {
            if (isFrontWheel)
                _bike.FrontSuspensionOffset = offset;
            else
                _bike.RearSuspensionOffset = offset;
        }

        private Vector CalculateCollisionReactionForce(double compressionRatio, double penetration)
        {
            var progressiveFactor = 1.0 + Math.Pow(compressionRatio, 2) * SuspensionProgressiveFactor;
            var adjustedSuspensionStrength = _config.SuspensionStrength * progressiveFactor;
            var normalForce = adjustedSuspensionStrength * penetration;
            var frictionCoefficient = CalculateFrictionCoefficient(normalForce);

            return new Vector(
                -_bike.Velocity.X * frictionCoefficient,
                -adjustedSuspensionStrength * penetration - _config.SuspensionDamping * _bike.Velocity.Y
            );
        }

        private double CalculateFrictionCoefficient(double normalForce)
        {
            var frictionCoefficient = _config.GroundFriction * FrictionMultiplier;
            var loadDependentFriction = frictionCoefficient * Math.Min(1.0, _physics.SafeDivide(normalForce, _config.Mass * NormalForceGravityFactor));
            var speedFactor = 1.0 / (1.0 + Math.Pow(_bike.Velocity.Length / SpeedFrictionThreshold, 2) * SpeedFrictionReductionFactor);
            var effectiveFriction = loadDependentFriction * speedFactor;

            if (_physics._leanController.AirTransitionFactor < 0.5)
                effectiveFriction = ApplyGroundFrictionModifiers(effectiveFriction);
            else
                effectiveFriction *= AirFrictionMultiplier;

            return effectiveFriction;
        }

        private double ApplyGroundFrictionModifiers(double baseFriction)
        {
            double result = baseFriction * (1.0 + LeanFrictionMultiplier * Math.Abs(Math.Sin(_bike.Angle)));

            if (_bike.Throttle > MinThrottleForFriction)
            {
                var slipFactor = Math.Max(MinSlipFactor, 1.0 - _bike.Throttle * SlipThrottleMultiplier);
                result *= slipFactor;
            }

            if (_bike.IsMovingBackward)
                result *= ReverseControlMultiplier;

            return result;
        }
    }

    private class CollisionHandler
    {
        private readonly BikePhysics _physics;
        private readonly Motorcycle _bike;
        private readonly BikeConfig _config;

        public CollisionHandler(BikePhysics physics, Motorcycle bike, BikeConfig config)
        {
            _physics = physics;
            _bike = bike;
            _config = config;
        }

        public void CheckFrameCollision(Level level, double deltaTime)
        {
            if (ShouldSkipFrameCollisionCheck())
                return;

            var collisionInfo = DetectFrameCollision(level);
            if (collisionInfo.IsCollision && collisionInfo.MaxPenetration > _config.WheelRadius * FrameCollisionMinPenetration)
                HandleFrameCollision(collisionInfo, deltaTime);
        }

        private bool ShouldSkipFrameCollisionCheck() =>
            _bike.IsInAir || _bike.IsCrashed || _bike.WasInAir || _bike.Velocity.Length <= FrameCollisionMinVelocity;

        private (bool IsCollision, Point CollisionPoint, double MaxPenetration) DetectFrameCollision(Level level)
        {
            var framePoints = _bike.GetFramePoints();
            bool frameCollision = false;
            Point collisionPoint = default;
            double maxPenetration = 0;

            foreach (var point in framePoints)
            {
                double groundY = level.GetGroundYAtX(point.X);

                if (point.Y > groundY)
                {
                    frameCollision = true;
                    double penetration = point.Y - groundY;

                    if (penetration > maxPenetration)
                    {
                        maxPenetration = penetration;
                        collisionPoint = point;
                    }
                }
            }

            return (frameCollision, collisionPoint, maxPenetration);
        }

        private void HandleFrameCollision((bool IsCollision, Point CollisionPoint, double MaxPenetration) collisionInfo, double deltaTime)
        {
            double crashThreshold = _config.WheelRadius * FrameCrashThreshold;
            bool isHighSpeed = _bike.Velocity.Length > 200;
            bool isBadAngle = Math.Abs(_bike.Angle) > FrameCriticalBackwardTiltAngle;

            if (collisionInfo.MaxPenetration > crashThreshold && (isHighSpeed || isBadAngle))
            {
                _bike.IsCrashed = true;
                Logger.Warning(BikePhysicsTag,
                    $"Frame collision detected at {collisionInfo.CollisionPoint}, " +
                    $"penetration: {collisionInfo.MaxPenetration:F1}, " +
                    $"speed: {_bike.Velocity.Length:F1}, " +
                    $"angle: {_bike.Angle * 180 / Math.PI:F1}°");
            }
            else
            {
                ApplyFrameCollisionResponse(collisionInfo.CollisionPoint, collisionInfo.MaxPenetration, deltaTime);
                Logger.Debug(BikePhysicsTag, $"Minor frame collision detected, penetration: {collisionInfo.MaxPenetration:F1}");
            }
        }

        private void ApplyFrameCollisionResponse(Point collisionPoint, double penetration, double deltaTime)
        {
            ApplyVerticalReaction(penetration, deltaTime);
            ApplyAngularStabilization(deltaTime);
            ApplyLowSpeedBoost(deltaTime);
        }

        private void ApplyVerticalReaction(double penetration, double deltaTime)
        {
            double reactionForce = penetration * _config.SuspensionStrength * FrameCollisionReactionForce;
            double impulse = reactionForce * deltaTime;
            double deltaVelocityY = -impulse / _config.Mass;

            deltaVelocityY = Math.Max(deltaVelocityY, -FrameCollisionMaxDeltaVelocity * deltaTime);
            _bike.Velocity = new Vector(_bike.Velocity.X, _bike.Velocity.Y + deltaVelocityY);
        }

        private void ApplyAngularStabilization(double deltaTime)
        {
            double stabilizingFactor = Math.Abs(_bike.Angle) > FrameStabilizingAngleThreshold
                ? FrameStabilizingFactorStrong
                : FrameStabilizingFactorBase;

            double stabilizingTorque = -_bike.Angle * stabilizingFactor;
            double deltaAngularVelocity = stabilizingTorque * deltaTime / _config.MomentOfInertia;

            deltaAngularVelocity = Math.Max(
                Math.Min(deltaAngularVelocity, FrameCollisionMaxDeltaAngular * deltaTime),
                -FrameCollisionMaxDeltaAngular * deltaTime
            );

            _bike.AngularVelocity += deltaAngularVelocity;
        }

        private void ApplyLowSpeedBoost(double deltaTime)
        {
            if (_bike.Velocity.Length < FrameCollisionLowSpeedThreshold)
            {
                double forwardImpulse = FrameCollisionLowSpeedImpulse * deltaTime;
                _bike.Velocity = new Vector(
                    _bike.Velocity.X + forwardImpulse * Math.Cos(_bike.Angle),
                    _bike.Velocity.Y
                );
            }
        }
    }

    private class CrashDetector
    {
        private readonly BikePhysics _physics;
        private readonly Motorcycle _bike;
        private readonly BikeConfig _config;

        public CrashDetector(BikePhysics physics, Motorcycle bike, BikeConfig config)
        {
            _physics = physics;
            _bike = bike;
            _config = config;
        }

        public void CheckCrashConditions()
        {
            if (_bike.IsInAir || _bike.IsCrashed)
                return;

            if (CheckAngleCrashConditions())
                return;

            CheckWheelDistanceCrashCondition();
        }

        private bool CheckAngleCrashConditions()
        {
            if (Math.Abs(_bike.Angle) > CriticalLeanAngle)
            {
                _bike.IsCrashed = true;
                _physics.TryLog(LogLevel.W, $"Bike crashed due to excessive tilt: {_bike.Angle * 180 / Math.PI:F1}°");
                return true;
            }

            if (_bike.Angle < -FrameCriticalBackwardTiltAngle)
            {
                _bike.IsCrashed = true;
                _physics.TryLog(LogLevel.W, $"Bike crashed due to excessive backward tilt: {_bike.Angle * 180 / Math.PI:F1}°");
                return true;
            }

            return false;
        }

        private void CheckWheelDistanceCrashCondition()
        {
            double currentDistance = CalculateWheelDistance();

            if (currentDistance < _config.MinWheelDistance || currentDistance > _config.MaxWheelDistance)
            {
                _bike.IsCrashed = true;
                double deformation = Math.Abs(currentDistance - _config.NominalWheelBase) / _config.NominalWheelBase * 100;
                _physics.TryLog(LogLevel.W, $"Bike crashed due to frame deformation: {deformation:F1}% (distance: {currentDistance:F1})");
            }
        }

        private double CalculateWheelDistance()
        {
            double dx = _bike.FrontWheelPosition.X - _bike.RearWheelPosition.X;
            double dy = _bike.FrontWheelPosition.Y - _bike.RearWheelPosition.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }
    }
    #endregion
}