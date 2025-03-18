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
    private Vector _prevVelocity = new(0, 0);
    private double _prevAngularVelocity = 0;

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

    private record BikeConfig(
        double Mass,
        double Power,
        double BrakeForce,
        double Drag,
        double MaxLeanAngle,
        double LeanSpeed,
        double Friction,
        double SuspensionStrength,
        double SuspensionDamping,
        double SuspensionRestLength
    );
    #endregion

    #region Initialization and Configuration
    public BikePhysics(Motorcycle bike) : base(BikePhysicsTag)
    {
        _bike = bike;
        Gravity = DefaultGravity * GravityMultiplier;
    }

    public void InitializeProperties(BikeType bikeType)
    {
        var config = GetBikeConfig(bikeType);
        ApplyBikeConfig(config);
        WheelRadius = DefaultWheelRadius;
        CalculateDerivedProperties();
        ValidateProperties();
    }

    private BikeConfig GetBikeConfig(BikeType bikeType)
    {
        var props = bikeType switch
        {
            BikeType.Standard => GameConstants.Bike.Standard,
            BikeType.Sport => GameConstants.Bike.Sport,
            BikeType.OffRoad => GameConstants.Bike.OffRoad,
            _ => GameConstants.Bike.Standard
        };

        return new(
            props.mass,
            props.power,
            props.brakeForce,
            props.drag,
            props.maxLeanAngle,
            props.leanSpeed,
            props.friction,
            props.suspensionStrength,
            props.suspensionDamping,
            props.suspensionRestLength
        );
    }

    private void ApplyBikeConfig(BikeConfig config)
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
    }

    private void CalculateDerivedProperties() =>
        MomentOfInertia = Mass * Math.Pow(_bike.WheelBase / 2, 2) * MomentOfInertiaMultiplier;

    private void ValidateProperties() { }
    #endregion

    #region Main Update Loop
    public void UpdatePhysics(double deltaTime, CancellationToken cancellationToken = default)
    {
        if (cancellationToken.IsCancellationRequested)
            return;

        SavePreviousState();
        UpdateLean(deltaTime);
        ApplyForces(deltaTime);
        CheckCrashConditions();
        ValidatePhysicalState(deltaTime);
    }

    private void SavePreviousState()
    {
        _prevVelocity = _bike.Velocity;
        _prevAngularVelocity = _bike.AngularVelocity;
    }

    private void ValidatePhysicalState(double deltaTime)
    {
        SanitizeAngleValues();
        _bike.Velocity = SanitizeVector(_bike.Velocity, new Vector(0, 0), "Invalid velocity");
    }

    private void SanitizeAngleValues()
    {
        _bike.Angle = SanitizeValue(_bike.Angle, 0, "Invalid angle value");
        _bike.AngularVelocity = SanitizeValue(_bike.AngularVelocity, 0, "Invalid angular velocity");
    }
    #endregion

    #region Leaning and Rotation
    private void UpdateLean(double deltaTime) =>
        (_bike.IsInAir ? (PhysicsAction<double>)UpdateAirLean : UpdateGroundLean)(deltaTime);

    private void UpdateAirLean(double deltaTime)
    {
        var directControl = _bike.LeanAmount * RotationDirectControl;
        _bike.AngularVelocity += directControl * deltaTime * AirTorqueMultiplier;

        var maxAirVelocity = MaxAngularVelocity * AirRotationMultiplier;
        _bike.AngularVelocity = ClampValue(_bike.AngularVelocity, -maxAirVelocity, maxAirVelocity);

        ApplyRotation(deltaTime);
    }

    private void UpdateGroundLean(double deltaTime)
    {
        var targetLean = _bike.LeanAmount * MaxLeanAngle;
        var currentTarget = Lerp(_bike.Angle, targetLean, LeanSmoothingFactor);

        if (_bike.Throttle > StrongThrottleThreshold && _bike.LeanAmount > 0)
            currentTarget *= WheelieBoostFactor;

        var leanError = currentTarget - _bike.Angle;
        var leanErrorDerivative = -_bike.AngularVelocity;
        var stabilizationForce = StabilizationFactor * _bike.Angle * 1.5;

        var controlTorque = LeanProportionalCoefficient * leanError +
                          LeanDifferentialCoefficient * leanErrorDerivative * 1.2 +
                          stabilizationForce;

        _bike.AngularVelocity += controlTorque / MomentOfInertia * deltaTime;

        var maxAngularVel = MaxAngularVelocity * GroundAngularVelocityFactor;
        _bike.AngularVelocity = ClampValue(_bike.AngularVelocity, -maxAngularVel, maxAngularVel);
        _bike.Angle = ClampValue(_bike.Angle, -MaxLeanAngle, MaxLeanAngle);

        ApplyRotation(deltaTime);
    }

    private void ApplyRotation(double deltaTime)
    {
        _bike.Angle += _bike.AngularVelocity * deltaTime;
        _bike.Angle = NormalizeAngle(_bike.Angle);
    }
    #endregion

    #region Force Calculations
    private void ApplyForces(double deltaTime)
    {
        var gravityForce = new Vector(0, Gravity * Mass);
        var liftForce = new Vector(0, -CalculateLiftForce());
        var wheelieForce = new Vector(0, -CalculateWheelieForce());
        var thrustForce = CalculateThrustForce();
        var brakeForce = CalculateBrakeForce();
        var dragForce = CalculateDragForce();

        var totalForce = gravityForce + liftForce + wheelieForce + thrustForce + brakeForce + dragForce;
        _bike.Velocity += totalForce / Mass * deltaTime;
    }

    private double CalculateLiftForce() =>
        !_bike.IsInAir && _bike.Velocity.Length > LiftSpeedThreshold
            ? Math.Min(
                (_bike.Velocity.Length - LiftSpeedThreshold) * LiftForceMultiplier,
                MaxLiftForce)
            : 0;

    private double CalculateWheelieForce() =>
        !_bike.IsInAir &&
        _bike.Throttle > WheelieThrottleThreshold &&
        _bike.Angle > WheelieMinAngle
            ? EnginePower * _bike.Throttle * WheelieForceMultiplier *
              (1.0 + _bike.Angle * WheelieAngleBoost)
            : 0;

    private Vector CalculateThrustForce()
    {
        var speedBoostFactor = Math.Min(
            LowSpeedBoostMax,
            SafeDivide(
                LowSpeedBoostBase,
                1.0 + Math.Pow(_bike.Velocity.Length / SpeedFactorThreshold, 2)
            )
        );

        var engineForce = _bike.Throttle * EnginePower * (1.0 + speedBoostFactor);
        engineForce = Math.Min(engineForce, EnginePower * EnginePowerLimitMultiplier);

        if (!_bike.IsInAir)
        {
            var stabilityFactor = 1.0 - StabilityAngleFactor *
                                Math.Abs(_bike.Angle) / MaxLeanAngle;
            engineForce *= stabilityFactor;
        }

        var verticalMultiplier = _bike.Angle > WheelieMinAngle &&
                               _bike.Throttle > StrongThrottleThreshold
            ? WheelieVerticalMultiplier : 1.0;

        return new Vector(
            Math.Cos(_bike.Angle) * engineForce,
            Math.Sin(_bike.Angle) * engineForce * verticalMultiplier
        );
    }

    private Vector CalculateBrakeForce()
    {
        if (_bike.Brake <= MinBrakeInput || _bike.Velocity.Length <= 0)
            return new Vector(0, 0);

        if (_bike.IsMovingBackward)
            return new Vector(0, 0);

        Vector bikeDirection = new Vector(Math.Cos(_bike.Angle), Math.Sin(_bike.Angle));

        double forwardSpeed = Vector.Multiply(_bike.Velocity, bikeDirection);

        if (forwardSpeed > 0)
        {
            double brakeEfficiency = BrakeEfficiencyMultiplier;

            if (forwardSpeed < NearStopThreshold * 2)
            {
                brakeEfficiency *= 1.5;
            }

            var brakeForce = _bike.Brake * BrakeForce * brakeEfficiency;
            var brakeVector = -_bike.Velocity;
            return brakeVector * (brakeForce / brakeVector.Length);
        }

        return new Vector(0, 0);
    }

    private Vector CalculateDragForce()
    {
        var speed = _bike.Velocity.Length;
        if (speed <= 0)
            return new Vector(0, 0);

        var dragCoefficient = _bike.IsInAir
            ? DragCoefficient * AirDragReductor
            : DragCoefficient;

        return -_bike.Velocity * (dragCoefficient * speed);
    }
    #endregion

    #region Suspension and Collision
    public void HandleWheelCollision(bool isFrontWheel, double groundY, double deltaTime)
    {
        var wheelPosition = isFrontWheel ? _bike.FrontWheelPosition : _bike.RearWheelPosition;
        var penetration = wheelPosition.Y + WheelRadius - groundY;

        if (penetration <= 0)
        {
            SetSuspensionOffset(isFrontWheel, SuspensionRestLength);
            return;
        }

        var maxPenetration = WheelRadius * MaxWheelPenetration;
        penetration = Math.Min(penetration, maxPenetration);

        var desiredCompression = CalculateDesiredCompression(penetration);
        var currentOffset = isFrontWheel ? _bike.FrontSuspensionOffset : _bike.RearSuspensionOffset;
        var dampingFactor = SuspensionDampingFactor * deltaTime;
        var newOffset = Lerp(currentOffset, SuspensionRestLength - desiredCompression, dampingFactor);

        newOffset = ClampValue(newOffset, SuspensionRestLength * MinSuspensionCompression, SuspensionRestLength);
        var attachmentPoint = isFrontWheel ? _bike.FrontAttachmentPoint : _bike.RearAttachmentPoint;
        var compressionRatio = 1.0 - newOffset / SuspensionRestLength;
        var reactionForce = CalculateCollisionReactionForce(compressionRatio, penetration);

        ApplyReactionForceAndTorque(attachmentPoint, reactionForce, deltaTime);
        SetSuspensionOffset(isFrontWheel, newOffset);
    }

    private double CalculateDesiredCompression(double penetration)
    {
        var baseCompression = penetration * PenetrationBaseMultiplier;
        var progressiveFactor = 1.0 + Math.Pow(
            penetration / (WheelRadius * WheelRadiusHalfFactor), 2) *
            PenetrationProgressiveFactor;

        return baseCompression * progressiveFactor;
    }

    private void ApplyReactionForceAndTorque(Point attachmentPoint, Vector force, double deltaTime)
    {
        var r = attachmentPoint - _bike.Position;
        var torque = r.X * force.Y - r.Y * force.X;

        _bike.Velocity += force / Mass * deltaTime;
        _bike.AngularVelocity += torque / MomentOfInertia * deltaTime;
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
        var progressiveFactor = 1.0 + Math.Pow(compressionRatio, 2) *
                              SuspensionProgressiveFactor;
        var adjustedSuspensionStrength = SuspensionStrength * progressiveFactor;
        var normalForce = adjustedSuspensionStrength * penetration;
        var frictionCoefficient = CalculateFrictionCoefficient(normalForce);

        return new Vector(
            -_bike.Velocity.X * frictionCoefficient,
            -adjustedSuspensionStrength * penetration - SuspensionDamping * _bike.Velocity.Y
        );
    }
    #endregion

    #region Friction and Crash Detection
    private double CalculateFrictionCoefficient(double normalForce)
    {
        var frictionCoefficient = GroundFriction * FrictionMultiplier;
        var loadDependentFriction = frictionCoefficient *
                                  Math.Min(1.0, normalForce / (Mass * NormalForceGravityFactor));
        var speedFactor = 1.0 / (1.0 + Math.Pow(_bike.Velocity.Length / SpeedFrictionThreshold, 2) *
                               SpeedFrictionReductionFactor);
        var effectiveFriction = loadDependentFriction * speedFactor;

        if (!_bike.IsInAir)
        {
            effectiveFriction *= (1.0 + LeanFrictionMultiplier *
                                            Math.Abs(Math.Sin(_bike.Angle)));

            if (_bike.Throttle > MinThrottleForFriction)
            {
                var slipFactor = Math.Max(MinSlipFactor,
                                       1.0 - _bike.Throttle * SlipThrottleMultiplier);
                effectiveFriction *= slipFactor;
            }

            if (_bike.IsMovingBackward)
            {
                effectiveFriction *= ReverseControlMultiplier;
            }
        }

        return effectiveFriction;
    }

    private void CheckCrashConditions()
    {
        if (_bike.IsInAir || _bike.IsCrashed)
            return;

        if (Math.Abs(_bike.Angle) > CriticalLeanAngle)
        {
            _bike.IsCrashed = true;
            Logger.Warning(_logTag, $"Bike crashed due to excessive tilt: {_bike.Angle * 180 / Math.PI:F1}°");
        }
    }
    #endregion
}