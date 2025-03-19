using GravityDefiedGame.Utilities;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Windows;
using System.Windows.Media;
using static GravityDefiedGame.Utilities.GameConstants.Motorcycle;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;

namespace GravityDefiedGame.Models;

public class Motorcycle : PhysicsComponent
{
    #region Fields and Properties
    private readonly BikePhysics _physics;
    private readonly StateController _stateController;
    private readonly InputController _inputController;
    private readonly KinematicsController _kinematicsController;
    private readonly ValidationController _validationController;
    private readonly BikeGeom _geometry;

    public Point Position { get; internal set; }
    public Vector Velocity { get; internal set; }
    public double Angle { get; internal set; }
    public double AngularVelocity { get; internal set; }
    public double FrameHeight { get; private set; }

    public Point FrontWheelPosition { get; internal set; }
    public Point RearWheelPosition { get; internal set; }
    public double WheelBase { get; internal set; } = DefaultWheelBase;
    public double FrontWheelRotation { get; internal set; }
    public double RearWheelRotation { get; internal set; }

    public double Throttle { get; internal set; }
    public double Brake { get; internal set; }
    public double LeanAmount { get; internal set; }

    public BikeType BikeType { get; private set; }
    public Color BikeColor { get; private set; }

    public bool IsCrashed { get; internal set; }
    public bool IsInAir { get; internal set; }
    public bool WasInAir { get; internal set; }
    public bool IsInWheelie { get; internal set; }

    internal bool IsMovingBackward => _stateController.IsMovingBackward;

    public Point FrontAttachmentPoint { get; set; }
    public Point RearAttachmentPoint { get; set; }
    public double FrontSuspensionOffset { get; set; }
    public double RearSuspensionOffset { get; set; }
    #endregion

    public Motorcycle(BikeType bikeType = BikeType.Standard) : base(GameConstants.Debug.MotorcycleTag)
    {
        _physics = new BikePhysics(this);
        BikeType = bikeType;

        _stateController = new StateController(this);
        _inputController = new InputController(this);
        _kinematicsController = new KinematicsController(this);
        _validationController = new ValidationController(this);
        _geometry = new BikeGeom(this);

        InitializeBikeProperties();
        Reset();
    }

    private void InitializeBikeProperties()
    {
        _physics.InitializeProperties(BikeType);
        BikeColor = DefaultBikeColor;
        FrameHeight = _physics.WheelRadius * 1.5;
        Logger.Info(_logTag, $"Initialized {BikeType} motorcycle");
    }

    public void Reset()
    {
        _stateController.Reset();
        _kinematicsController.UpdateAttachmentPoints();
        _kinematicsController.UpdateWheelPositions();
        Logger.Info(_logTag, "Motorcycle reset to initial state");
    }

    public void SetPosition(Point position)
    {
        Position = position;
        _kinematicsController.UpdateAttachmentPoints();
        _kinematicsController.UpdateWheelPositions();
    }

    public void SetBikeType(BikeType bikeType)
    {
        BikeType = bikeType;
        InitializeBikeProperties();
        Logger.Info(_logTag, $"Bike type changed to {bikeType}");
    }

    public void SetBikeColor(Color color) => BikeColor = color;

    public void Update(double deltaTime, Level level, CancellationToken cancellationToken = default)
    {
        if (_stateController.ShouldSkipUpdate(cancellationToken))
            return;

        _validationController.UpdateLogTimer(deltaTime);

        try
        {
            _stateController.SavePreviousState();
            UpdateCycle(deltaTime, level, cancellationToken);
            _stateController.LogStateChanges();
        }
        catch (Exception ex) when (!cancellationToken.IsCancellationRequested)
        {
            _stateController.HandleUpdateException(ex);
        }
    }

    private void UpdateCycle(double deltaTime, Level level, CancellationToken cancellationToken)
    {
        if (cancellationToken.IsCancellationRequested)
            return;

        _stateController.UpdateCollisionState(level, deltaTime);
        _physics.UpdatePhysics(deltaTime, cancellationToken);
        _kinematicsController.UpdateKinematics(deltaTime);
        _validationController.ValidateAndSanitizeState(deltaTime);
    }

    public void ApplyThrottle(double amount) => _inputController.ApplyThrottle(amount);

    public void ApplyBrake(double amount) => _inputController.ApplyBrake(amount);

    public void Lean(double direction) => _inputController.Lean(direction);

    public double GetWheelRadius() => _physics.WheelRadius;

    public List<Point> GetFramePoints() => _geometry.GetFramePoints();

    public (List<BikeGeom.SkeletonPoint> Points, List<BikeGeom.SkeletonLine> Lines) GetSkeleton() =>
        _geometry.GetSkeleton();

    #region Controller Classes
    private class StateController
    {
        private readonly Motorcycle _bike;
        private bool _wasInWheelie;
        private double _airTime;
        private double _brakeHoldTime = 0;
        private bool _isMovingBackward = false;

        public bool IsMovingBackward => _isMovingBackward;

        public StateController(Motorcycle bike)
        {
            _bike = bike;
        }

        public void Reset()
        {
            ResetPositionAndVelocity();
            ResetControlState();
            ResetRotationState();
            ResetSuspension();
        }

        private void ResetPositionAndVelocity()
        {
            _bike.Position = DefaultStartPosition;
            _bike.Velocity = new Vector(0, 0);
        }

        private void ResetControlState()
        {
            (_bike.Throttle, _bike.Brake, _bike.LeanAmount) = (0, 0, 0);
            (_bike.IsCrashed, _bike.IsInAir, _bike.WasInAir, _bike.IsInWheelie, _wasInWheelie) = (false, false, false, false, false);
            (_brakeHoldTime, _isMovingBackward) = (0, false);
        }

        private void ResetRotationState() =>
            (_bike.Angle, _bike.AngularVelocity, _bike.FrontWheelRotation, _bike.RearWheelRotation) = (0, 0, 0, 0);

        private void ResetSuspension()
        {
            double restLength = _bike._physics.SuspensionRestLength;
            _bike.FrontSuspensionOffset = restLength;
            _bike.RearSuspensionOffset = restLength;
            _airTime = 0;
        }

        public bool ShouldSkipUpdate(CancellationToken token) =>
            _bike.IsCrashed || token.IsCancellationRequested;

        public void SavePreviousState()
        {
            _bike.WasInAir = _bike.IsInAir;
            _wasInWheelie = _bike.IsInWheelie;
        }

        public void HandleUpdateException(Exception ex)
        {
            Logger.Error(_bike._logTag, $"Error updating motorcycle: {ex.Message}");
            Logger.Exception(_bike._logTag, ex);
            _bike.IsCrashed = true;
        }

        public void LogStateChanges()
        {
            if (_bike.IsInWheelie != _wasInWheelie)
                _bike.TryLog(LogLevel.D, _bike.IsInWheelie ? "Wheelie started" : "Wheelie ended");

            LogLandingState();
        }

        private void LogLandingState()
        {
            if (_bike.IsInAir || !_bike.WasInAir || _airTime <= SignificantAirTimeThreshold)
                return;

            _bike.TryLog(LogLevel.D, $"Landed after {_airTime:F1}s with speed: {_bike.Velocity.Length:F1}");

            if (_bike.Velocity.Length > DangerLandingVelocity)
                _bike.TryLog(LogLevel.W, $"Hard landing with velocity: {_bike.Velocity.Length:F1}");

            _airTime = 0;
        }

        public void UpdateAirTime(double deltaTime)
        {
            if (!_bike.IsInAir)
                return;

            _airTime += deltaTime;

            if (_airTime > LongAirTimeThreshold)
                _bike.TryLog(LogLevel.D, $"Long air time: {_airTime:F1}s");

            _bike.IsExceedingSafeValue(-_bike.Position.Y, MaxSafeHeight,
                              $"Extreme height detected: {-_bike.Position.Y:F1}");
        }

        public void UpdateCollisionState(Level level, double deltaTime)
        {
            UpdateAirState(level);
            CheckGroundCollisions(level, deltaTime);

            if (!_bike.IsCrashed)
            {
                _bike._physics.CheckFrameCollision(level, deltaTime);
            }
        }

        private void UpdateAirState(Level level)
        {
            (double frontWheelGroundY, double rearWheelGroundY) = (
                level.GetGroundYAtX(_bike.FrontWheelPosition.X),
                level.GetGroundYAtX(_bike.RearWheelPosition.X)
            );

            bool frontInAir = IsWheelInAir(_bike.FrontWheelPosition, frontWheelGroundY);
            bool rearInAir = IsWheelInAir(_bike.RearWheelPosition, rearWheelGroundY);

            bool wasInAir = _bike.IsInAir;
            _bike.IsInAir = frontInAir && rearInAir;

            if (_bike.IsInAir && !wasInAir)
                _bike.TryLog(LogLevel.D, $"Became airborne at speed: {_bike.Velocity.Length:F1}, position: {_bike.Position}");
        }

        private bool IsWheelInAir(Point wheelPosition, double groundY)
        {
            double airThreshold = AirDetectionThreshold * AirDetectionMultiplier;
            return wheelPosition.Y + _bike._physics.WheelRadius < groundY - airThreshold;
        }

        private void CheckGroundCollisions(Level level, double deltaTime)
        {
            CheckCollisionForWheel(level, isFrontWheel: true, deltaTime);
            CheckCollisionForWheel(level, isFrontWheel: false, deltaTime);
        }

        private void CheckCollisionForWheel(Level level, bool isFrontWheel, double deltaTime)
        {
            Point wheelPosition = isFrontWheel ? _bike.FrontWheelPosition : _bike.RearWheelPosition;
            double groundY = level.GetGroundYAtX(wheelPosition.X);

            if (wheelPosition.Y + _bike._physics.WheelRadius >= groundY)
            {
                _bike._physics.HandleWheelCollision(isFrontWheel, groundY, deltaTime);
            }
        }

        public void SetMovingBackward(bool isMovingBackward)
        {
            _isMovingBackward = isMovingBackward;
        }
    }

    private class InputController
    {
        private readonly Motorcycle _bike;

        public InputController(Motorcycle bike)
        {
            _bike = bike;
        }

        public void ApplyThrottle(double amount) => _bike.Throttle = _bike.ClampValue(amount, 0, 1);

        public void ApplyBrake(double amount)
        {
            _bike.Brake = _bike.ClampValue(amount, 0, 1);

            if (_bike.Brake > MinBrakeInput && !_bike.IsInAir &&
                _bike.Velocity.Length < NearStopThreshold)
            {
                Vector bikeDirection = new(Math.Cos(_bike.Angle), Math.Sin(_bike.Angle));
                double forwardSpeed = Vector.Multiply(_bike.Velocity, bikeDirection);

                if (Math.Abs(forwardSpeed) < ReverseStartThreshold || forwardSpeed < 0)
                {
                    _bike.Velocity -= bikeDirection * (_bike.Brake * ReverseForceBase *
                                                FrameTimeApproximation);

                    double currentReverseSpeed = -Vector.Multiply(_bike.Velocity, bikeDirection);
                    if (currentReverseSpeed > MaxReverseSpeed)
                        _bike.Velocity = -bikeDirection * MaxReverseSpeed;

                    _bike._stateController.SetMovingBackward(true);
                }
            }
            else
            {
                _bike._stateController.SetMovingBackward(false);
            }
        }

        public void Lean(double direction) => _bike.LeanAmount = _bike.ClampValue(direction, -1, 1);
    }

    private class KinematicsController
    {
        private readonly Motorcycle _bike;

        public KinematicsController(Motorcycle bike)
        {
            _bike = bike;
        }

        public void UpdateKinematics(double deltaTime)
        {
            UpdatePosition(deltaTime);
            UpdateAttachmentPoints();
            UpdateWheelRotation(deltaTime);
            UpdateWheelPositions();
        }

        public void UpdatePosition(double deltaTime) =>
            _bike.Position = new Point(_bike.Position.X + _bike.Velocity.X * deltaTime, _bike.Position.Y + _bike.Velocity.Y * deltaTime);

        public void UpdateAttachmentPoints()
        {
            double halfWheelBase = _bike.WheelBase / 2;
            (double cosAngle, double sinAngle) = (Math.Cos(_bike.Angle), Math.Sin(_bike.Angle));

            Point rearBase = new Point(
                _bike.Position.X - cosAngle * halfWheelBase,
                _bike.Position.Y - sinAngle * halfWheelBase
            );

            Point frontBase = new Point(
                _bike.Position.X + cosAngle * halfWheelBase,
                _bike.Position.Y + sinAngle * halfWheelBase
            );

            double adjustedFrameHeight = -_bike.FrameHeight;

            _bike.RearAttachmentPoint = new Point(
                rearBase.X - sinAngle * adjustedFrameHeight,
                rearBase.Y + cosAngle * adjustedFrameHeight
            );

            _bike.FrontAttachmentPoint = new Point(
                frontBase.X - sinAngle * adjustedFrameHeight,
                frontBase.Y + cosAngle * adjustedFrameHeight
            );
        }

        public void UpdateWheelPositions()
        {
            _bike.FrontWheelPosition = new Point(
                _bike.FrontAttachmentPoint.X,
                _bike.FrontAttachmentPoint.Y + _bike.FrontSuspensionOffset
            );

            _bike.RearWheelPosition = new Point(
                _bike.RearAttachmentPoint.X,
                _bike.RearAttachmentPoint.Y + _bike.RearSuspensionOffset
            );

            UpdateWheelieState();
        }

        private void UpdateWheelieState() =>
            _bike.IsInWheelie = !_bike.IsInAir &&
                          _bike.FrontWheelPosition.Y < _bike.RearWheelPosition.Y - _bike._physics.WheelRadius * WheelieHeightFactor &&
                          _bike.Angle > WheelieMinAngle;

        private void UpdateWheelRotation(double deltaTime)
        {
            double groundSpeed = Vector.Multiply(_bike.Velocity, new Vector(Math.Cos(_bike.Angle), Math.Sin(_bike.Angle)));
            double wheelCircumference = WheelCircumferenceFactor * _bike._physics.WheelRadius;

            _bike.FrontWheelRotation = UpdateSingleWheelRotation(_bike.FrontWheelRotation, groundSpeed, deltaTime, true, wheelCircumference);
            _bike.RearWheelRotation = UpdateSingleWheelRotation(_bike.RearWheelRotation, groundSpeed, deltaTime, false, wheelCircumference);

            _bike.FrontWheelRotation %= FullRotation;
            _bike.RearWheelRotation %= FullRotation;
        }

        private double UpdateSingleWheelRotation(double currentRotation, double groundSpeed, double deltaTime,
                                              bool isFrontWheel, double wheelCircumference)
        {
            double rotationFactor = _bike.IsInAir
                ? AirRotationFactor
                : GroundRotationFactor;

            double rotationDelta = _bike.SafeDivide(groundSpeed, wheelCircumference);

            rotationDelta = _bike.IsInAir
                ? AdjustAirRotationDelta(rotationDelta, isFrontWheel, wheelCircumference)
                : AdjustGroundRotationDelta(rotationDelta, isFrontWheel);

            return currentRotation + rotationDelta * deltaTime * rotationFactor;
        }

        private double AdjustAirRotationDelta(double rotationDelta, bool isFrontWheel, double wheelCircumference) =>
            isFrontWheel
                ? rotationDelta
                : rotationDelta + _bike.SafeDivide(_bike.Throttle * ThrottleRotationFactor, wheelCircumference);

        private double AdjustGroundRotationDelta(double rotationDelta, bool isFrontWheel)
        {
            if (isFrontWheel)
                return rotationDelta;

            double slipFactor = CalculateWheelSlipFactor();
            rotationDelta *= (1 + slipFactor);

            _bike.CheckConditionWithLog(
                slipFactor > HighWheelSlipThreshold &&
                _bike.Throttle > HighThrottleThreshold,
                LogLevel.D,
                $"High wheel slip: {slipFactor:F2} at throttle {_bike.Throttle:F1}"
            );

            return rotationDelta;
        }

        private double CalculateWheelSlipFactor()
        {
            double slipThreshold = WheelSlipThreshold;

            if (_bike.Throttle <= slipThreshold)
                return 0;

            double slipFactor = (_bike.Throttle - slipThreshold) * SlipThrottleFactor;
            slipFactor *= Math.Min(1.0, _bike._physics.GroundFriction / SlipFrictionRatio);
            slipFactor *= Math.Min(1.0, _bike.Velocity.Length / SlipSpeedThreshold);

            return slipFactor;
        }
    }

    private class ValidationController
    {
        private readonly Motorcycle _bike;

        public ValidationController(Motorcycle bike)
        {
            _bike = bike;
        }

        public void UpdateLogTimer(double deltaTime)
        {
            _bike.UpdateLogTimer(deltaTime);
        }

        public void ValidateAndSanitizeState(double deltaTime)
        {
            SanitizePhysicalState();
            _bike._stateController.UpdateAirTime(deltaTime);
            ValidateState();
        }

        private void SanitizePhysicalState()
        {
            SanitizeSuspension();
            _bike.Position = _bike.SanitizePosition(_bike.Position, DefaultStartPosition, "Invalid position detected");
        }

        private void SanitizeSuspension()
        {
            double restLength = _bike._physics.SuspensionRestLength;
            double minOffset = MinSuspensionOffset;

            if (_bike.FrontSuspensionOffset < minOffset)
            {
                _bike.TryLog(LogLevel.W, $"Correcting invalid front suspension offset: {_bike.FrontSuspensionOffset:F1} → {minOffset:F1}");
                _bike.FrontSuspensionOffset = minOffset;
            }
            else if (_bike.FrontSuspensionOffset > restLength)
            {
                _bike.FrontSuspensionOffset = restLength;
            }

            if (_bike.RearSuspensionOffset < minOffset)
            {
                _bike.TryLog(LogLevel.W, $"Correcting invalid rear suspension offset: {_bike.RearSuspensionOffset:F1} → {minOffset:F1}");
                _bike.RearSuspensionOffset = minOffset;
            }
            else if (_bike.RearSuspensionOffset > restLength)
            {
                _bike.RearSuspensionOffset = restLength;
            }
        }

        private void ValidateState()
        {
            _bike.IsExceedingSafeValue(_bike.Velocity, MaxSafeSpeed,
                              $"Extreme velocity: {_bike.Velocity.Length:F1}");

            ValidateWheelRotation();
            CheckSuspensionCompression();
        }

        private void ValidateWheelRotation()
        {
            if (Math.Abs(_bike.FrontWheelRotation) > MaxSafeRotation ||
                Math.Abs(_bike.RearWheelRotation) > MaxSafeRotation)
                _bike.TryLog(LogLevel.D, $"High wheel rotation: F={_bike.FrontWheelRotation:F1}, R={_bike.RearWheelRotation:F1}");
        }

        private void CheckSuspensionCompression()
        {
            double restLength = _bike._physics.SuspensionRestLength;
            double threshold = HighSuspensionCompressionThreshold;

            double frontCompression = 1.0 - _bike.SafeDivide(_bike.FrontSuspensionOffset, restLength);
            double rearCompression = 1.0 - _bike.SafeDivide(_bike.RearSuspensionOffset, restLength);

            _bike.CheckConditionWithLog(frontCompression > threshold, LogLevel.W,
                $"Front suspension compressed to {frontCompression:P0}");

            _bike.CheckConditionWithLog(rearCompression > threshold, LogLevel.W,
                $"Rear suspension compressed to {rearCompression:P0}");
        }
    }
    #endregion
}