using GravityDefiedGame.Utilities;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Windows;
using System.Windows.Media;
using static GravityDefiedGame.Utilities.GameConstants.Motorcycle;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;

namespace GravityDefiedGame.Models
{
    public class Motorcycle : PhysicsComponent
    {
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
        public (Point Front, Point Rear) WheelPositions
        {
            get => (FrontWheelPosition, RearWheelPosition);
            internal set => (FrontWheelPosition, RearWheelPosition) = value;
        }

        public double WheelBase { get; internal set; } = DefaultWheelBase;
        public double FrontWheelRotation { get; internal set; }
        public double RearWheelRotation { get; internal set; }
        public (double Front, double Rear) WheelRotations
        {
            get => (FrontWheelRotation, RearWheelRotation);
            internal set => (FrontWheelRotation, RearWheelRotation) = value;
        }

        public double Throttle { get; internal set; }
        public double Brake { get; internal set; }
        public double LeanAmount { get; internal set; }
        public BikeType BikeType { get; private set; }
        public Color BikeColor { get; private set; }
        public BikeState State { get; internal set; }

        public bool IsCrashed
        {
            get => (State & BikeState.Crashed) == BikeState.Crashed;
            internal set => State = value ? State | BikeState.Crashed : State & ~BikeState.Crashed;
        }

        public bool IsInAir
        {
            get => (State & BikeState.InAir) == BikeState.InAir;
            internal set => State = value ? State | BikeState.InAir : State & ~BikeState.InAir;
        }

        public bool WasInAir { get; internal set; }

        public bool IsInWheelie
        {
            get => (State & BikeState.InWheelie) == BikeState.InWheelie;
            internal set => State = value ? State | BikeState.InWheelie : State & ~BikeState.InWheelie;
        }

        public bool IsInStoppie
        {
            get => (State & BikeState.InStoppie) == BikeState.InStoppie;
            internal set => State = value ? State | BikeState.InStoppie : State & ~BikeState.InStoppie;
        }

        public bool IsMovingBackward
        {
            get => (State & BikeState.MovingBackward) == BikeState.MovingBackward;
            internal set => State = value ? State | BikeState.MovingBackward : State & ~BikeState.MovingBackward;
        }

        public double WheelieTime { get; internal set; }
        public double StoppieTime { get; internal set; }

        public double WheelieIntensity => IsInWheelie ? Math.Min(1.0, WheelieTime / 1.0) : 0;
        public double StoppieIntensity => IsInStoppie ? Math.Min(1.0, StoppieTime / 0.5) : 0;

        public Point FrontAttachmentPoint { get; set; }
        public Point RearAttachmentPoint { get; set; }
        public (Point Front, Point Rear) AttachmentPoints
        {
            get => (FrontAttachmentPoint, RearAttachmentPoint);
            internal set => (FrontAttachmentPoint, RearAttachmentPoint) = value;
        }

        public double FrontSuspensionOffset { get; set; }
        public double RearSuspensionOffset { get; set; }
        public (double Front, double Rear) SuspensionOffsets
        {
            get => (FrontSuspensionOffset, RearSuspensionOffset);
            internal set => (FrontSuspensionOffset, RearSuspensionOffset) = value;
        }

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
            _physics.UpdatePhysics(deltaTime, level, cancellationToken);
            _kinematicsController.UpdateKinematics(deltaTime);
            _validationController.ValidateAndSanitizeState(deltaTime);
        }

        public void ApplyThrottle(double amount) => _inputController.ApplyThrottle(amount);
        public void ApplyBrake(double amount) => _inputController.ApplyBrake(amount);
        public void Lean(double direction) => _inputController.Lean(direction);
        public double GetWheelRadius() => _physics.WheelRadius;
        public List<Point> GetFramePoints() => _geometry.GetFramePoints();
        public (List<BikeGeom.SkeletonPoint> Points, List<BikeGeom.SkeletonLine> Lines) GetSkeleton() => _geometry.GetSkeleton();

        #region Controllers

        private class StateController
        {
            private readonly Motorcycle _bike;
            private bool _wasInWheelie;
            private bool _wasInStoppie;
            private double _airTime, _brakeHoldTime;

            public StateController(Motorcycle bike)
            {
                _bike = bike;
                _airTime = 0;
                _brakeHoldTime = 0;
            }

            public void Reset()
            {
                _bike.Position = DefaultStartPosition;
                _bike.Velocity = new Vector(0, 0);
                _bike.Throttle = 0;
                _bike.Brake = 0;
                _bike.LeanAmount = 0;
                _bike.State = BikeState.None;
                _bike.WasInAir = false;
                _wasInWheelie = false;
                _wasInStoppie = false;
                _bike.Angle = 0;
                _bike.AngularVelocity = 0;
                _bike.WheelRotations = (0, 0);
                _bike.SuspensionOffsets = (_bike._physics.SuspensionRestLength, _bike._physics.SuspensionRestLength);
                _brakeHoldTime = 0;
                _airTime = 0;
                _bike.WheelieTime = 0;
                _bike.StoppieTime = 0;
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
                Logger.Error(_bike._logTag, $"Error updating motorcycle: {ex.Message}");
                Logger.Exception(_bike._logTag, ex);
                _bike.IsCrashed = true;
            }

            public void LogStateChanges()
            {
                LogWheelieChanges();
                LogStoppieChanges();
                LogLandingState();
            }

            private void LogWheelieChanges()
            {
                if (_bike.IsInWheelie != _wasInWheelie)
                {
                    string msg = _bike.IsInWheelie ? "Wheelie started" : "Wheelie ended";
                    _bike.TryLog(LogLevel.D, msg);
                }
            }

            private void LogStoppieChanges()
            {
                if (_bike.IsInStoppie != _wasInStoppie)
                {
                    string msg = _bike.IsInStoppie ? "Stoppie started" : "Stoppie ended";
                    _bike.TryLog(LogLevel.D, msg);
                }
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

                _bike.IsExceedingSafeValue(-_bike.Position.Y, MaxSafeHeight, $"Extreme height detected: {-_bike.Position.Y:F1}");
            }

            public void UpdateCollisionState(Level level, double deltaTime)
            {
                UpdateAirState(level);
                CheckGroundCollisions(level, deltaTime);
                if (!_bike.IsCrashed)
                    _bike._physics.CheckFrameCollision(level, deltaTime);
            }

            private void UpdateAirState(Level level)
            {
                var wheelPositions = _bike.WheelPositions;
                double frontGroundY = level.GetGroundYAtX(wheelPositions.Front.X);
                double rearGroundY = level.GetGroundYAtX(wheelPositions.Rear.X);

                bool frontWheelInAir = IsWheelInAir(wheelPositions.Front, frontGroundY);
                bool rearWheelInAir = IsWheelInAir(wheelPositions.Rear, rearGroundY);

                bool wasInAir = _bike.IsInAir;
                _bike.IsInAir = frontWheelInAir && rearWheelInAir;

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
                ProcessWheelCollision(level, true, deltaTime);
                ProcessWheelCollision(level, false, deltaTime);
            }

            private void ProcessWheelCollision(Level level, bool isFrontWheel, double deltaTime)
            {
                Point wheelPosition = isFrontWheel ? _bike.WheelPositions.Front : _bike.WheelPositions.Rear;
                double groundY = level.GetGroundYAtX(wheelPosition.X);

                if (wheelPosition.Y + _bike._physics.WheelRadius >= groundY)
                    _bike._physics.HandleWheelCollision(isFrontWheel, groundY, deltaTime);
            }

            public void SetMovingBackward(bool isMovingBackward) =>
                _bike.IsMovingBackward = isMovingBackward;
        }

        private class InputController
        {
            private readonly Motorcycle _bike;

            public InputController(Motorcycle bike) => _bike = bike;

            public void ApplyThrottle(double amount)
            {
                _bike.Throttle = _bike.ClampValue(amount, 0, 1);
            }

            public void ApplyBrake(double amount)
            {
                _bike.Brake = _bike.ClampValue(amount, 0, 1);
                ProcessReverseLogic();
            }

            private void ProcessReverseLogic()
            {
                if (_bike.Brake <= MinBrakeInput || _bike.IsInAir || _bike.Velocity.Length >= NearStopThreshold)
                {
                    _bike.IsMovingBackward = false;
                    return;
                }

                Vector bikeDirection = new(Math.Cos(_bike.Angle), Math.Sin(_bike.Angle));
                double forwardSpeed = Vector.Multiply(_bike.Velocity, bikeDirection);

                if (Math.Abs(forwardSpeed) < ReverseStartThreshold || forwardSpeed < 0)
                {
                    _bike.Velocity -= bikeDirection * (_bike.Brake * ReverseForceBase * FrameTimeApproximation);
                    double currentReverseSpeed = -Vector.Multiply(_bike.Velocity, bikeDirection);
                    if (currentReverseSpeed > MaxReverseSpeed)
                        _bike.Velocity = -bikeDirection * MaxReverseSpeed;

                    _bike.IsMovingBackward = true;
                }
                else
                {
                    _bike.IsMovingBackward = false;
                }
            }

            public void Lean(double direction)
            {
                _bike.LeanAmount = _bike.ClampValue(direction, -1, 1);
            }
        }

        private class KinematicsController
        {
            private readonly Motorcycle _bike;

            public KinematicsController(Motorcycle bike) => _bike = bike;

            public void UpdateKinematics(double deltaTime)
            {
                UpdatePosition(deltaTime);
                UpdateAttachmentPoints();
                UpdateWheelRotations(deltaTime);
                UpdateWheelPositions();
            }

            private void UpdatePosition(double deltaTime)
            {
                _bike.Position = _bike.UpdatePosition(_bike.Position, _bike.Velocity, deltaTime);
            }

            public void UpdateAttachmentPoints()
            {
                double halfWheelBase = _bike.WheelBase / 2;
                var (cosAngle, sinAngle) = PhysicsComponent.GetTrigsFromAngle(_bike.Angle);
                double adjustedFrameHeight = -_bike.FrameHeight;

                Point rearBase = new(
                    _bike.Position.X - cosAngle * halfWheelBase,
                    _bike.Position.Y - sinAngle * halfWheelBase
                );

                Point frontBase = new(
                    _bike.Position.X + cosAngle * halfWheelBase,
                    _bike.Position.Y + sinAngle * halfWheelBase
                );

                _bike.AttachmentPoints = (
                    new Point(frontBase.X - sinAngle * adjustedFrameHeight, frontBase.Y + cosAngle * adjustedFrameHeight),
                    new Point(rearBase.X - sinAngle * adjustedFrameHeight, rearBase.Y + cosAngle * adjustedFrameHeight)
                );
            }

            public void UpdateWheelPositions()
            {
                _bike.WheelPositions = (
                    new Point(_bike.AttachmentPoints.Front.X, _bike.AttachmentPoints.Front.Y + _bike.SuspensionOffsets.Front),
                    new Point(_bike.AttachmentPoints.Rear.X, _bike.AttachmentPoints.Rear.Y + _bike.SuspensionOffsets.Rear)
                );
                UpdateWheelieState();
                UpdateStoppieState();
            }

            private void UpdateWheelieState()
            {
                _bike.IsInWheelie = !_bike.IsInAir &&
                                      _bike.WheelPositions.Front.Y < _bike.WheelPositions.Rear.Y - _bike._physics.WheelRadius * WheelieHeightFactor &&
                                      _bike.Angle > WheelieMinAngle;
            }

            private void UpdateStoppieState()
            {
                _bike.IsInStoppie = !_bike.IsInAir &&
                                      _bike.WheelPositions.Rear.Y < _bike.WheelPositions.Front.Y - _bike._physics.WheelRadius * StoppieHeightFactor &&
                                      _bike.Angle < -StoppieMinAngle;
            }

            private void UpdateWheelRotations(double deltaTime)
            {
                double groundSpeed = Vector.Multiply(_bike.Velocity, new Vector(Math.Cos(_bike.Angle), Math.Sin(_bike.Angle)));
                double wheelCircumference = WheelCircumferenceFactor * _bike._physics.WheelRadius;

                double frontRotation = UpdateSingleWheelRotation(_bike.WheelRotations.Front, groundSpeed, deltaTime, true, wheelCircumference);
                double rearRotation = UpdateSingleWheelRotation(_bike.WheelRotations.Rear, groundSpeed, deltaTime, false, wheelCircumference);

                _bike.WheelRotations = (frontRotation % FullRotation, rearRotation % FullRotation);
            }

            private double UpdateSingleWheelRotation(double currentRotation, double groundSpeed, double deltaTime,
                                                       bool isFrontWheel, double wheelCircumference)
            {
                double rotationFactor = _bike.IsInAir ? AirRotationFactor : GroundRotationFactor;
                double rotationDelta = _bike.SafeDivide(groundSpeed, wheelCircumference);

                rotationDelta = _bike.IsInAir
                    ? AdjustAirRotationDelta(rotationDelta, isFrontWheel, wheelCircumference)
                    : AdjustGroundRotationDelta(rotationDelta, isFrontWheel);

                return currentRotation + rotationDelta * deltaTime * rotationFactor;
            }

            private double AdjustAirRotationDelta(double rotationDelta, bool isFrontWheel, double wheelCircumference)
            {
                return isFrontWheel ? rotationDelta : rotationDelta + _bike.SafeDivide(_bike.Throttle * ThrottleRotationFactor, wheelCircumference);
            }

            private double AdjustGroundRotationDelta(double rotationDelta, bool isFrontWheel)
            {
                if (isFrontWheel)
                    return rotationDelta;

                double slipFactor = CalculateWheelSlipFactor();
                rotationDelta *= (1 + slipFactor);

                _bike.CheckConditionWithLog(
                    slipFactor > HighWheelSlipThreshold && _bike.Throttle > HighThrottleThreshold,
                    LogLevel.D, $"High wheel slip: {slipFactor:F2} at throttle {_bike.Throttle:F1}"
                );

                return rotationDelta;
            }

            private double CalculateWheelSlipFactor()
            {
                if (_bike.Throttle <= WheelSlipThreshold)
                    return 0;

                double slipFactor = (_bike.Throttle - WheelSlipThreshold) * SlipThrottleFactor;
                slipFactor *= Math.Min(1.0, _bike._physics.GroundFriction / SlipFrictionRatio);
                slipFactor *= Math.Min(1.0, _bike.Velocity.Length / SlipSpeedThreshold);
                return slipFactor;
            }
        }

        private class ValidationController
        {
            private readonly Motorcycle _bike;

            public ValidationController(Motorcycle bike) => _bike = bike;

            public void UpdateLogTimer(double deltaTime) => _bike.UpdateLogTimer(deltaTime);

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
                var offsets = _bike.SuspensionOffsets;

                offsets.Front = GetValidOffset(offsets.Front, restLength, minOffset, "front");
                offsets.Rear = GetValidOffset(offsets.Rear, restLength, minOffset, "rear");

                _bike.SuspensionOffsets = offsets;
            }

            private double GetValidOffset(double offset, double restLength, double minOffset, string wheel)
            {
                if (offset < minOffset)
                {
                    _bike.TryLog(LogLevel.W, $"Correcting invalid {wheel} suspension offset: {offset:F1} → {minOffset:F1}");
                    return minOffset;
                }
                if (offset > restLength)
                    return restLength;
                return offset;
            }

            private void ValidateState()
            {
                _bike.IsVectorExceedingSafeValue(_bike.Velocity, MaxSafeSpeed, $"Extreme velocity: {_bike.Velocity.Length:F1}");
                ValidateWheelRotation();
                CheckSuspensionCompression();
            }

            private void ValidateWheelRotation()
            {
                var rotations = _bike.WheelRotations;
                if (Math.Abs(rotations.Front) > MaxSafeRotation || Math.Abs(rotations.Rear) > MaxSafeRotation)
                    _bike.TryLog(LogLevel.D, $"High wheel rotation: F={rotations.Front:F1}, R={rotations.Rear:F1}");
            }

            private void CheckSuspensionCompression()
            {
                double restLength = _bike._physics.SuspensionRestLength;
                double threshold = HighSuspensionCompressionThreshold;
                var offsets = _bike.SuspensionOffsets;

                double frontCompression = 1.0 - _bike.SafeDivide(offsets.Front, restLength);
                double rearCompression = 1.0 - _bike.SafeDivide(offsets.Rear, restLength);

                _bike.CheckConditionWithLog(frontCompression > threshold, LogLevel.W, $"Front suspension compressed to {frontCompression:P0}");
                _bike.CheckConditionWithLog(rearCompression > threshold, LogLevel.W, $"Rear suspension compressed to {rearCompression:P0}");
            }
        }

        #endregion
    }
}