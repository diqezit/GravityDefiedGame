using GravityDefiedGame.Utilities;
using System.Windows;
using System.Windows.Media;
using static GravityDefiedGame.Utilities.GameConstants.Motorcycle;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Rendering;
using static GravityDefiedGame.Utilities.GameConstants.Validation;

namespace GravityDefiedGame.Models;

public class Motorcycle : PhysicsComponent
{
    private readonly BikePhysics _physics;
    private readonly BikeGeometry _geometry;
    private bool _wasInWheelie;
    private double _airTime;
    private double _brakeHoldTime = 0;
    private bool _isMovingBackward = false;

    public Point Position { get; internal set; }
    public Vector Velocity { get; internal set; }
    public double Angle { get; internal set; }
    public double AngularVelocity { get; internal set; }
    public double FrameHeight { get; private set; }

    public Point FrontWheelPosition { get; internal set; }
    public Point RearWheelPosition { get; internal set; }
    public double WheelBase { get; internal set; } = DefaultWheelBase;
    public double FrontWheelRotation { get; private set; }
    public double RearWheelRotation { get; private set; }

    public double Throttle { get; private set; }
    public double Brake { get; private set; }
    public double LeanAmount { get; private set; }

    public BikeType BikeType { get; private set; }
    public Color BikeColor { get; private set; }

    public bool IsCrashed { get; internal set; }
    public bool IsInAir { get; internal set; }
    public bool WasInAir { get; internal set; }
    public bool IsInWheelie { get; private set; }

    internal bool IsMovingBackward => _isMovingBackward;

    public Point FrontAttachmentPoint { get; set; }
    public Point RearAttachmentPoint { get; set; }
    public double FrontSuspensionOffset { get; set; }
    public double RearSuspensionOffset { get; set; }

    public Motorcycle(BikeType bikeType = BikeType.Standard) : base(GameConstants.Debug.MotorcycleTag)
    {
        _physics = new BikePhysics(this);
        BikeType = bikeType;
        _geometry = new BikeGeometry(this);
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
        ResetPositionAndVelocity();
        ResetControlState();
        ResetRotationState();
        ResetSuspension();

        UpdateAttachmentPoints();
        UpdateWheelPositions();
        Logger.Info(_logTag, "Motorcycle reset to initial state");
    }

    private void ResetPositionAndVelocity()
    {
        Position = DefaultStartPosition;
        Velocity = new Vector(0, 0);
    }

    private void ResetControlState()
    {
        (Throttle, Brake, LeanAmount) = (0, 0, 0);
        (IsCrashed, IsInAir, WasInAir, IsInWheelie, _wasInWheelie) = (false, false, false, false, false);
        (_brakeHoldTime, _isMovingBackward) = (0, false);
    }

    private void ResetRotationState() =>
        (Angle, AngularVelocity, FrontWheelRotation, RearWheelRotation) = (0, 0, 0, 0);

    private void ResetSuspension()
    {
        double restLength = _physics.SuspensionRestLength;
        FrontSuspensionOffset = restLength;
        RearSuspensionOffset = restLength;
        _airTime = 0;
    }

    public void SetPosition(Point position)
    {
        Position = position;
        UpdateAttachmentPoints();
        UpdateWheelPositions();
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
        if (ShouldSkipUpdate(IsCrashed, cancellationToken))
            return;

        UpdateLogTimer(deltaTime);

        try
        {
            SavePreviousState();
            UpdateCycle(deltaTime, level, cancellationToken);
            LogStateChanges();
        }
        catch (Exception ex) when (!cancellationToken.IsCancellationRequested)
        {
            HandleUpdateException(ex);
        }
    }

    private bool ShouldSkipUpdate(bool isCrashed, CancellationToken token) =>
        isCrashed || token.IsCancellationRequested;

    private void SavePreviousState()
    {
        WasInAir = IsInAir;
        _wasInWheelie = IsInWheelie;
    }

    private void HandleUpdateException(Exception ex)
    {
        Logger.Error(_logTag, $"Error updating motorcycle: {ex.Message}");
        Logger.Exception(_logTag, ex);
        IsCrashed = true;
    }

    private void UpdateCycle(double deltaTime, Level level, CancellationToken cancellationToken)
    {
        if (cancellationToken.IsCancellationRequested)
            return;

        UpdateCollisionState(level, deltaTime);
        _physics.UpdatePhysics(deltaTime, cancellationToken);
        UpdateKinematics(deltaTime);
        ValidateAndSanitizeState(deltaTime);
    }

    private void UpdateKinematics(double deltaTime)
    {
        UpdatePosition(deltaTime);
        UpdateAttachmentPoints();
        UpdateWheelRotation(deltaTime);
        UpdateWheelPositions();
    }

    private void ValidateAndSanitizeState(double deltaTime)
    {
        SanitizePhysicalState();
        UpdateAirTime(deltaTime);
        ValidateState();
    }

    private void LogStateChanges()
    {
        if (IsInWheelie != _wasInWheelie)
            TryLog(LogLevel.D, IsInWheelie ? "Wheelie started" : "Wheelie ended");

        LogLandingState();
    }

    private void LogLandingState()
    {
        if (IsInAir || !WasInAir || _airTime <= SignificantAirTimeThreshold)
            return;

        TryLog(LogLevel.D, $"Landed after {_airTime:F1}s with speed: {Velocity.Length:F1}");

        if (Velocity.Length > DangerLandingVelocity)
            TryLog(LogLevel.W, $"Hard landing with velocity: {Velocity.Length:F1}");

        _airTime = 0;
    }

    private void UpdateAirTime(double deltaTime)
    {
        if (!IsInAir)
            return;

        _airTime += deltaTime;

        if (_airTime > LongAirTimeThreshold)
            TryLog(LogLevel.D, $"Long air time: {_airTime:F1}s");

        IsExceedingSafeValue(-Position.Y, MaxSafeHeight,
                          $"Extreme height detected: {-Position.Y:F1}");
    }

    private void SanitizePhysicalState()
    {
        SanitizeSuspension();
        Position = SanitizePosition(Position, DefaultStartPosition, "Invalid position detected");
    }

    private void SanitizeSuspension()
    {
        double restLength = _physics.SuspensionRestLength;
        double minOffset = MinSuspensionOffset;

        if (FrontSuspensionOffset < minOffset)
        {
            TryLog(LogLevel.W, $"Correcting invalid front suspension offset: {FrontSuspensionOffset:F1} → {minOffset:F1}");
            FrontSuspensionOffset = minOffset;
        }
        else if (FrontSuspensionOffset > restLength)
        {
            FrontSuspensionOffset = restLength;
        }

        if (RearSuspensionOffset < minOffset)
        {
            TryLog(LogLevel.W, $"Correcting invalid rear suspension offset: {RearSuspensionOffset:F1} → {minOffset:F1}");
            RearSuspensionOffset = minOffset;
        }
        else if (RearSuspensionOffset > restLength)
        {
            RearSuspensionOffset = restLength;
        }
    }

    private void ValidateState()
    {
        IsExceedingSafeValue(Velocity, MaxSafeSpeed,
                          $"Extreme velocity: {Velocity.Length:F1}");

        ValidateWheelRotation();
        CheckSuspensionCompression();
    }

    private void ValidateWheelRotation()
    {
        if (Math.Abs(FrontWheelRotation) > MaxSafeRotation ||
            Math.Abs(RearWheelRotation) > MaxSafeRotation)
            TryLog(LogLevel.D, $"High wheel rotation: F={FrontWheelRotation:F1}, R={RearWheelRotation:F1}");
    }

    private void CheckSuspensionCompression()
    {
        double restLength = _physics.SuspensionRestLength;
        double threshold = HighSuspensionCompressionThreshold;

        double frontCompression = 1.0 - SafeDivide(FrontSuspensionOffset, restLength);
        double rearCompression = 1.0 - SafeDivide(RearSuspensionOffset, restLength);

        CheckConditionWithLog(frontCompression > threshold, LogLevel.W,
            $"Front suspension compressed to {frontCompression:P0}");

        CheckConditionWithLog(rearCompression > threshold, LogLevel.W,
            $"Rear suspension compressed to {rearCompression:P0}");
    }

    private void UpdatePosition(double deltaTime) =>
        Position = new Point(Position.X + Velocity.X * deltaTime, Position.Y + Velocity.Y * deltaTime);

    private void UpdateAttachmentPoints()
    {
        double halfWheelBase = WheelBase / 2;
        (double cosAngle, double sinAngle) = (Math.Cos(Angle), Math.Sin(Angle));

        Point rearBase = new Point(
            Position.X - cosAngle * halfWheelBase,
            Position.Y - sinAngle * halfWheelBase
        );

        Point frontBase = new Point(
            Position.X + cosAngle * halfWheelBase,
            Position.Y + sinAngle * halfWheelBase
        );

        // Инвертируем FrameHeight, чтобы точки были внизу рамы
        double adjustedFrameHeight = -FrameHeight;

        // Точки крепления подвески к нижней части рамы
        RearAttachmentPoint = new Point(
            rearBase.X - sinAngle * adjustedFrameHeight,
            rearBase.Y + cosAngle * adjustedFrameHeight
        );

        FrontAttachmentPoint = new Point(
            frontBase.X - sinAngle * adjustedFrameHeight,
            frontBase.Y + cosAngle * adjustedFrameHeight
        );
    }

    private void UpdateWheelPositions()
    {
        FrontWheelPosition = new Point(
            FrontAttachmentPoint.X,
            FrontAttachmentPoint.Y + FrontSuspensionOffset
        );

        RearWheelPosition = new Point(
            RearAttachmentPoint.X,
            RearAttachmentPoint.Y + RearSuspensionOffset
        );

        UpdateWheelieState();
    }

    private void UpdateWheelieState() =>
        IsInWheelie = !IsInAir &&
                      FrontWheelPosition.Y < RearWheelPosition.Y - _physics.WheelRadius * WheelieHeightFactor &&
                      Angle > WheelieMinAngle;

    private void UpdateWheelRotation(double deltaTime)
    {
        double groundSpeed = Vector.Multiply(Velocity, new Vector(Math.Cos(Angle), Math.Sin(Angle)));
        double wheelCircumference = WheelCircumferenceFactor * _physics.WheelRadius;

        FrontWheelRotation = UpdateSingleWheelRotation(FrontWheelRotation, groundSpeed, deltaTime, true, wheelCircumference);
        RearWheelRotation = UpdateSingleWheelRotation(RearWheelRotation, groundSpeed, deltaTime, false, wheelCircumference);

        FrontWheelRotation %= FullRotation;
        RearWheelRotation %= FullRotation;
    }

    private double UpdateSingleWheelRotation(double currentRotation, double groundSpeed, double deltaTime,
                                          bool isFrontWheel, double wheelCircumference)
    {
        double rotationFactor = IsInAir
            ? AirRotationFactor
            : GroundRotationFactor;

        double rotationDelta = groundSpeed / wheelCircumference;

        rotationDelta = IsInAir
            ? AdjustAirRotationDelta(rotationDelta, isFrontWheel, wheelCircumference)
            : AdjustGroundRotationDelta(rotationDelta, isFrontWheel);

        return currentRotation + rotationDelta * deltaTime * rotationFactor;
    }

    private double AdjustAirRotationDelta(double rotationDelta, bool isFrontWheel, double wheelCircumference) =>
        isFrontWheel
            ? rotationDelta
            : rotationDelta + Throttle * ThrottleRotationFactor / wheelCircumference;

    private double AdjustGroundRotationDelta(double rotationDelta, bool isFrontWheel)
    {
        if (isFrontWheel)
            return rotationDelta;

        double slipFactor = CalculateWheelSlipFactor();
        rotationDelta *= (1 + slipFactor);

        CheckConditionWithLog(
            slipFactor > HighWheelSlipThreshold &&
            Throttle > HighThrottleThreshold,
            LogLevel.D,
            $"High wheel slip: {slipFactor:F2} at throttle {Throttle:F1}"
        );

        return rotationDelta;
    }

    private double CalculateWheelSlipFactor()
    {
        double slipThreshold = WheelSlipThreshold;

        if (Throttle <= slipThreshold)
            return 0;

        double slipFactor = (Throttle - slipThreshold) * SlipThrottleFactor;
        slipFactor *= Math.Min(1.0, _physics.GroundFriction / SlipFrictionRatio);
        slipFactor *= Math.Min(1.0, Velocity.Length / SlipSpeedThreshold);

        return slipFactor;
    }

    private void UpdateCollisionState(Level level, double deltaTime)
    {
        UpdateAirState(level);
        CheckGroundCollisions(level, deltaTime);
    }

    private void UpdateAirState(Level level)
    {
        (double frontWheelGroundY, double rearWheelGroundY) = (
            level.GetGroundYAtX(FrontWheelPosition.X),
            level.GetGroundYAtX(RearWheelPosition.X)
        );

        bool frontInAir = IsWheelInAir(FrontWheelPosition, frontWheelGroundY);
        bool rearInAir = IsWheelInAir(RearWheelPosition, rearWheelGroundY);

        bool wasInAir = IsInAir;
        IsInAir = frontInAir && rearInAir;

        if (IsInAir && !wasInAir)
            TryLog(LogLevel.D, $"Became airborne at speed: {Velocity.Length:F1}, position: {Position}");
    }

    private bool IsWheelInAir(Point wheelPosition, double groundY)
    {
        double airThreshold = AirDetectionThreshold *
                             AirDetectionMultiplier;

        return wheelPosition.Y + _physics.WheelRadius < groundY - airThreshold;
    }

    private void CheckGroundCollisions(Level level, double deltaTime)
    {
        CheckCollisionForWheel(level, isFrontWheel: true, deltaTime);
        CheckCollisionForWheel(level, isFrontWheel: false, deltaTime);
    }

    private void CheckCollisionForWheel(Level level, bool isFrontWheel, double deltaTime)
    {
        Point wheelPosition = isFrontWheel ? FrontWheelPosition : RearWheelPosition;
        double groundY = level.GetGroundYAtX(wheelPosition.X);

        if (wheelPosition.Y + _physics.WheelRadius >= groundY)
        {
            _physics.HandleWheelCollision(isFrontWheel, groundY, deltaTime);
        }
    }

    public void ApplyThrottle(double amount) => Throttle = ClampValue(amount, 0, 1);

    public void ApplyBrake(double amount)
    {
        Brake = ClampValue(amount, 0, 1);

        if (Brake > MinBrakeInput && !IsInAir &&
            Velocity.Length < NearStopThreshold)
        {
            Vector bikeDirection = new(Math.Cos(Angle), Math.Sin(Angle));
            double forwardSpeed = Vector.Multiply(Velocity, bikeDirection);

            if (Math.Abs(forwardSpeed) < ReverseStartThreshold || forwardSpeed < 0)
            {
                Velocity -= bikeDirection * (Brake * ReverseForceBase *
                                            FrameTimeApproximation);

                double currentReverseSpeed = -Vector.Multiply(Velocity, bikeDirection);
                if (currentReverseSpeed > MaxReverseSpeed)
                    Velocity = -bikeDirection * MaxReverseSpeed;

                _isMovingBackward = true;
            }
        }
        else
        {
            _isMovingBackward = false;
        }
    }

    public void Lean(double direction) => LeanAmount = ClampValue(direction, -1, 1);

    public double GetWheelRadius() => _physics.WheelRadius;

    public (List<BikeGeometry.SkeletonPoint> Points, List<BikeGeometry.SkeletonLine> Lines) GetSkeleton() =>
        _geometry.GetSkeleton();

    public class BikeGeometry
    {
        private readonly Motorcycle _bike;

        public record struct SkeletonPoint(Point Position, SkeletonPointType Type);

        public enum SkeletonPointType
        {
            FrontWheel,
            RearWheel,
            FrontSuspension,
            RearSuspension,
            Frame,
            Seat,
            Handlebar,
            Exhaust,
            Wheel
        }

        public record struct SkeletonLine(int StartPointIndex, int EndPointIndex, SkeletonLineType Type);

        public enum SkeletonLineType
        {
            MainFrame,
            Suspension,
            Wheel,
            Seat,
            Handlebar,
            Exhaust
        }

        public BikeGeometry(Motorcycle bike) => _bike = bike;

        public List<Point> GetChassisPoints()
        {
            double frameTopOffset = _bike.GetWheelRadius() * FrameHeightMultiplier;
            double cosAngle = Math.Cos(_bike.Angle), sinAngle = Math.Sin(_bike.Angle);

            Point frontLower = new(
                _bike.FrontAttachmentPoint.X + sinAngle * frameTopOffset * LowerFrameOffsetMultiplier,
                _bike.FrontAttachmentPoint.Y - cosAngle * frameTopOffset * LowerFrameOffsetMultiplier
            );

            Point rearLower = new(
                _bike.RearAttachmentPoint.X + sinAngle * frameTopOffset * LowerFrameOffsetMultiplier,
                _bike.RearAttachmentPoint.Y - cosAngle * frameTopOffset * LowerFrameOffsetMultiplier
            );

            Point frontUpper = new(
                frontLower.X + sinAngle * frameTopOffset * UpperFrameOffsetMultiplier,
                frontLower.Y - cosAngle * frameTopOffset * UpperFrameOffsetMultiplier
            );

            Point rearUpper = new(
                rearLower.X + sinAngle * frameTopOffset * UpperFrameOffsetMultiplier,
                rearLower.Y - cosAngle * frameTopOffset * UpperFrameOffsetMultiplier
            );

            return new List<Point> { frontLower, rearLower, rearUpper, frontUpper };
        }

        public (List<SkeletonPoint> Points, List<SkeletonLine> Lines) GetSkeleton()
        {
            var points = new List<SkeletonPoint>();
            var lines = new List<SkeletonLine>();

            AddBasicSkeletonElements(points, lines);
            AddDetailSkeletonElements(points, lines);

            return (points, lines);
        }

        private void AddBasicSkeletonElements(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            points.Add(new SkeletonPoint(_bike.FrontWheelPosition, SkeletonPointType.FrontWheel));
            points.Add(new SkeletonPoint(_bike.RearWheelPosition, SkeletonPointType.RearWheel));
            points.Add(new SkeletonPoint(_bike.FrontAttachmentPoint, SkeletonPointType.FrontSuspension));
            points.Add(new SkeletonPoint(_bike.RearAttachmentPoint, SkeletonPointType.RearSuspension));

            var chassisPoints = GetChassisPoints();
            int frameStartIndex = points.Count;

            points.AddRange(chassisPoints.Select(point => new SkeletonPoint(point, SkeletonPointType.Frame)));

            int frontLowerFrameIndex = frameStartIndex;
            int rearLowerFrameIndex = frameStartIndex + 1;

            lines.Add(new SkeletonLine(0, 2, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(1, 3, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(2, frontLowerFrameIndex, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(3, rearLowerFrameIndex, SkeletonLineType.Suspension));

            int frameCount = chassisPoints.Count;
            for (int i = frameStartIndex; i < frameStartIndex + frameCount - 1; i++)
                lines.Add(new SkeletonLine(i, i + 1, SkeletonLineType.MainFrame));

            lines.Add(new SkeletonLine(frameStartIndex + frameCount - 1, frameStartIndex, SkeletonLineType.MainFrame));

            AddWheelSpokes(points, lines, 0, _bike.FrontWheelPosition, _bike.FrontWheelRotation);
            AddWheelSpokes(points, lines, 1, _bike.RearWheelPosition, _bike.RearWheelRotation);
        }

        private void AddWheelSpokes(List<SkeletonPoint> points, List<SkeletonLine> lines, int wheelIndex,
                                   Point wheelCenter, double wheelRotation)
        {
            const int spokeCount = 16;
            double wheelRadius = _bike.GetWheelRadius();
            int centerPointIndex = wheelIndex;
            var spokeEndIndices = new List<int>(spokeCount);

            for (int i = 0; i < spokeCount; i++)
            {
                double angle = wheelRotation + i * (2 * Math.PI / spokeCount);
                Point spokeEnd = new(
                    wheelCenter.X + wheelRadius * Math.Cos(angle),
                    wheelCenter.Y + wheelRadius * Math.Sin(angle)
                );
                points.Add(new SkeletonPoint(spokeEnd, SkeletonPointType.Wheel));
                int spokeEndIndex = points.Count - 1;
                spokeEndIndices.Add(spokeEndIndex);
                lines.Add(new SkeletonLine(centerPointIndex, spokeEndIndex, SkeletonLineType.Wheel));
            }

            for (int i = 0; i < spokeCount; i++)
                lines.Add(new SkeletonLine(spokeEndIndices[i], spokeEndIndices[(i + 1) % spokeCount], SkeletonLineType.Wheel));
        }

        private void AddDetailSkeletonElements(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            int basePointsCount = points.Count;

            AddDetailPoints(points, CreateSeatPoints(), SkeletonPointType.Seat);
            AddDetailPoints(points, CreateHandlebarPoints(), SkeletonPointType.Handlebar);
            AddDetailPoints(points, CreateExhaustPoints(), SkeletonPointType.Exhaust);

            AddClosedPolygonLines(lines, basePointsCount, 4, SkeletonLineType.Seat);
            AddClosedPolygonLines(lines, basePointsCount + 4, 4, SkeletonLineType.Handlebar);
            AddClosedPolygonLines(lines, basePointsCount + 8, 4, SkeletonLineType.Exhaust);
        }

        private void AddDetailPoints(List<SkeletonPoint> points, List<Point> detailPoints, SkeletonPointType type) =>
            points.AddRange(detailPoints.Select(p => new SkeletonPoint(p, type)));

        private List<Point> CreateSeatPoints()
        {
            double cosAngle = Math.Cos(_bike.Angle), sinAngle = Math.Sin(_bike.Angle);
            var chassisPoints = GetChassisPoints();
            Point rearUpper = chassisPoints[2], frontUpper = chassisPoints[3];

            Point seatBase = new(
                (frontUpper.X + rearUpper.X) / 2 - 5 * cosAngle - 8 * sinAngle,
                (frontUpper.Y + rearUpper.Y) / 2 - 5 * sinAngle + 8 * cosAngle
            );

            return new List<Point>
        {
            new(seatBase.X + SeatOffsetX1 * 0.8 * cosAngle - SeatOffsetY1 * 0.7 * sinAngle,
                seatBase.Y + SeatOffsetX1 * 0.8 * sinAngle + SeatOffsetY1 * 0.7 * cosAngle),

            new(seatBase.X + SeatOffsetX2 * 0.9 * cosAngle - SeatOffsetY1 * 0.7 * sinAngle,
                seatBase.Y + SeatOffsetX2 * 0.9 * sinAngle + SeatOffsetY1 * 0.7 * cosAngle),

            new(seatBase.X + SeatOffsetX3 * 1.2 * cosAngle - SeatOffsetY2 * 0.8 * sinAngle,
                seatBase.Y + SeatOffsetX3 * 1.2 * sinAngle + SeatOffsetY2 * 0.8 * cosAngle),

            new(seatBase.X + SeatOffsetX4 * 0.8 * cosAngle - SeatOffsetY2 * 0.8 * sinAngle,
                seatBase.Y + SeatOffsetX4 * 0.8 * sinAngle + SeatOffsetY2 * 0.8 * cosAngle)
        };
        }

        private List<Point> CreateHandlebarPoints()
        {
            double cosAngle = Math.Cos(_bike.Angle), sinAngle = Math.Sin(_bike.Angle);
            var chassisPoints = GetChassisPoints();
            Point frontUpper = chassisPoints[3];

            double baseWidth = HandlebarOffsetX3 * 1.8;
            double baseHeight = HandlebarOffsetY1 * 0.5;
            double handleHeight = HandlebarOffsetY1 * 1.5;

            return new List<Point>
        {
            new(frontUpper.X - baseWidth * cosAngle - handleHeight * sinAngle,
                frontUpper.Y - baseWidth * sinAngle + handleHeight * cosAngle),

            new(frontUpper.X - baseWidth * 0.4 * cosAngle - baseHeight * sinAngle,
                frontUpper.Y - baseWidth * 0.4 * sinAngle + baseHeight * cosAngle),

            new(frontUpper.X + baseWidth * 0.4 * cosAngle - baseHeight * sinAngle,
                frontUpper.Y + baseWidth * 0.4 * sinAngle + baseHeight * cosAngle),

            new(frontUpper.X + baseWidth * cosAngle - handleHeight * sinAngle,
                frontUpper.Y + baseWidth * sinAngle + handleHeight * cosAngle)
        };
        }

        private List<Point> CreateExhaustPoints()
        {
            double cosAngle = Math.Cos(_bike.Angle), sinAngle = Math.Sin(_bike.Angle);
            var chassisPoints = GetChassisPoints();
            Point exhaustBase = chassisPoints[1];

            double pipeLength = 15, pipeHeight = 1, downOffset = 1, rightOffset = 1;

            return new List<Point>
        {
            new(exhaustBase.X, exhaustBase.Y),

            new(exhaustBase.X - rightOffset * sinAngle,
                exhaustBase.Y + rightOffset * cosAngle + downOffset),

            new(exhaustBase.X - pipeLength * cosAngle - (rightOffset + pipeHeight) * sinAngle,
                exhaustBase.Y - pipeLength * sinAngle + (rightOffset + pipeHeight) * cosAngle + downOffset),

            new(exhaustBase.X - pipeLength * cosAngle - rightOffset * sinAngle,
                exhaustBase.Y - pipeLength * sinAngle + rightOffset * cosAngle)
        };
        }

        private void AddClosedPolygonLines(
            List<SkeletonLine> lines,
            int startIndex,
            int pointCount,
            SkeletonLineType lineType)
        {
            for (int i = 0; i < pointCount; i++)
                lines.Add(new SkeletonLine(startIndex + i, startIndex + (i + 1) % pointCount, lineType));
        }
    }
}

public enum BikeType
{
    Standard,
    Sport,
    OffRoad
}