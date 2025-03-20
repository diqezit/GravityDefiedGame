using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Models
{
    public class BikeGeom
    {
        private static class GeometryConstants
        {
            public const double
                FrameHeightMultiplier = 1.2,
                LowerFrameOffsetMultiplier = 0.15,
                UpperFrameOffsetMultiplier = 0.4;

            public const double
                SeatOffsetX1 = -20.0,
                SeatOffsetX2 = 25.0,
                SeatOffsetY = -10.0;

            public const double
                HandlebarWidth = 24.0,
                HandlebarHeight = 8.0,
                HandlebarStemLength = 15.0;

            public const double
                ExhaustOffsetX1 = -15.0,
                ExhaustOffsetX2 = -28.0,
                ExhaustOffsetX3 = -32.0,
                ExhaustOffsetX4 = -20.0,
                ExhaustOffsetY1 = -12.0,
                ExhaustOffsetY2 = -18.0,
                ExhaustOffsetY3 = -6.0,
                ExhaustOffsetY4 = 0.0;

            public const double
                ShockAbsorberWidth = 5.0,
                ForkThickness = 3.0,
                ChainOffsetY = 3.0,
                EngineOffsetY = -5.0,
                EngineWidth = 25.0,
                EngineHeight = 20.0;
        }

        private readonly Motorcycle _bike;
        private readonly PhysicsComponent _physics;

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
            Wheel,
            Engine,
            Chain,
            ShockAbsorber,
            Fork
        }

        public record struct SkeletonLine(int StartPointIndex, int EndPointIndex, SkeletonLineType Type);
        public enum SkeletonLineType
        {
            MainFrame,
            Suspension,
            Wheel,
            Seat,
            Handlebar,
            Exhaust,
            Engine,
            Chain,
            ShockAbsorber,
            Fork
        }

        public BikeGeom(Motorcycle bike)
        {
            _bike = bike;
            _physics = bike;
        }

        private double GetScaleFactor()
        {
            switch (_bike.BikeType)
            {
                case BikeType.Sport:
                    return 0.9;
                case BikeType.OffRoad:
                    return 1.1;
                default:
                    return 1.0;
            }
        }

        public List<Point> GetFramePoints()
        {
            var (cosAngle, sinAngle) = PhysicsComponent.GetTrigsFromAngle(_bike.Angle);
            double halfWheelBase = _bike.WheelBase / 2;
            double frameHeight = _bike.FrameHeight * 0.8;

            return new List<Point>
            {
                CreateFramePoint(halfWheelBase, frameHeight, cosAngle, sinAngle, true),
                CreateFramePoint(halfWheelBase, frameHeight, cosAngle, sinAngle, false)
            };
        }

        private Point CreateFramePoint(double halfWheelBase, double frameHeight, double cosAngle, double sinAngle, bool isFront)
        {
            double multiplier = isFront ? 0.7 : -0.7;
            return new Point(
                _bike.Position.X + halfWheelBase * multiplier * cosAngle - frameHeight * sinAngle,
                _bike.Position.Y + halfWheelBase * multiplier * sinAngle + frameHeight * cosAngle
            );
        }

        public List<Point> GetChassisPoints()
        {
            double scaleFactor = GetScaleFactor();
            double frameTopOffset = _bike.GetWheelRadius() * GeometryConstants.FrameHeightMultiplier * scaleFactor;
            var (cosAngle, sinAngle) = PhysicsComponent.GetTrigsFromAngle(_bike.Angle);

            double lowerOffsetMultiplier = GeometryConstants.LowerFrameOffsetMultiplier;
            double upperOffsetMultiplier = GeometryConstants.UpperFrameOffsetMultiplier;

            if (_bike.BikeType == BikeType.Sport)
            {
                upperOffsetMultiplier *= 1.2;
            }
            else if (_bike.BikeType == BikeType.OffRoad)
            {
                lowerOffsetMultiplier *= 0.9;
            }

            Point frontLower = OffsetPoint(_bike.AttachmentPoints.Front, frameTopOffset, lowerOffsetMultiplier, cosAngle, sinAngle);
            Point rearLower = OffsetPoint(_bike.AttachmentPoints.Rear, frameTopOffset, lowerOffsetMultiplier, cosAngle, sinAngle);
            Point frontUpper = OffsetPoint(frontLower, frameTopOffset, upperOffsetMultiplier, cosAngle, sinAngle);
            Point rearUpper = OffsetPoint(rearLower, frameTopOffset, upperOffsetMultiplier, cosAngle, sinAngle);

            return new List<Point> { frontLower, rearLower, rearUpper, frontUpper };
        }

        private Point OffsetPoint(Point basePoint, double offset, double multiplier, double cosAngle, double sinAngle)
        {
            return new Point(
                basePoint.X + sinAngle * offset * multiplier,
                basePoint.Y - cosAngle * offset * multiplier
            );
        }

        public (List<SkeletonPoint> Points, List<SkeletonLine> Lines) GetSkeleton()
        {
            var points = new List<SkeletonPoint>();
            var lines = new List<SkeletonLine>();

            AddBasicSkeletonElements(points, lines);
            AddDetailSkeletonElements(points, lines);
            AddEnhancedDetailElements(points, lines);

            return (points, lines);
        }

        private void AddBasicSkeletonElements(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var wheelPositions = _bike.WheelPositions;
            var attachmentPoints = _bike.AttachmentPoints;

            points.Add(new SkeletonPoint(wheelPositions.Front, SkeletonPointType.FrontWheel));
            points.Add(new SkeletonPoint(wheelPositions.Rear, SkeletonPointType.RearWheel));
            points.Add(new SkeletonPoint(attachmentPoints.Front, SkeletonPointType.FrontSuspension));
            points.Add(new SkeletonPoint(attachmentPoints.Rear, SkeletonPointType.RearSuspension));

            var chassisPoints = GetChassisPoints();
            int frameStartIndex = points.Count;
            points.AddRange(chassisPoints.Select(p => new SkeletonPoint(p, SkeletonPointType.Frame)));

            lines.Add(new SkeletonLine(0, 2, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(1, 3, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(2, frameStartIndex, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(3, frameStartIndex + 1, SkeletonLineType.Suspension));

            AddFrameLines(points, lines, frameStartIndex, chassisPoints.Count);
            AddWheelSpokes(points, lines, wheelPositions);
        }

        private void AddFrameLines(List<SkeletonPoint> points, List<SkeletonLine> lines, int startIndex, int count)
        {
            for (int i = startIndex; i < startIndex + count - 1; i++)
                lines.Add(new SkeletonLine(i, i + 1, SkeletonLineType.MainFrame));

            lines.Add(new SkeletonLine(startIndex + count - 1, startIndex, SkeletonLineType.MainFrame));
        }

        private void AddWheelSpokes(List<SkeletonPoint> points, List<SkeletonLine> lines, (Point Front, Point Rear) wheelPositions)
        {
            AddWheelSpokesForWheel(points, lines, 0, wheelPositions.Front, _bike.WheelRotations.Front);
            AddWheelSpokesForWheel(points, lines, 1, wheelPositions.Rear, _bike.WheelRotations.Rear);
        }

        private void AddWheelSpokesForWheel(List<SkeletonPoint> points, List<SkeletonLine> lines, int wheelIndex, Point center, double rotation)
        {
            int spokeCount;
            switch (_bike.BikeType)
            {
                case BikeType.Sport:
                    spokeCount = 20;
                    break;
                case BikeType.OffRoad:
                    spokeCount = 12;
                    break;
                default:
                    spokeCount = 16;
                    break;
            }

            double wheelRadius = _bike.GetWheelRadius();
            int centerPointIndex = wheelIndex;
            var spokeEndIndices = new List<int>(spokeCount);

            double groundY = 0;
            if (wheelIndex == 0)
            {
                groundY = center.Y + wheelRadius;
            }
            else
            {
                groundY = center.Y + wheelRadius;
            }

            for (int i = 0; i < spokeCount; i++)
            {
                double angle = rotation + i * (2 * Math.PI / spokeCount);

                double effectiveRadius = wheelRadius;

                Point spokeEnd = PhysicsComponent.Offset(center, wheelRadius, 0, angle);

                if (!_bike.IsInAir && spokeEnd.Y >= groundY - 1)
                {
                    double penetration = spokeEnd.Y - (groundY - 1);
                    double deformationFactor = Math.Min(0.2, penetration / wheelRadius);

                    effectiveRadius = wheelRadius * (1.0 - deformationFactor);
                }

                spokeEnd = PhysicsComponent.Offset(center, effectiveRadius, 0, angle);

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
            AddSeat(points, lines);
            AddHandlebar(points, lines);
            AddExhaust(points, lines);
        }

        private void AddSeat(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var chassisPoints = GetChassisPoints();
            Point rearUpper = chassisPoints[2];

            double xOffset, width, height;

            switch (_bike.BikeType)
            {
                case BikeType.Sport:
                    xOffset = -15;
                    width = 35;
                    height = 3;
                    break;
                case BikeType.OffRoad:
                    xOffset = -10;
                    width = 45;
                    height = 4;
                    break;
                default:
                    xOffset = -12;
                    width = 40;
                    height = 3;
                    break;
            }

            Point seatStart = PhysicsComponent.Offset(rearUpper, xOffset, -5, _bike.Angle);
            Point seatEnd = PhysicsComponent.Offset(seatStart, width, 0, _bike.Angle);

            int startIndex = points.Count;

            points.Add(new SkeletonPoint(seatStart, SkeletonPointType.Seat));
            points.Add(new SkeletonPoint(seatEnd, SkeletonPointType.Seat));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(seatEnd, 0, height, _bike.Angle), SkeletonPointType.Seat));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(seatStart, 0, height, _bike.Angle), SkeletonPointType.Seat));

            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.Seat));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, SkeletonLineType.Seat));
            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 3, SkeletonLineType.Seat));
            lines.Add(new SkeletonLine(startIndex + 3, startIndex, SkeletonLineType.Seat));
        }

        private void AddHandlebar(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var chassisPoints = GetChassisPoints();
            Point frontUpper = chassisPoints[3];

            double handleWidth, stemLength;

            switch (_bike.BikeType)
            {
                case BikeType.Sport:
                    handleWidth = GeometryConstants.HandlebarWidth * 1.1;
                    stemLength = GeometryConstants.HandlebarStemLength * 0.8;
                    break;
                case BikeType.OffRoad:
                    handleWidth = GeometryConstants.HandlebarWidth * 1.2;
                    stemLength = GeometryConstants.HandlebarStemLength * 1.2;
                    break;
                default:
                    handleWidth = GeometryConstants.HandlebarWidth;
                    stemLength = GeometryConstants.HandlebarStemLength;
                    break;
            }

            double steeringOffset = _bike.LeanAmount * 5.0;

            int startIndex = points.Count;

            Point stemBase = frontUpper;
            points.Add(new SkeletonPoint(stemBase, SkeletonPointType.Handlebar));

            Point stemTop = PhysicsComponent.Offset(stemBase, 0, -stemLength, _bike.Angle);
            stemTop = PhysicsComponent.Offset(stemTop, steeringOffset, 0, _bike.Angle);
            points.Add(new SkeletonPoint(stemTop, SkeletonPointType.Handlebar));

            Point handleLeft = PhysicsComponent.Offset(stemTop, -handleWidth / 2, 0, _bike.Angle);
            points.Add(new SkeletonPoint(handleLeft, SkeletonPointType.Handlebar));

            Point handleRight = PhysicsComponent.Offset(stemTop, handleWidth / 2, 0, _bike.Angle);
            points.Add(new SkeletonPoint(handleRight, SkeletonPointType.Handlebar));

            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.Handlebar));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, SkeletonLineType.Handlebar));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 3, SkeletonLineType.Handlebar));
        }

        private void AddExhaust(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var chassisPoints = GetChassisPoints();
            Point exhaustBase = chassisPoints[1];

            double pipeLength, pipeHeight, downOffset, rightOffset;

            switch (_bike.BikeType)
            {
                case BikeType.Sport:
                    pipeLength = 12;
                    pipeHeight = 0.8;
                    downOffset = 1.5;
                    rightOffset = 1.2;
                    break;
                case BikeType.OffRoad:
                    pipeLength = 20;
                    pipeHeight = 2.0;
                    downOffset = 0.5;
                    rightOffset = 0.8;
                    break;
                default:
                    pipeLength = 15;
                    pipeHeight = 1;
                    downOffset = 1;
                    rightOffset = 1;
                    break;
            }

            int startIndex = points.Count;

            points.Add(new SkeletonPoint(exhaustBase, SkeletonPointType.Exhaust));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(exhaustBase, 0, rightOffset + downOffset, _bike.Angle + Math.PI / 2), SkeletonPointType.Exhaust));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(exhaustBase, -pipeLength, rightOffset + pipeHeight + downOffset, _bike.Angle), SkeletonPointType.Exhaust));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(exhaustBase, -pipeLength, rightOffset, _bike.Angle), SkeletonPointType.Exhaust));

            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.Exhaust));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, SkeletonLineType.Exhaust));
            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 3, SkeletonLineType.Exhaust));
            lines.Add(new SkeletonLine(startIndex + 3, startIndex, SkeletonLineType.Exhaust));
        }

        private void AddEnhancedDetailElements(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            AddRearSwingArm(points, lines);
            AddRearShockAbsorber(points, lines);
            AddEngine(points, lines);
            AddChain(points, lines);
            AddFrontForks(points, lines);
        }

        private void AddEngine(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var chassisPoints = GetChassisPoints();
            Point frontLower = chassisPoints[0];
            Point rearLower = chassisPoints[1];

            double engineWidth, engineHeight;
            double xOffset = 0, yOffset = 0;

            switch (_bike.BikeType)
            {
                case BikeType.Sport:
                    engineWidth = GeometryConstants.EngineWidth * 0.9;
                    engineHeight = GeometryConstants.EngineHeight * 0.8;
                    xOffset = -15;
                    yOffset = 5;
                    break;
                case BikeType.OffRoad:
                    engineWidth = GeometryConstants.EngineWidth * 1.1;
                    engineHeight = GeometryConstants.EngineHeight * 1.2;
                    xOffset = -12;
                    yOffset = 8;
                    break;
                default:
                    engineWidth = GeometryConstants.EngineWidth;
                    engineHeight = GeometryConstants.EngineHeight;
                    xOffset = -10;
                    yOffset = 7;
                    break;
            }

            double frameLength = PhysicsComponent.CalculateDistance(frontLower, rearLower);
            double enginePositionRatio = 0.3;

            Point engineCenter = new Point(
                frontLower.X - (frontLower.X - rearLower.X) * enginePositionRatio + xOffset,
                frontLower.Y - (frontLower.Y - rearLower.Y) * enginePositionRatio + yOffset
            );

            double halfWidth = engineWidth / 2;
            double halfHeight = engineHeight / 2;

            int startIndex = points.Count;

            points.Add(new SkeletonPoint(PhysicsComponent.Offset(engineCenter, -halfWidth, -halfHeight * 0.7, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(engineCenter, halfWidth, -halfHeight * 0.7, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(engineCenter, halfWidth, halfHeight, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(engineCenter, -halfWidth, halfHeight, _bike.Angle), SkeletonPointType.Engine));

            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.Engine));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, SkeletonLineType.Engine));
            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 3, SkeletonLineType.Engine));
            lines.Add(new SkeletonLine(startIndex + 3, startIndex, SkeletonLineType.Engine));

            AddEngineCylinder(points, lines, engineCenter, halfWidth, halfHeight);
        }

        private void AddEngineCylinder(List<SkeletonPoint> points, List<SkeletonLine> lines, Point engineCenter, double width, double height)
        {
            int startIndex = points.Count;

            double cylinderWidth, cylinderHeight;
            double offsetX, offsetY;

            switch (_bike.BikeType)
            {
                case BikeType.Sport:
                    cylinderWidth = width * 0.7;
                    cylinderHeight = width * 0.5;
                    offsetX = width * 0.9;
                    offsetY = -height * 0.2;
                    break;
                case BikeType.OffRoad:
                    cylinderWidth = width * 0.8;
                    cylinderHeight = width * 0.6;
                    offsetX = width * 0.9;
                    offsetY = -height * 0.1;
                    break;
                default:
                    cylinderWidth = width * 0.7;
                    cylinderHeight = width * 0.5;
                    offsetX = width * 0.9;
                    offsetY = -height * 0.15;
                    break;
            }

            // Базовая точка цилиндра - на правой стороне двигателя
            Point cylinderBase = PhysicsComponent.Offset(engineCenter, offsetX, offsetY, _bike.Angle);

            // Создаем прямоугольник для цилиндра
            points.Add(new SkeletonPoint(cylinderBase, SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(cylinderBase, cylinderWidth, 0, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(cylinderBase, cylinderWidth, cylinderHeight, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(cylinderBase, 0, cylinderHeight, _bike.Angle), SkeletonPointType.Engine));

            // Соединяем точки линиями
            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.Engine));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, SkeletonLineType.Engine));
            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 3, SkeletonLineType.Engine));
            lines.Add(new SkeletonLine(startIndex + 3, startIndex, SkeletonLineType.Engine));
        }

        private void AddChain(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            Point rearWheel = _bike.WheelPositions.Rear;

            var enginePoints = new List<Point>();
            var chassisPoints = GetChassisPoints();
            Point frontLower = chassisPoints[0];
            Point rearLower = chassisPoints[1];

            double engineWidth = GeometryConstants.EngineWidth;
            double engineHeight = GeometryConstants.EngineHeight;
            double xOffset = -10;
            double yOffset = 7;

            double frameLength = PhysicsComponent.CalculateDistance(frontLower, rearLower);
            double enginePositionRatio = 0.3;

            Point engineCenter = new Point(
                frontLower.X - (frontLower.X - rearLower.X) * enginePositionRatio + xOffset,
                frontLower.Y - (frontLower.Y - rearLower.Y) * enginePositionRatio + yOffset
            );

            double halfWidth = engineWidth / 2;
            double halfHeight = engineHeight / 2;

            Point engineSprocket = PhysicsComponent.Offset(engineCenter, -halfWidth, halfHeight, _bike.Angle);

            double wheelRadius = _bike.GetWheelRadius() * 0.7;
            double sprocketRadius = wheelRadius * 0.3;
            double rearSprocketRadius = wheelRadius * 0.5;

            int segmentCount = 8;
            int startIndex = points.Count;

            points.Add(new SkeletonPoint(engineSprocket, SkeletonPointType.Chain));

            double totalDistance = PhysicsComponent.CalculateDistance(engineSprocket, rearWheel);

            for (int i = 1; i < segmentCount; i++)
            {
                double ratio = (double)i / segmentCount;
                double x = engineSprocket.X - (engineSprocket.X - rearWheel.X) * ratio;
                double y = engineSprocket.Y - (engineSprocket.Y - rearWheel.Y) * ratio;

                double sag = Math.Sin(ratio * Math.PI) * 3;
                y += sag;

                points.Add(new SkeletonPoint(new Point(x, y), SkeletonPointType.Chain));
            }

            Point rearSprocketPoint = PhysicsComponent.Offset(rearWheel, -rearSprocketRadius, 0, _bike.Angle);
            points.Add(new SkeletonPoint(rearSprocketPoint, SkeletonPointType.Chain));

            for (int i = segmentCount - 1; i > 0; i--)
            {
                double ratio = (double)i / segmentCount;
                double x = engineSprocket.X - (engineSprocket.X - rearWheel.X) * ratio;
                double y = engineSprocket.Y - (engineSprocket.Y - rearWheel.Y) * ratio;

                double tension = -Math.Sin(ratio * Math.PI) * 1.5;
                y += tension;

                points.Add(new SkeletonPoint(new Point(x, y), SkeletonPointType.Chain));
            }

            int totalPoints = 2 * segmentCount;
            for (int i = 0; i < totalPoints; i++)
            {
                lines.Add(new SkeletonLine(
                    startIndex + i,
                    startIndex + (i + 1) % totalPoints,
                    SkeletonLineType.Chain
                ));
            }

            int sprocketSegments = 8;
            int sprocketStartIndex = points.Count;

            for (int i = 0; i < sprocketSegments; i++)
            {
                double angle = i * (2 * Math.PI / sprocketSegments);
                points.Add(new SkeletonPoint(
                    PhysicsComponent.Offset(engineSprocket, sprocketRadius, 0, angle),
                    SkeletonPointType.Chain
                ));
            }

            for (int i = 0; i < sprocketSegments; i++)
            {
                lines.Add(new SkeletonLine(
                    sprocketStartIndex + i,
                    sprocketStartIndex + (i + 1) % sprocketSegments,
                    SkeletonLineType.Chain
                ));
            }

            int rearSprocketStartIndex = points.Count;

            for (int i = 0; i < sprocketSegments; i++)
            {
                double angle = i * (2 * Math.PI / sprocketSegments);
                points.Add(new SkeletonPoint(
                    PhysicsComponent.Offset(rearWheel, rearSprocketRadius, 0, angle),
                    SkeletonPointType.Chain
                ));
            }

            for (int i = 0; i < sprocketSegments; i++)
            {
                lines.Add(new SkeletonLine(
                    rearSprocketStartIndex + i,
                    rearSprocketStartIndex + (i + 1) % sprocketSegments,
                    SkeletonLineType.Chain
                ));
            }
        }

        private void AddFrontForks(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            Point frontWheel = _bike.WheelPositions.Front;
            Point frontAttachment = _bike.AttachmentPoints.Front;

            double forkThickness = GeometryConstants.ForkThickness;
            double wheelRadius = _bike.GetWheelRadius();

            int startIndex = points.Count;

            double forkOffset = forkThickness / 2;

            Point forkTop1 = PhysicsComponent.Offset(frontAttachment, -forkOffset, 0, _bike.Angle);
            Point forkTop2 = PhysicsComponent.Offset(frontAttachment, forkOffset, 0, _bike.Angle);

            Point forkBottom1 = PhysicsComponent.Offset(frontWheel, -forkOffset, -wheelRadius * 0.1, _bike.Angle);
            Point forkBottom2 = PhysicsComponent.Offset(frontWheel, forkOffset, -wheelRadius * 0.1, _bike.Angle);

            points.Add(new SkeletonPoint(forkTop1, SkeletonPointType.Fork));
            points.Add(new SkeletonPoint(forkTop2, SkeletonPointType.Fork));
            points.Add(new SkeletonPoint(forkBottom1, SkeletonPointType.Fork));
            points.Add(new SkeletonPoint(forkBottom2, SkeletonPointType.Fork));

            lines.Add(new SkeletonLine(startIndex, startIndex + 2, SkeletonLineType.Fork));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 3, SkeletonLineType.Fork));
        }

        private void AddRearSwingArm(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            Point rearWheel = _bike.WheelPositions.Rear;
            Point rearAttachment = _bike.AttachmentPoints.Rear;

            double swingArmWidth = 3.0;
            double wheelRadius = _bike.GetWheelRadius();

            double dx = rearWheel.X - rearAttachment.X;
            double dy = rearWheel.Y - rearAttachment.Y;
            double length = Math.Sqrt(dx * dx + dy * dy);
            double normalizedX = dx / length;
            double normalizedY = dy / length;

            double perpX = -normalizedY;
            double perpY = normalizedX;

            int startIndex = points.Count;

            points.Add(new SkeletonPoint(rearAttachment, SkeletonPointType.ShockAbsorber));

            Point swingArmTopFrame = new Point(
                rearAttachment.X + perpX * swingArmWidth / 2,
                rearAttachment.Y + perpY * swingArmWidth / 2
            );

            Point swingArmBottomFrame = new Point(
                rearAttachment.X - perpX * swingArmWidth / 2,
                rearAttachment.Y - perpY * swingArmWidth / 2
            );

            Point swingArmTopWheel = new Point(
                rearWheel.X + perpX * swingArmWidth / 2,
                rearWheel.Y + perpY * swingArmWidth / 2
            );

            Point swingArmBottomWheel = new Point(
                rearWheel.X - perpX * swingArmWidth / 2,
                rearWheel.Y - perpY * swingArmWidth / 2
            );

            points.Add(new SkeletonPoint(swingArmTopFrame, SkeletonPointType.ShockAbsorber));
            points.Add(new SkeletonPoint(swingArmBottomFrame, SkeletonPointType.ShockAbsorber));
            points.Add(new SkeletonPoint(swingArmTopWheel, SkeletonPointType.ShockAbsorber));
            points.Add(new SkeletonPoint(swingArmBottomWheel, SkeletonPointType.ShockAbsorber));

            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 3, SkeletonLineType.ShockAbsorber));
            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 4, SkeletonLineType.ShockAbsorber));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, SkeletonLineType.ShockAbsorber));
            lines.Add(new SkeletonLine(startIndex + 3, startIndex + 4, SkeletonLineType.ShockAbsorber));
        }

        private void AddRearShockAbsorber(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var chassisPoints = GetChassisPoints();
            Point rearUpper = chassisPoints[2];
            Point rearAttachment = _bike.AttachmentPoints.Rear;

            double shockWidth = GeometryConstants.ShockAbsorberWidth;

            switch (_bike.BikeType)
            {
                case BikeType.Sport:
                    shockWidth *= 0.8;
                    break;
                case BikeType.OffRoad:
                    shockWidth *= 1.2;
                    break;
            }

            double compressionRatio = 1.0;
            if (!_bike.IsInAir)
            {
                double suspensionOffset = _bike.SuspensionOffsets.Rear;
                double restLength = _bike.GetWheelRadius() * 2;
                compressionRatio = suspensionOffset / restLength;
            }

            int startIndex = points.Count;

            Point shockTop = PhysicsComponent.Offset(rearUpper, -5, 0, _bike.Angle);

            double dx = _bike.WheelPositions.Rear.X - rearAttachment.X;
            double dy = _bike.WheelPositions.Rear.Y - rearAttachment.Y;
            double attachRatio = 0.3;

            Point shockBottom = new Point(
                rearAttachment.X + dx * attachRatio,
                rearAttachment.Y + dy * attachRatio
            );

            shockBottom = new Point(
                shockBottom.X + (1.0 - compressionRatio) * 3,
                shockBottom.Y - 5 * (1.0 - compressionRatio)
            );

            points.Add(new SkeletonPoint(shockTop, SkeletonPointType.ShockAbsorber));
            points.Add(new SkeletonPoint(shockBottom, SkeletonPointType.ShockAbsorber));

            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.ShockAbsorber));

            Point shockMid = new Point(
                (shockTop.X + shockBottom.X) / 2,
                (shockTop.Y + shockBottom.Y) / 2
            );

            double shockAngle = Math.Atan2(shockBottom.Y - shockTop.Y, shockBottom.X - shockTop.X);
            double perpAngle = shockAngle + Math.PI / 2;

            Point spring1 = PhysicsComponent.Offset(shockMid, 0, shockWidth / 2, perpAngle);
            Point spring2 = PhysicsComponent.Offset(shockMid, 0, -shockWidth / 2, perpAngle);

            points.Add(new SkeletonPoint(spring1, SkeletonPointType.ShockAbsorber));
            points.Add(new SkeletonPoint(spring2, SkeletonPointType.ShockAbsorber));

            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 3, SkeletonLineType.ShockAbsorber));
        }
    }
}