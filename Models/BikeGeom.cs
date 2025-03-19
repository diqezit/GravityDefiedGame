using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;

namespace GravityDefiedGame.Models
{
    public class BikeGeom
    {
        // Локальные константы для геометрии мотоцикла
        private static class GeometryConstants
        {
            // Множители для рамы мотоцикла
            public const double
                FrameHeightMultiplier = 1.5,
                LowerFrameOffsetMultiplier = 0.1,
                UpperFrameOffsetMultiplier = 0.5;

            // Смещения для сиденья
            public const double
                SeatOffsetX1 = -15.0,
                SeatOffsetX2 = 25.0,
                SeatOffsetX3 = 30.0,
                SeatOffsetX4 = -10.0,
                SeatOffsetY1 = -15.0,
                SeatOffsetY2 = -5.0;

            // Смещения для руля
            public const double
                HandlebarOffsetX1 = -7.5,
                HandlebarOffsetX2 = 7.5,
                HandlebarOffsetX3 = 15.0,
                HandlebarOffsetX4 = 0.0,
                HandlebarOffsetY1 = -7.5,
                HandlebarOffsetY2 = 7.5;

            // Смещения для выхлопной трубы
            public const double
                ExhaustOffsetX1 = -15.0,
                ExhaustOffsetX2 = -25.0,
                ExhaustOffsetX3 = -30.0,
                ExhaustOffsetX4 = -20.0,
                ExhaustOffsetY1 = -15.0,
                ExhaustOffsetY2 = -20.0,
                ExhaustOffsetY3 = -7.5,
                ExhaustOffsetY4 = 0.0;
        }

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

        public BikeGeom(Motorcycle bike) => _bike = bike;

        public List<Point> GetFramePoints()
        {
            var points = new List<Point>();
            double cosAngle = Math.Cos(_bike.Angle);
            double sinAngle = Math.Sin(_bike.Angle);

            double halfWheelBase = _bike.WheelBase / 2;
            double frameHeight = _bike.FrameHeight * 0.8;

            points.Add(new Point(
                _bike.Position.X + halfWheelBase * 0.7 * cosAngle - frameHeight * sinAngle,
                _bike.Position.Y + halfWheelBase * 0.7 * sinAngle + frameHeight * cosAngle
            ));

            points.Add(new Point(
                _bike.Position.X - halfWheelBase * 0.7 * cosAngle - frameHeight * sinAngle,
                _bike.Position.Y - halfWheelBase * 0.7 * sinAngle + frameHeight * cosAngle
            ));

            return points;
        }

        public List<Point> GetChassisPoints()
        {
            double frameTopOffset = _bike.GetWheelRadius() * GeometryConstants.FrameHeightMultiplier;
            double cosAngle = Math.Cos(_bike.Angle), sinAngle = Math.Sin(_bike.Angle);

            Point frontLower = new(
                _bike.FrontAttachmentPoint.X + sinAngle * frameTopOffset * GeometryConstants.LowerFrameOffsetMultiplier,
                _bike.FrontAttachmentPoint.Y - cosAngle * frameTopOffset * GeometryConstants.LowerFrameOffsetMultiplier
            );

            Point rearLower = new(
                _bike.RearAttachmentPoint.X + sinAngle * frameTopOffset * GeometryConstants.LowerFrameOffsetMultiplier,
                _bike.RearAttachmentPoint.Y - cosAngle * frameTopOffset * GeometryConstants.LowerFrameOffsetMultiplier
            );

            Point frontUpper = new(
                frontLower.X + sinAngle * frameTopOffset * GeometryConstants.UpperFrameOffsetMultiplier,
                frontLower.Y - cosAngle * frameTopOffset * GeometryConstants.UpperFrameOffsetMultiplier
            );

            Point rearUpper = new(
                rearLower.X + sinAngle * frameTopOffset * GeometryConstants.UpperFrameOffsetMultiplier,
                rearLower.Y - cosAngle * frameTopOffset * GeometryConstants.UpperFrameOffsetMultiplier
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
                new(seatBase.X + GeometryConstants.SeatOffsetX1 * 0.8 * cosAngle - GeometryConstants.SeatOffsetY1 * 0.7 * sinAngle,
                    seatBase.Y + GeometryConstants.SeatOffsetX1 * 0.8 * sinAngle + GeometryConstants.SeatOffsetY1 * 0.7 * cosAngle),

                new(seatBase.X + GeometryConstants.SeatOffsetX2 * 0.9 * cosAngle - GeometryConstants.SeatOffsetY1 * 0.7 * sinAngle,
                    seatBase.Y + GeometryConstants.SeatOffsetX2 * 0.9 * sinAngle + GeometryConstants.SeatOffsetY1 * 0.7 * cosAngle),

                new(seatBase.X + GeometryConstants.SeatOffsetX3 * 1.2 * cosAngle - GeometryConstants.SeatOffsetY2 * 0.8 * sinAngle,
                    seatBase.Y + GeometryConstants.SeatOffsetX3 * 1.2 * sinAngle + GeometryConstants.SeatOffsetY2 * 0.8 * cosAngle),

                new(seatBase.X + GeometryConstants.SeatOffsetX4 * 0.8 * cosAngle - GeometryConstants.SeatOffsetY2 * 0.8 * sinAngle,
                    seatBase.Y + GeometryConstants.SeatOffsetX4 * 0.8 * sinAngle + GeometryConstants.SeatOffsetY2 * 0.8 * cosAngle)
            };
        }

        private List<Point> CreateHandlebarPoints()
        {
            double cosAngle = Math.Cos(_bike.Angle), sinAngle = Math.Sin(_bike.Angle);
            var chassisPoints = GetChassisPoints();
            Point frontUpper = chassisPoints[3];

            double baseWidth = GeometryConstants.HandlebarOffsetX3 * 1.8;
            double baseHeight = GeometryConstants.HandlebarOffsetY1 * 0.5;
            double handleHeight = GeometryConstants.HandlebarOffsetY1 * 1.5;

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