// BikeGeom.cs
using System.Collections.Generic;
using System.Linq;
using System.Windows;

namespace GravityDefiedGame.Models
{
    /// <summary>
    /// Класс BikeGeom отвечает за расчёт геометрии мотоцикла, включая точки для рисования шасси, скелета и деталей (сиденье, руль, выхлоп).
    /// Методы разбиты на небольшие части для улучшения читаемости и поддержки кода.
    /// </summary>
    public class BikeGeom
    {
        private static class GeometryConstants
        {
            public const double
                FrameHeightMultiplier = 1.5,
                LowerFrameOffsetMultiplier = 0.1,
                UpperFrameOffsetMultiplier = 0.5;

            public const double
                SeatOffsetX1 = -15.0,
                SeatOffsetX2 = 25.0,
                SeatOffsetX3 = 30.0,
                SeatOffsetX4 = -10.0,
                SeatOffsetY1 = -15.0,
                SeatOffsetY2 = -5.0;

            public const double
                HandlebarOffsetX1 = -7.5,
                HandlebarOffsetX2 = 7.5,
                HandlebarOffsetX3 = 15.0,
                HandlebarOffsetX4 = 0.0,
                HandlebarOffsetY1 = -7.5,
                HandlebarOffsetY2 = 7.5;

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

        public BikeGeom(Motorcycle bike)
        {
            _bike = bike;
            _physics = bike;
        }

        #region Геометрия шасси

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
            double frameTopOffset = _bike.GetWheelRadius() * GeometryConstants.FrameHeightMultiplier;
            var (cosAngle, sinAngle) = PhysicsComponent.GetTrigsFromAngle(_bike.Angle);

            Point frontLower = OffsetPoint(_bike.AttachmentPoints.Front, frameTopOffset, GeometryConstants.LowerFrameOffsetMultiplier, cosAngle, sinAngle);
            Point rearLower = OffsetPoint(_bike.AttachmentPoints.Rear, frameTopOffset, GeometryConstants.LowerFrameOffsetMultiplier, cosAngle, sinAngle);
            Point frontUpper = OffsetPoint(frontLower, frameTopOffset, GeometryConstants.UpperFrameOffsetMultiplier, cosAngle, sinAngle);
            Point rearUpper = OffsetPoint(rearLower, frameTopOffset, GeometryConstants.UpperFrameOffsetMultiplier, cosAngle, sinAngle);

            return new List<Point> { frontLower, rearLower, rearUpper, frontUpper };
        }

        private Point OffsetPoint(Point basePoint, double offset, double multiplier, double cosAngle, double sinAngle)
        {
            return new Point(
                basePoint.X + sinAngle * offset * multiplier,
                basePoint.Y - cosAngle * offset * multiplier
            );
        }

        #endregion

        #region Скелет мотоцикла

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
            // Добавляем колёсные и точки крепления подвески
            var wheelPositions = _bike.WheelPositions;
            var attachmentPoints = _bike.AttachmentPoints;

            points.Add(new SkeletonPoint(wheelPositions.Front, SkeletonPointType.FrontWheel));
            points.Add(new SkeletonPoint(wheelPositions.Rear, SkeletonPointType.RearWheel));
            points.Add(new SkeletonPoint(attachmentPoints.Front, SkeletonPointType.FrontSuspension));
            points.Add(new SkeletonPoint(attachmentPoints.Rear, SkeletonPointType.RearSuspension));

            // Добавляем точки шасси
            var chassisPoints = GetChassisPoints();
            int frameStartIndex = points.Count;
            points.AddRange(chassisPoints.Select(p => new SkeletonPoint(p, SkeletonPointType.Frame)));

            // Линии между подвеской и шасси
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
            const int spokeCount = 16;
            double wheelRadius = _bike.GetWheelRadius();
            int centerPointIndex = wheelIndex;
            var spokeEndIndices = new List<int>(spokeCount);

            for (int i = 0; i < spokeCount; i++)
            {
                double angle = rotation + i * (2 * Math.PI / spokeCount);
                Point spokeEnd = PhysicsComponent.Offset(center, wheelRadius, 0, angle);
                points.Add(new SkeletonPoint(spokeEnd, SkeletonPointType.Wheel));
                int spokeEndIndex = points.Count - 1;
                spokeEndIndices.Add(spokeEndIndex);
                lines.Add(new SkeletonLine(centerPointIndex, spokeEndIndex, SkeletonLineType.Wheel));
            }

            for (int i = 0; i < spokeCount; i++)
                lines.Add(new SkeletonLine(spokeEndIndices[i], spokeEndIndices[(i + 1) % spokeCount], SkeletonLineType.Wheel));
        }

        #endregion

        #region Детали скелета

        private void AddDetailSkeletonElements(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            AddSkeletonGroup(points, lines, CreateSeatPoints(), SkeletonPointType.Seat, SkeletonLineType.Seat);
            AddSkeletonGroup(points, lines, CreateHandlebarPoints(), SkeletonPointType.Handlebar, SkeletonLineType.Handlebar);
            AddSkeletonGroup(points, lines, CreateExhaustPoints(), SkeletonPointType.Exhaust, SkeletonLineType.Exhaust);
        }

        private void AddSkeletonGroup(List<SkeletonPoint> points, List<SkeletonLine> lines,
                                        List<Point> detailPoints, SkeletonPointType pointType,
                                        SkeletonLineType lineType)
        {
            int baseIndex = points.Count;
            foreach (var point in detailPoints)
                points.Add(new SkeletonPoint(point, pointType));

            ((PhysicsComponent)_bike).AddClosedPolygonLines(
                lines, baseIndex, detailPoints.Count, lineType,
                (start, end, type) => new SkeletonLine(start, end, type));
        }

        private List<Point> CreateSeatPoints()
        {
            var (cosAngle, sinAngle) = PhysicsComponent.GetTrigsFromAngle(_bike.Angle);
            var chassisPoints = GetChassisPoints();

            Point rearUpper = chassisPoints[2];
            Point frontUpper = chassisPoints[3];

            Point seatBase = new Point(
                (frontUpper.X + rearUpper.X) / 2 - 5 * cosAngle - 8 * sinAngle,
                (frontUpper.Y + rearUpper.Y) / 2 - 5 * sinAngle + 8 * cosAngle
            );

            return new List<Point>
            {
                PhysicsComponent.Offset(seatBase, GeometryConstants.SeatOffsetX1 * 0.8, GeometryConstants.SeatOffsetY1 * 0.7, _bike.Angle),
                PhysicsComponent.Offset(seatBase, GeometryConstants.SeatOffsetX2 * 0.9, GeometryConstants.SeatOffsetY1 * 0.7, _bike.Angle),
                PhysicsComponent.Offset(seatBase, GeometryConstants.SeatOffsetX3 * 1.2, GeometryConstants.SeatOffsetY2 * 0.8, _bike.Angle),
                PhysicsComponent.Offset(seatBase, GeometryConstants.SeatOffsetX4 * 0.8, GeometryConstants.SeatOffsetY2 * 0.8, _bike.Angle)
            };
        }

        private List<Point> CreateHandlebarPoints()
        {
            var chassisPoints = GetChassisPoints();
            Point frontUpper = chassisPoints[3];

            double baseWidth = GeometryConstants.HandlebarOffsetX3 * 1.8;
            double baseHeight = GeometryConstants.HandlebarOffsetY1 * 0.5;
            double handleHeight = GeometryConstants.HandlebarOffsetY1 * 1.5;

            return new List<Point>
            {
                PhysicsComponent.Offset(frontUpper, -baseWidth, handleHeight, _bike.Angle),
                PhysicsComponent.Offset(frontUpper, -baseWidth * 0.4, baseHeight, _bike.Angle),
                PhysicsComponent.Offset(frontUpper, baseWidth * 0.4, baseHeight, _bike.Angle),
                PhysicsComponent.Offset(frontUpper, baseWidth, handleHeight, _bike.Angle)
            };
        }

        private List<Point> CreateExhaustPoints()
        {
            var chassisPoints = GetChassisPoints();
            Point exhaustBase = chassisPoints[1];

            double pipeLength = 15, pipeHeight = 1, downOffset = 1, rightOffset = 1;

            return new List<Point>
            {
                exhaustBase,
                PhysicsComponent.Offset(exhaustBase, 0, rightOffset + downOffset, _bike.Angle + Math.PI / 2),
                PhysicsComponent.Offset(exhaustBase, -pipeLength, rightOffset + pipeHeight + downOffset, _bike.Angle),
                PhysicsComponent.Offset(exhaustBase, -pipeLength, rightOffset, _bike.Angle)
            };
        }

        #endregion
    }
}