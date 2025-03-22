using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System;
using static System.Math;
using static GravityDefiedGame.Models.PhysicsComponent;
using static GravityDefiedGame.Models.BikeGeom.GeometryConstants;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Models
{
    public class BikeGeom
    {
        public static class GeometryConstants
        {
            public const double
                FrameHeightMultiplier = 1.2,
                LowerFrameOffsetMultiplier = 0.15,
                UpperFrameOffsetMultiplier = 0.4;

            public const double
                SeatOffsetX1 = -20.0, SeatOffsetX2 = 25.0, SeatOffsetY = -10.0;

            public const double
                HandlebarWidth = 24.0, HandlebarHeight = 8.0, HandlebarStemLength = 15.0;

            public const double
                ExhaustOffsetX1 = -15.0, ExhaustOffsetX2 = -28.0, ExhaustOffsetX3 = -32.0, ExhaustOffsetX4 = -20.0,
                ExhaustOffsetY1 = -12.0, ExhaustOffsetY2 = -18.0, ExhaustOffsetY3 = -6.0, ExhaustOffsetY4 = 0.0;

            public const double
                ShockAbsorberWidth = 5.0, ForkThickness = 3.0, ChainOffsetY = 3.0,
                EngineOffsetY = -5.0, EngineWidth = 25.0, EngineHeight = 20.0;
        }

        private readonly Motorcycle _bike;

        public record struct SkeletonPoint(Point Position, SkeletonPointType Type);
        public enum SkeletonPointType { FrontWheel, RearWheel, FrontSuspension, RearSuspension, Frame, Seat, Handlebar, Exhaust, Wheel, Engine, Chain, ShockAbsorber, Fork }

        public record struct SkeletonLine(int StartPointIndex, int EndPointIndex, SkeletonLineType Type);
        public enum SkeletonLineType { MainFrame, Suspension, Wheel, Seat, Handlebar, Exhaust, Engine, Chain, ShockAbsorber, Fork }

        public BikeGeom(Motorcycle bike) => _bike = bike;

        private double GetScaleFactor() => _bike.BikeType switch
        {
            BikeType.Sport => 0.9,
            BikeType.OffRoad => 1.1,
            _ => 1.0
        };

        public List<Point> GetFramePoints()
        {
            var (cosAngle, sinAngle) = GetTrigsFromAngle(_bike.Angle);
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
            double frameTopOffset = _bike.GetWheelRadius() * FrameHeightMultiplier * scaleFactor;
            var (cosAngle, sinAngle) = GetTrigsFromAngle(_bike.Angle);

            double lowerOffset = LowerFrameOffsetMultiplier;
            double upperOffset = UpperFrameOffsetMultiplier;

            if (_bike.BikeType == BikeType.Sport)
                upperOffset *= 1.2;
            else if (_bike.BikeType == BikeType.OffRoad)
                lowerOffset *= 0.9;

            Point frontLower = OffsetPoint(_bike.AttachmentPoints.Front, frameTopOffset, lowerOffset, cosAngle, sinAngle);
            Point rearLower = OffsetPoint(_bike.AttachmentPoints.Rear, frameTopOffset, lowerOffset, cosAngle, sinAngle);
            Point frontUpper = OffsetPoint(frontLower, frameTopOffset, upperOffset, cosAngle, sinAngle);
            Point rearUpper = OffsetPoint(rearLower, frameTopOffset, upperOffset, cosAngle, sinAngle);

            return new List<Point> { frontLower, rearLower, rearUpper, frontUpper };
        }

        private Point OffsetPoint(Point basePoint, double offset, double multiplier, double cosAngle, double sinAngle) =>
            new(
                basePoint.X + sinAngle * offset * multiplier,
                basePoint.Y - cosAngle * offset * multiplier
            );

        public (List<SkeletonPoint> Points, List<SkeletonLine> Lines) GetSkeleton()
        {
            var points = new List<SkeletonPoint>();
            var lines = new List<SkeletonLine>();

            AddBasicElements(points, lines);
            AddDetailElements(points, lines);
            AddEnhancedElements(points, lines);

            return (points, lines);
        }

        private void AddBasicElements(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var wheelPositions = _bike.WheelPositions;
            var attachmentPoints = _bike.AttachmentPoints;

            // Добавляем основные точки
            points.Add(new SkeletonPoint(wheelPositions.Front, SkeletonPointType.FrontWheel));
            points.Add(new SkeletonPoint(wheelPositions.Rear, SkeletonPointType.RearWheel));
            points.Add(new SkeletonPoint(attachmentPoints.Front, SkeletonPointType.FrontSuspension));
            points.Add(new SkeletonPoint(attachmentPoints.Rear, SkeletonPointType.RearSuspension));

            // Добавляем раму
            var chassisPoints = GetChassisPoints();
            int frameStart = points.Count;
            points.AddRange(chassisPoints.Select(p => new SkeletonPoint(p, SkeletonPointType.Frame)));

            // Добавляем линии подвески
            lines.Add(new SkeletonLine(0, 2, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(1, 3, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(2, frameStart, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(3, frameStart + 1, SkeletonLineType.Suspension));

            // Замыкаем раму
            for (int i = frameStart; i < frameStart + chassisPoints.Count - 1; i++)
                lines.Add(new SkeletonLine(i, i + 1, SkeletonLineType.MainFrame));
            lines.Add(new SkeletonLine(frameStart + chassisPoints.Count - 1, frameStart, SkeletonLineType.MainFrame));

            // Добавляем колеса
            AddWheelSpokes(points, lines);
        }

        private void AddWheelSpokes(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            AddWheelSpokesForWheel(points, lines, 0, _bike.WheelPositions.Front, _bike.WheelRotations.Front);
            AddWheelSpokesForWheel(points, lines, 1, _bike.WheelPositions.Rear, _bike.WheelRotations.Rear);
        }

        private void AddWheelSpokesForWheel(List<SkeletonPoint> points, List<SkeletonLine> lines, int wheelIndex, Point center, double rotation)
        {
            int spokeCount = _bike.BikeType switch
            {
                BikeType.Sport => 20,
                BikeType.OffRoad => 12,
                _ => 16
            };

            double wheelRadius = _bike.GetWheelRadius();
            int centerPointIndex = wheelIndex;
            var spokeEndIndices = new List<int>(spokeCount);
            double groundY = center.Y + wheelRadius;

            for (int i = 0; i < spokeCount; i++)
            {
                double angle = rotation + i * (2 * PI / spokeCount);
                double effectiveRadius = wheelRadius;

                Point spokeEnd = Offset(center, wheelRadius, 0, angle);
                if (!_bike.IsInAir && spokeEnd.Y >= groundY - 1)
                {
                    double penetration = spokeEnd.Y - (groundY - 1);
                    effectiveRadius = wheelRadius * (1.0 - Min(0.2, penetration / wheelRadius));
                }

                spokeEnd = Offset(center, effectiveRadius, 0, angle);
                points.Add(new SkeletonPoint(spokeEnd, SkeletonPointType.Wheel));
                int spokeEndIndex = points.Count - 1;
                spokeEndIndices.Add(spokeEndIndex);
                lines.Add(new SkeletonLine(centerPointIndex, spokeEndIndex, SkeletonLineType.Wheel));
            }

            for (int i = 0; i < spokeCount; i++)
                lines.Add(new SkeletonLine(spokeEndIndices[i], spokeEndIndices[(i + 1) % spokeCount], SkeletonLineType.Wheel));
        }

        private void AddDetailElements(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            AddSeat(points, lines);
            AddHandlebar(points, lines);
            AddExhaust(points, lines);
        }

        private void AddSeat(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var rearUpper = GetChassisPoints()[2];

            (double xOffset, double width, double height) = _bike.BikeType switch
            {
                BikeType.Sport => (-15, 35, 3),
                BikeType.OffRoad => (-10, 45, 4),
                _ => (-12, 40, 3)
            };

            Point seatStart = Offset(rearUpper, xOffset, -5, _bike.Angle);
            int startIndex = points.Count;

            points.Add(new SkeletonPoint(seatStart, SkeletonPointType.Seat));
            points.Add(new SkeletonPoint(Offset(seatStart, width, 0, _bike.Angle), SkeletonPointType.Seat));
            points.Add(new SkeletonPoint(Offset(seatStart, width, height, _bike.Angle), SkeletonPointType.Seat));
            points.Add(new SkeletonPoint(Offset(seatStart, 0, height, _bike.Angle), SkeletonPointType.Seat));

            AddRectangleLines(lines, startIndex, SkeletonLineType.Seat);
        }

        private void AddHandlebar(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var frontUpper = GetChassisPoints()[3];

            (double handleWidth, double stemLength) = _bike.BikeType switch
            {
                BikeType.Sport => (HandlebarWidth * 1.1, HandlebarStemLength * 0.8),
                BikeType.OffRoad => (HandlebarWidth * 1.2, HandlebarStemLength * 1.2),
                _ => (HandlebarWidth, HandlebarStemLength)
            };

            double steeringOffset = _bike.LeanAmount * 5.0;
            int startIndex = points.Count;

            Point stemBase = frontUpper;
            points.Add(new SkeletonPoint(stemBase, SkeletonPointType.Handlebar));

            Point stemTop = Offset(stemBase, 0, -stemLength, _bike.Angle);
            stemTop = Offset(stemTop, steeringOffset, 0, _bike.Angle);
            points.Add(new SkeletonPoint(stemTop, SkeletonPointType.Handlebar));

            points.Add(new SkeletonPoint(Offset(stemTop, -handleWidth / 2, 0, _bike.Angle), SkeletonPointType.Handlebar));
            points.Add(new SkeletonPoint(Offset(stemTop, handleWidth / 2, 0, _bike.Angle), SkeletonPointType.Handlebar));

            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.Handlebar));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, SkeletonLineType.Handlebar));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 3, SkeletonLineType.Handlebar));
        }

        private void AddExhaust(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var exhaustBase = GetChassisPoints()[1];

            (double pipeLength, double pipeHeight, double downOffset, double rightOffset) = _bike.BikeType switch
            {
                BikeType.Sport => (12, 0.8, 1.5, 1.2),
                BikeType.OffRoad => (20, 2.0, 0.5, 0.8),
                _ => (15, 1, 1, 1)
            };

            int startIndex = points.Count;

            points.Add(new SkeletonPoint(exhaustBase, SkeletonPointType.Exhaust));
            points.Add(new SkeletonPoint(Offset(exhaustBase, 0, rightOffset + downOffset, _bike.Angle + PI / 2), SkeletonPointType.Exhaust));
            points.Add(new SkeletonPoint(Offset(exhaustBase, -pipeLength, rightOffset + pipeHeight + downOffset, _bike.Angle), SkeletonPointType.Exhaust));
            points.Add(new SkeletonPoint(Offset(exhaustBase, -pipeLength, rightOffset, _bike.Angle), SkeletonPointType.Exhaust));

            AddRectangleLines(lines, startIndex, SkeletonLineType.Exhaust);
        }

        private void AddRectangleLines(List<SkeletonLine> lines, int startIndex, SkeletonLineType type)
        {
            lines.Add(new SkeletonLine(startIndex, startIndex + 1, type));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, type));
            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 3, type));
            lines.Add(new SkeletonLine(startIndex + 3, startIndex, type));
        }

        private void AddEnhancedElements(List<SkeletonPoint> points, List<SkeletonLine> lines)
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

            (double engineWidth, double engineHeight, double xOffset, double yOffset) = _bike.BikeType switch
            {
                BikeType.Sport => (EngineWidth * 0.9, EngineHeight * 0.8, -15, 5),
                BikeType.OffRoad => (EngineWidth * 1.1, EngineHeight * 1.2, -12, 8),
                _ => (EngineWidth, EngineHeight, -10, 7)
            };

            double enginePositionRatio = 0.3;
            Point engineCenter = new Point(
                frontLower.X - (frontLower.X - rearLower.X) * enginePositionRatio + xOffset,
                frontLower.Y - (frontLower.Y - rearLower.Y) * enginePositionRatio + yOffset
            );

            double halfWidth = engineWidth / 2;
            double halfHeight = engineHeight / 2;
            int startIndex = points.Count;

            // Основной блок двигателя
            points.Add(new SkeletonPoint(Offset(engineCenter, -halfWidth, -halfHeight * 0.7, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(Offset(engineCenter, halfWidth, -halfHeight * 0.7, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(Offset(engineCenter, halfWidth, halfHeight, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(Offset(engineCenter, -halfWidth, halfHeight, _bike.Angle), SkeletonPointType.Engine));

            AddRectangleLines(lines, startIndex, SkeletonLineType.Engine);

            // Цилиндр двигателя
            AddEngineCylinder(points, lines, engineCenter, halfWidth, halfHeight);
        }

        private void AddEngineCylinder(List<SkeletonPoint> points, List<SkeletonLine> lines, Point engineCenter, double width, double height)
        {
            (double cylinderWidth, double cylinderHeight, double offsetX, double offsetY) = _bike.BikeType switch
            {
                BikeType.Sport => (width * 0.7, width * 0.5, width * 0.9, -height * 0.2),
                BikeType.OffRoad => (width * 0.8, width * 0.6, width * 0.9, -height * 0.1),
                _ => (width * 0.7, width * 0.5, width * 0.9, -height * 0.15)
            };

            Point cylinderBase = Offset(engineCenter, offsetX, offsetY, _bike.Angle);
            int startIndex = points.Count;

            points.Add(new SkeletonPoint(cylinderBase, SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(Offset(cylinderBase, cylinderWidth, 0, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(Offset(cylinderBase, cylinderWidth, cylinderHeight, _bike.Angle), SkeletonPointType.Engine));
            points.Add(new SkeletonPoint(Offset(cylinderBase, 0, cylinderHeight, _bike.Angle), SkeletonPointType.Engine));

            AddRectangleLines(lines, startIndex, SkeletonLineType.Engine);
        }

        private void AddChain(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            Point rearWheel = _bike.WheelPositions.Rear;
            var chassisPoints = GetChassisPoints();

            // Расчет позиции двигателя (аналогично методу AddEngine)
            double enginePositionRatio = 0.3;
            Point engineCenter = new Point(
                chassisPoints[0].X - (chassisPoints[0].X - chassisPoints[1].X) * enginePositionRatio - 10,
                chassisPoints[0].Y - (chassisPoints[0].Y - chassisPoints[1].Y) * enginePositionRatio + 7
            );

            double halfWidth = EngineWidth / 2;
            double halfHeight = EngineHeight / 2;
            Point engineSprocket = Offset(engineCenter, -halfWidth, halfHeight, _bike.Angle);

            double wheelRadius = _bike.GetWheelRadius() * 0.7;
            double sprocketRadius = wheelRadius * 0.3;
            double rearSprocketRadius = wheelRadius * 0.5;
            int segmentCount = 8;
            int startIndex = points.Count;

            // Верхняя часть цепи
            points.Add(new SkeletonPoint(engineSprocket, SkeletonPointType.Chain));
            for (int i = 1; i < segmentCount; i++)
            {
                double ratio = (double)i / segmentCount;
                double x = engineSprocket.X - (engineSprocket.X - rearWheel.X) * ratio;
                double y = engineSprocket.Y - (engineSprocket.Y - rearWheel.Y) * ratio;
                double sag = Sin(ratio * PI) * 3;
                points.Add(new SkeletonPoint(new Point(x, y + sag), SkeletonPointType.Chain));
            }

            // Нижняя часть цепи
            points.Add(new SkeletonPoint(Offset(rearWheel, -rearSprocketRadius, 0, _bike.Angle), SkeletonPointType.Chain));
            for (int i = segmentCount - 1; i > 0; i--)
            {
                double ratio = (double)i / segmentCount;
                double x = engineSprocket.X - (engineSprocket.X - rearWheel.X) * ratio;
                double y = engineSprocket.Y - (engineSprocket.Y - rearWheel.Y) * ratio;
                double tension = -Sin(ratio * PI) * 1.5;
                points.Add(new SkeletonPoint(new Point(x, y + tension), SkeletonPointType.Chain));
            }

            // Соединяем точки цепи
            int totalPoints = 2 * segmentCount;
            for (int i = 0; i < totalPoints; i++)
                lines.Add(new SkeletonLine(startIndex + i, startIndex + (i + 1) % totalPoints, SkeletonLineType.Chain));

            // Добавляем звездочки
            AddSprockets(points, lines, engineSprocket, rearWheel, sprocketRadius, rearSprocketRadius);
        }

        private void AddSprockets(List<SkeletonPoint> points, List<SkeletonLine> lines,
                                Point engineSprocket, Point rearWheel, double sprocketRadius, double rearSprocketRadius)
        {
            int sprocketSegments = 8;

            // Звездочка двигателя
            int frontStartIndex = points.Count;
            for (int i = 0; i < sprocketSegments; i++)
            {
                double angle = i * (2 * PI / sprocketSegments);
                points.Add(new SkeletonPoint(Offset(engineSprocket, sprocketRadius, 0, angle), SkeletonPointType.Chain));
            }

            for (int i = 0; i < sprocketSegments; i++)
                lines.Add(new SkeletonLine(frontStartIndex + i, frontStartIndex + (i + 1) % sprocketSegments, SkeletonLineType.Chain));

            // Звездочка заднего колеса
            int rearStartIndex = points.Count;
            for (int i = 0; i < sprocketSegments; i++)
            {
                double angle = i * (2 * PI / sprocketSegments);
                points.Add(new SkeletonPoint(Offset(rearWheel, rearSprocketRadius, 0, angle), SkeletonPointType.Chain));
            }

            for (int i = 0; i < sprocketSegments; i++)
                lines.Add(new SkeletonLine(rearStartIndex + i, rearStartIndex + (i + 1) % sprocketSegments, SkeletonLineType.Chain));
        }

        private void AddFrontForks(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            Point frontWheel = _bike.WheelPositions.Front;
            Point frontAttachment = _bike.AttachmentPoints.Front;
            double forkOffset = ForkThickness / 2;
            double wheelRadius = _bike.GetWheelRadius();
            int startIndex = points.Count;

            Point forkTop1 = Offset(frontAttachment, -forkOffset, 0, _bike.Angle);
            Point forkTop2 = Offset(frontAttachment, forkOffset, 0, _bike.Angle);
            Point forkBottom1 = Offset(frontWheel, -forkOffset, -wheelRadius * 0.1, _bike.Angle);
            Point forkBottom2 = Offset(frontWheel, forkOffset, -wheelRadius * 0.1, _bike.Angle);

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

            // Вычисляем нормализованный вектор направления маятника
            double dx = rearWheel.X - rearAttachment.X;
            double dy = rearWheel.Y - rearAttachment.Y;
            double length = Sqrt(dx * dx + dy * dy);
            double normalizedX = dx / length;
            double normalizedY = dy / length;

            // Перпендикулярный вектор для смещения в стороны
            double perpX = -normalizedY;
            double perpY = normalizedX;

            int startIndex = points.Count;
            points.Add(new SkeletonPoint(rearAttachment, SkeletonPointType.ShockAbsorber));

            // Создаем точки маятника
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

            // Соединяем точки линиями
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 3, SkeletonLineType.ShockAbsorber));
            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 4, SkeletonLineType.ShockAbsorber));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, SkeletonLineType.ShockAbsorber));
            lines.Add(new SkeletonLine(startIndex + 3, startIndex + 4, SkeletonLineType.ShockAbsorber));
        }

        private void AddRearShockAbsorber(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            Point rearUpper = GetChassisPoints()[2];
            Point rearAttachment = _bike.AttachmentPoints.Rear;
            double shockWidth = _bike.BikeType switch
            {
                BikeType.Sport => ShockAbsorberWidth * 0.8,
                BikeType.OffRoad => ShockAbsorberWidth * 1.2,
                _ => ShockAbsorberWidth
            };

            // Расчет сжатия амортизатора
            double compressionRatio = 1.0;
            if (!_bike.IsInAir)
            {
                double suspensionOffset = _bike.SuspensionOffsets.Rear;
                double restLength = _bike.GetWheelRadius() * 2;
                compressionRatio = suspensionOffset / restLength;
            }

            int startIndex = points.Count;
            Point shockTop = Offset(rearUpper, -5, 0, _bike.Angle);

            // Расчет нижней точки амортизатора
            double dx = _bike.WheelPositions.Rear.X - rearAttachment.X;
            double dy = _bike.WheelPositions.Rear.Y - rearAttachment.Y;
            double attachRatio = 0.3;
            Point shockBottom = new Point(
                rearAttachment.X + dx * attachRatio + (1.0 - compressionRatio) * 3,
                rearAttachment.Y + dy * attachRatio - 5 * (1.0 - compressionRatio)
            );

            points.Add(new SkeletonPoint(shockTop, SkeletonPointType.ShockAbsorber));
            points.Add(new SkeletonPoint(shockBottom, SkeletonPointType.ShockAbsorber));
            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.ShockAbsorber));

            // Добавляем пружину амортизатора
            Point shockMid = new Point(
                (shockTop.X + shockBottom.X) / 2,
                (shockTop.Y + shockBottom.Y) / 2
            );

            double shockAngle = Atan2(shockBottom.Y - shockTop.Y, shockBottom.X - shockTop.X);
            double perpAngle = shockAngle + PI / 2;

            Point spring1 = Offset(shockMid, 0, shockWidth / 2, perpAngle);
            Point spring2 = Offset(shockMid, 0, -shockWidth / 2, perpAngle);

            points.Add(new SkeletonPoint(spring1, SkeletonPointType.ShockAbsorber));
            points.Add(new SkeletonPoint(spring2, SkeletonPointType.ShockAbsorber));
            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 3, SkeletonLineType.ShockAbsorber));
        }
    }
}