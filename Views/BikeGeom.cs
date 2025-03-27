#nullable enable
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using System;
using static System.Math;
using static GravityDefiedGame.Models.PhysicsComponent;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Wheels;

namespace GravityDefiedGame.Models
{
    public class BikeGeom : DrawingComponent
    {
        private static class GeomConstants
        {
            public const float
                FrameHeightMultiplier = 1.2f,
                LowerFrameOffsetMultiplier = 0.15f,
                UpperFrameOffsetMultiplier = 0.4f;

            public const float
                SeatOffsetX1 = -20.0f,
                SeatOffsetX2 = 25.0f,
                SeatOffsetY = -10.0f;

            public const float
                HandlebarWidth = 24.0f,
                HandlebarHeight = 8.0f,
                HandlebarStemLength = 15.0f;

            public const float
                ExhaustOffsetX1 = -15.0f,
                ExhaustOffsetX2 = -28.0f,
                ExhaustOffsetX3 = -32.0f,
                ExhaustOffsetX4 = -20.0f,
                ExhaustOffsetY1 = -12.0f,
                ExhaustOffsetY2 = -18.0f,
                ExhaustOffsetY3 = -6.0f,
                ExhaustOffsetY4 = 0.0f;

            public const float
                ShockAbsorberWidth = 5.0f,
                ForkThickness = 3.0f,
                ChainOffsetY = 3.0f,
                EngineOffsetY = -5.0f,
                EngineWidth = 20.0f,
                EngineHeight = 15.0f;
        }

        private readonly Motorcycle _bike;

        public record struct SkeletonPoint(Vector2 Position, SkeletonPointType Type);
        public enum SkeletonPointType { FrontWheel, RearWheel, FrontSuspension, RearSuspension, Frame, Seat, Handlebar, Exhaust, Wheel, Engine, Chain, ShockAbsorber, Fork }

        public record struct SkeletonLine(int StartPointIndex, int EndPointIndex, SkeletonLineType Type);
        public enum SkeletonLineType { MainFrame, Suspension, Wheel, Seat, Handlebar, Exhaust, Engine, Chain, ShockAbsorber, Fork }

        public BikeGeom(Motorcycle bike) => _bike = bike;

        private float GetScaleFactor() => _bike.BikeType switch
        {
            BikeType.Sport => 0.9f,
            BikeType.OffRoad => 1.1f,
            _ => 1.0f
        };

        public List<Vector2> GetFramePoints()
        {
            var (cosAngle, sinAngle) = GetTrigsFromAngle(_bike.Angle);
            float halfWheelBase = _bike.WheelBase / 2;
            float frameHeight = _bike.FrameHeight * 0.8f;

            return new List<Vector2>
            {
                CreateFramePoint(halfWheelBase, frameHeight, cosAngle, sinAngle, true),
                CreateFramePoint(halfWheelBase, frameHeight, cosAngle, sinAngle, false)
            };
        }

        private Vector2 CreateFramePoint(float halfWheelBase, float frameHeight, double cosAngle, double sinAngle, bool isFront)
        {
            float multiplier = isFront ? 0.7f : -0.7f;
            return new Vector2(
                _bike.Position.X + halfWheelBase * multiplier * (float)cosAngle - frameHeight * (float)sinAngle,
                _bike.Position.Y + halfWheelBase * multiplier * (float)sinAngle + frameHeight * (float)cosAngle
            );
        }

        public List<Vector2> GetChassisPoints()
        {
            float scaleFactor = GetScaleFactor();
            float frameTopOffset = _bike.GetWheelRadius() * GeomConstants.FrameHeightMultiplier * scaleFactor;
            var (cosAngle, sinAngle) = GetTrigsFromAngle(_bike.Angle);

            float lowerOffset = GeomConstants.LowerFrameOffsetMultiplier;
            float upperOffset = GeomConstants.UpperFrameOffsetMultiplier;

            (lowerOffset, upperOffset) = _bike.BikeType switch
            {
                BikeType.Sport => (
                    lowerOffset,
                    upperOffset * 1.2f * (1 + _bike.Velocity.Length() * 0.0005f)
                ),
                BikeType.OffRoad => (
                    lowerOffset * 0.9f * (_bike.IsInAir ? 0.9f : 1.0f),
                    upperOffset
                ),
                _ => (lowerOffset, upperOffset)
            };

            if (_bike.IsInWheelie)
            {
                lowerOffset *= (1.0f - _bike.WheelieIntensity * 0.2f);
                upperOffset *= (1.0f + _bike.WheelieIntensity * 0.1f);
            }
            else if (_bike.IsInStoppie)
            {
                lowerOffset *= (1.0f + _bike.StoppieIntensity * 0.1f);
                upperOffset *= (1.0f - _bike.StoppieIntensity * 0.2f);
            }

            Vector2 frontLower = OffsetPoint(_bike.AttachmentPoints.Front, frameTopOffset, lowerOffset, cosAngle, sinAngle);
            Vector2 rearLower = OffsetPoint(_bike.AttachmentPoints.Rear, frameTopOffset, lowerOffset, cosAngle, sinAngle);
            Vector2 frontUpper = OffsetPoint(frontLower, frameTopOffset, upperOffset, cosAngle, sinAngle);
            Vector2 rearUpper = OffsetPoint(rearLower, frameTopOffset, upperOffset, cosAngle, sinAngle);

            return new List<Vector2> { frontLower, rearLower, rearUpper, frontUpper };
        }

        private Vector2 OffsetPoint(Vector2 basePoint, float offset, float multiplier, double cosAngle, double sinAngle) =>
            new(
                basePoint.X + (float)sinAngle * offset * multiplier,
                basePoint.Y - (float)cosAngle * offset * multiplier
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

            points.Add(new SkeletonPoint(wheelPositions.Front, SkeletonPointType.FrontWheel));
            points.Add(new SkeletonPoint(wheelPositions.Rear, SkeletonPointType.RearWheel));
            points.Add(new SkeletonPoint(attachmentPoints.Front, SkeletonPointType.FrontSuspension));
            points.Add(new SkeletonPoint(attachmentPoints.Rear, SkeletonPointType.RearSuspension));

            var chassisPoints = GetChassisPoints();
            int frameStart = points.Count;
            points.AddRange(chassisPoints.Select(p => new SkeletonPoint(p, SkeletonPointType.Frame)));

            lines.Add(new SkeletonLine(0, 2, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(1, 3, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(2, frameStart, SkeletonLineType.Suspension));
            lines.Add(new SkeletonLine(3, frameStart + 1, SkeletonLineType.Suspension));

            for (int i = frameStart; i < frameStart + chassisPoints.Count - 1; i++)
                lines.Add(new SkeletonLine(i, i + 1, SkeletonLineType.MainFrame));
            lines.Add(new SkeletonLine(frameStart + chassisPoints.Count - 1, frameStart, SkeletonLineType.MainFrame));

            AddWheelSpokes(points, lines);
        }

        private void AddWheelSpokes(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            AddWheelSpokesForWheel(points, lines, 0, _bike.WheelPositions.Front, _bike.WheelRotations.Front);
            AddWheelSpokesForWheel(points, lines, 1, _bike.WheelPositions.Rear, _bike.WheelRotations.Rear);
        }

        private void AddWheelSpokesForWheel(List<SkeletonPoint> points, List<SkeletonLine> lines, int wheelIndex, Vector2 center, float rotation)
        {
            int spokeCount = _bike.BikeType switch
            {
                BikeType.Sport => 20,
                BikeType.OffRoad => 12,
                _ => 16
            };

            float wheelRadius = _bike.GetWheelRadius();
            int centerPointIndex = wheelIndex;
            var spokeEndIndices = new List<int>(spokeCount);

            for (int i = 0; i < spokeCount; i++)
            {
                double angle = rotation + i * (2 * Math.PI / spokeCount);
                Vector2 spokeEnd = Offset(center, wheelRadius, 0, (float)angle);
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
            if (_bike.IsInWheelie)
            {
                AddWheelieAdjustedSeat(points, lines);
                AddHandlebar(points, lines);
            }
            else if (_bike.IsInStoppie)
            {
                AddSeat(points, lines);
                AddStoppieAdjustedHandlebar(points, lines);
            }
            else
            {
                AddSeat(points, lines);
                AddHandlebar(points, lines);
            }

            AddExhaust(points, lines);
        }

        private void AddWheelieAdjustedSeat(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var rearUpper = GetChassisPoints()[2];

            float wheelieAdjust = _bike.WheelieIntensity * 5.0f;

            (float xOffset, float width, float height) = _bike.BikeType switch
            {
                BikeType.Sport => (-15f - wheelieAdjust, 35f, 3f),
                BikeType.OffRoad => (-10f - wheelieAdjust * 1.5f, 45f, 4f),
                _ => (-12f - wheelieAdjust * 1.2f, 40f, 3f)
            };

            Vector2 seatStart = Offset(rearUpper, xOffset, -5f, _bike.Angle);

            AddRectangle(points, lines, seatStart, width, height, _bike.Angle,
                       SkeletonPointType.Seat, SkeletonLineType.Seat);
        }

        private void AddSeat(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var rearUpper = GetChassisPoints()[2];

            (float xOffset, float width, float height) = _bike.BikeType switch
            {
                BikeType.Sport => (-15f, 35f, 3f),
                BikeType.OffRoad => (-10f, 45f, 4f),
                _ => (-12f, 40f, 3f)
            };

            Vector2 seatStart = Offset(rearUpper, xOffset, -5f, _bike.Angle);

            AddRectangle(points, lines, seatStart, width, height, _bike.Angle,
                       SkeletonPointType.Seat, SkeletonLineType.Seat);
        }

        private void AddStoppieAdjustedHandlebar(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var frontUpper = GetChassisPoints()[3];

            float stoppieAdjust = _bike.StoppieIntensity * 5.0f;

            (float handleWidth, float stemLength) = _bike.BikeType switch
            {
                BikeType.Sport => (GeomConstants.HandlebarWidth * 1.1f, GeomConstants.HandlebarStemLength * 0.8f + stoppieAdjust),
                BikeType.OffRoad => (GeomConstants.HandlebarWidth * 1.2f, GeomConstants.HandlebarStemLength * 1.2f + stoppieAdjust * 1.5f),
                _ => (GeomConstants.HandlebarWidth, GeomConstants.HandlebarStemLength + stoppieAdjust * 1.2f)
            };

            float steeringOffset = _bike.LeanAmount * 5.0f;
            int startIndex = points.Count;

            Vector2 stemBase = frontUpper;
            points.Add(new SkeletonPoint(stemBase, SkeletonPointType.Handlebar));

            Vector2 stemTop = Offset(stemBase, 0, -stemLength, _bike.Angle);
            stemTop = Offset(stemTop, steeringOffset, 0, _bike.Angle);
            points.Add(new SkeletonPoint(stemTop, SkeletonPointType.Handlebar));

            points.Add(new SkeletonPoint(Offset(stemTop, -handleWidth / 2, 0, _bike.Angle), SkeletonPointType.Handlebar));
            points.Add(new SkeletonPoint(Offset(stemTop, handleWidth / 2, 0, _bike.Angle), SkeletonPointType.Handlebar));

            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.Handlebar));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, SkeletonLineType.Handlebar));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 3, SkeletonLineType.Handlebar));
        }

        private void AddHandlebar(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            var frontUpper = GetChassisPoints()[3];

            (float handleWidth, float stemLength) = _bike.BikeType switch
            {
                BikeType.Sport => (GeomConstants.HandlebarWidth * 1.1f, GeomConstants.HandlebarStemLength * 0.8f),
                BikeType.OffRoad => (GeomConstants.HandlebarWidth * 1.2f, GeomConstants.HandlebarStemLength * 1.2f),
                _ => (GeomConstants.HandlebarWidth, GeomConstants.HandlebarStemLength)
            };

            float steeringOffset = _bike.LeanAmount * 5.0f;
            int startIndex = points.Count;

            Vector2 stemBase = frontUpper;
            points.Add(new SkeletonPoint(stemBase, SkeletonPointType.Handlebar));

            Vector2 stemTop = Offset(stemBase, 0, -stemLength, _bike.Angle);
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

            (float pipeLength, float pipeHeight, float downOffset, float rightOffset) = _bike.BikeType switch
            {
                BikeType.Sport => (12f, 0.8f, 1.5f, 1.2f),
                BikeType.OffRoad => (20f, 2.0f, 0.5f, 0.8f),
                _ => (15f, 1f, 1f, 1f)
            };

            int startIndex = points.Count;

            points.Add(new SkeletonPoint(exhaustBase, SkeletonPointType.Exhaust));
            points.Add(new SkeletonPoint(Offset(exhaustBase, 0, rightOffset + downOffset, _bike.Angle + (float)Math.PI / 2), SkeletonPointType.Exhaust));
            points.Add(new SkeletonPoint(Offset(exhaustBase, -pipeLength, rightOffset + pipeHeight + downOffset, _bike.Angle), SkeletonPointType.Exhaust));
            points.Add(new SkeletonPoint(Offset(exhaustBase, -pipeLength, rightOffset, _bike.Angle), SkeletonPointType.Exhaust));

            AddRectangleLines(lines, startIndex, SkeletonLineType.Exhaust);
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
            Vector2 frontLower = chassisPoints[0];
            Vector2 rearLower = chassisPoints[1];

            (float engineWidth, float engineHeight, float xOffset, float yOffset) = _bike.BikeType switch
            {
                BikeType.Sport => (GeomConstants.EngineWidth * 0.9f, GeomConstants.EngineHeight * 0.8f, -15f, 5f),
                BikeType.OffRoad => (GeomConstants.EngineWidth * 1.1f, GeomConstants.EngineHeight * 1.2f, -12f, 8f),
                _ => (GeomConstants.EngineWidth, GeomConstants.EngineHeight, -10f, 7f)
            };

            float enginePositionRatio = 0.3f;
            Vector2 engineCenter = new Vector2(
                frontLower.X - (frontLower.X - rearLower.X) * enginePositionRatio + xOffset,
                frontLower.Y - (frontLower.Y - rearLower.Y) * enginePositionRatio + yOffset
            );

            if (_bike.Throttle > 0.3f)
            {
                float vibrationX = (float)Math.Sin(_bike.WheelRotations.Rear * 10) * _bike.Throttle * 0.5f;
                float vibrationY = (float)Math.Cos(_bike.WheelRotations.Rear * 15) * _bike.Throttle * 0.3f;

                engineCenter = new Vector2(
                    engineCenter.X + vibrationX,
                    engineCenter.Y + vibrationY
                );
            }

            float halfWidth = engineWidth / 2;
            float halfHeight = engineHeight / 2;

            AddRectangle(
                points, lines,
                Offset(engineCenter, -halfWidth, -halfHeight * 0.7f, _bike.Angle),
                engineWidth, engineHeight * 1.7f, _bike.Angle,
                SkeletonPointType.Engine, SkeletonLineType.Engine
            );

            AddEngineCylinder(points, lines, engineCenter, halfWidth, halfHeight);
        }

        private void AddEngineCylinder(List<SkeletonPoint> points, List<SkeletonLine> lines, Vector2 engineCenter, float width, float height)
        {
            (float cylinderWidth, float cylinderHeight, float offsetX, float offsetY) = _bike.BikeType switch
            {
                BikeType.Sport => (width * 0.7f, width * 0.5f, width * 0.9f, -height * 0.2f),
                BikeType.OffRoad => (width * 0.8f, width * 0.6f, width * 0.9f, -height * 0.1f),
                _ => (width * 0.7f, width * 0.5f, width * 0.9f, -height * 0.15f)
            };

            Vector2 cylinderBase = Offset(engineCenter, offsetX, offsetY, _bike.Angle);

            AddRectangle(
                points, lines, cylinderBase, cylinderWidth, cylinderHeight, _bike.Angle,
                SkeletonPointType.Engine, SkeletonLineType.Engine
            );
        }

        private void AddChain(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            Vector2 rearWheel = _bike.WheelPositions.Rear;
            var chassisPoints = GetChassisPoints();

            float enginePositionRatio = 0.3f;
            Vector2 engineCenter = new Vector2(
                chassisPoints[0].X - (chassisPoints[0].X - chassisPoints[1].X) * enginePositionRatio - 10f,
                chassisPoints[0].Y - (chassisPoints[0].Y - chassisPoints[1].Y) * enginePositionRatio + 7f
            );

            float halfWidth = GeomConstants.EngineWidth / 2;
            float halfHeight = GeomConstants.EngineHeight / 2;
            Vector2 engineSprocket = Offset(engineCenter, -halfWidth, halfHeight, _bike.Angle);

            float wheelRadius = _bike.GetWheelRadius() * 0.7f;
            float sprocketRadius = wheelRadius * 0.3f;
            float rearSprocketRadius = wheelRadius * 0.5f;
            int segmentCount = 8;
            int startIndex = points.Count;

            float acceleration = _bike.Throttle > 0 ? _bike.Throttle * 3.0f : 0;

            points.Add(new SkeletonPoint(engineSprocket, SkeletonPointType.Chain));
            for (int i = 1; i < segmentCount; i++)
            {
                float ratio = (float)i / segmentCount;
                float x = engineSprocket.X - (engineSprocket.X - rearWheel.X) * ratio;
                float y = engineSprocket.Y - (engineSprocket.Y - rearWheel.Y) * ratio;

                float sag = (float)Math.Sin(ratio * Math.PI) * (3f - acceleration);
                points.Add(new SkeletonPoint(new Vector2(x, y + sag), SkeletonPointType.Chain));
            }

            points.Add(new SkeletonPoint(Offset(rearWheel, -rearSprocketRadius, 0, _bike.Angle), SkeletonPointType.Chain));
            for (int i = segmentCount - 1; i > 0; i--)
            {
                float ratio = (float)i / segmentCount;
                float x = engineSprocket.X - (engineSprocket.X - rearWheel.X) * ratio;
                float y = engineSprocket.Y - (engineSprocket.Y - rearWheel.Y) * ratio;
                float tension = -(float)Math.Sin(ratio * Math.PI) * 1.5f;
                points.Add(new SkeletonPoint(new Vector2(x, y + tension), SkeletonPointType.Chain));
            }

            int totalPoints = 2 * segmentCount;
            for (int i = 0; i < totalPoints; i++)
                lines.Add(new SkeletonLine(startIndex + i, startIndex + (i + 1) % totalPoints, SkeletonLineType.Chain));

            AddSprockets(points, lines, engineSprocket, rearWheel, sprocketRadius, rearSprocketRadius);
        }

        private void AddSprockets(List<SkeletonPoint> points, List<SkeletonLine> lines,
                                Vector2 engineSprocket, Vector2 rearWheel, float sprocketRadius, float rearSprocketRadius)
        {
            int sprocketSegments = 8;

            int frontStartIndex = points.Count;
            for (int i = 0; i < sprocketSegments; i++)
            {
                double angle = i * (2 * Math.PI / sprocketSegments);
                points.Add(new SkeletonPoint(Offset(engineSprocket, sprocketRadius, 0, (float)angle), SkeletonPointType.Chain));
            }

            for (int i = 0; i < sprocketSegments; i++)
                lines.Add(new SkeletonLine(frontStartIndex + i, frontStartIndex + (i + 1) % sprocketSegments, SkeletonLineType.Chain));

            int rearStartIndex = points.Count;
            for (int i = 0; i < sprocketSegments; i++)
            {
                double angle = i * (2 * Math.PI / sprocketSegments);
                points.Add(new SkeletonPoint(Offset(rearWheel, rearSprocketRadius, 0, (float)angle), SkeletonPointType.Chain));
            }

            for (int i = 0; i < sprocketSegments; i++)
                lines.Add(new SkeletonLine(rearStartIndex + i, rearStartIndex + (i + 1) % sprocketSegments, SkeletonLineType.Chain));
        }

        private void AddFrontForks(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            Vector2 frontWheel = _bike.WheelPositions.Front;
            Vector2 frontAttachment = _bike.AttachmentPoints.Front;
            float forkOffset = GeomConstants.ForkThickness / 2;
            float wheelRadius = _bike.GetWheelRadius();
            int startIndex = points.Count;

            float suspensionOffset = _bike.SuspensionOffsets.Front;
            float restLength = _bike.GetWheelRadius() * 2;
            float compressionRatio = 1.0f - (suspensionOffset / restLength);

            float compressionVisualOffset = compressionRatio * 2.0f;

            Vector2 forkTop1 = Offset(frontAttachment, -forkOffset, 0, _bike.Angle);
            Vector2 forkTop2 = Offset(frontAttachment, forkOffset, 0, _bike.Angle);
            Vector2 forkBottom1 = Offset(frontWheel, -forkOffset, -wheelRadius * 0.1f - compressionVisualOffset, _bike.Angle);
            Vector2 forkBottom2 = Offset(frontWheel, forkOffset, -wheelRadius * 0.1f - compressionVisualOffset, _bike.Angle);

            points.Add(new SkeletonPoint(forkTop1, SkeletonPointType.Fork));
            points.Add(new SkeletonPoint(forkTop2, SkeletonPointType.Fork));
            points.Add(new SkeletonPoint(forkBottom1, SkeletonPointType.Fork));
            points.Add(new SkeletonPoint(forkBottom2, SkeletonPointType.Fork));

            lines.Add(new SkeletonLine(startIndex, startIndex + 2, SkeletonLineType.Fork));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 3, SkeletonLineType.Fork));
        }

        private void AddRearSwingArm(List<SkeletonPoint> points, List<SkeletonLine> lines)
        {
            Vector2 rearWheel = _bike.WheelPositions.Rear;
            Vector2 rearAttachment = _bike.AttachmentPoints.Rear;
            float swingArmWidth = 3.0f;

            float dx = rearWheel.X - rearAttachment.X;
            float dy = rearWheel.Y - rearAttachment.Y;
            float length = (float)Math.Sqrt(dx * dx + dy * dy);
            float normalizedX = dx / length;
            float normalizedY = dy / length;

            float perpX = -normalizedY;
            float perpY = normalizedX;

            int startIndex = points.Count;
            points.Add(new SkeletonPoint(rearAttachment, SkeletonPointType.ShockAbsorber));

            Vector2 swingArmTopFrame = new Vector2(
                rearAttachment.X + perpX * swingArmWidth / 2,
                rearAttachment.Y + perpY * swingArmWidth / 2
            );
            Vector2 swingArmBottomFrame = new Vector2(
                rearAttachment.X - perpX * swingArmWidth / 2,
                rearAttachment.Y - perpY * swingArmWidth / 2
            );
            Vector2 swingArmTopWheel = new Vector2(
                rearWheel.X + perpX * swingArmWidth / 2,
                rearWheel.Y + perpY * swingArmWidth / 2
            );
            Vector2 swingArmBottomWheel = new Vector2(
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
            Vector2 rearUpper = GetChassisPoints()[2];
            Vector2 rearAttachment = _bike.AttachmentPoints.Rear;

            float shockWidth = _bike.BikeType switch
            {
                BikeType.Sport => GeomConstants.ShockAbsorberWidth * 0.8f,
                BikeType.OffRoad => GeomConstants.ShockAbsorberWidth * 1.2f,
                _ => GeomConstants.ShockAbsorberWidth
            };

            float compressionRatio = 1.0f;
            if (!_bike.IsInAir)
            {
                float suspensionOffset = _bike.SuspensionOffsets.Rear;
                float restLength = _bike.GetWheelRadius() * 2;
                compressionRatio = suspensionOffset / restLength;
            }

            int startIndex = points.Count;
            Vector2 shockTop = Offset(rearUpper, -5, 0, _bike.Angle);

            float dx = _bike.WheelPositions.Rear.X - rearAttachment.X;
            float dy = _bike.WheelPositions.Rear.Y - rearAttachment.Y;
            float attachRatio = 0.3f;
            Vector2 shockBottom = new Vector2(
                rearAttachment.X + dx * attachRatio + (1.0f - compressionRatio) * 3,
                rearAttachment.Y + dy * attachRatio - 5 * (1.0f - compressionRatio)
            );

            points.Add(new SkeletonPoint(shockTop, SkeletonPointType.ShockAbsorber));
            points.Add(new SkeletonPoint(shockBottom, SkeletonPointType.ShockAbsorber));
            lines.Add(new SkeletonLine(startIndex, startIndex + 1, SkeletonLineType.ShockAbsorber));

            Vector2 shockMid = new Vector2(
                (shockTop.X + shockBottom.X) / 2,
                (shockTop.Y + shockBottom.Y) / 2
            );

            float shockAngle = (float)Math.Atan2(shockBottom.Y - shockTop.Y, shockBottom.X - shockTop.X);
            float perpAngle = shockAngle + (float)Math.PI / 2;

            float shockLength = Vector2.Distance(shockTop, shockBottom);
            int coilCount = 6;
            float coilSpacing = shockLength / (coilCount + 1);
            float coilWidth = shockWidth * 1.2f;
            float compressedSpacing = coilSpacing * compressionRatio;

            for (int i = 1; i <= coilCount; i++)
            {
                float t = i / (float)(coilCount + 1);
                Vector2 coilCenter = Vector2.Lerp(shockTop, shockBottom, t);

                Vector2 coil1 = Offset(coilCenter, 0, coilWidth / 2, perpAngle);
                Vector2 coil2 = Offset(coilCenter, 0, -coilWidth / 2, perpAngle);

                points.Add(new SkeletonPoint(coil1, SkeletonPointType.ShockAbsorber));
                points.Add(new SkeletonPoint(coil2, SkeletonPointType.ShockAbsorber));

                int idx = points.Count - 2;
                lines.Add(new SkeletonLine(idx, idx + 1, SkeletonLineType.ShockAbsorber));
            }
        }
    }
}