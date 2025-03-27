﻿using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using static System.Math;
using static GravityDefiedGame.Utilities.LoggerCore;
using static GravityDefiedGame.Utilities.Logger;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Models.BikeGeom;
using System.Collections.Generic;

namespace GravityDefiedGame.Models
{
    public abstract class PhysicsComponent
    {
        protected const float FullRotation = (float)(2 * PI);

        protected class TrigCache
        {
            public float Sin { get; private set; }
            public float Cos { get; private set; }

            public void Update(float angle)
            {
                Sin = (float)System.Math.Sin(angle);
                Cos = (float)System.Math.Cos(angle);
            }
        }

        protected static float ClampValue(float value, float min, float max) =>
            MathHelper.Clamp(value, min, max);

        protected static float SafeDivide(float num, float den, float def = 0) =>
            Abs(den) < 1e-6f ? def : num / den;

        protected static float Lerp(float a, float b, float t) =>
            MathHelper.Lerp(a, b, MathHelper.Clamp(t, 0f, 1f));

        protected static Vector2 LerpVector(Vector2 a, Vector2 b, float t) =>
            Vector2.Lerp(a, b, MathHelper.Clamp(t, 0f, 1f));

        protected static float NormalizeAngle(float angle)
        {
            const float twoPi = (float)(2 * PI);
            if (angle >= -PI && angle <= PI)
                return angle;

            angle %= twoPi;
            if (angle < 0)
                angle += twoPi;
            if (angle > PI)
                angle -= twoPi;
            return angle;
        }

        protected internal static T SanitizeValue<T>(T value, T def, string err) where T : IConvertible
        {
            if (float.IsNaN(Convert.ToSingle(value)) || float.IsInfinity(Convert.ToSingle(value)))
            {
                Error("PhysicsComponent", $"{err}: {value}");
                return def;
            }
            return value;
        }

        protected internal static Vector2 SanitizeVector(Vector2 v, Vector2 def, string err) =>
            IsVectorInvalid(v) ? LogErrorAndReturnDefault(err, v, def) : v;

        protected internal static Vector2 SanitizePosition(Vector2 p, Vector2 def, string err) =>
            IsVectorInvalid(p) ? LogErrorAndReturnDefault(err, p, def) : p;

        private static T LogErrorAndReturnDefault<T>(string err, T value, T def)
        {
            Error("PhysicsComponent", $"{err}: {value}");
            return def;
        }

        private static bool IsVectorInvalid(Vector2 v) =>
            float.IsNaN(v.X) || float.IsNaN(v.Y) ||
            float.IsInfinity(v.X) || float.IsInfinity(v.Y);

        protected internal static bool ValidatePhysicalParameter(string param, float value, float minSafe, float maxSafe, string comp = "Unknown")
        {
            if (float.IsNaN(value) || float.IsInfinity(value))
            {
                Error(comp, $"CRITICAL: {param} has invalid value: {value}");
                return false;
            }

            if (value < minSafe || value > maxSafe)
            {
                var lvl = (value < minSafe * 2 || value > maxSafe * 0.5) ? LogLevel.W : LogLevel.E;
                WriteLog(lvl, comp, $"{param} outside safe range: {value:F2} (safe: {minSafe:F2} to {maxSafe:F2})");
                return false;
            }

            return true;
        }

        protected internal static bool ValidateVectorParameter(string param, Vector2 v, float maxMag, string comp = "Unknown")
        {
            if (IsVectorInvalid(v))
            {
                Error(comp, $"CRITICAL: {param} has invalid values: {v}");
                return false;
            }

            if (v.Length() > maxMag)
            {
                var lvl = v.Length() > maxMag * 1.5 ? LogLevel.E : LogLevel.W;
                WriteLog(lvl, comp, $"{param} magnitude too high: {v.Length():F2} (max safe: {maxMag:F2})");
                return false;
            }

            return true;
        }

        internal static Vector2 Offset(Vector2 p, float dx, float dy, float angle)
        {
            var (c, s) = GetTrigsFromAngle(angle);
            return new Vector2(p.X + dx * c - dy * s, p.Y + dx * s + dy * c);
        }

        internal static float CalculateDistance(Vector2 a, Vector2 b) => Vector2.Distance(a, b);

        internal static (float cos, float sin) GetTrigsFromAngle(float angle) =>
            ((float)Cos(angle), (float)Sin(angle));

        public static bool IsValidPoint(Vector2 point) =>
            !float.IsNaN(point.X) && !float.IsNaN(point.Y) &&
            !float.IsInfinity(point.X) && !float.IsInfinity(point.Y);
    }

    public abstract class DrawingComponent
    {
        public static class DrawingConstants
        {
            public const float
                BikeStrokeThickness = 3.0f,
                TerrainStrokeThickness = 3.0f,
                VerticalLineStrokeThickness = 0.5f;

            public const float
                ShadowOpacity = 0.5f,
                ShadowOffsetFactor = 0.2f,
                ShadowScaleFactor = 0.01f;

            public const int
                ShadowInterpolationSteps = 5;

            public const float
                Perspective = 0.2f,
                FrameHeightMultiplier = 1.2f,
                LowerFrameOffsetMultiplier = 0.15f,
                UpperFrameOffsetMultiplier = 0.4f;
        }

        public static void DrawLine(SpriteBatch spriteBatch, Texture2D pixelTexture,
                                  Vector2 start, Vector2 end, Color color, float thickness)
        {
            if (!PhysicsComponent.IsValidPoint(start) || !PhysicsComponent.IsValidPoint(end)) return;

            float length = Vector2.Distance(start, end);
            if (length < 0.001f) return;

            float rotation = (float)Math.Atan2(end.Y - start.Y, end.X - start.X);

            spriteBatch.Draw(
                pixelTexture,
                start,
                null,
                color,
                rotation,
                Vector2.Zero,
                new Vector2(length, thickness),
                SpriteEffects.None,
                0
            );
        }

        public static void AddRectangle(
            List<SkeletonPoint> points,
            List<SkeletonLine> lines,
            Vector2 start,
            float width,
            float height,
            float angle,
            SkeletonPointType pointType,
            SkeletonLineType lineType)
        {
            int startIndex = points.Count;

            points.Add(new SkeletonPoint(start, pointType));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(start, width, 0, angle), pointType));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(start, width, height, angle), pointType));
            points.Add(new SkeletonPoint(PhysicsComponent.Offset(start, 0, height, angle), pointType));

            AddRectangleLines(lines, startIndex, lineType);
        }

        public static void AddRectangleLines(List<SkeletonLine> lines, int startIndex, SkeletonLineType type)
        {
            lines.Add(new SkeletonLine(startIndex, startIndex + 1, type));
            lines.Add(new SkeletonLine(startIndex + 1, startIndex + 2, type));
            lines.Add(new SkeletonLine(startIndex + 2, startIndex + 3, type));
            lines.Add(new SkeletonLine(startIndex + 3, startIndex, type));
        }
    }
}