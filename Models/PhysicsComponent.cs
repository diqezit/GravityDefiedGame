using System;
using Microsoft.Xna.Framework;
using static System.Math;
using static GravityDefiedGame.Utilities.LoggerCore;
using static GravityDefiedGame.Utilities.Logger;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Models
{
    public abstract class PhysicsComponent
    {
        protected const float FullRotation = (float)(2 * PI);

        protected class TrigCache
        {
            public float Sin { get; private set; }
            public float Cos { get; private set; }

            public void Update(float angle) =>
                Log("TrigCache", "updating trig values", () =>
                {
                    Sin = (float)System.Math.Sin(angle);
                    Cos = (float)System.Math.Cos(angle);
                });
        }

        protected static float ClampValue(float value, float min, float max) =>
            Log("PhysicsComponent", "clamping value", () => MathHelper.Clamp(value, min, max), min);

        protected static float SafeDivide(float num, float den, float def = 0) =>
            Log("PhysicsComponent", "safe division", () =>
                Abs(den) < 1e-6f ? def : num / den, def);

        protected static float Lerp(float a, float b, float t) =>
            Log("PhysicsComponent", "linear interpolation", () =>
                MathHelper.Lerp(a, b, MathHelper.Clamp(t, 0f, 1f)), a);

        protected static Vector2 LerpVector(Vector2 a, Vector2 b, float t) =>
            Log("PhysicsComponent", "vector interpolation", () =>
                Vector2.Lerp(a, b, MathHelper.Clamp(t, 0f, 1f)), a);

        protected static float NormalizeAngle(float angle) =>
            Log("PhysicsComponent", "normalizing angle", () =>
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
            }, 0f);

        protected internal static T SanitizeValue<T>(T value, T def, string err) where T : IConvertible =>
            Log("PhysicsComponent", "sanitizing value", () =>
                float.IsNaN(Convert.ToSingle(value)) || float.IsInfinity(Convert.ToSingle(value))
                    ? LogErrorAndReturnDefault(err, value, def)
                    : value, def);

        private static T LogErrorAndReturnDefault<T>(string err, T value, T def)
        {
            Error("PhysicsComponent", $"{err}: {value}");
            return def;
        }

        protected internal static Vector2 SanitizeVector(Vector2 v, Vector2 def, string err) =>
            Log("PhysicsComponent", "sanitizing vector", () =>
                IsVectorInvalid(v) ? LogErrorAndReturnDefault(err, v, def) : v, def);

        protected internal static Vector2 SanitizePosition(Vector2 p, Vector2 def, string err) =>
            Log("PhysicsComponent", "sanitizing position", () =>
                IsVectorInvalid(p) ? LogErrorAndReturnDefault(err, p, def) : p, def);

        private static bool IsVectorInvalid(Vector2 v) =>
            Log("PhysicsComponent", "checking vector validity", () =>
                float.IsNaN(v.X) || float.IsNaN(v.Y) ||
                float.IsInfinity(v.X) || float.IsInfinity(v.Y), false);

        protected internal static bool ValidatePhysicalParameter(string param, float value, float minSafe, float maxSafe, string comp = "Unknown") =>
            Log("PhysicsComponent", $"validating parameter: {param}", () =>
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
            }, false);

        protected internal static bool ValidateVectorParameter(string param, Vector2 v, float maxMag, string comp = "Unknown") =>
            Log("PhysicsComponent", $"validating vector parameter: {param}", () =>
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
            }, false);

        internal static Vector2 Offset(Vector2 p, float dx, float dy, float angle) =>
            Log("PhysicsComponent", "offsetting point", () =>
            {
                var (c, s) = GetTrigsFromAngle(angle);
                return new Vector2(p.X + dx * c - dy * s, p.Y + dx * s + dy * c);
            }, p);

        internal static float CalculateDistance(Vector2 a, Vector2 b) =>
            Log("PhysicsComponent", "calculating distance", () => Vector2.Distance(a, b), 0f);

        internal static (float cos, float sin) GetTrigsFromAngle(float angle) =>
            Log("PhysicsComponent", "getting trigs from angle", () =>
                ((float)Cos(angle), (float)Sin(angle)), (1.0f, 0.0f));
    }
}