using System;
using System.Windows;
using static System.Math;
using static GravityDefiedGame.Utilities.LoggerCore;
using static GravityDefiedGame.Utilities.Logger;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Models
{
    public abstract class PhysicsComponent
    {
        protected const double FullRotation = 2 * PI;

        protected class TrigCache
        {
            public double Sin { get; private set; }
            public double Cos { get; private set; }

            public void Update(double angle) =>
                Log("TrigCache", "updating trig values", () =>
                {
                    Sin = Sin(angle);
                    Cos = Cos(angle);
                });
        }

        // Utility methods
        protected static double ClampValue(double value, double min, double max) =>
            Log("PhysicsComponent", "clamping value", () => Max(min, Min(max, value)), min);

        protected static double SafeDivide(double num, double den, double def = 0) =>
            Log("PhysicsComponent", "safe division", () =>
                Abs(den) < 1e-10 ? def : num / den, def);

        protected static double Lerp(double a, double b, double t) =>
            Log("PhysicsComponent", "linear interpolation", () =>
                a + (b - a) * ClampValue(t, 0, 1), a);

        protected static Vector LerpVector(Vector a, Vector b, double t) =>
            Log("PhysicsComponent", "vector interpolation", () =>
            {
                double f = ClampValue(t, 0, 1);
                return new Vector(a.X + (b.X - a.X) * f, a.Y + (b.Y - a.Y) * f);
            }, a);

        protected static double NormalizeAngle(double angle) =>
            Log("PhysicsComponent", "normalizing angle", () =>
            {
                const double twoPi = 2 * PI;
                if (angle >= -PI && angle <= PI)
                {
                    return angle;
                }

                angle %= twoPi;
                if (angle < 0)
                {
                    angle += twoPi;
                }
                if (angle > PI)
                {
                    angle -= twoPi;
                }
                return angle;
            }, 0.0);

        // Sanitization
        protected internal static T SanitizeValue<T>(T value, T def, string err) where T : IConvertible =>
            Log("PhysicsComponent", "sanitizing value", () =>
                double.IsNaN(Convert.ToDouble(value)) || double.IsInfinity(Convert.ToDouble(value))
                    ? LogErrorAndReturnDefault(err, value, def)
                    : value, def);

        private static T LogErrorAndReturnDefault<T>(string err, T value, T def)
        {
            Error("PhysicsComponent", $"{err}: {value}");
            return def;
        }

        protected internal static Vector SanitizeVector(Vector v, Vector def, string err) =>
            Log("PhysicsComponent", "sanitizing vector", () =>
                IsVectorInvalid(v) ? LogErrorAndReturnDefault(err, v, def) : v, def);

        protected internal static Point SanitizePosition(Point p, Point def, string err) =>
            Log("PhysicsComponent", "sanitizing position", () =>
                IsPointInvalid(p) ? LogErrorAndReturnDefault(err, p, def) : p, def);

        private static bool IsVectorInvalid(Vector v) =>
            Log("PhysicsComponent", "checking vector validity", () =>
                double.IsNaN(v.X) || double.IsNaN(v.Y) ||
                double.IsInfinity(v.X) || double.IsInfinity(v.Y), false);

        private static bool IsPointInvalid(Point p) =>
            Log("PhysicsComponent", "checking point validity", () =>
                double.IsNaN(p.X) || double.IsNaN(p.Y) ||
                double.IsInfinity(p.X) || double.IsInfinity(p.Y), false);

        // Parameter Validation

        protected internal static bool ValidatePhysicalParameter(string param, double value, double minSafe, double maxSafe, string comp = "Unknown") =>
            Log("PhysicsComponent", $"validating parameter: {param}", () =>
            {
                if (double.IsNaN(value) || double.IsInfinity(value))
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

        protected internal static bool ValidateVectorParameter(string param, Vector v, double maxMag, string comp = "Unknown") =>
            Log("PhysicsComponent", $"validating vector parameter: {param}", () =>
            {
                if (IsVectorInvalid(v))
                {
                    Error(comp, $"CRITICAL: {param} has invalid values: {v}");
                    return false;
                }

                if (v.Length > maxMag)
                {
                    var lvl = v.Length > maxMag * 1.5 ? LogLevel.E : LogLevel.W;
                    WriteLog(lvl, comp, $"{param} magnitude too high: {v.Length:F2} (max safe: {maxMag:F2})");
                    return false;
                }

                return true;
            }, false);


        // Geometric & Position Update methods
        internal static Point Offset(Point p, double dx, double dy, double angle) =>
            Log("PhysicsComponent", "offsetting point", () =>
            {
                var (c, s) = GetTrigsFromAngle(angle);
                return new Point(p.X + dx * c - dy * s, p.Y + dx * s + dy * c);
            }, p);

        internal static double CalculateDistance(Point a, Point b) =>
            Log("PhysicsComponent", "calculating distance", () =>
            {
                double dx = a.X - b.X;
                double dy = a.Y - b.Y;
                return Sqrt(dx * dx + dy * dy);
            }, 0.0);

        internal static (double cos, double sin) GetTrigsFromAngle(double angle) =>
            Log("PhysicsComponent", "getting trigs from angle", () => (Cos(angle), Sin(angle)), (1.0, 0.0));

    }
}