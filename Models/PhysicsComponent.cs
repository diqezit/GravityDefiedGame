using System;
using System.Collections.Generic;
using System.Windows;
using GravityDefiedGame.Utilities;
using static System.Math;
using static GravityDefiedGame.Utilities.LoggerCore;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame.Models
{
    public abstract class PhysicsComponent
    {
        protected const double FullRotation = 2 * PI;

        public PhysicsComponent() { }

        // Inner class for caching trigonometric values
        protected class TrigCache
        {
            public double Sin { get; private set; }
            public double Cos { get; private set; }

            public void Update(double angle)
            {
                Log("TrigCache", "updating trig values", () =>
                {
                    Sin = Math.Sin(angle);
                    Cos = Math.Cos(angle);
                });
            }
        }

        #region Utility Methods

        protected static double ClampValue(double value, double min, double max) =>
            Log("PhysicsComponent", "clamping value", () => Max(min, Min(max, value)), min);

        protected static double SafeDivide(double numerator, double denominator, double defaultValue = 0) =>
            Log("PhysicsComponent", "safe division", () =>
                Abs(denominator) < 1e-10 ? defaultValue : numerator / denominator, defaultValue);

        protected static double Lerp(double a, double b, double t) =>
            Log("PhysicsComponent", "linear interpolation", () => a + (b - a) * ClampValue(t, 0, 1), a);

        protected static Vector LerpVector(Vector a, Vector b, double t)
        {
            return Log("PhysicsComponent", "vector interpolation", () =>
            {
                double factor = ClampValue(t, 0, 1);
                return new Vector(
                    a.X + (b.X - a.X) * factor,
                    a.Y + (b.Y - a.Y) * factor
                );
            }, a);
        }

        protected static double Smoothstep(double edge0, double edge1, double x)
        {
            return Log("PhysicsComponent", "smoothstep", () =>
            {
                double t = ClampValue((x - edge0) / (edge1 - edge0), 0, 1);
                return t * t * (3 - 2 * t);
            }, 0.0);
        }

        protected static double NormalizeAngle(double angle)
        {
            return Log("PhysicsComponent", "normalizing angle", () =>
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
        }

        #endregion

        #region Sanitization Methods

        protected internal static T SanitizeValue<T>(T value, T defaultValue, string errorMessage) where T : IConvertible =>
            Log("PhysicsComponent", "sanitizing value", () =>
                double.IsNaN(Convert.ToDouble(value)) || double.IsInfinity(Convert.ToDouble(value))
                    ? LogErrorAndReturnDefault(errorMessage, value, defaultValue)
                    : value, defaultValue);

        private static T LogErrorAndReturnDefault<T>(string errorMessage, T value, T defaultValue)
        {
            Error("PhysicsComponent", $"{errorMessage}: {value}");
            return defaultValue;
        }

        protected internal static Vector SanitizeVector(Vector value, Vector defaultValue, string errorMessage) =>
            Log("PhysicsComponent", "sanitizing vector", () =>
                IsVectorInvalid(value)
                    ? LogErrorAndReturnDefault(errorMessage, value, defaultValue)
                    : value, defaultValue);

        protected internal static Point SanitizePosition(Point value, Point defaultValue, string errorMessage) =>
            Log("PhysicsComponent", "sanitizing position", () =>
                IsPointInvalid(value)
                    ? LogErrorAndReturnDefault(errorMessage, value, defaultValue)
                    : value, defaultValue);

        private static bool IsVectorInvalid(Vector value) =>
            Log("PhysicsComponent", "checking vector validity", () =>
                double.IsNaN(value.X) || double.IsNaN(value.Y) ||
                double.IsInfinity(value.X) || double.IsInfinity(value.Y), false);

        private static bool IsPointInvalid(Point value) =>
            Log("PhysicsComponent", "checking point validity", () =>
                double.IsNaN(value.X) || double.IsNaN(value.Y) ||
                double.IsInfinity(value.X) || double.IsInfinity(value.Y), false);

        #endregion

        #region Parameter Validation

        protected internal static bool IsExceedingSafeValue<T>(T value, T threshold, string message) where T : IComparable<T> =>
            Log("PhysicsComponent", "checking safe value", () =>
            {
                if (value.CompareTo(threshold) > 0)
                {
                    Warning("PhysicsComponent", message);
                    return true;
                }
                return false;
            }, false);

        protected static bool IsVectorExceedingSafeValue(Vector vector, double threshold, string message) =>
            CheckConditionWithLog(vector.Length > threshold, LogLevel.W, message);

        protected static bool CheckConditionWithLog(bool condition, LogLevel level, string message) =>
            Log("PhysicsComponent", "checking condition", () =>
            {
                if (condition)
                {
                    WriteLog(level, "PhysicsComponent", message);
                }
                return condition;
            }, false);

        protected internal static bool ValidatePhysicalParameter(
            string parameterName,
            double value,
            double minSafe,
            double maxSafe,
            string componentName = "Unknown") =>
            Log("PhysicsComponent", $"validating parameter: {parameterName}", () =>
            {
                if (double.IsNaN(value) || double.IsInfinity(value))
                {
                    Error(componentName, $"CRITICAL: {parameterName} has invalid value: {value}");
                    return false;
                }

                if (value < minSafe || value > maxSafe)
                {
                    var logLevel = (value < minSafe * 2 || value > maxSafe * 0.5) ? LogLevel.W : LogLevel.E;
                    WriteLog(logLevel, componentName, $"{parameterName} outside safe range: {value:F2} (safe range: {minSafe:F2} to {maxSafe:F2})");
                    return false;
                }

                return true;
            }, false);

        protected internal static bool ValidateVectorParameter(
            string parameterName,
            Vector vector,
            double maxMagnitude,
            string componentName = "Unknown") =>
            Log("PhysicsComponent", $"validating vector parameter: {parameterName}", () =>
            {
                if (IsVectorInvalid(vector))
                {
                    Error(componentName, $"CRITICAL: {parameterName} has invalid values: {vector}");
                    return false;
                }

                double magnitude = vector.Length;
                if (magnitude > maxMagnitude)
                {
                    var logLevel = magnitude > maxMagnitude * 1.5 ? LogLevel.E : LogLevel.W;
                    WriteLog(logLevel, componentName, $"{parameterName} magnitude too high: {magnitude:F2} (max safe: {maxMagnitude:F2})");
                    return false;
                }

                return true;
            }, false);

        protected internal static bool ValidateConnectionStrain(
            string connectionName,
            double currentLength,
            double restLength,
            double maxCompressionRatio,
            double maxExtensionRatio,
            string componentName = "Unknown") =>
            Log("PhysicsComponent", $"validating connection strain: {connectionName}", () =>
            {
                double ratio = currentLength / restLength;

                if (ratio < maxCompressionRatio)
                {
                    ProcessConnectionStrain(connectionName, ratio, maxCompressionRatio, true, componentName);
                    return false;
                }

                if (ratio > maxExtensionRatio)
                {
                    ProcessConnectionStrain(connectionName, ratio, maxExtensionRatio, false, componentName);
                    return false;
                }

                return true;
            }, false);

        private static void ProcessConnectionStrain(
            string connectionName,
            double ratio,
            double safeRatio,
            bool isCompression,
            string componentName) =>
            Log("PhysicsComponent", "processing connection strain", () =>
            {
                double severity = isCompression ? safeRatio / Max(ratio, 0.001) : ratio / safeRatio;
                var logLevel = severity > 1.5 ? LogLevel.E : LogLevel.W;
                var strainType = isCompression ? "compressed" : "stretched";

                WriteLog(logLevel, componentName, $"{connectionName} excessively {strainType}: {ratio:P2} (safe {(isCompression ? "min" : "max")}: {safeRatio:P2})");
            });

        #endregion

        #region Geometric Transformation Methods

        internal static Point Offset(Point point, double dx, double dy, double angle) =>
            Log("PhysicsComponent", "offsetting point", () =>
            {
                var (cosAngle, sinAngle) = GetTrigsFromAngle(angle);
                return new Point(
                    point.X + dx * cosAngle - dy * sinAngle,
                    point.Y + dx * sinAngle + dy * cosAngle
                );
            }, point);

        internal static double CalculateDistance(Point pointA, Point pointB) =>
            Log("PhysicsComponent", "calculating distance", () =>
            {
                double dx = pointA.X - pointB.X;
                double dy = pointA.Y - pointB.Y;
                return Sqrt(dx * dx + dy * dy);
            }, 0.0);

        internal static (double cosAngle, double sinAngle) GetTrigsFromAngle(double angle) =>
            Log("PhysicsComponent", "getting trigs from angle", () =>
                (Cos(angle), Sin(angle)), (1.0, 0.0));

        #endregion

        #region Position Update Methods

        protected static Point UpdatePosition(Point currentPosition, Vector velocity, double deltaTime) =>
            Log("PhysicsComponent", "updating position", () =>
            {
                double newX = currentPosition.X + velocity.X * deltaTime;
                double newY = currentPosition.Y + velocity.Y * deltaTime;
                Point newPosition = new Point(newX, newY);
                return SanitizePosition(newPosition, currentPosition, "Invalid position update detected");
            }, currentPosition);

        #endregion

        #region Miscellaneous Methods

        internal static void AddClosedPolygonLines<T, U>(List<T> lines, int startIndex, int pointCount, U lineType, Func<int, int, U, T> createLineFunc) =>
            Log("PhysicsComponent", "adding closed polygon lines", () =>
            {
                for (int i = 0; i < pointCount - 1; i++)
                {
                    lines.Add(createLineFunc(startIndex + i, startIndex + i + 1, lineType));
                }
                lines.Add(createLineFunc(startIndex + pointCount - 1, startIndex, lineType));
            });

        #endregion

        #region Logging Integration

        protected static void TryLog(LogLevel level, string message) =>
            WriteLog(level, "PhysicsComponent", message);

        #endregion
    }
}