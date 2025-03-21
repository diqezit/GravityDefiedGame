using System;
using System.Collections.Generic;
using System.Windows;
using GravityDefiedGame.Utilities;
using static System.Math;

namespace GravityDefiedGame.Models
{
    public abstract class PhysicsComponent
    {
        protected const double FullRotation = 2 * PI;
        protected readonly string _logTag;
        protected internal double _logThrottleTime;

        public PhysicsComponent(string logTag) => _logTag = logTag;

        // кэшированиt тригонометрических значений
        protected class TrigCache
        {
            public double Sin { get; private set; }
            public double Cos { get; private set; }

            public void Update(double angle)
            {
                Sin = Math.Sin(angle);
                Cos = Math.Cos(angle);
            }
        }

        #region Logging and Utility Methods

        protected void UpdateLogTimer(double deltaTime)
        {
            if (_logThrottleTime > 0)
            {
                _logThrottleTime -= deltaTime;
            }
        }

        protected void TryLog(LogLevel level, string message)
        {
            if (_logThrottleTime <= 0)
            {
                LogMessage(level, message);
                _logThrottleTime = GameConstants.Debug.LogThrottle;
            }
        }

        private void LogMessage(LogLevel level, string message)
        {
            switch (level)
            {
                case LogLevel.D:
                    Logger.Debug(_logTag, message);
                    break;
                case LogLevel.I:
                    Logger.Info(_logTag, message);
                    break;
                case LogLevel.W:
                    Logger.Warning(_logTag, message);
                    break;
                case LogLevel.E:
                    Logger.Error(_logTag, message);
                    break;
                default:
                    Logger.Info(_logTag, message);
                    break;
            }
        }

        protected double ClampValue(double value, double min, double max) => Max(min, Min(max, value));

        protected double SafeDivide(double numerator, double denominator, double defaultValue = 0) =>
            Abs(denominator) < 1e-10 ? defaultValue : numerator / denominator;

        protected double Lerp(double a, double b, double t) => a + (b - a) * ClampValue(t, 0, 1);

        protected double Smoothstep(double edge0, double edge1, double x)
        {
            double t = ClampValue((x - edge0) / (edge1 - edge0), 0, 1);
            return t * t * (3 - 2 * t);
        }

        protected static double NormalizeAngle(double angle)
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
        }

        #endregion

        #region Sanitization Methods

        protected internal T SanitizeValue<T>(T value, T defaultValue, string errorMessage) where T : IConvertible =>
            double.IsNaN(Convert.ToDouble(value)) || double.IsInfinity(Convert.ToDouble(value))
                ? LogErrorAndReturnDefault(errorMessage, value, defaultValue)
                : value;

        private T LogErrorAndReturnDefault<T>(string errorMessage, T value, T defaultValue)
        {
            TryLog(LogLevel.E, $"{errorMessage}: {value}");
            return defaultValue;
        }

        protected internal Vector SanitizeVector(Vector value, Vector defaultValue, string errorMessage) =>
            IsVectorInvalid(value)
                ? LogErrorAndReturnDefault(errorMessage, value, defaultValue)
                : value;

        protected internal Point SanitizePosition(Point value, Point defaultValue, string errorMessage) =>
            IsPointInvalid(value)
                ? LogErrorAndReturnDefault(errorMessage, value, defaultValue)
                : value;

        private bool IsVectorInvalid(Vector value) =>
            double.IsNaN(value.X) || double.IsNaN(value.Y) ||
            double.IsInfinity(value.X) || double.IsInfinity(value.Y);

        private bool IsPointInvalid(Point value) =>
            double.IsNaN(value.X) || double.IsNaN(value.Y) ||
            double.IsInfinity(value.X) || double.IsInfinity(value.Y);

        #endregion

        #region Parameter Validation

        protected internal bool IsExceedingSafeValue<T>(T value, T threshold, string message) where T : IComparable<T>
        {
            if (value.CompareTo(threshold) > 0)
            {
                TryLog(LogLevel.W, message);
                return true;
            }
            return false;
        }

        protected bool IsVectorExceedingSafeValue(Vector vector, double threshold, string message) =>
            CheckConditionWithLog(vector.Length > threshold, LogLevel.W, message);

        protected bool CheckConditionWithLog(bool condition, LogLevel level, string message)
        {
            if (condition)
            {
                TryLog(level, message);
            }
            return condition;
        }

        protected internal bool ValidatePhysicalParameter(
            string parameterName,
            double value,
            double minSafe,
            double maxSafe,
            string componentName = "Unknown")
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
            {
                TryLog(LogLevel.E, $"CRITICAL: {componentName} - {parameterName} has invalid value: {value}");
                return false;
            }

            if (value < minSafe || value > maxSafe)
            {
                var level = (value < minSafe * 2 || value > maxSafe * 0.5) ? LogLevel.W : LogLevel.E;
                TryLog(level, $"{componentName} - {parameterName} outside safe range: {value:F2} (safe range: {minSafe:F2} to {maxSafe:F2})");
                return false;
            }

            return true;
        }

        protected internal bool ValidateVectorParameter(
            string parameterName,
            Vector vector,
            double maxMagnitude,
            string componentName = "Unknown")
        {
            if (IsVectorInvalid(vector))
            {
                TryLog(LogLevel.E, $"CRITICAL: {componentName} - {parameterName} has invalid values: {vector}");
                return false;
            }

            double magnitude = vector.Length;
            if (magnitude > maxMagnitude)
            {
                var level = magnitude > maxMagnitude * 1.5 ? LogLevel.E : LogLevel.W;
                TryLog(level, $"{componentName} - {parameterName} magnitude too high: {magnitude:F2} (max safe: {maxMagnitude:F2})");
                return false;
            }

            return true;
        }

        protected internal bool ValidateConnectionStrain(
            string connectionName,
            double currentLength,
            double restLength,
            double maxCompressionRatio,
            double maxExtensionRatio,
            string componentName = "Unknown")
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
        }

        private void ProcessConnectionStrain(
            string connectionName,
            double ratio,
            double safeRatio,
            bool isCompression,
            string componentName)
        {
            double severity = isCompression ? safeRatio / Max(ratio, 0.001) : ratio / safeRatio;
            var level = severity > 1.5 ? LogLevel.E : LogLevel.W;
            var strainType = isCompression ? "compressed" : "stretched";

            TryLog(level, $"{componentName} - {connectionName} excessively {strainType}: {ratio:P2} (safe {(isCompression ? "min" : "max")}: {safeRatio:P2})");
        }

        #endregion

        #region Geometric Transformation Methods

        internal static Point Offset(Point point, double dx, double dy, double angle)
        {
            var (cosAngle, sinAngle) = GetTrigsFromAngle(angle);
            return new Point(
                point.X + dx * cosAngle - dy * sinAngle,
                point.Y + dx * sinAngle + dy * cosAngle
            );
        }

        internal static double CalculateDistance(Point pointA, Point pointB)
        {
            double dx = pointA.X - pointB.X;
            double dy = pointA.Y - pointB.Y;
            return Sqrt(dx * dx + dy * dy);
        }

        internal static (double cosAngle, double sinAngle) GetTrigsFromAngle(double angle) =>
            (Cos(angle), Sin(angle));

        #endregion

        #region Position Update Methods

        protected Point UpdatePosition(Point currentPosition, Vector velocity, double deltaTime) =>
            new(
                currentPosition.X + velocity.X * deltaTime,
                currentPosition.Y + velocity.Y * deltaTime
            );

        #endregion

        #region Miscellaneous Methods

        internal void AddClosedPolygonLines<T, U>(List<T> lines, int startIndex, int pointCount, U lineType, Func<int, int, U, T> createLineFunc)
        {
            for (int i = 0; i < pointCount - 1; i++)
            {
                lines.Add(createLineFunc(startIndex + i, startIndex + i + 1, lineType));
            }
            lines.Add(createLineFunc(startIndex + pointCount - 1, startIndex, lineType));
        }

        #endregion
    }
}