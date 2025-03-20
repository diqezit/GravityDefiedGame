// PhysicsComponent.cs
using System;
using System.Collections.Generic;
using System.Windows;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Models
{
    /// <summary>
    /// Класс компонента игры, предоставляющий базовые методы для работы с физикой объектов,
    /// логирования, проверки параметров и диагностики.
    /// </summary>
    public abstract class PhysicsComponent
    {
        protected const double FullRotation = 2 * Math.PI;
        protected readonly string _logTag;
        protected internal double _logThrottleTime; // время блокировки логирования

        public PhysicsComponent(string logTag) => _logTag = logTag;

        #region Логирование и вспомогательные методы

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

        protected double ClampValue(double value, double min, double max) => Math.Max(min, Math.Min(max, value));

        protected double SafeDivide(double numerator, double denominator, double defaultValue = 0) =>
            Math.Abs(denominator) < 1e-10 ? defaultValue : numerator / denominator;

        protected double Lerp(double a, double b, double t) => a + (b - a) * ClampValue(t, 0, 1);

        protected double Smoothstep(double edge0, double edge1, double x)
        {
            double t = ClampValue((x - edge0) / (edge1 - edge0), 0, 1);
            return t * t * (3 - 2 * t);
        }

        protected double NormalizeAngle(double angle)
        {
            const double twoPi = 2 * Math.PI;
            if (angle >= -Math.PI && angle <= Math.PI)
            {
                return angle;
            }

            angle %= twoPi;
            if (angle < 0)
            {
                angle += twoPi;
            }
            if (angle > Math.PI)
            {
                angle -= twoPi;
            }
            return angle;
        }

        #endregion

        #region Методы санитации

        protected internal T SanitizeValue<T>(T value, T defaultValue, string errorMessage) where T : IConvertible
        {
            if (double.IsNaN(Convert.ToDouble(value)) || double.IsInfinity(Convert.ToDouble(value)))
            {
                TryLog(LogLevel.E, $"{errorMessage}: {value}");
                return defaultValue;
            }
            return value;
        }

        protected internal Vector SanitizeVector(Vector value, Vector defaultValue, string errorMessage)
        {
            if (double.IsNaN(value.X) || double.IsNaN(value.Y) || double.IsInfinity(value.X) || double.IsInfinity(value.Y))
            {
                TryLog(LogLevel.E, $"{errorMessage}: {value}");
                return defaultValue;
            }
            return value;
        }

        protected internal Point SanitizePosition(Point value, Point defaultValue, string errorMessage)
        {
            if (double.IsNaN(value.X) || double.IsNaN(value.Y) || double.IsInfinity(value.X) || double.IsInfinity(value.Y))
            {
                TryLog(LogLevel.E, $"{errorMessage}: {value}");
                return defaultValue;
            }
            return value;
        }

        #endregion

        #region Валидация параметров

        protected internal bool IsExceedingSafeValue<T>(T value, T threshold, string message) where T : IComparable<T>
        {
            if (value.CompareTo(threshold) > 0)
            {
                TryLog(LogLevel.W, message);
                return true;
            }
            return false;
        }

        protected bool IsVectorExceedingSafeValue(Vector vector, double threshold, string message)
        {
            if (vector.Length > threshold)
            {
                TryLog(LogLevel.W, message);
                return true;
            }
            return false;
        }

        protected bool CheckConditionWithLog(bool condition, LogLevel level, string message)
        {
            if (condition)
            {
                TryLog(level, message);
                return true;
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
                LogLevel level = (value < minSafe * 2 || value > maxSafe * 0.5) ? LogLevel.W : LogLevel.E;
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
            if (double.IsNaN(vector.X) || double.IsNaN(vector.Y) || double.IsInfinity(vector.X) || double.IsInfinity(vector.Y))
            {
                TryLog(LogLevel.E, $"CRITICAL: {componentName} - {parameterName} has invalid values: {vector}");
                return false;
            }

            double magnitude = vector.Length;
            if (magnitude > maxMagnitude)
            {
                LogLevel level = magnitude > maxMagnitude * 1.5 ? LogLevel.E : LogLevel.W;
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
            else if (ratio > maxExtensionRatio)
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
            double severity = isCompression ? safeRatio / Math.Max(ratio, 0.001) : ratio / safeRatio;
            LogLevel level = severity > 1.5 ? LogLevel.E : LogLevel.W;
            string strainType = isCompression ? "compressed" : "stretched";

            TryLog(level, $"{componentName} - {connectionName} excessively {strainType}: {ratio:P2} (safe {(isCompression ? "min" : "max")}: {safeRatio:P2})");
        }

        protected internal bool ValidateComponents(IEnumerable<(string Name, double Value, double Min, double Max)> parameters, string componentName)
        {
            bool hasValidationErrors = false;
            foreach (var (Name, Value, Min, Max) in parameters)
            {
                if (!ValidatePhysicalParameter(Name, Value, Min, Max, componentName))
                {
                    hasValidationErrors = true;
                }
            }
            return !hasValidationErrors;
        }

        #endregion

        #region Статические методы для геометрических преобразований

        internal static Point Offset(Point point, double dx, double dy, double angle)
        {
            double cosAngle = Math.Cos(angle);
            double sinAngle = Math.Sin(angle);
            return new Point(point.X + dx * cosAngle - dy * sinAngle, point.Y + dx * sinAngle + dy * cosAngle);
        }

        internal static double CalculateDistance(Point pointA, Point pointB)
        {
            double dx = pointA.X - pointB.X;
            double dy = pointA.Y - pointB.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        internal static (double cosAngle, double sinAngle) GetTrigsFromAngle(double angle) => (Math.Cos(angle), Math.Sin(angle));

        #endregion

        #region Методы обновления и вычисления позиций

        protected Point UpdatePosition(Point currentPosition, Vector velocity, double deltaTime) =>
            new Point(currentPosition.X + velocity.X * deltaTime, currentPosition.Y + velocity.Y * deltaTime);

        #endregion

        #region Прочие методы

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