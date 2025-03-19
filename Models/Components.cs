using System;
using System.Collections.Generic;
using System.Text;
using System.Windows;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Models
{
    /// <summary>
    /// Абстрактный базовый класс для компонентов игры.
    /// Содержит методы логирования, проверки и корректировки физических параметров,
    /// а также генерацию диагностических отчётов.
    /// </summary>
    public abstract class Component
    {
        protected readonly string _logTag;
        protected internal double _logThrottleTime = 0;  // время блокировки логирования
        protected internal List<DiagnosticInfo> _diagnostics = new List<DiagnosticInfo>();

        public Component(string logTag)
        {
            _logTag = logTag;
        }

        #region Логирование и вспомогательные методы

        protected void UpdateLogTimer(double deltaTime)
        {
            if (_logThrottleTime > 0)
                _logThrottleTime -= deltaTime;
        }

        protected void TryLog(LogLevel level, string message)
        {
            if (_logThrottleTime > 0)
                return;

            LogMessage(level, message);
            _logThrottleTime = GameConstants.Debug.LogThrottle;
        }

        private void LogMessage(LogLevel level, string message)
        {
            switch (level)
            {
                case LogLevel.D: Logger.Debug(_logTag, message); break;
                case LogLevel.I: Logger.Info(_logTag, message); break;
                case LogLevel.W: Logger.Warning(_logTag, message); break;
                case LogLevel.E: Logger.Error(_logTag, message); break;
                default: Logger.Info(_logTag, message); break;
            }
        }

        protected double ClampValue(double value, double min, double max) =>
            Math.Max(min, Math.Min(max, value));

        protected double SafeDivide(double numerator, double denominator, double defaultValue = 0)
        {
            if (Math.Abs(denominator) < 1e-10)
                return defaultValue;
            return numerator / denominator;
        }

        protected double Lerp(double a, double b, double t) =>
            a + (b - a) * ClampValue(t, 0, 1);

        protected double Smoothstep(double edge0, double edge1, double x)
        {
            double t = ClampValue((x - edge0) / (edge1 - edge0), 0, 1);
            return t * t * (3 - 2 * t);
        }

        protected double SmoothLerp(double a, double b, double t, double smoothness) =>
            Lerp(a, b, Smoothstep(0, 1, t));

        protected Vector LerpVector(Vector a, Vector b, double t) =>
            new Vector(Lerp(a.X, b.X, t), Lerp(a.Y, b.Y, t));

        protected double NormalizeAngle(double angle)
        {
            const double twoPi = 2 * Math.PI;
            if (angle >= -Math.PI && angle <= Math.PI)
                return angle;

            angle %= twoPi;
            if (angle < 0)
                angle += twoPi;
            if (angle > Math.PI)
                angle -= twoPi;
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
            if (double.IsNaN(value.X) || double.IsNaN(value.Y) ||
                double.IsInfinity(value.X) || double.IsInfinity(value.Y))
            {
                TryLog(LogLevel.E, $"{errorMessage}: {value}");
                return defaultValue;
            }
            return value;
        }

        protected internal Point SanitizePosition(Point value, Point defaultValue, string errorMessage)
        {
            if (double.IsNaN(value.X) || double.IsNaN(value.Y) ||
                double.IsInfinity(value.X) || double.IsInfinity(value.Y))
            {
                TryLog(LogLevel.E, $"{errorMessage}: {value}");
                return defaultValue;
            }
            return value;
        }

        protected bool IsPhysicallyValid(double value, double minValid, double maxValid) =>
            !double.IsNaN(value) && !double.IsInfinity(value) &&
            value >= minValid && value <= maxValid;

        protected bool IsPhysicallyInRange(Vector velocity, double maxMagnitude) =>
            !double.IsNaN(velocity.X) && !double.IsNaN(velocity.Y) &&
            !double.IsInfinity(velocity.X) && !double.IsInfinity(velocity.Y) &&
            velocity.Length <= maxMagnitude;

        #endregion

        #region Валидация параметров и диагностика

        protected internal bool IsExceedingSafeValue<T>(T value, T threshold, string message) where T : IComparable<T>
        {
            if (value.CompareTo(threshold) > 0)
            {
                TryLog(LogLevel.W, message);
                return true;
            }
            return false;
        }

        protected bool CheckConditionWithLog(bool condition, LogLevel level, string message)
        {
            if (condition)
                TryLog(level, message);
            return condition;
        }

        protected internal bool ValidateComponents(IEnumerable<(string Name, double Value, double Min, double Max)> parameters,
                                                   string componentName)
        {
            bool hasValidationErrors = false;
            foreach (var param in parameters)
            {
                ValidatePhysicalParameter(param.Name, param.Value, param.Min, param.Max, componentName);
                if (double.IsNaN(param.Value) || double.IsInfinity(param.Value) ||
                    param.Value < param.Min || param.Value > param.Max)
                {
                    hasValidationErrors = true;
                }
            }
            return hasValidationErrors;
        }

        protected internal void ValidatePhysicalParameter(
            string parameterName,
            double value,
            double minSafe,
            double maxSafe,
            string componentName = "Unknown")
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
            {
                TryLog(LogLevel.E, $"CRITICAL: {componentName} - {parameterName} has invalid value: {value}");
                AddDiagnostic(parameterName, value, 0, "NaN or Infinity detected", componentName);
                return;
            }

            if (value < minSafe || value > maxSafe)
            {
                LogLevel level = (value < minSafe * 2 || value > maxSafe * 0.5) ? LogLevel.W : LogLevel.E;
                TryLog(level, $"{componentName} - {parameterName} outside safe range: {value:F2} " +
                              $"(safe range: {minSafe:F2} to {maxSafe:F2})");

                AddDiagnostic(parameterName, value, value > maxSafe ? maxSafe : minSafe, "Value outside safe range", componentName);
            }
        }

        protected internal void ValidateVectorParameter(
            string parameterName,
            Vector vector,
            double maxMagnitude,
            string componentName = "Unknown")
        {
            if (double.IsNaN(vector.X) || double.IsNaN(vector.Y) ||
                double.IsInfinity(vector.X) || double.IsInfinity(vector.Y))
            {
                TryLog(LogLevel.E, $"CRITICAL: {componentName} - {parameterName} has invalid values: {vector}");
                AddDiagnostic(parameterName, 0, 0, $"Vector contains NaN or Infinity: ({vector.X}, {vector.Y})", componentName);
                return;
            }

            double magnitude = vector.Length;
            if (magnitude > maxMagnitude)
            {
                LogLevel level = magnitude > maxMagnitude * 1.5 ? LogLevel.E : LogLevel.W;
                TryLog(level, $"{componentName} - {parameterName} magnitude too high: {magnitude:F2} " +
                              $"(max safe: {maxMagnitude:F2})");

                AddDiagnostic(parameterName, magnitude, maxMagnitude, "Vector magnitude exceeds safe limit", componentName);
            }
        }

        protected internal void ValidateConnectionStrain(
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
            }
            else if (ratio > maxExtensionRatio)
            {
                ProcessConnectionStrain(connectionName, ratio, maxExtensionRatio, false, componentName);
            }
        }

        private void ProcessConnectionStrain(string connectionName, double ratio, double safeRatio, bool isCompression, string componentName)
        {
            double severity = isCompression ? safeRatio / Math.Max(ratio, 0.001) : ratio / safeRatio;
            LogLevel level = severity > 1.5 ? LogLevel.E : LogLevel.W;
            string strainType = isCompression ? "compressed" : "stretched";

            TryLog(level, $"{componentName} - {connectionName} excessively {strainType}: {ratio:P2} " +
                           $"(safe {(isCompression ? "min" : "max")}: {safeRatio:P2})");

            AddDiagnostic(connectionName + (isCompression ? "_compression" : "_extension"), ratio, safeRatio,
                $"Connection excessively {strainType}", componentName);
        }

        protected internal string GenerateDiagnosticReport()
        {
            if (_diagnostics.Count == 0)
                return "No diagnostic issues detected.";

            var report = new StringBuilder();
            report.AppendLine($"Diagnostic report - {_diagnostics.Count} issues detected:");
            foreach (var issue in _diagnostics)
                report.AppendLine($"- {issue}");
            _diagnostics.Clear();
            return report.ToString();
        }

        private void AddDiagnostic(string parameter, double value, double threshold, string description, string component)
        {
            _diagnostics.Add(new DiagnosticInfo
            {
                Component = component,
                Parameter = parameter,
                Value = value,
                Threshold = threshold,
                Description = description
            });
        }

        #endregion

        #region Дополнительные методы

        protected internal struct DiagnosticInfo
        {
            public string Component;
            public string Parameter;
            public double Value;
            public double Threshold;
            public string Description;

            public override string ToString() =>
                $"{Component}:{Parameter}={Value:F2} (threshold: {Threshold:F2}) - {Description}";
        }

        #endregion
    }
}