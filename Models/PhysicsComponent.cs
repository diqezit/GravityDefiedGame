using System;
using System.Windows;
using System.Threading;
using System.Collections.Generic;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Models
{
    public abstract class PhysicsComponent
    {
        protected double _timeSinceLastLog = 0;
        protected readonly string _logTag;

        protected const double FullRotation = 2 * Math.PI;

        protected delegate void PhysicsAction<T>(T param);

        protected PhysicsComponent(string tag)
        {
            _logTag = tag;
        }

        protected static double NormalizeAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        protected Vector SanitizeVector(Vector vector, Vector defaultValue, string errorMessage)
        {
            if (double.IsNaN(vector.X) || double.IsInfinity(vector.X) ||
                double.IsNaN(vector.Y) || double.IsInfinity(vector.Y))
            {
                Logger.Error(_logTag, $"{errorMessage}: {vector}");
                return defaultValue;
            }
            return vector;
        }

        protected double SanitizeValue(double value, double defaultValue, string errorMessage)
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
            {
                Logger.Error(_logTag, $"{errorMessage}: {value}");
                return defaultValue;
            }
            return value;
        }

        protected Point SanitizePosition(Point position, Point defaultValue, string errorMessage)
        {
            if (double.IsNaN(position.X) || double.IsInfinity(position.X) ||
                double.IsNaN(position.Y) || double.IsInfinity(position.Y))
            {
                Logger.Error(_logTag, $"{errorMessage}: {position}");
                return defaultValue;
            }
            return position;
        }

        protected void TryLog(LogLevel level, string message)
        {
            if (_timeSinceLastLog <= GameConstants.Debug.LogThrottle)
                return;

            switch (level)
            {
                case LogLevel.D: Logger.Debug(_logTag, message); break;
                case LogLevel.I: Logger.Info(_logTag, message); break;
                case LogLevel.W: Logger.Warning(_logTag, message); break;
                case LogLevel.E: Logger.Error(_logTag, message); break;
                case LogLevel.F: Logger.Fatal(_logTag, message); break;
            }

            _timeSinceLastLog = 0;
        }

        protected void UpdateLogTimer(double deltaTime)
        {
            _timeSinceLastLog += deltaTime;
        }

        protected bool IsExceedingSafeValue(double value, double threshold, string warningMessage)
        {
            if (Math.Abs(value) > threshold)
            {
                TryLog(LogLevel.W, warningMessage);
                return true;
            }
            return false;
        }

        protected bool IsExceedingSafeValue(Vector vector, double threshold, string warningMessage)
        {
            if (vector.Length > threshold)
            {
                TryLog(LogLevel.W, warningMessage);
                return true;
            }
            return false;
        }

        protected void HandleUpdateException(Exception ex, bool crashState)
        {
            Logger.Error(_logTag, $"Error updating physics: {ex.Message}");
            Logger.Exception(_logTag, ex);
        }

        protected bool ShouldSkipUpdate(bool crashState, CancellationToken cancellationToken)
        {
            if (crashState || cancellationToken.IsCancellationRequested)
            {
                TryLog(LogLevel.D, "Skipping update due to crash state or cancellation");
                return true;
            }
            return false;
        }

        protected Point CalculatePointWithOffset(Point center, double angle, double distance) =>
            new(center.X + Math.Cos(angle) * distance, center.Y + Math.Sin(angle) * distance);

        protected void AddClosedPolygonLines<T, U>(List<T> lines, int startIndex, int pointCount, U lineType,
            Func<int, int, U, T> createLineFunc)
        {
            for (int i = 0; i < pointCount - 1; i++)
                lines.Add(createLineFunc(startIndex + i, startIndex + i + 1, lineType));

            lines.Add(createLineFunc(startIndex + pointCount - 1, startIndex, lineType));
        }

        protected double ClampValue(double value, double min, double max) =>
            Math.Min(Math.Max(value, min), max);

        protected bool CheckConditionWithLog(bool condition, LogLevel level, string message)
        {
            if (condition)
                TryLog(level, message);

            return condition;
        }

        protected double SafeDivide(double numerator, double denominator, double defaultValue = 0)
        {
            if (Math.Abs(denominator) < 1e-10)
                return defaultValue;

            return numerator / denominator;
        }

        protected double Lerp(double start, double end, double t) =>
            start + (end - start) * Math.Clamp(t, 0, 1);

        protected double Distance(Point a, Point b) =>
            Math.Sqrt(Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2));

        protected double EnforceBounds(double value, double min, double max, string warningMessage = null)
        {
            if (value < min || value > max)
            {
                if (!string.IsNullOrEmpty(warningMessage))
                    TryLog(LogLevel.W, $"{warningMessage}: {value} → {Math.Clamp(value, min, max)}");

                return Math.Clamp(value, min, max);
            }
            return value;
        }
    }
}