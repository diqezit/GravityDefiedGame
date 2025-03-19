using System;
using System.Windows;
using System.Collections.Generic;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Models
{
    /// <summary>
    /// Абстрактный класс, расширяющий Component, предоставляя базовые
    /// методы для работы с физикой объектов: повороты, смещения, расчёт сил и энергии.
    /// </summary>
    public abstract class PhysicsComponent : Component
    {
        protected const double FullRotation = 2 * Math.PI;

        protected PhysicsComponent(string tag) : base(tag) { }

        #region Статические методы для геометрических преобразований

        internal static Point RotatePoint(Point origin, Point point, double angle)
        {
            double cosAngle = Math.Cos(angle);
            double sinAngle = Math.Sin(angle);
            double dx = point.X - origin.X;
            double dy = point.Y - origin.Y;

            return new Point(
                origin.X + dx * cosAngle - dy * sinAngle,
                origin.Y + dx * sinAngle + dy * cosAngle
            );
        }

        internal static Point Offset(Point point, double dx, double dy, double angle)
        {
            double cosAngle = Math.Cos(angle);
            double sinAngle = Math.Sin(angle);
            return new Point(
                point.X + dx * cosAngle - dy * sinAngle,
                point.Y + dx * sinAngle + dy * cosAngle
            );
        }

        internal static double CalculateDistance(Point pointA, Point pointB)
        {
            double dx = pointA.X - pointB.X;
            double dy = pointA.Y - pointB.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        internal static (double cosAngle, double sinAngle) GetTrigsFromAngle(double angle) =>
            (Math.Cos(angle), Math.Sin(angle));

        #endregion

        #region Методы обновления и вычисления позиций

        protected Point CalculatePointWithOffset(Point center, double angle, double distance) =>
            Offset(center, distance, 0, angle);

        protected Point UpdatePosition(Point currentPosition, Vector velocity, double deltaTime) =>
            new Point(currentPosition.X + velocity.X * deltaTime, currentPosition.Y + velocity.Y * deltaTime);

        protected Vector NormalizeVector(Vector v)
        {
            double length = v.Length;
            return length > 1e-10 ? v * (1.0 / length) : new Vector(0, 0);
        }

        #endregion

        #region Методы для векторных операций

        protected double CrossProduct(Vector a, Vector b) => a.X * b.Y - a.Y * b.X;
        protected double DotProduct(Vector a, Vector b) => a.X * b.X + a.Y * b.Y;
        protected Vector GetDirectionVector(double angle) => new Vector(Math.Cos(angle), Math.Sin(angle));
        protected double GetAngleFromVector(Vector v) => Math.Atan2(v.Y, v.X);
        protected Vector CalculateNormalForce(Vector normal, double magnitude) => normal * magnitude;

        protected Vector CalculateFrictionForce(Vector tangent, double normalForceMagnitude, double frictionCoefficient) =>
            tangent * (-normalForceMagnitude * frictionCoefficient);

        #endregion

        #region Методы расчёта импульсов и энергий

        protected (Vector LinearImpulse, double AngularImpulse) CalculateImpulse(
            Vector point, Vector force, Point centerOfMass, double deltaTime)
        {
            Vector relativePosition = new Vector(point.X - centerOfMass.X, point.Y - centerOfMass.Y);
            Vector linearImpulse = force * deltaTime;
            double angularImpulse = CrossProduct(relativePosition, force) * deltaTime;
            return (linearImpulse, angularImpulse);
        }

        protected double CalculateMomentOfInertia(double mass, double radius) =>
            mass * radius * radius;

        protected double CalculateTorque(Point applicationPoint, Point pivot, Vector force)
        {
            Vector r = new Vector(applicationPoint.X - pivot.X, applicationPoint.Y - pivot.Y);
            return CrossProduct(r, force);
        }

        protected double CalculateKineticEnergy(double mass, Vector velocity) =>
            0.5 * mass * velocity.LengthSquared;

        protected double CalculateRotationalEnergy(double momentOfInertia, double angularVelocity) =>
            0.5 * momentOfInertia * angularVelocity * angularVelocity;

        protected double CalculatePotentialEnergy(double mass, double height, double gravity) =>
            mass * gravity * height;

        #endregion

        #region Прочие методы

        protected Vector ProjectPointOntoLine(Point point, Point lineStart, Point lineEnd)
        {
            Vector line = new Vector(lineEnd.X - lineStart.X, lineEnd.Y - lineStart.Y);
            Vector pointVector = new Vector(point.X - lineStart.X, point.Y - lineStart.Y);
            double dotProduct = DotProduct(pointVector, line);
            double lineLength = line.Length;

            if (lineLength < 1e-10)
                return new Vector(lineStart.X, lineStart.Y);

            double t = ClampValue(dotProduct / (lineLength * lineLength), 0, 1);
            return new Vector(lineStart.X + t * line.X, lineStart.Y + t * line.Y);
        }

        protected double CalculatePenetrationDepth(Point point, Vector normal, double distance) =>
            DotProduct(new Vector(point.X, point.Y), normal) - distance;

        protected bool IsVectorExceedingSafeValue(Vector vector, double threshold, string message)
        {
            if (vector.Length > threshold)
            {
                TryLog(LogLevel.W, message);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Добавляет линии, замыкающие многоугольник, соединяя последовательные точки.
        /// </summary>
        /// <typeparam name="T">Тип линии.</typeparam>
        /// <typeparam name="U">Тип, характеризующий тип линии.</typeparam>
        /// <param name="lines">Список, куда добавляются линии.</param>
        /// <param name="startIndex">Индекс начала в списке точек.</param>
        /// <param name="pointCount">Количество точек, образующих многоугольник.</param>
        /// <param name="lineType">Тип линии.</param>
        /// <param name="createLineFunc">Функция для создания линии по индексам и типу.</param>
        internal void AddClosedPolygonLines<T, U>(List<T> lines, int startIndex, int pointCount, U lineType,
            Func<int, int, U, T> createLineFunc)
        {
            for (int i = 0; i < pointCount - 1; i++)
                lines.Add(createLineFunc(startIndex + i, startIndex + i + 1, lineType));

            lines.Add(createLineFunc(startIndex + pointCount - 1, startIndex, lineType));
        }

        #endregion
    }
}