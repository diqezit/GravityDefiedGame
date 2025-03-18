using System;
using System.Windows;
using System.Windows.Controls;

namespace GravityDefiedGame.Utilities
{
    public class Camera
    {
        private readonly Canvas _canvas;
        public Vector Offset { get; private set; }
        public double Zoom { get; private set; } = 1.0;
        private readonly double _followSpeed = 5.0;
        private Point _targetPosition;

        public Camera(Canvas canvas)
        {
            _canvas = canvas;
            Reset();
        }

        public void Reset() =>
            Offset = new(_canvas.ActualWidth / 2, _canvas.ActualHeight / 2);

        public void CenterOn(Point position)
        {
            Offset = new(
                _canvas.ActualWidth / 2 - position.X * Zoom,
                _canvas.ActualHeight / 2 - position.Y * Zoom);
            _targetPosition = position;
        }

        public void Update(Point targetPosition)
        {
            _targetPosition = targetPosition;
            Vector targetOffset = new(
                _canvas.ActualWidth / 2 - targetPosition.X * Zoom,
                _canvas.ActualHeight / 2 - targetPosition.Y * Zoom);

            Offset = Vector.Add(Offset, Vector.Multiply((targetOffset - Offset), _followSpeed * 0.016));
        }

        public Point WorldToScreen(Point worldPoint) =>
            new(worldPoint.X * Zoom + Offset.X, worldPoint.Y * Zoom + Offset.Y);

        public Point ScreenToWorld(Point screenPoint) =>
            new((screenPoint.X - Offset.X) / Zoom, (screenPoint.Y - Offset.Y) / Zoom);
    }
}