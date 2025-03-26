using Microsoft.Xna.Framework;
using System;

namespace GravityDefiedGame.Utilities
{
    public class Camera
    {
        public Vector2 Position { get; private set; }
        public Vector2 Offset { get; private set; }
        public float Zoom { get; private set; } = 1.0f;

        private readonly float _followSpeed = 5.0f;
        private int _screenWidth;
        private int _screenHeight;
        private Vector2 _targetPosition;

        public Matrix TransformMatrix =>
            Matrix.CreateTranslation(new Vector3(-Position.X, -Position.Y, 0)) *
            Matrix.CreateScale(Zoom) *
            Matrix.CreateTranslation(new Vector3(Offset.X, Offset.Y, 0));

        public Camera(int screenWidth, int screenHeight)
        {
            _screenWidth = screenWidth;
            _screenHeight = screenHeight;
            Reset();
        }

        public void Reset()
        {
            Offset = new Vector2(_screenWidth / 2, _screenHeight / 2);
            Position = Vector2.Zero;
            _targetPosition = Vector2.Zero;
        }

        public void CenterOn(Vector2 position)
        {
            Offset = new Vector2(
                _screenWidth / 2,
                _screenHeight / 2);

            Position = position;
            _targetPosition = position;
        }

        public void Update(Vector2 targetPosition)
        {
            _targetPosition = targetPosition;

            Vector2 desiredPosition = targetPosition;
            Position = Vector2.Lerp(Position, desiredPosition, _followSpeed * 0.016f);
        }

        public Vector2 WorldToScreen(Vector2 worldPoint)
        {
            return Vector2.Transform(worldPoint, TransformMatrix);
        }

        public Vector2 ScreenToWorld(Vector2 screenPoint)
        {
            return Vector2.Transform(screenPoint, Matrix.Invert(TransformMatrix));
        }

        public void UpdateScreenSize(int screenWidth, int screenHeight)
        {
            _screenWidth = screenWidth;
            _screenHeight = screenHeight;
            Offset = new Vector2(_screenWidth / 2, _screenHeight / 2);
        }
    }
}