using Microsoft.Xna.Framework;

namespace GravityDefiedGame.Utilities
{
    public class Camera
    {
        private int _screenWidth;
        private int _screenHeight;
        private Vector2 _position;
        public float Zoom { get; private set; } = 1.0f;

        public Matrix TransformMatrix
        {
            get
            {
                return Matrix.CreateTranslation(new Vector3(-_position.X, -_position.Y, 0)) *
                       Matrix.CreateScale(Zoom) *
                       Matrix.CreateTranslation(new Vector3(_screenWidth / 2, _screenHeight / 2, 0));
            }
        }

        public Camera(int screenWidth, int screenHeight)
        {
            _screenWidth = screenWidth;
            _screenHeight = screenHeight;
            Reset();
        }

        public void Reset()
        {
            _position = Vector2.Zero;
            Zoom = 1.0f;
        }

        public void CenterOn(Vector2 position)
        {
            _position = position;
        }

        public void Update(Vector2 targetPosition)
        {
            _position = Vector2.Lerp(_position, targetPosition, 0.1f); // Плавное движение
        }

        public Vector2 WorldToScreen(Vector2 worldPoint)
        {
            Vector2 relativeToCamera = worldPoint - _position;
            Vector2 scaled = relativeToCamera * Zoom;
            return scaled + new Vector2(_screenWidth / 2, _screenHeight / 2);
        }

        public Vector2 ScreenToWorld(Vector2 screenPoint)
        {
            Vector2 centered = screenPoint - new Vector2(_screenWidth / 2, _screenHeight / 2);
            Vector2 scaled = centered / Zoom;
            return scaled + _position;
        }

        public Vector2 GetPosition()
        {
            return _position;
        }

        public void UpdateScreenSize(int screenWidth, int screenHeight)
        {
            _screenWidth = screenWidth;
            _screenHeight = screenHeight;
        }
    }
}