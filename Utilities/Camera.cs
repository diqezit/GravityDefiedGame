using Microsoft.Xna.Framework;

namespace GravityDefiedGame.Utilities
{
    public class Camera
    {
        private int _screenWidth;
        private int _screenHeight;
        private Vector2 _position;
        public float Zoom { get; private set; }

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
            Zoom = 3.0f;
        }

        public void CenterOn(Vector2 position)
        {
            _position = position;
        }

        public void Update(Vector2 targetPosition)
        {
            _position = Vector2.Lerp(_position, targetPosition, 0.1f); // Плавное движение
        }

        // преобразования координат из мировых в экранные
        public Vector2 TransformToScreen(Vector2 worldPosition)
        {
            Vector2 viewCenter = new Vector2(_screenWidth / 2, _screenHeight / 2);
            return viewCenter + (worldPosition - _position) * Zoom;
        }

        // преобразования координат из экранных в мировые
        public Vector2 TransformToWorld(Vector2 screenPosition)
        {
            Vector2 viewCenter = new Vector2(_screenWidth / 2, _screenHeight / 2);
            return _position + (screenPosition - viewCenter) / Zoom;
        }
    }
}