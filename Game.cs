using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Threading;
using GravityDefiedGame.Controllers;
using GravityDefiedGame.Models;
using GravityDefiedGame.Utilities;
using GravityDefiedGame.Views;
using static GravityDefiedGame.Utilities.Logger;
using FontStashSharp;

namespace GravityDefiedGame
{
    public class Game : Microsoft.Xna.Framework.Game
    {
        #region Constants
        private const float MAX_DELTA_TIME = 0.1f;
        private const int SCREEN_WIDTH = 1000;
        private const int SCREEN_HEIGHT = 700;
        #endregion

        #region Fields
        private FontSystem _fontSystem;
        private FontSystem _symbolFontSystem;
        private DynamicSpriteFont _uiFont;
        private DynamicSpriteFont _titleFont;
        private DynamicSpriteFont _unicodeFont;
        private Texture2D _pixelTexture;
        private Texture2D _gradientTexture;
        private readonly GraphicsDeviceManager _graphics;
        private SpriteBatch _spriteBatch;
        private GameController _gameController;
        private Camera _camera;
        private Renderer _renderer;
        private float _elapsedTime;
        private UIController _uiController;
        private CancellationTokenSource _renderCancellationTokenSource;
        #endregion

        public Game()
        {
            _graphics = new GraphicsDeviceManager(this)
            {
                PreferredBackBufferWidth = SCREEN_WIDTH,
                PreferredBackBufferHeight = SCREEN_HEIGHT
            };

            Content.RootDirectory = "Content";
            IsMouseVisible = true;

            _gameController = new GameController();
            _gameController.GameEvent += GameController_GameEvent;
            Log("Game", "Инициализация игры началась", () => Info("Game", "Конструктор завершен"));
        }

        protected override void Initialize()
        {
            _camera = new Camera(SCREEN_WIDTH, SCREEN_HEIGHT);
            _renderCancellationTokenSource = new CancellationTokenSource();

            ThemeManager.ThemeChanged += OnThemeChanged;

            base.Initialize();
            Log("Game", "Игра инициализирована", () => Info("Game", "Initialize завершен"));
        }

        private void OnThemeChanged()
        {
            if (_gameController?.Motorcycle != null)
            {
                _gameController.Motorcycle.UpdateFromTheme();
            }
        }

        protected override void LoadContent()
        {
            _spriteBatch = new SpriteBatch(GraphicsDevice);

            // Инициализация текстур
            _pixelTexture = new Texture2D(GraphicsDevice, 1, 1);
            _pixelTexture.SetData(new[] { Color.White });
            _gradientTexture = CreateGradientTexture(GraphicsDevice, SCREEN_WIDTH, SCREEN_HEIGHT);

            // Инициализация шрифтов
            _fontSystem = new FontSystem();
            _symbolFontSystem = new FontSystem();

            using (var regularFont = TitleContainer.OpenStream("Content/Fonts/NotoSans-Regular.ttf"))
            using (var symbolFont = TitleContainer.OpenStream("Content/Fonts/NotoSansSymbols-VariableFont_wght.ttf"))
            {
                _fontSystem.AddFont(regularFont);
                _symbolFontSystem.AddFont(symbolFont);
            }

            _uiFont = _fontSystem.GetFont(20);
            _titleFont = _fontSystem.GetFont(40);
            _unicodeFont = _symbolFontSystem.GetFont(24);

            // Инициализация компонентов
            _renderer = new Renderer(_spriteBatch, GraphicsDevice, _gameController, _camera);
            _uiController = new UIController(
                this,
                _gameController,
                _spriteBatch,
                _uiFont,
                _titleFont,
                _unicodeFont,
                _pixelTexture,
                _gradientTexture,
                SCREEN_WIDTH,
                SCREEN_HEIGHT);

            _gameController.LoadLevels();
            _uiController.ShowMainMenu();

            Log("Game", "Контент загружен", () => Info("Game", "LoadContent завершен"));
        }

        private Texture2D CreateGradientTexture(GraphicsDevice graphicsDevice, int width, int height)
        {
            Texture2D texture = new Texture2D(graphicsDevice, width, height);
            Color[] data = new Color[width * height];

            for (int y = 0; y < height; y++)
            {
                float t = (float)y / height;
                var color = Color.Lerp(new Color(20, 20, 50), new Color(80, 80, 120), t);

                for (int x = 0; x < width; x++)
                {
                    data[y * width + x] = color;
                }
            }

            texture.SetData(data);
            return texture;
        }

        protected override void Update(GameTime gameTime)
        {
            try
            {
                _elapsedTime = Math.Min((float)gameTime.ElapsedGameTime.TotalSeconds, MAX_DELTA_TIME);
                _uiController.HandleInput(_gameController, _elapsedTime);

                if (_gameController.CurrentGameState == GameState.Playing)
                {
                    _gameController.Update(_elapsedTime, _renderCancellationTokenSource.Token);
                    UpdateCamera();
                }

                base.Update(gameTime);
            }
            catch (OperationCanceledException) { Debug("Game", "Обновление отменено"); }
            catch (Exception ex) { Error("Game", $"Ошибка обновления: {ex.Message}"); }
        }

        protected override void Draw(GameTime gameTime)
        {
            try
            {
                GraphicsDevice.Clear(ThemeManager.CurrentTheme.BackgroundColor);

                // Отрисовка игрового мира с учетом камеры
                DrawGameWorld();

                // Отрисовка UI
                DrawUserInterface();

                base.Draw(gameTime);
            }
            catch (OperationCanceledException) { Debug("Game", "Отрисовка отменена"); }
            catch (Exception ex) { Error("Game", $"Ошибка отрисовки: {ex.Message}"); }
        }

        private void DrawGameWorld()
        {
            _spriteBatch.Begin(transformMatrix: _camera.TransformMatrix);
            try
            {
                if (_gameController.CurrentLevel != null)
                {
                    _renderer.Render(_renderCancellationTokenSource.Token);
                }
            }
            catch (Exception ex)
            {
                Error("Game", $"Error rendering game: {ex.Message}");
            }
            finally
            {
                _spriteBatch.End();
            }
        }

        private void DrawUserInterface()
        {
            _spriteBatch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);
            try
            {
                _uiController.DrawUI(_gameController);
            }
            catch (Exception ex)
            {
                Error("Game", $"Error rendering UI: {ex.Message}");
            }
            finally
            {
                _spriteBatch.End();
            }
        }

        private void UpdateCamera()
        {
            if (_gameController.Motorcycle != null)
            {
                Vector2 visualCenter = _gameController.Motorcycle.GetVisualCenter();

                if (IsValidPosition(visualCenter))
                {
                    _camera.Update(visualCenter);
                }
                else
                {
                    Warning("Game", $"Invalid visual center: {visualCenter}");
                }
            }
        }

        private static bool IsValidPosition(Vector2 position) =>
            !float.IsNaN(position.X) && !float.IsNaN(position.Y) &&
            !float.IsInfinity(position.X) && !float.IsInfinity(position.Y);

        private void GameController_GameEvent(object sender, GameEventArgs e)
        {
            switch (e.Type)
            {
                case GameEventType.LevelComplete:
                case GameEventType.GameOver:
                case GameEventType.BikeChanged:
                case GameEventType.CheckpointReached:
                case GameEventType.Error:
                    _uiController.ShowMessage(e.Message);
                    break;
            }
        }

        protected override void UnloadContent()
        {
            _renderCancellationTokenSource?.Cancel();
            _renderCancellationTokenSource?.Dispose();
            _pixelTexture?.Dispose();
            _gradientTexture?.Dispose();

            ThemeManager.ThemeChanged -= OnThemeChanged;

            base.UnloadContent();
            Log("Game", "Контент выгружен", () => Info("Game", "UnloadContent завершен"));
        }
    }
}