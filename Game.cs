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
        private FontSystem _fontSystem;          // Для обычного текста
        private FontSystem _symbolFontSystem;    // Для символов (звезд)
        private DynamicSpriteFont _uiFont;       // Обычный шрифт UI
        private DynamicSpriteFont _titleFont;    // Шрифт заголовков
        private DynamicSpriteFont _unicodeFont;  // Шрифт для символов Unicode (звезд)
        private Texture2D _pixelTexture;
        private Texture2D _gradientTexture;      // Для градиентного фона
        private Texture2D _starFilledTexture;    // Текстура заполненной звезды
        private Texture2D _starEmptyTexture;     // Текстура пустой звезды
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
            _graphics = new GraphicsDeviceManager(this);
            _graphics.PreferredBackBufferWidth = SCREEN_WIDTH;
            _graphics.PreferredBackBufferHeight = SCREEN_HEIGHT;
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
            base.Initialize();
            Log("Game", "Игра инициализирована", () => Info("Game", "Initialize завершен"));
        }

        protected override void LoadContent()
        {
            _spriteBatch = new SpriteBatch(GraphicsDevice);
            _pixelTexture = new Texture2D(GraphicsDevice, 1, 1);
            _pixelTexture.SetData(new[] { Color.White });

            _gradientTexture = CreateGradientTexture(GraphicsDevice, SCREEN_WIDTH, SCREEN_HEIGHT);

            _fontSystem = new FontSystem();
            using (var stream = TitleContainer.OpenStream("Content/Fonts/NotoSans-Regular.ttf"))
            {
                _fontSystem.AddFont(stream);
            }

            _symbolFontSystem = new FontSystem();
            using (var stream = TitleContainer.OpenStream("Content/Fonts/NotoSansSymbols-VariableFont_wght.ttf"))
            {
                _symbolFontSystem.AddFont(stream);
            }

            _uiFont = _fontSystem.GetFont(20);
            _titleFont = _fontSystem.GetFont(40);
            _unicodeFont = _symbolFontSystem.GetFont(24);

            _starFilledTexture = Content.Load<Texture2D>("star_filled");
            _starEmptyTexture = Content.Load<Texture2D>("star_empty");

            _renderer = new Renderer(_spriteBatch, GraphicsDevice, _gameController, _camera);
            _uiController = new UIController(this, _gameController, _spriteBatch, _uiFont, _titleFont, _unicodeFont,
                _pixelTexture, _gradientTexture, _starFilledTexture, _starEmptyTexture, SCREEN_WIDTH, SCREEN_HEIGHT);
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
                for (int x = 0; x < width; x++)
                {
                    float t = (float)y / height;
                    data[y * width + x] = Color.Lerp(new Color(20, 20, 50), new Color(80, 80, 120), t);
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
                GraphicsDevice.Clear(ThemeConstants.BackgroundColor);

                _spriteBatch.Begin(transformMatrix: _camera.TransformMatrix);
                try
                {
                    if (_gameController.CurrentLevel != null)
                    {
                        _renderer.Render(_renderCancellationTokenSource.Token);
                    }
                }
                finally
                {
                    _spriteBatch.End();
                }

                _spriteBatch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);
                try
                {
                    _uiController.DrawUI(_gameController);
                }
                finally
                {
                    _spriteBatch.End();
                }

                base.Draw(gameTime);
            }
            catch (OperationCanceledException) { Debug("Game", "Отрисовка отменена"); }
            catch (Exception ex) { Error("Game", $"Ошибка отрисовки: {ex.Message}"); }
        }

        private void UpdateCamera()
        {
            if (_gameController.Motorcycle != null)
            {
                Vector2 visualCenter = _gameController.Motorcycle.GetVisualCenter();
                _camera.Update(visualCenter);
            }
        }

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
            _starFilledTexture?.Dispose();
            _starEmptyTexture?.Dispose();
            base.UnloadContent();
            Log("Game", "Контент выгружен", () => Info("Game", "UnloadContent завершен"));
        }
    }
}