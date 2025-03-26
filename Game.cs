using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Threading;
using GravityDefiedGame.Controllers;
using GravityDefiedGame.Models;
using GravityDefiedGame.Utilities;
using GravityDefiedGame.Views;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame
{
    public class Game : Microsoft.Xna.Framework.Game
    {
        #region Fields
        private GraphicsDeviceManager _graphics;
        private SpriteBatch _spriteBatch;
        public static Texture2D PixelTexture;

        private GameController _gameController;
        private Camera _camera;
        private Renderer _renderer;

        private GameState _currentGameState = GameState.Playing;

        private TimeSpan _totalGameTime = TimeSpan.Zero;
        private float _elapsedTime;

        private SpriteFont _font;
        private Dictionary<string, UIButton> _buttons = new Dictionary<string, UIButton>();

        private const float DEFAULT_DELTA_TIME = 0.016f;
        private const float MAX_DELTA_TIME = 0.1f;

        private KeyboardState _previousKeyboardState;
        private MouseState? _previousMouseState;
        private CancellationTokenSource _renderCancellationTokenSource;
        #endregion

        public Game()
        {
            _graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";
            IsMouseVisible = true;

            _graphics.PreferredBackBufferWidth = 1000;
            _graphics.PreferredBackBufferHeight = 700;

            _gameController = new GameController();
            _gameController.GameEvent += GameController_GameEvent;

            Log("Game1", "Initializing game", () => {
                Info("Game1", "Game initialization started");
            });
        }

        protected override void Initialize()
        {
            _camera = new Camera(_graphics.PreferredBackBufferWidth, _graphics.PreferredBackBufferHeight);
            _renderCancellationTokenSource = new CancellationTokenSource();

            base.Initialize();

            Log("Game1", "Initialize completed", () => {
                Info("Game1", "Game initialized successfully");
            });
        }

        protected override void LoadContent()
        {
            _spriteBatch = new SpriteBatch(GraphicsDevice);

            PixelTexture = new Texture2D(GraphicsDevice, 1, 1);
            PixelTexture.SetData(new[] { Color.White });

            try
            {
                _font = Content.Load<SpriteFont>("MainFont");
            }
            catch (Exception ex)
            {
                Error("Game1", $"Could not load font: {ex.Message}");
            }

            _renderer = new Renderer(_spriteBatch, GraphicsDevice, _gameController, _camera);

            InitializeUI();

            _gameController.LoadLevels();
            StartGame(1);

            Log("Game1", "Content loaded", () => {
                Info("Game1", "Game content loaded successfully");
            });
        }

        protected override void Update(GameTime gameTime)
        {
            _elapsedTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
            _elapsedTime = Math.Min(_elapsedTime, MAX_DELTA_TIME);

            _totalGameTime += gameTime.ElapsedGameTime;

            try
            {
                HandleInput();

                if (_currentGameState == GameState.Playing)
                {
                    _gameController.Update(_elapsedTime, _renderCancellationTokenSource.Token);
                    UpdateCamera();
                    CheckGameConditions();
                }

                UpdateUI(_elapsedTime);
            }
            catch (Exception ex)
            {
                Error("Game1", $"Update error: {ex.Message}");
            }

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            try
            {
                GraphicsDevice.Clear(new Color(40, 40, 40));

                _spriteBatch.Begin(transformMatrix: _camera.TransformMatrix);
                _renderer.Render(_renderCancellationTokenSource.Token);
                _spriteBatch.End();

                _spriteBatch.Begin();
                DrawUI();
                _spriteBatch.End();
            }
            catch (Exception ex)
            {
                Error("Game1", $"Draw error: {ex.Message}");
            }

            base.Draw(gameTime);
        }

        #region Game State Management
        private void StartGame(int levelId)
        {
            Log("Game1", $"Starting level {levelId}", () => {
                _renderCancellationTokenSource?.Cancel();
                _renderCancellationTokenSource = new CancellationTokenSource();

                _gameController.StartLevel(levelId);
                _currentGameState = GameState.Playing;
                _camera.CenterOn(_gameController.Motorcycle.Position);

                Info("Game1", $"Level {levelId} started");
            });
        }

        private void PauseGame()
        {
            Log("Game1", "Pausing game", () => {
                if (_currentGameState != GameState.Playing) return;

                _currentGameState = GameState.Paused;
                _gameController.PauseGame();

                Info("Game1", "Game paused");
            });
        }

        private void ResumeGame()
        {
            Log("Game1", "Resuming game", () => {
                if (_currentGameState != GameState.Paused) return;

                _currentGameState = GameState.Playing;
                _gameController.ResumeGame();

                Info("Game1", "Game resumed");
            });
        }

        private void RestartCurrentLevel()
        {
            int currentLevelId = _gameController.CurrentLevel?.Id ?? 1;
            StartGame(currentLevelId);
        }

        private void StartNextLevel()
        {
            int nextLevelId = (_gameController.CurrentLevel?.Id ?? 0) + 1;

            if (nextLevelId <= _gameController.Levels.Count)
            {
                StartGame(nextLevelId);
            }
            else
            {
                ShowMessage("Congratulations! All levels completed!");
            }
        }

        private void GameOver()
        {
            Log("Game1", "Game over", () => {
                _currentGameState = GameState.GameOver;
                ShowMessage("Game Over");
                Info("Game1", "Game over");
            });
        }

        private void LevelComplete()
        {
            Log("Game1", "Level complete", () => {
                _currentGameState = GameState.LevelComplete;
                ShowMessage("Level Complete!");
                Info("Game1", "Level completed");
            });
        }

        private void CheckGameConditions()
        {
            if (_gameController.IsLevelComplete && _currentGameState == GameState.Playing)
                LevelComplete();

            if (_gameController.IsGameOver && _currentGameState == GameState.Playing)
                GameOver();
        }

        private void ShowMessage(string message)
        {
            // In a real implementation, this would show a message on screen
            Log("Game1", $"Message: {message}", () => {
                Debug("Game1", $"Show message: {message}");
            });
        }
        #endregion

        #region Input Handling
        private void HandleInput()
        {
            var keyboardState = Keyboard.GetState();
            var mouseState = Mouse.GetState();

            if (keyboardState.IsKeyDown(Keys.Escape) && _previousKeyboardState.IsKeyUp(Keys.Escape))
            {
                if (_currentGameState == GameState.Playing)
                    PauseGame();
                else if (_currentGameState == GameState.Paused)
                    ResumeGame();
            }

            if (keyboardState.IsKeyDown(Keys.C) && _previousKeyboardState.IsKeyUp(Keys.C) &&
                _currentGameState == GameState.LevelComplete)
            {
                StartNextLevel();
            }

            if (keyboardState.IsKeyDown(Keys.R) && _previousKeyboardState.IsKeyUp(Keys.R))
            {
                if (_currentGameState == GameState.GameOver || _currentGameState == GameState.LevelComplete)
                    RestartCurrentLevel();
            }

            if (_currentGameState == GameState.Playing)
            {
                if (keyboardState.IsKeyDown(Keys.W))
                    _gameController.HandleKeyDown("W");
                else if (_previousKeyboardState.IsKeyDown(Keys.W))
                    _gameController.HandleKeyUp("W");

                if (keyboardState.IsKeyDown(Keys.S))
                    _gameController.HandleKeyDown("S");
                else if (_previousKeyboardState.IsKeyDown(Keys.S))
                    _gameController.HandleKeyUp("S");

                if (keyboardState.IsKeyDown(Keys.A))
                    _gameController.HandleKeyDown("A");
                else if (_previousKeyboardState.IsKeyDown(Keys.A))
                    _gameController.HandleKeyUp("A");

                if (keyboardState.IsKeyDown(Keys.D))
                    _gameController.HandleKeyDown("D");
                else if (_previousKeyboardState.IsKeyDown(Keys.D))
                    _gameController.HandleKeyUp("D");
            }

            HandleUIInput(mouseState, keyboardState);

            _previousKeyboardState = keyboardState;
        }
        #endregion

        #region UI Management
        private void InitializeUI()
        {
            // Create restart button
            _buttons["restart"] = new UIButton
            {
                Bounds = new Rectangle(10, 10, 100, 40),
                Text = "Restart",
                IsVisible = false,
                OnClick = RestartCurrentLevel
            };

            // Create continue button for level complete
            _buttons["continue"] = new UIButton
            {
                Bounds = new Rectangle(450, 350, 100, 40),
                Text = "Continue",
                IsVisible = false,
                OnClick = StartNextLevel
            };
        }

        private void UpdateUI(float deltaTime)
        {
            // Update button visibility based on game state
            _buttons["restart"].IsVisible = _currentGameState == GameState.GameOver;
            _buttons["continue"].IsVisible = _currentGameState == GameState.LevelComplete;
        }

        private void DrawUI()
        {
            // Draw game info
            DrawInfoPanel();

            // Draw state-specific UI
            switch (_currentGameState)
            {
                case GameState.Paused:
                    DrawPauseMenu();
                    break;
                case GameState.GameOver:
                    DrawGameOverMenu();
                    break;
                case GameState.LevelComplete:
                    DrawLevelCompleteMenu();
                    break;
            }

            // Draw buttons
            foreach (var button in _buttons.Values)
            {
                if (button.IsVisible)
                {
                    DrawButton(button);
                }
            }
        }

        private void DrawButton(UIButton button)
        {
            // Draw background
            _spriteBatch.Draw(PixelTexture, button.Bounds, new Color(64, 64, 64));

            // Draw border
            DrawRectangleBorder(button.Bounds, Color.White, 2);

            // Draw text if font is available
            if (_font != null && !string.IsNullOrEmpty(button.Text))
            {
                Vector2 textSize = _font.MeasureString(button.Text);
                Vector2 textPosition = new Vector2(
                    button.Bounds.X + (button.Bounds.Width - textSize.X) / 2,
                    button.Bounds.Y + (button.Bounds.Height - textSize.Y) / 2
                );

                _spriteBatch.DrawString(_font, button.Text, textPosition, Color.White);
            }
        }

        private void DrawRectangleBorder(Rectangle rectangle, Color color, int thickness)
        {
            // Top
            _spriteBatch.Draw(PixelTexture, new Rectangle(rectangle.X, rectangle.Y, rectangle.Width, thickness), color);
            // Bottom
            _spriteBatch.Draw(PixelTexture, new Rectangle(rectangle.X, rectangle.Y + rectangle.Height - thickness, rectangle.Width, thickness), color);
            // Left
            _spriteBatch.Draw(PixelTexture, new Rectangle(rectangle.X, rectangle.Y, thickness, rectangle.Height), color);
            // Right
            _spriteBatch.Draw(PixelTexture, new Rectangle(rectangle.X + rectangle.Width - thickness, rectangle.Y, thickness, rectangle.Height), color);
        }

        private void DrawInfoPanel()
        {
            if (_font != null)
            {
                string levelName = _gameController.CurrentLevel?.Name ?? "Level";
                string timeText = $"{_totalGameTime.Minutes:00}:{_totalGameTime.Seconds:00}";

                _spriteBatch.DrawString(_font, levelName, new Vector2(20, 20), Color.White);
                _spriteBatch.DrawString(_font, timeText, new Vector2(_graphics.PreferredBackBufferWidth - 100, 20), Color.White);

                // Draw controls help
                string controlsText = "Controls: W/S/A/D, R to restart, ESC to pause";
                Vector2 controlsSize = _font.MeasureString(controlsText);
                _spriteBatch.DrawString(_font, controlsText,
                    new Vector2((_graphics.PreferredBackBufferWidth - controlsSize.X) / 2, _graphics.PreferredBackBufferHeight - 40),
                    Color.White);
            }
        }

        private void DrawPauseMenu()
        {
            // Semi-transparent overlay
            _spriteBatch.Draw(PixelTexture, new Rectangle(0, 0, _graphics.PreferredBackBufferWidth, _graphics.PreferredBackBufferHeight),
                new Color(0, 0, 0, 128));

            if (_font != null)
            {
                string pauseText = "GAME PAUSED";
                Vector2 textSize = _font.MeasureString(pauseText);
                _spriteBatch.DrawString(_font, pauseText,
                    new Vector2((_graphics.PreferredBackBufferWidth - textSize.X) / 2, 200),
                    Color.White);

                string resumeText = "Press ESC to resume";
                Vector2 resumeSize = _font.MeasureString(resumeText);
                _spriteBatch.DrawString(_font, resumeText,
                    new Vector2((_graphics.PreferredBackBufferWidth - resumeSize.X) / 2, 250),
                    Color.White);
            }
        }

        private void DrawGameOverMenu()
        {
            // Semi-transparent overlay
            _spriteBatch.Draw(PixelTexture, new Rectangle(0, 0, _graphics.PreferredBackBufferWidth, _graphics.PreferredBackBufferHeight),
                new Color(0, 0, 0, 128));

            if (_font != null)
            {
                string gameOverText = "GAME OVER";
                Vector2 textSize = _font.MeasureString(gameOverText);
                _spriteBatch.DrawString(_font, gameOverText,
                    new Vector2((_graphics.PreferredBackBufferWidth - textSize.X) / 2, 200),
                    Color.Red);

                string restartText = "Press R to restart";
                Vector2 restartSize = _font.MeasureString(restartText);
                _spriteBatch.DrawString(_font, restartText,
                    new Vector2((_graphics.PreferredBackBufferWidth - restartSize.X) / 2, 250),
                    Color.White);
            }
        }

        private void DrawLevelCompleteMenu()
        {
            // Semi-transparent overlay
            _spriteBatch.Draw(PixelTexture, new Rectangle(0, 0, _graphics.PreferredBackBufferWidth, _graphics.PreferredBackBufferHeight),
                new Color(0, 0, 0, 128));

            if (_font != null)
            {
                string completeText = "LEVEL COMPLETE!";
                Vector2 textSize = _font.MeasureString(completeText);
                _spriteBatch.DrawString(_font, completeText,
                    new Vector2((_graphics.PreferredBackBufferWidth - textSize.X) / 2, 200),
                    Color.Green);

                string timeText = $"Time: {_gameController.GameTime.Minutes:00}:{_gameController.GameTime.Seconds:00}";
                Vector2 timeSize = _font.MeasureString(timeText);
                _spriteBatch.DrawString(_font, timeText,
                    new Vector2((_graphics.PreferredBackBufferWidth - timeSize.X) / 2, 250),
                    Color.White);

                string continueText = "Press C to continue, R to restart";
                Vector2 continueSize = _font.MeasureString(continueText);
                _spriteBatch.DrawString(_font, continueText,
                    new Vector2((_graphics.PreferredBackBufferWidth - continueSize.X) / 2, 300),
                    Color.White);
            }
        }

        private void HandleUIInput(MouseState mouseState, KeyboardState keyboardState)
        {
            if (mouseState.LeftButton == ButtonState.Pressed && _previousMouseState?.LeftButton == ButtonState.Released)
            {
                foreach (var button in _buttons.Values)
                {
                    if (button.IsVisible && button.Bounds.Contains(mouseState.X, mouseState.Y))
                    {
                        button.OnClick?.Invoke();
                    }
                }
            }

            _previousMouseState = mouseState;
        }
        #endregion

        #region Helper Methods
        private void UpdateCamera()
        {
            if (_gameController.Motorcycle != null)
            {
                _camera.Update(_gameController.Motorcycle.Position);
            }
        }

        private void GameController_GameEvent(object sender, GameEventArgs e)
        {
            switch (e.Type)
            {
                case GameEventType.LevelComplete:
                    LevelComplete();
                    break;
                case GameEventType.GameOver:
                    GameOver();
                    break;
                case GameEventType.BikeChanged:
                    ShowMessage(e.Message);
                    break;
            }
        }

        protected override void UnloadContent()
        {
            _renderCancellationTokenSource?.Cancel();
            base.UnloadContent();
        }
        #endregion
    }

    public class UIButton
    {
        public Rectangle Bounds { get; set; }
        public bool IsVisible { get; set; } = true;
        public Action OnClick { get; set; }
        public string Text { get; set; }
    }

    public enum GameState
    {
        Playing,
        Paused,
        GameOver,
        LevelComplete
    }
}