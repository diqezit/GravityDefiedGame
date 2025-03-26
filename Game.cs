using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Linq;
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
        #region Constants
        private const float MAX_DELTA_TIME = 0.1f;
        private const int SCREEN_WIDTH = 1000;
        private const int SCREEN_HEIGHT = 700;
        #endregion

        #region Fields
        private readonly GraphicsDeviceManager _graphics;
        private SpriteBatch _spriteBatch;
        private Texture2D _pixelTexture;
        private GameController _gameController;
        private Camera _camera;
        private Renderer _renderer;
        private float _elapsedTime;
        private SpriteFont _font;
        private readonly Dictionary<string, UIButton> _buttons = new();
        private KeyboardState _previousKeyboardState;
        private MouseState _previousMouseState;
        private CancellationTokenSource _renderCancellationTokenSource;

        private int _selectedBikeIndex = 0;
        private int _selectedLevelIndex = 0;
        private readonly List<Color> _availableBikeColors = new()
        {
            Color.Red, Color.Blue, Color.Green, Color.Yellow, Color.Purple, Color.Orange
        };
        private int _selectedColorIndex = 0;
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
            Log("Game", "Game initialization started", () => Info("Game", "Constructor completed"));
        }

        protected override void Initialize()
        {
            _camera = new Camera(SCREEN_WIDTH, SCREEN_HEIGHT);
            _renderCancellationTokenSource = new CancellationTokenSource();
            base.Initialize();
            Log("Game", "Game initialized", () => Info("Game", "Initialize completed"));
        }

        protected override void LoadContent()
        {
            _spriteBatch = new SpriteBatch(GraphicsDevice);
            _pixelTexture = new Texture2D(GraphicsDevice, 1, 1);
            _pixelTexture.SetData(new[] { Color.White });

            _font = Content.Load<SpriteFont>("Arial");

            _renderer = new Renderer(_spriteBatch, GraphicsDevice, _gameController, _camera);
            InitializeUI();
            _gameController.LoadLevels();
            ShowMainMenu();
            Log("Game", "Content loaded", () => Info("Game", "LoadContent completed"));
        }

        protected override void Update(GameTime gameTime)
        {
            try
            {
                _elapsedTime = Math.Min((float)gameTime.ElapsedGameTime.TotalSeconds, MAX_DELTA_TIME);

                HandleInput();

                if (_gameController.CurrentGameState == GameState.Playing)
                {
                    _gameController.Update(_elapsedTime, _renderCancellationTokenSource.Token);
                    UpdateCamera();
                }

                UpdateUI(_elapsedTime);
                base.Update(gameTime);
            }
            catch (OperationCanceledException) { Debug("Game", "Update cancelled"); }
            catch (Exception ex) { Error("Game", $"Update error: {ex.Message}"); }
        }

        protected override void Draw(GameTime gameTime)
        {
            try
            {
                GraphicsDevice.Clear(ThemeConstants.BackgroundColor);

                _spriteBatch.Begin(transformMatrix: _camera.TransformMatrix);
                if (_gameController.CurrentLevel != null)
                {
                    _renderer.Render(_renderCancellationTokenSource.Token);
                }
                _spriteBatch.End();

                _spriteBatch.Begin();
                DrawUI();
                _spriteBatch.End();

                base.Draw(gameTime);
            }
            catch (OperationCanceledException) { Debug("Game", "Draw cancelled"); }
            catch (Exception ex) { Error("Game", $"Draw error: {ex.Message}"); }
        }

        private void ShowMainMenu()
        {
            _gameController.EnterMainMenu();
            Log("Game", "Main menu displayed", () => Info("Game", "ShowMainMenu completed"));
        }

        private void ShowBikeSelection()
        {
            _gameController.EnterBikeSelection();
            var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();
            _gameController.SetBikeType(availableBikes[_selectedBikeIndex]);
            _gameController.SetBikeColor(_availableBikeColors[_selectedColorIndex]);
            Log("Game", "Bike selection menu displayed", () => Info("Game", "ShowBikeSelection completed"));
        }

        private void ShowLevelSelection()
        {
            _gameController.EnterLevelSelection();
            _gameController.SelectLevel(_selectedLevelIndex + 1);
            Log("Game", "Level selection menu displayed", () => Info("Game", "ShowLevelSelection completed"));
        }

        private void StartGame(int levelId)
        {
            _renderCancellationTokenSource?.Cancel();
            _renderCancellationTokenSource = new CancellationTokenSource();
            _gameController.StartLevel(levelId);

            if (_gameController.Motorcycle != null)
            {
                _camera.Reset();
                _camera.CenterOn(_gameController.Motorcycle.GetVisualCenter());
            }

            Log("Game", $"Level {levelId} started", () => Info("Game", "StartGame completed"));
        }

        private void HandleInput()
        {
            var keyboardState = Keyboard.GetState();
            var mouseState = Mouse.GetState();

            if (_gameController.CurrentGameState == GameState.MainMenu ||
                _gameController.CurrentGameState == GameState.BikeSelection ||
                _gameController.CurrentGameState == GameState.LevelSelection)
            {
                HandleMenuInput(keyboardState);
            }
            else
            {
                _gameController.HandleInput(keyboardState, _previousKeyboardState);
            }

            if (mouseState.LeftButton == ButtonState.Pressed && _previousMouseState.LeftButton == ButtonState.Released)
            {
                foreach (var button in _buttons.Values)
                {
                    if (button.IsVisible && button.Bounds.Contains(mouseState.X, mouseState.Y))
                        button.OnClick?.Invoke();
                }
            }

            _previousKeyboardState = keyboardState;
            _previousMouseState = mouseState;
        }

        private void HandleMenuInput(KeyboardState keyboardState)
        {
            if (keyboardState.IsKeyDown(Keys.Escape) && _previousKeyboardState.IsKeyUp(Keys.Escape))
            {
                if (_gameController.CurrentGameState == GameState.BikeSelection ||
                    _gameController.CurrentGameState == GameState.LevelSelection)
                {
                    ShowMainMenu();
                }
            }

            if (_gameController.CurrentGameState == GameState.BikeSelection)
            {
                var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();

                if (keyboardState.IsKeyDown(Keys.Left) && _previousKeyboardState.IsKeyUp(Keys.Left))
                {
                    _selectedBikeIndex = (_selectedBikeIndex - 1 + availableBikes.Count) % availableBikes.Count;
                    _gameController.SetBikeType(availableBikes[_selectedBikeIndex]);
                }
                else if (keyboardState.IsKeyDown(Keys.Right) && _previousKeyboardState.IsKeyUp(Keys.Right))
                {
                    _selectedBikeIndex = (_selectedBikeIndex + 1) % availableBikes.Count;
                    _gameController.SetBikeType(availableBikes[_selectedBikeIndex]);
                }
                else if (keyboardState.IsKeyDown(Keys.Up) && _previousKeyboardState.IsKeyUp(Keys.Up))
                {
                    _selectedColorIndex = (_selectedColorIndex - 1 + _availableBikeColors.Count) % _availableBikeColors.Count;
                    _gameController.SetBikeColor(_availableBikeColors[_selectedColorIndex]);
                }
                else if (keyboardState.IsKeyDown(Keys.Down) && _previousKeyboardState.IsKeyUp(Keys.Down))
                {
                    _selectedColorIndex = (_selectedColorIndex + 1) % _availableBikeColors.Count;
                    _gameController.SetBikeColor(_availableBikeColors[_selectedColorIndex]);
                }
                else if (keyboardState.IsKeyDown(Keys.Enter) && _previousKeyboardState.IsKeyUp(Keys.Enter))
                {
                    ShowLevelSelection();
                }
            }

            if (_gameController.CurrentGameState == GameState.LevelSelection)
            {
                if (keyboardState.IsKeyDown(Keys.Left) && _previousKeyboardState.IsKeyUp(Keys.Left))
                {
                    _selectedLevelIndex = Math.Max(0, _selectedLevelIndex - 1);
                    _gameController.SelectLevel(_selectedLevelIndex + 1);
                }
                else if (keyboardState.IsKeyDown(Keys.Right) && _previousKeyboardState.IsKeyUp(Keys.Right))
                {
                    _selectedLevelIndex = Math.Min(_gameController.Levels.Count - 1, _selectedLevelIndex + 1);
                    _gameController.SelectLevel(_selectedLevelIndex + 1);
                }
                else if (keyboardState.IsKeyDown(Keys.Enter) && _previousKeyboardState.IsKeyUp(Keys.Enter))
                {
                    StartGame(_selectedLevelIndex + 1);
                }
            }
        }

        private void InitializeUI()
        {
            _buttons["restart"] = new UIButton
            {
                Bounds = new Rectangle(10, 10, 100, 40),
                Text = "Restart",
                IsVisible = false,
                OnClick = () => _gameController.RestartLevel()
            };

            _buttons["continue"] = new UIButton
            {
                Bounds = new Rectangle(450, 350, 100, 40),
                Text = "Continue",
                IsVisible = false,
                OnClick = () => _gameController.StartNextLevel()
            };

            _buttons["pause"] = new UIButton
            {
                Bounds = new Rectangle(SCREEN_WIDTH - 110, 10, 100, 40),
                Text = "Pause",
                IsVisible = false,
                OnClick = () => _gameController.PauseGame()
            };

            _buttons["play"] = new UIButton
            {
                Bounds = new Rectangle(SCREEN_WIDTH / 2 - 150, 250, 300, 50),
                Text = "Play Game",
                IsVisible = true,
                OnClick = () => ShowBikeSelection()
            };

            _buttons["exit"] = new UIButton
            {
                Bounds = new Rectangle(SCREEN_WIDTH / 2 - 150, 320, 300, 50),
                Text = "Exit",
                IsVisible = true,
                OnClick = () => Exit()
            };

            _buttons["selectBike"] = new UIButton
            {
                Bounds = new Rectangle(SCREEN_WIDTH / 2 - 100, 400, 200, 50),
                Text = "Select This Bike",
                IsVisible = false,
                OnClick = () => ShowLevelSelection()
            };

            _buttons["startLevel"] = new UIButton
            {
                Bounds = new Rectangle(SCREEN_WIDTH / 2 - 100, 400, 200, 50),
                Text = "Start Level",
                IsVisible = false,
                OnClick = () => StartGame(_selectedLevelIndex + 1)
            };

            Log("Game", "UI initialized", () => Info("Game", "InitializeUI completed"));
        }

        private void UpdateUI(float deltaTime)
        {
            _buttons["restart"].IsVisible = _gameController.CurrentGameState == GameState.GameOver;
            _buttons["continue"].IsVisible = _gameController.CurrentGameState == GameState.LevelComplete;
            _buttons["pause"].IsVisible = _gameController.CurrentGameState == GameState.Playing;

            _buttons["play"].IsVisible = _gameController.CurrentGameState == GameState.MainMenu;
            _buttons["exit"].IsVisible = _gameController.CurrentGameState == GameState.MainMenu;

            _buttons["selectBike"].IsVisible = _gameController.CurrentGameState == GameState.BikeSelection;

            _buttons["startLevel"].IsVisible = _gameController.CurrentGameState == GameState.LevelSelection;
        }

        private void DrawUI()
        {
            switch (_gameController.CurrentGameState)
            {
                case GameState.MainMenu:
                    DrawMainMenu();
                    break;
                case GameState.BikeSelection:
                    DrawBikeSelectionMenu();
                    break;
                case GameState.LevelSelection:
                    DrawLevelSelectionMenu();
                    break;
                case GameState.Playing:
                    DrawInfoPanel();
                    break;
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

            foreach (var button in _buttons.Values)
            {
                if (button.IsVisible)
                    DrawButton(button);
            }
        }

        private void DrawMainMenu()
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT), new Color(0, 0, 0, 100));

            Rectangle menuRect = new Rectangle(SCREEN_WIDTH / 2 - 250, SCREEN_HEIGHT / 2 - 200, 500, 400);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(40, 40, 40, 230));
            DrawRectangleBorder(menuRect, new Color(150, 150, 150), 2);

            string titleText = "GRAVITY DEFIED";
            Vector2 titleSize = _font.MeasureString(titleText);
            _spriteBatch.DrawString(_font, titleText,
                new Vector2((SCREEN_WIDTH - titleSize.X) / 2, menuRect.Y + 50),
                Color.White);

            string subtitleText = "A Motorcycle Physics Game";
            Vector2 subtitleSize = _font.MeasureString(subtitleText);
            _spriteBatch.DrawString(_font, subtitleText,
                new Vector2((SCREEN_WIDTH - subtitleSize.X) / 2, menuRect.Y + 100),
                Color.LightGray);
        }

        private void DrawBikeSelectionMenu()
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT), new Color(0, 0, 0, 100));

            Rectangle menuRect = new Rectangle(SCREEN_WIDTH / 2 - 300, SCREEN_HEIGHT / 2 - 200, 600, 400);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(40, 40, 40, 230));
            DrawRectangleBorder(menuRect, new Color(150, 150, 150), 2);

            string titleText = "SELECT YOUR BIKE";
            Vector2 titleSize = _font.MeasureString(titleText);
            _spriteBatch.DrawString(_font, titleText,
                new Vector2((SCREEN_WIDTH - titleSize.X) / 2, menuRect.Y + 30),
                Color.White);

            var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();
            string bikeTypeText = availableBikes[_selectedBikeIndex].ToString();
            Vector2 bikeTypeSize = _font.MeasureString(bikeTypeText);
            _spriteBatch.DrawString(_font, bikeTypeText,
                new Vector2((SCREEN_WIDTH - bikeTypeSize.X) / 2, menuRect.Y + 100),
                _availableBikeColors[_selectedColorIndex]);

            string instructionText = "< Left/Right: Change Bike | Up/Down: Change Color | Enter: Confirm >";
            Vector2 instructionSize = _font.MeasureString(instructionText);
            _spriteBatch.DrawString(_font, instructionText,
                new Vector2((SCREEN_WIDTH - instructionSize.X) / 2, menuRect.Y + 300),
                Color.LightGray);

            DrawBikePreview(menuRect);
        }

        private void DrawBikePreview(Rectangle menuRect)
        {
            int previewX = menuRect.X + 150;
            int previewY = menuRect.Y + 150;
            int previewWidth = 300;
            int previewHeight = 120;

            Rectangle previewRect = new Rectangle(previewX, previewY, previewWidth, previewHeight);
            _spriteBatch.Draw(_pixelTexture, previewRect, new Color(20, 20, 20, 150));
            DrawRectangleBorder(previewRect, new Color(100, 100, 100), 1);

            string previewText = "Bike Preview";
            Vector2 previewTextSize = _font.MeasureString(previewText);
            _spriteBatch.DrawString(_font, previewText,
                new Vector2(previewX + (previewWidth - previewTextSize.X) / 2, previewY + 10),
                Color.Gray);

            string statsText = GetBikeStats(Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList()[_selectedBikeIndex]);
            Vector2 statsSize = _font.MeasureString(statsText);
            _spriteBatch.DrawString(_font, statsText,
                new Vector2(previewX + 20, previewY + 50),
                Color.White);
        }

        private string GetBikeStats(BikeType bikeType)
        {
            switch (bikeType)
            {
                case BikeType.Standard:
                    return "Speed: 3/5\nHandling: 3/5\nStability: 3/5";
                case BikeType.Sport:
                    return "Speed: 5/5\nHandling: 2/5\nStability: 2/5";
                case BikeType.OffRoad:
                    return "Speed: 2/5\nHandling: 4/5\nStability: 5/5";
                default:
                    return "Speed: 3/5\nHandling: 3/5\nStability: 3/5";
            }
        }

        private void DrawLevelSelectionMenu()
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT), new Color(0, 0, 0, 100));

            Rectangle menuRect = new Rectangle(SCREEN_WIDTH / 2 - 300, SCREEN_HEIGHT / 2 - 200, 600, 400);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(40, 40, 40, 230));
            DrawRectangleBorder(menuRect, new Color(150, 150, 150), 2);

            string titleText = "SELECT LEVEL";
            Vector2 titleSize = _font.MeasureString(titleText);
            _spriteBatch.DrawString(_font, titleText,
                new Vector2((SCREEN_WIDTH - titleSize.X) / 2, menuRect.Y + 30),
                Color.White);

            string levelText = $"Level {_selectedLevelIndex + 1}: {_gameController.Levels[_selectedLevelIndex].Name}";
            Vector2 levelSize = _font.MeasureString(levelText);
            _spriteBatch.DrawString(_font, levelText,
                new Vector2((SCREEN_WIDTH - levelSize.X) / 2, menuRect.Y + 100),
                Color.Yellow);

            string difficultyText = $"Difficulty: {GetLevelDifficulty(_selectedLevelIndex + 1)}";
            Vector2 difficultySize = _font.MeasureString(difficultyText);
            _spriteBatch.DrawString(_font, difficultyText,
                new Vector2((SCREEN_WIDTH - difficultySize.X) / 2, menuRect.Y + 150),
                Color.White);

            string instructionText = "< Left/Right: Change Level | Enter: Start Game >";
            Vector2 instructionSize = _font.MeasureString(instructionText);
            _spriteBatch.DrawString(_font, instructionText,
                new Vector2((SCREEN_WIDTH - instructionSize.X) / 2, menuRect.Y + 300),
                Color.LightGray);
        }

        private string GetLevelDifficulty(int levelId)
        {
            if (levelId <= 5) return "Easy";
            if (levelId <= 15) return "Medium";
            return "Hard";
        }

        private void DrawButton(UIButton button)
        {
            _spriteBatch.Draw(_pixelTexture, button.Bounds, new Color(64, 64, 64));
            DrawRectangleBorder(button.Bounds, Color.White, 2);

            if (!string.IsNullOrEmpty(button.Text))
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
            _spriteBatch.Draw(_pixelTexture, new Rectangle(rectangle.X, rectangle.Y, rectangle.Width, thickness), color);
            _spriteBatch.Draw(_pixelTexture, new Rectangle(rectangle.X, rectangle.Y + rectangle.Height - thickness, rectangle.Width, thickness), color);
            _spriteBatch.Draw(_pixelTexture, new Rectangle(rectangle.X, rectangle.Y, thickness, rectangle.Height), color);
            _spriteBatch.Draw(_pixelTexture, new Rectangle(rectangle.X + rectangle.Width - thickness, rectangle.Y, thickness, rectangle.Height), color);
        }

        private void DrawInfoPanel()
        {
            string levelName = _gameController.CurrentLevel?.Name ?? "Level";
            string timeText = $"Time: {_gameController.GameTime.Minutes:00}:{_gameController.GameTime.Seconds:00}";
            string directionText = _gameController.Motorcycle.Direction == 1 ? "Forward" : "Backward";

            Rectangle infoPanelRect = new Rectangle(10, 10, 300, 80);
            _spriteBatch.Draw(_pixelTexture, infoPanelRect, new Color(0, 0, 0, 128));
            DrawRectangleBorder(infoPanelRect, new Color(150, 150, 150), 1);

            _spriteBatch.DrawString(_font, levelName, new Vector2(20, 20), Color.White);
            _spriteBatch.DrawString(_font, timeText, new Vector2(20, 45), Color.White);
            _spriteBatch.DrawString(_font, $"Direction: {directionText}", new Vector2(20, 70), Color.White);

            string controlsText = "Controls: W - Forward, S - Backward, Space - Brake, A/D - Lean, R - Restart, ESC - Pause";
            Vector2 controlsSize = _font.MeasureString(controlsText);
            _spriteBatch.DrawString(_font, controlsText,
                new Vector2((SCREEN_WIDTH - controlsSize.X) / 2, SCREEN_HEIGHT - 35),
                Color.White);
        }

        private void DrawPauseMenu()
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT), new Color(0, 0, 0, 180));

            Rectangle menuRect = new Rectangle(SCREEN_WIDTH / 2 - 200, SCREEN_HEIGHT / 2 - 150, 400, 300);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(40, 40, 40, 230));
            DrawRectangleBorder(menuRect, new Color(150, 150, 150), 2);

            string pauseText = "GAME PAUSED";
            Vector2 textSize = _font.MeasureString(pauseText);
            _spriteBatch.DrawString(_font, pauseText,
                new Vector2((SCREEN_WIDTH - textSize.X) / 2, menuRect.Y + 30),
                Color.White);

            string resumeText = "Press ESC to resume";
            Vector2 resumeSize = _font.MeasureString(resumeText);
            _spriteBatch.DrawString(_font, resumeText,
                new Vector2((SCREEN_WIDTH - resumeSize.X) / 2, menuRect.Y + 100),
                Color.White);

            string restartText = "Press R to restart level";
            Vector2 restartSize = _font.MeasureString(restartText);
            _spriteBatch.DrawString(_font, restartText,
                new Vector2((SCREEN_WIDTH - restartSize.X) / 2, menuRect.Y + 140),
                Color.White);

            string levelText = $"Level: {_gameController.CurrentLevel?.Name ?? "Unknown"}";
            string timeText = $"Current time: {_gameController.GameTime.Minutes:00}:{_gameController.GameTime.Seconds:00}";
            _spriteBatch.DrawString(_font, levelText,
                new Vector2(menuRect.X + 30, menuRect.Y + 200),
                Color.LightGray);
            _spriteBatch.DrawString(_font, timeText,
                new Vector2(menuRect.X + 30, menuRect.Y + 230),
                Color.LightGray);
        }

        private void DrawGameOverMenu()
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT), new Color(0, 0, 0, 180));

            Rectangle menuRect = new Rectangle(SCREEN_WIDTH / 2 - 200, SCREEN_HEIGHT / 2 - 150, 400, 300);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(40, 40, 40, 230));
            DrawRectangleBorder(menuRect, new Color(150, 0, 0), 2);

            string gameOverText = "GAME OVER";
            Vector2 textSize = _font.MeasureString(gameOverText);
            _spriteBatch.DrawString(_font, gameOverText,
                new Vector2((SCREEN_WIDTH - textSize.X) / 2, menuRect.Y + 30),
                Color.Red);

            string restartText = "Press R to restart level";
            Vector2 restartSize = _font.MeasureString(restartText);
            _spriteBatch.DrawString(_font, restartText,
                new Vector2((SCREEN_WIDTH - restartSize.X) / 2, menuRect.Y + 100),
                Color.White);

            string levelText = $"Level: {_gameController.CurrentLevel?.Name ?? "Unknown"}";
            string timeText = $"Time: {_gameController.GameTime.Minutes:00}:{_gameController.GameTime.Seconds:00}";
            _spriteBatch.DrawString(_font, levelText,
                new Vector2(menuRect.X + 30, menuRect.Y + 180),
                Color.LightGray);
            _spriteBatch.DrawString(_font, timeText,
                new Vector2(menuRect.X + 30, menuRect.Y + 210),
                Color.LightGray);
        }

        private void DrawLevelCompleteMenu()
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT), new Color(0, 0, 0, 180));

            Rectangle menuRect = new Rectangle(SCREEN_WIDTH / 2 - 200, SCREEN_HEIGHT / 2 - 150, 400, 300);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(40, 40, 40, 230));
            DrawRectangleBorder(menuRect, new Color(0, 150, 0), 2);

            string completeText = "LEVEL COMPLETE!";
            Vector2 textSize = _font.MeasureString(completeText);
            _spriteBatch.DrawString(_font, completeText,
                new Vector2((SCREEN_WIDTH - textSize.X) / 2, menuRect.Y + 30),
                Color.Green);

            string timeText = $"Time: {_gameController.GameTime.Minutes:00}:{_gameController.GameTime.Seconds:00}";
            Vector2 timeSize = _font.MeasureString(timeText);
            _spriteBatch.DrawString(_font, timeText,
                new Vector2((SCREEN_WIDTH - timeSize.X) / 2, menuRect.Y + 80),
                Color.White);

            string continueText = "Press C to continue to the next level";
            Vector2 continueSize = _font.MeasureString(continueText);
            _spriteBatch.DrawString(_font, continueText,
                new Vector2((SCREEN_WIDTH - continueSize.X) / 2, menuRect.Y + 130),
                Color.White);

            string restartText = "Press R to replay this level";
            Vector2 restartSize = _font.MeasureString(restartText);
            _spriteBatch.DrawString(_font, restartText,
                new Vector2((SCREEN_WIDTH - restartSize.X) / 2, menuRect.Y + 160),
                Color.White);

            string levelText = $"Level: {_gameController.CurrentLevel?.Name ?? "Unknown"}";
            _spriteBatch.DrawString(_font, levelText,
                new Vector2(menuRect.X + 30, menuRect.Y + 210),
                Color.LightGray);
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
                    ShowMessage(e.Message);
                    break;
            }
        }

        private void ShowMessage(string message)
        {
            Log("Game", $"Message: {message}", () => Debug("Game", $"ShowMessage: {message}"));
        }

        protected override void UnloadContent()
        {
            _renderCancellationTokenSource?.Cancel();
            _renderCancellationTokenSource?.Dispose();
            _pixelTexture?.Dispose();
            base.UnloadContent();
            Log("Game", "Content unloaded", () => Info("Game", "UnloadContent completed"));
        }
    }

    public class UIButton
    {
        public Rectangle Bounds { get; set; }
        public bool IsVisible { get; set; } = true;
        public Action OnClick { get; set; }
        public string Text { get; set; }
    }
}