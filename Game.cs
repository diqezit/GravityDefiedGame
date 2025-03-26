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
using System.Xml.Linq;

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
        private SpriteFont _titleFont; // Шрифт для заголовков
        private SpriteFont _unicodeFont; // Шрифт с поддержкой Unicode
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

            _font = Content.Load<SpriteFont>("Arial"); // all
            _titleFont = Content.Load<SpriteFont>("TitleFont"); // title
            _unicodeFont = Content.Load<SpriteFont>("Arial"); // symbols

            _renderer = new Renderer(_spriteBatch, GraphicsDevice, _gameController, _camera);
            _uiController = new UIController(this, _gameController, _spriteBatch, _font, _titleFont, _unicodeFont, _pixelTexture, SCREEN_WIDTH, SCREEN_HEIGHT);
            _gameController.LoadLevels();
            _uiController.ShowMainMenu();
            Log("Game", "Content loaded", () => Info("Game", "LoadContent completed"));
        }

        protected override void Update(GameTime gameTime)
        {
            try
            {
                _elapsedTime = Math.Min((float)gameTime.ElapsedGameTime.TotalSeconds, MAX_DELTA_TIME);

                _uiController.HandleInput(_gameController);

                if (_gameController.CurrentGameState == GameState.Playing)
                {
                    _gameController.Update(_elapsedTime, _renderCancellationTokenSource.Token);
                    UpdateCamera();
                }

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

                _spriteBatch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);
                _uiController.DrawUI(_gameController);
                _spriteBatch.End();

                base.Draw(gameTime);
            }
            catch (OperationCanceledException) { Debug("Game", "Draw cancelled"); }
            catch (Exception ex) { Error("Game", $"Draw error: {ex.Message}"); }
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
            base.UnloadContent();
            Log("Game", "Content unloaded", () => Info("Game", "UnloadContent completed"));
        }
    }

    public class UIController
    {
        private readonly Game _game;
        private readonly SpriteBatch _spriteBatch;
        private readonly SpriteFont _font;
        private readonly SpriteFont _titleFont;
        private readonly SpriteFont _unicodeFont;
        private readonly Texture2D _pixelTexture;
        private readonly int _screenWidth;
        private readonly int _screenHeight;
        private readonly Dictionary<string, UIButton> _buttons = new();
        private KeyboardState _previousKeyboardState;
        private MouseState _previousMouseState;
        private int _selectedBikeIndex = 0;
        private int _selectedLevelIndex = 0;
        private readonly List<Color> _availableBikeColors = new()
        {
            Color.Red, Color.Blue, Color.Green, Color.Yellow, Color.Purple, Color.Orange
        };
        private int _selectedColorIndex = 0;
        private float _buttonHoverScale = 1f;

        public UIController(
            Game game, 
            GameController gameController, 
            SpriteBatch spriteBatch, 
            SpriteFont font, 
            SpriteFont titleFont, 
            SpriteFont unicodeFont, 
            Texture2D pixelTexture, 
            int screenWidth, 
            int screenHeight)
        {
            _game = game;
            _gameController = gameController;
            _spriteBatch = spriteBatch;
            _font = font;
            _titleFont = titleFont;
            _unicodeFont = unicodeFont;
            _pixelTexture = pixelTexture;
            _screenWidth = screenWidth;
            _screenHeight = screenHeight;
            InitializeUI();
        }

        private void InitializeUI()
        {
            _buttons["restart"] = new UIButton
            {
                Bounds = new Rectangle(10, 10, 120, 50),
                Text = "Restart",
                IsVisible = false,
                OnClick = () => _gameController.RestartLevel(),
                HoverColor = Color.DarkGray
            };

            _buttons["continue"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - 60, _screenHeight / 2 + 20, 120, 50),
                Text = "Continue",
                IsVisible = false,
                OnClick = () => _gameController.StartNextLevel(),
                HoverColor = Color.DarkGreen
            };

            _buttons["pause"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth - 130, 10, 120, 50),
                Text = "Pause",
                IsVisible = false,
                OnClick = () => _gameController.PauseGame(),
                HoverColor = Color.DarkBlue
            };

            _buttons["play"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - 150, 250, 300, 60),
                Text = "Play Game",
                IsVisible = true,
                OnClick = () => ShowBikeSelection(),
                HoverColor = Color.DarkGreen
            };

            _buttons["exit"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - 150, 320, 300, 60),
                Text = "Exit",
                IsVisible = true,
                OnClick = () => _game.Exit(),
                HoverColor = Color.DarkRed
            };

            _buttons["selectBike"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - 100, 450, 200, 50),
                Text = "Select Bike",
                IsVisible = false,
                OnClick = () => ShowLevelSelection(),
                HoverColor = Color.DarkOrange
            };

            _buttons["startLevel"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - 100, 450, 200, 50),
                Text = "Start Level",
                IsVisible = false,
                OnClick = () => StartGame(_selectedLevelIndex + 1),
                HoverColor = Color.DarkCyan
            };

            Log("UIController", "UI initialized", () => Info("UIController", "InitializeUI completed"));
        }

        public void HandleInput(GameController gameController)
        {
            var keyboardState = Keyboard.GetState();
            var mouseState = Mouse.GetState();
            _gameController = gameController;

            if (gameController.CurrentGameState == GameState.MainMenu ||
                gameController.CurrentGameState == GameState.BikeSelection ||
                gameController.CurrentGameState == GameState.LevelSelection)
            {
                HandleMenuInput(keyboardState);
            }
            else
            {
                gameController.HandleInput(keyboardState, _previousKeyboardState);
            }

            foreach (var button in _buttons.Values)
            {
                if (button.IsVisible && button.Bounds.Contains(mouseState.X, mouseState.Y))
                {
                    _buttonHoverScale = 1.05f;
                    if (mouseState.LeftButton == ButtonState.Pressed && _previousMouseState.LeftButton == ButtonState.Released)
                        button.OnClick?.Invoke();
                }
                else
                {
                    _buttonHoverScale = 1f;
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

        public void DrawUI(GameController gameController)
        {
            _buttons["restart"].IsVisible = gameController.CurrentGameState == GameState.GameOver;
            _buttons["continue"].IsVisible = gameController.CurrentGameState == GameState.LevelComplete;
            _buttons["pause"].IsVisible = gameController.CurrentGameState == GameState.Playing;
            _buttons["play"].IsVisible = gameController.CurrentGameState == GameState.MainMenu;
            _buttons["exit"].IsVisible = gameController.CurrentGameState == GameState.MainMenu;
            _buttons["selectBike"].IsVisible = gameController.CurrentGameState == GameState.BikeSelection;
            _buttons["startLevel"].IsVisible = gameController.CurrentGameState == GameState.LevelSelection;

            switch (gameController.CurrentGameState)
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
                    DrawInfoPanel(gameController);
                    break;
                case GameState.Paused:
                    DrawPauseMenu(gameController);
                    break;
                case GameState.GameOver:
                    DrawGameOverMenu(gameController);
                    break;
                case GameState.LevelComplete:
                    DrawLevelCompleteMenu(gameController);
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
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), new Color(0, 0, 0, 150));

            Rectangle menuRect = new Rectangle(_screenWidth / 2 - 300, _screenHeight / 2 - 250, 600, 500);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(30, 30, 30, 240));
            DrawRectangleBorder(menuRect, Color.Goldenrod, 3);

            string titleText = "GRAVITY DEFIED";
            Vector2 titleSize = _titleFont.MeasureString(titleText);
            _spriteBatch.DrawString(_titleFont, titleText,
                new Vector2((_screenWidth - titleSize.X) / 2, menuRect.Y + 50),
                Color.Gold);

            string subtitleText = "A Motorcycle Physics Adventure";
            Vector2 subtitleSize = _font.MeasureString(subtitleText);
            _spriteBatch.DrawString(_font, subtitleText,
                new Vector2((_screenWidth - subtitleSize.X) / 2, menuRect.Y + 120),
                Color.LightGoldenrodYellow);
        }

        private void DrawBikeSelectionMenu()
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), new Color(0, 0, 0, 150));

            Rectangle menuRect = new Rectangle(_screenWidth / 2 - 350, _screenHeight / 2 - 250, 700, 500);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(30, 30, 30, 240));
            DrawRectangleBorder(menuRect, Color.Cyan, 3);

            string titleText = "SELECT YOUR BIKE";
            Vector2 titleSize = _titleFont.MeasureString(titleText);
            _spriteBatch.DrawString(_titleFont, titleText,
                new Vector2((_screenWidth - titleSize.X) / 2, menuRect.Y + 30),
                Color.Cyan);

            var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();
            string bikeTypeText = availableBikes[_selectedBikeIndex].ToString();
            Vector2 bikeTypeSize = _font.MeasureString(bikeTypeText);
            _spriteBatch.DrawString(_font, bikeTypeText,
                new Vector2((_screenWidth - bikeTypeSize.X) / 2, menuRect.Y + 100),
                _availableBikeColors[_selectedColorIndex]);

            string instructionText = "← Left/Right: Change Bike | ↑/↓: Change Color | Enter: Confirm";
            Vector2 instructionSize = _font.MeasureString(instructionText);
            _spriteBatch.DrawString(_font, instructionText,
                new Vector2((_screenWidth - instructionSize.X) / 2, menuRect.Y + 400),
                Color.LightCyan);

            DrawBikePreview(menuRect);
        }

        private void DrawBikePreview(Rectangle menuRect)
        {
            int previewX = menuRect.X + 200;
            int previewY = menuRect.Y + 150;
            int previewWidth = 300;
            int previewHeight = 150;

            Rectangle previewRect = new Rectangle(previewX, previewY, previewWidth, previewHeight);
            _spriteBatch.Draw(_pixelTexture, previewRect, new Color(20, 20, 20, 200));
            DrawRectangleBorder(previewRect, Color.LightGray, 2);

            string previewText = "Bike Preview";
            Vector2 previewTextSize = _font.MeasureString(previewText);
            _spriteBatch.DrawString(_font, previewText,
                new Vector2(previewX + (previewWidth - previewTextSize.X) / 2, previewY + 10),
                Color.LightGray);

            string statsText = GetBikeStats(Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList()[_selectedBikeIndex]);
            Vector2 statsSize = _unicodeFont.MeasureString(statsText);
            _spriteBatch.DrawString(_unicodeFont, statsText,
                new Vector2(previewX + 20, previewY + 50),
                Color.White);
        }

        private string GetBikeStats(BikeType bikeType)
        {
            switch (bikeType)
            {
                case BikeType.Standard:
                    return "Speed: ***--\nHandling: ***--\nStability: ***--";
                case BikeType.Sport:
                    return "Speed: *****\nHandling: **---\nStability: **---";
                case BikeType.OffRoad:
                    return "Speed: **---\nHandling: ****-\nStability: *****";
                default:
                    return "Speed: ***--\nHandling: ***--\nStability: ***--";
            }
        }

        private void DrawLevelSelectionMenu()
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), new Color(0, 0, 0, 150));

            Rectangle menuRect = new Rectangle(_screenWidth / 2 - 350, _screenHeight / 2 - 250, 700, 500);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(30, 30, 30, 240));
            DrawRectangleBorder(menuRect, Color.Magenta, 3);

            string titleText = "SELECT LEVEL";
            Vector2 titleSize = _titleFont.MeasureString(titleText);
            _spriteBatch.DrawString(_titleFont, titleText,
                new Vector2((_screenWidth - titleSize.X) / 2, menuRect.Y + 30),
                Color.Magenta);

            string levelText = $"Level {_selectedLevelIndex + 1}: {_gameController.Levels[_selectedLevelIndex].Name}";
            Vector2 levelSize = _font.MeasureString(levelText);
            _spriteBatch.DrawString(_font, levelText,
                new Vector2((_screenWidth - levelSize.X) / 2, menuRect.Y + 100),
                Color.Yellow);

            string difficultyText = $"Difficulty: {GetLevelDifficulty(_selectedLevelIndex + 1)}";
            Vector2 difficultySize = _unicodeFont.MeasureString(difficultyText);
            _spriteBatch.DrawString(_unicodeFont, difficultyText,
                new Vector2((_screenWidth - difficultySize.X) / 2, menuRect.Y + 150),
                Color.White);

            string instructionText = "← Left/Right: Change Level | Enter: Start";
            Vector2 instructionSize = _font.MeasureString(instructionText);
            _spriteBatch.DrawString(_font, instructionText,
                new Vector2((_screenWidth - instructionSize.X) / 2, menuRect.Y + 400),
                Color.LightPink);
        }

        private string GetLevelDifficulty(int levelId)
        {
            if (levelId <= 5) return "Easy ★☆☆";
            if (levelId <= 15) return "Medium ★★☆";
            return "Hard ★★★";
        }

        private void DrawButton(UIButton button)
        {
            var mouseState = Mouse.GetState();
            Color buttonColor = button.Bounds.Contains(mouseState.X, mouseState.Y) ? button.HoverColor : new Color(50, 50, 50);
            Rectangle scaledBounds = new Rectangle(
                (int)(button.Bounds.X - button.Bounds.Width * (_buttonHoverScale - 1) / 2),
                (int)(button.Bounds.Y - button.Bounds.Height * (_buttonHoverScale - 1) / 2),
                (int)(button.Bounds.Width * _buttonHoverScale),
                (int)(button.Bounds.Height * _buttonHoverScale)
            );

            _spriteBatch.Draw(_pixelTexture, scaledBounds, buttonColor);
            DrawRectangleBorder(scaledBounds, Color.White, 2);

            if (!string.IsNullOrEmpty(button.Text))
            {
                Vector2 textSize = _font.MeasureString(button.Text);
                Vector2 textPosition = new Vector2(
                    scaledBounds.X + (scaledBounds.Width - textSize.X) / 2,
                    scaledBounds.Y + (scaledBounds.Height - textSize.Y) / 2
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

        private void DrawInfoPanel(GameController gameController)
        {
            string levelName = gameController.CurrentLevel?.Name ?? "Level";
            string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
            string directionText = gameController.Motorcycle.Direction == 1 ? "Forward" : "Backward";

            Rectangle infoPanelRect = new Rectangle(10, 10, 350, 100);
            _spriteBatch.Draw(_pixelTexture, infoPanelRect, new Color(0, 0, 0, 150));
            DrawRectangleBorder(infoPanelRect, Color.LightBlue, 2);

            _spriteBatch.DrawString(_font, $"Level: {levelName}", new Vector2(20, 20), Color.White);
            _spriteBatch.DrawString(_font, timeText, new Vector2(20, 45), Color.White);
            _spriteBatch.DrawString(_font, $"Direction: {directionText}", new Vector2(20, 70), Color.White);

            string controlsText = "W: Forward | S: Backward | Space: Brake | A/D: Lean | R: Restart | ESC: Pause";
            Vector2 controlsSize = _font.MeasureString(controlsText);
            _spriteBatch.DrawString(_font, controlsText,
                new Vector2((_screenWidth - controlsSize.X) / 2, _screenHeight - 40),
                Color.LightBlue);
        }

        private void DrawPauseMenu(GameController gameController)
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), new Color(0, 0, 0, 200));

            Rectangle menuRect = new Rectangle(_screenWidth / 2 - 250, _screenHeight / 2 - 200, 500, 400);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(30, 30, 30, 240));
            DrawRectangleBorder(menuRect, Color.LightBlue, 3);

            string pauseText = "GAME PAUSED";
            Vector2 textSize = _titleFont.MeasureString(pauseText);
            _spriteBatch.DrawString(_titleFont, pauseText,
                new Vector2((_screenWidth - textSize.X) / 2, menuRect.Y + 30),
                Color.LightBlue);

            string resumeText = "Press ESC to Resume";
            Vector2 resumeSize = _font.MeasureString(resumeText);
            _spriteBatch.DrawString(_font, resumeText,
                new Vector2((_screenWidth - resumeSize.X) / 2, menuRect.Y + 120),
                Color.White);

            string restartText = "Press R to Restart";
            Vector2 restartSize = _font.MeasureString(restartText);
            _spriteBatch.DrawString(_font, restartText,
                new Vector2((_screenWidth - restartSize.X) / 2, menuRect.Y + 160),
                Color.White);

            string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
            string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
            _spriteBatch.DrawString(_font, levelText,
                new Vector2(menuRect.X + 30, menuRect.Y + 300),
                Color.LightGray);
            _spriteBatch.DrawString(_font, timeText,
                new Vector2(menuRect.X + 30, menuRect.Y + 330),
                Color.LightGray);
        }

        private void DrawGameOverMenu(GameController gameController)
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), new Color(0, 0, 0, 200));

            Rectangle menuRect = new Rectangle(_screenWidth / 2 - 250, _screenHeight / 2 - 200, 500, 400);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(30, 30, 30, 240));
            DrawRectangleBorder(menuRect, Color.Red, 3);

            string gameOverText = "GAME OVER";
            Vector2 textSize = _titleFont.MeasureString(gameOverText);
            _spriteBatch.DrawString(_titleFont, gameOverText,
                new Vector2((_screenWidth - textSize.X) / 2, menuRect.Y + 30),
                Color.Red);

            string restartText = "Press R to Restart";
            Vector2 restartSize = _font.MeasureString(restartText);
            _spriteBatch.DrawString(_font, restartText,
                new Vector2((_screenWidth - restartSize.X) / 2, menuRect.Y + 120),
                Color.White);

            string mainMenuText = "Press ESC for Main Menu";
            Vector2 mainMenuSize = _font.MeasureString(mainMenuText);
            _spriteBatch.DrawString(_font, mainMenuText,
                new Vector2((_screenWidth - mainMenuSize.X) / 2, menuRect.Y + 160),
                Color.White);

            string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
            string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
            _spriteBatch.DrawString(_font, levelText,
                new Vector2(menuRect.X + 30, menuRect.Y + 300),
                Color.LightGray);
            _spriteBatch.DrawString(_font, timeText,
                new Vector2(menuRect.X + 30, menuRect.Y + 330),
                Color.LightGray);
        }

        private void DrawLevelCompleteMenu(GameController gameController)
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), new Color(0, 0, 0, 200));

            Rectangle menuRect = new Rectangle(_screenWidth / 2 - 250, _screenHeight / 2 - 200, 500, 400);
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(30, 30, 30, 240));
            DrawRectangleBorder(menuRect, Color.LimeGreen, 3);

            string completeText = "LEVEL COMPLETE!";
            Vector2 textSize = _titleFont.MeasureString(completeText);
            _spriteBatch.DrawString(_titleFont, completeText,
                new Vector2((_screenWidth - textSize.X) / 2, menuRect.Y + 30),
                Color.LimeGreen);

            string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
            Vector2 timeSize = _font.MeasureString(timeText);
            _spriteBatch.DrawString(_font, timeText,
                new Vector2((_screenWidth - timeSize.X) / 2, menuRect.Y + 100),
                Color.White);

            string continueText = "Press C to Continue";
            Vector2 continueSize = _font.MeasureString(continueText);
            _spriteBatch.DrawString(_font, continueText,
                new Vector2((_screenWidth - continueSize.X) / 2, menuRect.Y + 140),
                Color.White);

            string restartText = "Press R to Replay";
            Vector2 restartSize = _font.MeasureString(restartText);
            _spriteBatch.DrawString(_font, restartText,
                new Vector2((_screenWidth - restartSize.X) / 2, menuRect.Y + 180),
                Color.White);

            string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
            _spriteBatch.DrawString(_font, levelText,
                new Vector2(menuRect.X + 30, menuRect.Y + 330),
                Color.LightGray);
        }

        private GameController _gameController;

        public void ShowMainMenu()
        {
            _gameController.EnterMainMenu();
            Log("UIController", "Main menu displayed", () => Info("UIController", "ShowMainMenu completed"));
        }

        private void ShowBikeSelection()
        {
            _gameController.EnterBikeSelection();
            var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();
            _gameController.SetBikeType(availableBikes[_selectedBikeIndex]);
            _gameController.SetBikeColor(_availableBikeColors[_selectedColorIndex]);
            Log("UIController", "Bike selection menu displayed", () => Info("UIController", "ShowBikeSelection completed"));
        }

        private void ShowLevelSelection()
        {
            _gameController.EnterLevelSelection();
            _gameController.SelectLevel(_selectedLevelIndex + 1);
            Log("UIController", "Level selection menu displayed", () => Info("UIController", "ShowLevelSelection completed"));
        }

        private void StartGame(int levelId)
        {
            _gameController.StartLevel(levelId);
            Log("UIController", $"Level {levelId} started", () => Info("UIController", "StartGame completed"));
        }

        public void ShowMessage(string message)
        {
            Log("UIController", $"Message: {message}", () => Debug("UIController", $"ShowMessage: {message}"));
        }
    }

    public class UIButton
    {
        public Rectangle Bounds { get; set; }
        public bool IsVisible { get; set; } = true;
        public Action OnClick { get; set; }
        public string Text { get; set; }
        public Color HoverColor { get; set; } = Color.Gray;
    }
}