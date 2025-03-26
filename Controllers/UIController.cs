using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Linq;
using FontStashSharp;
using GravityDefiedGame.Controllers;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame
{
    public class UIController
    {
        private readonly Game _game;
        private readonly SpriteBatch _spriteBatch;
        private readonly DynamicSpriteFont _font;
        private readonly DynamicSpriteFont _titleFont;
        private readonly DynamicSpriteFont _unicodeFont;
        private readonly Texture2D _pixelTexture;
        private readonly Texture2D _gradientTexture;
        private readonly Texture2D _starFilledTexture;
        private readonly Texture2D _starEmptyTexture;
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
        private float _backgroundAnimationOffset = 0f; // Для анимации фона
        private const float BUTTON_SCALE_SPEED = 10f;
        private const float BACKGROUND_ANIMATION_SPEED = 20f;
        private GameController _gameController;
        private string _tooltipText = ""; // Текст подсказки
        private Vector2 _tooltipPosition;

        public UIController(
            Game game,
            GameController gameController,
            SpriteBatch spriteBatch,
            DynamicSpriteFont font,
            DynamicSpriteFont titleFont,
            DynamicSpriteFont unicodeFont,
            Texture2D pixelTexture,
            Texture2D gradientTexture,
            Texture2D starFilledTexture,
            Texture2D starEmptyTexture,
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
            _gradientTexture = gradientTexture;
            _starFilledTexture = starFilledTexture;
            _starEmptyTexture = starEmptyTexture;
            _screenWidth = screenWidth;
            _screenHeight = screenHeight;
            InitializeUI();
        }

        private void InitializeUI()
        {
            // Адаптивные размеры кнопок
            int buttonWidth = (int)(_screenWidth * 0.15f);
            int buttonHeight = (int)(_screenHeight * 0.07f);

            _buttons["restart"] = new UIButton
            {
                Bounds = new Rectangle(10, 10, buttonWidth, buttonHeight),
                Text = "Restart",
                IsVisible = false,
                OnClick = () => _gameController.RestartLevel(),
                BaseColor = new Color(50, 50, 50),
                HoverColor = Color.DarkGray,
                Tooltip = "Restart the current level"
            };

            _buttons["continue"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - buttonWidth / 2, _screenHeight / 2 + 20, buttonWidth, buttonHeight),
                Text = "Continue",
                IsVisible = false,
                OnClick = () => _gameController.StartNextLevel(),
                BaseColor = new Color(0, 80, 0),
                HoverColor = Color.DarkGreen,
                Tooltip = "Proceed to the next level"
            };

            _buttons["pause"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth - buttonWidth - 10, 10, buttonWidth, buttonHeight),
                Text = "Pause",
                IsVisible = false,
                OnClick = () => _gameController.PauseGame(),
                BaseColor = new Color(0, 50, 80),
                HoverColor = Color.DarkBlue,
                Tooltip = "Pause the game"
            };

            _buttons["play"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - buttonWidth, (int)(_screenHeight * 0.4f), buttonWidth * 2, buttonHeight),
                Text = "Play Game",
                IsVisible = true,
                OnClick = () => ShowBikeSelection(),
                BaseColor = new Color(0, 80, 0),
                HoverColor = Color.DarkGreen,
                Tooltip = "Start playing"
            };

            _buttons["exit"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - buttonWidth, (int)(_screenHeight * 0.5f), buttonWidth * 2, buttonHeight),
                Text = "Exit",
                IsVisible = true,
                OnClick = () => _game.Exit(),
                BaseColor = new Color(80, 0, 0),
                HoverColor = Color.DarkRed,
                Tooltip = "Exit the game"
            };

            _buttons["selectBike"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - buttonWidth / 2, (int)(_screenHeight * 0.7f), buttonWidth, buttonHeight),
                Text = "Select Bike",
                IsVisible = false,
                OnClick = () => ShowLevelSelection(),
                BaseColor = new Color(80, 50, 0),
                HoverColor = Color.DarkOrange,
                Tooltip = "Confirm bike selection"
            };

            _buttons["startLevel"] = new UIButton
            {
                Bounds = new Rectangle(_screenWidth / 2 - buttonWidth / 2, (int)(_screenHeight * 0.7f), buttonWidth, buttonHeight),
                Text = "Start Level",
                IsVisible = false,
                OnClick = () => StartGame(_selectedLevelIndex + 1),
                BaseColor = new Color(0, 50, 80),
                HoverColor = Color.DarkCyan,
                Tooltip = "Start the selected level"
            };

            Log("UIController", "UI инициализирован", () => Info("UIController", "InitializeUI завершен"));
        }

        public void HandleInput(GameController gameController, float elapsedTime)
        {
            var keyboardState = Keyboard.GetState();
            var mouseState = Mouse.GetState();
            _gameController = gameController;

            _backgroundAnimationOffset += BACKGROUND_ANIMATION_SPEED * elapsedTime; // Анимация фона
            if (_backgroundAnimationOffset > _screenWidth) _backgroundAnimationOffset -= _screenWidth;

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

            _tooltipText = ""; // Сброс подсказки
            foreach (var button in _buttons.Values)
            {
                if (button.IsVisible)
                {
                    bool isHovered = button.Bounds.Contains(mouseState.X, mouseState.Y);
                    _buttonHoverScale = MathHelper.Lerp(_buttonHoverScale, isHovered ? 1.1f : 1f, BUTTON_SCALE_SPEED * elapsedTime);
                    if (isHovered)
                    {
                        _tooltipText = button.Tooltip;
                        _tooltipPosition = new Vector2(mouseState.X + 10, mouseState.Y + 10);
                    }
                    if (isHovered && mouseState.LeftButton == ButtonState.Pressed && _previousMouseState.LeftButton == ButtonState.Released)
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

            if (!string.IsNullOrEmpty(_tooltipText))
                DrawTooltip();
        }

        private void DrawMainMenu()
        {
            DrawAnimatedBackground();
            Rectangle menuRect = new Rectangle(_screenWidth / 2 - (int)(_screenWidth * 0.3f), _screenHeight / 2 - (int)(_screenHeight * 0.35f), (int)(_screenWidth * 0.6f), (int)(_screenHeight * 0.7f));
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(20, 20, 20, 220));
            DrawRectangleBorder(menuRect, Color.Goldenrod, 3);

            string titleText = "GRAVITY DEFIED";
            Vector2 titleSize = _titleFont.MeasureString(titleText);
            _titleFont.DrawText(_spriteBatch, titleText, new Vector2((_screenWidth - titleSize.X) / 2 + 2, menuRect.Y + 50 + 2), Color.Black * 0.5f); // Тень
            _titleFont.DrawText(_spriteBatch, titleText, new Vector2((_screenWidth - titleSize.X) / 2, menuRect.Y + 50), Color.Gold);

            string subtitleText = "A Motorcycle Physics Adventure";
            Vector2 subtitleSize = _font.MeasureString(subtitleText);
            _font.DrawText(_spriteBatch, subtitleText, new Vector2((_screenWidth - subtitleSize.X) / 2 + 1, menuRect.Y + 120 + 1), Color.Black * 0.5f); // Тень
            _font.DrawText(_spriteBatch, subtitleText, new Vector2((_screenWidth - subtitleSize.X) / 2, menuRect.Y + 120), Color.LightGoldenrodYellow);
        }

        private void DrawBikeSelectionMenu()
        {
            DrawAnimatedBackground();
            Rectangle menuRect = new Rectangle(_screenWidth / 2 - (int)(_screenWidth * 0.35f), _screenHeight / 2 - (int)(_screenHeight * 0.35f), (int)(_screenWidth * 0.7f), (int)(_screenHeight * 0.7f));
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(20, 20, 20, 220));
            DrawRectangleBorder(menuRect, Color.Cyan, 3);

            string titleText = "SELECT YOUR BIKE";
            Vector2 titleSize = _titleFont.MeasureString(titleText);
            _titleFont.DrawText(_spriteBatch, titleText, new Vector2((_screenWidth - titleSize.X) / 2 + 2, menuRect.Y + 30 + 2), Color.Black * 0.5f);
            _titleFont.DrawText(_spriteBatch, titleText, new Vector2((_screenWidth - titleSize.X) / 2, menuRect.Y + 30), Color.Cyan);

            var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();
            string bikeTypeText = availableBikes[_selectedBikeIndex].ToString();
            Vector2 bikeTypeSize = _font.MeasureString(bikeTypeText);
            _font.DrawText(_spriteBatch, bikeTypeText, new Vector2((_screenWidth - bikeTypeSize.X) / 2 + 1, menuRect.Y + 100 + 1), Color.Black * 0.5f);
            _font.DrawText(_spriteBatch, bikeTypeText, new Vector2((_screenWidth - bikeTypeSize.X) / 2, menuRect.Y + 100), _availableBikeColors[_selectedColorIndex]);

            string instructionText = "← Left/Right: Change Bike | ↑/↓: Change Color | Enter: Confirm";
            Vector2 instructionSize = _font.MeasureString(instructionText);
            _font.DrawText(_spriteBatch, instructionText, new Vector2((_screenWidth - instructionSize.X) / 2, menuRect.Y + (int)(menuRect.Height * 0.8f)), Color.LightCyan);

            DrawBikePreview(menuRect);
        }

        private void DrawBikePreview(Rectangle menuRect)
        {
            int previewWidth = (int)(menuRect.Width * 0.4f);
            int previewHeight = (int)(menuRect.Height * 0.3f);
            int previewX = menuRect.X + (menuRect.Width - previewWidth) / 2;
            int previewY = menuRect.Y + (int)(menuRect.Height * 0.3f);

            Rectangle previewRect = new Rectangle(previewX, previewY, previewWidth, previewHeight);
            _spriteBatch.Draw(_pixelTexture, previewRect, new Color(30, 30, 30, 200));
            DrawRectangleBorder(previewRect, Color.LightGray, 2);

            string previewText = "Bike Preview";
            Vector2 previewTextSize = _font.MeasureString(previewText);
            _font.DrawText(_spriteBatch, previewText, new Vector2(previewX + (previewWidth - previewTextSize.X) / 2, previewY + 10), Color.LightGray);

            var bikeType = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList()[_selectedBikeIndex];
            string statsText = GetBikeStats(bikeType);
            Vector2 statsPos = new Vector2(previewX + 20, previewY + 40);

            string[] statsLines = statsText.Split('\n');
            for (int i = 0; i < statsLines.Length; i++)
            {
                string lineLabel = statsLines[i].Split(':')[0] + ": ";
                Vector2 lineSize = _font.MeasureString(lineLabel);
                _font.DrawText(_spriteBatch, lineLabel, statsPos + new Vector2(0, i * 30), Color.Yellow);

                int filledStars = statsLines[i].Count(c => c == '★');
                for (int j = 0; j < 5; j++)
                {
                    Texture2D starTexture = j < filledStars ? _starFilledTexture : _starEmptyTexture;
                    _spriteBatch.Draw(starTexture, statsPos + new Vector2(lineSize.X + j * 20, i * 30), null, Color.White, 0f, Vector2.Zero, 0.75f, SpriteEffects.None, 0f);
                }
            }
        }

        private string GetBikeStats(BikeType bikeType)
        {
            switch (bikeType)
            {
                case BikeType.Standard:
                    return "Speed: ★★★☆☆\nHandling: ★★★☆☆\nStability: ★★★☆☆";
                case BikeType.Sport:
                    return "Speed: ★★★★★\nHandling: ★★☆☆☆\nStability: ★★☆☆☆";
                case BikeType.OffRoad:
                    return "Speed: ★★☆☆☆\nHandling: ★★★★☆\nStability: ★★★★★";
                default:
                    return "Speed: ★★★☆☆\nHandling: ★★★☆☆\nStability: ★★★☆☆";
            }
        }

        private void DrawLevelSelectionMenu()
        {
            DrawAnimatedBackground();
            Rectangle menuRect = new Rectangle(_screenWidth / 2 - (int)(_screenWidth * 0.35f), _screenHeight / 2 - (int)(_screenHeight * 0.35f), (int)(_screenWidth * 0.7f), (int)(_screenHeight * 0.7f));
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(20, 20, 20, 220));
            DrawRectangleBorder(menuRect, Color.Magenta, 3);

            string titleText = "SELECT LEVEL";
            Vector2 titleSize = _titleFont.MeasureString(titleText);
            _titleFont.DrawText(_spriteBatch, titleText, new Vector2((_screenWidth - titleSize.X) / 2 + 2, menuRect.Y + 30 + 2), Color.Black * 0.5f);
            _titleFont.DrawText(_spriteBatch, titleText, new Vector2((_screenWidth - titleSize.X) / 2, menuRect.Y + 30), Color.Magenta);

            string levelText = $"Level {_selectedLevelIndex + 1}: {_gameController.Levels[_selectedLevelIndex].Name}";
            Vector2 levelSize = _font.MeasureString(levelText);
            _font.DrawText(_spriteBatch, levelText, new Vector2((_screenWidth - levelSize.X) / 2 + 1, menuRect.Y + 100 + 1), Color.Black * 0.5f);
            _font.DrawText(_spriteBatch, levelText, new Vector2((_screenWidth - levelSize.X) / 2, menuRect.Y + 100), Color.Yellow);

            string difficultyLabel = "Difficulty: ";
            Vector2 difficultyPos = new Vector2((_screenWidth - _font.MeasureString(difficultyLabel).X) / 2, menuRect.Y + 150);
            _font.DrawText(_spriteBatch, difficultyLabel, difficultyPos, Color.Yellow);

            string difficultyStars = GetLevelDifficulty(_selectedLevelIndex + 1).Split(' ')[1];
            int filledStars = difficultyStars.Count(c => c == '★');
            for (int j = 0; j < 3; j++)
            {
                Texture2D starTexture = j < filledStars ? _starFilledTexture : _starEmptyTexture;
                _spriteBatch.Draw(starTexture, difficultyPos + new Vector2(_font.MeasureString(difficultyLabel).X + j * 20, 0), null, Color.White, 0f, Vector2.Zero, 0.75f, SpriteEffects.None, 0f);
            }

            string instructionText = "← Left/Right: Change Level | Enter: Start";
            Vector2 instructionSize = _font.MeasureString(instructionText);
            _font.DrawText(_spriteBatch, instructionText, new Vector2((_screenWidth - instructionSize.X) / 2, menuRect.Y + (int)(menuRect.Height * 0.8f)), Color.LightPink);
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
            bool isHovered = button.Bounds.Contains(mouseState.X, mouseState.Y);
            Color buttonColor = isHovered ? button.HoverColor : button.BaseColor;
            Rectangle scaledBounds = new Rectangle(
                (int)(button.Bounds.X - button.Bounds.Width * (_buttonHoverScale - 1) / 2),
                (int)(button.Bounds.Y - button.Bounds.Height * (_buttonHoverScale - 1) / 2),
                (int)(button.Bounds.Width * _buttonHoverScale),
                (int)(button.Bounds.Height * _buttonHoverScale)
            );

            // Тень кнопки
            _spriteBatch.Draw(_pixelTexture, new Rectangle(scaledBounds.X + 4, scaledBounds.Y + 4, scaledBounds.Width, scaledBounds.Height), Color.Black * 0.3f);
            _spriteBatch.Draw(_pixelTexture, scaledBounds, buttonColor);
            DrawRectangleBorder(scaledBounds, Color.White, 2);

            if (!string.IsNullOrEmpty(button.Text))
            {
                Vector2 textSize = _font.MeasureString(button.Text);
                Vector2 textPosition = new Vector2(
                    scaledBounds.X + (scaledBounds.Width - textSize.X) / 2,
                    scaledBounds.Y + (scaledBounds.Height - textSize.Y) / 2
                );
                _font.DrawText(_spriteBatch, button.Text, textPosition + new Vector2(1, 1), Color.Black * 0.5f); // Тень текста
                _font.DrawText(_spriteBatch, button.Text, textPosition, Color.White);
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

            Rectangle infoPanelRect = new Rectangle(10, 10, (int)(_screenWidth * 0.25f), (int)(_screenHeight * 0.15f));
            _spriteBatch.Draw(_pixelTexture, infoPanelRect, new Color(0, 0, 0, 150));
            DrawRectangleBorder(infoPanelRect, Color.LightBlue, 2);

            _font.DrawText(_spriteBatch, $"Level: {levelName}", new Vector2(20, 20), Color.White);
            _font.DrawText(_spriteBatch, timeText, new Vector2(20, 45), Color.White);
            _font.DrawText(_spriteBatch, $"Direction: {directionText}", new Vector2(20, 70), Color.White);

            string controlsText = "W: Forward | S: Backward | Space: Brake | A/D: Lean | R: Restart | ESC: Pause";
            Vector2 controlsSize = _font.MeasureString(controlsText);
            _font.DrawText(_spriteBatch, controlsText, new Vector2((_screenWidth - controlsSize.X) / 2, _screenHeight - 40), Color.LightBlue);
        }

        private void DrawPauseMenu(GameController gameController)
        {
            DrawAnimatedBackground();
            Rectangle menuRect = new Rectangle(_screenWidth / 2 - (int)(_screenWidth * 0.25f), _screenHeight / 2 - (int)(_screenHeight * 0.3f), (int)(_screenWidth * 0.5f), (int)(_screenHeight * 0.6f));
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(20, 20, 20, 220));
            DrawRectangleBorder(menuRect, Color.LightBlue, 3);

            string pauseText = "GAME PAUSED";
            Vector2 textSize = _titleFont.MeasureString(pauseText);
            _titleFont.DrawText(_spriteBatch, pauseText, new Vector2((_screenWidth - textSize.X) / 2 + 2, menuRect.Y + 30 + 2), Color.Black * 0.5f);
            _titleFont.DrawText(_spriteBatch, pauseText, new Vector2((_screenWidth - textSize.X) / 2, menuRect.Y + 30), Color.LightBlue);

            string resumeText = "Press ESC to Resume";
            Vector2 resumeSize = _font.MeasureString(resumeText);
            _font.DrawText(_spriteBatch, resumeText, new Vector2((_screenWidth - resumeSize.X) / 2, menuRect.Y + 120), Color.White);

            string restartText = "Press R to Restart";
            Vector2 restartSize = _font.MeasureString(restartText);
            _font.DrawText(_spriteBatch, restartText, new Vector2((_screenWidth - restartSize.X) / 2, menuRect.Y + 160), Color.White);

            string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
            string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
            _font.DrawText(_spriteBatch, levelText, new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.75f)), Color.LightGray);
            _font.DrawText(_spriteBatch, timeText, new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.85f)), Color.LightGray);
        }

        private void DrawGameOverMenu(GameController gameController)
        {
            DrawAnimatedBackground();
            Rectangle menuRect = new Rectangle(_screenWidth / 2 - (int)(_screenWidth * 0.25f), _screenHeight / 2 - (int)(_screenHeight * 0.3f), (int)(_screenWidth * 0.5f), (int)(_screenHeight * 0.6f));
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(20, 20, 20, 220));
            DrawRectangleBorder(menuRect, Color.Red, 3);

            string gameOverText = "GAME OVER";
            Vector2 textSize = _titleFont.MeasureString(gameOverText);
            _titleFont.DrawText(_spriteBatch, gameOverText, new Vector2((_screenWidth - textSize.X) / 2 + 2, menuRect.Y + 30 + 2), Color.Black * 0.5f);
            _titleFont.DrawText(_spriteBatch, gameOverText, new Vector2((_screenWidth - textSize.X) / 2, menuRect.Y + 30), Color.Red);

            string restartText = "Press R to Restart";
            Vector2 restartSize = _font.MeasureString(restartText);
            _font.DrawText(_spriteBatch, restartText, new Vector2((_screenWidth - restartSize.X) / 2, menuRect.Y + 120), Color.White);

            string mainMenuText = "Press ESC for Main Menu";
            Vector2 mainMenuSize = _font.MeasureString(mainMenuText);
            _font.DrawText(_spriteBatch, mainMenuText, new Vector2((_screenWidth - mainMenuSize.X) / 2, menuRect.Y + 160), Color.White);

            string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
            string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
            _font.DrawText(_spriteBatch, levelText, new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.75f)), Color.LightGray);
            _font.DrawText(_spriteBatch, timeText, new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.85f)), Color.LightGray);
        }

        private void DrawLevelCompleteMenu(GameController gameController)
        {
            DrawAnimatedBackground();
            Rectangle menuRect = new Rectangle(_screenWidth / 2 - (int)(_screenWidth * 0.25f), _screenHeight / 2 - (int)(_screenHeight * 0.3f), (int)(_screenWidth * 0.5f), (int)(_screenHeight * 0.6f));
            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(20, 20, 20, 220));
            DrawRectangleBorder(menuRect, Color.LimeGreen, 3);

            string completeText = "LEVEL COMPLETE!";
            Vector2 textSize = _titleFont.MeasureString(completeText);
            _titleFont.DrawText(_spriteBatch, completeText, new Vector2((_screenWidth - textSize.X) / 2 + 2, menuRect.Y + 30 + 2), Color.Black * 0.5f);
            _titleFont.DrawText(_spriteBatch, completeText, new Vector2((_screenWidth - textSize.X) / 2, menuRect.Y + 30), Color.LimeGreen);

            string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
            Vector2 timeSize = _font.MeasureString(timeText);
            _font.DrawText(_spriteBatch, timeText, new Vector2((_screenWidth - timeSize.X) / 2, menuRect.Y + 100), Color.White);

            string continueText = "Press C to Continue";
            Vector2 continueSize = _font.MeasureString(continueText);
            _font.DrawText(_spriteBatch, continueText, new Vector2((_screenWidth - continueSize.X) / 2, menuRect.Y + 140), Color.White);

            string restartText = "Press R to Replay";
            Vector2 restartSize = _font.MeasureString(restartText);
            _font.DrawText(_spriteBatch, restartText, new Vector2((_screenWidth - restartSize.X) / 2, menuRect.Y + 180), Color.White);

            string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
            _font.DrawText(_spriteBatch, levelText, new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.85f)), Color.LightGray);
        }

        private void DrawAnimatedBackground()
        {
            _spriteBatch.Draw(_gradientTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), Color.White);
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), new Color(0, 0, 0, 100));

            // Простая анимация фона (облака)
            for (int i = 0; i < 5; i++)
            {
                float x = (_backgroundAnimationOffset + i * _screenWidth / 5) % _screenWidth;
                _spriteBatch.Draw(_pixelTexture, new Rectangle((int)x, _screenHeight / 4 + i * 20, 100, 20), Color.White * 0.2f);
            }
        }

        private void DrawTooltip()
        {
            Vector2 tooltipSize = _font.MeasureString(_tooltipText);
            Rectangle tooltipRect = new Rectangle((int)_tooltipPosition.X, (int)_tooltipPosition.Y, (int)tooltipSize.X + 10, (int)tooltipSize.Y + 5);
            _spriteBatch.Draw(_pixelTexture, tooltipRect, Color.Black * 0.8f);
            _font.DrawText(_spriteBatch, _tooltipText, _tooltipPosition + new Vector2(5, 2), Color.White);
        }

        public void ShowMainMenu()
        {
            _gameController.EnterMainMenu();
            Log("UIController", "Главное меню отображено", () => Info("UIController", "ShowMainMenu завершен"));
        }

        private void ShowBikeSelection()
        {
            _gameController.EnterBikeSelection();
            var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();
            _gameController.SetBikeType(availableBikes[_selectedBikeIndex]);
            _gameController.SetBikeColor(_availableBikeColors[_selectedColorIndex]);
            Log("UIController", "Меню выбора мотоцикла отображено", () => Info("UIController", "ShowBikeSelection завершен"));
        }

        private void ShowLevelSelection()
        {
            _gameController.EnterLevelSelection();
            _gameController.SelectLevel(_selectedLevelIndex + 1);
            Log("UIController", "Меню выбора уровня отображено", () => Info("UIController", "ShowLevelSelection завершен"));
        }

        private void StartGame(int levelId)
        {
            _gameController.StartLevel(levelId);
            Log("UIController", $"Уровень {levelId} начат", () => Info("UIController", "StartGame завершен"));
        }

        public void ShowMessage(string message)
        {
            Log("UIController", $"Сообщение: {message}", () => Debug("UIController", $"ShowMessage: {message}"));
        }
    }

    public class UIButton
    {
        public Rectangle Bounds { get; set; }
        public bool IsVisible { get; set; } = true;
        public Action OnClick { get; set; }
        public string Text { get; set; }
        public Color BaseColor { get; set; } = Color.Gray;
        public Color HoverColor { get; set; } = Color.DarkGray;
        public string Tooltip { get; set; } = ""; // Подсказка
    }
}