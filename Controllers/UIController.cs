using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Linq;
using FontStashSharp;
using GravityDefiedGame.Controllers;
using GravityDefiedGame.Utilities;
using GravityDefiedGame.Views;
using static GravityDefiedGame.Utilities.Logger;
using static GravityDefiedGame.Models.BikeGeom;

namespace GravityDefiedGame
{
    #region UI Constants and Types
    public static class UIConstants
    {
        public static class Animation
        {
            public const float ButtonScaleSpeed = 10f;
            public const float BackgroundSpeed = 20f;
            public const float HoverScale = 1.1f;
        }

        public static class Visual
        {
            public const int DefaultBorderThickness = 2;
            public const int LargeBorderThickness = 3;
            public const int TitleVerticalOffset = 30;
            public const int MainTextVerticalOffset = 100;
            public const int SubtitleVerticalOffset = 120;
            public const int TooltipPadding = 5;
            public const float ShadowOffset = 2f;
            public const float ShadowOpacity = 0.5f;
        }

        public static class MenuSizes
        {
            public const float StandardWidthRatio = 0.6f;
            public const float StandardHeightRatio = 0.7f;
            public const float WideWidthRatio = 0.7f;
        }

        public static class Layout
        {
            public const int ButtonSpacing = 20;
            public const int ButtonRowHeight = 60;
            public const float MainButtonWidthRatio = 0.2f;
            public const float SmallButtonWidthRatio = 0.12f;
            public const float NavButtonWidthRatio = 0.08f;
        }
    }

    public record UIButtonDefinition(
        Rectangle Bounds,
        string Text,
        Action OnClick,
        Color BaseColor,
        Color HoverColor,
        string Tooltip);

    public class UIButton
    {
        private readonly UIButtonDefinition _definition;
        public bool IsVisible { get; set; } = true;
        public bool IsHovered { get; set; } = false;
        public float Scale { get; set; } = 1.0f;

        public UIButton(UIButtonDefinition definition) =>
            _definition = definition;

        public Rectangle Bounds => _definition.Bounds;
        public string Text => _definition.Text;
        public Action OnClick => _definition.OnClick;
        public Color BaseColor => _definition.BaseColor;
        public Color HoverColor => _definition.HoverColor;
        public string Tooltip => _definition.Tooltip;
    }
    #endregion

    public class UIController
    {
        #region Private Fields
        private readonly Game _game;
        private readonly SpriteBatch _spriteBatch;
        private readonly DynamicSpriteFont _font;
        private readonly DynamicSpriteFont _titleFont;
        private readonly DynamicSpriteFont _unicodeFont;
        private readonly Texture2D _pixelTexture;
        private readonly Texture2D _gradientTexture;
        private readonly int _screenWidth;
        private readonly int _screenHeight;
        private readonly Dictionary<string, UIButton> _buttons = new();

        private KeyboardState _previousKeyboardState;
        private MouseState _previousMouseState;

        private int _selectedBikeIndex = 0;
        private int _selectedLevelIndex = 0;
        private int _selectedColorIndex = 0;
        private int _selectedThemeIndex = 0;
        private float _backgroundAnimationOffset = 0f;
        private string _tooltipText = "";
        private Vector2 _tooltipPosition;
        private GameController _gameController;

        private static readonly List<Color> _availableBikeColors = new()
        {
            Color.Red, Color.Blue, Color.Green, Color.Yellow, Color.Purple, Color.Orange
        };

        private static readonly List<string> _colorNames = new()
        {
            "Red", "Blue", "Green", "Yellow", "Purple", "Orange"
        };
        #endregion

        public UIController(
            Game game,
            GameController gameController,
            SpriteBatch spriteBatch,
            DynamicSpriteFont font,
            DynamicSpriteFont titleFont,
            DynamicSpriteFont unicodeFont,
            Texture2D pixelTexture,
            Texture2D gradientTexture,
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
            _screenWidth = screenWidth;
            _screenHeight = screenHeight;

            InitializeUI();
        }

        #region Initialization
        private void InitializeUI()
        {
            int centerX = _screenWidth / 2;
            int centerY = _screenHeight / 2;

            int standardWidth = (int)(_screenWidth * UIConstants.Layout.MainButtonWidthRatio);
            int smallWidth = (int)(_screenWidth * UIConstants.Layout.SmallButtonWidthRatio);
            int navWidth = (int)(_screenWidth * UIConstants.Layout.NavButtonWidthRatio);
            int standardHeight = UIConstants.Layout.ButtonRowHeight;
            int spacing = UIConstants.Layout.ButtonSpacing;

            int menuStartY = centerY - standardHeight * 2;

            var buttonDefinitions = new Dictionary<string, UIButtonDefinition>
            {
                // Главное меню
                ["play"] = new(
                    new Rectangle(centerX - standardWidth / 2, menuStartY, standardWidth, standardHeight),
                    "Play Game",
                    () => ShowBikeSelection(),
                    new Color(0, 100, 0),
                    Color.Green,
                    "Start playing"),

                ["theme"] = new(
                    new Rectangle(centerX - standardWidth / 2, menuStartY + standardHeight + spacing, standardWidth, standardHeight),
                    "Change Theme",
                    () => ShowThemeSelection(),
                    new Color(70, 70, 150),
                    Color.RoyalBlue,
                    "Change visual theme"),

                ["exit"] = new(
                    new Rectangle(centerX - standardWidth / 2, menuStartY + 2 * (standardHeight + spacing), standardWidth, standardHeight),
                    "Exit",
                    () => _game.Exit(),
                    new Color(150, 30, 30),
                    Color.Firebrick,
                    "Exit the game"),

                // Выбор мотоцикла
                ["prevBike"] = new(
                    new Rectangle(centerX - standardWidth / 2 - navWidth - spacing, menuStartY, navWidth, standardHeight),
                    "←",
                    () => ChangeBike(-1),
                    new Color(60, 60, 100),
                    Color.RoyalBlue,
                    "Previous bike type"),

                ["nextBike"] = new(
                    new Rectangle(centerX + standardWidth / 2 + spacing, menuStartY, navWidth, standardHeight),
                    "→",
                    () => ChangeBike(1),
                    new Color(60, 60, 100),
                    Color.RoyalBlue,
                    "Next bike type"),

                ["prevColor"] = new(
                    new Rectangle(centerX - standardWidth / 2 - navWidth - spacing, menuStartY + standardHeight + spacing, navWidth, standardHeight),
                    "←",
                    () => ChangeBikeColor(-1),
                    new Color(100, 60, 60),
                    Color.Firebrick,
                    "Previous color"),

                ["nextColor"] = new(
                    new Rectangle(centerX + standardWidth / 2 + spacing, menuStartY + standardHeight + spacing, navWidth, standardHeight),
                    "→",
                    () => ChangeBikeColor(1),
                    new Color(100, 60, 60),
                    Color.Firebrick,
                    "Next color"),

                ["selectBike"] = new(
                    new Rectangle(centerX - smallWidth / 2, menuStartY + 3 * (standardHeight + spacing), smallWidth, standardHeight),
                    "Select",
                    () => ShowLevelSelection(),
                    new Color(80, 50, 0),
                    Color.DarkOrange,
                    "Confirm bike selection"),

                // Выбор уровня
                ["prevLevel"] = new(
                    new Rectangle(centerX - standardWidth / 2 - navWidth - spacing, menuStartY, navWidth, standardHeight),
                    "←",
                    () => ChangeLevel(-1),
                    new Color(60, 100, 60),
                    Color.ForestGreen,
                    "Previous level"),

                ["nextLevel"] = new(
                    new Rectangle(centerX + standardWidth / 2 + spacing, menuStartY, navWidth, standardHeight),
                    "→",
                    () => ChangeLevel(1),
                    new Color(60, 100, 60),
                    Color.ForestGreen,
                    "Next level"),

                ["startLevel"] = new(
                    new Rectangle(centerX - smallWidth / 2, menuStartY + 3 * (standardHeight + spacing), smallWidth, standardHeight),
                    "Start",
                    () => StartGame(_selectedLevelIndex + 1),
                    new Color(0, 80, 0),
                    Color.Green,
                    "Start the selected level"),

                // Выбор темы
                ["prevTheme"] = new(
                    new Rectangle(centerX - standardWidth / 2 - navWidth - spacing, menuStartY, navWidth, standardHeight),
                    "←",
                    () => ChangeTheme(-1),
                    new Color(100, 60, 100),
                    Color.MediumOrchid,
                    "Previous theme"),

                ["nextTheme"] = new(
                    new Rectangle(centerX + standardWidth / 2 + spacing, menuStartY, navWidth, standardHeight),
                    "→",
                    () => ChangeTheme(1),
                    new Color(100, 60, 100),
                    Color.MediumOrchid,
                    "Next theme"),

                ["confirmTheme"] = new(
                    new Rectangle(centerX - smallWidth / 2, menuStartY + 3 * (standardHeight + spacing), smallWidth, standardHeight),
                    "Confirm",
                    () => ShowMainMenu(),
                    new Color(80, 50, 100),
                    Color.MediumPurple,
                    "Confirm theme selection"),

                // Игровые кнопки
                ["pause"] = new(
                    new Rectangle(_screenWidth - smallWidth - spacing, spacing, smallWidth, standardHeight),
                    "Pause",
                    () => _gameController.PauseGame(),
                    new Color(0, 50, 80),
                    Color.DarkBlue,
                    "Pause the game"),

                // Кнопки для экранов паузы, завершения и проигрыша
                ["restart"] = new(
                    new Rectangle(centerX - smallWidth - spacing, centerY + 2 * spacing, smallWidth, standardHeight),
                    "Restart",
                    () => _gameController.RestartLevel(),
                    new Color(50, 50, 50),
                    Color.DarkGray,
                    "Restart the current level"),

                ["mainMenu"] = new(
                    new Rectangle(centerX - smallWidth / 2, centerY + 2 * spacing, smallWidth, standardHeight),
                    "Menu",
                    () => ShowMainMenu(),
                    new Color(50, 50, 100),
                    Color.SlateBlue,
                    "Return to main menu"),

                ["continue"] = new(
                    new Rectangle(centerX + spacing, centerY + 2 * spacing, smallWidth, standardHeight),
                    "Continue",
                    () => _gameController.StartNextLevel(),
                    new Color(0, 80, 0),
                    Color.DarkGreen,
                    "Proceed to the next level"),

                ["back"] = new(
                    new Rectangle(spacing, spacing, navWidth, standardHeight),
                    "Back",
                    () => ShowMainMenu(),
                    new Color(80, 30, 30),
                    Color.Firebrick,
                    "Back to main menu")
            };

            foreach (var (key, definition) in buttonDefinitions)
            {
                _buttons[key] = new UIButton(definition) { IsVisible = false };
            }

            _buttons["play"].IsVisible = true;
            _buttons["theme"].IsVisible = true;
            _buttons["exit"].IsVisible = true;

            _selectedThemeIndex = 0;
            Info("UIController", "UI initialized");
        }
        #endregion

        #region Input Handling
        public void HandleInput(GameController gameController, float elapsedTime)
        {
            var keyboardState = Keyboard.GetState();
            var mouseState = Mouse.GetState();
            _gameController = gameController;

            _backgroundAnimationOffset = (_backgroundAnimationOffset + UIConstants.Animation.BackgroundSpeed * elapsedTime) % _screenWidth;

            var currentState = gameController.CurrentGameState;

            if (keyboardState.IsKeyDown(Keys.Escape) && _previousKeyboardState.IsKeyUp(Keys.Escape))
            {
                switch (currentState)
                {
                    case GameState.Playing:
                        _gameController.PauseGame();
                        break;
                    case GameState.Paused:
                        _gameController.ResumeGame();
                        break;
                    case GameState.BikeSelection:
                    case GameState.LevelSelection:
                    case GameState.ThemeSelection:
                    case GameState.GameOver:
                    case GameState.LevelComplete:
                        ShowMainMenu();
                        break;
                }
            }

            if (currentState == GameState.Playing)
            {
                gameController.HandleInput(keyboardState, _previousKeyboardState);
            }

            HandleMouseInput(mouseState, elapsedTime);

            _previousKeyboardState = keyboardState;
            _previousMouseState = mouseState;
        }

        private void HandleMouseInput(MouseState mouseState, float elapsedTime)
        {
            _tooltipText = "";

            foreach (var button in _buttons.Values)
            {
                button.IsHovered = false;
            }

            foreach (var button in _buttons.Values.Where(b => b.IsVisible))
            {
                bool isHovered = button.Bounds.Contains(mouseState.X, mouseState.Y);
                button.IsHovered = isHovered;

                button.Scale = MathHelper.Lerp(
                    button.Scale,
                    isHovered ? UIConstants.Animation.HoverScale : 1f,
                    UIConstants.Animation.ButtonScaleSpeed * elapsedTime);

                if (isHovered)
                {
                    _tooltipText = button.Tooltip;
                    _tooltipPosition = new Vector2(mouseState.X + 10, mouseState.Y + 10);

                    if (mouseState.LeftButton == ButtonState.Pressed && _previousMouseState.LeftButton == ButtonState.Released)
                        button.OnClick?.Invoke();
                }
            }
        }
        #endregion

        #region Selection Methods
        private void ChangeBike(int direction)
        {
            var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();
            _selectedBikeIndex = (_selectedBikeIndex + direction + availableBikes.Count) % availableBikes.Count;
            _gameController.SetBikeType(availableBikes[_selectedBikeIndex]);
        }

        private void ChangeBikeColor(int direction)
        {
            _selectedColorIndex = (_selectedColorIndex + direction + _availableBikeColors.Count) % _availableBikeColors.Count;
            _gameController.SetBikeColor(_availableBikeColors[_selectedColorIndex]);
        }

        private void ChangeLevel(int direction)
        {
            _selectedLevelIndex = Math.Clamp(
                _selectedLevelIndex + direction,
                0,
                _gameController.Levels.Count - 1);
            _gameController.SelectLevel(_selectedLevelIndex + 1);
        }

        private void ChangeTheme(int direction)
        {
            var themes = ThemeManager.AvailableThemes;
            _selectedThemeIndex = (_selectedThemeIndex + direction + themes.Count) % themes.Count;
            ThemeManager.SetTheme(_selectedThemeIndex);
            ShowMessage($"Theme changed to {ThemeManager.CurrentTheme.Name}");
        }
        #endregion

        #region UI Drawing
        public void DrawUI(GameController gameController)
        {
            UpdateButtonVisibility(gameController.CurrentGameState);
            DrawScreen(gameController);

            foreach (var button in _buttons.Values.Where(b => b.IsVisible))
                DrawButton(button);

            if (!string.IsNullOrEmpty(_tooltipText))
                DrawTooltip();
        }

        private void UpdateButtonVisibility(GameState state)
        {
            foreach (var button in _buttons.Values)
            {
                button.IsVisible = false;
            }

            switch (state)
            {
                case GameState.MainMenu:
                    _buttons["play"].IsVisible = true;
                    _buttons["theme"].IsVisible = true;
                    _buttons["exit"].IsVisible = true;
                    break;

                case GameState.BikeSelection:
                    _buttons["selectBike"].IsVisible = true;
                    _buttons["nextBike"].IsVisible = true;
                    _buttons["prevBike"].IsVisible = true;
                    _buttons["nextColor"].IsVisible = true;
                    _buttons["prevColor"].IsVisible = true;
                    _buttons["back"].IsVisible = true;
                    break;

                case GameState.LevelSelection:
                    _buttons["startLevel"].IsVisible = true;
                    _buttons["nextLevel"].IsVisible = true;
                    _buttons["prevLevel"].IsVisible = true;
                    _buttons["back"].IsVisible = true;
                    break;

                case GameState.ThemeSelection:
                    _buttons["nextTheme"].IsVisible = true;
                    _buttons["prevTheme"].IsVisible = true;
                    _buttons["confirmTheme"].IsVisible = true;
                    _buttons["back"].IsVisible = true;
                    break;

                case GameState.Playing:
                    _buttons["pause"].IsVisible = true;
                    break;

                case GameState.Paused:
                    _buttons["mainMenu"].IsVisible = true;
                    _buttons["restart"].IsVisible = true;
                    break;

                case GameState.GameOver:
                    _buttons["mainMenu"].IsVisible = true;
                    _buttons["restart"].IsVisible = true;
                    break;

                case GameState.LevelComplete:
                    _buttons["mainMenu"].IsVisible = true;
                    _buttons["restart"].IsVisible = true;
                    _buttons["continue"].IsVisible = true;
                    break;
            }
        }

        private void DrawScreen(GameController gameController)
        {
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
                case GameState.ThemeSelection:
                    DrawThemeSelectionMenu();
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
        }

        private Rectangle CreateMenuRect(float widthRatio = UIConstants.MenuSizes.StandardWidthRatio,
                                        float heightRatio = UIConstants.MenuSizes.StandardHeightRatio) =>
            new(
                _screenWidth / 2 - (int)(_screenWidth * widthRatio / 2),
                _screenHeight / 2 - (int)(_screenHeight * heightRatio / 2),
                (int)(_screenWidth * widthRatio),
                (int)(_screenHeight * heightRatio));

        private void DrawMenuBase(
            string title,
            Color borderColor,
            Action<Rectangle>? drawContent = null,
            float widthRatio = UIConstants.MenuSizes.StandardWidthRatio,
            float heightRatio = UIConstants.MenuSizes.StandardHeightRatio)
        {
            DrawAnimatedBackground();

            Rectangle menuRect = CreateMenuRect(widthRatio, heightRatio);

            _spriteBatch.Draw(_pixelTexture, menuRect, new Color(20, 20, 20, 220));
            DrawRectangleBorder(menuRect, borderColor, UIConstants.Visual.LargeBorderThickness);

            DrawMenuTitle(title, menuRect.Y + UIConstants.Visual.TitleVerticalOffset, borderColor);

            drawContent?.Invoke(menuRect);
        }

        private void DrawMenuTitle(string title, int yPosition, Color color)
        {
            Vector2 titleSize = _titleFont.MeasureString(title);
            float x = (_screenWidth - titleSize.X) / 2;

            _titleFont.DrawText(_spriteBatch, title,
                new Vector2(x + UIConstants.Visual.ShadowOffset, yPosition + UIConstants.Visual.ShadowOffset),
                Color.Black * UIConstants.Visual.ShadowOpacity);
            _titleFont.DrawText(_spriteBatch, title, new Vector2(x, yPosition), color);
        }

        private void DrawMainMenu() =>
            DrawMenuBase("GRAVITY DEFIED", Color.Goldenrod, menuRect =>
            {
                string subtitle = "A Motorcycle Physics Adventure";
                Vector2 subtitleSize = _font.MeasureString(subtitle);
                float x = (_screenWidth - subtitleSize.X) / 2;

                _font.DrawText(_spriteBatch, subtitle,
                    new Vector2(x + 1, menuRect.Y + UIConstants.Visual.SubtitleVerticalOffset + 1),
                    Color.Black * UIConstants.Visual.ShadowOpacity);
                _font.DrawText(_spriteBatch, subtitle,
                    new Vector2(x, menuRect.Y + UIConstants.Visual.SubtitleVerticalOffset),
                    Color.LightGoldenrodYellow);

                string themeText = $"Current Theme: {ThemeManager.CurrentTheme.Name}";
                Vector2 themeSize = _font.MeasureString(themeText);
                _font.DrawText(_spriteBatch, themeText,
                    new Vector2((_screenWidth - themeSize.X) / 2, menuRect.Y + 160),
                    Color.LightGoldenrodYellow);
            });

        private void DrawBikeSelectionMenu() =>
            DrawMenuBase("SELECT YOUR BIKE", Color.Cyan, menuRect =>
            {
                var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();
                string bikeTypeText = availableBikes[_selectedBikeIndex].ToString();
                Vector2 bikeTypeSize = _font.MeasureString(bikeTypeText);
                float x = (_screenWidth - bikeTypeSize.X) / 2;

                _font.DrawText(_spriteBatch, bikeTypeText,
                    new Vector2(x + 1, menuRect.Y + UIConstants.Visual.MainTextVerticalOffset + 1),
                    Color.Black * UIConstants.Visual.ShadowOpacity);
                _font.DrawText(_spriteBatch, bikeTypeText,
                    new Vector2(x, menuRect.Y + UIConstants.Visual.MainTextVerticalOffset),
                    _availableBikeColors[_selectedColorIndex]);

                string colorText = $"Color: {_colorNames[_selectedColorIndex]}";
                Vector2 colorTextSize = _font.MeasureString(colorText);
                _font.DrawText(_spriteBatch, colorText,
                    new Vector2((_screenWidth - colorTextSize.X) / 2, menuRect.Y + UIConstants.Visual.MainTextVerticalOffset + 40),
                    _availableBikeColors[_selectedColorIndex]);

                DrawBikePreview(menuRect);
            }, UIConstants.MenuSizes.WideWidthRatio);

        private void DrawBikePreview(Rectangle menuRect)
        {
            int previewWidth = (int)(menuRect.Width * 0.4f);
            int previewHeight = (int)(menuRect.Height * 0.3f);
            int previewX = menuRect.X + (menuRect.Width - previewWidth) / 2;
            int previewY = menuRect.Y + (int)(menuRect.Height * 0.3f);

            Rectangle previewRect = new Rectangle(previewX, previewY, previewWidth, previewHeight);
            _spriteBatch.Draw(_pixelTexture, previewRect, new Color(30, 30, 30, 200));
            DrawRectangleBorder(previewRect, Color.LightGray, UIConstants.Visual.DefaultBorderThickness);

            string previewText = "Bike Preview";
            Vector2 previewTextSize = _font.MeasureString(previewText);
            _font.DrawText(_spriteBatch, previewText,
                new Vector2(previewX + (previewWidth - previewTextSize.X) / 2, previewY + 10),
                Color.LightGray);

            var bikeType = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList()[_selectedBikeIndex];
            DrawBikeStats(previewX + 20, previewY + 40, bikeType);
        }

        private void DrawBikeStats(int x, int y, BikeType bikeType)
        {
            (string label, int value)[] stats = bikeType switch
            {
                BikeType.Standard => new[] { ("Speed", 3), ("Handling", 3), ("Stability", 3) },
                BikeType.Sport => new[] { ("Speed", 5), ("Handling", 2), ("Stability", 2) },
                BikeType.OffRoad => new[] { ("Speed", 2), ("Handling", 4), ("Stability", 5) },
                _ => new[] { ("Speed", 3), ("Handling", 3), ("Stability", 3) }
            };

            for (int i = 0; i < stats.Length; i++)
            {
                var (label, value) = stats[i];
                string statText = $"{label}: {new string('|', value)}{new string('.', 5 - value)}";
                _font.DrawText(_spriteBatch, statText, new Vector2(x, y + i * 30), Color.Yellow);
            }
        }

        private void DrawLevelSelectionMenu() =>
            DrawMenuBase("SELECT LEVEL", Color.Magenta, menuRect =>
            {
                string levelText = $"Level {_selectedLevelIndex + 1}: {_gameController.Levels[_selectedLevelIndex].Name}";
                Vector2 levelSize = _font.MeasureString(levelText);
                float x = (_screenWidth - levelSize.X) / 2;

                _font.DrawText(_spriteBatch, levelText,
                    new Vector2(x + 1, menuRect.Y + UIConstants.Visual.MainTextVerticalOffset + 1),
                    Color.Black * UIConstants.Visual.ShadowOpacity);
                _font.DrawText(_spriteBatch, levelText,
                    new Vector2(x, menuRect.Y + UIConstants.Visual.MainTextVerticalOffset),
                    Color.Yellow);

                string difficultyText = $"Difficulty: {GetLevelDifficulty(_selectedLevelIndex + 1)}";
                Vector2 difficultyPos = new Vector2(
                    (_screenWidth - _font.MeasureString(difficultyText).X) / 2,
                    menuRect.Y + 150);
                _font.DrawText(_spriteBatch, difficultyText, difficultyPos, Color.Yellow);
            }, UIConstants.MenuSizes.WideWidthRatio);

        private static string GetLevelDifficulty(int levelId) => levelId switch
        {
            <= 5 => "[|..]",
            <= 15 => "[||.]",
            _ => "[|||]"
        };

        private void DrawThemeSelectionMenu() =>
            DrawMenuBase("SELECT THEME", Color.Purple, menuRect =>
            {
                var themes = ThemeManager.AvailableThemes;
                string themeName = themes[_selectedThemeIndex].Name;
                Vector2 themeNameSize = _font.MeasureString(themeName);

                _font.DrawText(_spriteBatch, themeName,
                    new Vector2((_screenWidth - themeNameSize.X) / 2 + 1, menuRect.Y + 100 + 1),
                    Color.Black * UIConstants.Visual.ShadowOpacity);
                _font.DrawText(_spriteBatch, themeName,
                    new Vector2((_screenWidth - themeNameSize.X) / 2, menuRect.Y + 100),
                    Color.White);

                string themeDesc = themes[_selectedThemeIndex].Description;
                Vector2 descSize = _font.MeasureString(themeDesc);
                _font.DrawText(_spriteBatch, themeDesc,
                    new Vector2((_screenWidth - descSize.X) / 2, menuRect.Y + 140),
                    Color.LightGray);

                DrawThemePreview(menuRect);
            });

        private void DrawThemePreview(Rectangle menuRect)
        {
            var theme = ThemeManager.AvailableThemes[_selectedThemeIndex];
            int previewY = menuRect.Y + 180;
            int swatchSize = 30;
            int margin = 10;
            int totalWidth = 6 * (swatchSize + margin);
            int startX = (_screenWidth - totalWidth) / 2;

            DrawColorSwatch(startX, previewY, swatchSize, theme.BackgroundColor, "BG");
            DrawColorSwatch(startX + swatchSize + margin, previewY, swatchSize, theme.TerrainColor, "Terrain");
            DrawColorSwatch(startX + 2 * (swatchSize + margin), previewY, swatchSize, theme.SafeZoneColor, "Safe");
            DrawColorSwatch(startX + 3 * (swatchSize + margin), previewY, swatchSize, theme.BikeColors[SkeletonLineType.MainFrame], "Bike");
            DrawColorSwatch(startX + 4 * (swatchSize + margin), previewY, swatchSize, theme.WheelFill, "Wheel");
            DrawColorSwatch(startX + 5 * (swatchSize + margin), previewY, swatchSize, theme.ShadowColor, "Shadow");
        }

        private void DrawColorSwatch(int x, int y, int size, Color color, string label)
        {
            _spriteBatch.Draw(_pixelTexture, new Rectangle(x, y, size, size), color);
            _font.DrawText(_spriteBatch, label, new Vector2(x, y + size + 2), Color.LightGray);
        }

        private void DrawInfoPanel(GameController gameController)
        {
            string levelName = gameController.CurrentLevel?.Name ?? "Level";
            string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
            string directionText = gameController.Motorcycle.Direction == 1 ? "Forward" : "Backward";

            Rectangle infoPanelRect = new Rectangle(10, 10, (int)(_screenWidth * 0.25f), (int)(_screenHeight * 0.15f));
            _spriteBatch.Draw(_pixelTexture, infoPanelRect, new Color(0, 0, 0, 150));
            DrawRectangleBorder(infoPanelRect, Color.LightBlue, UIConstants.Visual.DefaultBorderThickness);

            _font.DrawText(_spriteBatch, $"Level: {levelName}", new Vector2(20, 20), Color.White);
            _font.DrawText(_spriteBatch, timeText, new Vector2(20, 45), Color.White);
            _font.DrawText(_spriteBatch, $"Direction: {directionText}", new Vector2(20, 70), Color.White);

            string controlsText = "W: Forward | S: Backward | Space: Brake | A/D: Lean | ESC: Pause";
            Vector2 controlsSize = _font.MeasureString(controlsText);
            _font.DrawText(_spriteBatch, controlsText,
                new Vector2((_screenWidth - controlsSize.X) / 2, _screenHeight - 40),
                Color.LightBlue);
        }

        private void DrawPauseMenu(GameController gameController) =>
            DrawMenuBase("GAME PAUSED", Color.LightBlue, menuRect =>
            {
                string resumeText = "Press ESC to Resume";
                Vector2 resumeSize = _font.MeasureString(resumeText);
                _font.DrawText(_spriteBatch, resumeText,
                    new Vector2((_screenWidth - resumeSize.X) / 2, menuRect.Y + 120),
                    Color.White);

                string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
                string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
                _font.DrawText(_spriteBatch, levelText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.6f)),
                    Color.LightGray);
                _font.DrawText(_spriteBatch, timeText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.6f) + 25),
                    Color.LightGray);
            });

        private void DrawGameOverMenu(GameController gameController) =>
            DrawMenuBase("GAME OVER", Color.Red, menuRect =>
            {
                string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
                string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
                _font.DrawText(_spriteBatch, levelText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.4f)),
                    Color.LightGray);
                _font.DrawText(_spriteBatch, timeText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.4f) + 25),
                    Color.LightGray);
            });

        private void DrawLevelCompleteMenu(GameController gameController) =>
            DrawMenuBase("LEVEL COMPLETE!", Color.LimeGreen, menuRect =>
            {
                string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
                Vector2 timeSize = _font.MeasureString(timeText);
                _font.DrawText(_spriteBatch, timeText,
                    new Vector2((_screenWidth - timeSize.X) / 2, menuRect.Y + 100),
                    Color.White);

                string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
                _font.DrawText(_spriteBatch, levelText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.6f)),
                    Color.LightGray);
            });

        private void DrawButton(UIButton button)
        {
            Color buttonColor = button.IsHovered ? button.HoverColor : button.BaseColor;

            Rectangle scaledBounds = new Rectangle(
                (int)(button.Bounds.X - button.Bounds.Width * (button.Scale - 1) / 2),
                (int)(button.Bounds.Y - button.Bounds.Height * (button.Scale - 1) / 2),
                (int)(button.Bounds.Width * button.Scale),
                (int)(button.Bounds.Height * button.Scale)
            );

            _spriteBatch.Draw(_pixelTexture,
                new Rectangle(scaledBounds.X + 4, scaledBounds.Y + 4, scaledBounds.Width, scaledBounds.Height),
                Color.Black * 0.3f);

            _spriteBatch.Draw(_pixelTexture, scaledBounds, buttonColor);
            DrawRectangleBorder(scaledBounds, Color.White, UIConstants.Visual.DefaultBorderThickness);

            if (!string.IsNullOrEmpty(button.Text))
            {
                Vector2 textSize = _font.MeasureString(button.Text);
                Vector2 textPosition = new Vector2(
                    scaledBounds.X + (scaledBounds.Width - textSize.X) / 2,
                    scaledBounds.Y + (scaledBounds.Height - textSize.Y) / 2
                );

                _font.DrawText(_spriteBatch, button.Text,
                    textPosition + new Vector2(1, 1),
                    Color.Black * 0.5f);

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

        private void DrawAnimatedBackground()
        {
            _spriteBatch.Draw(_gradientTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), ThemeManager.CurrentTheme.BackgroundColor);
            _spriteBatch.Draw(_pixelTexture, new Rectangle(0, 0, _screenWidth, _screenHeight), new Color(0, 0, 0, 100));

            for (int i = 0; i < 5; i++)
            {
                float x = (_backgroundAnimationOffset + i * _screenWidth / 5) % _screenWidth;
                _spriteBatch.Draw(_pixelTexture,
                    new Rectangle((int)x, _screenHeight / 4 + i * 20, 100, 20),
                    Color.White * 0.2f);
            }
        }

        private void DrawTooltip()
        {
            Vector2 tooltipSize = _font.MeasureString(_tooltipText);
            Rectangle tooltipRect = new Rectangle(
                (int)_tooltipPosition.X,
                (int)_tooltipPosition.Y,
                (int)tooltipSize.X + UIConstants.Visual.TooltipPadding * 2,
                (int)tooltipSize.Y + UIConstants.Visual.TooltipPadding);

            _spriteBatch.Draw(_pixelTexture, tooltipRect, Color.Black * 0.8f);
            _font.DrawText(_spriteBatch, _tooltipText,
                _tooltipPosition + new Vector2(UIConstants.Visual.TooltipPadding, UIConstants.Visual.TooltipPadding / 2),
                Color.White);
        }
        #endregion

        #region Navigation Methods
        public void ShowMainMenu()
        {
            _gameController.EnterMainMenu();
            Info("UIController", "Main menu displayed");
        }

        private void ShowBikeSelection()
        {
            _gameController.EnterBikeSelection();
            var availableBikes = Enum.GetValues(typeof(BikeType)).Cast<BikeType>().ToList();
            _gameController.SetBikeType(availableBikes[_selectedBikeIndex]);
            _gameController.SetBikeColor(_availableBikeColors[_selectedColorIndex]);
            Info("UIController", "Bike selection menu displayed");
        }

        private void ShowLevelSelection()
        {
            _gameController.EnterLevelSelection();
            _gameController.SelectLevel(_selectedLevelIndex + 1);
            Info("UIController", "Level selection menu displayed");
        }

        public void ShowThemeSelection()
        {
            _gameController.EnterThemeSelection();
            Info("UIController", "Theme selection menu displayed");
        }

        private void StartGame(int levelId)
        {
            _gameController.StartLevel(levelId);
            Info("UIController", $"Level {levelId} started");
        }
        #endregion

        public void ShowMessage(string message) => Debug("UIController", $"Message: {message}");
    }
}