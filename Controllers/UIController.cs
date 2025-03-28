#nullable enable
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using FontStashSharp;
using static GravityDefiedGame.Controllers.GameState;
using static GravityDefiedGame.Utilities.Logger;
using static GravityDefiedGame.Models.BikeGeom;
using static GravityDefiedGame.Views.ThemeManager;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Views
{
    public enum TransitionState
    {
        None,
        FadeIn,
        FadeOut,
        Loading
    }

    public static class UI
    {
        public static class Animation
        {
            public const float
                ButtonScaleSpeed = 10f,     // Animation speed for button scaling
                BackgroundSpeed = 20f,      // Background scrolling speed
                HoverScale = 1.1f,          // Scale factor when hovering buttons
                FadeSpeed = 2.0f,           // Screen fade in/out speed
                IntroFadeSpeed = 1.2f,      // Intro sequence fade speed
                LoadingBarSpeed = 0.6f,     // Loading bar fill speed
                FogAnimationSpeed = 1.5f,   // Fog effect animation speed
                SmokeAnimationSpeed = 15f,  // Smoke effect animation speed
                SmokeParticleLifespan = 3.0f; // Smoke particle lifetime in seconds
        }

        public static class Visual
        {
            public const int
                BorderThin = 2,             // Thin border width
                BorderThick = 3,            // Thick border width
                Padding = 5,                // Standard UI padding
                TitleOffset = 30,           // Title text vertical offset
                MainTextOffset = 100,       // Main text vertical offset
                SubtitleOffset = 120,       // Subtitle vertical offset
                LoadingBarHeight = 6,       // Height of loading progress bar
                LoadingBarWidth = 400,      // Width of loading progress bar
                LoadingBarPadding = 20,     // Padding around loading bar
                SmokeParticleCount = 100,   // Total smoke particle count
                SmokeLowerLayerCount = 60,  // Particles in lower layer
                SmokeMiddleLayerCount = 30, // Particles in middle layer
                SmokeUpperLayerCount = 20;  // Particles in upper layer

            public const float
                ShadowOffset = 2f,          // Text shadow offset
                ShadowOpacity = 0.5f;       // Text shadow opacity
        }

        public static class Layout
        {
            public const float
                MenuWidthStandard = 0.6f,   // Standard menu width (% of screen)
                MenuHeightStandard = 0.7f,  // Standard menu height (% of screen)
                MenuWidthWide = 0.7f,       // Wide menu width (% of screen)
                ButtonWidthMain = 0.25f,    // Main button width (% of screen)
                ButtonWidthSmall = 0.15f,   // Small button width (% of screen)
                ButtonWidthNav = 0.08f;     // Navigation button width (% of screen)

            public const int
                ButtonSpacing = 30,         // Vertical space between buttons
                ButtonHeight = 60,          // Standard button height
                NavButtonOffset = 20,       // Navigation button horizontal offset
                ButtonGroupSpacing = 50;    // Space between button groups
        }
    }

    public record UIButton(
        Rectangle Bounds,
        string Text,
        Action OnClick,
        Color BaseColor,
        Color HoverColor,
        string Tooltip)
    {
        public bool IsVisible { get; set; } = true;
        public bool IsHovered { get; set; } = false;
        public float Scale { get; set; } = 1.0f;
    }

    public enum SmokeLayer
    {
        Lower,
        Middle,
        Upper
    }

    public class UIController
    {
        #region Fields
        private readonly Game _game;
        private readonly SpriteBatch _spriteBatch;
        private readonly (
            DynamicSpriteFont standard,
            DynamicSpriteFont title,
            DynamicSpriteFont unicode
        ) _fonts;
        private readonly (
            Texture2D pixel,
            Texture2D gradient,
            Texture2D? logo
        ) _textures;
        private readonly (int width, int height) _screenSize;

        private readonly Dictionary<string, UIButton> _buttons = [];
        private (KeyboardState keyboard, MouseState mouse) _previousInputState;

        private (
            int bikeIndex,
            int levelIndex,
            int colorIndex,
            int themeIndex
        ) _selection = (0, 0, 0, 0);

        private float _backgroundOffset = 0f;
        private bool _showSmokeEffect = true;
        private (string text, Vector2 position) _tooltip = (string.Empty, Vector2.Zero);
        private Controllers.GameController _gameController = null!;

        private static readonly (Color color, string name)[] _bikeColors =
        [
            (Color.Red, "Red"),
            (Color.Blue, "Blue"),
            (Color.Green, "Green"),
            (Color.Yellow, "Yellow"),
            (Color.Purple, "Purple"),
            (Color.Orange, "Orange")
        ];

        private TransitionState _transitionState = TransitionState.None;
        private float _fadeAlpha = 1.0f;
        private float _loadingProgress = 0f;
        private bool _introComplete = false;
        private Controllers.GameState _nextGameState;
        private int _nextLevelId = 0;
        private float _transitionTimer = 0f;
        private bool _isSplashScreenActive = true;
        private bool _isCameraCentered = false;
        private float _cameraWaitTimer = 0f;
        private UIDrawer _drawer;
        #endregion

        public UIController(
           Game game,
           Controllers.GameController gameController,
           SpriteBatch spriteBatch,
           DynamicSpriteFont standardFont,
           DynamicSpriteFont titleFont,
           DynamicSpriteFont unicodeFont,
           Texture2D pixelTexture,
           Texture2D gradientTexture,
           Texture2D? logoTexture,
           int screenWidth,
           int screenHeight)
        {
            _game = game;
            _gameController = gameController;
            _spriteBatch = spriteBatch;
            _fonts = (standardFont, titleFont, unicodeFont);
            _textures = (pixelTexture, gradientTexture, logoTexture);
            _screenSize = (screenWidth, screenHeight);

            _drawer = new UIDrawer(
                _spriteBatch,
                _fonts,
                _textures,
                _screenSize);

            InitializeUI();
            _drawer.InitializeSmokeParticles();

            _isSplashScreenActive = true;
            _introComplete = false;
            _transitionState = TransitionState.FadeIn;
            _fadeAlpha = 1.0f;
            _loadingProgress = 0f;
        }

        #region Initialization
        private void InitializeUI()
        {
            var (width, height) = _screenSize;
            var centerX = width / 2;
            var centerY = height / 2;

            var btnWidth = (int)(width * UI.Layout.ButtonWidthMain);
            var smallWidth = (int)(width * UI.Layout.ButtonWidthSmall);
            var navWidth = (int)(width * UI.Layout.ButtonWidthNav);
            var btnHeight = UI.Layout.ButtonHeight;
            var spacing = UI.Layout.ButtonSpacing;
            var groupSpacing = UI.Layout.ButtonGroupSpacing;

            var menuStartY = centerY - btnHeight * 2 - groupSpacing / 2;

            var buttonDefs = new Dictionary<string, (Rectangle bounds, string text, Action action,
                                                  Color baseColor, Color hoverColor, string tooltip)>
            {
                ["play"] = (
                    new Rectangle(centerX - btnWidth / 2, menuStartY, btnWidth, btnHeight),
                    "Play Game",
                    () => ShowBikeSelection(),
                    new Color(0, 100, 0),
                    Color.Green,
                    "Start playing"
                ),

                ["theme"] = (
                    new Rectangle(centerX - btnWidth / 2, menuStartY + btnHeight + spacing, btnWidth, btnHeight),
                    "Change Theme",
                    () => ShowThemeSelection(),
                    new Color(70, 70, 150),
                    Color.RoyalBlue,
                    "Change visual theme"
                ),

                ["exit"] = (
                    new Rectangle(centerX - btnWidth / 2, menuStartY + 2 * (btnHeight + spacing), btnWidth, btnHeight),
                    "Exit",
                    () => _game.Exit(),
                    new Color(150, 30, 30),
                    Color.Firebrick,
                    "Exit the game"
                ),

                ["prevBike"] = (
                    new Rectangle(centerX - btnWidth / 2 - navWidth - spacing, menuStartY, navWidth, btnHeight),
                    "←",
                    () => ChangeBike(-1),
                    new Color(60, 60, 100),
                    Color.RoyalBlue,
                    "Previous bike type"
                ),

                ["nextBike"] = (
                    new Rectangle(centerX + btnWidth / 2 + spacing, menuStartY, navWidth, btnHeight),
                    "→",
                    () => ChangeBike(1),
                    new Color(60, 60, 100),
                    Color.RoyalBlue,
                    "Next bike type"
                ),

                ["prevColor"] = (
                    new Rectangle(centerX - btnWidth / 2 - navWidth - spacing, menuStartY + btnHeight + spacing, navWidth, btnHeight),
                    "←",
                    () => ChangeBikeColor(-1),
                    new Color(100, 60, 60),
                    Color.Firebrick,
                    "Previous color"
                ),

                ["nextColor"] = (
                    new Rectangle(centerX + btnWidth / 2 + spacing, menuStartY + btnHeight + spacing, navWidth, btnHeight),
                    "→",
                    () => ChangeBikeColor(1),
                    new Color(100, 60, 60),
                    Color.Firebrick,
                    "Next color"
                ),

                ["selectBike"] = (
                    new Rectangle(centerX - smallWidth / 2, menuStartY + 2 * (btnHeight + spacing) + groupSpacing, smallWidth, btnHeight),
                    "Select",
                    () => ShowLevelSelection(),
                    new Color(80, 50, 0),
                    Color.DarkOrange,
                    "Confirm bike selection"
                ),

                ["prevLevel"] = (
                    new Rectangle(centerX - btnWidth / 2 - navWidth - spacing, menuStartY, navWidth, btnHeight),
                    "←",
                    () => ChangeLevel(-1),
                    new Color(60, 100, 60),
                    Color.ForestGreen,
                    "Previous level"
                ),

                ["nextLevel"] = (
                    new Rectangle(centerX + btnWidth / 2 + spacing, menuStartY, navWidth, btnHeight),
                    "→",
                    () => ChangeLevel(1),
                    new Color(60, 100, 60),
                    Color.ForestGreen,
                    "Next level"
                ),

                ["startLevel"] = (
                    new Rectangle(centerX - smallWidth / 2, menuStartY + 2 * (btnHeight + spacing) + groupSpacing, smallWidth, btnHeight),
                    "Start",
                    () => StartGameWithTransition(_selection.levelIndex + 1),
                    new Color(0, 80, 0),
                    Color.Green,
                    "Start the selected level"
                ),

                ["prevTheme"] = (
                    new Rectangle(centerX - btnWidth / 2 - navWidth - spacing, menuStartY, navWidth, btnHeight),
                    "←",
                    () => ChangeTheme(-1),
                    new Color(100, 60, 100),
                    Color.MediumOrchid,
                    "Previous theme"
                ),

                ["nextTheme"] = (
                    new Rectangle(centerX + btnWidth / 2 + spacing, menuStartY, navWidth, btnHeight),
                    "→",
                    () => ChangeTheme(1),
                    new Color(100, 60, 100),
                    Color.MediumOrchid,
                    "Next theme"
                ),

                ["confirmTheme"] = (
                    new Rectangle(centerX - smallWidth / 2, menuStartY + 2 * (btnHeight + spacing) + groupSpacing, smallWidth, btnHeight),
                    "Confirm",
                    () => ShowMainMenu(),
                    new Color(80, 50, 100),
                    Color.MediumPurple,
                    "Confirm theme selection"
                ),

                ["pause"] = (
                    new Rectangle(width - smallWidth - spacing, spacing, smallWidth, btnHeight),
                    "Pause",
                    () => _gameController.PauseGame(),
                    new Color(0, 50, 80),
                    Color.DarkBlue,
                    "Pause the game"
                ),

                ["restart"] = (
                    new Rectangle(centerX - smallWidth - spacing * 2, centerY + groupSpacing, smallWidth, btnHeight),
                    "Restart",
                    () => StartGameWithTransition(_gameController.CurrentLevel!.Id, true),
                    new Color(50, 50, 50),
                    Color.DarkGray,
                    "Restart the current level"
                ),

                ["mainMenu"] = (
                    new Rectangle(centerX - smallWidth / 2, centerY + groupSpacing, smallWidth, btnHeight),
                    "Menu",
                    () => ShowMainMenu(),
                    new Color(50, 50, 100),
                    Color.SlateBlue,
                    "Return to main menu"
                ),

                ["continue"] = (
                    new Rectangle(centerX + spacing * 2, centerY + groupSpacing, smallWidth, btnHeight),
                    "Continue",
                    () => StartGameWithTransition(_gameController.CurrentLevel!.Id + 1),
                    new Color(0, 80, 0),
                    Color.DarkGreen,
                    "Proceed to the next level"
                ),

                ["back"] = (
                    new Rectangle(spacing, spacing, navWidth, btnHeight),
                    "Back",
                    () => ShowMainMenu(),
                    new Color(80, 30, 30),
                    Color.Firebrick,
                    "Back to main menu"
                )
            };

            foreach (var (key, (bounds, text, action, baseColor, hoverColor, tooltip)) in buttonDefs)
            {
                _buttons[key] = new UIButton(
                    bounds, text, action, baseColor, hoverColor, tooltip)
                {
                    IsVisible = false
                };
            }

            Info("UIController", "UI initialized");
        }
        #endregion

        #region Input
        public void HandleInput(Controllers.GameController gameController, float elapsedTime)
        {
            var keyboardState = Keyboard.GetState();
            var mouseState = Mouse.GetState();
            _gameController = gameController;

            _backgroundOffset = (_backgroundOffset + UI.Animation.BackgroundSpeed * elapsedTime) % _screenSize.width;
            _drawer.UpdateSmokeParticles(elapsedTime);

            if (_isSplashScreenActive)
            {
                if (_introComplete && (keyboardState.GetPressedKeys().Length > 0 ||
                                    mouseState.LeftButton == ButtonState.Pressed ||
                                    mouseState.RightButton == ButtonState.Pressed))
                {
                    StartTransition(TransitionState.FadeOut, Controllers.GameState.MainMenu);
                }

                UpdateTransitionState(elapsedTime);
                _previousInputState = (keyboardState, mouseState);
                return;
            }

            if (_transitionState == TransitionState.Loading)
            {
                UpdateTransitionState(elapsedTime);
                _previousInputState = (keyboardState, mouseState);
                return;
            }

            if (_transitionState != TransitionState.None)
            {
                UpdateTransitionState(elapsedTime);
                _previousInputState = (keyboardState, mouseState);
                return;
            }

            if (KeyPressed(keyboardState, Keys.Escape))
            {
                switch (gameController.CurrentGameState)
                {
                    case Controllers.GameState.Playing:
                        _gameController.PauseGame();
                        break;
                    case Controllers.GameState.Paused:
                        _gameController.ResumeGame();
                        break;
                    case Controllers.GameState.BikeSelection:
                    case Controllers.GameState.LevelSelection:
                    case Controllers.GameState.ThemeSelection:
                    case Controllers.GameState.GameOver:
                    case Controllers.GameState.LevelComplete:
                        StartTransition(TransitionState.FadeOut, Controllers.GameState.MainMenu);
                        break;
                }
            }

            if (gameController.CurrentGameState == Controllers.GameState.Playing)
                gameController.HandleInput(keyboardState, _previousInputState.keyboard);

            HandleMouseInput(mouseState, elapsedTime);

            _previousInputState = (keyboardState, mouseState);
        }

        private bool KeyPressed(KeyboardState current, Keys key) =>
            current.IsKeyDown(key) && _previousInputState.keyboard.IsKeyUp(key);

        private void HandleMouseInput(MouseState mouseState, float elapsedTime)
        {
            _tooltip = (string.Empty, Vector2.Zero);

            foreach (var button in _buttons.Values)
                button.IsHovered = false;

            foreach (var button in _buttons.Values.Where(b => b.IsVisible))
            {
                var isHovered = button.Bounds.Contains(mouseState.X, mouseState.Y);
                button.IsHovered = isHovered;

                button.Scale = MathHelper.Lerp(
                    button.Scale,
                    isHovered ? UI.Animation.HoverScale : 1f,
                    UI.Animation.ButtonScaleSpeed * elapsedTime);

                if (isHovered)
                {
                    _tooltip = (button.Tooltip, new Vector2(mouseState.X + 10, mouseState.Y + 10));

                    if (mouseState.LeftButton == ButtonState.Pressed &&
                        _previousInputState.mouse.LeftButton == ButtonState.Released)
                        button.OnClick?.Invoke();
                }
            }
        }
        #endregion

        #region Transitions and Animation
        private void UpdateTransitionState(float elapsedTime)
        {
            _transitionTimer += elapsedTime;

            switch (_transitionState)
            {
                case TransitionState.FadeIn:
                    if (!_isCameraCentered && _nextGameState == Controllers.GameState.Playing)
                    {
                        _cameraWaitTimer += elapsedTime;

                        if (_cameraWaitTimer >= 0.5f)
                            _isCameraCentered = true;

                        _fadeAlpha = 1.0f;
                        return;
                    }

                    _fadeAlpha = Math.Max(0, _fadeAlpha - UI.Animation.FadeSpeed * elapsedTime);
                    if (_fadeAlpha <= 0)
                    {
                        if (_isSplashScreenActive)
                        {
                            if (!_introComplete)
                            {
                                _loadingProgress = Math.Min(1.0f, _loadingProgress + UI.Animation.LoadingBarSpeed * elapsedTime);

                                if (_loadingProgress >= 1.0f)
                                    _introComplete = true;
                            }
                        }
                        else
                            _transitionState = TransitionState.None;
                    }
                    break;

                case TransitionState.FadeOut:
                    _fadeAlpha = Math.Min(1.0f, _fadeAlpha + UI.Animation.FadeSpeed * elapsedTime);
                    if (_fadeAlpha >= 1.0f)
                    {
                        if (_isSplashScreenActive)
                        {
                            _isSplashScreenActive = false;
                            _gameController.EnterMainMenu();
                            StartTransition(TransitionState.FadeIn, Controllers.GameState.MainMenu);
                        }
                        else if (_nextGameState == Controllers.GameState.Playing && _nextLevelId > 0)
                        {
                            _gameController.StartLevel(_nextLevelId);
                            _cameraWaitTimer = 0f;
                            StartTransition(TransitionState.FadeIn, Controllers.GameState.Playing);
                        }
                        else
                        {
                            SwitchToNextGameState();
                            StartTransition(TransitionState.FadeIn, _nextGameState);
                        }
                    }
                    break;
            }
        }

        private void StartTransition(TransitionState newState, Controllers.GameState nextGameState, int levelId = 0)
        {
            _transitionState = newState;
            _nextGameState = nextGameState;
            _nextLevelId = levelId;
            _transitionTimer = 0f;

            if (newState == TransitionState.FadeIn)
                _fadeAlpha = 1.0f;
            else if (newState == TransitionState.FadeOut)
                _fadeAlpha = 0f;
            else if (newState == TransitionState.Loading)
                _loadingProgress = 0f;
        }

        private void SwitchToNextGameState()
        {
            switch (_nextGameState)
            {
                case Controllers.GameState.MainMenu:
                    _gameController.EnterMainMenu();
                    break;

                case Controllers.GameState.BikeSelection:
                    _gameController.EnterBikeSelection();
                    var bikes = Enum.GetValues<BikeType>();
                    _gameController.SetBikeType(bikes[_selection.bikeIndex]);
                    _gameController.SetBikeColor(_bikeColors[_selection.colorIndex].color);
                    break;

                case Controllers.GameState.LevelSelection:
                    _gameController.EnterLevelSelection();
                    _gameController.SelectLevel(_selection.levelIndex + 1);
                    break;

                case Controllers.GameState.ThemeSelection:
                    _gameController.EnterThemeSelection();
                    break;

                case Controllers.GameState.Playing:
                    if (_nextLevelId > 0)
                        _gameController.StartLevel(_nextLevelId);
                    break;
            }
        }
        #endregion

        #region Selection Methods
        private void ChangeBike(int direction)
        {
            var bikes = Enum.GetValues<BikeType>();
            _selection.bikeIndex = (_selection.bikeIndex + direction + bikes.Length) % bikes.Length;
            _gameController.SetBikeType(bikes[_selection.bikeIndex]);
        }

        private void ChangeBikeColor(int direction)
        {
            _selection.colorIndex = (_selection.colorIndex + direction + _bikeColors.Length) % _bikeColors.Length;
            _gameController.SetBikeColor(_bikeColors[_selection.colorIndex].color);
        }

        private void ChangeLevel(int direction)
        {
            _selection.levelIndex = Math.Clamp(
                _selection.levelIndex + direction,
                0,
                _gameController.Levels.Count - 1);
            _gameController.SelectLevel(_selection.levelIndex + 1);
        }

        private void ChangeTheme(int direction)
        {
            var themes = AvailableThemes;
            _selection.themeIndex = (_selection.themeIndex + direction + themes.Count) % themes.Count;
            SetTheme(_selection.themeIndex);
            ShowMessage($"Theme changed to {CurrentTheme.Name}");
        }
        #endregion

        #region UI Drawing
        public void DrawUI(Controllers.GameController gameController)
        {
            if (_isSplashScreenActive)
            {
                _drawer.DrawSplashScreen(_fadeAlpha, _loadingProgress, _transitionTimer, _introComplete);
                return;
            }

            if (_transitionState == TransitionState.Loading)
            {
                _drawer.DrawLoadingScreen(_nextLevelId, _gameController.Levels[_nextLevelId - 1].Name, _loadingProgress);
                return;
            }

            UpdateButtonVisibility(gameController.CurrentGameState);

            bool isMenuState = gameController.CurrentGameState is
                Controllers.GameState.MainMenu or
                Controllers.GameState.BikeSelection or
                Controllers.GameState.LevelSelection or
                Controllers.GameState.ThemeSelection or
                Controllers.GameState.LevelComplete or
                Controllers.GameState.GameOver or
                Controllers.GameState.Paused;

            if (isMenuState)
            {
                _drawer.DrawAnimatedBackground(_backgroundOffset, CurrentTheme.BackgroundColor);

                if (_showSmokeEffect && gameController.CurrentGameState is
                    Controllers.GameState.MainMenu or
                    Controllers.GameState.BikeSelection or
                    Controllers.GameState.LevelSelection or
                    Controllers.GameState.ThemeSelection or
                    Controllers.GameState.LevelComplete)
                {
                    _drawer.DrawSmokeEffect();
                }
            }

            DrawScreen(gameController);

            foreach (var button in _buttons.Values.Where(b => b.IsVisible))
                _drawer.DrawButton(button);

            if (!string.IsNullOrEmpty(_tooltip.text))
                _drawer.DrawTooltip(_tooltip.text, _tooltip.position);

            if (_transitionState != TransitionState.None)
                _drawer.DrawTransitionOverlay(_fadeAlpha, _transitionState, _nextGameState, _isSplashScreenActive, _transitionTimer);
        }

        private void UpdateButtonVisibility(Controllers.GameState state)
        {
            if (_transitionState != TransitionState.None && _transitionState != TransitionState.FadeIn)
                return;

            foreach (var button in _buttons.Values)
                button.IsVisible = false;

            switch (state)
            {
                case Controllers.GameState.MainMenu:
                    _buttons["play"].IsVisible = true;
                    _buttons["theme"].IsVisible = true;
                    _buttons["exit"].IsVisible = true;
                    break;

                case Controllers.GameState.BikeSelection:
                    _buttons["selectBike"].IsVisible = true;
                    _buttons["nextBike"].IsVisible = true;
                    _buttons["prevBike"].IsVisible = true;
                    _buttons["nextColor"].IsVisible = true;
                    _buttons["prevColor"].IsVisible = true;
                    _buttons["back"].IsVisible = true;
                    break;

                case Controllers.GameState.LevelSelection:
                    _buttons["startLevel"].IsVisible = true;
                    _buttons["nextLevel"].IsVisible = true;
                    _buttons["prevLevel"].IsVisible = true;
                    _buttons["back"].IsVisible = true;
                    break;

                case Controllers.GameState.ThemeSelection:
                    _buttons["nextTheme"].IsVisible = true;
                    _buttons["prevTheme"].IsVisible = true;
                    _buttons["confirmTheme"].IsVisible = true;
                    _buttons["back"].IsVisible = true;
                    break;

                case Controllers.GameState.Playing:
                    _buttons["pause"].IsVisible = true;
                    break;

                case Controllers.GameState.Paused:
                    _buttons["mainMenu"].IsVisible = true;
                    _buttons["restart"].IsVisible = true;
                    break;

                case Controllers.GameState.GameOver:
                    _buttons["mainMenu"].IsVisible = true;
                    _buttons["restart"].IsVisible = true;
                    break;

                case Controllers.GameState.LevelComplete:
                    _buttons["mainMenu"].IsVisible = true;
                    _buttons["restart"].IsVisible = true;
                    _buttons["continue"].IsVisible = true;
                    break;
            }
        }

        private void DrawScreen(Controllers.GameController gameController)
        {
            switch (gameController.CurrentGameState)
            {
                case Controllers.GameState.MainMenu:
                    _drawer.DrawMainMenu();
                    break;
                case Controllers.GameState.BikeSelection:
                    _drawer.DrawBikeSelectionMenu(_selection.bikeIndex, _selection.colorIndex, _bikeColors);
                    break;
                case Controllers.GameState.LevelSelection:
                    _drawer.DrawLevelSelectionMenu(_selection.levelIndex, _gameController.Levels);
                    break;
                case Controllers.GameState.ThemeSelection:
                    _drawer.DrawThemeSelectionMenu(_selection.themeIndex, AvailableThemes);
                    break;
                case Controllers.GameState.Playing:
                    _drawer.DrawInfoPanel(gameController.CurrentLevel?.Name, gameController.GameTime, gameController.Motorcycle.Direction);
                    break;
                case Controllers.GameState.Paused:
                    _drawer.DrawPauseMenu(gameController.CurrentLevel?.Name, gameController.GameTime);
                    break;
                case Controllers.GameState.GameOver:
                    _drawer.DrawGameOverMenu(gameController.CurrentLevel?.Name, gameController.GameTime);
                    break;
                case Controllers.GameState.LevelComplete:
                    _drawer.DrawLevelCompleteMenu(gameController.CurrentLevel?.Name, gameController.GameTime);
                    break;
            }
        }
        #endregion

        #region Navigation
        public void ShowMainMenu()
        {
            StartTransition(TransitionState.FadeOut, Controllers.GameState.MainMenu);
            Info("UIController", "Main menu displayed");
        }

        private void ShowBikeSelection()
        {
            StartTransition(TransitionState.FadeOut, Controllers.GameState.BikeSelection);
            Info("UIController", "Bike selection menu displayed");
        }

        private void ShowLevelSelection()
        {
            StartTransition(TransitionState.FadeOut, Controllers.GameState.LevelSelection);
            Info("UIController", "Level selection menu displayed");
        }

        public void ShowThemeSelection()
        {
            StartTransition(TransitionState.FadeOut, Controllers.GameState.ThemeSelection);
            Info("UIController", "Theme selection menu displayed");
        }

        private void StartGameWithTransition(int levelId, bool isRestart = false)
        {
            _nextLevelId = levelId;
            _transitionState = TransitionState.FadeOut;
            _fadeAlpha = 0f;
            _transitionTimer = 0f;
            _cameraWaitTimer = 0f;
            _nextGameState = Controllers.GameState.Playing;
            _isCameraCentered = false;
            Info("UIController", $"{(isRestart ? "Restarting" : "Starting")} level {levelId} with transition");
        }
        #endregion

        public void ShowMessage(string message) => Debug("UIController", $"Message: {message}");
    }
}