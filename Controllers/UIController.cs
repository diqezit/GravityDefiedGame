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
            public const float ButtonScaleSpeed = 10f;
            public const float BackgroundSpeed = 20f;
            public const float HoverScale = 1.1f;

            // Параметры анимаций переходов
            public const float FadeSpeed = 2.0f;
            public const float IntroFadeSpeed = 1.2f;
            public const float LoadingBarSpeed = 0.6f;
            public const float FogAnimationSpeed = 1.5f;
        }

        public static class Visual
        {
            public const int BorderThin = 2;
            public const int BorderThick = 3;
            public const int Padding = 5;

            public const int TitleOffset = 30;
            public const int MainTextOffset = 100;
            public const int SubtitleOffset = 120;

            public const float ShadowOffset = 2f;
            public const float ShadowOpacity = 0.5f;

            // Параметры загрузочной полосы
            public const int LoadingBarHeight = 6;
            public const int LoadingBarWidth = 400;
            public const int LoadingBarPadding = 20;
        }

        public static class Layout
        {
            public const float MenuWidthStandard = 0.6f;
            public const float MenuHeightStandard = 0.7f;
            public const float MenuWidthWide = 0.7f;

            public const int ButtonSpacing = 30;
            public const int ButtonHeight = 60;
            public const float ButtonWidthMain = 0.25f;
            public const float ButtonWidthSmall = 0.15f;
            public const float ButtonWidthNav = 0.08f;

            public const int NavButtonOffset = 20;
            public const int ButtonGroupSpacing = 50;
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
            Texture2D? logo,
            Texture2D? fog
        ) _textures;
        private readonly (int width, int height) _screenSize;

        private readonly Dictionary<string, UIButton> _buttons = new();
        private (KeyboardState keyboard, MouseState mouse) _previousInputState;

        private (
            int bikeIndex,
            int levelIndex,
            int colorIndex,
            int themeIndex
        ) _selection = (0, 0, 0, 0);

        private float _backgroundOffset = 0f;
        private (string text, Vector2 position) _tooltip = (string.Empty, Vector2.Zero);
        private Controllers.GameController _gameController = null!;

        private static readonly (Color color, string name)[] _bikeColors =
        {
            (Color.Red, "Red"),
            (Color.Blue, "Blue"),
            (Color.Green, "Green"),
            (Color.Yellow, "Yellow"),
            (Color.Purple, "Purple"),
            (Color.Orange, "Orange")
        };

        // Параметры переходов и анимаций
        private TransitionState _transitionState = TransitionState.None;
        private float _fadeAlpha = 1.0f;
        private float _loadingProgress = 0f;
        private float _fogOffset = 0f;
        private bool _introComplete = false;
        private Controllers.GameState _nextGameState;
        private int _nextLevelId = 0;
        private float _transitionTimer = 0f;
        private bool _isSplashScreenActive = true;
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
            Texture2D? fogTexture,
            int screenWidth,
            int screenHeight)
        {
            _game = game;
            _gameController = gameController;
            _spriteBatch = spriteBatch;
            _fonts = (standardFont, titleFont, unicodeFont);
            _textures = (pixelTexture, gradientTexture, logoTexture, fogTexture);
            _screenSize = (screenWidth, screenHeight);

            InitializeUI();

            // Начинаем с заставки и эффекта появления
            _transitionState = TransitionState.FadeIn;
            _fadeAlpha = 1.0f;
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
                // Главное меню - центрированные кнопки с равным расстоянием
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

                // Выбор мотоцикла - с навигационными кнопками по бокам
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

                // Выбор уровня
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

                // Выбор темы
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

                // Игровые кнопки - по краям экрана
                ["pause"] = (
                    new Rectangle(width - smallWidth - spacing, spacing, smallWidth, btnHeight),
                    "Pause",
                    () => _gameController.PauseGame(),
                    new Color(0, 50, 80),
                    Color.DarkBlue,
                    "Pause the game"
                ),

                // Кнопки для паузы, завершения и проигрыша - горизонтальное расположение
                ["restart"] = (
                    new Rectangle(centerX - smallWidth - spacing * 2, centerY + groupSpacing, smallWidth, btnHeight),
                    "Restart",
                    () => _gameController.RestartLevel(),
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
                    () => _gameController.StartNextLevel(),
                    new Color(0, 80, 0),
                    Color.DarkGreen,
                    "Proceed to the next level"
                ),

                // Кнопка назад - всегда в верхнем левом углу
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

            // Обработка заставки - пропуск при нажатии любой клавиши
            if (_isSplashScreenActive)
            {
                if (keyboardState.GetPressedKeys().Length > 0 ||
                    mouseState.LeftButton == ButtonState.Pressed ||
                    mouseState.RightButton == ButtonState.Pressed)
                {
                    StartTransition(TransitionState.FadeOut, Controllers.GameState.MainMenu);
                }

                UpdateTransitionState(elapsedTime);
                _previousInputState = (keyboardState, mouseState);
                return;
            }

            // Пропускаем ввод во время переходов
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
            _fogOffset = (_fogOffset + UI.Animation.FogAnimationSpeed * elapsedTime) % _screenSize.width;

            switch (_transitionState)
            {
                case TransitionState.FadeIn:
                    // Уменьшаем прозрачность черного экрана
                    _fadeAlpha = Math.Max(0, _fadeAlpha - UI.Animation.FadeSpeed * elapsedTime);
                    if (_fadeAlpha <= 0)
                    {
                        if (_isSplashScreenActive)
                        {
                            // После показа заставки и загрузки, перейдем к главному меню
                            if (_loadingProgress >= 1.0f && _transitionTimer > 3.0f)
                            {
                                StartTransition(TransitionState.FadeOut, Controllers.GameState.MainMenu);
                            }
                            else
                            {
                                // Имитация загрузки
                                _loadingProgress = Math.Min(1.0f, _loadingProgress + UI.Animation.LoadingBarSpeed * elapsedTime);
                            }
                        }
                        else
                        {
                            _transitionState = TransitionState.None;
                        }
                    }
                    break;

                case TransitionState.FadeOut:
                    // Увеличиваем прозрачность черного экрана
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
                            // Начать загрузку уровня
                            _transitionState = TransitionState.Loading;
                            _loadingProgress = 0f;
                        }
                        else
                        {
                            // Переходим к новому состоянию
                            SwitchToNextGameState();
                            StartTransition(TransitionState.FadeIn, _nextGameState);
                        }
                    }
                    break;

                case TransitionState.Loading:
                    // Имитация процесса загрузки уровня
                    _loadingProgress = Math.Min(1.0f, _loadingProgress + UI.Animation.LoadingBarSpeed * elapsedTime);

                    if (_loadingProgress >= 1.0f)
                    {
                        // Загрузка завершена, запускаем уровень
                        _gameController.StartLevel(_nextLevelId);
                        StartTransition(TransitionState.FadeIn, Controllers.GameState.Playing);
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
            var themes = ThemeManager.AvailableThemes;
            _selection.themeIndex = (_selection.themeIndex + direction + themes.Count) % themes.Count;
            ThemeManager.SetTheme(_selection.themeIndex);
            ShowMessage($"Theme changed to {ThemeManager.CurrentTheme.Name}");
        }
        #endregion

        #region UI Drawing
        public void DrawUI(Controllers.GameController gameController)
        {
            if (_isSplashScreenActive)
            {
                DrawSplashScreen();
                return;
            }

            UpdateButtonVisibility(gameController.CurrentGameState);

            // Если в состоянии перехода, рисуем предыдущий экран
            if (_transitionState == TransitionState.FadeOut)
            {
                DrawScreen(gameController);

                foreach (var button in _buttons.Values.Where(b => b.IsVisible))
                    DrawButton(button);

                if (!string.IsNullOrEmpty(_tooltip.text))
                    DrawTooltip();
            }
            // Если загрузка, рисуем экран загрузки
            else if (_transitionState == TransitionState.Loading)
            {
                DrawLoadingScreen();
            }
            // В остальных случаях рисуем текущий экран
            else
            {
                DrawScreen(gameController);

                foreach (var button in _buttons.Values.Where(b => b.IsVisible))
                    DrawButton(button);

                if (!string.IsNullOrEmpty(_tooltip.text))
                    DrawTooltip();
            }

            // Отрисовка эффекта перехода поверх всего
            if (_transitionState != TransitionState.None)
            {
                DrawTransitionOverlay();
            }
        }

        private void DrawSplashScreen()
        {
            // Фон
            _spriteBatch.Draw(_textures.gradient, new Rectangle(0, 0, _screenSize.width, _screenSize.height), Color.Black);

            // Логотип (если есть)
            if (_textures.logo != null)
            {
                int logoWidth = Math.Min(_screenSize.width - 100, _textures.logo.Width);
                int logoHeight = (int)(logoWidth * ((float)_textures.logo.Height / _textures.logo.Width));

                Rectangle logoRect = new Rectangle(
                    (_screenSize.width - logoWidth) / 2,
                    (_screenSize.height - logoHeight) / 2 - 50,
                    logoWidth,
                    logoHeight);

                _spriteBatch.Draw(_textures.logo, logoRect, Color.White);
            }
            else
            {
                // Если логотипа нет, просто отображаем текст
                string gameTitle = "GRAVITY DEFIED";
                Vector2 titleSize = _fonts.title.MeasureString(gameTitle);
                float titleX = (_screenSize.width - titleSize.X) / 2;
                float titleY = _screenSize.height * 0.4f;

                _fonts.title.DrawText(_spriteBatch, gameTitle,
                    new Vector2(titleX + UI.Visual.ShadowOffset, titleY + UI.Visual.ShadowOffset),
                    Color.Black * UI.Visual.ShadowOpacity);
                _fonts.title.DrawText(_spriteBatch, gameTitle,
                    new Vector2(titleX, titleY),
                    Color.Goldenrod);
            }

            // Текст загрузки
            string loadingText = "Loading game...";
            Vector2 loadingTextSize = _fonts.standard.MeasureString(loadingText);
            _fonts.standard.DrawText(_spriteBatch, loadingText,
                new Vector2((_screenSize.width - loadingTextSize.X) / 2, _screenSize.height * 0.7f),
                Color.White);

            // Полоса загрузки
            int barWidth = UI.Visual.LoadingBarWidth;
            int barHeight = UI.Visual.LoadingBarHeight;
            int barX = (_screenSize.width - barWidth) / 2;
            int barY = (int)(_screenSize.height * 0.75f);

            // Фон полосы
            _spriteBatch.Draw(_textures.pixel,
                new Rectangle(barX, barY, barWidth, barHeight),
                new Color(40, 40, 40, 200));

            // Прогресс
            _spriteBatch.Draw(_textures.pixel,
                new Rectangle(barX, barY, (int)(barWidth * _loadingProgress), barHeight),
                Color.LightGreen);

            // Рамка полосы
            DrawBorder(new Rectangle(barX - 1, barY - 1, barWidth + 2, barHeight + 2),
                Color.White, 1);

            // Подсказка
            string hintText = "Press any key to skip";
            Vector2 hintSize = _fonts.standard.MeasureString(hintText);
            float hintAlpha = (float)Math.Sin(_transitionTimer * 2) * 0.5f + 0.5f;
            _fonts.standard.DrawText(_spriteBatch, hintText,
                new Vector2((_screenSize.width - hintSize.X) / 2, _screenSize.height * 0.85f),
                Color.White * hintAlpha);
        }

        private void DrawLoadingScreen()
        {
            // Затемненный фон
            _spriteBatch.Draw(_textures.gradient,
                new Rectangle(0, 0, _screenSize.width, _screenSize.height),
                new Color(0, 0, 0, 200));

            // Название уровня
            string levelText = $"Loading Level {_nextLevelId}: {_gameController.Levels[_nextLevelId - 1].Name}";
            Vector2 levelTextSize = _fonts.title.MeasureString(levelText);
            _fonts.title.DrawText(_spriteBatch, levelText,
                new Vector2((_screenSize.width - levelTextSize.X) / 2, _screenSize.height * 0.4f),
                Color.White);

            // Полоса загрузки
            int barWidth = UI.Visual.LoadingBarWidth;
            int barHeight = UI.Visual.LoadingBarHeight * 2;
            int barX = (_screenSize.width - barWidth) / 2;
            int barY = (int)(_screenSize.height * 0.6f);

            // Фон полосы
            _spriteBatch.Draw(_textures.pixel,
                new Rectangle(barX, barY, barWidth, barHeight),
                new Color(40, 40, 40, 200));

            // Прогресс
            _spriteBatch.Draw(_textures.pixel,
                new Rectangle(barX, barY, (int)(barWidth * _loadingProgress), barHeight),
                Color.LightBlue);

            // Рамка полосы
            DrawBorder(new Rectangle(barX - 1, barY - 1, barWidth + 2, barHeight + 2),
                Color.White, 1);

            // Процент загрузки
            string percentText = $"{(int)(_loadingProgress * 100)}%";
            Vector2 percentSize = _fonts.standard.MeasureString(percentText);
            _fonts.standard.DrawText(_spriteBatch, percentText,
                new Vector2((_screenSize.width - percentSize.X) / 2, barY + barHeight + 10),
                Color.White);
        }

        private void DrawTransitionOverlay()
        {
            // Эффект затемнения для смены экранов
            _spriteBatch.Draw(_textures.pixel,
                new Rectangle(0, 0, _screenSize.width, _screenSize.height),
                new Color(0, 0, 0, _fadeAlpha));

            // Эффект "тумана" при появлении главного меню
            if (_transitionState == TransitionState.FadeIn &&
                _nextGameState == Controllers.GameState.MainMenu &&
                _textures.fog != null &&
                !_isSplashScreenActive)
            {
                float fogAlpha = MathHelper.Clamp(_fadeAlpha * 2, 0, 0.8f);

                // Создаем движущийся туман
                for (int y = 0; y < _screenSize.height; y += _textures.fog.Height / 2)
                {
                    // Слой тумана 1
                    _spriteBatch.Draw(_textures.fog,
                        new Rectangle((int)(-_fogOffset * 0.5f) % _screenSize.width - _textures.fog.Width, y,
                            _screenSize.width * 2, _textures.fog.Height / 2),
                        new Color((byte)255, (byte)255, (byte)255, (byte)(fogAlpha * 255)));

                    // Слой тумана 2 (движется быстрее)
                    _spriteBatch.Draw(_textures.fog,
                        new Rectangle((int)(-_fogOffset) % _screenSize.width, y + _textures.fog.Height / 4,
                            _screenSize.width * 2, _textures.fog.Height / 2),
                        new Color((byte)255, (byte)255, (byte)255, (byte)(fogAlpha * 200)));
                }
            }
        }

        private void UpdateButtonVisibility(Controllers.GameState state)
        {
            // Во время переходов не меняем видимость кнопок
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
                    DrawMainMenu();
                    break;
                case Controllers.GameState.BikeSelection:
                    DrawBikeSelectionMenu();
                    break;
                case Controllers.GameState.LevelSelection:
                    DrawLevelSelectionMenu();
                    break;
                case Controllers.GameState.ThemeSelection:
                    DrawThemeSelectionMenu();
                    break;
                case Controllers.GameState.Playing:
                    DrawInfoPanel(gameController);
                    break;
                case Controllers.GameState.Paused:
                    DrawPauseMenu(gameController);
                    break;
                case Controllers.GameState.GameOver:
                    DrawGameOverMenu(gameController);
                    break;
                case Controllers.GameState.LevelComplete:
                    DrawLevelCompleteMenu(gameController);
                    break;
            }
        }

        private Rectangle CreateMenuRect(float widthRatio = UI.Layout.MenuWidthStandard,
                                         float heightRatio = UI.Layout.MenuHeightStandard) =>
            new(
                _screenSize.width / 2 - (int)(_screenSize.width * widthRatio / 2),
                _screenSize.height / 2 - (int)(_screenSize.height * heightRatio / 2),
                (int)(_screenSize.width * widthRatio),
                (int)(_screenSize.height * heightRatio));

        private void DrawMenuBase(
            string title,
            Color borderColor,
            Action<Rectangle>? drawContent = null,
            float widthRatio = UI.Layout.MenuWidthStandard,
            float heightRatio = UI.Layout.MenuHeightStandard)
        {
            DrawAnimatedBackground();

            Rectangle menuRect = CreateMenuRect(widthRatio, heightRatio);

            _spriteBatch.Draw(_textures.pixel, menuRect, new Color(20, 20, 20, 220));
            DrawBorder(menuRect, borderColor, UI.Visual.BorderThick);

            DrawTitle(title, menuRect.Y + UI.Visual.TitleOffset, borderColor);

            drawContent?.Invoke(menuRect);
        }

        private void DrawTitle(string title, int yPosition, Color color)
        {
            Vector2 titleSize = _fonts.title.MeasureString(title);
            float x = (_screenSize.width - titleSize.X) / 2;

            _fonts.title.DrawText(_spriteBatch, title,
                new Vector2(x + UI.Visual.ShadowOffset, yPosition + UI.Visual.ShadowOffset),
                Color.Black * UI.Visual.ShadowOpacity);
            _fonts.title.DrawText(_spriteBatch, title, new Vector2(x, yPosition), color);
        }

        // [Остальные методы DrawMainMenu, DrawBikeSelectionMenu и т.д. остаются без изменений]
        private void DrawMainMenu() =>
            DrawMenuBase("GRAVITY DEFIED", Color.Goldenrod, menuRect =>
            {
                string subtitle = "A Motorcycle Physics Adventure";
                Vector2 subtitleSize = _fonts.standard.MeasureString(subtitle);
                float x = (_screenSize.width - subtitleSize.X) / 2;

                _fonts.standard.DrawText(_spriteBatch, subtitle,
                    new Vector2(x + 1, menuRect.Y + UI.Visual.SubtitleOffset + 1),
                    Color.Black * UI.Visual.ShadowOpacity);
                _fonts.standard.DrawText(_spriteBatch, subtitle,
                    new Vector2(x, menuRect.Y + UI.Visual.SubtitleOffset),
                    Color.LightGoldenrodYellow);

                string themeText = $"Current Theme: {ThemeManager.CurrentTheme.Name}";
                Vector2 themeSize = _fonts.standard.MeasureString(themeText);
                _fonts.standard.DrawText(_spriteBatch, themeText,
                    new Vector2((_screenSize.width - themeSize.X) / 2, menuRect.Y + 160),
                    Color.LightGoldenrodYellow);
            });

        private void DrawBikeSelectionMenu() =>
            DrawMenuBase("SELECT YOUR BIKE", Color.Cyan, menuRect =>
            {
                var bikes = Enum.GetValues<BikeType>();
                string bikeTypeText = bikes[_selection.bikeIndex].ToString();
                Vector2 bikeTypeSize = _fonts.standard.MeasureString(bikeTypeText);
                float x = (_screenSize.width - bikeTypeSize.X) / 2;

                _fonts.standard.DrawText(_spriteBatch, bikeTypeText,
                    new Vector2(x + 1, menuRect.Y + UI.Visual.MainTextOffset + 1),
                    Color.Black * UI.Visual.ShadowOpacity);
                _fonts.standard.DrawText(_spriteBatch, bikeTypeText,
                    new Vector2(x, menuRect.Y + UI.Visual.MainTextOffset),
                    _bikeColors[_selection.colorIndex].color);

                string colorText = $"Color: {_bikeColors[_selection.colorIndex].name}";
                Vector2 colorTextSize = _fonts.standard.MeasureString(colorText);
                _fonts.standard.DrawText(_spriteBatch, colorText,
                    new Vector2((_screenSize.width - colorTextSize.X) / 2, menuRect.Y + UI.Visual.MainTextOffset + 40),
                    _bikeColors[_selection.colorIndex].color);

                DrawBikePreview(menuRect);
            }, UI.Layout.MenuWidthWide);

        private void DrawBikePreview(Rectangle menuRect)
        {
            int previewWidth = (int)(menuRect.Width * 0.4f);
            int previewHeight = (int)(menuRect.Height * 0.3f);
            int previewX = menuRect.X + (menuRect.Width - previewWidth) / 2;
            int previewY = menuRect.Y + (int)(menuRect.Height * 0.3f);

            Rectangle previewRect = new Rectangle(previewX, previewY, previewWidth, previewHeight);
            _spriteBatch.Draw(_textures.pixel, previewRect, new Color(30, 30, 30, 200));
            DrawBorder(previewRect, Color.LightGray, UI.Visual.BorderThin);

            string previewText = "Bike Preview";
            Vector2 previewTextSize = _fonts.standard.MeasureString(previewText);
            _fonts.standard.DrawText(_spriteBatch, previewText,
                new Vector2(previewX + (previewWidth - previewTextSize.X) / 2, previewY + 10),
                Color.LightGray);

            var bikeType = Enum.GetValues<BikeType>()[_selection.bikeIndex];
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
                _fonts.standard.DrawText(_spriteBatch, statText, new Vector2(x, y + i * 30), Color.Yellow);
            }
        }

        private void DrawLevelSelectionMenu() =>
            DrawMenuBase("SELECT LEVEL", Color.Magenta, menuRect =>
            {
                string levelText = $"Level {_selection.levelIndex + 1}: {_gameController.Levels[_selection.levelIndex].Name}";
                Vector2 levelSize = _fonts.standard.MeasureString(levelText);
                float x = (_screenSize.width - levelSize.X) / 2;

                _fonts.standard.DrawText(_spriteBatch, levelText,
                    new Vector2(x + 1, menuRect.Y + UI.Visual.MainTextOffset + 1),
                    Color.Black * UI.Visual.ShadowOpacity);
                _fonts.standard.DrawText(_spriteBatch, levelText,
                    new Vector2(x, menuRect.Y + UI.Visual.MainTextOffset),
                    Color.Yellow);

                string difficultyText = $"Difficulty: {GetLevelDifficulty(_selection.levelIndex + 1)}";
                Vector2 difficultyPos = new Vector2(
                    (_screenSize.width - _fonts.standard.MeasureString(difficultyText).X) / 2,
                    menuRect.Y + 150);
                _fonts.standard.DrawText(_spriteBatch, difficultyText, difficultyPos, Color.Yellow);
            }, UI.Layout.MenuWidthWide);

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
                string themeName = themes[_selection.themeIndex].Name;
                Vector2 themeNameSize = _fonts.standard.MeasureString(themeName);

                _fonts.standard.DrawText(_spriteBatch, themeName,
                    new Vector2((_screenSize.width - themeNameSize.X) / 2 + 1, menuRect.Y + 100 + 1),
                    Color.Black * UI.Visual.ShadowOpacity);
                _fonts.standard.DrawText(_spriteBatch, themeName,
                    new Vector2((_screenSize.width - themeNameSize.X) / 2, menuRect.Y + 100),
                    Color.White);

                string themeDesc = themes[_selection.themeIndex].Description;
                Vector2 descSize = _fonts.standard.MeasureString(themeDesc);
                _fonts.standard.DrawText(_spriteBatch, themeDesc,
                    new Vector2((_screenSize.width - descSize.X) / 2, menuRect.Y + 140),
                    Color.LightGray);

                DrawThemePreview(menuRect);
            });

        private void DrawThemePreview(Rectangle menuRect)
        {
            var theme = ThemeManager.AvailableThemes[_selection.themeIndex];
            int previewY = menuRect.Y + 180;
            int swatchSize = 30;
            int margin = 10;
            int totalWidth = 6 * (swatchSize + margin);
            int startX = (_screenSize.width - totalWidth) / 2;

            DrawColorSwatch(startX, previewY, swatchSize, theme.BackgroundColor, "BG");
            DrawColorSwatch(startX + swatchSize + margin, previewY, swatchSize, theme.TerrainColor, "Terrain");
            DrawColorSwatch(startX + 2 * (swatchSize + margin), previewY, swatchSize, theme.SafeZoneColor, "Safe");
            DrawColorSwatch(startX + 3 * (swatchSize + margin), previewY, swatchSize, theme.BikeColors[SkeletonLineType.MainFrame], "Bike");
            DrawColorSwatch(startX + 4 * (swatchSize + margin), previewY, swatchSize, theme.WheelFill, "Wheel");
            DrawColorSwatch(startX + 5 * (swatchSize + margin), previewY, swatchSize, theme.ShadowColor, "Shadow");
        }

        private void DrawColorSwatch(int x, int y, int size, Color color, string label)
        {
            _spriteBatch.Draw(_textures.pixel, new Rectangle(x, y, size, size), color);
            _fonts.standard.DrawText(_spriteBatch, label, new Vector2(x, y + size + 2), Color.LightGray);
        }

        private void DrawInfoPanel(Controllers.GameController gameController)
        {
            string levelName = gameController.CurrentLevel?.Name ?? "Level";
            string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
            string directionText = gameController.Motorcycle.Direction == 1 ? "Forward" : "Backward";

            Rectangle infoPanelRect = new Rectangle(10, 10, (int)(_screenSize.width * 0.25f), (int)(_screenSize.height * 0.15f));
            _spriteBatch.Draw(_textures.pixel, infoPanelRect, new Color(0, 0, 0, 150));
            DrawBorder(infoPanelRect, Color.LightBlue, UI.Visual.BorderThin);

            _fonts.standard.DrawText(_spriteBatch, $"Level: {levelName}", new Vector2(20, 20), Color.White);
            _fonts.standard.DrawText(_spriteBatch, timeText, new Vector2(20, 45), Color.White);
            _fonts.standard.DrawText(_spriteBatch, $"Direction: {directionText}", new Vector2(20, 70), Color.White);

            string controlsText = "W: Forward | S: Backward | Space: Brake | A/D: Lean | ESC: Pause";
            Vector2 controlsSize = _fonts.standard.MeasureString(controlsText);
            _fonts.standard.DrawText(_spriteBatch, controlsText,
                new Vector2((_screenSize.width - controlsSize.X) / 2, _screenSize.height - 40),
                Color.LightBlue);
        }

        private void DrawPauseMenu(Controllers.GameController gameController) =>
            DrawMenuBase("GAME PAUSED", Color.LightBlue, menuRect =>
            {
                string resumeText = "Press ESC to Resume";
                Vector2 resumeSize = _fonts.standard.MeasureString(resumeText);
                _fonts.standard.DrawText(_spriteBatch, resumeText,
                    new Vector2((_screenSize.width - resumeSize.X) / 2, menuRect.Y + 120),
                    Color.White);

                string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
                string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
                _fonts.standard.DrawText(_spriteBatch, levelText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.6f)),
                    Color.LightGray);
                _fonts.standard.DrawText(_spriteBatch, timeText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.6f) + 25),
                    Color.LightGray);
            });

        private void DrawGameOverMenu(Controllers.GameController gameController) =>
            DrawMenuBase("GAME OVER", Color.Red, menuRect =>
            {
                string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
                string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
                _fonts.standard.DrawText(_spriteBatch, levelText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.4f)),
                    Color.LightGray);
                _fonts.standard.DrawText(_spriteBatch, timeText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.4f) + 25),
                    Color.LightGray);
            });

        private void DrawLevelCompleteMenu(Controllers.GameController gameController) =>
            DrawMenuBase("LEVEL COMPLETE!", Color.LimeGreen, menuRect =>
            {
                string timeText = $"Time: {gameController.GameTime.Minutes:00}:{gameController.GameTime.Seconds:00}";
                Vector2 timeSize = _fonts.standard.MeasureString(timeText);
                _fonts.standard.DrawText(_spriteBatch, timeText,
                    new Vector2((_screenSize.width - timeSize.X) / 2, menuRect.Y + 100),
                    Color.White);

                string levelText = $"Level: {gameController.CurrentLevel?.Name ?? "Unknown"}";
                _fonts.standard.DrawText(_spriteBatch, levelText,
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

            _spriteBatch.Draw(_textures.pixel,
                new Rectangle(scaledBounds.X + 4, scaledBounds.Y + 4, scaledBounds.Width, scaledBounds.Height),
                Color.Black * 0.3f);

            _spriteBatch.Draw(_textures.pixel, scaledBounds, buttonColor);
            DrawBorder(scaledBounds, Color.White, UI.Visual.BorderThin);

            if (!string.IsNullOrEmpty(button.Text))
            {
                Vector2 textSize = _fonts.standard.MeasureString(button.Text);
                Vector2 textPosition = new Vector2(
                    scaledBounds.X + (scaledBounds.Width - textSize.X) / 2,
                    scaledBounds.Y + (scaledBounds.Height - textSize.Y) / 2
                );

                _fonts.standard.DrawText(_spriteBatch, button.Text,
                    textPosition + new Vector2(1, 1),
                    Color.Black * 0.5f);

                _fonts.standard.DrawText(_spriteBatch, button.Text, textPosition, Color.White);
            }
        }

        private void DrawBorder(Rectangle rectangle, Color color, int thickness)
        {
            _spriteBatch.Draw(_textures.pixel, new Rectangle(rectangle.X, rectangle.Y, rectangle.Width, thickness), color);
            _spriteBatch.Draw(_textures.pixel, new Rectangle(rectangle.X, rectangle.Y + rectangle.Height - thickness, rectangle.Width, thickness), color);
            _spriteBatch.Draw(_textures.pixel, new Rectangle(rectangle.X, rectangle.Y, thickness, rectangle.Height), color);
            _spriteBatch.Draw(_textures.pixel, new Rectangle(rectangle.X + rectangle.Width - thickness, rectangle.Y, thickness, rectangle.Height), color);
        }

        private void DrawAnimatedBackground()
        {
            _spriteBatch.Draw(_textures.gradient, new Rectangle(0, 0, _screenSize.width, _screenSize.height), ThemeManager.CurrentTheme.BackgroundColor);
            _spriteBatch.Draw(_textures.pixel, new Rectangle(0, 0, _screenSize.width, _screenSize.height), new Color(0, 0, 0, 100));

            for (int i = 0; i < 5; i++)
            {
                float x = (_backgroundOffset + i * _screenSize.width / 5) % _screenSize.width;
                _spriteBatch.Draw(_textures.pixel,
                    new Rectangle((int)x, _screenSize.height / 4 + i * 20, 100, 20),
                    Color.White * 0.2f);
            }
        }

        private void DrawTooltip()
        {
            Vector2 tooltipSize = _fonts.standard.MeasureString(_tooltip.text);
            Rectangle tooltipRect = new Rectangle(
                (int)_tooltip.position.X,
                (int)_tooltip.position.Y,
                (int)tooltipSize.X + UI.Visual.Padding * 2,
                (int)tooltipSize.Y + UI.Visual.Padding);

            _spriteBatch.Draw(_textures.pixel, tooltipRect, Color.Black * 0.8f);
            _fonts.standard.DrawText(_spriteBatch, _tooltip.text,
                _tooltip.position + new Vector2(UI.Visual.Padding, UI.Visual.Padding / 2),
                Color.White);
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

        private void StartGameWithTransition(int levelId)
        {
            StartTransition(TransitionState.FadeOut, Controllers.GameState.Playing, levelId);
            Info("UIController", $"Starting level {levelId} with transition");
        }
        #endregion

        public void ShowMessage(string message) => Debug("UIController", $"Message: {message}");
    }
}