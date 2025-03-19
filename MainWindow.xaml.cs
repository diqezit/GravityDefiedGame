using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Diagnostics;
using System.Windows.Threading;
using GravityDefiedGame.Models;
using GravityDefiedGame.Controllers;
using GravityDefiedGame.Utilities;
using GravityDefiedGame.Views;
using System.Windows.Media;

namespace GravityDefiedGame
{
    public partial class MainWindow : Window
    {
        private static class Constants
        {
            public const double MaxFrameTime = 0.1;
            public const double NotificationTopPosition = 1.0 / 3.0;
            public const int NotificationDuration = 3;
            public const int TimerIntervalMs = 16;
            public const int NotificationZIndex = 100;
            public const int NotificationFontSize = 24;
            public const int LevelButtonWidth = 150;
            public const int LevelButtonHeight = 150;
            public const int LevelButtonMargin = 10;
        }

        private readonly GameController _gameController;
        private Renderer _renderer = null!;
        private Camera _camera = null!;
        private readonly DispatcherTimer _gameLogicTimer = new();
        private readonly DispatcherTimer _renderTimer = new();
        private readonly Stopwatch _gameStopwatch = new();
        private TimeSpan _lastUpdateTime;

        private readonly InputState _inputState = new();
        private TextBlock _notificationText = null!;
        private DispatcherTimer _notificationTimer = null!;
        private GameState _currentGameState;

        public MainWindow()
        {
            Logger.Info("MainWindow", "Application starting...");
            InitializeComponent();
            _gameController = new();
            _gameController.GameEvent += GameController_GameEvent;
            RegisterEventHandlers();
        }

        private void RegisterEventHandlers()
        {
            Loaded += MainWindow_Loaded;
            Closing += (_, _) => Logger.Info("MainWindow", "Application closing");
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            Logger.Info("MainWindow", "Window loaded");
            InitializeGameComponents();
            InitializeTimers();
            InitializeUI();
            StartGame(1);
        }

        private void InitializeGameComponents()
        {
            _camera = new(GameCanvas);
            _renderer = new(GameCanvas, _gameController, _camera);
        }

        private void InitializeTimers()
        {
            ConfigureTimer(_gameLogicTimer, GameLogicTimer_Tick);
            ConfigureTimer(_renderTimer, RenderTimer_Tick);
        }

        private void ConfigureTimer(DispatcherTimer timer, EventHandler handler)
        {
            timer.Interval = TimeSpan.FromMilliseconds(Constants.TimerIntervalMs);
            timer.Tick += handler;
        }

        private void InitializeUI()
        {
            InitializeNotifications();
            LoadLevels();
        }

        private void InitializeNotifications()
        {
            CreateNotificationText();
            ConfigureNotificationTimer();
        }

        private void CreateNotificationText()
        {
            _notificationText = new()
            {
                FontSize = Constants.NotificationFontSize,
                FontWeight = FontWeights.Bold,
                Foreground = Brushes.White,
                HorizontalAlignment = HorizontalAlignment.Center,
                VerticalAlignment = VerticalAlignment.Center,
                TextAlignment = TextAlignment.Center,
                Visibility = Visibility.Collapsed
            };

            Canvas.SetZIndex(_notificationText, Constants.NotificationZIndex);
            GameCanvas.Children.Add(_notificationText);
        }

        private void ConfigureNotificationTimer()
        {
            _notificationTimer = new() { Interval = TimeSpan.FromSeconds(Constants.NotificationDuration) };
            _notificationTimer.Tick += (_, _) =>
            {
                _notificationText.Visibility = Visibility.Collapsed;
                _notificationTimer.Stop();
            };
        }

        private void LoadLevels()
        {
            _gameController.LoadLevels();
            CreateLevelButtons();
        }

        private void CreateLevelButtons()
        {
            LevelsPanel.Children.Clear();
            foreach (var level in _gameController.Levels)
            {
                var levelButton = CreateLevelButton(level);
                LevelsPanel.Children.Add(levelButton);
            }
        }

        private void ShowLevelSelect()
        {
            MenuOverlay.Visibility = Visibility.Collapsed;
            LevelSelectOverlay.Visibility = Visibility.Visible;
            _gameController.LoadLevels();
            CreateLevelButtons();
        }

        private Button CreateLevelButton(Level level)
        {
            var button = new Button
            {
                Content = level.Name,
                Width = Constants.LevelButtonWidth,
                Height = Constants.LevelButtonHeight,
                Margin = new(Constants.LevelButtonMargin),
                Style = (Style)Resources["RoundButton"],
                Tag = level.Id
            };

            button.Click += LevelButton_Click;
            return button;
        }

        private void StartGame(int levelId)
        {
            Logger.Info("MainWindow", $"Starting level {levelId}");

            _gameController.StartLevel(levelId);
            _currentGameState = GameState.Playing;
            UpdateLevelNameDisplay();

            _camera.CenterOn(_gameController.Motorcycle.Position);
            StartGameTimers();
            HideMenus();
            Focus();
        }

        private void UpdateLevelNameDisplay()
        {
            LevelNameText.Text = _gameController.CurrentLevel?.Name ?? "Уровень";
        }

        private void StartGameTimers()
        {
            _gameStopwatch.Restart();
            _lastUpdateTime = _gameStopwatch.Elapsed;
            _gameLogicTimer.Start();
            _renderTimer.Start();
        }

        private void HideMenus()
        {
            MenuOverlay.Visibility = Visibility.Collapsed;
            LevelSelectOverlay.Visibility = Visibility.Collapsed;
            BikeSelectOverlay.Visibility = Visibility.Collapsed;
        }

        private void GameLogicTimer_Tick(object sender, EventArgs e)
        {
            try
            {
                UpdateGameLogic();
            }
            catch (Exception ex)
            {
                LogError("Error in game logic loop", ex);
            }
        }

        private void LogError(string message, Exception ex)
        {
            Logger.Error("MainWindow", $"{message}: {ex.Message}");
            Logger.Exception("MainWindow", ex);
        }

        private void UpdateGameLogic()
        {
            double deltaTime = CalculateDeltaTime();
            _gameController.Update(deltaTime);
            UpdateTimeDisplay();
            CheckGameConditions();
        }

        private double CalculateDeltaTime()
        {
            var currentTime = _gameStopwatch.Elapsed;
            var deltaTime = (currentTime - _lastUpdateTime).TotalSeconds;
            _lastUpdateTime = currentTime;

            return Math.Min(deltaTime, Constants.MaxFrameTime);
        }

        private void UpdateTimeDisplay()
        {
            TimeText.Text = $"{_gameStopwatch.Elapsed.Minutes:00}:{_gameStopwatch.Elapsed.Seconds:00}";
        }

        private void RenderTimer_Tick(object sender, EventArgs e)
        {
            try
            {
                UpdateRender();
            }
            catch (Exception ex)
            {
                LogError("Error in render loop", ex);
            }
        }

        private void UpdateRender()
        {
            _camera.Update(_gameController.Motorcycle.Position);
            _renderer.Render();
        }

        private void CheckGameConditions()
        {
            if (_gameController.IsLevelComplete && _currentGameState == GameState.Playing)
                LevelComplete();

            if (_gameController.IsGameOver && _currentGameState == GameState.Playing)
                GameOver();
        }

        private void LevelComplete()
        {
            _currentGameState = GameState.LevelComplete;
            StopGameTimers();
            ShowMenu("УРОВЕНЬ ПРОЙДЕН!");
            Logger.Info("MainWindow", "Level completed");
        }

        private void GameOver()
        {
            _currentGameState = GameState.GameOver;
            StopGameTimers();
            ShowMenu("ПОРАЖЕНИЕ");
            Logger.Info("MainWindow", "Game over");
        }

        private void PauseGame()
        {
            _currentGameState = GameState.Paused;
            StopGameTimers();
            _gameController.PauseGame();
            ShowMenu("ПАУЗА");
            Logger.Info("MainWindow", "Game paused");
        }

        private void ShowMenu(string title)
        {
            MenuTitle.Text = title;
            MenuOverlay.Visibility = Visibility.Visible;
        }

        private void StopGameTimers()
        {
            _gameLogicTimer.Stop();
            _renderTimer.Stop();
        }

        private void GameController_GameEvent(object? sender, GameEventArgs e)
        {
            switch (e.Type)
            {
                case GameEventType.LevelComplete:
                    ShowNotification("Уровень пройден!");
                    break;
                case GameEventType.GameOver:
                    ShowNotification("Игра окончена!");
                    break;
                case GameEventType.BikeChanged:
                    ShowNotification(e.Message);
                    break;
            }
        }

        private void ShowNotification(string message)
        {
            _notificationText.Text = message;
            _notificationText.Visibility = Visibility.Visible;
            PositionNotification();
            RestartNotificationTimer();
            Logger.Debug("MainWindow", $"Showing notification: {message}");
        }

        private void PositionNotification()
        {
            Canvas.SetLeft(_notificationText, (GameCanvas.ActualWidth - _notificationText.ActualWidth) / 2);
            Canvas.SetTop(_notificationText, GameCanvas.ActualHeight * Constants.NotificationTopPosition);
        }

        private void RestartNotificationTimer()
        {
            _notificationTimer.Stop();
            _notificationTimer.Start();
        }

        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            if (_currentGameState != GameState.Playing)
            {
                HandleNonPlayingKeyDown(e);
                return;
            }

            HandlePlayingKeyDown(e);
        }

        private void HandleNonPlayingKeyDown(KeyEventArgs e)
        {
            if (e.Key == Key.Escape)
            {
                if (MenuOverlay.Visibility == Visibility.Visible)
                    ContinueButton_Click(this, e);
                else if (BikeSelectOverlay.Visibility == Visibility.Visible)
                    BikeSelectBackButton_Click(this, e);
                else if (LevelSelectOverlay.Visibility == Visibility.Visible)
                    BackButton_Click(this, e);
            }
        }

        private void HandlePlayingKeyDown(KeyEventArgs e)
        {
            switch (e.Key)
            {
                case Key.W:
                    HandleThrottleDown();
                    break;
                case Key.S:
                    HandleBrakeDown();
                    break;
                case Key.A:
                    HandleLeanLeftDown();
                    break;
                case Key.D:
                    HandleLeanRightDown();
                    break;
                case Key.R:
                    RestartButton_Click(this, e);
                    break;
                case Key.Escape:
                    PauseGame();
                    break;
            }
        }

        private void HandleThrottleDown()
        {
            _gameController.HandleKeyDown("W");
            _inputState.IsThrottlePressed = true;
        }

        private void HandleBrakeDown()
        {
            _gameController.HandleKeyDown("S");
            _inputState.IsBrakePressed = true;
        }

        private void HandleLeanLeftDown()
        {
            _gameController.HandleKeyDown("A");
            _inputState.IsLeaningLeft = true;
        }

        private void HandleLeanRightDown()
        {
            _gameController.HandleKeyDown("D");
            _inputState.IsLeaningRight = true;
        }

        private void Window_KeyUp(object sender, KeyEventArgs e)
        {
            switch (e.Key)
            {
                case Key.W:
                    HandleThrottleUp();
                    break;
                case Key.S:
                    HandleBrakeUp();
                    break;
                case Key.A:
                    HandleLeanLeftUp();
                    break;
                case Key.D:
                    HandleLeanRightUp();
                    break;
            }
        }

        private void HandleThrottleUp()
        {
            _gameController.HandleKeyUp("W");
            _inputState.IsThrottlePressed = false;
        }

        private void HandleBrakeUp()
        {
            _gameController.HandleKeyUp("S");
            _inputState.IsBrakePressed = false;
        }

        private void HandleLeanLeftUp()
        {
            _gameController.HandleKeyUp("A");
            _inputState.IsLeaningLeft = false;
        }

        private void HandleLeanRightUp()
        {
            _gameController.HandleKeyUp("D");
            _inputState.IsLeaningRight = false;
        }

        private void LevelButton_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button button && button.Tag is int levelId)
                StartGame(levelId);
        }

        private void ContinueButton_Click(object sender, RoutedEventArgs e)
        {
            switch (_currentGameState)
            {
                case GameState.Paused:
                    ResumeGame();
                    break;
                case GameState.LevelComplete:
                    StartNextLevel();
                    break;
                case GameState.GameOver:
                    RestartCurrentLevel();
                    break;
            }
        }

        private void ResumeGame()
        {
            _currentGameState = GameState.Playing;
            MenuOverlay.Visibility = Visibility.Collapsed;
            _gameLogicTimer.Start();
            _renderTimer.Start();
            _gameController.ResumeGame();
            _lastUpdateTime = _gameStopwatch.Elapsed;
            Focus();
        }

        private void StartNextLevel()
        {
            int nextLevelId = _gameController.CurrentLevel?.Id + 1 ?? 1;

            if (nextLevelId <= _gameController.Levels.Count)
                StartGame(nextLevelId);
            else
                ShowLevelSelect();
        }

        private void RestartCurrentLevel()
        {
            int currentLevelId = _gameController.CurrentLevel?.Id ?? 1;
            StartGame(currentLevelId);
        }

        private void RestartButton_Click(object sender, RoutedEventArgs e)
        {
            RestartCurrentLevel();
        }

        private void LevelSelectButton_Click(object sender, RoutedEventArgs e)
        {
            ShowLevelSelect();
        }


        private void BackButton_Click(object sender, RoutedEventArgs e)
        {
            LevelSelectOverlay.Visibility = Visibility.Collapsed;
            MenuOverlay.Visibility = Visibility.Visible;
        }

        private void ExitButton_Click(object sender, RoutedEventArgs e)
        {
            Application.Current.Shutdown();
        }

        private void BikeSelectButton_Click(object sender, RoutedEventArgs e)
        {
            MenuOverlay.Visibility = Visibility.Collapsed;
            BikeSelectOverlay.Visibility = Visibility.Visible;
        }

        private void BikeTypeButton_Click(object sender, RoutedEventArgs e)
        {
            if (sender is Button button && button.Tag is string bikeTypeStr)
            {
                if (Enum.TryParse<BikeType>(bikeTypeStr, out BikeType bikeType))
                {
                    string bikeName = GetBikeTypeName(bikeType);
                    _gameController.SetBikeType(bikeType);
                    ShowNotification($"Выбран байк: {bikeName}");
                    Logger.Info("MainWindow", $"Bike type changed to {bikeType}");

                    BikeSelectOverlay.Visibility = Visibility.Collapsed;

                    if (_currentGameState == GameState.Paused)
                    {
                        MenuOverlay.Visibility = Visibility.Visible;
                    }
                    else
                    {
                        ResumeGame();
                    }
                }
            }
        }

        private void BikeSelectBackButton_Click(object sender, RoutedEventArgs e)
        {
            BikeSelectOverlay.Visibility = Visibility.Collapsed;
            MenuOverlay.Visibility = Visibility.Visible;
        }

        private string GetBikeTypeName(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => "Стандартный",
            BikeType.Sport => "Спортивный",
            BikeType.OffRoad => "Внедорожный",
            _ => bikeType.ToString()
        };
    }

    public enum GameState
    {
        Playing,
        Paused,
        GameOver,
        LevelComplete
    }

    internal class InputState
    {
        public bool IsThrottlePressed { get; set; }
        public bool IsBrakePressed { get; set; }
        public bool IsLeaningLeft { get; set; }
        public bool IsLeaningRight { get; set; }
    }
}