using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Diagnostics;
using System.Windows.Threading;
using System.Threading;
using GravityDefiedGame.Models;
using GravityDefiedGame.Controllers;
using GravityDefiedGame.Utilities;
using GravityDefiedGame.Views;
using System.Windows.Media;
using System.Windows.Data;
using static GravityDefiedGame.Utilities.Logger;
using System.ComponentModel.DataAnnotations;

namespace GravityDefiedGame
{
    public partial class MainWindow : Window
    {
        private static class Constants
        {
            // Timing and performance
            public const double MaxFrameTime = 0.1;
            public const int TimerIntervalMs = 16;

            // UI positioning and styling
            public const double NotificationTopPosition = 1.0 / 3.0;
            public const int NotificationDuration = 3;
            public const int NotificationZIndex = 100;
            public const int NotificationFontSize = 24;

            // Level select UI
            public const int LevelButtonWidth = 150;
            public const int LevelButtonHeight = 150;
            public const int LevelButtonMargin = 10;
        }

        // Core game components
        private readonly GameController _gameController;
        private Renderer _renderer = null!;
        private Camera _camera = null!;

        // Game timing
        private readonly DispatcherTimer _gameLogicTimer = new();
        private readonly DispatcherTimer _renderTimer = new();
        private readonly System.Diagnostics.Stopwatch _gameStopwatch = new();
        private TimeSpan _lastUpdateTime;
        private CancellationTokenSource? _gameLoopCts;

        // UI state
        private DispatcherTimer _notificationTimer = null!;
        private GameState _currentGameState;

        // Commands
        [Required] public ICommand ContinueCommand { get; private set; } = null!;
        [Required] public ICommand RestartCommand { get; private set; } = null!;
        [Required] public ICommand BikeSelectCommand { get; private set; } = null!;
        [Required] public ICommand LevelSelectCommand { get; private set; } = null!;
        [Required] public ICommand ExitCommand { get; private set; } = null!;
        [Required] public ICommand BackCommand { get; private set; } = null!;
        [Required] public ICommand BikeSelectBackCommand { get; private set; } = null!;
        [Required] public ICommand BikeTypeCommand { get; private set; } = null!;
        [Required] public ICommand LevelButtonCommand { get; private set; } = null!;

        public MainWindow()
        {
            Info("MainWindow", "Application starting...");
            InitializeComponent();
            _gameController = new();
            _gameController.GameEvent += GameController_GameEvent;
            InitializeCommands();
            RegisterEventHandlers();
        }

        private void InitializeCommands()
        {
            ContinueCommand = new RelayCommand(ExecuteContinue);
            RestartCommand = new RelayCommand(ExecuteRestart);
            BikeSelectCommand = new RelayCommand(ExecuteBikeSelect);
            LevelSelectCommand = new RelayCommand(ExecuteLevelSelect);
            ExitCommand = new RelayCommand(ExecuteExit);
            BackCommand = new RelayCommand(ExecuteBack);
            BikeSelectBackCommand = new RelayCommand(ExecuteBikeSelectBack);
            BikeTypeCommand = new RelayCommand<string>(ExecuteBikeType);
            LevelButtonCommand = new RelayCommand<int>(ExecuteLevelButton);
        }

        private void ExecuteContinue()
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

        private void ExecuteRestart() => RestartCurrentLevel();
        private void ExecuteBikeSelect() => ShowBikeSelect();
        private void ExecuteLevelSelect() => ShowLevelSelect();
        private void ExecuteExit() => Application.Current.Shutdown();
        private void ExecuteBack() => ShowMainMenu();
        private void ExecuteBikeSelectBack() => ShowMainMenu();

        private void ExecuteBikeType(string bikeTypeStr)
        {
            if (Enum.TryParse<BikeType>(bikeTypeStr, out var bikeType))
            {
                string bikeName = GetBikeTypeName(bikeType);
                _gameController.SetBikeType(bikeType);
                ShowNotification($"Selected bike: {bikeName}");
                Info("MainWindow", $"Bike type changed to {bikeType}");

                BikeSelectOverlay.Visibility = Visibility.Collapsed;

                if (_currentGameState == GameState.Paused)
                    MenuOverlay.Visibility = Visibility.Visible;
                else
                    ResumeGame();
            }
        }

        private void ExecuteLevelButton(int levelId) => StartGame(levelId);

        private void RegisterEventHandlers()
        {
            Loaded += MainWindow_Loaded;
            Closing += (_, _) =>
            {
                _gameLoopCts?.Cancel();
                Info("MainWindow", "Application closing");
            };
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e) =>
            Log("MainWindow", "initializing window", () =>
            {
                InitializeWindow();
                InitializeGameComponents();
                InitializeTimers();
                InitializeUI();
                StartGame(1);
                Info("MainWindow", "Window loaded and game started");
            });

        #region Initialization

        private void InitializeWindow()
        {
            var minimizeButton = Template.FindName("MinimizeButton", this) as Button;
            var closeButton = Template.FindName("CloseButton", this) as Button;

            if (minimizeButton != null)
                minimizeButton.Click += (s, e) => WindowState = WindowState.Minimized;

            if (closeButton != null)
                closeButton.Click += (s, e) => Application.Current.Shutdown();

            MouseLeftButtonDown += (s, e) =>
            {
                if (e.ButtonState == MouseButtonState.Pressed)
                    DragMove();
            };
        }

        private void InitializeGameComponents()
        {
            _camera = new(GameCanvas);
            _renderer = new(GameCanvas, _gameController, _camera);
            _gameLoopCts = new CancellationTokenSource();
        }

        private void InitializeTimers()
        {
            _gameLogicTimer.Interval = TimeSpan.FromMilliseconds(Constants.TimerIntervalMs);
            _renderTimer.Interval = TimeSpan.FromMilliseconds(Constants.TimerIntervalMs);

            _gameLogicTimer.Tick += GameLogicTimer_Tick;
            _renderTimer.Tick += RenderTimer_Tick;
        }

        private void InitializeUI()
        {
            InitializeNotifications();
            LoadLevels();
        }

        private void InitializeNotifications()
        {
            _notificationTimer = new() { Interval = TimeSpan.FromSeconds(Constants.NotificationDuration) };
        }

        private void LoadLevels() =>
            Log("MainWindow", "loading levels", () =>
            {
                _gameController.LoadLevels();
                CreateLevelButtons();
            });

        private void CreateLevelButtons()
        {
            LevelsPanel.Children.Clear();

            foreach (var level in _gameController.Levels)
            {
                var button = new Button
                {
                    Content = level.Name,
                    Width = Constants.LevelButtonWidth,
                    Height = Constants.LevelButtonHeight,
                    Margin = new(Constants.LevelButtonMargin),
                    Style = (Style)FindResource("LevelButton"),
                    Tag = level.Id
                };

                button.SetBinding(Button.CommandProperty, new Binding("LevelButtonCommand")
                {
                    RelativeSource = new RelativeSource(RelativeSourceMode.FindAncestor, typeof(Window), 1)
                });
                button.SetBinding(Button.CommandParameterProperty, new Binding("Tag")
                {
                    RelativeSource = new RelativeSource(RelativeSourceMode.Self)
                });

                LevelsPanel.Children.Add(button);
            }
        }

        #endregion

        #region Game State Management

        private void StartGame(int levelId) =>
            Log("MainWindow", $"starting level {levelId}", () =>
            {
                // Reset cancellation token
                _gameLoopCts?.Cancel();
                _gameLoopCts = new CancellationTokenSource();

                // Initialize game state
                _gameController.StartLevel(levelId);
                _currentGameState = GameState.Playing;
                UpdateLevelNameDisplay();
                _camera.CenterOn(_gameController.Motorcycle.Position);

                // Start timers
                StartGameTimers();

                // Hide menus
                HideMenus();

                // Set focus for keyboard input
                Focus();

                Info("MainWindow", $"Level {levelId} started");
            });

        private void UpdateLevelNameDisplay() =>
            LevelNameText.Text = _gameController.CurrentLevel?.Name ?? "Level";

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

        private void PauseGame() =>
            Log("MainWindow", "pausing game", () =>
            {
                if (_currentGameState != GameState.Playing)
                    return;

                _currentGameState = GameState.Paused;
                StopGameTimers();
                _gameController.PauseGame();
                ShowMenu("ПАУЗА");
                Info("MainWindow", "Game paused");
            });

        private void ResumeGame() =>
            Log("MainWindow", "resuming game", () =>
            {
                _currentGameState = GameState.Playing;
                MenuOverlay.Visibility = Visibility.Collapsed;
                _gameLogicTimer.Start();
                _renderTimer.Start();
                _gameController.ResumeGame();
                _lastUpdateTime = _gameStopwatch.Elapsed;
                Focus();
                Info("MainWindow", "Game resumed");
            });

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
                ShowLevelSelect();
                Info("MainWindow", "All levels completed");
            }
        }

        private void StopGameTimers()
        {
            _gameLogicTimer.Stop();
            _renderTimer.Stop();
        }

        private void LevelComplete() =>
            Log("MainWindow", "handling level complete", () =>
            {
                _currentGameState = GameState.LevelComplete;
                StopGameTimers();
                ShowMenu("УРОВЕНЬ ПРОЙДЕН!");
                Info("MainWindow", "Level completed");
            });

        private void GameOver() =>
            Log("MainWindow", "handling game over", () =>
            {
                _currentGameState = GameState.GameOver;
                StopGameTimers();
                ShowMenu("ИГРА ОКОНЧЕНА");
                Info("MainWindow", "Game over");
            });

        #endregion

        #region UI Management

        private void ShowMenu(string title)
        {
            MenuTitle.Text = title;
            MenuOverlay.Visibility = Visibility.Visible;
        }

        private void ShowMainMenu()
        {
            BikeSelectOverlay.Visibility = Visibility.Collapsed;
            LevelSelectOverlay.Visibility = Visibility.Collapsed;
            MenuOverlay.Visibility = Visibility.Visible;
        }

        private void ShowBikeSelect()
        {
            MenuOverlay.Visibility = Visibility.Collapsed;
            BikeSelectOverlay.Visibility = Visibility.Visible;
        }

        private void ShowLevelSelect()
        {
            MenuOverlay.Visibility = Visibility.Collapsed;
            LevelSelectOverlay.Visibility = Visibility.Visible;
            _gameController.LoadLevels();
            CreateLevelButtons();
        }

        private void ShowNotification(string message)
        {
            NotificationText.Text = message;
            NotificationPanel.Visibility = Visibility.Visible;
            RestartNotificationTimer();
        }

        private void RestartNotificationTimer()
        {
            _notificationTimer.Stop();
            _notificationTimer.Tick += (_, _) =>
            {
                NotificationPanel.Visibility = Visibility.Collapsed;
                _notificationTimer.Stop();
            };
            _notificationTimer.Start();
        }

        #endregion

        #region Game Loop

        private void GameLogicTimer_Tick(object? sender, EventArgs e)
        {
            try
            {
                UpdateGameLogic();
            }
            catch (Exception ex)
            {
                Log("MainWindow", "game logic error", () =>
                {
                    Error("MainWindow", $"Error in game logic loop: {ex.Message}");
                    Exception("MainWindow", ex);
                });
            }
        }

        private void UpdateGameLogic()
        {
            double deltaTime = CalculateDeltaTime();
            _gameController.Update(deltaTime, _gameLoopCts!.Token);
            UpdateTimeDisplay();
            CheckGameConditions();
        }

        private double CalculateDeltaTime()
        {
            var currentTime = _gameStopwatch.Elapsed;
            var deltaTime = (currentTime - _lastUpdateTime).TotalSeconds;
            _lastUpdateTime = currentTime;

            // Cap maximum delta time to prevent physics issues
            return Math.Min(deltaTime, Constants.MaxFrameTime);
        }

        private void UpdateTimeDisplay() =>
            TimeText.Text = $"{_gameStopwatch.Elapsed.Minutes:00}:{_gameStopwatch.Elapsed.Seconds:00}";

        private void RenderTimer_Tick(object? sender, EventArgs e)
        {
            try
            {
                _camera.Update(_gameController.Motorcycle.Position);
                _renderer.Render();
            }
            catch (Exception ex)
            {
                Log("MainWindow", "render error", () =>
                {
                    Error("MainWindow", $"Error in render loop: {ex.Message}");
                    Exception("MainWindow", ex);
                });
            }
        }

        private void CheckGameConditions()
        {
            if (_gameController.IsLevelComplete && _currentGameState == GameState.Playing)
                LevelComplete();

            if (_gameController.IsGameOver && _currentGameState == GameState.Playing)
                GameOver();
        }

        #endregion

        #region Event Handlers

        private void GameController_GameEvent(object? sender, GameEventArgs e) =>
            Dispatcher.InvokeAsync(() =>
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
            });

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
                    ExecuteContinue();
                else if (BikeSelectOverlay.Visibility == Visibility.Visible)
                    ExecuteBikeSelectBack();
                else if (LevelSelectOverlay.Visibility == Visibility.Visible)
                    ExecuteBack();
            }
        }

        private void HandlePlayingKeyDown(KeyEventArgs e)
        {
            switch (e.Key)
            {
                case Key.W:
                    _gameController.HandleKeyDown("W");
                    break;
                case Key.S:
                    _gameController.HandleKeyDown("S");
                    break;
                case Key.A:
                    _gameController.HandleKeyDown("A");
                    break;
                case Key.D:
                    _gameController.HandleKeyDown("D");
                    break;
                case Key.R:
                    ExecuteRestart();
                    break;
                case Key.Escape:
                    PauseGame();
                    break;
            }
        }

        private void Window_KeyUp(object sender, KeyEventArgs e)
        {
            switch (e.Key)
            {
                case Key.W:
                    _gameController.HandleKeyUp("W");
                    break;
                case Key.S:
                    _gameController.HandleKeyUp("S");
                    break;
                case Key.A:
                    _gameController.HandleKeyUp("A");
                    break;
                case Key.D:
                    _gameController.HandleKeyUp("D");
                    break;
            }
        }

        private static string GetBikeTypeName(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => "Стандартный",
            BikeType.Sport => "Спортивный",
            BikeType.OffRoad => "Внедорожный",
            _ => bikeType.ToString()
        };

        #endregion
    }

    public enum GameState
    {
        Playing,
        Paused,
        GameOver,
        LevelComplete
    }
}