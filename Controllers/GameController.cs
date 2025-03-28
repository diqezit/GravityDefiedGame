#nullable enable

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Diagnostics;
using GravityDefiedGame.Models;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame.Controllers
{
    public enum GameState
    {
        MainMenu,
        BikeSelection,
        LevelSelection,
        ThemeSelection,
        Playing,
        Paused,
        GameOver,
        LevelComplete
    }

    public class GameController
    {
        #region Constants
        private static class Constants
        {
            // Значения управления
            public const float FullThrottle = 1.0f;
            public const float FullBrake = 1.0f;
            public const float LeftLean = -1.0f;
            public const float RightLean = 1.0f;
            public const float NoInput = 0.0f;

            // Игровые параметры
            public const float MaxFallHeight = 1000.0f;
        }
        #endregion

        #region Private Fields
        private readonly InputState _input = new();
        private readonly Stopwatch _gameStopwatch = new();
        private readonly Dictionary<string, Action> _keyDownActions;
        private readonly Dictionary<string, Action> _keyUpActions;
        private GameState _currentGameState = GameState.MainMenu;
        #endregion

        #region Public Properties
        public Motorcycle Motorcycle { get; }
        public bool IsGameOver => Motorcycle.IsCrashed;
        public bool IsLevelComplete { get; private set; }
        public bool IsPaused { get; private set; }
        public Level? CurrentLevel { get; private set; }
        public List<Level> Levels { get; } = new();
        public List<BikeType> AvailableBikes { get; } = new();
        public BikeType CurrentBikeType { get; private set; } = BikeType.Standard;
        public TimeSpan GameTime { get; private set; }

        public GameState CurrentGameState
        {
            get => _currentGameState;
            set
            {
                if (_currentGameState == value) return;

                var oldState = _currentGameState;
                _currentGameState = value;

                string message = GetGameStateMessage(oldState, value);
                OnGameEvent(GetEventTypeForStateTransition(oldState, value), message);
            }
        }
        #endregion

        public event EventHandler<GameEventArgs>? GameEvent;

        public GameController()
        {
            Motorcycle = new Motorcycle();

            // Инициализация действий для клавиш управления
            _keyDownActions = new Dictionary<string, Action>
            {
                ["W"] = () =>
                {
                    if (!_input.IsForwardPressed)
                    {
                        _input.IsForwardPressed = true;
                        Motorcycle.SetDirection(1);
                        Motorcycle.ApplyThrottle(Constants.FullThrottle);
                        Debug("GameController", "Throttle applied forward");
                    }
                },
                ["S"] = () =>
                {
                    if (!_input.IsBackwardPressed)
                    {
                        _input.IsBackwardPressed = true;
                        Motorcycle.SetDirection(-1);
                        Motorcycle.ApplyThrottle(Constants.FullThrottle);
                        Debug("GameController", "Throttle applied backward");
                    }
                },
                ["A"] = () =>
                {
                    if (!_input.IsLeaningLeft)
                    {
                        _input.IsLeaningLeft = true;
                        UpdateLeanState();
                        Debug("GameController", "Leaning left");
                    }
                },
                ["D"] = () =>
                {
                    if (!_input.IsLeaningRight)
                    {
                        _input.IsLeaningRight = true;
                        UpdateLeanState();
                        Debug("GameController", "Leaning right");
                    }
                }
            };

            _keyUpActions = new Dictionary<string, Action>
            {
                ["W"] = () =>
                {
                    _input.IsForwardPressed = false;
                    Motorcycle.ApplyThrottle(Constants.NoInput);
                    Debug("GameController", "Throttle released");
                },
                ["S"] = () =>
                {
                    _input.IsBackwardPressed = false;
                    Motorcycle.ApplyThrottle(Constants.NoInput);
                    Debug("GameController", "Backward throttle released");
                },
                ["A"] = () =>
                {
                    _input.IsLeaningLeft = false;
                    UpdateLeanState();
                    Debug("GameController", "Left lean released");
                },
                ["D"] = () =>
                {
                    _input.IsLeaningRight = false;
                    UpdateLeanState();
                    Debug("GameController", "Right lean released");
                }
            };

            InitializeAvailableBikes();
            LoadLevels();
            Info("GameController", "Game controller initialized");
        }

        #region Setup and Initialization
        private void InitializeAvailableBikes()
        {
            AvailableBikes.Clear();
            AvailableBikes.AddRange(new[] { BikeType.Standard, BikeType.Sport, BikeType.OffRoad });
            Info("GameController", $"Initialized {AvailableBikes.Count} available bike types");
        }

        public void LoadLevels()
        {
            Levels.Clear();
            var random = new Random();

            for (int i = 1; i <= 25; i++)
            {
                int seed = random.Next();
                Levels.Add(new Level(i, $"Level {i}", seed));
            }

            Info("GameController", $"Loaded {Levels.Count} levels");
        }

        private void InitializeGameState(int levelId)
        {
            GameTime = TimeSpan.Zero;
            _gameStopwatch.Restart();
            IsPaused = false;
            Info("GameController", $"Level {levelId} started");
        }

        private bool TryInitializeMotorcycle(out string errorMessage)
        {
            errorMessage = string.Empty;

            if (CurrentLevel is null)
            {
                errorMessage = "Cannot initialize motorcycle: no current level";
                Warning("GameController", errorMessage);
                return false;
            }

            try
            {
                Motorcycle.Reset();
                Motorcycle.SetPosition(CurrentLevel.StartPoint);
                Motorcycle.SetBikeType(CurrentBikeType);
                Debug("GameController", $"Motorcycle initialized at position {CurrentLevel.StartPoint} with bike type {CurrentBikeType}");
                return true;
            }
            catch (Exception ex)
            {
                errorMessage = $"Failed to initialize motorcycle: {ex.Message}";
                Error("GameController", errorMessage);
                return false;
            }
        }
        #endregion

        #region Game Flow Control
        public void StartLevel(int levelId)
        {
            Level? level = Levels.FirstOrDefault(l => l.Id == levelId);
            if (level is null)
            {
                Error("GameController", $"Failed to start level {levelId}: level not found");
                OnGameEvent(GameEventType.Error, $"Failed to start level {levelId}: level not found");
                return;
            }

            CurrentLevel = level;
            IsLevelComplete = false;
            InitializeGameState(levelId);
            ResetInputState();

            if (!TryInitializeMotorcycle(out var errorMessage))
            {
                OnGameEvent(GameEventType.Error, errorMessage);
                CurrentGameState = GameState.MainMenu;
                return;
            }

            CurrentGameState = GameState.Playing;
            OnGameEvent(GameEventType.LevelStart, $"Level {levelId} started");
        }

        public void PauseGame()
        {
            if (IsPaused) return;

            IsPaused = true;
            CurrentGameState = GameState.Paused;
            _gameStopwatch.Stop();
            Info("GameController", "Game paused");
        }

        public void ResumeGame()
        {
            if (!IsPaused) return;

            IsPaused = false;
            CurrentGameState = GameState.Playing;
            _gameStopwatch.Start();
            Info("GameController", "Game resumed");
        }
        #endregion

        #region Menu Navigation
        public void EnterMainMenu() => CurrentGameState = GameState.MainMenu;
        public void EnterBikeSelection() => CurrentGameState = GameState.BikeSelection;
        public void EnterLevelSelection() => CurrentGameState = GameState.LevelSelection;
        public void EnterThemeSelection() => CurrentGameState = GameState.ThemeSelection;

        public void SetBikeType(BikeType bikeType)
        {
            CurrentBikeType = bikeType;
            Motorcycle.SetBikeType(bikeType);
            OnGameEvent(GameEventType.BikeChanged, $"Selected bike: {bikeType}");
            Info("GameController", $"Bike type changed to {bikeType}");
        }

        public void SetBikeColor(Color color)
        {
            Motorcycle.SetBikeColor(color);
            OnGameEvent(GameEventType.BikeChanged, "Bike color changed");
            Info("GameController", "Bike color changed");
        }

        public void SelectLevel(int levelId)
        {
            if (levelId < 1 || levelId > Levels.Count)
            {
                Warning("GameController", $"Level {levelId} is out of range");
                return;
            }
            OnGameEvent(GameEventType.LevelSelected, $"Selected level: {levelId}");
            Info("GameController", $"Selected level: {levelId}");
        }
        #endregion

        #region Game Loop
        public void Update(float deltaTime, CancellationToken cancellationToken = default)
        {
            try
            {
                // Проверка условий для пропуска обновления
                if (cancellationToken.IsCancellationRequested ||
                    CurrentGameState != GameState.Playing ||
                    IsPaused || CurrentLevel == null)
                    return;

                // Обновление времени игры
                UpdateGameTime(deltaTime);

                // Обновление мотоцикла
                Motorcycle.Update(deltaTime, CurrentLevel, cancellationToken);

                // Проверка игровых условий
                CheckGameConditions();
            }
            catch (OperationCanceledException)
            {
                // Корректная обработка отмены операции
                PauseGame();
            }
            catch (Exception ex)
            {
                // Обработка неожиданных ошибок
                Error("GameController", $"Unexpected error during update: {ex.Message}");
                OnGameEvent(GameEventType.Error, $"Game error: {ex.Message}");
                PauseGame();
            }
        }

        // Обновление игрового времени
        private void UpdateGameTime(float deltaTime) => GameTime += TimeSpan.FromSeconds(deltaTime);

        // Проверка игровых условий
        private void CheckGameConditions()
        {
            // Проверяем в порядке приоритета, с ранним возвратом
            if (CheckFinishReached()) return;
            if (CheckFallOutOfBounds()) return;
            CheckMotorcycleCrash();

            if (IsLevelComplete && CurrentGameState == GameState.Playing)
                CurrentGameState = GameState.LevelComplete;

            if (IsGameOver && CurrentGameState == GameState.Playing)
                CurrentGameState = GameState.GameOver;
        }

        // Проверка достижения финиша
        private bool CheckFinishReached()
        {
            if (CurrentLevel?.IsFinishReached(Motorcycle.Position) != true)
                return false;

            Info("GameController", "Finish line reached");
            OnLevelComplete();
            return true;
        }

        // Проверка падения за пределы уровня
        private bool CheckFallOutOfBounds()
        {
            if (Motorcycle.Position.Y <= Constants.MaxFallHeight)
                return false;

            Warning("GameController", $"Motorcycle fell out of bounds: Y={Motorcycle.Position.Y}");
            OnGameOver("Fell out of level boundaries");
            return true;
        }

        // Проверка аварии мотоцикла
        private void CheckMotorcycleCrash()
        {
            if (Motorcycle.IsCrashed && !IsGameOver)
            {
                Warning("GameController", "Motorcycle crashed");
                OnGameOver("Motorcycle crashed");
            }
        }

        // Обработка завершения уровня
        private void OnLevelComplete()
        {
            IsLevelComplete = true;
            CurrentGameState = GameState.LevelComplete;
            string formattedTime = FormatGameTime();
            string message = $"Level completed!\nTime: {formattedTime}";
            OnGameEvent(GameEventType.LevelComplete, message);
            Info("GameController", $"Level {CurrentLevel?.Id} completed: Time={formattedTime}");
        }

        // Обработка проигрыша
        private void OnGameOver(string reason)
        {
            CurrentGameState = GameState.GameOver;
            OnGameEvent(GameEventType.GameOver, $"Game over: {reason}");
            Info("GameController", $"Game over: {reason}");
        }
        #endregion

        #region Input Handling
        // Обработка ввода с клавиатуры
        public void HandleInput(KeyboardState keyboardState, KeyboardState previousKeyboardState)
        {
            if (CurrentGameState == GameState.Playing)
            {
                CheckKeyState(keyboardState, previousKeyboardState, Keys.W, "W");
                CheckKeyState(keyboardState, previousKeyboardState, Keys.S, "S");
                CheckKeyState(keyboardState, previousKeyboardState, Keys.A, "A");
                CheckKeyState(keyboardState, previousKeyboardState, Keys.D, "D");
            }
        }

        // Проверка состояния клавиши и вызов соответствующего обработчика
        private void CheckKeyState(KeyboardState current, KeyboardState previous, Keys key, string keyName)
        {
            if (current.IsKeyDown(key) && previous.IsKeyUp(key))
                HandleKeyDown(keyName);
            else if (current.IsKeyUp(key) && previous.IsKeyDown(key))
                HandleKeyUp(keyName);
        }

        // Обработка нажатия клавиши
        public void HandleKeyDown(string key)
        {
            if (_keyDownActions.TryGetValue(key, out var action))
            {
                action();
            }
        }

        // Обработка отпускания клавиши
        public void HandleKeyUp(string key)
        {
            if (_keyUpActions.TryGetValue(key, out var action))
            {
                action();
            }
        }

        // Сброс состояния ввода
        public void ResetInputState()
        {
            _input.Reset();
            ResetMotorcycleControls();
            Debug("GameController", "Input state reset");
        }

        // Сброс управления мотоциклом
        private void ResetMotorcycleControls()
        {
            Motorcycle.ApplyThrottle(Constants.NoInput);
            Motorcycle.ApplyBrake(Constants.NoInput);
            Motorcycle.Lean(Constants.NoInput);
        }

        // Обновление состояния наклона мотоцикла
        private void UpdateLeanState()
        {
            float leanAmount = CalculateLeanAmount();
            Motorcycle.Lean(leanAmount);
        }

        // Расчет величины наклона на основе нажатых клавиш
        private float CalculateLeanAmount() => (_input.IsLeaningLeft, _input.IsLeaningRight) switch
        {
            (true, true) => Constants.NoInput,   // Обе клавиши - нейтральное положение
            (true, false) => Constants.LeftLean, // Только левая - наклон влево
            (false, true) => Constants.RightLean,// Только правая - наклон вправо
            _ => Constants.NoInput               // Ни одной - нейтральное положение
        };
        #endregion

        #region Utility Methods
        // Форматирование игрового времени
        private string FormatGameTime() =>
            GameTime.TotalHours >= 1
                ? $"{GameTime.Hours:00}:{GameTime.Minutes:00}:{GameTime.Seconds:00}"
                : $"{GameTime.Minutes:00}:{GameTime.Seconds:00}";

        // Получение сообщения для перехода между состояниями
        private string GetGameStateMessage(GameState oldState, GameState newState) =>
            (oldState, newState) switch
            {
                (_, GameState.MainMenu) => "Entered Main Menu",
                (_, GameState.BikeSelection) => "Entered Bike Selection",
                (_, GameState.LevelSelection) => "Entered Level Selection",
                (_, GameState.ThemeSelection) => "Entered Theme Selection",
                (_, GameState.Playing) when oldState == GameState.Paused => "Game resumed",
                (_, GameState.Playing) => $"Level {CurrentLevel?.Id ?? 0} started",
                (_, GameState.Paused) => "Game paused",
                (_, GameState.GameOver) => "Game over",
                (_, GameState.LevelComplete) => $"Level {CurrentLevel?.Id ?? 0} completed!",
                _ => $"State changed: {oldState} -> {newState}"
            };

        // Определение типа события для перехода между состояниями
        private GameEventType GetEventTypeForStateTransition(GameState oldState, GameState newState) =>
            (oldState, newState) switch
            {
                (_, GameState.MainMenu) => GameEventType.MenuChanged,
                (_, GameState.BikeSelection) => GameEventType.MenuChanged,
                (_, GameState.LevelSelection) => GameEventType.MenuChanged,
                (_, GameState.ThemeSelection) => GameEventType.MenuChanged,
                (_, GameState.Playing) when oldState == GameState.Paused => GameEventType.GameResumed,
                (_, GameState.Playing) => GameEventType.LevelStart,
                (_, GameState.Paused) => GameEventType.GamePaused,
                (_, GameState.GameOver) => GameEventType.GameOver,
                (_, GameState.LevelComplete) => GameEventType.LevelComplete,
                _ => GameEventType.MenuChanged
            };

        // Генерация события игры
        private void OnGameEvent(GameEventType type, string message)
        {
            Debug("GameController", $"Game event: {type} - {message}");
            GameEvent?.Invoke(this, new GameEventArgs(type, message));
        }
        #endregion
    }

    #region Supporting Types
    // Типы игровых событий
    public enum GameEventType
    {
        LevelStart,
        LevelComplete,
        LevelRestart,
        CheckpointReached,
        FuelCollected,
        GameOver,
        GamePaused,
        GameResumed,
        BikeChanged,
        LevelSelected,
        ProgressSaved,
        ProgressLoaded,
        GameComplete,
        MenuChanged,
        Error
    }

    // Аргументы события
    public sealed record GameEventArgs(GameEventType Type, string Message);

    // Состояние ввода
    public sealed record InputState
    {
        public bool IsForwardPressed { get; set; } 
        public bool IsBackwardPressed { get; set; } 
        public bool IsLeaningLeft { get; set; }
        public bool IsLeaningRight { get; set; }

        public void Reset() =>
            (IsForwardPressed, IsBackwardPressed, IsLeaningLeft, IsLeaningRight) = (false, false, false, false);
    }
    #endregion
}