using Point = System.Windows.Point;
using System;
using System.Windows;
using System.Windows.Media;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using GravityDefiedGame.Models;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame.Controllers
{
    public class GameController
    {
        private static class Constants
        {
            // World boundaries
            public const double MaxFallHeight = 1000.0;

            // Motorcycle control values
            public const double FullThrottle = 1.0;
            public const double FullBrake = 1.0;
            public const double LeftLean = -1.0;
            public const double RightLean = 1.0;
            public const double NoInput = 0.0;
        }

        private readonly InputState _input = new();
        private readonly Stopwatch _gameStopwatch = new();

        public Motorcycle Motorcycle { get; }
        public bool IsGameOver => Motorcycle.IsCrashed;
        public bool IsLevelComplete { get; private set; }
        public bool IsPaused { get; private set; }
        public Level? CurrentLevel { get; private set; }
        public List<Level> Levels { get; } = new();
        public TimeSpan GameTime { get; private set; }
        public event EventHandler<GameEventArgs>? GameEvent;

        public GameController()
        {
            Motorcycle = new Motorcycle();
            LoadLevels();
            Info("GameController", "Game controller initialized");
        }

        public void LoadLevels() =>
            Log("GameController", "loading levels", () =>
            {
                Levels.Clear();
                var random = new Random();

                for (int i = 1; i <= 25; i++)  // Количество доступных уровней (25)
                {
                    int seed = random.Next();
                    Levels.Add(new Level(i, $"Level {i}", seed));
                }
                Info("GameController", $"Loaded {Levels.Count} levels");
            });

        public void StartLevel(int levelId) =>
            Log("GameController", $"starting level {levelId}", () =>
            {
                Level? level = Levels.FirstOrDefault(l => l.Id == levelId);
                if (level is null)
                {
                    Error("GameController", $"Failed to start level {levelId}: level not found");
                    return;
                }

                CurrentLevel = level;
                IsLevelComplete = false;
                InitializeGameState(levelId);
                ResetInputState();
                InitializeMotorcycle();
            });

        private void InitializeGameState(int levelId) =>
            Log("GameController", $"initializing game state for level {levelId}", () =>
            {
                GameTime = TimeSpan.Zero;
                _gameStopwatch.Restart();
                IsPaused = false;
                OnGameEvent(GameEventType.LevelStart, $"Level {levelId} started");
                Info("GameController", $"Level {levelId} started");
            });

        private void InitializeMotorcycle() =>
            Log("GameController", "initializing motorcycle", () =>
            {
                if (CurrentLevel is null)
                {
                    Warning("GameController", "Cannot initialize motorcycle: no current level");
                    return;
                }

                Motorcycle.Reset();
                Motorcycle.SetPosition(CurrentLevel.StartPoint);
                Debug("GameController", $"Motorcycle initialized at position {CurrentLevel.StartPoint}");
            });

        public void Update(double deltaTime, CancellationToken cancellationToken = default)
        {
            if (IsGameOver || IsLevelComplete || IsPaused || cancellationToken.IsCancellationRequested)
                return;

            Log("GameController", "updating game", () =>
            {
                UpdateGameTime(deltaTime);
                UpdateMotorcycle(deltaTime, cancellationToken);
                CheckGameConditions();
            });
        }

        private void UpdateGameTime(double deltaTime) => GameTime += TimeSpan.FromSeconds(deltaTime);

        private void UpdateMotorcycle(double deltaTime, CancellationToken cancellationToken = default) =>
            Log("GameController", "updating motorcycle", () =>
            {
                if (CurrentLevel is null)
                {
                    Warning("GameController", "Cannot update motorcycle: no current level");
                    return;
                }

                cancellationToken.ThrowIfCancellationRequested();
                Motorcycle.Update(deltaTime, CurrentLevel, cancellationToken);
            });

        private void CheckGameConditions() =>
            Log("GameController", "checking game conditions", () =>
            {
                if (CheckFinishReached()) return;
                if (CheckFallOutOfBounds()) return;
                CheckMotorcycleCrash();
            });

        private bool CheckFinishReached() =>
            Log("GameController", "checking finish reached", () =>
            {
                if (CurrentLevel?.IsFinishReached(Motorcycle.Position) != true)
                    return false;

                Info("GameController", "Finish line reached");
                OnLevelComplete();
                return true;
            }, false);

        private bool CheckFallOutOfBounds() =>
            Log("GameController", "checking fall out of bounds", () =>
            {
                if (Motorcycle.Position.Y <= Constants.MaxFallHeight)
                    return false;

                Warning("GameController", $"Motorcycle fell out of bounds: Y={Motorcycle.Position.Y}");
                OnGameOver("Fell out of level boundaries");
                return true;
            }, false);

        private void CheckMotorcycleCrash() =>
            Log("GameController", "checking motorcycle crash", () =>
            {
                if (Motorcycle.IsCrashed && !IsGameOver)
                {
                    Warning("GameController", "Motorcycle crashed");
                    OnGameOver("Motorcycle crashed");
                }
            });

        private void OnLevelComplete() =>
            Log("GameController", "handling level complete", () =>
            {
                IsLevelComplete = true;
                string formattedTime = FormatGameTime();
                string message = $"Level completed!\nTime: {formattedTime}";
                OnGameEvent(GameEventType.LevelComplete, message);
                Info("GameController", $"Level {CurrentLevel?.Id} completed: Time={formattedTime}");
            });

        private string FormatGameTime() =>
            GameTime.TotalHours >= 1
                ? $"{GameTime.Hours:00}:{GameTime.Minutes:00}:{GameTime.Seconds:00}"
                : $"{GameTime.Minutes:00}:{GameTime.Seconds:00}";

        private void OnGameOver(string reason) =>
            Log("GameController", $"handling game over: {reason}", () =>
            {
                OnGameEvent(GameEventType.GameOver, $"Game over: {reason}");
                Info("GameController", $"Game over: {reason}");
            });

        private void OnGameEvent(GameEventType type, string message) =>
            Log("GameController", $"firing game event: {type}", () =>
            {
                Debug("GameController", $"Game event: {type} - {message}");
                GameEvent?.Invoke(this, new GameEventArgs(type, message));
            });

        public void PauseGame() =>
            Log("GameController", "pausing game", () =>
            {
                if (IsPaused) return;

                IsPaused = true;
                _gameStopwatch.Stop();
                OnGameEvent(GameEventType.GamePaused, "Game paused");
                Info("GameController", "Game paused");
            });

        public void ResumeGame() =>
            Log("GameController", "resuming game", () =>
            {
                if (!IsPaused) return;

                IsPaused = false;
                _gameStopwatch.Start();
                OnGameEvent(GameEventType.GameResumed, "Game resumed");
                Info("GameController", "Game resumed");
            });

        public void RestartLevel() =>
            Log("GameController", "restarting level", () =>
            {
                if (CurrentLevel is null)
                {
                    Warning("GameController", "Cannot restart level: no current level");
                    return;
                }

                StartLevel(CurrentLevel.Id);
                OnGameEvent(GameEventType.LevelRestart, "Level restarted");
                Info("GameController", $"Level {CurrentLevel.Id} restarted");
            });

        public void SetBikeType(BikeType bikeType) =>
            Log("GameController", $"setting bike type to {bikeType}", () =>
            {
                Motorcycle.SetBikeType(bikeType);
                OnGameEvent(GameEventType.BikeChanged, $"Selected bike: {bikeType}");
                Info("GameController", $"Bike type changed to {bikeType}");
            });

        public void SetBikeColor(Color color) =>
            Log("GameController", "setting bike color", () =>
            {
                Motorcycle.SetBikeColor(color);
                OnGameEvent(GameEventType.BikeChanged, "Bike color changed");
                Info("GameController", $"Bike color changed to {color}");
            });

        #region Input Handling

        public void ResetInputState() =>
            Log("GameController", "resetting input state", () =>
            {
                _input.Reset();
                ResetMotorcycleControls();
                Debug("GameController", "Input state reset");
            });

        private void ResetMotorcycleControls() =>
            Log("GameController", "resetting motorcycle controls", () =>
            {
                Motorcycle.ApplyThrottle(Constants.NoInput);
                Motorcycle.ApplyBrake(Constants.NoInput);
                Motorcycle.Lean(Constants.NoInput);
            });

        public void HandleKeyDown(string key) =>
            Log("GameController", $"handling key down: {key}", () =>
            {
                switch (key)
                {
                    case "W" when !_input.IsThrottlePressed:
                        _input.IsThrottlePressed = true;
                        Motorcycle.ApplyThrottle(Constants.FullThrottle);
                        Debug("GameController", "Throttle applied");
                        break;

                    case "S" when !_input.IsBrakePressed:
                        _input.IsBrakePressed = true;
                        Motorcycle.ApplyBrake(Constants.FullBrake);
                        Debug("GameController", "Brake applied");
                        break;

                    case "A" when !_input.IsLeaningLeft:
                        _input.IsLeaningLeft = true;
                        _input.IsLeaningRight = false;
                        Motorcycle.Lean(Constants.LeftLean);
                        Debug("GameController", "Leaning left");
                        break;

                    case "D" when !_input.IsLeaningRight:
                        _input.IsLeaningRight = true;
                        _input.IsLeaningLeft = false;
                        Motorcycle.Lean(Constants.RightLean);
                        Debug("GameController", "Leaning right");
                        break;
                }
            });

        public void HandleKeyUp(string key) =>
            Log("GameController", $"handling key up: {key}", () =>
            {
                switch (key)
                {
                    case "W":
                        _input.IsThrottlePressed = false;
                        Motorcycle.ApplyThrottle(Constants.NoInput);
                        Debug("GameController", "Throttle released");
                        break;

                    case "S":
                        _input.IsBrakePressed = false;
                        Motorcycle.ApplyBrake(Constants.NoInput);
                        Debug("GameController", "Brake released");
                        break;

                    case "A":
                        _input.IsLeaningLeft = false;
                        UpdateLeanState();
                        Debug("GameController", "Left lean released");
                        break;

                    case "D":
                        _input.IsLeaningRight = false;
                        UpdateLeanState();
                        Debug("GameController", "Right lean released");
                        break;
                }
            });

        private void UpdateLeanState() =>
            Log("GameController", "updating lean state", () =>
            {
                double leanAmount = CalculateLeanAmount();
                Motorcycle.Lean(leanAmount);
            });

        private double CalculateLeanAmount() =>
            Log("GameController", "calculating lean amount", () =>
                (_input.IsLeaningLeft, _input.IsLeaningRight) switch
                {
                    (true, false) => Constants.LeftLean,
                    (false, true) => Constants.RightLean,
                    _ => Constants.NoInput
                }
            , Constants.NoInput);

        #endregion
    }

    public sealed class Stopwatch
    {
        private DateTime _startTime;
        private TimeSpan _elapsedTime = TimeSpan.Zero;
        private bool _isRunning;

        public TimeSpan Elapsed => _isRunning
            ? _elapsedTime + (DateTime.Now - _startTime)
            : _elapsedTime;

        public void Restart()
        {
            _elapsedTime = TimeSpan.Zero;
            _startTime = DateTime.Now;
            _isRunning = true;
        }

        public void Start()
        {
            if (!_isRunning)
            {
                _startTime = DateTime.Now;
                _isRunning = true;
            }
        }

        public void Stop()
        {
            if (_isRunning)
            {
                _elapsedTime += DateTime.Now - _startTime;
                _isRunning = false;
            }
        }
    }

    public sealed record InputState
    {
        public bool IsThrottlePressed { get; set; }
        public bool IsBrakePressed { get; set; }
        public bool IsLeaningLeft { get; set; }
        public bool IsLeaningRight { get; set; }

        public void Reset() =>
            (IsThrottlePressed, IsBrakePressed, IsLeaningLeft, IsLeaningRight) = (false, false, false, false);
    }

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
        ProgressSaved,
        ProgressLoaded,
        GameComplete,
        Error
    }

    public sealed record GameEventArgs(GameEventType Type, string Message);
}