using Point = System.Windows.Point;
using System;
using System.Windows;
using System.Windows.Media;
using System.Collections.Generic;
using GravityDefiedGame.Models;
using GravityDefiedGame.Utilities;
using System.Linq;

namespace GravityDefiedGame.Controllers
{
    public class GameController
    {
        private static class Constants
        {
            public const double MaxFallHeight = 1000.0;
            public const double FullThrottle = 1.0;
            public const double FullBrake = 1.0;
            public const double LeftLean = -1.0;
            public const double RightLean = 1.0;
            public const double NoInput = 0.0;
        }

        private readonly InputState _input = new();
        public Motorcycle Motorcycle { get; }
        public bool IsGameOver => Motorcycle.IsCrashed;
        public bool IsLevelComplete { get; private set; }
        public bool IsPaused { get; private set; }
        public Level? CurrentLevel { get; private set; }
        public List<Level> Levels { get; private set; } = new List<Level>();
        public TimeSpan GameTime { get; private set; }
        public event EventHandler<GameEventArgs>? GameEvent;

        public GameController()
        {
            Motorcycle = new Motorcycle();
            LoadLevels();
            Logger.Info("GameController", "Game controller initialized");
        }

        public void LoadLevels()
        {
            Levels.Clear();
            Random random = new Random(); // Create Random instance for seed generation
            for (int i = 1; i <= 5; i++) // Generate 5 levels as an example
            {
                int seed = random.Next(); // Random seed for each level
                Levels.Add(new Level(i, $"Уровень {i}", seed));
            }
            Logger.Info("GameController", $"Loaded {Levels.Count} levels");
        }

        public void StartLevel(int levelId)
        {
            var level = Levels.FirstOrDefault(l => l.Id == levelId);
            if (level == null)
            {
                Logger.Error("GameController", $"Failed to start level {levelId}: level not found");
                return;
            }

            CurrentLevel = level;
            IsLevelComplete = false;
            InitializeGameState(levelId);
            ResetInputState();
            InitializeMotorcycle();
        }

        private void InitializeGameState(int levelId)
        {
            GameTime = TimeSpan.Zero;
            IsPaused = false;
            OnGameEvent(GameEventType.LevelStart, $"Уровень {levelId} начат");
            Logger.Info("GameController", $"Level {levelId} started");
        }

        private void InitializeMotorcycle()
        {
            if (CurrentLevel == null)
                return;

            Motorcycle.Reset();
            Motorcycle.SetPosition(CurrentLevel.StartPoint);
        }

        public void Update(double deltaTime)
        {
            if (IsGameOver || IsLevelComplete || IsPaused)
                return;

            UpdateGameTime(deltaTime);
            UpdateMotorcycle(deltaTime);
            CheckGameConditions();
        }

        private void UpdateGameTime(double deltaTime) =>
            GameTime += TimeSpan.FromSeconds(deltaTime);

        private void UpdateMotorcycle(double deltaTime) =>
            Motorcycle.Update(deltaTime, CurrentLevel!);

        private void CheckGameConditions()
        {
            CheckFinishReached();
            CheckFallOutOfBounds();
            CheckMotorcycleCrash();
        }

        private void CheckFinishReached()
        {
            if (CurrentLevel?.IsFinishReached(Motorcycle.Position) == true)
                OnLevelComplete();
        }

        private void CheckFallOutOfBounds()
        {
            if (Motorcycle.Position.Y > Constants.MaxFallHeight)
                OnGameOver("Падение за пределы уровня");
        }

        private void CheckMotorcycleCrash()
        {
            if (Motorcycle.IsCrashed && !IsGameOver)
                OnGameOver("Мотоцикл разбился");
        }

        private void OnLevelComplete()
        {
            IsLevelComplete = true;
            string formattedTime = FormatGameTime();
            string message = $"Уровень пройден!\nВремя: {formattedTime}";
            OnGameEvent(GameEventType.LevelComplete, message);
            Logger.Info("GameController", $"Level {CurrentLevel?.Id} completed: Time={formattedTime}");
        }

        private string FormatGameTime() =>
            $"{GameTime.Minutes:00}:{GameTime.Seconds:00}";

        private void OnGameOver(string reason)
        {
            OnGameEvent(GameEventType.GameOver, $"Игра окончена: {reason}");
            Logger.Info("GameController", $"Game over: {reason}");
        }

        private void OnGameEvent(GameEventType type, string message)
        {
            Logger.Debug("GameController", $"Game event: {type} - {message}");
            GameEvent?.Invoke(this, new GameEventArgs(type, message));
        }

        public void PauseGame()
        {
            IsPaused = true;
            OnGameEvent(GameEventType.GamePaused, "Игра приостановлена");
            Logger.Info("GameController", "Game paused");
        }

        public void ResumeGame()
        {
            IsPaused = false;
            OnGameEvent(GameEventType.GameResumed, "Игра продолжена");
            Logger.Info("GameController", "Game resumed");
        }

        public void RestartLevel()
        {
            if (CurrentLevel == null)
                return;

            StartLevel(CurrentLevel.Id);
            OnGameEvent(GameEventType.LevelRestart, "Уровень перезапущен");
            Logger.Info("GameController", $"Level {CurrentLevel.Id} restarted");
        }

        public void SetBikeType(BikeType bikeType)
        {
            Motorcycle.SetBikeType(bikeType);
            OnGameEvent(GameEventType.BikeChanged, $"Выбран мотоцикл: {bikeType}");
            Logger.Info("GameController", $"Bike type changed to {bikeType}");
        }

        public void SetBikeColor(Color color)
        {
            Motorcycle.SetBikeColor(color);
            OnGameEvent(GameEventType.BikeChanged, "Цвет мотоцикла изменен");
            Logger.Info("GameController", $"Bike color changed to {color}");
        }

        #region Input Handling

        public void ResetInputState()
        {
            _input.Reset();
            ResetMotorcycleControls();
        }

        private void ResetMotorcycleControls()
        {
            Motorcycle.ApplyThrottle(Constants.NoInput);
            Motorcycle.ApplyBrake(Constants.NoInput);
            Motorcycle.Lean(Constants.NoInput);
        }

        public void HandleKeyDown(string key)
        {
            switch (key)
            {
                case "W" when !_input.IsThrottlePressed:
                    _input.IsThrottlePressed = true;
                    Motorcycle.ApplyThrottle(Constants.FullThrottle);
                    break;
                case "S" when !_input.IsBrakePressed:
                    _input.IsBrakePressed = true;
                    Motorcycle.ApplyBrake(Constants.FullBrake);
                    break;
                case "A" when !_input.IsLeaningLeft:
                    _input.IsLeaningLeft = true;
                    Motorcycle.Lean(Constants.LeftLean);
                    break;
                case "D" when !_input.IsLeaningRight:
                    _input.IsLeaningRight = true;
                    Motorcycle.Lean(Constants.RightLean);
                    break;
            }
        }

        public void HandleKeyUp(string key)
        {
            switch (key)
            {
                case "W":
                    _input.IsThrottlePressed = false;
                    Motorcycle.ApplyThrottle(Constants.NoInput);
                    break;
                case "S":
                    _input.IsBrakePressed = false;
                    Motorcycle.ApplyBrake(Constants.NoInput);
                    break;
                case "A":
                    _input.IsLeaningLeft = false;
                    UpdateLeanState();
                    break;
                case "D":
                    _input.IsLeaningRight = false;
                    UpdateLeanState();
                    break;
            }
        }

        private void UpdateLeanState()
        {
            double leanAmount = CalculateLeanAmount();
            Motorcycle.Lean(leanAmount);
        }

        private double CalculateLeanAmount()
        {
            if (_input.IsLeaningLeft && !_input.IsLeaningRight)
                return Constants.LeftLean;
            if (!_input.IsLeaningLeft && _input.IsLeaningRight)
                return Constants.RightLean;
            return Constants.NoInput;
        }

        #endregion
    }

    internal class InputState
    {
        public bool IsThrottlePressed { get; set; }
        public bool IsBrakePressed { get; set; }
        public bool IsLeaningLeft { get; set; }
        public bool IsLeaningRight { get; set; }

        public void Reset()
        {
            IsThrottlePressed = false;
            IsBrakePressed = false;
            IsLeaningLeft = false;
            IsLeaningRight = false;
        }
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

    public class GameEventArgs : EventArgs
    {
        public GameEventType Type { get; }
        public string Message { get; }

        public GameEventArgs(GameEventType type, string message)
        {
            Type = type;
            Message = message;
        }
    }
}