using Point = System.Windows.Point;
using System;
using System.Windows;
using System.Windows.Media;
using System.Collections.Generic;
using GravityDefiedGame.Models;
using GravityDefiedGame.Utilities;

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

        private readonly LevelManager _levelManager;
        private readonly InputState _input = new();

        public Motorcycle Motorcycle { get; }
        public bool IsGameOver => Motorcycle.IsCrashed;
        public bool IsLevelComplete => _levelManager.IsLevelComplete;
        public bool IsPaused { get; private set; }
        public Level? CurrentLevel => _levelManager.CurrentLevel;
        public List<Level> Levels => _levelManager.Levels;
        public TimeSpan GameTime { get; private set; }
        public event EventHandler<GameEventArgs>? GameEvent;

        public GameController()
        {
            _levelManager = new LevelManager();
            Motorcycle = new Motorcycle();
            _levelManager.LevelEvent += LevelManager_LevelEvent;
            LoadLevels();
            Logger.Info("GameController", "Game controller initialized");
        }

        public void LoadLevels() => _levelManager.LoadLevels();

        public void StartLevel(int levelId)
        {
            if (!_levelManager.StartLevel(levelId))
                return;

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
            if (CurrentLevel is null)
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
            if (_levelManager.CheckFinish(Motorcycle.Position))
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

        private void LevelManager_LevelEvent(object? sender, GameEventArgs e) =>
            OnGameEvent(e.Type, e.Message);

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
            if (CurrentLevel is null)
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

    /// <summary>
    /// Класс для хранения состояния ввода
    /// </summary>
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