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
        Playing,
        Paused,
        GameOver,
        LevelComplete
    }

    public class GameController
    {
        private static class Constants
        {
            public const float MaxFallHeight = 1000.0f;
            public const float FullThrottle = 1.0f;
            public const float FullBrake = 1.0f;
            public const float LeftLean = -1.0f;
            public const float RightLean = 1.0f;
            public const float NoInput = 0.0f;
        }

        private readonly InputState _input = new();
        private readonly Stopwatch _gameStopwatch = new();

        public Motorcycle Motorcycle { get; }
        public bool IsGameOver => Motorcycle.IsCrashed;
        public bool IsLevelComplete { get; private set; }
        public bool IsPaused { get; private set; }
        public Level? CurrentLevel { get; private set; }
        public List<Level> Levels { get; } = new();
        public List<BikeType> AvailableBikes { get; } = new();
        public BikeType CurrentBikeType { get; private set; } = BikeType.Standard;
        public TimeSpan GameTime { get; private set; }
        public GameState CurrentGameState { get; set; } = GameState.MainMenu;
        public event EventHandler<GameEventArgs>? GameEvent;

        public GameController()
        {
            Motorcycle = new Motorcycle();
            InitializeAvailableBikes();
            LoadLevels();
            Info("GameController", "Game controller initialized");
        }

        private void InitializeAvailableBikes()
        {
            AvailableBikes.Clear();
            AvailableBikes.AddRange(new[] { BikeType.Standard, BikeType.Sport, BikeType.OffRoad });
            Info("GameController", $"Initialized {AvailableBikes.Count} available bike types");
        }

        public void LoadLevels()
        {
            Log("GameController", "loading levels", () =>
            {
                Levels.Clear();
                var random = new Random();
                for (int i = 1; i <= 25; i++)
                {
                    int seed = random.Next();
                    Levels.Add(new Level(i, $"Level {i}", seed));
                }
                Info("GameController", $"Loaded {Levels.Count} levels");
            });
        }

        public void StartLevel(int levelId)
        {
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
                CurrentGameState = GameState.Playing;
                InitializeGameState(levelId);
                ResetInputState();
                InitializeMotorcycle();
            });
        }

        public void StartNextLevel()
        {
            Log("GameController", "starting next level", () =>
            {
                int nextLevelId = (CurrentLevel?.Id ?? 0) + 1;
                if (nextLevelId <= Levels.Count)
                {
                    StartLevel(nextLevelId);
                    OnGameEvent(GameEventType.LevelStart, $"Level {nextLevelId} started");
                }
                else
                {
                    CurrentGameState = GameState.MainMenu;
                    OnGameEvent(GameEventType.GameComplete, "Congratulations! All levels completed!");
                }
            });
        }

        public void EnterMainMenu()
        {
            CurrentGameState = GameState.MainMenu;
            OnGameEvent(GameEventType.MenuChanged, "Entered Main Menu");
        }

        public void EnterBikeSelection()
        {
            CurrentGameState = GameState.BikeSelection;
            OnGameEvent(GameEventType.MenuChanged, "Entered Bike Selection");
        }

        public void EnterLevelSelection()
        {
            CurrentGameState = GameState.LevelSelection;
            OnGameEvent(GameEventType.MenuChanged, "Entered Level Selection");
        }

        public void HandleInput(KeyboardState keyboardState, KeyboardState previousKeyboardState)
        {
            Log("GameController", "handling input", () =>
            {
                if (CurrentGameState == GameState.Playing)
                {
                    HandleGameplayInput(keyboardState, previousKeyboardState);
                }

                foreach (Keys key in new[] { Keys.Escape, Keys.C, Keys.R })
                {
                    if (keyboardState.IsKeyDown(key) && previousKeyboardState.IsKeyUp(key))
                        HandleKeyPress(key, true);
                }
            });
        }

        private void HandleGameplayInput(KeyboardState keyboardState, KeyboardState previousKeyboardState)
        {
            if (keyboardState.IsKeyDown(Keys.W) && !previousKeyboardState.IsKeyDown(Keys.W))
                HandleKeyDown("W");
            else if (!keyboardState.IsKeyDown(Keys.W) && previousKeyboardState.IsKeyDown(Keys.W))
                HandleKeyUp("W");

            if (keyboardState.IsKeyDown(Keys.S) && !previousKeyboardState.IsKeyDown(Keys.S))
                HandleKeyDown("S");
            else if (!keyboardState.IsKeyDown(Keys.S) && previousKeyboardState.IsKeyDown(Keys.S))
                HandleKeyUp("S");

            if (keyboardState.IsKeyDown(Keys.A) && !previousKeyboardState.IsKeyDown(Keys.A))
                HandleKeyDown("A");
            else if (!keyboardState.IsKeyDown(Keys.A) && previousKeyboardState.IsKeyDown(Keys.A))
                HandleKeyUp("A");

            if (keyboardState.IsKeyDown(Keys.D) && !previousKeyboardState.IsKeyDown(Keys.D))
                HandleKeyDown("D");
            else if (!keyboardState.IsKeyDown(Keys.D) && previousKeyboardState.IsKeyDown(Keys.D))
                HandleKeyDown("D");
        }

        public void HandleKeyPress(Keys key, bool isKeyDown)
        {
            Log("GameController", $"handling key press: {key}, isDown: {isKeyDown}", () =>
            {
                if (isKeyDown)
                {
                    switch (key)
                    {
                        case Keys.Escape:
                            if (CurrentGameState == GameState.Playing) PauseGame();
                            else if (CurrentGameState == GameState.Paused) ResumeGame();
                            break;
                        case Keys.C:
                            if (CurrentGameState == GameState.LevelComplete)
                                StartNextLevel();
                            break;
                        case Keys.R:
                            if (CurrentGameState == GameState.GameOver || CurrentGameState == GameState.LevelComplete)
                                RestartLevel();
                            break;
                    }
                }
            });
        }

        private void InitializeGameState(int levelId)
        {
            Log("GameController", $"initializing game state for level {levelId}", () =>
            {
                GameTime = TimeSpan.Zero;
                _gameStopwatch.Restart();
                IsPaused = false;
                OnGameEvent(GameEventType.LevelStart, $"Level {levelId} started");
                Info("GameController", $"Level {levelId} started");
            });
        }

        private void InitializeMotorcycle()
        {
            Log("GameController", "initializing motorcycle", () =>
            {
                if (CurrentLevel is null)
                {
                    Warning("GameController", "Cannot initialize motorcycle: no current level");
                    return;
                }

                Motorcycle.Reset();
                Motorcycle.SetPosition(CurrentLevel.StartPoint);
                Motorcycle.SetBikeType(CurrentBikeType);
                Debug("GameController", $"Motorcycle initialized at position {CurrentLevel.StartPoint} with bike type {CurrentBikeType}");
            });
        }

        public void Update(float deltaTime, CancellationToken cancellationToken = default)
        {
            if (cancellationToken.IsCancellationRequested ||
                CurrentGameState != GameState.Playing ||
                IsPaused)
                return;

            Log("GameController", "updating game", () =>
            {
                UpdateGameTime(deltaTime);
                UpdateMotorcycle(deltaTime, cancellationToken);
                CheckGameConditions();
            });
        }

        private void UpdateGameTime(float deltaTime) => GameTime += TimeSpan.FromSeconds(deltaTime);

        private void UpdateMotorcycle(float deltaTime, CancellationToken cancellationToken = default)
        {
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
        }

        private void CheckGameConditions()
        {
            Log("GameController", "checking game conditions", () =>
            {
                if (CheckFinishReached()) return;
                if (CheckFallOutOfBounds()) return;
                CheckMotorcycleCrash();

                if (IsLevelComplete && CurrentGameState == GameState.Playing)
                    CurrentGameState = GameState.LevelComplete;

                if (IsGameOver && CurrentGameState == GameState.Playing)
                    CurrentGameState = GameState.GameOver;
            });
        }

        private bool CheckFinishReached()
        {
            Log("GameController", "checking finish reached", () =>
            {
                if (CurrentLevel?.IsFinishReached(Motorcycle.Position) != true)
                    return false;

                Info("GameController", "Finish line reached");
                OnLevelComplete();
                return true;
            }, false);
            return false;
        }

        private bool CheckFallOutOfBounds()
        {
            Log("GameController", "checking fall out of bounds", () =>
            {
                if (Motorcycle.Position.Y <= Constants.MaxFallHeight)
                    return false;

                Warning("GameController", $"Motorcycle fell out of bounds: Y={Motorcycle.Position.Y}");
                OnGameOver("Fell out of level boundaries");
                return true;
            }, false);
            return false;
        }

        private void CheckMotorcycleCrash()
        {
            Log("GameController", "checking motorcycle crash", () =>
            {
                if (Motorcycle.IsCrashed && !IsGameOver)
                {
                    Warning("GameController", "Motorcycle crashed");
                    OnGameOver("Motorcycle crashed");
                }
            });
        }

        private void OnLevelComplete()
        {
            Log("GameController", "handling level complete", () =>
            {
                IsLevelComplete = true;
                CurrentGameState = GameState.LevelComplete;
                string formattedTime = FormatGameTime();
                string message = $"Level completed!\nTime: {formattedTime}";
                OnGameEvent(GameEventType.LevelComplete, message);
                Info("GameController", $"Level {CurrentLevel?.Id} completed: Time={formattedTime}");
            });
        }

        private string FormatGameTime() =>
            GameTime.TotalHours >= 1
                ? $"{GameTime.Hours:00}:{GameTime.Minutes:00}:{GameTime.Seconds:00}"
                : $"{GameTime.Minutes:00}:{GameTime.Seconds:00}";

        private void OnGameOver(string reason)
        {
            Log("GameController", $"handling game over: {reason}", () =>
            {
                CurrentGameState = GameState.GameOver;
                OnGameEvent(GameEventType.GameOver, $"Game over: {reason}");
                Info("GameController", $"Game over: {reason}");
            });
        }

        private void OnGameEvent(GameEventType type, string message)
        {
            Log("GameController", $"firing game event: {type}", () =>
            {
                Debug("GameController", $"Game event: {type} - {message}");
                GameEvent?.Invoke(this, new GameEventArgs(type, message));
            });
        }

        public void PauseGame()
        {
            Log("GameController", "pausing game", () =>
            {
                if (IsPaused) return;

                IsPaused = true;
                CurrentGameState = GameState.Paused;
                _gameStopwatch.Stop();
                OnGameEvent(GameEventType.GamePaused, "Game paused");
                Info("GameController", "Game paused");
            });
        }

        public void ResumeGame()
        {
            Log("GameController", "resuming game", () =>
            {
                if (!IsPaused) return;

                IsPaused = false;
                CurrentGameState = GameState.Playing;
                _gameStopwatch.Start();
                OnGameEvent(GameEventType.GameResumed, "Game resumed");
                Info("GameController", "Game resumed");
            });
        }

        public void RestartLevel()
        {
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
        }

        public void SetBikeType(BikeType bikeType)
        {
            Log("GameController", $"setting bike type to {bikeType}", () =>
            {
                CurrentBikeType = bikeType;
                Motorcycle.SetBikeType(bikeType);
                OnGameEvent(GameEventType.BikeChanged, $"Selected bike: {bikeType}");
                Info("GameController", $"Bike type changed to {bikeType}");
            });
        }

        public void SetBikeColor(Color color)
        {
            Log("GameController", "setting bike color", () =>
            {
                Motorcycle.SetBikeColor(color);
                OnGameEvent(GameEventType.BikeChanged, "Bike color changed");
                Info("GameController", $"Bike color changed");
            });
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

        #region Input Handling

        public void ResetInputState()
        {
            Log("GameController", "resetting input state", () =>
            {
                _input.Reset();
                ResetMotorcycleControls();
                Debug("GameController", "Input state reset");
            });
        }

        private void ResetMotorcycleControls()
        {
            Log("GameController", "resetting motorcycle controls", () =>
            {
                Motorcycle.ApplyThrottle(Constants.NoInput);
                Motorcycle.ApplyBrake(Constants.NoInput);
                Motorcycle.Lean(Constants.NoInput);
            });
        }

        public void HandleKeyDown(string key)
        {
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
        }

        public void HandleKeyUp(string key)
        {
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
        }

        private void UpdateLeanState()
        {
            Log("GameController", "updating lean state", () =>
            {
                float leanAmount = CalculateLeanAmount();
                Motorcycle.Lean(leanAmount);
            });
        }

        private float CalculateLeanAmount()
        {
            Log("GameController", "calculating lean amount", () =>
                (_input.IsLeaningLeft, _input.IsLeaningRight) switch
                {
                    (true, false) => Constants.LeftLean,
                    (false, true) => Constants.RightLean,
                    _ => Constants.NoInput
                }, Constants.NoInput);
            return (_input.IsLeaningLeft, _input.IsLeaningRight) switch
            {
                (true, false) => Constants.LeftLean,
                (false, true) => Constants.RightLean,
                _ => Constants.NoInput
            };
        }

        #endregion
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
        LevelSelected,
        ProgressSaved,
        ProgressLoaded,
        GameComplete,
        MenuChanged,
        Error
    }

    public sealed record GameEventArgs(GameEventType Type, string Message);

    public sealed record InputState
    {
        public bool IsThrottlePressed { get; set; }
        public bool IsBrakePressed { get; set; }
        public bool IsLeaningLeft { get; set; }
        public bool IsLeaningRight { get; set; }

        public void Reset() =>
            (IsThrottlePressed, IsBrakePressed, IsLeaningLeft, IsLeaningRight) = (false, false, false, false);
    }
}