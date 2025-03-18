using System;
using System.Windows;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;
using GravityDefiedGame.Utilities;
using GravityDefiedGame.Controllers;

namespace GravityDefiedGame.Models
{
    public record struct TerrainConfig(double Length = 3000.0, TerrainStyle Style = TerrainStyle.Flat);
    public record struct LevelThemeColors(Color Background, Color Terrain);

    public enum TerrainStyle
    {
        Default,
        Flat
    }

    public enum LevelTheme
    {
        Desert
    }

    public class Level
    {
        // Константы для уровня
        private static class Constants
        {
            public const double
                DefaultGroundY = 500.0,
                FinishReachDistance = 50.0,
                DefaultTerrainLength = 3000.0;

            public static readonly LevelThemeColors DesertTheme = new(
                Background: Color.FromRgb(255, 204, 102),
                Terrain: Color.FromRgb(204, 153, 102)
            );
        }

        public int Id { get; }
        public string Name { get; }
        public List<Point> TerrainPoints { get; private set; } = [];
        public Point StartPoint { get; private set; }
        public Point FinishPoint { get; private set; }
        public double Length { get; private set; }
        public LevelTheme Theme { get; } = LevelTheme.Desert;

        public Color BackgroundColor => Constants.DesertTheme.Background;
        public Color TerrainColor => Constants.DesertTheme.Terrain;

        public Level(int id, string name, int? seed = null)
        {
            Id = id;
            Name = name;

            Logger.Info("Level", $"Creating level {id}: {name}");
            GenerateLevel(seed ?? id * 100);
            Logger.Info("Level", $"Level {id} created: Length={Length:F0}");
        }

        private void GenerateLevel(int seed)
        {
            var generator = new LevelGenerator(seed);
            var config = new TerrainConfig(Constants.DefaultTerrainLength, TerrainStyle.Flat);

            (TerrainPoints, Length, StartPoint, FinishPoint) = generator.GenerateTerrain(config);
        }

        #region Ground Calculation

        public double GetGroundYAtX(double x) =>
            IsOutOfBounds(x) ? GetOutOfBoundsValue(x) : InterpolateGroundY(x);

        private bool IsOutOfBounds(double x) =>
            TerrainPoints.Count < 2 || x < TerrainPoints[0].X || x > TerrainPoints[^1].X;

        private double GetOutOfBoundsValue(double x) =>
            x < TerrainPoints[0].X || x > TerrainPoints[^1].X ? double.MaxValue : Constants.DefaultGroundY;

        private double InterpolateGroundY(double x)
        {
            for (int i = 0; i < TerrainPoints.Count - 1; i++)
            {
                var (p1, p2) = (TerrainPoints[i], TerrainPoints[i + 1]);

                if (x >= p1.X && x <= p2.X)
                    return LinearInterpolate(x, p1, p2);
            }

            return Constants.DefaultGroundY;
        }

        private double LinearInterpolate(double x, Point p1, Point p2)
        {
            double t = (x - p1.X) / (p2.X - p1.X);
            return p1.Y * (1 - t) + p2.Y * t;
        }

        #endregion

        #region Finish Detection

        public bool IsFinishReached(Point position)
        {
            double distance = CalculateDistance(position, FinishPoint);
            bool isReached = distance <= Constants.FinishReachDistance;

            if (isReached)
                Logger.Info("Level", $"Finish reached at distance {distance:F1}");

            return isReached;
        }

        private double CalculateDistance(Point p1, Point p2)
        {
            double dx = p1.X - p2.X, dy = p1.Y - p2.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        #endregion

        // Вложенный класс для генерации ландшафта
        public class LevelGenerator
        {
            // Константы для генерации ландшафта
            public static class Constants
            {
                // Базовые параметры ландшафта
                public const double
                    DefaultTerrainHeight = 500.0,
                    WaveAmplitude = 150.0,
                    WaveFrequency = 5.0,
                    SmoothingFactor = 0.7,
                    PreviousPointWeight = 0.3,
                    FlatTerrainVariationFactor = 0.15,
                    StartZonePercent = 0.1,
                    EndZonePercent = 0.9,
                    StartPointXOffset = 100.0,
                    StartPointYOffset = -50.0,
                    FinishPointXOffset = -100.0,
                    FinishPointYOffset = -50.0;

                public const int DefaultPointCount = 60;
            }

            private readonly Random _random;

            public LevelGenerator(int seed) => _random = new Random(seed);

            public (List<Point> Points, double Length, Point StartPoint, Point FinishPoint) GenerateTerrain(TerrainConfig config)
            {
                double length = config.Length;
                var terrainPoints = new List<Point>();
                Point startPoint, finishPoint;

                var startY = Constants.DefaultTerrainHeight;
                AddStartPoint(terrainPoints, startY, out startPoint);
                GenerateTerrainPoints(terrainPoints, config.Style, startY, length);
                AddEndPoint(terrainPoints, startY, length, out finishPoint);

                Logger.Info("LevelGenerator", $"Terrain generated with style: {config.Style}");
                return (terrainPoints, length, startPoint, finishPoint);
            }

            private void AddStartPoint(List<Point> points, double y, out Point playerStart)
            {
                points.Add(new(0, y));
                playerStart = new(Constants.StartPointXOffset, y + Constants.StartPointYOffset);
            }

            private void AddEndPoint(List<Point> points, double y, double length, out Point finishPoint)
            {
                points.Add(new(length, y));
                finishPoint = new(length + Constants.FinishPointXOffset, y + Constants.FinishPointYOffset);
            }

            private void GenerateTerrainPoints(List<Point> points, TerrainStyle style, double baseY, double length)
            {
                double lastY = baseY;

                for (int i = 1; i < Constants.DefaultPointCount; i++)
                {
                    double progress = (double)i / Constants.DefaultPointCount;
                    double x = progress * length;
                    double y = CalculateHeight(style, baseY, progress);
                    y = ApplyTransitions(y, baseY, progress);
                    y = SmoothValue(lastY, y);

                    points.Add(new(x, y));
                    lastY = y;
                }
            }

            private double CalculateHeight(TerrainStyle style, double baseY, double progress) => style switch
            {
                TerrainStyle.Flat => baseY + GetSmallRandomVariation(),
                _ => baseY + GetSineWaveVariation(progress)
            };

            private double GetSmallRandomVariation() =>
                (_random.NextDouble() * 2 - 1) * Constants.WaveAmplitude * Constants.FlatTerrainVariationFactor;

            private double GetSineWaveVariation(double progress) =>
                Math.Sin(progress * Math.PI * Constants.WaveFrequency) * Constants.WaveAmplitude;

            private double SmoothValue(double prevValue, double currentValue) =>
                prevValue * Constants.PreviousPointWeight + currentValue * Constants.SmoothingFactor;

            private double ApplyTransitions(double y, double baseY, double progress) => progress switch
            {
                < Constants.StartZonePercent => baseY + (y - baseY) * (progress / Constants.StartZonePercent),
                > Constants.EndZonePercent => y + (baseY - y) * ((progress - Constants.EndZonePercent) / (1 - Constants.EndZonePercent)),
                _ => y
            };
        }
    }

    public class LevelManager
    {
        private static class Constants
        {
            public const double DefaultGroundY = 500.0;
            public static readonly (int id, string name)[] DefaultLevels = [(1, "Тестовый уровень")];
        }

        public List<Level> Levels { get; private set; } = [];
        public Level? CurrentLevel { get; private set; }
        public bool IsLevelComplete { get; private set; }
        public event EventHandler<GameEventArgs>? LevelEvent;

        public void LoadLevels()
        {
            Levels.Clear();

            foreach (var (id, name) in Constants.DefaultLevels)
                Levels.Add(new Level(id, name));

            Logger.Info("LevelManager", $"Loaded {Levels.Count} levels");
        }

        public bool StartLevel(int levelId)
        {
            var level = Levels.FirstOrDefault(l => l.Id == levelId);

            if (level is null)
            {
                Logger.Error("LevelManager", $"Failed to start level {levelId}: level not found");
                return false;
            }

            (CurrentLevel, IsLevelComplete) = (level, false);
            OnLevelEvent(GameEventType.LevelStart, $"Уровень {level.Id}: {level.Name}");
            Logger.Info("LevelManager", $"Started level {level.Id}: {level.Name}");

            return true;
        }

        public bool CheckFinish(Point position)
        {
            if (CurrentLevel is null || IsLevelComplete)
                return false;

            if (CurrentLevel.IsFinishReached(position))
            {
                IsLevelComplete = true;
                Logger.Info("LevelManager", $"Level {CurrentLevel.Id} completed");
                OnLevelEvent(GameEventType.LevelComplete, $"Уровень {CurrentLevel.Id} пройден!");
                return true;
            }

            return false;
        }

        private void OnLevelEvent(GameEventType type, string message) =>
            LevelEvent?.Invoke(this, new GameEventArgs(type, message));

        public double GetGroundYAtX(double x) =>
            CurrentLevel?.GetGroundYAtX(x) ?? Constants.DefaultGroundY;
    }
}