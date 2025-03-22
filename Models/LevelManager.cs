using System;
using System.Windows;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Models.Level.Constants;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame.Models
{
    public record TerrainConfig(
        double Length = 3000.0,
        TerrainStyle Style = TerrainStyle.Flat,
        double TopOffset = 50.0,
        double BottomOffset = 50.0
    );

    public record LevelThemeColors(Color Background, Color Terrain);

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
        public struct TerrainPoint
        {
            public double X { get; set; }
            public double YTop { get; set; }
            public double YMiddle { get; set; }
            public double YBottom { get; set; }
        }

        public static class Constants
        {
            public const double
                DefaultGroundY = 500.0,         // Базовая высота земли (400-600)
                FinishReachDistance = 50.0,     // Расстояние достижения финиша (10-100)
                DefaultTerrainLength = 3000.0,  // Длина уровня по умолчанию (1000-5000)
                deltaX = 0.1;                   // Шаг для расчета наклона (0.05-0.5)

            public static readonly LevelThemeColors DesertTheme = new(
                Background: Color.FromRgb(255, 204, 102),
                Terrain: Color.FromRgb(204, 153, 102)
            );
        }

        public int Id { get; }
        public string Name { get; }
        public List<TerrainPoint> TerrainPoints { get; private set; } = new();
        public Point StartPoint { get; private set; }
        public Point FinishPoint { get; private set; }
        public double Length { get; private set; }
        public LevelTheme Theme { get; } = LevelTheme.Desert;

        public Color BackgroundColor => DesertTheme.Background;
        public Color TerrainColor => DesertTheme.Terrain;

        public Level(int id, string name, int? seed = null)
        {
            Id = id;
            Name = name;

            Log("Level", $"creating level {id}", () => {
                GenerateLevel(seed ?? id * 100);
                Info("Level", $"Level {id} created: Length={Length:F0}");
            });
        }

        private void GenerateLevel(int seed)
        {
            Log("Level", $"generating with seed {seed}", () => {
                var generator = new LevelGenerator(seed);
                var config = new TerrainConfig(DefaultTerrainLength, TerrainStyle.Default, 50.0, 50.0);
                (TerrainPoints, Length, StartPoint, FinishPoint) = generator.GenerateTerrain(config);
            });
        }

        public double GetGroundYAtX(double x) =>
            Log("Level", "get ground Y", () =>
                IsOutOfBounds(x) ? GetOutOfBoundsValue(x) : InterpolateGroundY(x), DefaultGroundY);

        private bool IsOutOfBounds(double x) =>
            Log("Level", "check bounds", () =>
                TerrainPoints.Count < 2 || x < TerrainPoints[0].X || x > TerrainPoints[^1].X, true);

        private double GetOutOfBoundsValue(double x) =>
            Log("Level", "get OOB value", () => {
                if (x < TerrainPoints[0].X || x > TerrainPoints[^1].X)
                {
                    Warning("Level", $"Out of bounds X={x:F1}");
                    return double.MaxValue;
                }
                return DefaultGroundY;
            }, DefaultGroundY);

        private double InterpolateGroundY(double x) =>
            Log("Level", "interpolate Y", () => {
                int segmentIndex = FindTerrainSegmentIndex(x);
                if (segmentIndex < 0)
                {
                    Warning("Level", $"No segment for X={x:F1}");
                    return DefaultGroundY;
                }

                var (p1, p2) = (TerrainPoints[segmentIndex], TerrainPoints[segmentIndex + 1]);
                return LinearInterpolate(x, p1.X, p1.YMiddle, p2.X, p2.YMiddle);
            }, DefaultGroundY);

        private int FindTerrainSegmentIndex(double x) =>
            Log("Level", "find segment", () => {
                for (int i = 0; i < TerrainPoints.Count - 1; i++)
                {
                    if (x >= TerrainPoints[i].X && x <= TerrainPoints[i + 1].X)
                        return i;
                }
                return -1;
            }, -1);

        private double LinearInterpolate(double x, double x1, double y1, double x2, double y2) =>
            Log("Level", "interpolate", () => {
                double t = (x - x1) / (x2 - x1);
                return y1 * (1 - t) + y2 * t;
            }, y1);

        public double CalculateSlopeAngle(double x) =>
            Log("Level", "calc slope", () => {
                double y1 = GetGroundYAtX(x - deltaX);
                double y2 = GetGroundYAtX(x + deltaX);
                return Math.Atan2(y2 - y1, 2 * deltaX);
            }, 0.0);

        public bool IsFinishReached(Point position) =>
            Log("Level", "check finish", () => {
                double distance = CalculateDistance(position, FinishPoint);
                bool isReached = distance <= FinishReachDistance;
                if (isReached)
                    Info("Level", $"Finish reached at {distance:F1}");
                return isReached;
            }, false);

        private double CalculateDistance(Point p1, Point p2) =>
            Log("Level", "calc distance", () => {
                double dx = p1.X - p2.X, dy = p1.Y - p2.Y;
                return Math.Sqrt(dx * dx + dy * dy);
            }, double.MaxValue);

        public class LevelGenerator
        {
            public static class Constants
            {
                public const double
                    DefaultTerrainHeight = 500.0,      // Стандартная высота местности (400-600)
                    SpikeMaxHeight = 150.0,            // Максимальная высота пиков (50-200)
                    DepressionMaxDepth = 100.0,        // Максимальная глубина впадин (50-150)
                    SmoothingFactor = 0.7,             // Фактор сглаживания (0.5-0.9)
                    PreviousPointWeight = 0.3,         // Вес предыдущей точки (0.1-0.5)
                    StartZonePercent = 0.1,            // Процент начальной зоны (0.05-0.2)
                    EndZonePercent = 0.9,              // Процент конечной зоны (0.8-0.95)
                    StartPointXOffset = 100.0,         // Смещение старта по X (50-200)
                    StartPointYOffset = -80.0,         // Смещение старта по Y (-100-0)
                    FinishPointXOffset = -100.0,       // Смещение финиша по X (-200-0)
                    FinishPointYOffset = -50.0;        // Смещение финиша по Y (-100-0)

                public const int
                    DefaultPointCount = 60,            // Количество точек (30-100)
                    SegmentCount = 5;                  // Количество сегментов (3-10)
            }

            private readonly Random _random;
            private readonly List<TerrainSegmentType> _segmentTypes = new();

            public enum TerrainSegmentType
            {
                Plateau,    // Плато (ровный участок)
                Spike,      // Пик (возвышенность)
                Depression  // Впадина (углубление)
            }

            public LevelGenerator(int seed)
            {
                _random = new Random(seed);
                Log("LevelGenerator", $"init with seed {seed}", () => {
                    GenerateSegmentTypes();
                });
            }

            private void GenerateSegmentTypes() =>
                Log("LevelGenerator", "gen segments", () => {
                    for (int i = 0; i < Constants.SegmentCount; i++)
                    {
                        _segmentTypes.Add((TerrainSegmentType)_random.Next(0, 3));
                    }
                });

            public (List<TerrainPoint> Points, double Length, Point StartPoint, Point FinishPoint)
                GenerateTerrain(TerrainConfig config) =>
                Log("LevelGenerator", "gen terrain", () => {
                    double length = config.Length;
                    var terrainPoints = new List<TerrainPoint>();
                    double startYMiddle = Constants.DefaultTerrainHeight;

                    Point startPoint = CreateStartPoint(terrainPoints, startYMiddle, config);
                    GenerateTerrainPoints(terrainPoints, config, startYMiddle, length);
                    Point finishPoint = CreateEndPoint(terrainPoints, startYMiddle, length, config);

                    Info("LevelGenerator", $"Terrain generated: {config.Style}");
                    return (terrainPoints, length, startPoint, finishPoint);
                }, (new List<TerrainPoint>(), 0.0, new Point(), new Point()));

            private Point CreateStartPoint(List<TerrainPoint> points, double yMiddle, TerrainConfig config) =>
                Log("LevelGenerator", "create start", () => {
                    points.Add(new TerrainPoint
                    {
                        X = 0,
                        YTop = yMiddle - config.TopOffset,
                        YMiddle = yMiddle,
                        YBottom = yMiddle + config.BottomOffset
                    });
                    return new Point(Constants.StartPointXOffset, yMiddle + Constants.StartPointYOffset);
                }, new Point());

            private Point CreateEndPoint(List<TerrainPoint> points, double yMiddle, double length, TerrainConfig config) =>
                Log("LevelGenerator", "create end", () => {
                    points.Add(new TerrainPoint
                    {
                        X = length,
                        YTop = yMiddle - config.TopOffset,
                        YMiddle = yMiddle,
                        YBottom = yMiddle + config.BottomOffset
                    });
                    return new Point(length + Constants.FinishPointXOffset, yMiddle + Constants.FinishPointYOffset);
                }, new Point());

            private void GenerateTerrainPoints(
                List<TerrainPoint> points,
                TerrainConfig config,
                double baseY,
                double length) =>
                Log("LevelGenerator", "gen points", () => {
                    double lastYMiddle = baseY;
                    for (int i = 1; i < Constants.DefaultPointCount; i++)
                    {
                        double progress = (double)i / Constants.DefaultPointCount;
                        double x = progress * length;
                        double yMiddle = CalculateTerrainHeight(config.Style, baseY, progress, lastYMiddle);

                        points.Add(new TerrainPoint
                        {
                            X = x,
                            YTop = yMiddle - config.TopOffset,
                            YMiddle = yMiddle,
                            YBottom = yMiddle + config.BottomOffset
                        });

                        lastYMiddle = yMiddle;
                    }
                });

            private double CalculateTerrainHeight(TerrainStyle style, double baseY, double progress, double lastYMiddle) =>
                Log("LevelGenerator", "calc height", () => {
                    double rawHeight = GetRawTerrainHeight(baseY, progress);
                    double transitionedHeight = ApplyTransitions(rawHeight, baseY, progress);
                    return lastYMiddle * Constants.PreviousPointWeight +
                           transitionedHeight * Constants.SmoothingFactor;
                }, baseY);

            private double GetRawTerrainHeight(double baseY, double progress) =>
                Log("LevelGenerator", "get raw height", () => {
                    int segmentIndex = Math.Min((int)(progress * Constants.SegmentCount), Constants.SegmentCount - 1);
                    TerrainSegmentType segmentType = _segmentTypes[segmentIndex];
                    double segmentProgress = (progress * Constants.SegmentCount) - segmentIndex;

                    return segmentType switch
                    {
                        TerrainSegmentType.Plateau => baseY,
                        TerrainSegmentType.Spike => baseY + GetSpikeVariation(segmentProgress),
                        TerrainSegmentType.Depression => baseY + GetDepressionVariation(segmentProgress),
                        _ => baseY
                    };
                }, baseY);

            private double GetSpikeVariation(double segmentProgress) =>
                Log("LevelGenerator", "spike var", () => {
                    double spikeHeight = _random.NextDouble() * Constants.SpikeMaxHeight;
                    double spikePosition = _random.NextDouble();
                    double distance = Math.Abs(segmentProgress - spikePosition);
                    return spikeHeight * Math.Exp(-2.0 * distance * distance);
                }, 0.0);

            private double GetDepressionVariation(double segmentProgress) =>
                Log("LevelGenerator", "depression var", () => {
                    double depressionDepth = _random.NextDouble() * Constants.DepressionMaxDepth;
                    double depressionPosition = _random.NextDouble();
                    double distance = Math.Abs(segmentProgress - depressionPosition);
                    return -depressionDepth * Math.Exp(-2.0 * distance * distance);
                }, 0.0);

            private double ApplyTransitions(double y, double baseY, double progress) =>
                Log("LevelGenerator", "apply transitions", () => progress switch {
                    < Constants.StartZonePercent =>
                        baseY + (y - baseY) * (progress / Constants.StartZonePercent),
                    > Constants.EndZonePercent =>
                        y + (baseY - y) * ((progress - Constants.EndZonePercent) / (1 - Constants.EndZonePercent)),
                    _ => y
                }, y);
        }
    }
}