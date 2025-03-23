using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using GravityDefiedGame.Utilities;
using GravityDefiedGame.Views;
using static System.Math;
using static GravityDefiedGame.Utilities.Logger;
using static GravityDefiedGame.Models.LevelConstants;
using static GravityDefiedGame.Models.GeneratorConstants;

namespace GravityDefiedGame.Models
{
    #region Constants

    public static class LevelConstants
    {
        public const double DefaultGroundY = 500.0;           // Базовая высота земли
        public const double FinishReachDistance = 50.0;       // Дистанция для достижения финиша
        public const double DefaultTerrainLength = 3000.0;    // Стандартная длина уровня
        public const double DeltaX = 0.1;                     // Шаг для расчета наклона
        public const double DefaultSafeZoneLength = 300.0;    // Длина безопасной зоны
        public const double InterpolationFactor = 1.0;        // Фактор интерполяции

        public const int DefaultSeedMultiplier = 100;         // Множитель для генерации
    }

    public static class GeneratorConstants
    {
        public const double DefaultTerrainHeight = 500.0;     // Высота ландшафта
        public const double SpikeMaxHeight = 150.0;           // Максимальная высота пиков
        public const double DepressionMaxDepth = 100.0;       // Максимальная глубина впадин
        public const double SmoothingFactor = 0.7;            // Фактор сглаживания
        public const double PreviousPointWeight = 0.3;        // Вес предыдущей точки
        public const double StartZonePercent = 0.1;           // Процент начальной зоны
        public const double EndZonePercent = 0.9;             // Процент конечной зоны
        public const double PointOffset = 50.0;               // Смещение точек
        public const double StartPointXOffset = 100.0;        // Смещение старта по X
        public const double StartPointYOffset = -80.0;        // Смещение старта по Y
        public const double FinishPointXOffset = -100.0;      // Смещение финиша по X
        public const double FinishPointYOffset = -50.0;       // Смещение финиша по Y
        public const double GaussianFactor = 2.0;             // Фактор Гаусса для рельефа

        public const int DefaultPointCount = 60;              // Количество точек рельефа
        public const int SegmentCount = 5;                    // Количество сегментов
        public const int MinSafeZonePointCount = 3;           // Мин. точек в безопасной зоне
        public const int TerrainTypeCount = 3;                // Количество типов местности
        public const int PointCountReductionFactor = 5;       // Фактор сокращения точек
    }

    #endregion Constants

    #region Types

    public record TerrainConfig(
        double Length = 3000.0,
        TerrainStyle Style = TerrainStyle.Flat,
        double TopOffset = 50.0,
        double BottomOffset = 50.0,
        double SafeZoneStartLength = 300.0,
        double SafeZoneEndLength = 300.0
    );

    public enum TerrainStyle
    {
        Default,
        Flat
    }

    public enum LevelTheme
    {
        Desert
    }

    #endregion Types

    public class Level : PhysicsComponent
    {
        #region Structs

        public struct TerrainPoint
        {
            public double X { get; set; }
            public double YTop { get; set; }
            public double YMiddle { get; set; }
            public double YBottom { get; set; }
            public bool IsSafeZone { get; set; }
        }

        #endregion Structs

        #region Properties

        public int Id { get; internal set; }
        public string Name { get; internal set; } = string.Empty;
        public List<TerrainPoint> TerrainPoints { get; private set; } = new();
        public Point StartPoint { get; private set; }
        public Point FinishPoint { get; private set; }
        public double Length { get; private set; }
        public LevelTheme Theme { get; } = LevelTheme.Desert;
        public double SafeZoneStartLength { get; private set; }
        public double SafeZoneEndLength { get; private set; }

        public Color VerticalLineColor => ThemeConstants.VerticalLineColor;
        public Color BackgroundColor => ThemeConstants.BackgroundColor;
        public Color TerrainColor => ThemeConstants.TerrainColor;
        public Color SafeZoneColor => ThemeConstants.SafeZoneColor;

        #endregion Properties

        #region Constructors

        public Level(int id, string name, int? seed = null)
        {
            Log("Level", $"Creating level {id}", () => {
                Id = id;
                Name = name;
                GenerateLevel(seed ?? id * DefaultSeedMultiplier);
                Info("Level", $"Level {id} created: Length={Length:F0}, SafeZones: {SafeZoneStartLength:F0}/{SafeZoneEndLength:F0}");
            });
        }

        public Level() { }

        #endregion Constructors

        #region Public Methods

        public double GetGroundYAtX(double x) =>
            IsOutOfBounds(x) ? GetOutOfBoundsValue(x) : InterpolateGroundY(x);

        public bool IsInSafeZone(double x) =>
            !IsOutOfBounds(x) && (x <= SafeZoneStartLength || x >= Length - SafeZoneEndLength);

        public double CalculateSlopeAngle(double x) =>
            Atan2(GetGroundYAtX(x + DeltaX) - GetGroundYAtX(x - DeltaX), 2 * DeltaX);

        public bool IsFinishReached(Point pos) =>
            Log("Level", "Checking finish", () => {
                double distance = CalculateDistance(pos, FinishPoint);
                bool reached = distance <= FinishReachDistance;

                if (reached)
                    Info("Level", $"Finish reached at distance {distance:F1}");

                return reached;
            }, false);

        #endregion Public Methods

        #region Private Methods

        private void GenerateLevel(int seed) =>
            Log("Level", $"Generating with seed {seed}", () => {
                var gen = new LevelGenerator(seed);
                var config = new TerrainConfig(
                    DefaultTerrainLength,
                    TerrainStyle.Default,
                    50.0,
                    50.0,
                    DefaultSafeZoneLength,
                    DefaultSafeZoneLength
                );

                (TerrainPoints, Length, StartPoint, FinishPoint, SafeZoneStartLength, SafeZoneEndLength) =
                    gen.GenerateTerrain(config);

                Debug("Level", $"Generated terrain with {TerrainPoints.Count} points");
            });

        private bool IsOutOfBounds(double x) =>
            TerrainPoints.Count < 2 || x < TerrainPoints[0].X || x > TerrainPoints[^1].X;

        private double GetOutOfBoundsValue(double x) =>
            Log("Level", "Getting OOB value", () => {
                if (x < TerrainPoints[0].X || x > TerrainPoints[^1].X)
                {
                    Warning("Level", $"Out of bounds X={x:F1}");
                    return double.MaxValue;
                }
                return DefaultGroundY;
            }, DefaultGroundY);

        private double InterpolateGroundY(double x)
        {
            int idx = FindTerrainSegmentIndex(x);
            if (idx < 0)
            {
                Warning("Level", $"No segment found for X={x:F1}");
                return DefaultGroundY;
            }

            var p1 = TerrainPoints[idx];
            var p2 = TerrainPoints[idx + 1];
            return LinearInterpolate(x, p1.X, p1.YMiddle, p2.X, p2.YMiddle);
        }

        private int FindTerrainSegmentIndex(double x)
        {
            for (int i = 0; i < TerrainPoints.Count - 1; i++)
                if (x >= TerrainPoints[i].X && x <= TerrainPoints[i + 1].X)
                    return i;
            return -1;
        }

        private double LinearInterpolate(double x, double x1, double y1, double x2, double y2) =>
            y1 + (y2 - y1) * ((x - x1) / (x2 - x1));

        #endregion Private Methods

        #region LevelGenerator

        private class LevelGenerator
        {
            #region Enums and Fields

            private enum TerrainSegmentType { Plateau, Spike, Depression }

            private readonly Random _rand;
            private readonly List<TerrainSegmentType> _segmentTypes = new();
            private readonly Dictionary<TerrainSegmentType, Func<double, double>> _variationStrategies;

            #endregion Enums and Fields

            #region Constructor

            public LevelGenerator(int seed)
            {
                _rand = new Random(seed);

                _variationStrategies = new Dictionary<TerrainSegmentType, Func<double, double>>
                {
                    [TerrainSegmentType.Plateau] = _ => 0,
                    [TerrainSegmentType.Spike] = prog => {
                        double spike = _rand.NextDouble() * SpikeMaxHeight;
                        double pos = _rand.NextDouble();
                        return spike * Exp(-GaussianFactor * Pow(prog - pos, 2));
                    },
                    [TerrainSegmentType.Depression] = prog => {
                        double dep = _rand.NextDouble() * DepressionMaxDepth;
                        double pos = _rand.NextDouble();
                        return -dep * Exp(-GaussianFactor * Pow(prog - pos, 2));
                    }
                };

                for (int i = 0; i < SegmentCount; i++)
                    _segmentTypes.Add((TerrainSegmentType)_rand.Next(TerrainTypeCount));

                Debug("LevelGenerator", $"Initialized with {SegmentCount} segments, seed={seed}");
            }

            #endregion Constructor

            #region Public Methods

            public (List<TerrainPoint> Points, double Length, Point StartPoint, Point FinishPoint,
                    double SafeZoneStart, double SafeZoneEnd) GenerateTerrain(TerrainConfig config) =>
                Log("LevelGenerator", "Generating terrain", () => {
                    double length = config.Length;
                    var pts = new List<TerrainPoint>();
                    double baseY = DefaultTerrainHeight;

                    Point sp = CreateStartPoint(pts, baseY, config);
                    GenerateSafeZone(pts, baseY, config, 0, config.SafeZoneStartLength);
                    GenerateTerrainPoints(pts, config, baseY, length, config.SafeZoneStartLength, config.SafeZoneEndLength);
                    GenerateSafeZone(pts, baseY, config, length - config.SafeZoneEndLength, config.SafeZoneEndLength);
                    Point fp = CreateEndPoint(pts, baseY, length, config);

                    Info("LevelGenerator", $"Generated {pts.Count} terrain points for style {config.Style}");
                    return (pts, length, sp, fp, config.SafeZoneStartLength, config.SafeZoneEndLength);
                }, (new List<TerrainPoint>(), 0, new Point(), new Point(), 0, 0));

            #endregion Public Methods

            #region Private Methods

            private Point CreateStartPoint(List<TerrainPoint> pts, double baseY, TerrainConfig config)
            {
                pts.Add(new TerrainPoint
                {
                    X = 0,
                    YTop = baseY - config.TopOffset,
                    YMiddle = baseY,
                    YBottom = baseY + config.BottomOffset,
                    IsSafeZone = true
                });
                return new Point(StartPointXOffset, baseY + StartPointYOffset);
            }

            private Point CreateEndPoint(List<TerrainPoint> pts, double baseY, double length, TerrainConfig config)
            {
                pts.Add(new TerrainPoint
                {
                    X = length,
                    YTop = baseY - config.TopOffset,
                    YMiddle = baseY,
                    YBottom = baseY + config.BottomOffset,
                    IsSafeZone = true
                });
                return new Point(length + FinishPointXOffset, baseY + FinishPointYOffset);
            }

            private void GenerateSafeZone(List<TerrainPoint> pts, double baseY, TerrainConfig config, double startX, double length)
            {
                int count = Max(MinSafeZonePointCount, (int)(length / config.Length * DefaultPointCount * 0.5));

                for (int i = 1; i <= count; i++)
                {
                    pts.Add(new TerrainPoint
                    {
                        X = startX + i * length / count,
                        YTop = baseY - config.TopOffset,
                        YMiddle = baseY,
                        YBottom = baseY + config.BottomOffset,
                        IsSafeZone = true
                    });
                }

                Debug("LevelGenerator", $"Safe zone generated at X={startX:F0}, length={length:F0}, points={count}");
            }

            private void GenerateTerrainPoints(List<TerrainPoint> pts, TerrainConfig config, double baseY, double length,
                                              double safeStart, double safeEnd) =>
                Log("LevelGenerator", "Generating main terrain", () => {
                    double midLen = length - safeStart - safeEnd;
                    double lastY = baseY;
                    int count = DefaultPointCount - pts.Count - PointCountReductionFactor;

                    for (int i = 1; i <= count; i++)
                    {
                        double progress = (double)i / count;
                        double x = safeStart + progress * midLen;
                        double y = CalculateTerrainHeight(config.Style, baseY, progress, lastY);

                        pts.Add(new TerrainPoint
                        {
                            X = x,
                            YTop = y - config.TopOffset,
                            YMiddle = y,
                            YBottom = y + config.BottomOffset,
                            IsSafeZone = false
                        });

                        lastY = y;
                    }

                    Debug("LevelGenerator", $"Generated {count} main terrain points");
                });

            private double CalculateTerrainHeight(TerrainStyle style, double baseY, double progress, double lastY)
            {
                double raw = GetRawTerrainHeight(baseY, progress);
                double trans = ApplyTransitions(raw, baseY, progress);
                return lastY * PreviousPointWeight + trans * SmoothingFactor;
            }

            private double GetRawTerrainHeight(double baseY, double progress)
            {
                int segmentIndex = Min((int)(progress * SegmentCount), SegmentCount - 1);
                double segmentProgress = progress * SegmentCount - segmentIndex;
                TerrainSegmentType segmentType = _segmentTypes[segmentIndex];

                return baseY + _variationStrategies[segmentType](segmentProgress);
            }

            private double ApplyTransitions(double y, double baseY, double progress)
            {
                if (progress < StartZonePercent)
                    return baseY + (y - baseY) * (progress / StartZonePercent);

                if (progress > EndZonePercent)
                {
                    double normalizedProgress = (progress - EndZonePercent) / (1 - EndZonePercent);
                    return y + (baseY - y) * normalizedProgress;
                }

                return y;
            }

            #endregion Private Methods
        }

        #endregion LevelGenerator
    }
}