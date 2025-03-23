using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;
using GravityDefiedGame.Utilities;
using GravityDefiedGame.Views;
using static System.Math;
using static GravityDefiedGame.Utilities.Logger;
using static GravityDefiedGame.Models.GeneratorConstants;
using static GravityDefiedGame.Models.LevelConstants;
using LevelGenerator = GravityDefiedGame.Models.Level.LevelGenerator;
using TerrainSegmentType = GravityDefiedGame.Models.Level.LevelGenerator.TerrainSegmentType;

namespace GravityDefiedGame.Models
{
    #region Constants

    public static class LevelConstants
    {
        public const double
            DefaultGroundY = 500.0,
            FinishReachDistance = 50.0,
            DeltaX = 0.1,
            InterpolationFactor = 1.0;

        public const int DefaultSeedMultiplier = 100;
        public const double
            BaseTerrainLength = 3000.0,
            TerrainLengthIncreasePerLevel = 500.0,
            MaxTerrainLength = 10000.0;

        // safe zone length for all levels
        public const double FixedSafeZoneLength = 300.0;
    }

    public static class GeneratorConstants
    {
        public const double DefaultTerrainHeight = 500.0;

        public const double
            BaseSpikeHeight = 50.0,
            SpikeHeightIncreasePerLevel = 10.0,
            MaxSpikeHeight = 200.0,
            SpikeHeightFactor = 0.5,
            SpikeMidFactor = 0.7;

        public const double
            BaseDepressionDepth = 30.0,
            DepressionDepthIncreasePerLevel = 7.5,
            MaxDepressionDepth = 150.0;

        public const double
            SmoothingFactor = 0.7,
            BasePreviousPointWeight = 0.3,
            PreviousPointWeightDecreasePerLevel = 0.02,
            MinPreviousPointWeight = 0.1,
            GaussianFactor = 2.0;

        public const double
            StartZonePercent = 0.1,
            EndZonePercent = 0.9;

        public const double
            PointOffset = 50.0,
            StartPointXOffset = 100.0,
            StartPointYOffset = -80.0,
            FinishPointXOffset = -100.0,
            FinishPointYOffset = -50.0;

        public const int
            BasePointCount = 60,
            PointCountIncreasePerLevel = 5,
            MaxPointCount = 120,
            BaseSegmentCount = 5,
            SegmentCountIncreasePerLevel = 1,
            MaxSegmentCount = 15,
            MinSafeZonePointCount = 3,
            PointCountReductionFactor = 5;

        public const int
            MinTerrainTypeCount = 3,
            MaxTerrainTypeCount = 5,
            TerrainTypeUnlockLevel = 5;

        public const double
            SegmentDifficultyThreshold = 5,
            SegmentDifficultyChance = 0.2,
            SegmentDifficultyDivisor = 10.0,
            FinalPlateauChance = 0.7;

        public const double
            NoiseThreshold = 3,
            NoiseAmplitudeFactor = 5.0,
            MaxNoiseAmplitude = 25.0;
    }

    #endregion Constants

    #region Types

    public record TerrainConfig(
        double Length,
        double TopOffset,
        double BottomOffset,
        int Difficulty
    );

    public enum LevelTheme
    {
        Desert,
        Mountain,
        Arctic,
        Volcano
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
        public int Difficulty { get; internal set; }
        public List<TerrainPoint> TerrainPoints { get; private set; } = new();
        public Point StartPoint { get; private set; }
        public Point FinishPoint { get; private set; }
        public double Length { get; private set; }
        public LevelTheme Theme { get; private set; }
        public double SafeZoneStartLength { get; private set; }
        public double SafeZoneEndLength { get; private set; }
        public Color VerticalLineColor => ThemeConstants.VerticalLineColor;
        public Color BackgroundColor => ThemeConstants.BackgroundColor;
        public Color TerrainColor => ThemeConstants.TerrainColor;
        public Color SafeZoneColor => ThemeConstants.SafeZoneColor;

        #endregion Properties

        #region Constructors

        public Level(int id, string name, int? seed = null, int? difficulty = null)
        {
            Log("Level", $"Creating level {id}", () =>
            {
                Id = id;
                Name = name;
                Difficulty = difficulty ?? id;
                Theme = GetThemeForLevel(id);
                GenerateLevel(seed ?? id * DefaultSeedMultiplier);
                Info("Level", $"Level {id} created: Length={Length:F0}, Difficulty={Difficulty}");
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
            Log("Level", "Checking finish", () =>
            {
                double distance = CalculateDistance(pos, FinishPoint);
                bool reached = distance <= FinishReachDistance;

                if (reached)
                    Info("Level", $"Finish reached at distance {distance:F1}");

                return reached;
            }, false);

        #endregion Public Methods

        #region Private Methods

        private LevelTheme GetThemeForLevel(int level)
        {
            int themeCount = Enum.GetValues(typeof(LevelTheme)).Length;
            return (LevelTheme)(level % themeCount);
        }

        private void GenerateLevel(int seed) =>
            Log("Level", $"Generating with seed {seed}", () =>
            {
                var gen = new LevelGenerator(seed, Difficulty, new DefaultSegmentSelectionStrategy());

                double terrainLength = Min(BaseTerrainLength + TerrainLengthIncreasePerLevel * Difficulty, MaxTerrainLength);

                // Using fixed safe zone length for all levels
                var config = new TerrainConfig(terrainLength, PointOffset, PointOffset, Difficulty);

                (TerrainPoints, Length, StartPoint, FinishPoint, SafeZoneStartLength, SafeZoneEndLength) = gen.GenerateTerrain(config);
            });

        private bool IsOutOfBounds(double x) =>
            TerrainPoints.Count < 2 || x < TerrainPoints[0].X || x > TerrainPoints[^1].X;

        private double GetOutOfBoundsValue(double x) =>
            Log("Level", "Getting OOB value", () =>
            {
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

            var (p1, p2) = (TerrainPoints[idx], TerrainPoints[idx + 1]);
            return LinearInterpolate(x, p1.X, p1.YMiddle, p2.X, p2.YMiddle);
        }

        private int FindTerrainSegmentIndex(double x)
        {
            int left = 0, right = TerrainPoints.Count - 1;
            while (left <= right)
            {
                int mid = left + (right - left) / 2;
                if (TerrainPoints[mid].X <= x && (mid == TerrainPoints.Count - 1 || TerrainPoints[mid + 1].X > x))
                    return mid;
                else if (TerrainPoints[mid].X < x)
                    left = mid + 1;
                else
                    right = mid - 1;
            }
            return -1;
        }

        private double LinearInterpolate(double x, double x1, double y1, double x2, double y2) =>
            y1 + (y2 - y1) * ((x - x1) / (x2 - x1));

        private double CalculateDistance(Point p1, Point p2) =>
            Sqrt(Pow(p2.X - p1.X, 2) + Pow(p2.Y - p1.Y, 2));

        #endregion Private Methods

        #region LevelGenerator

        public class LevelGenerator
        {
            #region Enums and Fields

            public enum TerrainSegmentType
            {
                Plateau,
                Spike,
                Depression,
                StepUp,
                StepDown
            }

            private readonly Random _rand;
            private readonly List<TerrainSegmentType> _segmentTypes = new();
            private readonly Dictionary<TerrainSegmentType, Func<double, double>> _variationStrategies;
            private readonly int _difficulty;
            private readonly double _spikeHeight;
            private readonly double _depressionDepth;
            private readonly double _previousPointWeight;
            private readonly int _segmentCount;
            private readonly int _pointCount;
            private readonly int _terrainTypeCount;
            private readonly ISegmentSelectionStrategy _segmentStrategy;

            #endregion Enums and Fields

            #region Constructor

            public LevelGenerator(int seed, int difficulty, ISegmentSelectionStrategy segmentStrategy)
            {
                _rand = new Random(seed);
                _difficulty = Max(1, difficulty);
                _segmentStrategy = segmentStrategy ?? throw new ArgumentNullException(nameof(segmentStrategy));

                _spikeHeight = Min(BaseSpikeHeight + SpikeHeightIncreasePerLevel * _difficulty, MaxSpikeHeight);
                _depressionDepth = Min(BaseDepressionDepth + DepressionDepthIncreasePerLevel * _difficulty, MaxDepressionDepth);
                _previousPointWeight = Max(BasePreviousPointWeight - PreviousPointWeightDecreasePerLevel * _difficulty, MinPreviousPointWeight);
                _segmentCount = Min(BaseSegmentCount + SegmentCountIncreasePerLevel * _difficulty, MaxSegmentCount);
                _pointCount = Min(BasePointCount + PointCountIncreasePerLevel * _difficulty, MaxPointCount);

                if (_pointCount <= 0)
                    throw new ArgumentException("Количество точек должно быть положительным", nameof(_pointCount));

                _terrainTypeCount = Min(MinTerrainTypeCount + (_difficulty / TerrainTypeUnlockLevel), MaxTerrainTypeCount);

                _variationStrategies = new Dictionary<TerrainSegmentType, Func<double, double>>
                {
                    [TerrainSegmentType.Plateau] = _ => 0,
                    [TerrainSegmentType.Spike] = prog =>
                    {
                        double spike = (_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _spikeHeight;
                        double pos = _rand.NextDouble();
                        return spike * Exp(-GaussianFactor * Pow(prog - pos, 2));
                    },
                    [TerrainSegmentType.Depression] = prog =>
                    {
                        double dep = (_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _depressionDepth;
                        double pos = _rand.NextDouble();
                        return -dep * Exp(-GaussianFactor * Pow(prog - pos, 2));
                    },
                    [TerrainSegmentType.StepUp] = prog =>
                    {
                        double height = _rand.NextDouble() * _spikeHeight * SpikeMidFactor;
                        return height * Min(1.0, prog * 2);
                    },
                    [TerrainSegmentType.StepDown] = prog =>
                    {
                        double depth = _rand.NextDouble() * _depressionDepth * SpikeMidFactor;
                        return -depth * Min(1.0, prog * 2);
                    }
                };

                GenerateSegmentTypes();
            }

            private void GenerateSegmentTypes()
            {
                _segmentTypes.Add(TerrainSegmentType.Plateau);
                for (int i = 1; i < _segmentCount; i++)
                {
                    TerrainSegmentType nextType = _segmentStrategy.SelectSegment(_segmentTypes, i, _terrainTypeCount, _rand, _difficulty);
                    _segmentTypes.Add(nextType);
                }

                if (_segmentTypes.Count > 2 && _rand.NextDouble() < FinalPlateauChance)
                    _segmentTypes[^1] = TerrainSegmentType.Plateau;
            }

            #endregion Constructor

            #region Public Methods

            public (List<TerrainPoint> Points, double Length, Point StartPoint, Point FinishPoint,
                    double SafeZoneStart, double SafeZoneEnd) GenerateTerrain(TerrainConfig config) =>
                Log("LevelGenerator", "Generating terrain", () =>
                {
                    double length = config.Length;
                    var pts = new List<TerrainPoint>();
                    double baseY = DefaultTerrainHeight;

                    // Using fixed safe zone length for all levels
                    double safeZoneLength = FixedSafeZoneLength;

                    Point sp = CreateStartPoint(pts, baseY, config);
                    GenerateSafeZone(pts, baseY, config, 0, safeZoneLength);
                    GenerateTerrainPoints(pts, config, baseY, length, safeZoneLength, safeZoneLength);
                    GenerateSafeZone(pts, baseY, config, length - safeZoneLength, safeZoneLength);
                    Point fp = CreateEndPoint(pts, baseY, length, config);

                    Info("LevelGenerator", $"Generated {pts.Count} terrain points with difficulty {_difficulty}");
                    return (pts, length, sp, fp, safeZoneLength, safeZoneLength);
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
                int count = Max(MinSafeZonePointCount, (int)(length / config.Length * _pointCount * 0.5));
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
            }

            private void GenerateTerrainPoints(List<TerrainPoint> pts, TerrainConfig config, double baseY, double length, double safeStart, double safeEnd) =>
                Log("LevelGenerator", "Generating main terrain", () =>
                {
                    double midLen = length - safeStart - safeEnd;
                    double lastY = baseY;
                    int count = _pointCount - pts.Count - PointCountReductionFactor;
                    for (int i = 1; i <= count; i++)
                    {
                        double progress = (double)i / count;
                        double x = safeStart + progress * midLen;
                        double y = CalculateTerrainHeight(baseY, progress, lastY);
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
                });

            private double CalculateTerrainHeight(double baseY, double progress, double lastY)
            {
                double raw = GetRawTerrainHeight(baseY, progress);
                double trans = ApplyTransitions(raw, baseY, progress);
                double smoothValue = lastY * _previousPointWeight + trans * SmoothingFactor;
                smoothValue = ApplyNoise(smoothValue);
                return smoothValue;
            }

            private double ApplyNoise(double value)
            {
                if (_difficulty > NoiseThreshold)
                {
                    double noiseAmplitude = Min(NoiseAmplitudeFactor * (_difficulty - NoiseThreshold), MaxNoiseAmplitude);
                    value += (_rand.NextDouble() * 2 - 1) * noiseAmplitude;
                }
                return value;
            }

            private double GetRawTerrainHeight(double baseY, double progress)
            {
                int segmentIndex = Min((int)(progress * _segmentCount), _segmentCount - 1);
                double segmentProgress = progress * _segmentCount - segmentIndex;
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

    #region Segment Selection Strategy

    public interface ISegmentSelectionStrategy
    {
        TerrainSegmentType SelectSegment(
            List<TerrainSegmentType> previousSegments,
            int currentIndex,
            int terrainTypeCount,
            Random rand,
            int difficulty);
    }

    public class DefaultSegmentSelectionStrategy : ISegmentSelectionStrategy
    {
        public TerrainSegmentType SelectSegment(
            List<TerrainSegmentType> previousSegments,
            int currentIndex,
            int terrainTypeCount,
            Random rand,
            int difficulty)
        {
            // выбираем случайный тип, не равный предыдущему.
            var types = Enum.GetValues(typeof(TerrainSegmentType));
            TerrainSegmentType nextType = (TerrainSegmentType)rand.Next(terrainTypeCount);

            if (currentIndex > 0 && nextType == previousSegments[currentIndex - 1])
            {
                nextType = (TerrainSegmentType)((rand.Next(terrainTypeCount - 1) + (int)nextType + 1) % terrainTypeCount);
            }
            if (difficulty > SegmentDifficultyThreshold &&
                rand.NextDouble() < SegmentDifficultyChance * (difficulty / SegmentDifficultyDivisor))
            {
                nextType = (TerrainSegmentType)rand.Next(1, terrainTypeCount);
            }
            return nextType;
        }
    }

    #endregion Segment Selection Strategy
}