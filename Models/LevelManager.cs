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
            MaxTerrainTypeCount = 7,
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
            public double X { get; }
            public double YTop { get; }
            public double YMiddle { get; }
            public double YBottom { get; }
            public bool IsSafeZone { get; }

            public TerrainPoint(double x, double yTop, double yMiddle, double yBottom, bool isSafeZone)
            {
                X = x;
                YTop = yTop;
                YMiddle = yMiddle;
                YBottom = yBottom;
                IsSafeZone = isSafeZone;
            }
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

        private int _currentSegmentIndex = 0;

        #endregion Properties

        #region Constructors

        public Level(int id, string name, int? seed = null, int? difficulty = null)
        {
#if DEBUG
            Log("Level", $"Creating level {id}", () =>
            {
#endif
                Id = id;
                Name = name;
                Difficulty = difficulty ?? id;
                Theme = GetThemeForLevel(id);

                int levelSeed = seed ?? (id * DefaultSeedMultiplier + DateTime.Now.Millisecond);
                GenerateLevel(levelSeed);
#if DEBUG
                Info("Level", $"Level {id} created: Length={Length:F0}, Difficulty={Difficulty}");
            });
#endif
        }

        public Level() { }

        #endregion Constructors

        #region Public Methods

        public double GetGroundYAtX(double x)
        {
            if (IsOutOfBounds(x)) return GetOutOfBoundsValue(x);
            UpdateSegmentIndex(x);
            return InterpolateGroundY(x);
        }

        public bool IsInSafeZone(double x) =>
            !IsOutOfBounds(x) && (x <= SafeZoneStartLength || x >= Length - SafeZoneEndLength);

        public double CalculateSlopeAngle(double x) =>
            Atan2(GetGroundYAtX(x + DeltaX) - GetGroundYAtX(x - DeltaX), 2 * DeltaX);

        public bool IsFinishReached(Point pos) =>
#if DEBUG
            Log("Level", "Checking finish", () =>
            {
#endif
                double distance = CalculateDistance(pos, FinishPoint);
                bool reached = distance <= FinishReachDistance;
#if DEBUG
                if (reached)
                    Info("Level", $"Finish reached at distance {distance:F1}");
#endif
                return reached;
#if DEBUG
            }, false);
#endif

        #endregion Public Methods

        #region Private Methods

        private LevelTheme GetThemeForLevel(int level)
        {
            int themeCount = Enum.GetValues(typeof(LevelTheme)).Length;
            return (LevelTheme)(level % themeCount);
        }

        private void GenerateLevel(int seed)
        {
#if DEBUG
            Log("Level", $"Generating with seed {seed}", () =>
            {
#endif
                var gen = new LevelGenerator(seed, Difficulty, new SegmentSelection());
                double terrainLength = Min(BaseTerrainLength + TerrainLengthIncreasePerLevel * Difficulty, MaxTerrainLength);
                var config = new TerrainConfig(terrainLength, PointOffset, PointOffset, Difficulty);
                (TerrainPoints, Length, StartPoint, FinishPoint, SafeZoneStartLength, SafeZoneEndLength) = gen.GenerateTerrain(config);
#if DEBUG
            });
#endif
        }

        private bool IsOutOfBounds(double x) =>
            TerrainPoints.Count < 2 || x < TerrainPoints[0].X || x > TerrainPoints[TerrainPoints.Count - 1].X;

        private double GetOutOfBoundsValue(double x)
        {
#if DEBUG
            return Log("Level", "Getting OOB value", () =>
            {
                if (x < TerrainPoints[0].X || x > TerrainPoints[TerrainPoints.Count - 1].X)
                {
                    Warning("Level", $"Out of bounds X={x:F1}");
                    return double.MaxValue;
                }
                return DefaultGroundY;
            }, DefaultGroundY);
#else
            return DefaultGroundY;
#endif
        }

        private void UpdateSegmentIndex(double x)
        {
            if (_currentSegmentIndex >= 0 && _currentSegmentIndex < TerrainPoints.Count - 1)
            {
                var p1 = TerrainPoints[_currentSegmentIndex];
                var p2 = TerrainPoints[_currentSegmentIndex + 1];
                if (x >= p1.X && x < p2.X)
                    return;
            }

            if (_currentSegmentIndex < TerrainPoints.Count - 2)
            {
                var p1 = TerrainPoints[_currentSegmentIndex + 1];
                var p2 = TerrainPoints[_currentSegmentIndex + 2];
                if (x >= p1.X && x < p2.X)
                {
                    _currentSegmentIndex++;
                    return;
                }
            }

            if (_currentSegmentIndex > 0)
            {
                var p1 = TerrainPoints[_currentSegmentIndex - 1];
                var p2 = TerrainPoints[_currentSegmentIndex];
                if (x >= p1.X && x < p2.X)
                {
                    _currentSegmentIndex--;
                    return;
                }
            }

            _currentSegmentIndex = FindTerrainSegmentIndex(x);
        }

        private double InterpolateGroundY(double x)
        {
            if (_currentSegmentIndex < 0 || _currentSegmentIndex >= TerrainPoints.Count - 1)
            {
#if DEBUG
                Warning("Level", $"No segment found for X={x:F1}");
#endif
                return DefaultGroundY;
            }

            var p1 = TerrainPoints[_currentSegmentIndex];
            var p2 = TerrainPoints[_currentSegmentIndex + 1];
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
                StepDown,
                Wave,
                Jump
            }

            private readonly Random _rand;
            private readonly List<(TerrainSegmentType Type, object? Parameters)> _segments = new();
            private readonly int _difficulty;
            private readonly double _spikeHeight;
            private readonly double _depressionDepth;
            private readonly double _previousPointWeight;
            private readonly int _segmentCount;
            private readonly int _pointCount;
            private readonly int _terrainTypeCount;
            private readonly ISegmentSelection _segment;

            #endregion Enums and Fields

            #region Constructor

            public LevelGenerator(int seed, int difficulty, ISegmentSelection segment)
            {
                _rand = new Random(seed);
                _difficulty = Max(1, difficulty);
                _segment = segment ?? throw new ArgumentNullException(nameof(segment));

                _spikeHeight = Min(BaseSpikeHeight + SpikeHeightIncreasePerLevel * _difficulty, MaxSpikeHeight);
                _depressionDepth = Min(BaseDepressionDepth + DepressionDepthIncreasePerLevel * _difficulty, MaxDepressionDepth);
                _previousPointWeight = Max(BasePreviousPointWeight - PreviousPointWeightDecreasePerLevel * _difficulty, MinPreviousPointWeight);

                int baseSegments = BaseSegmentCount + SegmentCountIncreasePerLevel * _difficulty;
                int variation = _rand.Next(-2, 3);
                _segmentCount = Max(1, Min(baseSegments + variation, MaxSegmentCount));

                _pointCount = Min(BasePointCount + PointCountIncreasePerLevel * _difficulty, MaxPointCount);

                if (_pointCount <= 0)
                    throw new ArgumentException("Количество точек должно быть положительным", nameof(_pointCount));

                _terrainTypeCount = Min(MinTerrainTypeCount + (_difficulty / TerrainTypeUnlockLevel), MaxTerrainTypeCount);

                GenerateSegmentTypes();
            }

            private void GenerateSegmentTypes()
            {
                _segments.Clear();
                _segments.Add((TerrainSegmentType.Plateau, null));

                for (int i = 1; i < _segmentCount; i++)
                {
                    TerrainSegmentType nextType = _segment.SelectSegment(_segments.Select(s => s.Type).ToList(), i, _terrainTypeCount, _rand, _difficulty);
                    object parameters = GenerateParameters(nextType);
                    _segments.Add((nextType, parameters));
                }

                if (_segments.Count > 2 && _rand.NextDouble() < FinalPlateauChance)
                    _segments[_segments.Count - 1] = (TerrainSegmentType.Plateau, null);
            }

            private object? GenerateParameters(TerrainSegmentType type)
            {
                switch (type)
                {
                    case TerrainSegmentType.Plateau:
                        return null;
                    case TerrainSegmentType.Spike:
                        return new { Height = (_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _spikeHeight, Pos = _rand.NextDouble() };
                    case TerrainSegmentType.Depression:
                        return new { Depth = (_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _depressionDepth, Pos = _rand.NextDouble() };
                    case TerrainSegmentType.StepUp:
                        return new { Height = _rand.NextDouble() * _spikeHeight * SpikeMidFactor };
                    case TerrainSegmentType.StepDown:
                        return new { Depth = _rand.NextDouble() * _depressionDepth * SpikeMidFactor };
                    case TerrainSegmentType.Wave:
                        return new { Amplitude = _rand.NextDouble() * _spikeHeight * 0.5, Frequency = 2 + _rand.NextDouble() * 2 };
                    case TerrainSegmentType.Jump:
                        return new { Height = _rand.NextDouble() * _spikeHeight, JumpPos = 0.3 + _rand.NextDouble() * 0.4 };
                    default:
                        throw new ArgumentException("Неизвестный тип сегмента");
                }
            }

            #endregion Constructor

            #region Public Methods

            public (List<TerrainPoint> Points, double Length, Point StartPoint, Point FinishPoint,
                    double SafeZoneStart, double SafeZoneEnd) GenerateTerrain(TerrainConfig config) =>
#if DEBUG
                Log("LevelGenerator", "Generating terrain", () =>
                {
#endif
                    double length = config.Length;
                    var pts = new List<TerrainPoint>();
                    double baseY = DefaultTerrainHeight;

                    double safeZoneLength = FixedSafeZoneLength;

                    Point sp = CreateStartPoint(pts, baseY, config);
                    GenerateSafeZone(pts, baseY, config, 0, safeZoneLength);
                    GenerateTerrainPoints(pts, config, baseY, length, safeZoneLength, safeZoneLength);
                    GenerateSafeZone(pts, baseY, config, length - safeZoneLength, safeZoneLength);
                    Point fp = CreateEndPoint(pts, baseY, length, config);

#if DEBUG
                    Info("LevelGenerator", $"Generated {pts.Count} terrain points with difficulty {_difficulty}");
#endif
                    return (pts, length, sp, fp, safeZoneLength, safeZoneLength);
#if DEBUG
                }, (new List<TerrainPoint>(), 0, new Point(), new Point(), 0, 0));
#endif

            #endregion Public Methods

            #region Private Methods

            private Point CreateStartPoint(List<TerrainPoint> pts, double baseY, TerrainConfig config)
            {
                pts.Add(new TerrainPoint(
                    x: 0,
                    yTop: baseY - config.TopOffset,
                    yMiddle: baseY,
                    yBottom: baseY + config.BottomOffset,
                    isSafeZone: true
                ));
                return new Point(StartPointXOffset, baseY + StartPointYOffset);
            }

            private Point CreateEndPoint(List<TerrainPoint> pts, double baseY, double length, TerrainConfig config)
            {
                pts.Add(new TerrainPoint(
                    x: length,
                    yTop: baseY - config.TopOffset,
                    yMiddle: baseY,
                    yBottom: baseY + config.BottomOffset,
                    isSafeZone: true
                ));
                return new Point(length + FinishPointXOffset, baseY + FinishPointYOffset);
            }

            private void GenerateSafeZone(List<TerrainPoint> pts, double baseY, TerrainConfig config, double startX, double length)
            {
                int count = Max(MinSafeZonePointCount, (int)(length / config.Length * _pointCount * 0.5));
                for (int i = 1; i <= count; i++)
                {
                    pts.Add(new TerrainPoint(
                        x: startX + i * length / count,
                        yTop: baseY - config.TopOffset,
                        yMiddle: baseY,
                        yBottom: baseY + config.BottomOffset,
                        isSafeZone: true
                    ));
                }
            }

            private void GenerateTerrainPoints(List<TerrainPoint> pts, TerrainConfig config, double baseY, double length, double safeStart, double safeEnd) =>
#if DEBUG
                Log("LevelGenerator", "Generating main terrain", () =>
                {
#endif
                    double midLen = length - safeStart - safeEnd;
                    double lastY = baseY;
                    int count = _pointCount - pts.Count - PointCountReductionFactor;
                    for (int i = 1; i <= count; i++)
                    {
                        double progress = (double)i / count;
                        double x = safeStart + progress * midLen;
                        double y = CalculateTerrainHeight(baseY, progress, lastY);
                        pts.Add(new TerrainPoint(
                            x: x,
                            yTop: y - config.TopOffset,
                            yMiddle: y,
                            yBottom: y + config.BottomOffset,
                            isSafeZone: false
                        ));
                        lastY = y;
                    }
#if DEBUG
                });
#endif

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

                segmentProgress = (segmentProgress + (_rand.NextDouble() * 0.3)) % 1.0;

                var (type, parameters) = _segments[segmentIndex];
                return baseY + ApplyVariation(type, parameters, segmentProgress);
            }

            private double ApplyVariation(TerrainSegmentType type, object? parameters, double progress)
            {
                switch (type)
                {
                    case TerrainSegmentType.Plateau:
                        return 0;
                    case TerrainSegmentType.Spike:
                        if (parameters == null) throw new InvalidOperationException("Parameters for Spike cannot be null");
                        var spikeParams = (dynamic)parameters;
                        return spikeParams.Height * Math.Exp(-GaussianFactor * Math.Pow(progress - spikeParams.Pos, 2));
                    case TerrainSegmentType.Depression:
                        if (parameters == null) throw new InvalidOperationException("Parameters for Depression cannot be null");
                        var depParams = (dynamic)parameters;
                        return -depParams.Depth * Math.Exp(-GaussianFactor * Math.Pow(progress - depParams.Pos, 2));
                    case TerrainSegmentType.StepUp:
                        if (parameters == null) throw new InvalidOperationException("Parameters for StepUp cannot be null");
                        var stepUpParams = (dynamic)parameters;
                        return stepUpParams.Height * Math.Min(1.0, progress * 2);
                    case TerrainSegmentType.StepDown:
                        if (parameters == null) throw new InvalidOperationException("Parameters for StepDown cannot be null");
                        var stepDownParams = (dynamic)parameters;
                        return -stepDownParams.Depth * Math.Min(1.0, progress * 2);
                    case TerrainSegmentType.Wave:
                        if (parameters == null) throw new InvalidOperationException("Parameters for Wave cannot be null");
                        var waveParams = (dynamic)parameters;
                        return waveParams.Amplitude * Math.Sin((waveParams.Frequency * progress + _rand.NextDouble()) * Math.PI);
                    case TerrainSegmentType.Jump:
                        if (parameters == null) throw new InvalidOperationException("Parameters for Jump cannot be null");
                        var jumpParams = (dynamic)parameters;
                        return progress > jumpParams.JumpPos ? jumpParams.Height : 0;
                    default:
                        throw new ArgumentException("Неизвестный тип сегмента");
                }
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

    #region Segment Selection

    public interface ISegmentSelection
    {
        TerrainSegmentType SelectSegment(
            List<TerrainSegmentType> previousSegments,
            int currentIndex,
            int terrainTypeCount,
            Random rand,
            int difficulty);
    }

    public class SegmentSelection : ISegmentSelection
    {
        public TerrainSegmentType SelectSegment(
            List<TerrainSegmentType> previousSegments,
            int currentIndex,
            int terrainTypeCount,
            Random rand,
            int difficulty)
        {
            TerrainSegmentType nextType = (TerrainSegmentType)rand.Next(terrainTypeCount);

            if (currentIndex > 0 && nextType == previousSegments[currentIndex - 1])
            {
                int offset = rand.Next(1, terrainTypeCount);
                nextType = (TerrainSegmentType)(((int)nextType + offset) % terrainTypeCount);
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