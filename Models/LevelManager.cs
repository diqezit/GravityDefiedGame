using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Content;
using System.Linq;
using System.Threading.Tasks;
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
        public const float
            DefaultGroundY = 500.0f,
            FinishReachDistance = 50.0f,
            DeltaX = 0.1f,
            InterpolationFactor = 1.0f;

        public const int DefaultSeedMultiplier = 100;
        public const float
            BaseTerrainLength = 3000.0f,
            TerrainLengthIncreasePerLevel = 500.0f,
            MaxTerrainLength = 10000.0f;

        public const float FixedSafeZoneLength = 300.0f;
    }

    public static class GeneratorConstants
    {
        public const float DefaultTerrainHeight = 500.0f;

        public const float
            BaseSpikeHeight = 50.0f,
            SpikeHeightIncreasePerLevel = 10.0f,
            MaxSpikeHeight = 200.0f,
            SpikeHeightFactor = 0.5f,
            SpikeMidFactor = 0.7f;

        public const float
            BaseDepressionDepth = 30.0f,
            DepressionDepthIncreasePerLevel = 7.5f,
            MaxDepressionDepth = 150.0f;

        public const float
            SmoothingFactor = 0.7f,
            BasePreviousPointWeight = 0.3f,
            PreviousPointWeightDecreasePerLevel = 0.02f,
            MinPreviousPointWeight = 0.1f,
            GaussianFactor = 2.0f;

        public const float
            StartZonePercent = 0.1f,
            EndZonePercent = 0.9f;

        public const float
            PointOffset = 50.0f,
            StartPointXOffset = 100.0f,
            StartPointYOffset = -80.0f,
            FinishPointXOffset = -100.0f,
            FinishPointYOffset = -50.0f;

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

        public const float
            SegmentDifficultyThreshold = 5,
            SegmentDifficultyChance = 0.2f,
            SegmentDifficultyDivisor = 10.0f,
            FinalPlateauChance = 0.7f;

        public const float
            NoiseThreshold = 3,
            NoiseAmplitudeFactor = 5.0f,
            MaxNoiseAmplitude = 25.0f;
    }

    #endregion Constants

    #region Types

    public record TerrainConfig(
        float Length,
        float TopOffset,
        float BottomOffset,
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
            public float X { get; }
            public float YTop { get; }
            public float YMiddle { get; }
            public float YBottom { get; }
            public bool IsSafeZone { get; }

            public TerrainPoint(float x, float yTop, float yMiddle, float yBottom, bool isSafeZone)
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
        public Vector2 StartPoint { get; private set; }
        public Vector2 FinishPoint { get; private set; }
        public float Length { get; private set; }
        public LevelTheme Theme { get; private set; }
        public float SafeZoneStartLength { get; private set; }
        public float SafeZoneEndLength { get; private set; }
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

        public float GetGroundYAtX(float x)
        {
            if (IsOutOfBounds(x)) return GetOutOfBoundsValue(x);
            UpdateSegmentIndex(x);
            return InterpolateGroundY(x);
        }

        public bool IsInSafeZone(float x) =>
            !IsOutOfBounds(x) && (x <= SafeZoneStartLength || x >= Length - SafeZoneEndLength);

        public float CalculateSlopeAngle(float x) =>
            (float)Atan2(GetGroundYAtX(x + DeltaX) - GetGroundYAtX(x - DeltaX), 2 * DeltaX);

        public bool IsFinishReached(Vector2 pos) =>
#if DEBUG
            Log("Level", "Checking finish", () =>
            {
#endif
                float distance = CalculateDistance(pos, FinishPoint);
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
                float terrainLength = (float)Min(BaseTerrainLength + TerrainLengthIncreasePerLevel * Difficulty, MaxTerrainLength);
                var config = new TerrainConfig(terrainLength, PointOffset, PointOffset, Difficulty);
                (TerrainPoints, Length, StartPoint, FinishPoint, SafeZoneStartLength, SafeZoneEndLength) = gen.GenerateTerrain(config);
#if DEBUG
            });
#endif
        }

        private bool IsOutOfBounds(float x) =>
            TerrainPoints.Count < 2 || x < TerrainPoints[0].X || x > TerrainPoints[TerrainPoints.Count - 1].X;

        private float GetOutOfBoundsValue(float x)
        {
#if DEBUG
            return Log("Level", "Getting OOB value", () =>
            {
                if (x < TerrainPoints[0].X || x > TerrainPoints[TerrainPoints.Count - 1].X)
                {
                    Warning("Level", $"Out of bounds X={x:F1}");
                    return float.MaxValue;
                }
                return DefaultGroundY;
            }, DefaultGroundY);
#else
            return DefaultGroundY;
#endif
        }

        private void UpdateSegmentIndex(float x)
        {
            if (_currentSegmentIndex >= 0 && _currentSegmentIndex < TerrainPoints.Count - 1)
            {
                var p1 = TerrainPoints[_currentSegmentIndex];
                var p2 = TerrainPoints[_currentSegmentIndex + 1];
                if (x >= p1.X && x < p2.X)
                    return;
            }

            int[] segmentsToCheck = new[] { _currentSegmentIndex + 1, _currentSegmentIndex - 1 };

            foreach (int idx in segmentsToCheck)
            {
                if (idx >= 0 && idx < TerrainPoints.Count - 1)
                {
                    var p1 = TerrainPoints[idx];
                    var p2 = TerrainPoints[idx + 1];
                    if (x >= p1.X && x < p2.X)
                    {
                        _currentSegmentIndex = idx;
                        return;
                    }
                }
            }

            _currentSegmentIndex = FindTerrainSegmentIndex(x);
        }

        private float InterpolateGroundY(float x)
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

        private int FindTerrainSegmentIndex(float x)
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

        private float LinearInterpolate(float x, float x1, float y1, float x2, float y2) =>
            y1 + (y2 - y1) * ((x - x1) / (x2 - x1));

        private float CalculateDistance(Vector2 p1, Vector2 p2) =>
            Vector2.Distance(p1, p2);

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
            private readonly float _spikeHeight;
            private readonly float _depressionDepth;
            private readonly float _previousPointWeight;
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

                _spikeHeight = (float)Min(BaseSpikeHeight + SpikeHeightIncreasePerLevel * _difficulty, MaxSpikeHeight);
                _depressionDepth = (float)Min(BaseDepressionDepth + DepressionDepthIncreasePerLevel * _difficulty, MaxDepressionDepth);
                _previousPointWeight = (float)Max(BasePreviousPointWeight - PreviousPointWeightDecreasePerLevel * _difficulty, MinPreviousPointWeight);

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
                        return new { Height = (float)(_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _spikeHeight, Pos = (float)_rand.NextDouble() };
                    case TerrainSegmentType.Depression:
                        return new { Depth = (float)(_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _depressionDepth, Pos = (float)_rand.NextDouble() };
                    case TerrainSegmentType.StepUp:
                        return new { Height = (float)_rand.NextDouble() * _spikeHeight * SpikeMidFactor };
                    case TerrainSegmentType.StepDown:
                        return new { Depth = (float)_rand.NextDouble() * _depressionDepth * SpikeMidFactor };
                    case TerrainSegmentType.Wave:
                        return new { Amplitude = (float)_rand.NextDouble() * _spikeHeight * 0.5f, Frequency = 2 + (float)_rand.NextDouble() * 2 };
                    case TerrainSegmentType.Jump:
                        return new { Height = (float)_rand.NextDouble() * _spikeHeight, JumpPos = 0.3f + (float)_rand.NextDouble() * 0.4f };
                    default:
                        throw new ArgumentException("Неизвестный тип сегмента");
                }
            }

            #endregion Constructor

            #region Public Methods

            public (List<TerrainPoint> Points, float Length, Vector2 StartPoint, Vector2 FinishPoint,
                    float SafeZoneStart, float SafeZoneEnd) GenerateTerrain(TerrainConfig config) =>
#if DEBUG
                Log("LevelGenerator", "Generating terrain", () =>
                {
#endif
                    float length = config.Length;
                    var pts = new List<TerrainPoint>();
                    float baseY = DefaultTerrainHeight;

                    float safeZoneLength = FixedSafeZoneLength;

                    Vector2 sp = CreateStartPoint(pts, baseY, config);
                    GenerateSafeZone(pts, baseY, config, 0, safeZoneLength);
                    GenerateTerrainPoints(pts, config, baseY, length, safeZoneLength, safeZoneLength);
                    GenerateSafeZone(pts, baseY, config, length - safeZoneLength, safeZoneLength);
                    Vector2 fp = CreateEndPoint(pts, baseY, length, config);

#if DEBUG
                    Info("LevelGenerator", $"Generated {pts.Count} terrain points with difficulty {_difficulty}");
#endif
                    return (pts, length, sp, fp, safeZoneLength, safeZoneLength);
#if DEBUG
                }, (new List<TerrainPoint>(), 0, new Vector2(), new Vector2(), 0, 0));
#endif

            #endregion Public Methods

            #region Private Methods

            private Vector2 CreateStartPoint(List<TerrainPoint> pts, float baseY, TerrainConfig config)
            {
                pts.Add(new TerrainPoint(
                    x: 0,
                    yTop: baseY - config.TopOffset,
                    yMiddle: baseY,
                    yBottom: baseY + config.BottomOffset,
                    isSafeZone: true
                ));
                return new Vector2(StartPointXOffset, baseY + StartPointYOffset);
            }

            private Vector2 CreateEndPoint(List<TerrainPoint> pts, float baseY, float length, TerrainConfig config)
            {
                pts.Add(new TerrainPoint(
                    x: length,
                    yTop: baseY - config.TopOffset,
                    yMiddle: baseY,
                    yBottom: baseY + config.BottomOffset,
                    isSafeZone: true
                ));
                return new Vector2(length + FinishPointXOffset, baseY + FinishPointYOffset);
            }

            private void GenerateSafeZone(List<TerrainPoint> pts, float baseY, TerrainConfig config, float startX, float length)
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

            private void GenerateTerrainPoints(List<TerrainPoint> pts, TerrainConfig config, float baseY, float length, float safeStart, float safeEnd) =>
#if DEBUG
                Log("LevelGenerator", "Generating main terrain", () =>
                {
#endif
                    float midLen = length - safeStart - safeEnd;
                    float lastY = baseY;
                    int count = _pointCount - pts.Count - PointCountReductionFactor;
                    for (int i = 1; i <= count; i++)
                    {
                        float progress = (float)i / count;
                        float x = safeStart + progress * midLen;
                        float y = CalculateTerrainHeight(baseY, progress, lastY);
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

            private float CalculateTerrainHeight(float baseY, float progress, float lastY)
            {
                float raw = GetRawTerrainHeight(baseY, progress);
                float trans = ApplyTransitions(raw, baseY, progress);
                float smoothValue = lastY * _previousPointWeight + trans * SmoothingFactor;
                smoothValue = ApplyNoise(smoothValue);
                return smoothValue;
            }

            private float ApplyNoise(float value)
            {
                if (_difficulty > NoiseThreshold)
                {
                    float noiseAmplitude = (float)Min(NoiseAmplitudeFactor * (_difficulty - NoiseThreshold), MaxNoiseAmplitude);
                    value += (float)(_rand.NextDouble() * 2 - 1) * noiseAmplitude;
                }
                return value;
            }

            private float GetRawTerrainHeight(float baseY, float progress)
            {
                int segmentIndex = (int)Min((int)(progress * _segmentCount), _segmentCount - 1);

                float segmentProgress = progress * _segmentCount - segmentIndex;

                segmentProgress = (segmentProgress + (float)(_rand.NextDouble() * 0.3)) % 1.0f;

                var (type, parameters) = _segments[segmentIndex];
                return baseY + ApplyVariation(type, parameters, segmentProgress);
            }

            private float ApplyVariation(TerrainSegmentType type, object? parameters, float progress)
            {
                switch (type)
                {
                    case TerrainSegmentType.Plateau:
                        return 0;
                    case TerrainSegmentType.Spike:
                        if (parameters == null) throw new InvalidOperationException("Parameters for Spike cannot be null");
                        var spikeParams = (dynamic)parameters;
                        return spikeParams.Height * (float)Math.Exp(-GaussianFactor * Math.Pow(progress - spikeParams.Pos, 2));
                    case TerrainSegmentType.Depression:
                        if (parameters == null) throw new InvalidOperationException("Parameters for Depression cannot be null");
                        var depParams = (dynamic)parameters;
                        return -depParams.Depth * (float)Math.Exp(-GaussianFactor * Math.Pow(progress - depParams.Pos, 2));
                    case TerrainSegmentType.StepUp:
                        if (parameters == null) throw new InvalidOperationException("Parameters for StepUp cannot be null");
                        var stepUpParams = (dynamic)parameters;
                        return stepUpParams.Height * (float)Math.Min(1.0, progress * 2);
                    case TerrainSegmentType.StepDown:
                        if (parameters == null) throw new InvalidOperationException("Parameters for StepDown cannot be null");
                        var stepDownParams = (dynamic)parameters;
                        return -stepDownParams.Depth * (float)Math.Min(1.0, progress * 2);
                    case TerrainSegmentType.Wave:
                        if (parameters == null) throw new InvalidOperationException("Parameters for Wave cannot be null");
                        var waveParams = (dynamic)parameters;
                        return waveParams.Amplitude * (float)Math.Sin((waveParams.Frequency * progress + _rand.NextDouble()) * Math.PI);
                    case TerrainSegmentType.Jump:
                        if (parameters == null) throw new InvalidOperationException("Parameters for Jump cannot be null");
                        var jumpParams = (dynamic)parameters;
                        return progress > jumpParams.JumpPos ? jumpParams.Height : 0;
                    default:
                        throw new ArgumentException("Неизвестный тип сегмента");
                }
            }

            private float ApplyTransitions(float y, float baseY, float progress)
            {
                if (progress < StartZonePercent)
                    return baseY + (y - baseY) * (progress / StartZonePercent);
                if (progress > EndZonePercent)
                {
                    float normalizedProgress = (progress - EndZonePercent) / (1 - EndZonePercent);
                    return y + (baseY - y) * normalizedProgress;
                }
                return y;
            }

            #endregion Private Methods
        }

        #endregion LevelGenerator

        #region Level Manager Functionality

        private static List<Level> _allLevels = new List<Level>();
        private static Level _currentLevel;
        public static event EventHandler<LevelEventArgs> LevelChanged;

        public static void InitializeLevels(int count = 25)
        {
            _allLevels.Clear();
            var random = new Random();

            for (int i = 1; i <= count; i++)
            {
                int seed = random.Next();
                _allLevels.Add(new Level(i, $"Level {i}", seed));
            }

            Info("LevelManager", $"Initialized {_allLevels.Count} levels");
        }

        public static bool LoadLevel(int levelId)
        {
            var level = _allLevels.FirstOrDefault(l => l.Id == levelId);
            if (level == null)
            {
                Warning("LevelManager", $"Level {levelId} not found");
                return false;
            }

            _currentLevel = level;
            LevelChanged?.Invoke(null, new LevelEventArgs(LevelEventType.LevelLoaded, level));

            Info("LevelManager", $"Loaded level {levelId}");
            return true;
        }

        public static Level GetCurrentLevel()
        {
            return _currentLevel;
        }

        public static List<Level> GetAllLevels()
        {
            return _allLevels;
        }

        public static bool ReloadCurrentLevel()
        {
            if (_currentLevel == null)
                return false;

            return LoadLevel(_currentLevel.Id);
        }

        public static bool LoadNextLevel()
        {
            if (_currentLevel == null)
                return false;

            int nextId = _currentLevel.Id + 1;
            if (nextId > _allLevels.Count)
                return false;

            return LoadLevel(nextId);
        }

        public static bool LoadPreviousLevel()
        {
            if (_currentLevel == null)
                return false;

            int prevId = _currentLevel.Id - 1;
            if (prevId < 1)
                return false;

            return LoadLevel(prevId);
        }

        #endregion
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

    #region Level Events

    public enum LevelEventType
    {
        LevelLoaded,
        LevelCompleted,
        LevelRestarted,
        LevelUnlocked
    }

    public class LevelEventArgs : EventArgs
    {
        public LevelEventType Type { get; }
        public Level Level { get; }

        public LevelEventArgs(LevelEventType type, Level level)
        {
            Type = type;
            Level = level;
        }
    }

    #endregion
}