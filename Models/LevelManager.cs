#nullable enable
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Microsoft.Xna.Framework;
using static System.Math;
using static GravityDefiedGame.Models.LevelConstants;
using static GravityDefiedGame.Models.GeneratorConstants;
using GravityDefiedGame.Views;

namespace GravityDefiedGame.Models
{
    public interface ILevelPhysics
    {
        int Id { get; }
        string Name { get; }
        int Difficulty { get; }
        List<Level.TerrainPoint> TerrainPoints { get; }
        Vector2 StartPoint { get; }
        Vector2 FinishPoint { get; }
        float Length { get; }
        LevelTheme Theme { get; }
        float SafeZoneStartLength { get; }
        float SafeZoneEndLength { get; }
        float GetGroundYAtX(float x);
        bool IsInSafeZone(float x);
        float CalculateSlopeAngle(float x);
        bool IsFinishReached(Vector2 pos);
    }

    public interface ILevelVisualData
    {
        Color VerticalLineColor { get; }
        Color BackgroundColor { get; }
        Color TerrainColor { get; }
        Color SafeZoneColor { get; }
        List<Level.TerrainPoint> TerrainPoints { get; }
        Vector2 StartPoint { get; }
        Vector2 FinishPoint { get; }
        float Length { get; }
        LevelTheme Theme { get; }
    }

    public record struct LevelVisualProperties(
        Color VerticalLineColor,
        Color BackgroundColor,
        Color TerrainColor,
        Color SafeZoneColor)
    {
        public static LevelVisualProperties FromTheme() => new(
            ThemeManager.CurrentTheme.VerticalLineColor,
            ThemeManager.CurrentTheme.BackgroundColor,
            ThemeManager.CurrentTheme.TerrainColor,
            ThemeManager.CurrentTheme.SafeZoneColor
        );
    }

    public enum LevelTheme { Desert, Mountain, Arctic, Volcano }

    public enum TerrainSegmentType
    {
        Plateau,
        Spike,
        Depression,
        StepUp,
        StepDown,
        Wave,
        Jump,
        SmoothHill,
        Valley,
        Ridges,
        Rugged,
        SlantedPlateau
    }

    public record TerrainConfig(float Length, float TopOffset, float BottomOffset, int Difficulty);

    // Parameter classes for terrain segments
    public abstract record TerrainParameters;
    public record SpikeParameters(float Height, float Pos) : TerrainParameters;
    public record DepressionParameters(float Depth, float Pos) : TerrainParameters;
    public record StepUpParameters(float Height) : TerrainParameters;
    public record StepDownParameters(float Depth) : TerrainParameters;
    public record WaveParameters(float Amplitude, float Frequency) : TerrainParameters;
    public record JumpParameters(float Height, float JumpPos) : TerrainParameters;
    public record SmoothHillParameters(float Height, float Width) : TerrainParameters;
    public record ValleyParameters(float Depth, float Width) : TerrainParameters;
    public record RidgesParameters(float Height, int Count, float Sharpness) : TerrainParameters;
    public record RuggedParameters(float BaseHeight, float NoiseAmplitude, float NoiseFrequency) : TerrainParameters;
    public record SlantedPlateauParameters(float StartHeight, float EndHeight, float PlateauStart, float PlateauEnd) : TerrainParameters;

    public static class LevelConstants
    {
        // Basic terrain configuration values
        public const float DefaultGroundY = 500.0f;          // Default Y position for flat ground
        public const float FinishReachDistance = 50.0f;      // Distance to consider finish reached
        public const float DeltaX = 0.1f;                    // X increment for slope calculations

        // Terrain size parameters
        public const float BaseTerrainLength = 3000.0f;      // Base length for all terrain
        public const float TerrainLengthIncreasePerLevel = 500.0f; // Length increase per difficulty level
        public const float MaxTerrainLength = 10000.0f;      // Maximum terrain length
        public const float FixedSafeZoneLength = 300.0f;     // Length of safe zones

        // Seed generation
        public const int DefaultSeedMultiplier = 100;        // Multiplier for seed generation
    }

    public static class GeneratorConstants
    {
        // Base terrain parameters
        public const float DefaultTerrainHeight = 500.0f;    // Default height of terrain

        // Spike parameters
        public const float BaseSpikeHeight = 50.0f;          // Initial spike height
        public const float SpikeHeightIncreasePerLevel = 10.0f; // Spike height increase per level
        public const float MaxSpikeHeight = 200.0f;          // Maximum spike height
        public const float SpikeHeightFactor = 0.5f;         // Multiplier for spike height
        public const float SpikeMidFactor = 0.7f;            // Mid-point factor for spikes

        // Depression parameters
        public const float BaseDepressionDepth = 30.0f;      // Initial depression depth
        public const float DepressionDepthIncreasePerLevel = 7.5f; // Depth increase per level
        public const float MaxDepressionDepth = 150.0f;      // Maximum depression depth

        // Smoothing parameters
        public const float SmoothingFactor = 0.7f;           // Base smoothing factor
        public const float BasePreviousPointWeight = 0.3f;   // Initial weight for previous points
        public const float PreviousPointWeightDecreasePerLevel = 0.02f; // Weight decrease per level
        public const float MinPreviousPointWeight = 0.1f;    // Minimum weight for previous points
        public const float GaussianFactor = 2.0f;            // Factor for Gaussian functions

        // Zone configuration
        public const float StartZonePercent = 0.1f;          // Percentage for start zone
        public const float EndZonePercent = 0.9f;            // Percentage for end zone
        public const float PointOffset = 50.0f;              // Offset for terrain points

        // Start and finish point offsets
        public const float StartPointXOffset = 100.0f;       // X offset for start point
        public const float StartPointYOffset = -80.0f;       // Y offset for start point
        public const float FinishPointXOffset = -100.0f;     // X offset for finish point
        public const float FinishPointYOffset = -50.0f;      // Y offset for finish point

        // Segment difficulty parameters
        public const float SegmentDifficultyThreshold = 5;   // Threshold for difficult segments
        public const float SegmentDifficultyChance = 0.2f;   // Chance for difficult segments
        public const float SegmentDifficultyDivisor = 10.0f; // Divisor for difficulty calculation
        public const float FinalPlateauChance = 0.7f;        // Chance for final plateau

        // Noise parameters
        public const int NoiseThreshold = 3;                 // Difficulty threshold for noise
        public const float NoiseAmplitudeFactor = 5.0f;      // Amplitude factor for noise
        public const float MaxNoiseAmplitude = 25.0f;        // Maximum noise amplitude

        // Point count parameters
        public const int BasePointCount = 60;                // Initial point count
        public const int PointCountIncreasePerLevel = 5;     // Point count increase per level
        public const int MaxPointCount = 120;                // Maximum point count

        // Segment count parameters
        public const int BaseSegmentCount = 5;               // Initial segment count
        public const int SegmentCountIncreasePerLevel = 1;   // Segment count increase per level
        public const int MaxSegmentCount = 15;               // Maximum segment count
        public const int MinSafeZonePointCount = 3;          // Minimum points in safe zone
        public const int PointCountReductionFactor = 5;      // Factor for point count reduction

        // Terrain type parameters
        public const int MinTerrainTypeCount = 3;            // Minimum terrain type count
        public const int MaxTerrainTypeCount = 7;            // Maximum terrain type count
        public const int TerrainTypeUnlockLevel = 5;         // Level to unlock new terrain types
    }

    public class SpatialTerrainIndex
    {
        private readonly Dictionary<int, int> _bucketToSegmentMap = [];
        private readonly float _bucketSize;
        private readonly int _bucketCount;
        private readonly float _terrainLength;

        public SpatialTerrainIndex(List<Level.TerrainPoint> points, float terrainLength, int bucketCount = 50)
        {
            _terrainLength = terrainLength;
            _bucketCount = bucketCount;
            _bucketSize = terrainLength / bucketCount;

            for (int i = 0; i < points.Count - 1; i++)
            {
                int startBucket = GetBucketIndex(points[i].X);
                int endBucket = GetBucketIndex(points[i + 1].X);

                for (int bucket = startBucket; bucket <= endBucket; bucket++)
                {
                    if (!_bucketToSegmentMap.ContainsKey(bucket))
                        _bucketToSegmentMap[bucket] = i;
                }
            }
        }

        public int GetSegmentIndex(float x) =>
            _bucketToSegmentMap.TryGetValue(GetBucketIndex(x), out int segmentIndex)
                ? segmentIndex
                : GetFallbackSegmentIndex(x);

        private int GetBucketIndex(float x) =>
            Clamp((int)(x / _bucketSize), 0, _bucketCount - 1);

        private int GetFallbackSegmentIndex(float x)
        {
            if (x <= 0) return 0;
            if (x >= _terrainLength) return _bucketToSegmentMap.Values.Max();

            int bucket = GetBucketIndex(x);
            int closestBucket = _bucketToSegmentMap.Keys
                .MinBy(k => Abs(k - bucket));

            return _bucketToSegmentMap[closestBucket];
        }
    }

    public class TerrainCache
    {
        private readonly record struct CacheEntry(float X, float Y, long LastAccessed);

        private readonly Dictionary<int, CacheEntry> _cache = [];
        private readonly int _capacity;
        private readonly float _precision;
        private long _accessCounter;

        public TerrainCache(int capacity = 100, float precision = 0.5f)
        {
            _capacity = capacity;
            _precision = precision;
        }

        public bool TryGetValue(float x, out float y)
        {
            int key = GetKey(x);

            if (_cache.TryGetValue(key, out var entry))
            {
                y = entry.Y;
                Interlocked.Increment(ref _accessCounter);
                return true;
            }

            y = 0;
            return false;
        }

        public void Add(float x, float y)
        {
            if (_cache.Count >= _capacity)
            {
                var leastUsed = _cache.MinBy(pair => pair.Value.LastAccessed);
                _cache.Remove(leastUsed.Key);
            }

            _cache[GetKey(x)] = new CacheEntry(x, y, Interlocked.Increment(ref _accessCounter));
        }

        private int GetKey(float x) => (int)Round(x / _precision);
    }

    public class Level : PhysicsComponent, ILevelPhysics, ILevelVisualData
    {
        public readonly record struct TerrainPoint(
            float X,
            float YTop,
            float YMiddle,
            float YBottom,
            bool IsSafeZone) : IComparable<TerrainPoint>, IComparable<float>
        {
            public int CompareTo(TerrainPoint other) => X.CompareTo(other.X);
            public int CompareTo(float x) => X.CompareTo(x);
        }

        public int Id { get; internal set; }
        public string Name { get; internal set; } = string.Empty;
        public int Difficulty { get; internal set; }
        public List<TerrainPoint> TerrainPoints { get; private set; } = [];
        public Vector2 StartPoint { get; private set; }
        public Vector2 FinishPoint { get; private set; }
        public float Length { get; private set; }
        public LevelTheme Theme { get; private set; }
        public float SafeZoneStartLength { get; private set; }
        public float SafeZoneEndLength { get; private set; }

        private LevelVisualProperties _visualProperties;
        private int _currentSegmentIndex;
        private float _lastQueryX = float.NaN;
        private float _lastGroundY = DefaultGroundY;
        private readonly SpatialTerrainIndex _spatialIndex;
        private readonly TerrainCache _terrainCache;

        public Color VerticalLineColor => _visualProperties.VerticalLineColor;
        public Color BackgroundColor => _visualProperties.BackgroundColor;
        public Color TerrainColor => _visualProperties.TerrainColor;
        public Color SafeZoneColor => _visualProperties.SafeZoneColor;

        public Level(int id, string name, int? seed = null, int? difficulty = null)
        {
            Id = id;
            Name = name;
            Difficulty = difficulty ?? id;
            Theme = (LevelTheme)(id % Enum.GetValues(typeof(LevelTheme)).Length);
            _visualProperties = LevelVisualProperties.FromTheme();

            int levelSeed = seed ?? (id * DefaultSeedMultiplier + DateTime.Now.Millisecond);
            GenerateLevel(levelSeed);

            _terrainCache = new TerrainCache(100, 0.5f);
            _spatialIndex = new SpatialTerrainIndex(TerrainPoints, Length);
        }

        public Level()
        {
            _visualProperties = LevelVisualProperties.FromTheme();
            _terrainCache = new TerrainCache(100, 0.5f);
            _spatialIndex = new SpatialTerrainIndex([], 0);
        }

        public float GetGroundYAtX(float x)
        {
            if (_terrainCache.TryGetValue(x, out float cachedY))
                return cachedY;

            if (!float.IsNaN(_lastQueryX) && Abs(_lastQueryX - x) < float.Epsilon)
                return _lastGroundY;

            if (IsOutOfBounds(x))
                return _lastGroundY = GetOutOfBoundsValue(x);

            _currentSegmentIndex = _spatialIndex.GetSegmentIndex(x);
            if (!IsInSegment(_currentSegmentIndex, x))
                _currentSegmentIndex = FindTerrainSegmentIndexImproved(x);

            _lastQueryX = x;
            _lastGroundY = InterpolateGroundY(x);
            _terrainCache.Add(x, _lastGroundY);
            return _lastGroundY;
        }

        public bool IsInSafeZone(float x) =>
                !IsOutOfBounds(x) && (x <= SafeZoneStartLength || x >= Length - SafeZoneEndLength);

        public float CalculateSlopeAngle(float x) =>
            (float)Atan2(GetGroundYAtX(x + DeltaX) - GetGroundYAtX(x - DeltaX), 2 * DeltaX);

        public bool IsFinishReached(Vector2 pos) =>
            Vector2.Distance(pos, FinishPoint) <= FinishReachDistance;

        public void UpdateVisualProperties(LevelVisualProperties visualProps) =>
            _visualProperties = visualProps;

        private void GenerateLevel(int seed)
        {
            var gen = new LevelGenerator(seed, Difficulty);
            float terrainLength = Min(
                BaseTerrainLength + TerrainLengthIncreasePerLevel * Difficulty,
                MaxTerrainLength
            );

            var config = new TerrainConfig(terrainLength, PointOffset, PointOffset, Difficulty);
            (TerrainPoints, Length, StartPoint, FinishPoint, SafeZoneStartLength, SafeZoneEndLength) =
                gen.GenerateTerrain(config);
        }

        private bool IsOutOfBounds(float x) =>
            TerrainPoints.Count < 2 || x < TerrainPoints[0].X || x > TerrainPoints[^1].X;

        private float GetOutOfBoundsValue(float x)
        {
            if (TerrainPoints.Count == 0)
                return DefaultGroundY;

            return x < TerrainPoints[0].X
                ? TerrainPoints[0].YMiddle
                : TerrainPoints[^1].YMiddle;
        }

        private bool IsInSegment(int idx, float x)
        {
            if (idx < 0 || idx >= TerrainPoints.Count - 1)
                return false;

            var (p1, p2) = (TerrainPoints[idx], TerrainPoints[idx + 1]);
            return x >= p1.X && x < p2.X;
        }

        private float InterpolateGroundY(float x)
        {
            if (_currentSegmentIndex < 0 || _currentSegmentIndex >= TerrainPoints.Count - 1)
                return DefaultGroundY;

            var (p1, p2) = (
                TerrainPoints[_currentSegmentIndex],
                TerrainPoints[_currentSegmentIndex + 1]
            );

            float t = (x - p1.X) / (p2.X - p1.X);
            return p1.YMiddle + (p2.YMiddle - p1.YMiddle) * t;
        }

        private int FindTerrainSegmentIndexImproved(float x)
        {
            if (TerrainPoints.Count < 2) return 0;
            if (x <= TerrainPoints[0].X) return 0;
            if (x >= TerrainPoints[^1].X) return TerrainPoints.Count - 2;

            // Initial position estimate based on linear distribution
            float firstX = TerrainPoints[0].X;
            float lastX = TerrainPoints[^1].X;
            int pos = (int)((x - firstX) / (lastX - firstX) * (TerrainPoints.Count - 1));
            pos = Clamp(pos, 0, TerrainPoints.Count - 2);

            // Direct hit check
            if (x >= TerrainPoints[pos].X && x < TerrainPoints[pos + 1].X)
                return pos;

            // Binary search
            int left = (x < TerrainPoints[pos].X) ? 0 : pos;
            int right = (x < TerrainPoints[pos].X) ? pos : TerrainPoints.Count - 2;

            while (left <= right)
            {
                int mid = left + (right - left) / 2;

                if (x >= TerrainPoints[mid].X && x < TerrainPoints[mid + 1].X)
                    return mid;
                else if (x < TerrainPoints[mid].X)
                    right = mid - 1;
                else
                    left = mid + 1;
            }

            return Min(Max(0, left), TerrainPoints.Count - 2);
        }
    }

    public class LevelGenerator
    {
        private readonly Random _rand;
        private readonly int _difficulty;
        private readonly float _spikeHeight;
        private readonly float _depressionDepth;
        private readonly float _previousPointWeight;
        private readonly int _segmentCount;
        private readonly int _pointCount;
        private readonly int _terrainTypeCount;
        private readonly float[] _segmentBoundaries;
        private readonly List<(TerrainSegmentType Type, TerrainParameters? Parameters)> _segments = [];

        public LevelGenerator(int seed, int difficulty)
        {
            _rand = new Random(seed);
            _difficulty = Max(1, difficulty);

            // Calculate difficulty-scaled parameters
            _spikeHeight = Min(
                BaseSpikeHeight + SpikeHeightIncreasePerLevel * _difficulty,
                MaxSpikeHeight
            );

            _depressionDepth = Min(
                BaseDepressionDepth + DepressionDepthIncreasePerLevel * _difficulty,
                MaxDepressionDepth
            );

            _previousPointWeight = Max(
                BasePreviousPointWeight - PreviousPointWeightDecreasePerLevel * _difficulty,
                MinPreviousPointWeight
            );

            // Calculate segment and point counts with variation
            int baseSegments = BaseSegmentCount + SegmentCountIncreasePerLevel * _difficulty;
            int variation = _rand.Next(-2, 3);
            _segmentCount = Max(1, Min(baseSegments + variation, MaxSegmentCount));

            _pointCount = Min(
                BasePointCount + PointCountIncreasePerLevel * _difficulty,
                MaxPointCount
            );

            _terrainTypeCount = Min(
                MinTerrainTypeCount + (_difficulty / TerrainTypeUnlockLevel),
                MaxTerrainTypeCount
            );

            // Initialize segment boundaries
            _segmentBoundaries = new float[_segmentCount + 1];
            _segmentBoundaries[0] = 0f;
            for (int i = 1; i < _segmentCount; i++)
                _segmentBoundaries[i] = (float)_rand.NextDouble();
            _segmentBoundaries[_segmentCount] = 1f;
            Array.Sort(_segmentBoundaries);

            GenerateSegmentTypes();
        }

        private void GenerateSegmentTypes()
        {
            _segments.Clear();
            _segments.Add((TerrainSegmentType.Plateau, null));

            var availableTypes = GetAvailableSegmentTypes();

            for (int i = 1; i < _segmentCount; i++)
            {
                TerrainSegmentType nextType = SelectSegment(i, availableTypes);
                _segments.Add((nextType, GenerateParameters(nextType)));
            }

            // Add final plateau with probability
            if (_segments.Count > 2 && _rand.NextDouble() < FinalPlateauChance)
                _segments[^1] = (TerrainSegmentType.Plateau, null);
        }

        private List<TerrainSegmentType> GetAvailableSegmentTypes()
        {
            var types = new List<TerrainSegmentType>
        {
            TerrainSegmentType.Plateau,
            TerrainSegmentType.Spike,
            TerrainSegmentType.Depression,
            TerrainSegmentType.StepUp,
            TerrainSegmentType.StepDown,
            TerrainSegmentType.Wave
        };

            if (_difficulty >= 3)
                types.Add(TerrainSegmentType.Jump);

            if (_difficulty >= 4)
            {
                types.Add(TerrainSegmentType.SmoothHill);
                types.Add(TerrainSegmentType.Valley);
            }

            if (_difficulty >= 6)
            {
                types.Add(TerrainSegmentType.Ridges);
                types.Add(TerrainSegmentType.SlantedPlateau);
            }

            if (_difficulty >= 8)
                types.Add(TerrainSegmentType.Rugged);

            return types;
        }

        private TerrainSegmentType SelectSegment(int currentIndex, List<TerrainSegmentType> availableTypes)
        {
            var excludedTypes = new HashSet<TerrainSegmentType>();

            // Avoid repeating the previous segment type
            if (currentIndex > 0 && _segments.Count > 0)
                excludedTypes.Add(_segments[currentIndex - 1].Type);

            // Higher difficulties have less plateaus
            if (_difficulty > SegmentDifficultyThreshold &&
                _rand.NextDouble() < SegmentDifficultyChance * (_difficulty / SegmentDifficultyDivisor))
                excludedTypes.Add(TerrainSegmentType.Plateau);

            var validTypes = availableTypes.Where(t => !excludedTypes.Contains(t)).ToList();

            return validTypes.Count > 0
                ? validTypes[_rand.Next(validTypes.Count)]
                : availableTypes[_rand.Next(availableTypes.Count)];
        }

        private TerrainParameters? GenerateParameters(TerrainSegmentType type) => type switch
        {
            TerrainSegmentType.Plateau => null,

            TerrainSegmentType.Spike => new SpikeParameters(
                Height: (float)(_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _spikeHeight,
                Pos: (float)_rand.NextDouble()
            ),

            TerrainSegmentType.Depression => new DepressionParameters(
                Depth: (float)(_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _depressionDepth,
                Pos: (float)_rand.NextDouble()
            ),

            TerrainSegmentType.StepUp => new StepUpParameters(
                Height: (float)_rand.NextDouble() * _spikeHeight * SpikeMidFactor
            ),

            TerrainSegmentType.StepDown => new StepDownParameters(
                Depth: (float)_rand.NextDouble() * _depressionDepth * SpikeMidFactor
            ),

            TerrainSegmentType.Wave => new WaveParameters(
                Amplitude: (float)_rand.NextDouble() * _spikeHeight * 0.5f,
                Frequency: 2 + (float)_rand.NextDouble() * 2
            ),

            TerrainSegmentType.Jump => new JumpParameters(
                Height: (float)_rand.NextDouble() * _spikeHeight,
                JumpPos: 0.3f + (float)_rand.NextDouble() * 0.4f
            ),

            TerrainSegmentType.SmoothHill => new SmoothHillParameters(
                Height: (float)_rand.NextDouble() * _spikeHeight * 0.8f,
                Width: 0.4f + (float)_rand.NextDouble() * 0.4f
            ),

            TerrainSegmentType.Valley => new ValleyParameters(
                Depth: (float)_rand.NextDouble() * _depressionDepth * 0.7f,
                Width: 0.5f + (float)_rand.NextDouble() * 0.3f
            ),

            TerrainSegmentType.Ridges => new RidgesParameters(
                Height: (float)_rand.NextDouble() * _spikeHeight * 0.6f,
                Count: 2 + _rand.Next(3),
                Sharpness: 0.3f + (float)_rand.NextDouble() * 0.6f
            ),

            TerrainSegmentType.Rugged => new RuggedParameters(
                BaseHeight: (float)_rand.NextDouble() * _spikeHeight * 0.3f,
                NoiseAmplitude: (float)_rand.NextDouble() * _spikeHeight * 0.2f,
                NoiseFrequency: 5 + (float)_rand.NextDouble() * 5
            ),

            TerrainSegmentType.SlantedPlateau => new SlantedPlateauParameters(
                StartHeight: (float)_rand.NextDouble() * _spikeHeight * 0.5f,
                EndHeight: (float)_rand.NextDouble() * _spikeHeight * 0.5f,
                PlateauStart: 0.2f + (float)_rand.NextDouble() * 0.2f,
                PlateauEnd: 0.6f + (float)_rand.NextDouble() * 0.2f
            ),

            _ => throw new ArgumentException($"Unknown segment type: {type}")
        };

        private float ApplyVariation(TerrainSegmentType type, TerrainParameters? parameters, float progress) => type switch
        {
            TerrainSegmentType.Plateau => 0,

            TerrainSegmentType.Spike => parameters is SpikeParameters sp
                ? sp.Height * (float)Exp(-GaussianFactor * Pow(progress - sp.Pos, 2))
                : 0,

            TerrainSegmentType.Depression => parameters is DepressionParameters dp
                ? -dp.Depth * (float)Exp(-GaussianFactor * Pow(progress - dp.Pos, 2))
                : 0,

            TerrainSegmentType.StepUp => parameters is StepUpParameters sup
                ? sup.Height * (float)Min(1.0, progress * 2)
                : 0,

            TerrainSegmentType.StepDown => parameters is StepDownParameters sdp
                ? -sdp.Depth * (float)Min(1.0, progress * 2)
                : 0,

            TerrainSegmentType.Wave => parameters is WaveParameters wp
                ? wp.Amplitude * (float)Sin((wp.Frequency * progress + _rand.NextDouble()) * PI)
                : 0,

            TerrainSegmentType.Jump => parameters is JumpParameters jp
                ? progress > jp.JumpPos ? jp.Height : 0
                : 0,

            TerrainSegmentType.SmoothHill => parameters is SmoothHillParameters shp
                ? CalculateSmoothHill(progress, shp.Height, shp.Width)
                : 0,

            TerrainSegmentType.Valley => parameters is ValleyParameters vp
                ? CalculateValley(progress, vp.Depth, vp.Width)
                : 0,

            TerrainSegmentType.Ridges => parameters is RidgesParameters rp
                ? CalculateRidges(progress, rp.Height, rp.Count, rp.Sharpness)
                : 0,

            TerrainSegmentType.Rugged => parameters is RuggedParameters rup
                ? CalculateRugged(progress, rup.BaseHeight, rup.NoiseAmplitude, rup.NoiseFrequency)
                : 0,

            TerrainSegmentType.SlantedPlateau => parameters is SlantedPlateauParameters spp
                ? CalculateSlantedPlateau(progress, spp.StartHeight, spp.EndHeight, spp.PlateauStart, spp.PlateauEnd)
                : 0,

            _ => throw new ArgumentException($"Unknown segment type: {type}")
        };

        private static float CalculateSmoothHill(float progress, float height, float width)
        {
            float centerPos = 0.5f;
            float normalizedPos = (progress - centerPos) / (width / 2);
            return Abs(normalizedPos) > 1 ? 0 : height * (1 - normalizedPos * normalizedPos);
        }

        private static float CalculateValley(float progress, float depth, float width)
        {
            float centerPos = 0.5f;
            float normalizedPos = (progress - centerPos) / (width / 2);
            return Abs(normalizedPos) > 1 ? 0 : -depth * (1 - normalizedPos * normalizedPos);
        }

        private static float CalculateRidges(float progress, float height, int count, float sharpness)
        {
            float ridgeWidth = 1.0f / count;
            float localProgress = (progress * count) % 1.0f;
            float normalizedPos = Abs(localProgress - 0.5f) * 2;
            float sharpnessFactor = (float)Pow(normalizedPos, sharpness);
            return height * (1 - sharpnessFactor);
        }

        private static float CalculateRugged(float progress, float baseHeight, float noiseAmplitude, float noiseFrequency)
        {
            float baseValue = baseHeight * (0.8f + 0.4f * progress);
            float noise = noiseAmplitude * (
                (float)Sin(noiseFrequency * progress * PI) * 0.5f +
                (float)Sin(noiseFrequency * 2.7f * progress * PI) * 0.3f +
                (float)Sin(noiseFrequency * 5.1f * progress * PI) * 0.2f
            );
            return baseValue + noise;
        }

        private static float CalculateSlantedPlateau(float progress, float startHeight, float endHeight, float plateauStart, float plateauEnd)
        {
            if (progress < plateauStart)
                return startHeight * (progress / plateauStart);
            else if (progress > plateauEnd)
            {
                float descentProgress = (progress - plateauEnd) / (1 - plateauEnd);
                return endHeight * (1 - descentProgress);
            }
            else
            {
                float plateauProgress = (progress - plateauStart) / (plateauEnd - plateauStart);
                return startHeight + (endHeight - startHeight) * plateauProgress;
            }
        }

        private static float ApplyTransitions(float y, float baseY, float progress)
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

        private float CalculateTerrainHeight(float baseY, float progress, float lastY)
        {
            float raw = GetRawTerrainHeight(baseY, progress);
            float trans = ApplyTransitions(raw, baseY, progress);

            // Adaptive smoothing based on difficulty
            float adaptiveSmoothFactor = SmoothingFactor * (1 - 0.05f * _difficulty);
            float smoothWeight = _previousPointWeight * (float)Pow(adaptiveSmoothFactor, 1 + _difficulty * 0.1);
            float smoothValue = lastY * smoothWeight + trans * (1 - smoothWeight);

            return ApplyNoise(smoothValue);
        }

        private float ApplyNoise(float value)
        {
            if (_difficulty <= NoiseThreshold)
                return value;

            float noiseAmplitude = Min(
                NoiseAmplitudeFactor * (_difficulty - NoiseThreshold),
                MaxNoiseAmplitude
            );

            // Generate coherent noise
            float coherentNoise = (float)(
                _rand.NextDouble() * 0.6 +
                _rand.NextDouble() * 0.3 +
                _rand.NextDouble() * 0.1 - 0.5
            ) * 2;

            return value + coherentNoise * noiseAmplitude;
        }

        private float GetRawTerrainHeight(float baseY, float progress)
        {
            int segmentIndex = FindSegmentIndex(progress);
            float segStart = _segmentBoundaries[segmentIndex];
            float segEnd = _segmentBoundaries[segmentIndex + 1];

            float segmentProgress = (progress - segStart) / (segEnd - segStart);
            float transitionZone = 0.15f;

            var (currentType, currentParams) = _segments[segmentIndex];
            float currentValue = ApplyVariation(currentType, currentParams, segmentProgress);

            // Handle transitions between segments
            if (segmentProgress < transitionZone && segmentIndex > 0)
            {
                var (prevType, prevParams) = _segments[segmentIndex - 1];
                float prevValue = ApplyVariation(prevType, prevParams, 1.0f);

                float t = segmentProgress / transitionZone;
                float blendFactor = (float)(1 / (1 + Exp(-12 * (t - 0.5))));

                return baseY + prevValue * (1 - blendFactor) + currentValue * blendFactor;
            }
            else if (segmentProgress > (1 - transitionZone) && segmentIndex < _segments.Count - 1)
            {
                var (nextType, nextParams) = _segments[segmentIndex + 1];
                float nextValue = ApplyVariation(nextType, nextParams, 0.0f);

                float t = (segmentProgress - (1 - transitionZone)) / transitionZone;
                float blendFactor = (float)(1 / (1 + Exp(-12 * (t - 0.5))));

                return baseY + currentValue * (1 - blendFactor) + nextValue * blendFactor;
            }

            return baseY + currentValue;
        }

        private int FindSegmentIndex(float progress)
        {
            int index = Array.BinarySearch(_segmentBoundaries, progress);
            if (index < 0)
                index = ~index - 1;

            return Max(0, Min(index, _segmentBoundaries.Length - 2));
        }

        public (List<Level.TerrainPoint> Points, float Length, Vector2 StartPoint, Vector2 FinishPoint,
               float SafeZoneStart, float SafeZoneEnd) GenerateTerrain(TerrainConfig config)
        {
            float length = config.Length;
            var pts = new List<Level.TerrainPoint>(_pointCount + 10);
            float baseY = DefaultTerrainHeight;
            float safeZoneLength = FixedSafeZoneLength;

            // Create terrain in sequence: start point, safe zone, main terrain, end safe zone, finish point
            Vector2 startPoint = CreateStartPoint(pts, baseY, config);
            GenerateSafeZone(pts, baseY, config, 0, safeZoneLength);
            GenerateTerrainPoints(pts, config, baseY, length, safeZoneLength, safeZoneLength);
            GenerateSafeZone(pts, baseY, config, length - safeZoneLength, safeZoneLength);
            Vector2 finishPoint = CreateEndPoint(pts, baseY, length, config);

            return (pts, length, startPoint, finishPoint, safeZoneLength, safeZoneLength);
        }

        private static Vector2 CreateStartPoint(List<Level.TerrainPoint> pts, float baseY, TerrainConfig config)
        {
            pts.Add(new Level.TerrainPoint(
                0, baseY - config.TopOffset, baseY, baseY + config.BottomOffset, true
            ));

            return new Vector2(StartPointXOffset, baseY + StartPointYOffset);
        }

        private static Vector2 CreateEndPoint(
            List<Level.TerrainPoint> pts, float baseY, float length, TerrainConfig config)
        {
            pts.Add(new Level.TerrainPoint(
                length, baseY - config.TopOffset, baseY, baseY + config.BottomOffset, true
            ));

            return new Vector2(length + FinishPointXOffset, baseY + FinishPointYOffset);
        }

        private void GenerateSafeZone(
            List<Level.TerrainPoint> pts, float baseY, TerrainConfig config, float startX, float length)
        {
            int count = Max(
                MinSafeZonePointCount,
                (int)(length / config.Length * _pointCount * 0.5)
            );

            float step = length / count;

            for (int i = 1; i <= count; i++)
            {
                pts.Add(new Level.TerrainPoint(
                    startX + i * step,
                    baseY - config.TopOffset,
                    baseY,
                    baseY + config.BottomOffset,
                    true
                ));
            }
        }

        private void GenerateTerrainPoints(
            List<Level.TerrainPoint> pts, TerrainConfig config, float baseY,
            float length, float safeStart, float safeEnd)
        {
            float midLen = length - safeStart - safeEnd;
            float lastY = baseY;

            int count = Max(1, _pointCount - pts.Count - PointCountReductionFactor);
            float step = midLen / count;

            for (int i = 1; i <= count; i++)
            {
                float progress = (float)i / count;
                float x = safeStart + i * step;
                float y = CalculateTerrainHeight(baseY, progress, lastY);

                pts.Add(new Level.TerrainPoint(
                    x,
                    y - config.TopOffset,
                    y,
                    y + config.BottomOffset,
                    false
                ));

                lastY = y;
            }
        }
    }
}