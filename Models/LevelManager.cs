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

    public record TerrainConfig(float Length, float TopOffset, float BottomOffset, int Difficulty);

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
        // Basic terrain configuration
        public const float DefaultGroundY = 500.0f;         // Default Y position for flat ground
        public const float FinishReachDistance = 50.0f;     // Distance to consider finish reached
        public const float DeltaX = 0.1f;                   // X increment for slope calculations

        // Terrain size parameters
        public const float BaseTerrainLength = 3000.0f;     // Base length for all terrain
        public const float TerrainLengthIncreasePerLevel = 500.0f;  // Length increase per difficulty level
        public const float MaxTerrainLength = 10000.0f;     // Maximum terrain length
        public const float FixedSafeZoneLength = 300.0f;    // Length of safe zones

        // Seed generation
        public const int DefaultSeedMultiplier = 100;       // Multiplier for seed generation
    }

    public class SpatialTerrainIndex
    {
        private readonly Dictionary<int, int> _bucketToSegmentMap = [];
        private readonly float _bucketSize;
        private readonly int _bucketCount;
        private readonly float _terrainLength;

        public SpatialTerrainIndex(List<Level.TerrainPoint> points, float terrainLength, int bucketCount = 50)
        {
            (_terrainLength, _bucketCount) = (terrainLength, bucketCount);
            _bucketSize = terrainLength / bucketCount;

            for (int i = 0; i < points.Count - 1; i++)
            {
                int startBucket = GetBucketIndex(points[i].X),
                    endBucket = GetBucketIndex(points[i + 1].X);

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

        public TerrainCache(int capacity = 100, float precision = 0.5f) =>
            (_capacity, _precision) = (capacity, precision);

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

            _terrainCache = new(100, 0.5f);
            _spatialIndex = new(TerrainPoints, Length);
        }

        public Level()
        {
            _visualProperties = LevelVisualProperties.FromTheme();
            _terrainCache = new(100, 0.5f);
            _spatialIndex = new([], 0);
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

        private float GetOutOfBoundsValue(float x) =>
            TerrainPoints.Count == 0
                ? DefaultGroundY
                : x < TerrainPoints[0].X
                    ? TerrainPoints[0].YMiddle
                    : TerrainPoints[^1].YMiddle;

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

}