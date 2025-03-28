﻿#nullable enable

using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using static System.Math;

using GravityDefiedGame.Views;
using static GravityDefiedGame.Models.LevelConstants;
using static GravityDefiedGame.Models.GeneratorConstants;
using static GravityDefiedGame.Models.Level.LevelGenerator.TerrainSegmentType;

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

    public struct LevelVisualProperties
    {
        public Color VerticalLineColor;
        public Color BackgroundColor;
        public Color TerrainColor;
        public Color SafeZoneColor;

        public static LevelVisualProperties FromTheme()
        {
            return new LevelVisualProperties
            {
                VerticalLineColor = ThemeManager.CurrentTheme.VerticalLineColor,
                BackgroundColor = ThemeManager.CurrentTheme.BackgroundColor,
                TerrainColor = ThemeManager.CurrentTheme.TerrainColor,
                SafeZoneColor = ThemeManager.CurrentTheme.SafeZoneColor
            };
        }
    }

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

    public record TerrainConfig(
        float Length,
        float TopOffset,
        float BottomOffset,
        int Difficulty
    );

    public enum LevelTheme { Desert, Mountain, Arctic, Volcano }

    public class Level : PhysicsComponent, ILevelPhysics, ILevelVisualData
    {
        public readonly struct TerrainPoint : IComparable<TerrainPoint>, IComparable<float>
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

            public int CompareTo(TerrainPoint other) => X.CompareTo(other.X);

            public int CompareTo(float x) => X.CompareTo(x);
        }

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

        private LevelVisualProperties _visualProperties;

        public Color VerticalLineColor => _visualProperties.VerticalLineColor;
        public Color BackgroundColor => _visualProperties.BackgroundColor;
        public Color TerrainColor => _visualProperties.TerrainColor;
        public Color SafeZoneColor => _visualProperties.SafeZoneColor;

        private int _currentSegmentIndex = 0;
        private float _lastQueryX = float.NaN;
        private float _lastGroundY = DefaultGroundY;

        public Level(int id, string name, int? seed = null, int? difficulty = null)
        {
            Id = id;
            Name = name;
            Difficulty = difficulty ?? id;
            Theme = (LevelTheme)(id % Enum.GetValues(typeof(LevelTheme)).Length);
            _visualProperties = LevelVisualProperties.FromTheme();

            int levelSeed = seed ?? (id * DefaultSeedMultiplier + DateTime.Now.Millisecond);
            GenerateLevel(levelSeed);
        }

        public Level()
        {
            _visualProperties = LevelVisualProperties.FromTheme();
        }

        public float GetGroundYAtX(float x)
        {
            if (!float.IsNaN(_lastQueryX) && Math.Abs(_lastQueryX - x) < float.Epsilon)
                return _lastGroundY;

            if (IsOutOfBounds(x))
                return _lastGroundY = GetOutOfBoundsValue(x);

            UpdateSegmentIndex(x);
            _lastQueryX = x;
            return _lastGroundY = InterpolateGroundY(x);
        }

        public bool IsInSafeZone(float x) =>
            !IsOutOfBounds(x) && (x <= SafeZoneStartLength || x >= Length - SafeZoneEndLength);

        public float CalculateSlopeAngle(float x) =>
            (float)Atan2(GetGroundYAtX(x + DeltaX) - GetGroundYAtX(x - DeltaX), 2 * DeltaX);

        public bool IsFinishReached(Vector2 pos) =>
            Vector2.Distance(pos, FinishPoint) <= FinishReachDistance;

        public void UpdateVisualProperties(LevelVisualProperties visualProps)
        {
            _visualProperties = visualProps;
        }

        private void GenerateLevel(int seed)
        {
            var gen = new LevelGenerator(seed, Difficulty, new SegmentSelection());
            float terrainLength = (float)Min(
                BaseTerrainLength +
                TerrainLengthIncreasePerLevel * Difficulty,
                MaxTerrainLength
            );

            var config = new TerrainConfig(
                terrainLength,
                PointOffset,
                PointOffset,
                Difficulty
            );

            (TerrainPoints, Length, StartPoint, FinishPoint, SafeZoneStartLength, SafeZoneEndLength) =
                gen.GenerateTerrain(config);
        }

        private bool IsOutOfBounds(float x) =>
            TerrainPoints.Count < 2 || x < TerrainPoints[0].X || x > TerrainPoints[^1].X;

        private float GetOutOfBoundsValue(float x)
        {
            if (TerrainPoints.Count == 0)
                return DefaultGroundY;

            if (x < TerrainPoints[0].X)
                return TerrainPoints[0].YMiddle;

            if (x > TerrainPoints[^1].X)
                return TerrainPoints[^1].YMiddle;

            return DefaultGroundY;
        }

        private void UpdateSegmentIndex(float x)
        {
            if (TerrainPoints.Count < 2)
            {
                _currentSegmentIndex = 0;
                return;
            }

            if (IsInSegment(_currentSegmentIndex, x))
                return;

            foreach (int idx in new[] { _currentSegmentIndex + 1, _currentSegmentIndex - 1 })
            {
                if (IsInSegment(idx, x))
                {
                    _currentSegmentIndex = idx;
                    return;
                }
            }

            _currentSegmentIndex = FindTerrainSegmentIndex(x);
        }

        private bool IsInSegment(int idx, float x)
        {
            if (idx < 0 || idx >= TerrainPoints.Count - 1)
                return false;

            var p1 = TerrainPoints[idx];
            var p2 = TerrainPoints[idx + 1];
            return x >= p1.X && x < p2.X;
        }

        private float InterpolateGroundY(float x)
        {
            if (_currentSegmentIndex < 0 || _currentSegmentIndex >= TerrainPoints.Count - 1)
                return DefaultGroundY;

            var p1 = TerrainPoints[_currentSegmentIndex];
            var p2 = TerrainPoints[_currentSegmentIndex + 1];

            float t = (x - p1.X) / (p2.X - p1.X);
            return p1.YMiddle + (p2.YMiddle - p1.YMiddle) * t;
        }

        private int FindTerrainSegmentIndex(float x)
        {
            if (TerrainPoints.Count < 2)
                return 0;

            if (x < TerrainPoints[0].X)
                return 0;

            if (x > TerrainPoints[^1].X)
                return TerrainPoints.Count - 2;

            int left = 0, right = TerrainPoints.Count - 2;

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

        public class LevelGenerator
        {
            public enum TerrainSegmentType
            {
                Plateau, Spike, Depression, StepUp, StepDown, Wave, Jump
            }

            public abstract class SegmentParameters { }

            public class SpikeParameters : SegmentParameters
            {
                public float Height { get; }
                public float Pos { get; }

                public SpikeParameters(float height, float pos)
                {
                    Height = height;
                    Pos = pos;
                }
            }

            public class DepressionParameters : SegmentParameters
            {
                public float Depth { get; }
                public float Pos { get; }

                public DepressionParameters(float depth, float pos)
                {
                    Depth = depth;
                    Pos = pos;
                }
            }

            public class StepUpParameters : SegmentParameters
            {
                public float Height { get; }

                public StepUpParameters(float height)
                {
                    Height = height;
                }
            }

            public class StepDownParameters : SegmentParameters
            {
                public float Depth { get; }

                public StepDownParameters(float depth)
                {
                    Depth = depth;
                }
            }

            public class WaveParameters : SegmentParameters
            {
                public float Amplitude { get; }
                public float Frequency { get; }

                public WaveParameters(float amplitude, float frequency)
                {
                    Amplitude = amplitude;
                    Frequency = frequency;
                }
            }

            public class JumpParameters : SegmentParameters
            {
                public float Height { get; }
                public float JumpPos { get; }

                public JumpParameters(float height, float jumpPos)
                {
                    Height = height;
                    JumpPos = jumpPos;
                }
            }

            private readonly Random _rand;
            private readonly List<(TerrainSegmentType Type, SegmentParameters? Parameters)> _segments = new();
            private readonly int _difficulty;
            private readonly float _spikeHeight;
            private readonly float _depressionDepth;
            private readonly float _previousPointWeight;
            private readonly int _segmentCount;
            private readonly int _pointCount;
            private readonly int _terrainTypeCount;
            private readonly ISegmentSelection _segment;
            private readonly float[] _segmentBoundaries;

            public LevelGenerator(int seed, int difficulty, ISegmentSelection segment)
            {
                _rand = new Random(seed);
                _difficulty = Max(1, difficulty);
                _segment = segment ?? throw new ArgumentNullException(nameof(segment));

                _spikeHeight = (float)Min(
                    BaseSpikeHeight +
                    SpikeHeightIncreasePerLevel * _difficulty,
                    MaxSpikeHeight
                );

                _depressionDepth = (float)Min(
                    BaseDepressionDepth +
                    DepressionDepthIncreasePerLevel * _difficulty,
                    MaxDepressionDepth
                );

                _previousPointWeight = (float)Max(
                    BasePreviousPointWeight -
                    PreviousPointWeightDecreasePerLevel * _difficulty,
                    MinPreviousPointWeight
                );

                int baseSegments = BaseSegmentCount +
                                  SegmentCountIncreasePerLevel * _difficulty;
                int variation = _rand.Next(-2, 3);
                _segmentCount = Max(1, Min(baseSegments + variation, MaxSegmentCount));

                _pointCount = Min(
                    BasePointCount +
                    PointCountIncreasePerLevel * _difficulty,
                    MaxPointCount
                );

                _terrainTypeCount = Min(
                    MinTerrainTypeCount +
                    (_difficulty / TerrainTypeUnlockLevel),
                    MaxTerrainTypeCount
                );

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
                _segments.Add((Plateau, null));

                for (int i = 1; i < _segmentCount; i++)
                {
                    TerrainSegmentType nextType = _segment.SelectSegment(
                        _segments.Select(s => s.Type).ToList(),
                        i,
                        _terrainTypeCount,
                        _rand,
                        _difficulty
                    );

                    _segments.Add((nextType, GenerateParameters(nextType)));
                }

                if (_segments.Count > 2 && _rand.NextDouble() < FinalPlateauChance)
                    _segments[^1] = (Plateau, null);
            }

            private SegmentParameters? GenerateParameters(TerrainSegmentType type)
            {
                switch (type)
                {
                    case Plateau:
                        return null;

                    case Spike:
                        return new SpikeParameters(
                            (float)(_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _spikeHeight,
                            (float)_rand.NextDouble()
                        );

                    case Depression:
                        return new DepressionParameters(
                            (float)(_rand.NextDouble() * SpikeHeightFactor + SpikeHeightFactor) * _depressionDepth,
                            (float)_rand.NextDouble()
                        );

                    case StepUp:
                        return new StepUpParameters(
                            (float)_rand.NextDouble() * _spikeHeight * SpikeMidFactor
                        );

                    case StepDown:
                        return new StepDownParameters(
                            (float)_rand.NextDouble() * _depressionDepth * SpikeMidFactor
                        );

                    case Wave:
                        return new WaveParameters(
                            (float)_rand.NextDouble() * _spikeHeight * 0.5f,
                            2 + (float)_rand.NextDouble() * 2
                        );

                    case Jump:
                        return new JumpParameters(
                            (float)_rand.NextDouble() * _spikeHeight,
                            0.3f + (float)_rand.NextDouble() * 0.4f
                        );

                    default:
                        throw new ArgumentException($"Неизвестный тип сегмента: {type}");
                }
            }

            private float GetRawTerrainHeight(float baseY, float progress)
            {
                int segmentIndex = FindSegmentIndex(progress);
                float segStart = _segmentBoundaries[segmentIndex];
                float segEnd = _segmentBoundaries[segmentIndex + 1];

                float segmentProgress = (progress - segStart) / (segEnd - segStart);
                segmentProgress = (segmentProgress + (float)(_rand.NextDouble() * 0.3)) % 1.0f;

                var (type, parameters) = _segments[segmentIndex];
                return baseY + ApplyVariation(type, parameters, segmentProgress);
            }

            private int FindSegmentIndex(float progress)
            {
                int index = Array.BinarySearch(_segmentBoundaries, progress);

                if (index < 0)
                    index = ~index - 1;

                return Max(0, Min(index, _segmentBoundaries.Length - 2));
            }

            private float ApplyVariation(TerrainSegmentType type, SegmentParameters? parameters, float progress)
            {
                switch (type)
                {
                    case Plateau:
                        return 0;

                    case Spike:
                        {
                            var p = (SpikeParameters)parameters!;
                            return p.Height * (float)Exp(-GaussianFactor * Pow(progress - p.Pos, 2));
                        }

                    case Depression:
                        {
                            var p = (DepressionParameters)parameters!;
                            return -p.Depth * (float)Exp(-GaussianFactor * Pow(progress - p.Pos, 2));
                        }

                    case StepUp:
                        {
                            var p = (StepUpParameters)parameters!;
                            return p.Height * (float)Min(1.0, progress * 2);
                        }

                    case StepDown:
                        {
                            var p = (StepDownParameters)parameters!;
                            return -p.Depth * (float)Min(1.0, progress * 2);
                        }

                    case Wave:
                        {
                            var p = (WaveParameters)parameters!;
                            return p.Amplitude * (float)Sin((p.Frequency * progress + _rand.NextDouble()) * PI);
                        }

                    case Jump:
                        {
                            var p = (JumpParameters)parameters!;
                            return progress > p.JumpPos ? p.Height : 0;
                        }

                    default:
                        throw new ArgumentException($"Неизвестный тип сегмента: {type}");
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
                float smoothValue = lastY * _previousPointWeight + trans * SmoothingFactor;
                return ApplyNoise(smoothValue);
            }

            private float ApplyNoise(float value)
            {
                if (_difficulty > NoiseThreshold)
                {
                    float noiseAmplitude = (float)Min(
                        NoiseAmplitudeFactor * (_difficulty - NoiseThreshold),
                        MaxNoiseAmplitude
                    );

                    value += (float)(_rand.NextDouble() * 2 - 1) * noiseAmplitude;
                }
                return value;
            }

            public (List<Level.TerrainPoint> Points, float Length, Vector2 StartPoint, Vector2 FinishPoint,
                   float SafeZoneStart, float SafeZoneEnd) GenerateTerrain(TerrainConfig config)
            {
                float length = config.Length;
                var pts = new List<Level.TerrainPoint>(capacity: _pointCount + 10);
                float baseY = DefaultTerrainHeight;
                float safeZoneLength = FixedSafeZoneLength;

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
                    x: 0,
                    yTop: baseY - config.TopOffset,
                    yMiddle: baseY,
                    yBottom: baseY + config.BottomOffset,
                    isSafeZone: true
                ));

                return new Vector2(
                    StartPointXOffset,
                    baseY + StartPointYOffset
                );
            }

            private static Vector2 CreateEndPoint(List<Level.TerrainPoint> pts, float baseY, float length, TerrainConfig config)
            {
                pts.Add(new Level.TerrainPoint(
                    x: length,
                    yTop: baseY - config.TopOffset,
                    yMiddle: baseY,
                    yBottom: baseY + config.BottomOffset,
                    isSafeZone: true
                ));

                return new Vector2(
                    length + FinishPointXOffset,
                    baseY + FinishPointYOffset
                );
            }

            private void GenerateSafeZone(List<Level.TerrainPoint> pts, float baseY, TerrainConfig config,
                                         float startX, float length)
            {
                int count = Max(
                    MinSafeZonePointCount,
                    (int)(length / config.Length * _pointCount * 0.5)
                );

                float step = length / count;

                for (int i = 1; i <= count; i++)
                {
                    pts.Add(new Level.TerrainPoint(
                        x: startX + i * step,
                        yTop: baseY - config.TopOffset,
                        yMiddle: baseY,
                        yBottom: baseY + config.BottomOffset,
                        isSafeZone: true
                    ));
                }
            }

            private void GenerateTerrainPoints(List<Level.TerrainPoint> pts, TerrainConfig config,
                                              float baseY, float length, float safeStart, float safeEnd)
            {
                float midLen = length - safeStart - safeEnd;
                float lastY = baseY;

                int count = _pointCount - pts.Count - PointCountReductionFactor;
                count = Max(1, count);

                float step = midLen / count;

                for (int i = 1; i <= count; i++)
                {
                    float progress = (float)i / count;
                    float x = safeStart + i * step;
                    float y = CalculateTerrainHeight(baseY, progress, lastY);

                    pts.Add(new Level.TerrainPoint(
                        x: x,
                        yTop: y - config.TopOffset,
                        yMiddle: y,
                        yBottom: y + config.BottomOffset,
                        isSafeZone: false
                    ));

                    lastY = y;
                }
            }
        }
    }

    public class LevelComponentFactory
    {
        public static ILevelPhysics CreatePhysicsData(Level level) => level;

        public static ILevelVisualData CreateVisualData(Level level) => level;
    }

    public interface ISegmentSelection
    {
        Level.LevelGenerator.TerrainSegmentType SelectSegment(
            List<Level.LevelGenerator.TerrainSegmentType> previousSegments,
            int currentIndex,
            int terrainTypeCount,
            Random rand,
            int difficulty);
    }

    public class SegmentSelection : ISegmentSelection
    {
        public Level.LevelGenerator.TerrainSegmentType SelectSegment(
            List<Level.LevelGenerator.TerrainSegmentType> previousSegments,
            int currentIndex,
            int terrainTypeCount,
            Random rand,
            int difficulty)
        {
            var excludedTypes = new HashSet<Level.LevelGenerator.TerrainSegmentType>();

            if (currentIndex > 0 && previousSegments.Count > 0)
                excludedTypes.Add(previousSegments[currentIndex - 1]);

            var nextType = GetRandomSegmentType(rand, terrainTypeCount, excludedTypes);

            if (difficulty > SegmentDifficultyThreshold &&
                rand.NextDouble() < SegmentDifficultyChance * (difficulty / SegmentDifficultyDivisor))
            {
                excludedTypes.Add(Level.LevelGenerator.TerrainSegmentType.Plateau);
                nextType = GetRandomSegmentType(rand, terrainTypeCount, excludedTypes);
            }

            return nextType;
        }

        private Level.LevelGenerator.TerrainSegmentType GetRandomSegmentType(
            Random rand,
            int terrainTypeCount,
            HashSet<Level.LevelGenerator.TerrainSegmentType> excludedTypes)
        {
            if (excludedTypes.Count >= terrainTypeCount)
                return (Level.LevelGenerator.TerrainSegmentType)rand.Next(terrainTypeCount);

            Level.LevelGenerator.TerrainSegmentType type;
            do
            {
                type = (Level.LevelGenerator.TerrainSegmentType)rand.Next(terrainTypeCount);
            } while (excludedTypes.Contains(type));

            return type;
        }
    }
}