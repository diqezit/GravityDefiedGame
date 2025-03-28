#nullable enable
using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using static System.Math;
using static GravityDefiedGame.Models.LevelConstants;
using static GravityDefiedGame.Models.GeneratorConstants;

namespace GravityDefiedGame.Models
{
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

    public static class GeneratorConstants
    {
        // Base terrain parameters
        public const float DefaultTerrainHeight = 500.0f;           // Default height of terrain

        // Spike parameters
        public const float BaseSpikeHeight = 50.0f;                 // Initial spike height
        public const float SpikeHeightIncreasePerLevel = 10.0f;     // Spike height increase per level
        public const float MaxSpikeHeight = 200.0f;                 // Maximum spike height
        public const float SpikeHeightFactor = 0.5f;                // Multiplier for spike height
        public const float SpikeMidFactor = 0.7f;                   // Mid-point factor for spikes

        // Depression parameters
        public const float BaseDepressionDepth = 30.0f;             // Initial depression depth
        public const float DepressionDepthIncreasePerLevel = 7.5f;  // Depth increase per level
        public const float MaxDepressionDepth = 150.0f;             // Maximum depression depth

        // Smoothing parameters
        public const float SmoothingFactor = 0.7f;                  // Base smoothing factor
        public const float BasePreviousPointWeight = 0.3f;          // Initial weight for previous points
        public const float PreviousPointWeightDecreasePerLevel = 0.02f; // Weight decrease per level
        public const float MinPreviousPointWeight = 0.1f;           // Minimum weight for previous points
        public const float GaussianFactor = 2.0f;                   // Factor for Gaussian functions

        // Zone configuration
        public const float StartZonePercent = 0.1f;                 // Percentage for start zone
        public const float EndZonePercent = 0.9f;                   // Percentage for end zone
        public const float PointOffset = 50.0f;                     // Offset for terrain points

        // Start and finish point offsets
        public const float StartPointXOffset = 100.0f;              // X offset for start point
        public const float StartPointYOffset = -80.0f;              // Y offset for start point
        public const float FinishPointXOffset = -100.0f;            // X offset for finish point
        public const float FinishPointYOffset = -50.0f;             // Y offset for finish point

        // Segment difficulty parameters
        public const float SegmentDifficultyThreshold = 5;          // Threshold for difficult segments
        public const float SegmentDifficultyChance = 0.2f;          // Chance for difficult segments
        public const float SegmentDifficultyDivisor = 10.0f;        // Divisor for difficulty calculation
        public const float FinalPlateauChance = 0.7f;               // Chance for final plateau

        // Noise parameters
        public const int NoiseThreshold = 3;                        // Difficulty threshold for noise
        public const float NoiseAmplitudeFactor = 5.0f;             // Amplitude factor for noise
        public const float MaxNoiseAmplitude = 25.0f;               // Maximum noise amplitude

        // Point count parameters
        public const int BasePointCount = 60;                       // Initial point count
        public const int PointCountIncreasePerLevel = 5;            // Point count increase per level
        public const int MaxPointCount = 120;                       // Maximum point count

        // Segment count parameters
        public const int BaseSegmentCount = 5;                      // Initial segment count
        public const int SegmentCountIncreasePerLevel = 1;          // Segment count increase per level
        public const int MaxSegmentCount = 15;                      // Maximum segment count
        public const int MinSafeZonePointCount = 3;                 // Minimum points in safe zone
        public const int PointCountReductionFactor = 5;             // Factor for point count reduction

        // Terrain type parameters
        public const int MinTerrainTypeCount = 3;                   // Minimum terrain type count
        public const int MaxTerrainTypeCount = 7;                   // Maximum terrain type count
        public const int TerrainTypeUnlockLevel = 5;                // Level to unlock new terrain types
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

            if (progress > plateauEnd)
            {
                float descentProgress = (progress - plateauEnd) / (1 - plateauEnd);
                return endHeight * (1 - descentProgress);
            }

            float plateauProgress = (progress - plateauStart) / (plateauEnd - plateauStart);
            return startHeight + (endHeight - startHeight) * plateauProgress;
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

            // Create terrain in sequence
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

            List<Level.TerrainPoint> densePoints = [];
            densePoints.Add(new Level.TerrainPoint(
                safeStart,
                lastY - config.TopOffset,
                lastY,
                lastY + config.BottomOffset,
                false
            ));

            int densePointCount = Max(
                BasePointCount * 3,
                (BasePointCount + PointCountIncreasePerLevel * _difficulty) * 2
            );
            densePointCount = Min(densePointCount, MaxPointCount * 3);

            float step = midLen / densePointCount;

            for (int i = 1; i <= densePointCount; i++)
            {
                float progress = (float)i / densePointCount;
                float x = safeStart + i * step;
                float y = CalculateTerrainHeight(baseY, progress, lastY);

                densePoints.Add(new Level.TerrainPoint(
                    x,
                    y - config.TopOffset,
                    y,
                    y + config.BottomOffset,
                    false
                ));

                lastY = y;
            }

            HashSet<int> criticalIndices = [0, densePoints.Count - 1];

            for (int i = 0; i < _segmentBoundaries.Length - 1; i++)
            {
                float startX = safeStart + _segmentBoundaries[i] * midLen;
                float endX = safeStart + _segmentBoundaries[i + 1] * midLen;

                int startIdx = FindNearestPointIndex(densePoints, startX);
                int endIdx = FindNearestPointIndex(densePoints, endX);

                for (int j = -1; j <= 1; j++)
                {
                    int idx = startIdx + j;
                    if (idx >= 0 && idx < densePoints.Count)
                        criticalIndices.Add(idx);

                    idx = endIdx + j;
                    if (idx >= 0 && idx < densePoints.Count)
                        criticalIndices.Add(idx);
                }
            }

            float baseEpsilon = Clamp(8.0f / (_difficulty + 1), 0.5f, 10.0f);
            List<Level.TerrainPoint> simplifiedPoints = DouglasPeuckerWithCriticalPoints(
                densePoints, criticalIndices, baseEpsilon, safeStart, midLen
            );

            pts.AddRange(simplifiedPoints);
        }

        private static int FindNearestPointIndex(List<Level.TerrainPoint> points, float targetX)
        {
            if (points.Count == 0) return -1;
            if (targetX <= points[0].X) return 0;
            if (targetX >= points[^1].X) return points.Count - 1;

            int left = 0, right = points.Count - 1;

            while (right - left > 1)
            {
                int mid = (left + right) / 2;
                if (points[mid].X < targetX) left = mid;
                else right = mid;
            }

            return Abs(points[left].X - targetX) < Abs(points[right].X - targetX) ? left : right;
        }

        private List<Level.TerrainPoint> DouglasPeuckerWithCriticalPoints(
            List<Level.TerrainPoint> points, HashSet<int> criticalIndices,
            float baseEpsilon, float safeStart, float midLen)
        {
            if (points.Count <= 2) return new(points);

            List<int> sortedIndices = [.. criticalIndices.OrderBy(i => i)];
            List<Level.TerrainPoint> result = [];

            for (int i = 0; i < sortedIndices.Count - 1; i++)
            {
                int startIdx = sortedIndices[i];
                int endIdx = sortedIndices[i + 1];

                if (endIdx - startIdx <= 1)
                {
                    result.Add(points[startIdx]);
                    continue;
                }

                var subPoints = points.GetRange(startIdx, endIdx - startIdx + 1);
                float epsilon = GetEpsilonForSegment(
                    subPoints, baseEpsilon, points[startIdx].X, points[endIdx].X, safeStart, midLen);
                var simplified = DouglasPeucker(subPoints, epsilon);

                if (i > 0) simplified = simplified.Skip(1).ToList();
                result.AddRange(simplified);
            }

            return result;
        }

        private float GetEpsilonForSegment(
            List<Level.TerrainPoint> points, float baseEpsilon,
            float startX, float endX, float safeStart, float midLen)
        {
            if (points.Count < 2) return baseEpsilon;

            float midX = (startX + endX) / 2;
            float progress = (midX - safeStart) / midLen;
            int segmentIndex = FindSegmentIndex(progress);

            if (segmentIndex < 0 || segmentIndex >= _segments.Count) return baseEpsilon;

            float typeModifier = _segments[segmentIndex].Type switch
            {
                TerrainSegmentType.Plateau => 2.5f,
                TerrainSegmentType.SlantedPlateau => 2.0f,
                TerrainSegmentType.StepUp => 1.2f,
                TerrainSegmentType.StepDown => 1.2f,
                TerrainSegmentType.SmoothHill => 1.0f,
                TerrainSegmentType.Valley => 1.0f,
                TerrainSegmentType.Jump => 0.8f,
                TerrainSegmentType.Spike => 0.7f,
                TerrainSegmentType.Depression => 0.7f,
                TerrainSegmentType.Wave => 0.5f,
                TerrainSegmentType.Ridges => 0.4f,
                TerrainSegmentType.Rugged => 0.3f,
                _ => 1.0f
            };

            float minY = float.MaxValue, maxY = float.MinValue;
            foreach (var point in points)
            {
                minY = Min(minY, point.YMiddle);
                maxY = Max(maxY, point.YMiddle);
            }

            float heightVariation = maxY - minY;
            float heightModifier = 1.0f / (1.0f + heightVariation * 0.1f);

            return baseEpsilon * typeModifier * heightModifier;
        }

        private static List<Level.TerrainPoint> DouglasPeucker(List<Level.TerrainPoint> points, float epsilon)
        {
            if (points.Count <= 2) return new(points);

            float maxDistance = 0;
            int index = 0;

            for (int i = 1; i < points.Count - 1; i++)
            {
                float distance = PerpendicularDistance(points[i], points[0], points[^1]);
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    index = i;
                }
            }

            if (maxDistance > epsilon)
            {
                if (index <= 0 || index >= points.Count - 1)
                    return [points[0], points[^1]];

                var firstPart = DouglasPeucker(points.GetRange(0, index + 1), epsilon);
                var secondPart = DouglasPeucker(points.GetRange(index, points.Count - index), epsilon);

                var result = new List<Level.TerrainPoint>(firstPart);
                if (secondPart.Count > 1) result.AddRange(secondPart.Skip(1));
                return result;
            }

            return [points[0], points[^1]];
        }

        private static float PerpendicularDistance(Level.TerrainPoint point, Level.TerrainPoint lineStart, Level.TerrainPoint lineEnd)
        {
            float x0 = point.X, y0 = point.YMiddle;
            float x1 = lineStart.X, y1 = lineStart.YMiddle;
            float x2 = lineEnd.X, y2 = lineEnd.YMiddle;

            if (Abs(x2 - x1) < float.Epsilon)
                return Abs(x0 - x1);

            if (Abs(x2 - x1) < float.Epsilon && Abs(y2 - y1) < float.Epsilon)
                return (float)Sqrt(Pow(x0 - x1, 2) + Pow(y0 - y1, 2));

            float m = (y2 - y1) / (x2 - x1);
            float b = y1 - m * x1;

            return Abs(y0 - (m * x0 + b)) / (float)Sqrt(1 + m * m);
        }
    }
}
