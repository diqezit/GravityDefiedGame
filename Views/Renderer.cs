﻿using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using System.Linq;
using System.Threading;
using GravityDefiedGame.Controllers;
using GravityDefiedGame.Models;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Models.BikeGeom;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame.Views
{
    public static class ThemeConstants
    {
        public static readonly Color
            BackgroundColor = new(255, 204, 102),
            TerrainColor = new(204, 153, 102),
            SafeZoneColor = new(152, 251, 152),
            VerticalLineColor = new(204, 153, 102);
    }

    public class Renderer
    {
        private const float
            Perspective = 0.2f,
            TerrainStrokeThickness = 3.0f,
            VerticalLineStrokeThickness = 0.5f,
            BikeStrokeThickness = 3.0f,
            ShadowOpacity = 0.5f,
            ShadowOffsetFactor = 0.2f,
            ShadowScaleFactor = 0.01f;

        private const int
            ShadowInterpolationSteps = 5;

        private readonly SpriteBatch _spriteBatch;
        private readonly GraphicsDevice _graphicsDevice;
        private readonly Texture2D _pixelTexture;
        private readonly Camera _camera;
        private readonly GameController _gameController;
        private readonly SkeletonRenderer _skeletonRenderer;
        private readonly ShadowRenderer _shadowRenderer;
        private readonly ColorSet _colors;

        public Renderer(SpriteBatch spriteBatch, GraphicsDevice graphicsDevice, GameController gameController, Camera camera)
        {
            _spriteBatch = spriteBatch ?? throw new ArgumentNullException(nameof(spriteBatch));
            _graphicsDevice = graphicsDevice ?? throw new ArgumentNullException(nameof(graphicsDevice));
            _gameController = gameController ?? throw new ArgumentNullException(nameof(gameController));
            _camera = camera ?? throw new ArgumentNullException(nameof(camera));

            _pixelTexture = new Texture2D(graphicsDevice, 1, 1);
            _pixelTexture.SetData(new[] { Color.White });

            _colors = CreateColorSet();
            _skeletonRenderer = new SkeletonRenderer(_spriteBatch, _pixelTexture, _camera, _colors.LineTypeColors);
            _shadowRenderer = new ShadowRenderer(_spriteBatch, _pixelTexture, _camera, _gameController);

            Log("Renderer", "Initializing renderer", () => {
                _shadowRenderer.Initialize();
                _skeletonRenderer.InitializeSkeletonLines();
                Info("Renderer", "Renderer initialized successfully");
            });
        }

        private ColorSet CreateColorSet() => new(
            LineTypeColors: new Dictionary<SkeletonLineType, Color>
            {
                [SkeletonLineType.MainFrame] = new Color(150, 150, 150),
                [SkeletonLineType.Suspension] = Color.Black,
                [SkeletonLineType.Wheel] = Color.Black,
                [SkeletonLineType.Seat] = new Color(200, 200, 200),
                [SkeletonLineType.Handlebar] = new Color(150, 150, 150),
                [SkeletonLineType.Exhaust] = new Color(100, 100, 100)
            },
            WheelFill: new Color(200, 200, 200),
            WheelStroke: Color.Black,
            SpokeStroke: Color.Black,
            TerrainFill: null,
            TerrainStroke: ThemeConstants.TerrainColor,
            SuspensionStroke: Color.DarkGray,
            SuspensionFill: new Color(180, 180, 180),
            ShadowStroke: new Color((byte)0, (byte)0, (byte)0, (255 * ShadowOpacity))
        );

        public void Render(CancellationToken cancellationToken = default) =>
            Log("Renderer", "Rendering frame", () => {
                if (cancellationToken.IsCancellationRequested) return;

                UpdateTerrainVisual(cancellationToken);
                if (cancellationToken.IsCancellationRequested) return;

                UpdateMotorcycleVisual(cancellationToken);
            });

        private void UpdateTerrainVisual(CancellationToken cancellationToken)
        {
            var terrainPoints = _gameController.CurrentLevel?.TerrainPoints ?? new List<Level.TerrainPoint>();
            if (terrainPoints.Count == 0) return;

            for (int i = 0; i < terrainPoints.Count - 1; i++)
            {
                if (cancellationToken.IsCancellationRequested) return;

                var point1 = terrainPoints[i];
                var point2 = terrainPoints[i + 1];

                RenderTerrainSegment(point1, point2);
            }

            UpdateVerticalLines(terrainPoints, cancellationToken);
        }

        private void RenderTerrainSegment(Level.TerrainPoint point1, Level.TerrainPoint point2)
        {
            var topPoint1 = GetTopPointWithOffset(point1);
            var topPoint2 = GetTopPointWithOffset(point2);

            if (IsValidPoint(topPoint1) && IsValidPoint(topPoint2))
            {
                DrawLine(
                    topPoint1,
                    topPoint2,
                    point1.IsSafeZone ? ThemeConstants.SafeZoneColor : _colors.TerrainStroke,
                    TerrainStrokeThickness
                );
            }

            var bottomPoint1 = new Vector2((float)point1.X, (float)point1.YBottom); // Мировые координаты
            var bottomPoint2 = new Vector2((float)point2.X, (float)point2.YBottom); // Мировые координаты

            if (IsValidPoint(bottomPoint1) && IsValidPoint(bottomPoint2))
            {
                DrawLine(
                    bottomPoint1,
                    bottomPoint2,
                    point1.IsSafeZone ? ThemeConstants.SafeZoneColor : _colors.TerrainStroke,
                    TerrainStrokeThickness
                );
            }
        }

        private Vector2 GetTopPointWithOffset(Level.TerrainPoint point)
        {
            float bikeX = _gameController.Motorcycle.Position.X;
            float dx = (float)(point.X - bikeX);
            float offsetX = -Perspective * dx;
            float worldX = (float)point.X + offsetX;
            float worldY = (float)point.YTop;
            return new Vector2(worldX, worldY); // Возвращаем мировые координаты
        }

        private void UpdateVerticalLines(List<Level.TerrainPoint> terrainPoints, CancellationToken cancellationToken)
        {
            foreach (var point in terrainPoints)
            {
                if (cancellationToken.IsCancellationRequested) return;

                var topPoint = GetTopPointWithOffset(point);
                var bottomPoint = new Vector2((float)point.X, (float)point.YBottom); // Мировые координаты

                if (IsValidPoint(topPoint) && IsValidPoint(bottomPoint))
                {
                    DrawLine(
                        topPoint,
                        bottomPoint,
                        ThemeConstants.VerticalLineColor,
                        VerticalLineStrokeThickness
                    );
                }
            }
        }

        private void UpdateMotorcycleVisual(CancellationToken cancellationToken)
        {
            var motorcycle = _gameController.Motorcycle;
            if (motorcycle is null) return;

            var (skeletonPoints, skeletonLines) = motorcycle.GetSkeleton();
            _skeletonRenderer.UpdateSkeletonVisuals(skeletonPoints, skeletonLines);
            _shadowRenderer.UpdateShadow(skeletonPoints, skeletonLines, cancellationToken);
        }

        private bool IsValidPoint(Vector2 point) =>
            !float.IsNaN(point.X) && !float.IsNaN(point.Y) &&
            !float.IsInfinity(point.X) && !float.IsInfinity(point.Y);

        private void DrawLine(Vector2 start, Vector2 end, Color color, float thickness)
        {
            if (!IsValidPoint(start) || !IsValidPoint(end)) return;

            float length = Vector2.Distance(start, end);
            if (length < 0.001f) return;

            float rotation = (float)Math.Atan2(end.Y - start.Y, end.X - start.X);

            _spriteBatch.Draw(
                _pixelTexture,
                start,
                null,
                color,
                rotation,
                Vector2.Zero,
                new Vector2(length, thickness),
                SpriteEffects.None,
                0
            );
        }

        private record ColorSet(
            Dictionary<SkeletonLineType, Color> LineTypeColors,
            Color WheelFill,
            Color WheelStroke,
            Color SpokeStroke,
            Color? TerrainFill,
            Color TerrainStroke,
            Color SuspensionStroke,
            Color SuspensionFill,
            Color ShadowStroke
        );

        #region ShadowRenderer
        public class ShadowRenderer
        {
            private readonly SpriteBatch _spriteBatch;
            private readonly Texture2D _pixelTexture;
            private readonly Camera _camera;
            private readonly GameController _gameController;

            public ShadowRenderer(SpriteBatch spriteBatch, Texture2D pixelTexture, Camera camera, GameController gameController)
            {
                _spriteBatch = spriteBatch;
                _pixelTexture = pixelTexture;
                _camera = camera;
                _gameController = gameController;
            }

            public void Initialize() =>
                Log("ShadowRenderer", "Initializing shadow", () => {
                    Info("ShadowRenderer", "Shadow initialized");
                });

            public void UpdateShadow(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines, CancellationToken cancellationToken) =>
                Log("ShadowRenderer", "Updating shadow", () => {
                    if (_gameController.CurrentLevel is null)
                    {
                        Warning("ShadowRenderer", "Current level is null");
                        return;
                    }

                    var framePoints = GetValidFramePoints(skeletonPoints, skeletonLines);
                    if (framePoints.Count < 2)
                    {
                        Debug("ShadowRenderer", "Not enough frame points");
                        return;
                    }

                    var (centerX, centerY, scale) = CalculateShadowParameters(framePoints);
                    if (scale == 0)
                    {
                        Debug("ShadowRenderer", "Invalid scale value");
                        return;
                    }

                    var shadowPoints = GenerateShadowPoints(framePoints, centerX, centerY, scale, cancellationToken);
                    if (shadowPoints.Count < 2)
                    {
                        Debug("ShadowRenderer", "Not enough shadow points");
                        return;
                    }

                    DrawShadow(shadowPoints);
                });

            private void DrawShadow(List<Vector2> shadowPoints)
            {
                if (shadowPoints.Count < 2)
                    return;

                for (int i = 0; i < shadowPoints.Count - 1; i++)
                {
                    var worldPoint1 = shadowPoints[i];     
                    var worldPoint2 = shadowPoints[i + 1];

                    if (IsValidPoint(worldPoint1) && IsValidPoint(worldPoint2))
                    {
                        float length = Vector2.Distance(worldPoint1, worldPoint2);
                        if (length < 0.001f)
                            continue;

                        float rotation = (float)Math.Atan2(worldPoint2.Y - worldPoint1.Y, worldPoint2.X - worldPoint1.X);

                        _spriteBatch.Draw(
                            _pixelTexture,
                            worldPoint1,
                            null,
                            new Color((byte)0, (byte)0, (byte)0, (byte)(255 * ShadowOpacity)),
                            rotation,
                            Vector2.Zero,
                            new Vector2(length, BikeStrokeThickness),
                            SpriteEffects.None,
                            0
                        );
                    }
                }
            }

            private bool IsValidPoint(Vector2 point) =>
                !float.IsNaN(point.X) && !float.IsNaN(point.Y) &&
                !float.IsInfinity(point.X) && !float.IsInfinity(point.Y);

            private Vector2 CalculateShadowPoint(Vector2 framePoint) =>
                Log("ShadowRenderer", $"Calculating shadow point for ({framePoint.X:F1}, {framePoint.Y:F1})", () => {
                    if (!IsValidPoint(framePoint) || _gameController.CurrentLevel is null)
                    {
                        Debug("ShadowRenderer", "Invalid frame point or null level");
                        return framePoint;
                    }

                    try
                    {
                        double groundY = _gameController.CurrentLevel.GetGroundYAtX(framePoint.X);
                        if (double.IsInfinity(groundY) || double.IsNaN(groundY))
                        {
                            Debug("ShadowRenderer", $"Invalid ground Y at X={framePoint.X:F1}");
                            return framePoint;
                        }

                        double heightAboveGround = groundY - framePoint.Y;
                        if (heightAboveGround <= 0 || double.IsInfinity(heightAboveGround) || double.IsNaN(heightAboveGround))
                            return framePoint;

                        double shadowX = framePoint.X + heightAboveGround * ShadowOffsetFactor;
                        if (double.IsInfinity(shadowX) || double.IsNaN(shadowX))
                        {
                            Debug("ShadowRenderer", $"Invalid shadow X={shadowX:F1}");
                            return framePoint;
                        }

                        double shadowGroundY = _gameController.CurrentLevel.GetGroundYAtX((float)shadowX);
                        return double.IsFinite(shadowGroundY) ? new Vector2((float)shadowX, (float)shadowGroundY) : framePoint;
                    }
                    catch (Exception ex)
                    {
                        Error("ShadowRenderer", $"Exception in shadow calculation: {ex.Message}");
                        return framePoint;
                    }
                }, framePoint);

            private List<SkeletonPoint> GetValidFramePoints(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines) =>
                Log("ShadowRenderer", "Getting valid frame points", () => {
                    var mainFrameLines = skeletonLines.Where(l => l.Type == SkeletonLineType.MainFrame).ToList();
                    var framePointsIndices = new HashSet<int>(mainFrameLines.SelectMany(line => new[] { line.StartPointIndex, line.EndPointIndex }));
                    return framePointsIndices
                        .Select(idx => skeletonPoints[idx])
                        .OrderBy(p => p.Position.X)
                        .Where(p => IsValidPoint(p.Position))
                        .ToList();
                }, new List<SkeletonPoint>());

            private (float centerX, float centerY, float scale) CalculateShadowParameters(List<SkeletonPoint> framePoints) =>
                Log("ShadowRenderer", "Calculating shadow parameters", () => {
                    if (_gameController.CurrentLevel is null || framePoints.Count < 2)
                    {
                        Debug("ShadowRenderer", "Invalid level or insufficient points");
                        return (0, 0, 1);
                    }

                    float totalHeight = 0;
                    int validPointsCount = 0;

                    foreach (var point in framePoints)
                    {
                        try
                        {
                            double groundY = _gameController.CurrentLevel.GetGroundYAtX(point.Position.X);
                            float heightAboveGround = (float)(groundY - point.Position.Y);
                            if (float.IsFinite(heightAboveGround))
                            {
                                totalHeight += heightAboveGround > 0 ? heightAboveGround : 0;
                                validPointsCount++;
                            }
                        }
                        catch (Exception ex)
                        {
                            Debug("ShadowRenderer", $"Error calculating height: {ex.Message}");
                            continue;
                        }
                    }

                    if (validPointsCount == 0)
                    {
                        Debug("ShadowRenderer", "No valid points for height calculation");
                        return (0, 0, 1);
                    }

                    float averageHeight = totalHeight / validPointsCount;
                    float scale = 1 / MathHelper.Max(1 + ShadowScaleFactor * averageHeight, 0.1f);

                    float minX = framePoints.Min(p => p.Position.X);
                    float maxX = framePoints.Max(p => p.Position.X);
                    float centerX = (minX + maxX) / 2;

                    try
                    {
                        double centerY = _gameController.CurrentLevel.GetGroundYAtX(centerX);
                        if (double.IsFinite(centerY))
                            return (centerX, (float)centerY, scale);
                    }
                    catch (Exception ex)
                    {
                        Debug("ShadowRenderer", $"Error getting center Y: {ex.Message}");
                    }

                    return (0, 0, 0);
                }, (0, 0, 0));

            private List<Vector2> GenerateShadowPoints(
                List<SkeletonPoint> framePoints,
                float centerX,
                float centerY,
                float scale,
                CancellationToken cancellationToken) =>
                Log("ShadowRenderer", "Generating shadow points", () => {
                    var shadowPoints = new List<Vector2>();
                    if (!float.IsFinite(centerX) || !float.IsFinite(centerY) || !float.IsFinite(scale))
                    {
                        Debug("ShadowRenderer", "Invalid shadow parameters");
                        return shadowPoints;
                    }

                    for (int i = 0; i < framePoints.Count - 1; i++)
                    {
                        if (cancellationToken.IsCancellationRequested)
                        {
                            Warning("ShadowRenderer", "Shadow points generation cancelled");
                            return shadowPoints;
                        }

                        var start = framePoints[i].Position;
                        var end = framePoints[i + 1].Position;
                        if (!IsValidPoint(start) || !IsValidPoint(end))
                            continue;

                        for (int step = 0; step <= ShadowInterpolationSteps; step++)
                        {
                            if (cancellationToken.IsCancellationRequested)
                            {
                                Warning("ShadowRenderer", "Shadow points generation cancelled");
                                return shadowPoints;
                            }

                            float t = (float)step / ShadowInterpolationSteps;
                            var interpolatedPoint = Vector2.Lerp(start, end, t);

                            if (!IsValidPoint(interpolatedPoint))
                                continue;

                            var shadowPoint = CalculateShadowPoint(interpolatedPoint);
                            if (!IsValidPoint(shadowPoint))
                                continue;

                            float dx = shadowPoint.X - centerX;
                            float dy = shadowPoint.Y - centerY;
                            shadowPoint = new Vector2(
                                centerX + scale * dx,
                                centerY + scale * dy
                            );

                            if (IsValidPoint(shadowPoint))
                                shadowPoints.Add(shadowPoint);
                        }
                    }
                    return shadowPoints;
                }, new List<Vector2>());
        }
        #endregion

        #region SkeletonRenderer
        public class SkeletonRenderer
        {
            private readonly SpriteBatch _spriteBatch;
            private readonly Texture2D _pixelTexture;
            private readonly Camera _camera;
            private readonly Dictionary<SkeletonLineType, Color> _lineTypeColors;

            public SkeletonRenderer(SpriteBatch spriteBatch, Texture2D pixelTexture, Camera camera,
                                   Dictionary<SkeletonLineType, Color> lineTypeColors)
            {
                _spriteBatch = spriteBatch;
                _pixelTexture = pixelTexture;
                _camera = camera;
                _lineTypeColors = lineTypeColors;
            }

            public void InitializeSkeletonLines() =>
                Log("SkeletonRenderer", "Initializing skeleton lines", () => {
                    Info("SkeletonRenderer", "Skeleton lines initialized");
                });

            public void UpdateSkeletonVisuals(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines) =>
                    Log("SkeletonRenderer", "Updating skeleton visuals", () => {
                        foreach (var line in skeletonLines)
                        {
                            if (line.StartPointIndex >= 0 && line.StartPointIndex < skeletonPoints.Count &&
                                line.EndPointIndex >= 0 && line.EndPointIndex < skeletonPoints.Count)
                            {
                                var startPoint = skeletonPoints[line.StartPointIndex].Position; 
                                var endPoint = skeletonPoints[line.EndPointIndex].Position;     

                                if (IsValidPoint(startPoint) && IsValidPoint(endPoint))
                                {
                                    DrawLine(startPoint, endPoint, GetColorForLineType(line.Type), BikeStrokeThickness);
                                }
                            }
                        }
                    });

            public void DrawSkeleton(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines)
            {
                UpdateSkeletonVisuals(skeletonPoints, skeletonLines);
            }

            private void DrawLine(Vector2 start, Vector2 end, Color color, float thickness)
            {
                if (!IsValidPoint(start) || !IsValidPoint(end))
                    return;

                float length = Vector2.Distance(start, end);
                if (length < 0.001f)
                    return;

                float rotation = (float)Math.Atan2(end.Y - start.Y, end.X - start.X);

                _spriteBatch.Draw(
                    _pixelTexture,
                    start,
                    null,
                    color,
                    rotation,
                    Vector2.Zero,
                    new Vector2(length, thickness),
                    SpriteEffects.None,
                    0
                );
            }

            private Color GetColorForLineType(SkeletonLineType lineType) =>
                _lineTypeColors.TryGetValue(lineType, out var color) ? color : _lineTypeColors[SkeletonLineType.MainFrame];

            private bool IsValidPoint(Vector2 point) =>
                !float.IsNaN(point.X) && !float.IsNaN(point.Y) &&
                !float.IsInfinity(point.X) && !float.IsInfinity(point.Y);
        }
        #endregion
    }
}