#nullable enable

using System;
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
using static GravityDefiedGame.Models.DrawingComponent.DrawingConstants;

namespace GravityDefiedGame.Views
{
    public class Renderer : DrawingComponent
    {
        #region Private Fields
        private readonly SpriteBatch _spriteBatch;
        private readonly GraphicsDevice _graphicsDevice;
        private readonly Texture2D _pixelTexture;
        private readonly Camera _camera;
        private readonly GameController _gameController;
        private readonly SkeletonRenderer _skeletonRenderer;
        private readonly ShadowRenderer _shadowRenderer;
        private ColorSet _colors;
        #endregion

        public Renderer(SpriteBatch spriteBatch, GraphicsDevice graphicsDevice, GameController gameController, Camera camera)
        {
            _spriteBatch = spriteBatch ?? throw new ArgumentNullException(nameof(spriteBatch));
            _graphicsDevice = graphicsDevice ?? throw new ArgumentNullException(nameof(graphicsDevice));
            _gameController = gameController ?? throw new ArgumentNullException(nameof(gameController));
            _camera = camera ?? throw new ArgumentNullException(nameof(camera));

            _pixelTexture = new Texture2D(graphicsDevice, 1, 1);
            _pixelTexture.SetData(new[] { Color.White });

            _colors = CreateColorSetFromTheme(ThemeManager.CurrentTheme);
            _skeletonRenderer = new SkeletonRenderer(_spriteBatch, _pixelTexture, _camera, _colors.LineTypeColors);
            _shadowRenderer = new ShadowRenderer(_spriteBatch, _pixelTexture, _camera, _gameController);

            ThemeManager.ThemeChanged += OnThemeChanged;

            Log("Renderer", "Initializing renderer", () => {
                _shadowRenderer.Initialize();
                _skeletonRenderer.InitializeSkeletonLines();
                Info("Renderer", "Renderer initialized successfully");
            });
        }

        private void OnThemeChanged()
        {
            _colors = CreateColorSetFromTheme(ThemeManager.CurrentTheme);
            _skeletonRenderer.UpdateLineColors(_colors.LineTypeColors);
        }

        private static ColorSet CreateColorSetFromTheme(ThemeSettings theme) => new(
            LineTypeColors: theme.BikeColors,
            WheelFill: theme.WheelFill,
            WheelStroke: theme.WheelStroke,
            SpokeStroke: theme.SpokeStroke,
            TerrainFill: null,
            TerrainStroke: theme.TerrainColor,
            SuspensionStroke: Color.DarkGray,
            SuspensionFill: new Color(180, 180, 180),
            ShadowStroke: theme.ShadowColor
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

            if (PhysicsComponent.IsValidPoint(topPoint1) && PhysicsComponent.IsValidPoint(topPoint2))
            {
                DrawLine(
                    _spriteBatch,
                    _pixelTexture,
                    topPoint1,
                    topPoint2,
                    point1.IsSafeZone ? ThemeManager.CurrentTheme.SafeZoneColor : _colors.TerrainStroke,
                    TerrainStrokeThickness
                );
            }

            var bottomPoint1 = new Vector2((float)point1.X, (float)point1.YBottom);
            var bottomPoint2 = new Vector2((float)point2.X, (float)point2.YBottom);

            if (PhysicsComponent.IsValidPoint(bottomPoint1) && PhysicsComponent.IsValidPoint(bottomPoint2))
            {
                DrawLine(
                    _spriteBatch,
                    _pixelTexture,
                    bottomPoint1,
                    bottomPoint2,
                    point1.IsSafeZone ? ThemeManager.CurrentTheme.SafeZoneColor : _colors.TerrainStroke,
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
            return new Vector2(worldX, worldY);
        }

        private void UpdateVerticalLines(List<Level.TerrainPoint> terrainPoints, CancellationToken cancellationToken)
        {
            foreach (var point in terrainPoints)
            {
                if (cancellationToken.IsCancellationRequested) return;

                var topPoint = GetTopPointWithOffset(point);
                var bottomPoint = new Vector2((float)point.X, (float)point.YBottom);

                if (PhysicsComponent.IsValidPoint(topPoint) && PhysicsComponent.IsValidPoint(bottomPoint))
                {
                    DrawLine(
                        _spriteBatch,
                        _pixelTexture,
                        topPoint,
                        bottomPoint,
                        ThemeManager.CurrentTheme.VerticalLineColor,
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

            // Константы для теней
            private const float ShadowOffsetFactor = 0.2f;    // Фактор смещения тени
            private const float ShadowScaleFactor = 0.05f;    // Фактор масштаба тени
            private const int ShadowInterpolationSteps = 5;   // Шаги интерполяции для сглаживания

            public ShadowRenderer(SpriteBatch spriteBatch, Texture2D pixelTexture, Camera camera, GameController gameController)
            {
                _spriteBatch = spriteBatch;
                _pixelTexture = pixelTexture;
                _camera = camera;
                _gameController = gameController;
            }

            public void Initialize() =>
                Info("ShadowRenderer", "Shadow initialized");

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

                    if (PhysicsComponent.IsValidPoint(worldPoint1) && PhysicsComponent.IsValidPoint(worldPoint2))
                    {
                        DrawingComponent.DrawLine(
                            _spriteBatch,
                            _pixelTexture,
                            worldPoint1,
                            worldPoint2,
                            ThemeManager.CurrentTheme.ShadowColor,
                            BikeStrokeThickness
                        );
                    }
                }
            }

            private Vector2 CalculateShadowPoint(Vector2 framePoint) =>
                Log("ShadowRenderer", $"Calculating shadow point for ({framePoint.X:F1}, {framePoint.Y:F1})", () => {
                if (!PhysicsComponent.IsValidPoint(framePoint) || _gameController.CurrentLevel is null)
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

            private List<Vector2> GetValidFramePoints(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines)
            {
                // Выбираем точки основной рамы
                var framePointIndices = skeletonLines
                    .Where(l => l.Type == SkeletonLineType.MainFrame)
                    .SelectMany(l => new[] { l.StartPointIndex, l.EndPointIndex })
                    .Where(idx => idx >= 0 && idx < skeletonPoints.Count)
                    .ToHashSet();

                // Используем точки напрямую - они уже в мировой системе координат
                return framePointIndices
                    .Select(idx => skeletonPoints[idx].Position)
                    .Where(PhysicsComponent.IsValidPoint)
                    .OrderBy(p => p.X)
                    .ToList();
            }

            private (float centerX, float centerY, float scale) CalculateShadowParameters(List<Vector2> framePoints)
            {
                if (framePoints.Count == 0)
                    return (0, 0, 0);

                // Находим центр объекта
                float sumX = 0, sumY = 0;
                foreach (var point in framePoints)
                {
                    sumX += point.X;
                    sumY += point.Y;
                }

                float centerX = sumX / framePoints.Count;
                float centerY = sumY / framePoints.Count;

                // Вычисляем масштаб тени
                float scale = 1.0f - ShadowScaleFactor * framePoints.Count;
                return (centerX, centerY, float.IsNaN(scale) || scale <= 0 ? 0.5f : scale);
            }

            private List<Vector2> GenerateShadowPoints(List<Vector2> framePoints, float centerX, float centerY, float scale, CancellationToken cancellationToken)
            {
                var shadowPoints = new List<Vector2>();
                if (framePoints.Count == 0)
                    return shadowPoints;

                for (int i = 0; i < framePoints.Count - 1; i++)
                {
                    if (cancellationToken.IsCancellationRequested)
                        return shadowPoints;

                    var start = framePoints[i];
                    var end = framePoints[i + 1];

                    if (!PhysicsComponent.IsValidPoint(start) || !PhysicsComponent.IsValidPoint(end))
                        continue;

                    for (int step = 0; step <= ShadowInterpolationSteps; step++)
                    {
                        if (cancellationToken.IsCancellationRequested)
                            return shadowPoints;

                        float t = (float)step / ShadowInterpolationSteps;
                        var interpolatedPoint = Vector2.Lerp(start, end, t);

                        if (!PhysicsComponent.IsValidPoint(interpolatedPoint))
                            continue;

                        var shadowPoint = CalculateShadowPoint(interpolatedPoint);
                        if (!PhysicsComponent.IsValidPoint(shadowPoint))
                            continue;

                        float dx = shadowPoint.X - centerX;
                        float dy = shadowPoint.Y - centerY;
                        shadowPoint = new Vector2(
                            centerX + scale * dx,
                            centerY + scale * dy
                        );

                        if (PhysicsComponent.IsValidPoint(shadowPoint))
                            shadowPoints.Add(shadowPoint);
                    }
                }
                return shadowPoints;
            }
        }
        #endregion

        #region SkeletonRenderer
        public class SkeletonRenderer
        {
            private readonly SpriteBatch _spriteBatch;
            private readonly Texture2D _pixelTexture;
            private readonly Camera _camera;
            private Dictionary<SkeletonLineType, Color> _lineTypeColors;

            public SkeletonRenderer(SpriteBatch spriteBatch, Texture2D pixelTexture, Camera camera, Dictionary<SkeletonLineType, Color> lineTypeColors)
            {
                _spriteBatch = spriteBatch;
                _pixelTexture = pixelTexture;
                _camera = camera;
                _lineTypeColors = lineTypeColors;
            }

            public void InitializeSkeletonLines() =>
                Info("SkeletonRenderer", "Skeleton lines initialized");

            public void UpdateLineColors(Dictionary<SkeletonLineType, Color> newColors) =>
                _lineTypeColors = newColors;

            public void UpdateSkeletonVisuals(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines) =>
                Log("SkeletonRenderer", "Updating skeleton visuals", () => {
                    foreach (var line in skeletonLines)
                    {
                        if (line.StartPointIndex >= 0 && line.StartPointIndex < skeletonPoints.Count &&
                            line.EndPointIndex >= 0 && line.EndPointIndex < skeletonPoints.Count)
                        {
                            var startPoint = skeletonPoints[line.StartPointIndex].Position;
                            var endPoint = skeletonPoints[line.EndPointIndex].Position;

                            if (PhysicsComponent.IsValidPoint(startPoint) && PhysicsComponent.IsValidPoint(endPoint))
                            {
                                // Используем координаты напрямую без преобразования
                                DrawingComponent.DrawLine(
                                    _spriteBatch,
                                    _pixelTexture,
                                    startPoint,
                                    endPoint,
                                    GetColorForLineType(line.Type),
                                    BikeStrokeThickness
                                );
                            }
                        }
                    }
                });

            public void DrawSkeleton(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines) =>
                UpdateSkeletonVisuals(skeletonPoints, skeletonLines);

            private Color GetColorForLineType(SkeletonLineType lineType) =>
                _lineTypeColors.TryGetValue(lineType, out var color) ? color : _lineTypeColors[SkeletonLineType.MainFrame];
        }
        #endregion
    }
}