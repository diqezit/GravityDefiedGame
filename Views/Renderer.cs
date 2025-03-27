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
        private readonly TerrainRenderer _terrainRenderer;
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
            _terrainRenderer = new TerrainRenderer(_spriteBatch, _pixelTexture, _camera, _gameController, _colors);

            ThemeManager.ThemeChanged += OnThemeChanged;

            Log("Renderer", "Initializing renderer", () => {
                _shadowRenderer.Initialize();
                _skeletonRenderer.InitializeSkeletonLines();
                _terrainRenderer.Initialize();
                Info("Renderer", "Renderer initialized successfully");
            });
        }

        private void OnThemeChanged()
        {
            _colors = CreateColorSetFromTheme(ThemeManager.CurrentTheme);
            _skeletonRenderer.UpdateLineColors(_colors.LineTypeColors);
            _terrainRenderer.UpdateColors(_colors);
        }

        private static ColorSet CreateColorSetFromTheme(ThemeSettings theme) => new(
            LineTypeColors: theme.BikeColors,
            WheelFill: theme.WheelFill,
            WheelStroke: theme.WheelStroke,
            SpokeStroke: theme.SpokeStroke,
            TerrainFill: new Color((byte)theme.TerrainColor.R, (byte)theme.TerrainColor.G, (byte)theme.TerrainColor.B, (byte)100),
            TerrainStroke: theme.TerrainColor,
            SuspensionStroke: Color.DarkGray,
            SuspensionFill: new Color((byte)180, (byte)180, (byte)180),
            ShadowStroke: theme.ShadowColor,
            SafeZoneFill: new Color((byte)theme.SafeZoneColor.R, (byte)theme.SafeZoneColor.G, (byte)theme.SafeZoneColor.B, (byte)50)
        );

        public void Render(CancellationToken cancellationToken = default) =>
            Log("Renderer", "Rendering frame", () => {
                if (cancellationToken.IsCancellationRequested) return;

                _terrainRenderer.RenderTerrain(cancellationToken);
                if (cancellationToken.IsCancellationRequested) return;

                UpdateMotorcycleVisual(cancellationToken);
            });

        private void UpdateMotorcycleVisual(CancellationToken cancellationToken)
        {
            var motorcycle = _gameController.Motorcycle;
            if (motorcycle is null) return;

            var (skeletonPoints, skeletonLines) = motorcycle.GetSkeleton();
            _skeletonRenderer.UpdateSkeletonVisuals(skeletonPoints, skeletonLines);
            _shadowRenderer.UpdateShadow(skeletonPoints, skeletonLines, cancellationToken);
        }

        public record ColorSet(
            Dictionary<SkeletonLineType, Color> LineTypeColors,
            Color WheelFill,
            Color WheelStroke,
            Color SpokeStroke,
            Color? TerrainFill,
            Color TerrainStroke,
            Color SuspensionStroke,
            Color SuspensionFill,
            Color ShadowStroke,
            Color SafeZoneFill
        );

        #region TerrainRenderer
        public class TerrainRenderer
        {
            private readonly SpriteBatch _spriteBatch;
            private readonly Texture2D _pixelTexture;
            private readonly Camera _camera;
            private readonly GameController _gameController;
            private readonly Random _random;
            private ColorSet _colors;

            public TerrainRenderer(SpriteBatch spriteBatch, Texture2D pixelTexture, Camera camera,
                                 GameController gameController, ColorSet colors)
            {
                _spriteBatch = spriteBatch;
                _pixelTexture = pixelTexture;
                _camera = camera;
                _gameController = gameController;
                _colors = colors;
                _random = new Random();
            }

            public void Initialize() =>
                Info("TerrainRenderer", "Terrain renderer initialized");

            public void UpdateColors(ColorSet colors) =>
                _colors = colors;

            public void RenderTerrain(CancellationToken cancellationToken)
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

                RenderVerticalLines(terrainPoints, cancellationToken);
            }

            private void RenderTerrainSegment(Level.TerrainPoint point1, Level.TerrainPoint point2)
            {
                var topPoint1 = GetTopPointWithOffset(point1);
                var topPoint2 = GetTopPointWithOffset(point2);
                var bottomPoint1 = new Vector2((float)point1.X, (float)point1.YBottom);
                var bottomPoint2 = new Vector2((float)point2.X, (float)point2.YBottom);

                if (!PhysicsComponent.IsValidPoint(topPoint1) || !PhysicsComponent.IsValidPoint(topPoint2) ||
                    !PhysicsComponent.IsValidPoint(bottomPoint1) || !PhysicsComponent.IsValidPoint(bottomPoint2))
                    return;

                var fillColor = point1.IsSafeZone ? _colors.SafeZoneFill : _colors.TerrainFill;
                var strokeColor = point1.IsSafeZone ? ThemeManager.CurrentTheme.SafeZoneColor : _colors.TerrainStroke;

                RenderStructuredFill(topPoint1, topPoint2, bottomPoint1, bottomPoint2, fillColor);

                if (PhysicsComponent.IsValidPoint(topPoint1) && PhysicsComponent.IsValidPoint(topPoint2))
                {
                    DrawLine(
                        _spriteBatch,
                        _pixelTexture,
                        topPoint1,
                        topPoint2,
                        strokeColor,
                        TerrainStrokeThickness
                    );

                    RenderTerrainLighting(topPoint1, topPoint2, strokeColor);

                    if (point1.IsSafeZone)
                    {
                        RenderSafeZoneIndicators(topPoint1, topPoint2);
                    }
                    else
                    {
                        RenderSurfaceDetails(topPoint1, topPoint2, point1.IsSafeZone);
                    }
                }

                if (PhysicsComponent.IsValidPoint(bottomPoint1) && PhysicsComponent.IsValidPoint(bottomPoint2))
                {
                    DrawLine(
                        _spriteBatch,
                        _pixelTexture,
                        bottomPoint1,
                        bottomPoint2,
                        strokeColor,
                        TerrainStrokeThickness
                    );
                }
            }

            private void RenderStructuredFill(Vector2 topPoint1, Vector2 topPoint2, Vector2 bottomPoint1, Vector2 bottomPoint2, Color? fillColor)
            {
                if (fillColor == null) return;

                float minX = MathHelper.Min(topPoint1.X, topPoint2.X);
                float maxX = MathHelper.Max(topPoint1.X, topPoint2.X);

                float pixelStep = 5.0f;

                minX = (float)Math.Floor(minX / pixelStep) * pixelStep;

                for (float x = minX; x <= maxX; x += pixelStep)
                {
                    int seed = (int)(x * 1000);
                    Random rand = new Random(seed);

                    float t = (x - minX) / (maxX - minX);
                    if (float.IsNaN(t) || float.IsInfinity(t))
                        t = 0;

                    float topY = MathHelper.Lerp(topPoint1.Y, topPoint2.Y, t);
                    float bottomY = MathHelper.Lerp(bottomPoint1.Y, bottomPoint2.Y, t);

                    float height = bottomY - topY;
                    if (height <= 0) continue;

                    RenderTerrainColumn(x, topY, bottomY, height, fillColor.Value, rand);
                }
            }

            private void RenderTerrainColumn(float x, float topY, float bottomY, float height, Color baseColor, Random rand)
            {
                int layerCount = Math.Min(10, (int)(height / 5.0f));
                if (layerCount <= 0) return;

                for (int layer = 0; layer < layerCount; layer++)
                {
                    float layerDepth = (float)layer / layerCount;

                    int pixelsInLayer = 1 + (int)(layerDepth * 3);

                    for (int i = 0; i < pixelsInLayer; i++)
                    {
                        float layerStart = topY + height * layerDepth;
                        float layerEnd = layerStart + height / layerCount;
                        float pixelY = layerStart + (float)rand.NextDouble() * (layerEnd - layerStart);

                        if (pixelY < topY || pixelY > bottomY) continue;

                        int darkenAmount = (int)(layerDepth * 50);
                        byte r = (byte)Math.Max(0, baseColor.R - darkenAmount);
                        byte g = (byte)Math.Max(0, baseColor.G - darkenAmount);
                        byte b = (byte)Math.Max(0, baseColor.B - darkenAmount);

                        byte alpha = (byte)Math.Min(255, baseColor.A + (int)(layerDepth * 80));

                        Color pixelColor = new Color(r, g, b, alpha);

                        float pixelSize = 2.0f - layerDepth * 0.8f;

                        float offsetX = (float)(rand.NextDouble() - 0.5) * 2.0f;

                        _spriteBatch.Draw(
                            _pixelTexture,
                            new Vector2(x + offsetX, pixelY),
                            null,
                            pixelColor,
                            0f,
                            Vector2.Zero,
                            pixelSize,
                            SpriteEffects.None,
                            0f
                        );
                    }
                }
            }

            private void RenderSurfaceDetails(Vector2 topPoint1, Vector2 topPoint2, bool isSafeZone)
            {
                float length = Vector2.Distance(topPoint1, topPoint2);
                if (length < 10) return;

                int detailCount = (int)(length / 20);
                Vector2 direction = Vector2.Normalize(topPoint2 - topPoint1);

                for (int i = 0; i < detailCount; i++)
                {
                    float t = (float)i / detailCount;
                    Vector2 position = Vector2.Lerp(topPoint1, topPoint2, t);

                    int seed = (int)(position.X * 1000);
                    Random rand = new Random(seed);

                    if (rand.NextDouble() > 0.7)
                    {
                        float detailSize = 0.8f + (float)rand.NextDouble() * 1.2f;
                        Color detailColor = isSafeZone ?
                            new Color((byte)120, (byte)200, (byte)120, (byte)150) :
                            new Color((byte)100, (byte)70, (byte)40, (byte)150);

                        _spriteBatch.Draw(
                            _pixelTexture,
                            position,
                            null,
                            detailColor,
                            0f,
                            Vector2.Zero,
                            detailSize,
                            SpriteEffects.None,
                            0f
                        );
                    }
                }
            }

            private void RenderTerrainLighting(Vector2 topPoint1, Vector2 topPoint2, Color lightColor)
            {
                Vector2 direction = topPoint2 - topPoint1;
                float length = direction.Length();
                if (length <= 0) return;

                direction /= length;
                Vector2 normal = new Vector2(-direction.Y, direction.X);

                Vector2 lightDir = Vector2.Normalize(new Vector2(-0.5f, -1.0f));

                float lightIntensity = Math.Max(0, Vector2.Dot(normal, -lightDir));

                Color highlightColor = new Color(
                    (byte)Math.Min(255, lightColor.R + 50),
                    (byte)Math.Min(255, lightColor.G + 50),
                    (byte)Math.Min(255, lightColor.B + 50),
                    (byte)100
                );

                if (lightIntensity > 0.5f)
                {
                    DrawLine(
                        _spriteBatch,
                        _pixelTexture,
                        topPoint1,
                        topPoint2,
                        highlightColor,
                        2
                    );
                }
            }

            private void RenderSafeZoneIndicators(Vector2 topPoint1, Vector2 topPoint2)
            {
                float length = Vector2.Distance(topPoint1, topPoint2);
                if (length < 20) return;

                int markerCount = (int)(length / 30);

                for (int i = 0; i < markerCount; i++)
                {
                    float t = ((float)i + 0.5f) / markerCount;
                    Vector2 position = Vector2.Lerp(topPoint1, topPoint2, t);

                    _spriteBatch.Draw(
                        _pixelTexture,
                        position,
                        null,
                        new Color((byte)0, (byte)255, (byte)0, (byte)150),
                        0f,
                        Vector2.Zero,
                        3.0f,
                        SpriteEffects.None,
                        0f
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

            private void RenderVerticalLines(List<Level.TerrainPoint> terrainPoints, CancellationToken cancellationToken)
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
        }
        #endregion

        #region ShadowRenderer
        public class ShadowRenderer
        {
            private readonly SpriteBatch _spriteBatch;
            private readonly Texture2D _pixelTexture;
            private readonly Camera _camera;
            private readonly GameController _gameController;
            private readonly Random _random;

            private const float ShadowOffsetFactor = 0.15f;
            private const float ShadowScaleFactor = 0.04f;
            private const int ShadowInterpolationSteps = 6;
            private const float ShadowPixelSize = 2.5f;

            public ShadowRenderer(SpriteBatch spriteBatch, Texture2D pixelTexture, Camera camera, GameController gameController)
            {
                _spriteBatch = spriteBatch;
                _pixelTexture = pixelTexture;
                _camera = camera;
                _gameController = gameController;
                _random = new Random();
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

                    var wheelPoints = GetWheelPoints(skeletonPoints, skeletonLines);
                    foreach (var wheelPoint in wheelPoints)
                    {
                        DrawWheelShadow(wheelPoint);
                    }
                });

            private void DrawShadow(List<Vector2> shadowPoints)
            {
                if (shadowPoints.Count < 2) return;

                // Сначала рисуем более темную основную тень
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
                            new Color(
                                ThemeManager.CurrentTheme.ShadowColor.R,
                                ThemeManager.CurrentTheme.ShadowColor.G,
                                ThemeManager.CurrentTheme.ShadowColor.B,
                                (byte)(ThemeManager.CurrentTheme.ShadowColor.A * 0.6f)
                            ),
                            3.5f
                        );
                    }
                }

                // Затем добавляем пиксельные детали для ретро-эффекта
                for (int i = 0; i < shadowPoints.Count; i++)
                {
                    var point = shadowPoints[i];
                    int seed = (int)(point.X * 1000 + point.Y * 10);
                    Random rand = new Random(seed);

                    // Основной пиксель тени
                    _spriteBatch.Draw(
                        _pixelTexture,
                        point,
                        null,
                        new Color(
                            ThemeManager.CurrentTheme.ShadowColor.R,
                            ThemeManager.CurrentTheme.ShadowColor.G,
                            ThemeManager.CurrentTheme.ShadowColor.B,
                            (byte)(ThemeManager.CurrentTheme.ShadowColor.A * 0.8f)
                        ),
                        0f,
                        Vector2.Zero,
                        ShadowPixelSize,
                        SpriteEffects.None,
                        0f
                    );

                    // Создаем 3D-эффект добавляя пиксели с разной прозрачностью
                    int pixelCount = 3 + rand.Next(3);
                    for (int j = 0; j < pixelCount; j++)
                    {
                        float distance = 1.0f + j * 1.5f;
                        float angle = (float)(rand.NextDouble() * Math.PI * 2);
                        float offsetX = (float)Math.Cos(angle) * distance;
                        float offsetY = (float)Math.Sin(angle) * distance;

                        float alpha = 0.6f - (j * 0.15f);

                        _spriteBatch.Draw(
                            _pixelTexture,
                            new Vector2(point.X + offsetX, point.Y + offsetY),
                            null,
                            new Color(
                                ThemeManager.CurrentTheme.ShadowColor.R,
                                ThemeManager.CurrentTheme.ShadowColor.G,
                                ThemeManager.CurrentTheme.ShadowColor.B,
                                (byte)(ThemeManager.CurrentTheme.ShadowColor.A * alpha)
                            ),
                            0f,
                            Vector2.Zero,
                            ShadowPixelSize - j * 0.4f,
                            SpriteEffects.None,
                            0f
                        );
                    }

                    // Добавляем случайные точки для создания эффекта шума
                    if (rand.NextDouble() > 0.7f)
                    {
                        for (int k = 0; k < 2; k++)
                        {
                            float noiseOffsetX = (float)(rand.NextDouble() - 0.5) * 8.0f;
                            float noiseOffsetY = (float)(rand.NextDouble() - 0.5) * 8.0f;
                            float noiseSize = 1.0f + (float)rand.NextDouble() * 1.0f;

                            _spriteBatch.Draw(
                                _pixelTexture,
                                new Vector2(point.X + noiseOffsetX, point.Y + noiseOffsetY),
                                null,
                                new Color(
                                    ThemeManager.CurrentTheme.ShadowColor.R,
                                    ThemeManager.CurrentTheme.ShadowColor.G,
                                    ThemeManager.CurrentTheme.ShadowColor.B,
                                    (byte)(ThemeManager.CurrentTheme.ShadowColor.A * 0.2f)
                                ),
                                0f,
                                Vector2.Zero,
                                noiseSize,
                                SpriteEffects.None,
                                0f
                            );
                        }
                    }
                }
            }

            private void DrawWheelShadow(Vector2 wheelPosition)
            {
                var shadowPoint = CalculateShadowPoint(wheelPosition);
                if (!PhysicsComponent.IsValidPoint(shadowPoint)) return;

                // Основная тень колеса
                _spriteBatch.Draw(
                    _pixelTexture,
                    shadowPoint,
                    null,
                    new Color(
                        ThemeManager.CurrentTheme.ShadowColor.R,
                        ThemeManager.CurrentTheme.ShadowColor.G,
                        ThemeManager.CurrentTheme.ShadowColor.B,
                        (byte)(ThemeManager.CurrentTheme.ShadowColor.A * 0.8f)
                    ),
                    0f,
                    Vector2.Zero,
                    ShadowPixelSize * 2.0f,
                    SpriteEffects.None,
                    0f
                );

                int seed = (int)(shadowPoint.X * 1000);
                Random rand = new Random(seed);

                // Создаем эффект элипса для имитации 3D-тени
                int pixelCount = 8;
                for (int i = 0; i < pixelCount; i++)
                {
                    float angle = ((float)i / pixelCount) * MathHelper.TwoPi;
                    float radiusX = 4.0f + (float)rand.NextDouble() * 1.0f;
                    float radiusY = 2.0f + (float)rand.NextDouble() * 0.5f;

                    float offsetX = (float)Math.Cos(angle) * radiusX;
                    float offsetY = (float)Math.Sin(angle) * radiusY;

                    float distance = Vector2.Distance(Vector2.Zero, new Vector2(offsetX, offsetY)) / 5.0f;
                    float alpha = 0.7f - distance;

                    if (alpha <= 0) continue;

                    _spriteBatch.Draw(
                        _pixelTexture,
                        new Vector2(shadowPoint.X + offsetX, shadowPoint.Y + offsetY),
                        null,
                        new Color(
                            ThemeManager.CurrentTheme.ShadowColor.R,
                            ThemeManager.CurrentTheme.ShadowColor.G,
                            ThemeManager.CurrentTheme.ShadowColor.B,
                            (byte)(ThemeManager.CurrentTheme.ShadowColor.A * alpha)
                        ),
                        0f,
                        Vector2.Zero,
                        ShadowPixelSize * 1.2f,
                        SpriteEffects.None,
                        0f
                    );
                }

                // Добавляем пиксельные детали
                for (int i = 0; i < 4; i++)
                {
                    float offsetX = (float)(rand.NextDouble() - 0.5) * 6.0f;
                    float offsetY = (float)(rand.NextDouble() - 0.5) * 6.0f;
                    float size = ShadowPixelSize * 0.8f + (float)rand.NextDouble() * 0.6f;

                    _spriteBatch.Draw(
                        _pixelTexture,
                        new Vector2(shadowPoint.X + offsetX, shadowPoint.Y + offsetY),
                        null,
                        new Color(
                            ThemeManager.CurrentTheme.ShadowColor.R,
                            ThemeManager.CurrentTheme.ShadowColor.G,
                            ThemeManager.CurrentTheme.ShadowColor.B,
                            (byte)(ThemeManager.CurrentTheme.ShadowColor.A * 0.4f)
                        ),
                        0f,
                        Vector2.Zero,
                        size,
                        SpriteEffects.None,
                        0f
                    );
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
                var framePointIndices = skeletonLines
                    .Where(l => l.Type == SkeletonLineType.MainFrame)
                    .SelectMany(l => new[] { l.StartPointIndex, l.EndPointIndex })
                    .Where(idx => idx >= 0 && idx < skeletonPoints.Count)
                    .ToHashSet();

                return framePointIndices
                    .Select(idx => skeletonPoints[idx].Position)
                    .Where(PhysicsComponent.IsValidPoint)
                    .OrderBy(p => p.X)
                    .ToList();
            }

            private List<Vector2> GetWheelPoints(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines)
            {
                var wheelCenters = new List<Vector2>();
                var wheelLines = skeletonLines
                    .Where(l => l.Type == SkeletonLineType.Wheel)
                    .ToList();

                var wheelPointIndices = wheelLines
                    .SelectMany(l => new[] { l.StartPointIndex, l.EndPointIndex })
                    .Distinct()
                    .Where(idx => idx >= 0 && idx < skeletonPoints.Count)
                    .ToList();

                if (wheelPointIndices.Count < 3) return wheelCenters;

                var frontWheelIndices = wheelPointIndices.Take(wheelPointIndices.Count / 2).ToList();
                var rearWheelIndices = wheelPointIndices.Skip(wheelPointIndices.Count / 2).ToList();

                if (frontWheelIndices.Count > 0)
                {
                    Vector2 frontCenter = new Vector2(0, 0);
                    foreach (var idx in frontWheelIndices)
                        frontCenter += skeletonPoints[idx].Position;
                    frontCenter /= frontWheelIndices.Count;
                    wheelCenters.Add(frontCenter);
                }

                if (rearWheelIndices.Count > 0)
                {
                    Vector2 rearCenter = new Vector2(0, 0);
                    foreach (var idx in rearWheelIndices)
                        rearCenter += skeletonPoints[idx].Position;
                    rearCenter /= rearWheelIndices.Count;
                    wheelCenters.Add(rearCenter);
                }

                return wheelCenters;
            }

            private (float centerX, float centerY, float scale) CalculateShadowParameters(List<Vector2> framePoints)
            {
                if (framePoints.Count == 0)
                    return (0, 0, 0);

                float sumX = 0, sumY = 0;
                foreach (var point in framePoints)
                {
                    sumX += point.X;
                    sumY += point.Y;
                }

                float centerX = sumX / framePoints.Count;
                float centerY = sumY / framePoints.Count;

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