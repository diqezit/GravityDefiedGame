using GravityDefiedGame.Controllers;
using GravityDefiedGame.Models;
using GravityDefiedGame.Utilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using static GravityDefiedGame.Models.BikeGeom;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame.Views
{
    // Класс для хранения цветов уровней 
    public static class ThemeConstants
    {
        public static readonly Color BackgroundColor = Color.FromRgb(255, 204, 102);
        public static readonly Color TerrainColor = Color.FromRgb(204, 153, 102);
        public static readonly Color SafeZoneColor = Color.FromRgb(152, 251, 152);
        public static readonly Color VerticalLineColor = Color.FromRgb(204, 153, 102); // цвет для вертикальных линий
    }

    public class Renderer
    {
        #region Constants
        private const double
            Perspective = 0.2,
            TerrainStrokeThickness = 3.0,
            VerticalLineStrokeThickness = 0.5,
            BikeStrokeThickness = 3.0,
            ShadowBlurRadius = 5.0,
            ShadowOpacity = 0.5,
            ShadowOffsetFactor = 0.2,
            ShadowScaleFactor = 0.01;

        private const int
            InitialSkeletonLinesCount = 30,
            ShadowInterpolationSteps = 5;
        #endregion

        private readonly Canvas _canvas;
        private readonly Camera _camera;
        private readonly GameController _gameController;
        private Path? _terrainPath, _verticalLinesPath;
        private readonly SkeletonRenderer _skeletonRenderer;
        private readonly ShadowRenderer _shadowRenderer;
        private readonly BrushSet _brushes;

        public Renderer(Canvas canvas, GameController gameController, Camera camera)
        {
            _canvas = canvas;
            _gameController = gameController;
            _camera = camera;
            _brushes = CreateBrushSet();
            _skeletonRenderer = new SkeletonRenderer(_canvas, _camera, _brushes.LineTypeBrushes);
            _shadowRenderer = new ShadowRenderer(_canvas, _camera, _gameController);

            Log("Renderer", "Initializing renderer", () => {
                InitializeVisuals();
                Info("Renderer", "Renderer initialized successfully");
            });
        }

        private BrushSet CreateBrushSet() => new(
            LineTypeBrushes: new()
            {
                [SkeletonLineType.MainFrame] = new(Color.FromRgb(150, 150, 150)),
                [SkeletonLineType.Suspension] = new(Colors.Black),
                [SkeletonLineType.Wheel] = new(Colors.Black),
                [SkeletonLineType.Seat] = new(Color.FromRgb(200, 200, 200)),
                [SkeletonLineType.Handlebar] = new(Color.FromRgb(150, 150, 150)),
                [SkeletonLineType.Exhaust] = new(Color.FromRgb(100, 100, 100))
            },
            WheelFill: new(Color.FromRgb(200, 200, 200)),
            WheelStroke: new(Colors.Black),
            SpokeStroke: new(Colors.Black),
            TerrainFill: null,
            TerrainStroke: new(ThemeConstants.TerrainColor),
            SuspensionStroke: new(Colors.DarkGray),
            SuspensionFill: new(Color.FromRgb(180, 180, 180)),
            ShadowStroke: new(Color.FromArgb((byte)(255 * ShadowOpacity), 0, 0, 0))
        );

        private void InitializeVisuals() =>
            Log("Renderer", "Initializing visuals", () => {
                InitializeTerrain();
                _shadowRenderer.Initialize();
                _skeletonRenderer.InitializeSkeletonLines();
                Debug("Renderer", "Visuals initialized");
            });

        private void InitializeTerrain() =>
            Log("Renderer", "Initializing terrain", () => {
                _terrainPath = new Path
                {
                    Stroke = _brushes.TerrainStroke,
                    StrokeThickness = TerrainStrokeThickness,
                    Fill = _brushes.TerrainFill
                };
                _canvas.Children.Add(_terrainPath);

                _verticalLinesPath = new Path
                {
                    Stroke = new SolidColorBrush(Colors.Green),
                    StrokeThickness = VerticalLineStrokeThickness
                };
                _canvas.Children.Add(_verticalLinesPath);
                Debug("Renderer", "Terrain paths added to canvas");
            });

        public void Render(CancellationToken cancellationToken = default) =>
            Log("Renderer", "Rendering frame", () => {
                if (cancellationToken.IsCancellationRequested)
                {
                    Warning("Renderer", "Render cancelled due to cancellation token");
                    return;
                }

                UpdateTerrainVisual(cancellationToken);

                if (cancellationToken.IsCancellationRequested)
                {
                    Warning("Renderer", "Render cancelled after terrain update");
                    return;
                }

                UpdateMotorcycleVisual(cancellationToken);
            });

        private void UpdateTerrainVisual(CancellationToken cancellationToken) =>
            Log("Renderer", "Updating terrain visual", () => {
                var terrainPoints = _gameController.CurrentLevel?.TerrainPoints ?? new List<Level.TerrainPoint>();
                if (terrainPoints.Count == 0)
                {
                    Warning("Renderer", "No terrain points available");
                    return;
                }

                var compositeGeometry = new PathGeometry();
                var compositeFigure = new PathFigure();
                var startPoint = GetTopPointWithOffset(terrainPoints[0]);
                if (!IsValidPoint(startPoint))
                {
                    Warning("Renderer", "Invalid start point, defaulting to (0, 0)");
                    startPoint = new Point(0, 0);
                }

                compositeFigure.StartPoint = startPoint;

                foreach (var point in terrainPoints.Skip(1))
                {
                    if (cancellationToken.IsCancellationRequested)
                    {
                        Warning("Renderer", "Terrain update cancelled");
                        return;
                    }

                    var nextPoint = GetTopPointWithOffset(point);
                    if (IsValidPoint(nextPoint))
                        compositeFigure.Segments.Add(new LineSegment(nextPoint, true));
                }

                foreach (var point in terrainPoints.AsEnumerable().Reverse())
                {
                    if (cancellationToken.IsCancellationRequested)
                    {
                        Warning("Renderer", "Terrain update cancelled during bottom points");
                        return;
                    }

                    var bottomPoint = _camera.WorldToScreen(new Point(point.X, point.YBottom));
                    if (IsValidPoint(bottomPoint))
                        compositeFigure.Segments.Add(new LineSegment(bottomPoint, true));
                }

                compositeFigure.IsClosed = true;
                compositeGeometry.Figures.Add(compositeFigure);
                if (_terrainPath != null)
                    _terrainPath.Data = compositeGeometry;

                UpdateVerticalLines(terrainPoints, cancellationToken);
            });

        private bool IsValidPoint(Point point) =>
            Log("Renderer", $"Checking point validity: ({point.X:F1}, {point.Y:F1})", () =>
                double.IsFinite(point.X) && double.IsFinite(point.Y), false);

        private Point GetTopPointWithOffset(Level.TerrainPoint point) =>
            Log("Renderer", $"Getting top point for X={point.X:F1}", () => {
                double bikeX = _gameController.Motorcycle.Position.X;
                double dx = point.X - bikeX;
                double offsetX = -Perspective * dx;
                double worldX = point.X + offsetX;
                double worldY = point.YTop;
                var screenPoint = _camera.WorldToScreen(new Point(worldX, worldY));

                if (!IsValidPoint(screenPoint))
                    Debug("Renderer", $"Created potentially invalid point: ({screenPoint.X:F1}, {screenPoint.Y:F1})");

                return screenPoint;
            }, new Point());

        private void UpdateVerticalLines(List<Level.TerrainPoint> terrainPoints, CancellationToken cancellationToken) =>
            Log("Renderer", "Updating vertical lines", () => {
                var verticalGeometry = new PathGeometry();

                foreach (var point in terrainPoints)
                {
                    if (cancellationToken.IsCancellationRequested)
                    {
                        Warning("Renderer", "Vertical lines update cancelled");
                        return;
                    }

                    var topPoint = GetTopPointWithOffset(point);
                    var bottomPoint = _camera.WorldToScreen(new Point(point.X, point.YBottom));
                    if (!IsValidPoint(topPoint) || !IsValidPoint(bottomPoint))
                    {
                        Debug("Renderer", $"Skipping invalid points at X={point.X}");
                        continue;
                    }

                    var verticalFigure = new PathFigure
                    {
                        StartPoint = topPoint,
                        IsClosed = false
                    };
                    verticalFigure.Segments.Add(new LineSegment(bottomPoint, true));
                    verticalGeometry.Figures.Add(verticalFigure);
                }

                if (_verticalLinesPath != null)
                {
                    var currentLevel = _gameController.CurrentLevel;
                    if (currentLevel != null)
                    {
                        _verticalLinesPath.Stroke = new SolidColorBrush(currentLevel.VerticalLineColor);
                    }
                    _verticalLinesPath.Data = verticalGeometry;
                }
            });

        private void UpdateMotorcycleVisual(CancellationToken cancellationToken) =>
            Log("Renderer", "Updating motorcycle visual", () => {
                var motorcycle = _gameController.Motorcycle;
                if (motorcycle is null)
                {
                    Warning("Renderer", "Motorcycle is null");
                    return;
                }

                var (skeletonPoints, skeletonLines) = motorcycle.GetSkeleton();
                _skeletonRenderer.UpdateSkeletonVisuals(skeletonPoints, skeletonLines);
                _shadowRenderer.UpdateShadow(skeletonPoints, skeletonLines, cancellationToken);
            });

        private record BrushSet(
            Dictionary<SkeletonLineType, SolidColorBrush> LineTypeBrushes,
            SolidColorBrush WheelFill,
            SolidColorBrush WheelStroke,
            SolidColorBrush SpokeStroke,
            SolidColorBrush? TerrainFill,
            SolidColorBrush TerrainStroke,
            SolidColorBrush SuspensionStroke,
            SolidColorBrush SuspensionFill,
            SolidColorBrush ShadowStroke
        );

        #region ShadowRenderer
        public class ShadowRenderer
        {
            private readonly Canvas _canvas;
            private readonly Camera _camera;
            private readonly GameController _gameController;
            private Path? _shadowPath;

            public ShadowRenderer(Canvas canvas, Camera camera, GameController gameController)
            {
                _canvas = canvas;
                _camera = camera;
                _gameController = gameController;
            }

            public void Initialize() =>
                Log("ShadowRenderer", "Initializing shadow", () => {
                    _shadowPath = new Path
                    {
                        Stroke = new SolidColorBrush(Color.FromArgb((byte)(255 * ShadowOpacity), 0, 0, 0)),
                        StrokeThickness = BikeStrokeThickness,
                        StrokeLineJoin = PenLineJoin.Round,
                        StrokeEndLineCap = PenLineCap.Round,
                        StrokeStartLineCap = PenLineCap.Round,
                        Effect = new System.Windows.Media.Effects.BlurEffect
                        {
                            Radius = ShadowBlurRadius,
                            KernelType = System.Windows.Media.Effects.KernelType.Gaussian
                        }
                    };
                    _canvas.Children.Add(_shadowPath);
                    Info("ShadowRenderer", "Shadow path initialized");
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

                    var shadowFigure = CreateShadowFigure(shadowPoints);
                    if (shadowFigure == null)
                    {
                        Debug("ShadowRenderer", "Shadow figure is null");
                        return;
                    }

                    var shadowGeometry = new PathGeometry();
                    shadowGeometry.Figures.Add(shadowFigure);
                    if (_shadowPath != null)
                        _shadowPath.Data = shadowGeometry;
                });

            private bool IsValidPoint(Point point) =>
                double.IsFinite(point.X) && double.IsFinite(point.Y);

            private Point CalculateShadowPoint(Point framePoint) =>
                Log("ShadowRenderer", $"Calculating shadow point for ({framePoint.X:F1}, {framePoint.Y:F1})", () => {
                    if (!IsValidPoint(framePoint) || _gameController.CurrentLevel is null)
                    {
                        Debug("ShadowRenderer", "Invalid frame point or null level");
                        return framePoint;
                    }

                    try
                    {
                        double groundY = _gameController.CurrentLevel.GetGroundYAtX(framePoint.X);
                        if (!double.IsFinite(groundY))
                        {
                            Debug("ShadowRenderer", $"Invalid ground Y at X={framePoint.X:F1}");
                            return framePoint;
                        }

                        double heightAboveGround = groundY - framePoint.Y;
                        if (heightAboveGround <= 0 || !double.IsFinite(heightAboveGround))
                            return framePoint;

                        double shadowX = framePoint.X + heightAboveGround * ShadowOffsetFactor;
                        if (!double.IsFinite(shadowX))
                        {
                            Debug("ShadowRenderer", $"Invalid shadow X={shadowX:F1}");
                            return framePoint;
                        }

                        double shadowGroundY = _gameController.CurrentLevel.GetGroundYAtX(shadowX);
                        return double.IsFinite(shadowGroundY) ? new Point(shadowX, shadowGroundY) : framePoint;
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

            private (double centerX, double centerY, double scale) CalculateShadowParameters(List<SkeletonPoint> framePoints) =>
                Log("ShadowRenderer", "Calculating shadow parameters", () => {
                    if (_gameController.CurrentLevel is null || framePoints.Count < 2)
                    {
                        Debug("ShadowRenderer", "Invalid level or insufficient points");
                        return (0, 0, 1);
                    }

                    double totalHeight = 0;
                    int validPointsCount = 0;

                    foreach (var point in framePoints)
                    {
                        try
                        {
                            double groundY = _gameController.CurrentLevel.GetGroundYAtX(point.Position.X);
                            double heightAboveGround = groundY - point.Position.Y;
                            if (double.IsFinite(heightAboveGround))
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

                    double averageHeight = totalHeight / validPointsCount;
                    double scale = 1 / Math.Max(1 + ShadowScaleFactor * averageHeight, 0.1);
                    double minX = framePoints.Min(p => p.Position.X);
                    double maxX = framePoints.Max(p => p.Position.X);
                    double centerX = (minX + maxX) / 2;

                    try
                    {
                        double centerY = _gameController.CurrentLevel.GetGroundYAtX(centerX);
                        if (double.IsFinite(centerY))
                            return (centerX, centerY, scale);
                    }
                    catch (Exception ex)
                    {
                        Debug("ShadowRenderer", $"Error getting center Y: {ex.Message}");
                    }

                    return (0, 0, 0);
                }, (0, 0, 0));

            private List<Point> GenerateShadowPoints(
                List<SkeletonPoint> framePoints,
                double centerX,
                double centerY,
                double scale,
                CancellationToken cancellationToken) =>
                Log("ShadowRenderer", "Generating shadow points", () => {
                    var shadowPoints = new List<Point>();
                    if (!double.IsFinite(centerX) || !double.IsFinite(centerY) || !double.IsFinite(scale))
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

                            double t = (double)step / ShadowInterpolationSteps;
                            var interpolatedPoint = new Point(
                                start.X + t * (end.X - start.X),
                                start.Y + t * (end.Y - start.Y)
                            );
                            if (!IsValidPoint(interpolatedPoint))
                                continue;

                            var shadowPoint = CalculateShadowPoint(interpolatedPoint);
                            if (!IsValidPoint(shadowPoint))
                                continue;

                            double dx = shadowPoint.X - centerX;
                            double dy = shadowPoint.Y - centerY;
                            shadowPoint = new Point(centerX + scale * dx, centerY + scale * dy);
                            if (IsValidPoint(shadowPoint))
                                shadowPoints.Add(shadowPoint);
                        }
                    }
                    return shadowPoints;
                }, new List<Point>());

            private PathFigure? CreateShadowFigure(List<Point> shadowPoints) =>
                Log("ShadowRenderer", "Creating shadow figure", () => {
                    if (shadowPoints.Count < 2)
                    {
                        Debug("ShadowRenderer", "Not enough points for shadow figure");
                        return null;
                    }

                    var shadowFigure = new PathFigure
                    {
                        StartPoint = _camera.WorldToScreen(shadowPoints.First()),
                        IsClosed = false
                    };

                    foreach (var point in shadowPoints.Skip(1))
                    {
                        try
                        {
                            var screenPoint = _camera.WorldToScreen(point);
                            if (IsValidPoint(screenPoint))
                                shadowFigure.Segments.Add(new LineSegment(screenPoint, true));
                        }
                        catch (Exception ex)
                        {
                            Debug("ShadowRenderer", $"Error adding segment: {ex.Message}");
                        }
                    }

                    return shadowFigure;
                }, null);
        }
        #endregion

        #region SkeletonRenderer
        public class SkeletonRenderer
        {
            private readonly Canvas _canvas;
            private readonly Camera _camera;
            private readonly Dictionary<SkeletonLineType, List<Line>> _skeletonLines = new();
            private readonly Dictionary<SkeletonLineType, SolidColorBrush> _lineTypeBrushes;

            public SkeletonRenderer(Canvas canvas, Camera camera, Dictionary<SkeletonLineType, SolidColorBrush> lineTypeBrushes)
            {
                _canvas = canvas;
                _camera = camera;
                _lineTypeBrushes = lineTypeBrushes;
            }

            public void InitializeSkeletonLines() =>
                Log("SkeletonRenderer", "Initializing skeleton lines", () => {
                    foreach (var lineType in Enum.GetValues<SkeletonLineType>())
                    {
                        _skeletonLines[lineType] = new List<Line>();
                        CreateLinesForType(lineType);
                    }
                    Info("SkeletonRenderer", "Skeleton lines initialized");
                });

            private void CreateLinesForType(SkeletonLineType lineType) =>
                Log("SkeletonRenderer", $"Creating lines for type {lineType}", () => {
                    for (int i = 0; i < InitialSkeletonLinesCount; i++)
                    {
                        var line = new Line
                        {
                            Stroke = GetBrushForLineType(lineType),
                            StrokeThickness = BikeStrokeThickness,
                            Visibility = Visibility.Collapsed
                        };
                        _canvas.Children.Add(line);
                        _skeletonLines[lineType].Add(line);
                    }
                });

            public void UpdateSkeletonVisuals(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines) =>
                Log("SkeletonRenderer", "Updating skeleton visuals", () => {
                    HideAllSkeletonLines();
                    UpdateSkeletonLines(skeletonPoints, skeletonLines);
                });

            private void HideAllSkeletonLines() =>
                Log("SkeletonRenderer", "Hiding all skeleton lines", () => {
                    foreach (var line in _skeletonLines.Values.SelectMany(lines => lines))
                    {
                        line.Visibility = Visibility.Collapsed;
                    }
                });

            private void UpdateSkeletonLines(List<SkeletonPoint> points, List<SkeletonLine> lines) =>
                Log("SkeletonRenderer", "Updating skeleton lines", () => {
                    foreach (var lineType in _skeletonLines.Keys)
                    {
                        int lineIndex = 0;
                        foreach (var skeletonLine in lines.Where(l => l.Type == lineType))
                        {
                            EnsureLineExists(lineType, lineIndex);
                            UpdateSkeletonLine(lineType, lineIndex, skeletonLine, points);
                            lineIndex++;
                        }
                    }
                });

            private void EnsureLineExists(SkeletonLineType lineType, int index) =>
                Log("SkeletonRenderer", $"Ensuring line exists for type {lineType} at index {index}", () => {
                    if (index < _skeletonLines[lineType].Count)
                        return;

                    var line = new Line
                    {
                        Stroke = GetBrushForLineType(lineType),
                        StrokeThickness = BikeStrokeThickness
                    };
                    _canvas.Children.Add(line);
                    _skeletonLines[lineType].Add(line);
                });

            private void UpdateSkeletonLine(SkeletonLineType lineType, int lineIndex, SkeletonLine skeletonLine, List<SkeletonPoint> points) =>
                Log("SkeletonRenderer", $"Updating line type {lineType} at index {lineIndex}", () => {
                    var line = _skeletonLines[lineType][lineIndex];
                    var startPoint = _camera.WorldToScreen(points[skeletonLine.StartPointIndex].Position);
                    var endPoint = _camera.WorldToScreen(points[skeletonLine.EndPointIndex].Position);
                    (line.X1, line.Y1, line.X2, line.Y2) = (startPoint.X, startPoint.Y, endPoint.X, endPoint.Y);
                    line.Visibility = Visibility.Visible;
                });

            private SolidColorBrush GetBrushForLineType(SkeletonLineType lineType) =>
                _lineTypeBrushes.TryGetValue(lineType, out var brush) ? brush : _lineTypeBrushes[SkeletonLineType.MainFrame];
        }
        #endregion
    }
}