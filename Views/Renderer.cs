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
using static GravityDefiedGame.Views.Renderer.RenderConstants;

namespace GravityDefiedGame.Views
{
    public class Renderer
    {
        public static class RenderConstants
        {
            public const double
                TerrainStrokeThickness = 3.0,
                VerticalLineStrokeThickness = 0.5,
                BikeStrokeThickness = 3.0,
                SpokeStrokeThickness = 1.0,
                SuspensionStrokeThickness = 2.5,
                SuspensionSpringStrokeThickness = 1.5,
                MovementSpeedThreshold = 15.0,
                MovementBlurOpacity = 0.8,
                NormalOpacity = 1.0,
                TopLineOffsetFactor = 0.25,
                ShadowBlurRadius = 5.0,
                ShadowOpacity = 0.5,
                ShadowOffsetFactor = 0.2,
                ShadowScaleFactor = 0.01;

            public const int
                InitialSkeletonLinesCount = 30,
                SpokeCount = 6,
                ShadowInterpolationSteps = 5;
        }

        private readonly Canvas _canvas;
        private readonly Camera _camera;
        private readonly GameController _gameController;
        private Path _terrainPath = null!, _verticalLinesPath = null!, _shadowPath = null!;
        private readonly SkeletonRenderer _skeletonRenderer;
        private readonly ShadowRenderer _shadowRenderer;

        private readonly BrushSet Brushes = new(
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
            TerrainStroke: new(Colors.Green),
            SuspensionStroke: new(Colors.DarkGray),
            SuspensionFill: new(Color.FromRgb(180, 180, 180)),
            ShadowStroke: new(Color.FromArgb((byte)(255 * ShadowOpacity), 0, 0, 0))
        );

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

        public Renderer(Canvas canvas, GameController gameController, Camera camera)
        {
            _canvas = canvas;
            _gameController = gameController;
            _camera = camera;
            _skeletonRenderer = new SkeletonRenderer(_canvas, _camera, Brushes.LineTypeBrushes);
            _shadowRenderer = new ShadowRenderer(_canvas, _camera, _gameController, ShadowOpacity, ShadowBlurRadius,
                BikeStrokeThickness, ShadowOffsetFactor, ShadowScaleFactor, ShadowInterpolationSteps);
            InitializeVisuals();
        }

        private void InitializeVisuals()
        {
            InitializeTerrain();
            _shadowRenderer.Initialize();
            _skeletonRenderer.InitializeSkeletonLines();
        }

        private void InitializeTerrain()
        {
            _terrainPath = new Path
            {
                Stroke = Brushes.TerrainStroke,
                StrokeThickness = TerrainStrokeThickness,
                Fill = Brushes.TerrainFill
            };
            _canvas.Children.Add(_terrainPath);

            _verticalLinesPath = new Path
            {
                Stroke = new SolidColorBrush(Colors.Green),
                StrokeThickness = VerticalLineStrokeThickness
            };
            _canvas.Children.Add(_verticalLinesPath);
        }

        public void Render(CancellationToken cancellationToken = default)
        {
            if (cancellationToken.IsCancellationRequested) return;
            UpdateTerrainVisual(cancellationToken);
            if (cancellationToken.IsCancellationRequested) return;
            UpdateMotorcycleVisual(cancellationToken);
        }

        private void UpdateTerrainVisual(CancellationToken cancellationToken)
        {
            var terrainPoints = _gameController.CurrentLevel?.TerrainPoints ?? new List<Level.TerrainPoint>();
            if (terrainPoints.Count == 0) return;

            var compositeGeometry = new PathGeometry();
            var compositeFigure = new PathFigure();
            var startPoint = GetTopPointWithOffset(terrainPoints[0]);
            if (!IsValidPoint(startPoint)) startPoint = new Point(0, 0);
            compositeFigure.StartPoint = startPoint;

            foreach (var point in terrainPoints.Skip(1))
            {
                if (cancellationToken.IsCancellationRequested) return;
                var nextPoint = GetTopPointWithOffset(point);
                if (IsValidPoint(nextPoint))
                    compositeFigure.Segments.Add(new LineSegment(nextPoint, true));
            }

            foreach (var point in terrainPoints.AsEnumerable().Reverse())
            {
                if (cancellationToken.IsCancellationRequested) return;
                var bottomPoint = _camera.WorldToScreen(new Point(point.X, point.YBottom));
                if (IsValidPoint(bottomPoint))
                    compositeFigure.Segments.Add(new LineSegment(bottomPoint, true));
            }

            compositeFigure.IsClosed = true;
            compositeGeometry.Figures.Add(compositeFigure);
            _terrainPath.Data = compositeGeometry;

            UpdateVerticalLines(terrainPoints, cancellationToken);
        }

        private bool IsValidPoint(Point point) =>
            double.IsFinite(point.X) && double.IsFinite(point.Y);

        private Point GetTopPointWithOffset(Level.TerrainPoint point)
        {
            var result = _camera.WorldToScreen(new Point(point.X - (point.YTop - point.YBottom) * TopLineOffsetFactor, point.YTop));
            if (!IsValidPoint(result))
                Console.WriteLine($"Invalid point at X={point.X}: {result.X}, {result.Y}");
            return result;
        }

        private void UpdateVerticalLines(List<Level.TerrainPoint> terrainPoints, CancellationToken cancellationToken)
        {
            var verticalGeometry = new PathGeometry();

            foreach (var point in terrainPoints)
            {
                if (cancellationToken.IsCancellationRequested) return;
                var topPoint = GetTopPointWithOffset(point);
                var bottomPoint = _camera.WorldToScreen(new Point(point.X, point.YBottom));
                if (!IsValidPoint(topPoint) || !IsValidPoint(bottomPoint)) continue;

                var verticalFigure = new PathFigure
                {
                    StartPoint = topPoint,
                    IsClosed = false
                };
                verticalFigure.Segments.Add(new LineSegment(bottomPoint, true));
                verticalGeometry.Figures.Add(verticalFigure);
            }

            _verticalLinesPath.Data = verticalGeometry;
        }

        private void UpdateMotorcycleVisual(CancellationToken cancellationToken)
        {
            var motorcycle = _gameController.Motorcycle;
            if (motorcycle is null) return;

            var (skeletonPoints, skeletonLines) = motorcycle.GetSkeleton();
            _skeletonRenderer.UpdateSkeletonVisuals(skeletonPoints, skeletonLines);
            _shadowRenderer.UpdateShadow(skeletonPoints, skeletonLines, cancellationToken);
        }

        public class ShadowRenderer
        {
            private readonly Canvas _canvas;
            private readonly Camera _camera;
            private readonly GameController _gameController;
            private Path _shadowPath;
            private readonly double _shadowOpacity;
            private readonly double _shadowBlurRadius;
            private readonly double _bikeStrokeThickness;
            private readonly double _shadowOffsetFactor;
            private readonly double _shadowScaleFactor;
            private readonly int _shadowInterpolationSteps;

            public ShadowRenderer(Canvas canvas, Camera camera, GameController gameController,
                double shadowOpacity, double shadowBlurRadius, double bikeStrokeThickness,
                double shadowOffsetFactor, double shadowScaleFactor, int shadowInterpolationSteps)
            {
                _canvas = canvas;
                _camera = camera;
                _gameController = gameController;
                _shadowOpacity = shadowOpacity;
                _shadowBlurRadius = shadowBlurRadius;
                _bikeStrokeThickness = bikeStrokeThickness;
                _shadowOffsetFactor = shadowOffsetFactor;
                _shadowScaleFactor = shadowScaleFactor;
                _shadowInterpolationSteps = shadowInterpolationSteps;
            }

            public void Initialize()
            {
                _shadowPath = new Path
                {
                    Stroke = new SolidColorBrush(Color.FromArgb((byte)(255 * _shadowOpacity), 0, 0, 0)),
                    StrokeThickness = _bikeStrokeThickness,
                    StrokeLineJoin = PenLineJoin.Round,
                    StrokeEndLineCap = PenLineCap.Round,
                    StrokeStartLineCap = PenLineCap.Round,
                    Effect = new System.Windows.Media.Effects.BlurEffect
                    {
                        Radius = _shadowBlurRadius,
                        KernelType = System.Windows.Media.Effects.KernelType.Gaussian
                    }
                };
                _canvas.Children.Add(_shadowPath);
            }

            public void UpdateShadow(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines, CancellationToken cancellationToken)
            {
                if (_gameController.CurrentLevel is null) return;

                var framePoints = GetValidFramePoints(skeletonPoints, skeletonLines);
                if (framePoints.Count < 2) return;

                var (centerX, centerY, scale) = CalculateShadowParameters(framePoints);
                if (scale == 0) return;

                var shadowPoints = GenerateShadowPoints(framePoints, centerX, centerY, scale, cancellationToken);
                if (shadowPoints.Count < 2) return;

                var shadowFigure = CreateShadowFigure(shadowPoints);
                if (shadowFigure == null) return;

                var shadowGeometry = new PathGeometry();
                shadowGeometry.Figures.Add(shadowFigure);
                _shadowPath.Data = shadowGeometry;
            }

            private bool IsValidPoint(Point point) =>
                double.IsFinite(point.X) && double.IsFinite(point.Y);

            private Point CalculateShadowPoint(Point framePoint)
            {
                if (!IsValidPoint(framePoint) || _gameController.CurrentLevel is null)
                    return framePoint;

                try
                {
                    double groundY = _gameController.CurrentLevel.GetGroundYAtX(framePoint.X);
                    if (!double.IsFinite(groundY)) return framePoint;

                    double heightAboveGround = groundY - framePoint.Y;
                    if (heightAboveGround <= 0 || !double.IsFinite(heightAboveGround)) return framePoint;

                    double shadowX = framePoint.X + heightAboveGround * _shadowOffsetFactor;
                    if (!double.IsFinite(shadowX)) return framePoint;

                    double shadowGroundY = _gameController.CurrentLevel.GetGroundYAtX(shadowX);
                    return double.IsFinite(shadowGroundY) ? new Point(shadowX, shadowGroundY) : framePoint;
                }
                catch { return framePoint; }
            }

            private List<SkeletonPoint> GetValidFramePoints(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines)
            {
                var mainFrameLines = skeletonLines.Where(l => l.Type == SkeletonLineType.MainFrame).ToList();
                var framePointsIndices = new HashSet<int>(mainFrameLines.SelectMany(line => new[] { line.StartPointIndex, line.EndPointIndex }));
                return framePointsIndices
                    .Select(idx => skeletonPoints[idx])
                    .OrderBy(p => p.Position.X)
                    .Where(p => IsValidPoint(p.Position))
                    .ToList();
            }

            private (double centerX, double centerY, double scale) CalculateShadowParameters(List<SkeletonPoint> framePoints)
            {
                if (_gameController.CurrentLevel is null || framePoints.Count < 2) return (0, 0, 1);

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
                    catch { continue; }
                }

                if (validPointsCount == 0) return (0, 0, 1);

                double averageHeight = totalHeight / validPointsCount;
                double scale = 1 / Math.Max(1 + _shadowScaleFactor * averageHeight, 0.1);
                double minX = framePoints.Min(p => p.Position.X);
                double maxX = framePoints.Max(p => p.Position.X);
                double centerX = (minX + maxX) / 2;

                try
                {
                    double centerY = _gameController.CurrentLevel.GetGroundYAtX(centerX);
                    if (double.IsFinite(centerY)) return (centerX, centerY, scale);
                }
                catch { }

                return (0, 0, 0);
            }

            private List<Point> GenerateShadowPoints(
                List<SkeletonPoint> framePoints,
                double centerX,
                double centerY,
                double scale,
                CancellationToken cancellationToken)
            {
                var shadowPoints = new List<Point>();
                if (!double.IsFinite(centerX) || !double.IsFinite(centerY) || !double.IsFinite(scale)) return shadowPoints;

                for (int i = 0; i < framePoints.Count - 1; i++)
                {
                    if (cancellationToken.IsCancellationRequested) return shadowPoints;
                    var start = framePoints[i].Position;
                    var end = framePoints[i + 1].Position;
                    if (!IsValidPoint(start) || !IsValidPoint(end)) continue;

                    for (int step = 0; step <= _shadowInterpolationSteps; step++)
                    {
                        if (cancellationToken.IsCancellationRequested) return shadowPoints;
                        double t = (double)step / _shadowInterpolationSteps;
                        var interpolatedPoint = new Point(
                            start.X + t * (end.X - start.X),
                            start.Y + t * (end.Y - start.Y)
                        );
                        if (!IsValidPoint(interpolatedPoint)) continue;

                        var shadowPoint = CalculateShadowPoint(interpolatedPoint);
                        if (!IsValidPoint(shadowPoint)) continue;

                        double dx = shadowPoint.X - centerX;
                        double dy = shadowPoint.Y - centerY;
                        shadowPoint = new Point(centerX + scale * dx, centerY + scale * dy);
                        if (IsValidPoint(shadowPoint)) shadowPoints.Add(shadowPoint);
                    }
                }
                return shadowPoints;
            }

            private PathFigure CreateShadowFigure(List<Point> shadowPoints)
            {
                if (shadowPoints.Count < 2) return null;

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
                    catch { }
                }

                return shadowFigure;
            }
        }

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
                Enum.GetValues<SkeletonLineType>().ToList().ForEach(lineType =>
                {
                    _skeletonLines[lineType] = new List<Line>();
                    CreateLinesForType(lineType);
                });

            private void CreateLinesForType(SkeletonLineType lineType) =>
                Enumerable.Range(0, InitialSkeletonLinesCount).ToList().ForEach(_ =>
                {
                    var line = new Line
                    {
                        Stroke = GetBrushForLineType(lineType),
                        StrokeThickness = BikeStrokeThickness,
                        Visibility = Visibility.Collapsed
                    };
                    _canvas.Children.Add(line);
                    _skeletonLines[lineType].Add(line);
                });

            public void UpdateSkeletonVisuals(List<SkeletonPoint> skeletonPoints, List<SkeletonLine> skeletonLines)
            {
                HideAllSkeletonLines();
                UpdateSkeletonLines(skeletonPoints, skeletonLines);
            }

            private void HideAllSkeletonLines() =>
                _skeletonLines.Values.SelectMany(lines => lines).ToList().ForEach(line => line.Visibility = Visibility.Collapsed);

            private void UpdateSkeletonLines(List<SkeletonPoint> points, List<SkeletonLine> lines)
            {
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
            }

            private void EnsureLineExists(SkeletonLineType lineType, int index)
            {
                if (index < _skeletonLines[lineType].Count) return;
                var line = new Line
                {
                    Stroke = GetBrushForLineType(lineType),
                    StrokeThickness = BikeStrokeThickness
                };
                _canvas.Children.Add(line);
                _skeletonLines[lineType].Add(line);
            }

            private void UpdateSkeletonLine(SkeletonLineType lineType, int lineIndex, SkeletonLine skeletonLine, List<SkeletonPoint> points)
            {
                var line = _skeletonLines[lineType][lineIndex];
                var startPoint = _camera.WorldToScreen(points[skeletonLine.StartPointIndex].Position);
                var endPoint = _camera.WorldToScreen(points[skeletonLine.EndPointIndex].Position);
                (line.X1, line.Y1, line.X2, line.Y2) = (startPoint.X, startPoint.Y, endPoint.X, endPoint.Y);
                line.Visibility = Visibility.Visible;
            }

            private SolidColorBrush GetBrushForLineType(SkeletonLineType lineType) =>
                _lineTypeBrushes.TryGetValue(lineType, out var brush) ? brush : _lineTypeBrushes[SkeletonLineType.MainFrame];
        }
    }
}