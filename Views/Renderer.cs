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

namespace GravityDefiedGame.Views;

public class Renderer
{
    // Локальные константы рендеринга
    private static class RenderConstants
    {
        // Толщина линий
        public const double
            TerrainStrokeThickness = 3.0,
            BikeStrokeThickness = 3.0,
            SpokeStrokeThickness = 1.0,
            SuspensionStrokeThickness = 2.5,
            SuspensionSpringStrokeThickness = 1.5;

        // Настройки отображения
        public const double
            MovementSpeedThreshold = 15.0,
            MovementBlurOpacity = 0.8,
            NormalOpacity = 1.0;

        public const int
            InitialSkeletonLinesCount = 30,
            SpokeCount = 6;
    }

    private readonly Canvas _canvas;
    private readonly Camera _camera;
    private readonly GameController _gameController;
    private Path _terrainPath = null!;
    private readonly SkeletonRenderer _skeletonRenderer;

    private readonly BrushSet Brushes = new(
        LineTypeBrushes: new()
        {
            [BikeGeom.SkeletonLineType.MainFrame] = new(Color.FromRgb(150, 150, 150)),
            [BikeGeom.SkeletonLineType.Suspension] = new(Colors.Black),
            [BikeGeom.SkeletonLineType.Wheel] = new(Colors.Black),
            [BikeGeom.SkeletonLineType.Seat] = new(Color.FromRgb(200, 200, 200)),
            [BikeGeom.SkeletonLineType.Handlebar] = new(Color.FromRgb(150, 150, 150)),
            [BikeGeom.SkeletonLineType.Exhaust] = new(Color.FromRgb(100, 100, 100))
        },
        WheelFill: new(Color.FromRgb(200, 200, 200)),
        WheelStroke: new(Colors.Black),
        SpokeStroke: new(Colors.Black),
        TerrainFill: new(Color.FromRgb(220, 220, 220)),
        TerrainStroke: new(Color.FromRgb(100, 100, 100)),
        SuspensionStroke: new(Colors.DarkGray),
        SuspensionFill: new(Color.FromRgb(180, 180, 180))
    );

    private record struct BrushSet(
        Dictionary<BikeGeom.SkeletonLineType, SolidColorBrush> LineTypeBrushes,
        SolidColorBrush WheelFill,
        SolidColorBrush WheelStroke,
        SolidColorBrush SpokeStroke,
        SolidColorBrush TerrainFill,
        SolidColorBrush TerrainStroke,
        SolidColorBrush SuspensionStroke,
        SolidColorBrush SuspensionFill
    );

    public Renderer(Canvas canvas, GameController gameController, Camera camera)
    {
        _canvas = canvas;
        _gameController = gameController;
        _camera = camera;
        _skeletonRenderer = new SkeletonRenderer(_canvas, _camera, Brushes.LineTypeBrushes);
        InitializeVisuals();
    }

    private void InitializeVisuals()
    {
        InitializeTerrain();
        _skeletonRenderer.InitializeSkeletonLines();
    }

    private void InitializeTerrain()
    {
        _terrainPath = new()
        {
            Stroke = Brushes.TerrainStroke,
            StrokeThickness = RenderConstants.TerrainStrokeThickness,
            Fill = Brushes.TerrainFill
        };
        _canvas.Children.Add(_terrainPath);
    }

    public void Render(CancellationToken cancellationToken = default)
    {
        if (cancellationToken.IsCancellationRequested)
            return;

        UpdateTerrainVisual();

        if (cancellationToken.IsCancellationRequested)
            return;

        UpdateMotorcycleVisual();
    }

    private void UpdateTerrainVisual()
    {
        var terrainPoints = _gameController.CurrentLevel?.TerrainPoints ?? new List<Point>();
        if (terrainPoints.Count == 0)
            return;

        _terrainPath.Data = new PathGeometry { Figures = { CreateTerrainPathFigure(terrainPoints) } };
    }

    private PathFigure CreateTerrainPathFigure(List<Point> terrainPoints)
    {
        var figure = new PathFigure
        {
            StartPoint = _camera.WorldToScreen(terrainPoints[0]),
            IsClosed = true
        };

        foreach (var point in terrainPoints.Skip(1))
        {
            figure.Segments.Add(new LineSegment(_camera.WorldToScreen(point), true));
        }

        figure.Segments.Add(new LineSegment(
            new Point(_camera.WorldToScreen(terrainPoints[^1]).X, _canvas.ActualHeight), true));
        figure.Segments.Add(new LineSegment(
            new Point(_camera.WorldToScreen(terrainPoints[0]).X, _canvas.ActualHeight), true));

        return figure;
    }

    private void UpdateMotorcycleVisual()
    {
        var motorcycle = _gameController.Motorcycle;
        if (motorcycle is null)
            return;

        var (skeletonPoints, skeletonLines) = motorcycle.GetSkeleton();
        _skeletonRenderer.UpdateSkeletonVisuals(skeletonPoints, skeletonLines);
    }

    public class SkeletonRenderer
    {
        private readonly Canvas _canvas;
        private readonly Camera _camera;
        private readonly Dictionary<BikeGeom.SkeletonLineType, List<Line>> _skeletonLines = new();
        private readonly Dictionary<BikeGeom.SkeletonLineType, SolidColorBrush> _lineTypeBrushes;

        // Неиспользуемые переменные, закомментированы
        // private Ellipse _frontWheel = null!, _rearWheel = null!;
        // private Line[] _frontWheelSpokes = null!, _rearWheelSpokes = null!;
        // private Line _frontSuspension = null!, _rearSuspension = null!;
        // private Path _frontSuspensionSpring = null!, _rearSuspensionSpring = null!;

        public SkeletonRenderer(Canvas canvas, Camera camera,
            Dictionary<BikeGeom.SkeletonLineType, SolidColorBrush> lineTypeBrushes)
        {
            _canvas = canvas;
            _camera = camera;
            _lineTypeBrushes = lineTypeBrushes;
        }

        public void InitializeSkeletonLines()
        {
            foreach (BikeGeom.SkeletonLineType lineType in Enum.GetValues(typeof(BikeGeom.SkeletonLineType)))
            {
                _skeletonLines[lineType] = new List<Line>();
                CreateLinesForType(lineType);
            }
        }

        private void CreateLinesForType(BikeGeom.SkeletonLineType lineType)
        {
            var brush = GetBrushForLineType(lineType);

            for (int i = 0; i < RenderConstants.InitialSkeletonLinesCount; i++)
            {
                var line = new Line
                {
                    Stroke = brush,
                    StrokeThickness = RenderConstants.BikeStrokeThickness,
                    Visibility = Visibility.Collapsed
                };

                _canvas.Children.Add(line);
                _skeletonLines[lineType].Add(line);
            }
        }

        // Закомментированные неиспользуемые методы
        /*
        private (Ellipse front, Ellipse rear) InitializeWheels()
        {
            var front = CreateWheel();
            var rear = CreateWheel();

            _canvas.Children.Add(front);
            _canvas.Children.Add(rear);

            return (front, rear);
        }

        private (Line[] front, Line[] rear) InitializeSpokes() =>
            (CreateWheelSpokes(), CreateWheelSpokes());

        private (Line front, Line rear) InitializeSuspensions()
        {
            var front = new Line
            {
                Stroke = Brushes.SuspensionStroke,
                StrokeThickness = RenderConstants.SuspensionStrokeThickness
            };

            var rear = new Line
            {
                Stroke = Brushes.SuspensionStroke,
                StrokeThickness = RenderConstants.SuspensionStrokeThickness
            };

            _canvas.Children.Add(front);
            _canvas.Children.Add(rear);

            return (front, rear);
        }

        private (Path front, Path rear) InitializeSuspensionSprings()
        {
            var front = new Path
            {
                Stroke = Brushes.SuspensionStroke,
                StrokeThickness = RenderConstants.SuspensionSpringStrokeThickness,
                Fill = Brushes.SuspensionFill
            };

            var rear = new Path
            {
                Stroke = Brushes.SuspensionStroke,
                StrokeThickness = RenderConstants.SuspensionSpringStrokeThickness,
                Fill = Brushes.SuspensionFill
            };

            _canvas.Children.Add(front);
            _canvas.Children.Add(rear);

            return (front, rear);
        }

        private Ellipse CreateWheel() => new()
        {
            Width = _gameController.Motorcycle.GetWheelRadius() * 2,
            Height = _gameController.Motorcycle.GetWheelRadius() * 2,
            Fill = Brushes.WheelFill,
            Stroke = Brushes.WheelStroke,
            StrokeThickness = RenderConstants.BikeStrokeThickness
        };

        private Line[] CreateWheelSpokes()
        {
            var spokes = new Line[RenderConstants.SpokeCount];

            for (int i = 0; i < spokes.Length; i++)
            {
                spokes[i] = new()
                {
                    Stroke = Brushes.SpokeStroke,
                    StrokeThickness = RenderConstants.SpokeStrokeThickness
                };

                _canvas.Children.Add(spokes[i]);
            }

            return spokes;
        }

        private void UpdateWheels(Motorcycle motorcycle)
        {
            var frontWheelPos = _camera.WorldToScreen(motorcycle.FrontWheelPosition);
            var rearWheelPos = _camera.WorldToScreen(motorcycle.RearWheelPosition);
            double wheelRadius = motorcycle.GetWheelRadius();
        
            UpdateWheelVisual(_frontWheel, frontWheelPos, wheelRadius);
            UpdateWheelVisual(_rearWheel, rearWheelPos, wheelRadius);
        
            UpdateWheelSpokes(_frontWheelSpokes, frontWheelPos, motorcycle.FrontWheelRotation, wheelRadius);
            UpdateWheelSpokes(_rearWheelSpokes, rearWheelPos, motorcycle.RearWheelRotation, wheelRadius);
        }

        private void UpdateSuspensions(Motorcycle motorcycle)
        {
            var frontWheelPos = _camera.WorldToScreen(motorcycle.FrontWheelPosition);
            var rearWheelPos = _camera.WorldToScreen(motorcycle.RearWheelPosition);
            var frontAttachPos = _camera.WorldToScreen(motorcycle.FrontAttachmentPoint);
            var rearAttachPos = _camera.WorldToScreen(motorcycle.RearAttachmentPoint);
        
            UpdateSuspensionLine(_frontSuspension, frontWheelPos, frontAttachPos);
            UpdateSuspensionLine(_rearSuspension, rearWheelPos, rearAttachPos);
        
            UpdateSuspensionSpring(_frontSuspensionSpring, frontWheelPos, frontAttachPos, motorcycle.FrontSuspensionOffset);
            UpdateSuspensionSpring(_rearSuspensionSpring, rearWheelPos, rearAttachPos, motorcycle.RearSuspensionOffset);
        }

        private void UpdateSuspensionLine(Line suspensionLine, Point wheelPos, Point attachmentPos)
        {
            suspensionLine.X1 = wheelPos.X;
            suspensionLine.Y1 = wheelPos.Y;
            suspensionLine.X2 = attachmentPos.X;
            suspensionLine.Y2 = attachmentPos.Y;
            suspensionLine.Visibility = Visibility.Visible;
        }

        private void UpdateSuspensionSpring(Path springPath, Point wheelPos, Point attachmentPos, double compressionOffset)
        {
            springPath.Visibility = Visibility.Collapsed;
        }

        private void UpdateWheelVisual(Ellipse wheel, Point position, double radius)
        {
            wheel.Width = wheel.Height = radius * 2;
        
            Canvas.SetLeft(wheel, position.X - radius);
            Canvas.SetTop(wheel, position.Y - radius);
        }

        private void ApplyMovementEffects(Motorcycle motorcycle)
        {
            double opacity = motorcycle.Velocity.Length > RenderConstants.MovementSpeedThreshold
                ? RenderConstants.MovementBlurOpacity
                : RenderConstants.NormalOpacity;
        
            _frontWheel.Opacity = opacity;
            _rearWheel.Opacity = opacity;
        }

        private void UpdateWheelSpokes(Line[] spokes, Point wheelCenter, double wheelRotation, double wheelRadius)
        {
            for (int i = 0; i < spokes.Length; i++)
            {
                double angle = wheelRotation + i * (2 * Math.PI / spokes.Length);
                double cosAngle = Math.Cos(angle);
                double sinAngle = Math.Sin(angle);
        
                (spokes[i].X1, spokes[i].Y1) = (wheelCenter.X, wheelCenter.Y);
                (spokes[i].X2, spokes[i].Y2) = (
                    wheelCenter.X + cosAngle * wheelRadius,
                    wheelCenter.Y + sinAngle * wheelRadius
                );
            }
        }
        */

        public void UpdateSkeletonVisuals(
            List<BikeGeom.SkeletonPoint> skeletonPoints,
            List<BikeGeom.SkeletonLine> skeletonLines)
        {
            HideAllSkeletonLines();
            UpdateSkeletonLines(skeletonPoints, skeletonLines);

            // Закомментированные методы обновления неиспользуемых элементов
            // UpdateWheels(motorcycle);
            // UpdateSuspensions(motorcycle);
            // ApplyMovementEffects(motorcycle);
        }

        private void HideAllSkeletonLines() =>
            _skeletonLines.Values
                .SelectMany(lines => lines)
                .ToList()
                .ForEach(line => line.Visibility = Visibility.Collapsed);

        private void UpdateSkeletonLines(
                    List<BikeGeom.SkeletonPoint> points,
                    List<BikeGeom.SkeletonLine> lines)
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

        private void EnsureLineExists(BikeGeom.SkeletonLineType lineType, int index)
        {
            if (index < _skeletonLines[lineType].Count)
                return;

            var line = new Line
            {
                Stroke = GetBrushForLineType(lineType),
                StrokeThickness = RenderConstants.BikeStrokeThickness
            };

            _canvas.Children.Add(line);
            _skeletonLines[lineType].Add(line);
        }

        private void UpdateSkeletonLine(
            BikeGeom.SkeletonLineType lineType,
            int lineIndex,
            BikeGeom.SkeletonLine skeletonLine,
            List<BikeGeom.SkeletonPoint> points)
        {
            var line = _skeletonLines[lineType][lineIndex];
            var startPoint = _camera.WorldToScreen(points[skeletonLine.StartPointIndex].Position);
            var endPoint = _camera.WorldToScreen(points[skeletonLine.EndPointIndex].Position);

            (line.X1, line.Y1) = (startPoint.X, startPoint.Y);
            (line.X2, line.Y2) = (endPoint.X, endPoint.Y);
            line.Visibility = Visibility.Visible;
        }

        private SolidColorBrush GetBrushForLineType(BikeGeom.SkeletonLineType lineType) =>
            _lineTypeBrushes.TryGetValue(lineType, out var brush)
                ? brush
                : _lineTypeBrushes[BikeGeom.SkeletonLineType.MainFrame];
    }
}