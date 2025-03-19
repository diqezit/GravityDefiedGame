using System;
using System.Windows;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Models.Level.Constants;

namespace GravityDefiedGame.Models
{
    /// <summary>
    /// Конфигурация для генерации местности
    /// </summary>
    public record TerrainConfig(
        double Length = 3000.0,
        TerrainStyle Style = TerrainStyle.Flat,
        double TopOffset = 50.0,
        double BottomOffset = 50.0
    );

    /// <summary>
    /// Цветовая тема уровня
    /// </summary>
    public record LevelThemeColors(Color Background, Color Terrain);

    /// <summary>
    /// Стиль местности
    /// </summary>
    public enum TerrainStyle
    {
        Default,
        Flat
    }

    /// <summary>
    /// Тема уровня
    /// </summary>
    public enum LevelTheme
    {
        Desert
    }

    /// <summary>
    /// Класс, представляющий игровой уровень
    /// </summary>
    public class Level
    {
        /// <summary>
        /// Структура, описывающая точку местности
        /// </summary>
        public struct TerrainPoint
        {
            public double X { get; set; }
            public double YTop { get; set; }
            public double YMiddle { get; set; }
            public double YBottom { get; set; }
        }

        /// <summary>
        /// Константы, используемые в игровом уровне
        /// </summary>
        public static class Constants
        {
            public const double
                DefaultGroundY = 500.0,
                FinishReachDistance = 50.0,
                DefaultTerrainLength = 3000.0,
                deltaX = 0.1; // Шаг для расчета склона

            public static readonly LevelThemeColors DesertTheme = new(
                Background: Color.FromRgb(255, 204, 102),
                Terrain: Color.FromRgb(204, 153, 102)
            );
        }

        public int Id { get; }
        public string Name { get; }
        public List<TerrainPoint> TerrainPoints { get; private set; } = new();
        public Point StartPoint { get; private set; }
        public Point FinishPoint { get; private set; }
        public double Length { get; private set; }
        public LevelTheme Theme { get; } = LevelTheme.Desert;

        public Color BackgroundColor => DesertTheme.Background;
        public Color TerrainColor => DesertTheme.Terrain;

        /// <summary>
        /// Создает новый уровень с указанным идентификатором и названием
        /// </summary>
        public Level(int id, string name, int? seed = null)
        {
            Id = id;
            Name = name;

            Logger.Info("Level", $"Creating level {id}: {name}");
            GenerateLevel(seed ?? id * 100);
            Logger.Info("Level", $"Level {id} created: Length={Length:F0}");
        }

        /// <summary>
        /// Генерирует уровень с указанным зерном генератора случайных чисел
        /// </summary>
        private void GenerateLevel(int seed)
        {
            var generator = new LevelGenerator(seed);
            var config = new TerrainConfig(DefaultTerrainLength, TerrainStyle.Default, 50.0, 50.0);

            (TerrainPoints, Length, StartPoint, FinishPoint) = generator.GenerateTerrain(config);
        }

        #region Ground Calculation

        /// <summary>
        /// Возвращает высоту земли в указанной точке X
        /// </summary>
        public double GetGroundYAtX(double x) =>
            IsOutOfBounds(x) ? GetOutOfBoundsValue(x) : InterpolateGroundY(x);

        /// <summary>
        /// Проверяет, находится ли точка X за пределами местности
        /// </summary>
        private bool IsOutOfBounds(double x) =>
            TerrainPoints.Count < 2 || x < TerrainPoints[0].X || x > TerrainPoints[^1].X;

        /// <summary>
        /// Возвращает значение высоты для точки, находящейся за пределами местности
        /// </summary>
        private double GetOutOfBoundsValue(double x) =>
            x < TerrainPoints[0].X || x > TerrainPoints[^1].X ? double.MaxValue : DefaultGroundY;

        /// <summary>
        /// Интерполирует высоту земли для указанной позиции X
        /// </summary>
        private double InterpolateGroundY(double x)
        {
            int segmentIndex = FindTerrainSegmentIndex(x);
            if (segmentIndex < 0)
                return DefaultGroundY;

            var (p1, p2) = (TerrainPoints[segmentIndex], TerrainPoints[segmentIndex + 1]);
            return LinearInterpolate(x, p1.X, p1.YMiddle, p2.X, p2.YMiddle);
        }

        /// <summary>
        /// Находит индекс сегмента местности для указанной позиции X
        /// </summary>
        private int FindTerrainSegmentIndex(double x)
        {
            for (int i = 0; i < TerrainPoints.Count - 1; i++)
            {
                if (x >= TerrainPoints[i].X && x <= TerrainPoints[i + 1].X)
                    return i;
            }
            return -1;
        }

        /// <summary>
        /// Линейно интерполирует значение между двумя точками
        /// </summary>
        private double LinearInterpolate(double x, double x1, double y1, double x2, double y2)
        {
            double t = (x - x1) / (x2 - x1);
            return y1 * (1 - t) + y2 * t;
        }

        /// <summary>
        /// Вычисляет угол наклона в радианах для указанной позиции X
        /// </summary>
        public double CalculateSlopeAngle(double x)
        {
            double y1 = GetGroundYAtX(x - deltaX);
            double y2 = GetGroundYAtX(x + deltaX);
            double dy = y2 - y1;
            double dx = 2 * deltaX;
            return Math.Atan2(dy, dx); 
        }

        #endregion

        #region Finish Detection

        /// <summary>
        /// Проверяет, достигнута ли финишная точка
        /// </summary>
        public bool IsFinishReached(Point position)
        {
            double distance = CalculateDistance(position, FinishPoint);
            bool isReached = distance <= FinishReachDistance;

            if (isReached)
                LogFinishReached(distance);

            return isReached;
        }

        /// <summary>
        /// Логирует достижение финиша
        /// </summary>
        private void LogFinishReached(double distance) =>
            Logger.Info("Level", $"Finish reached at distance {distance:F1}");

        /// <summary>
        /// Вычисляет расстояние между двумя точками
        /// </summary>
        private double CalculateDistance(Point p1, Point p2)
        {
            double dx = p1.X - p2.X, dy = p1.Y - p2.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        #endregion

        /// <summary>
        /// Класс генератора уровня
        /// </summary>
        public class LevelGenerator
        {
            /// <summary>
            /// Константы для генерации уровня
            /// </summary>
            public static class Constants
            {
                public const double
                    DefaultTerrainHeight = 500.0,
                    SpikeMaxHeight = 150.0,
                    DepressionMaxDepth = 100.0,
                    SmoothingFactor = 0.7,
                    PreviousPointWeight = 0.3,
                    StartZonePercent = 0.1,
                    EndZonePercent = 0.9,
                    StartPointXOffset = 100.0,
                    StartPointYOffset = -80.0,
                    FinishPointXOffset = -100.0,
                    FinishPointYOffset = -50.0;

                public const int DefaultPointCount = 60, SegmentCount = 5;
            }

            private readonly Random _random;
            private readonly List<TerrainSegmentType> _segmentTypes = new();

            /// <summary>
            /// Типы сегментов местности
            /// </summary>
            public enum TerrainSegmentType
            {
                Plateau,    
                Spike,      
                Depression 
            }

            /// <summary>
            /// Создает новый генератор уровня с указанным зерном
            /// </summary>
            public LevelGenerator(int seed)
            {
                _random = new Random(seed);
                GenerateSegmentTypes();
            }

            /// <summary>
            /// Генерирует типы сегментов для уровня
            /// </summary>
            private void GenerateSegmentTypes()
            {
                for (int i = 0; i < Constants.SegmentCount; i++)
                {
                    _segmentTypes.Add((TerrainSegmentType)_random.Next(0, 3));
                }
            }

            /// <summary>
            /// Генерирует местность уровня на основе конфигурации
            /// </summary>
            public (List<TerrainPoint> Points, double Length, Point StartPoint, Point FinishPoint)
                GenerateTerrain(TerrainConfig config)
            {
                double length = config.Length;
                var terrainPoints = new List<TerrainPoint>();
                double startYMiddle = Constants.DefaultTerrainHeight;

                Point startPoint = CreateStartPoint(terrainPoints, startYMiddle, config);
                GenerateTerrainPoints(terrainPoints, config, startYMiddle, length);
                Point finishPoint = CreateEndPoint(terrainPoints, startYMiddle, length, config);

                LogTerrainGenerated(config);
                return (terrainPoints, length, startPoint, finishPoint);
            }

            /// <summary>
            /// Создает стартовую точку уровня
            /// </summary>
            private Point CreateStartPoint(List<TerrainPoint> points, double yMiddle, TerrainConfig config)
            {
                AddTerrainPointAtStart(points, yMiddle, config.TopOffset, config.BottomOffset);
                return CalculatePlayerStartPosition(yMiddle);
            }

            /// <summary>
            /// Добавляет точку местности в начало уровня
            /// </summary>
            private void AddTerrainPointAtStart(List<TerrainPoint> points, double yMiddle, double topOffset, double bottomOffset) =>
                points.Add(new TerrainPoint
                {
                    X = 0,
                    YTop = yMiddle - topOffset,
                    YMiddle = yMiddle,
                    YBottom = yMiddle + bottomOffset
                });

            /// <summary>
            /// Вычисляет стартовую позицию игрока
            /// </summary>
            private Point CalculatePlayerStartPosition(double yMiddle) =>
                new(Constants.StartPointXOffset, yMiddle + Constants.StartPointYOffset);

            /// <summary>
            /// Создает финишную точку уровня
            /// </summary>
            private Point CreateEndPoint(List<TerrainPoint> points, double yMiddle, double length, TerrainConfig config)
            {
                AddTerrainPointAtEnd(points, yMiddle, length, config.TopOffset, config.BottomOffset);
                return CalculateFinishPosition(yMiddle, length);
            }

            /// <summary>
            /// Добавляет точку местности в конец уровня
            /// </summary>
            private void AddTerrainPointAtEnd(List<TerrainPoint> points, double yMiddle, double length, double topOffset, double bottomOffset) =>
                points.Add(new TerrainPoint
                {
                    X = length,
                    YTop = yMiddle - topOffset,
                    YMiddle = yMiddle,
                    YBottom = yMiddle + bottomOffset
                });

            /// <summary>
            /// Вычисляет позицию финишной точки
            /// </summary>
            private Point CalculateFinishPosition(double yMiddle, double length) =>
                new(length + Constants.FinishPointXOffset, yMiddle + Constants.FinishPointYOffset);

            /// <summary>
            /// Логирует создание местности
            /// </summary>
            private void LogTerrainGenerated(TerrainConfig config) =>
                Logger.Info("LevelGenerator", $"Terrain generated with style: {config.Style}");

            /// <summary>
            /// Генерирует точки местности между стартом и финишем
            /// </summary>
            private void GenerateTerrainPoints(
                List<TerrainPoint> points,
                TerrainConfig config,
                double baseY,
                double length)
            {
                double lastYMiddle = baseY;

                for (int i = 1; i < Constants.DefaultPointCount; i++)
                {
                    double progress = (double)i / Constants.DefaultPointCount;
                    double x = progress * length;
                    double yMiddle = CalculateTerrainHeight(config.Style, baseY, progress, lastYMiddle);

                    AddTerrainPoint(points, x, yMiddle, config.TopOffset, config.BottomOffset);
                    lastYMiddle = yMiddle;
                }
            }

            /// <summary>
            /// Вычисляет высоту местности в зависимости от стиля и прогресса
            /// </summary>
            private double CalculateTerrainHeight(TerrainStyle style, double baseY, double progress, double lastYMiddle)
            {
                double rawHeight = GetRawTerrainHeight(baseY, progress);
                double transitionedHeight = ApplyTransitions(rawHeight, baseY, progress);
                return SmoothValue(lastYMiddle, transitionedHeight);
            }

            /// <summary>
            /// Возвращает "сырую" высоту местности без сглаживания
            /// </summary>
            private double GetRawTerrainHeight(double baseY, double progress)
            {
                int segmentIndex = Math.Min((int)(progress * Constants.SegmentCount), Constants.SegmentCount - 1);
                TerrainSegmentType segmentType = _segmentTypes[segmentIndex];
                double segmentProgress = (progress * Constants.SegmentCount) - segmentIndex;

                return segmentType switch
                {
                    TerrainSegmentType.Plateau => baseY,
                    TerrainSegmentType.Spike => baseY + GetSpikeVariation(segmentProgress),
                    TerrainSegmentType.Depression => baseY + GetDepressionVariation(segmentProgress),
                    _ => baseY
                };
            }

            /// <summary>
            /// Вычисляет вариацию высоты для типа сегмента "Пик"
            /// </summary>
            private double GetSpikeVariation(double segmentProgress)
            {
                double spikeHeight = _random.NextDouble() * Constants.SpikeMaxHeight;
                double spikePosition = _random.NextDouble();
                double distance = Math.Abs(segmentProgress - spikePosition);
                return spikeHeight * Math.Exp(-2.0 * distance * distance);
            }

            /// <summary>
            /// Вычисляет вариацию высоты для типа сегмента "Впадина"
            /// </summary>
            private double GetDepressionVariation(double segmentProgress)
            {
                double depressionDepth = _random.NextDouble() * Constants.DepressionMaxDepth;
                double depressionPosition = _random.NextDouble();
                double distance = Math.Abs(segmentProgress - depressionPosition);
                return -depressionDepth * Math.Exp(-2.0 * distance * distance);
            }

            /// <summary>
            /// Добавляет точку местности с указанными параметрами
            /// </summary>
            private void AddTerrainPoint(List<TerrainPoint> points, double x, double yMiddle, double topOffset, double bottomOffset) =>
                points.Add(new TerrainPoint
                {
                    X = x,
                    YTop = yMiddle - topOffset,
                    YMiddle = yMiddle,
                    YBottom = yMiddle + bottomOffset
                });

            /// <summary>
            /// Сглаживает значение высоты с учетом предыдущего значения
            /// </summary>
            private double SmoothValue(double prevValue, double currentValue) =>
                prevValue * Constants.PreviousPointWeight + currentValue * Constants.SmoothingFactor;

            /// <summary>
            /// Применяет переходы для начальной и конечной зон
            /// </summary>
            private double ApplyTransitions(double y, double baseY, double progress) => progress switch
            {
                < Constants.StartZonePercent => ApplyStartZoneTransition(y, baseY, progress),
                > Constants.EndZonePercent => ApplyEndZoneTransition(y, baseY, progress),
                _ => y
            };

            /// <summary>
            /// Применяет переход для начальной зоны
            /// </summary>
            private double ApplyStartZoneTransition(double y, double baseY, double progress) =>
                baseY + (y - baseY) * (progress / Constants.StartZonePercent);

            /// <summary>
            /// Применяет переход для конечной зоны
            /// </summary>
            private double ApplyEndZoneTransition(double y, double baseY, double progress) =>
                y + (baseY - y) * ((progress - Constants.EndZonePercent) / (1 - Constants.EndZonePercent));
        }
    }
}