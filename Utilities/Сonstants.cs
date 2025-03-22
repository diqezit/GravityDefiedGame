using GravityDefiedGame.Models;
using System;
using System.Windows;
using System.Windows.Media;

namespace GravityDefiedGame.Utilities
{
    public enum BikeType
    {
        Standard,
        Sport,
        OffRoad
    }

    public record BikeProperties(
        double mass,
        double power,
        double brakeForce,
        double drag,
        double maxLeanAngle,
        double leanSpeed,
        double friction,
        double suspensionStrength,
        double suspensionDamping,
        double suspensionRestLength,
        double maxSuspensionAngle
    );

    public record WheelProperties(
        double radius,
        double friction,
        double suspensionStrength,
        double suspensionDamping
    );

    public static class GameConstants
    {
        public static class Bike
        {
            public static readonly BikeProperties Standard = new(
                mass: 180.0,                // Масса стандартного мотоцикла в кг
                power: 10000.0,             // Мощность двигателя в Н
                brakeForce: 2000.0,         // Сила торможения в Н
                drag: 0.35,                 // Коэффициент сопротивления воздуха
                maxLeanAngle: Math.PI / 2.5,// Максимальный угол наклона
                leanSpeed: 6.0,             // Скорость изменения наклона
                friction: 1.0,              // Базовый коэффициент трения
                suspensionStrength: 8000.0, // Жесткость подвески
                suspensionDamping: 800.0,   // Демпфирование подвески
                suspensionRestLength: 25.0, // Длина подвески в покое
                maxSuspensionAngle: Math.PI / 12 // Максимальный угол отклонения подвески
            );

            public static readonly BikeProperties Sport = new(
                mass: 150.0,                // Масса спортивного мотоцикла в кг
                power: 14000.0,             // Мощность двигателя в Н
                brakeForce: 2400.0,         // Сила торможения в Н
                drag: 0.25,                 // Коэффициент сопротивления воздуха
                maxLeanAngle: Math.PI / 2,  // Максимальный угол наклона
                leanSpeed: 7.0,             // Скорость изменения наклона
                friction: 1.0,              // Базовый коэффициент трения
                suspensionStrength: 4500.0, // Жесткость подвески
                suspensionDamping: 450.0,   // Демпфирование подвески
                suspensionRestLength: 20.0, // Длина подвески в покое
                maxSuspensionAngle: Math.PI / 5.14 // Максимальный угол отклонения подвески
            );

            public static readonly BikeProperties OffRoad = new(
                mass: 200.0,                // Масса внедорожного мотоцикла в кг
                power: 12000.0,             // Мощность двигателя в Н
                brakeForce: 1800.0,         // Сила торможения в Н
                drag: 0.45,                 // Коэффициент сопротивления воздуха
                maxLeanAngle: Math.PI / 3,  // Максимальный угол наклона
                leanSpeed: 5.5,             // Скорость изменения наклона
                friction: 1.0,              // Базовый коэффициент трения
                suspensionStrength: 5500.0, // Жесткость подвески
                suspensionDamping: 550.0,   // Демпфирование подвески
                suspensionRestLength: 30.0, // Длина подвески в покое
                maxSuspensionAngle: Math.PI / 7.2 // Максимальный угол отклонения подвески
            );
        }

        public static class Debug
        {
            public const string
                BikePhysicsTag = nameof(BikePhysics), // Тег для логирования физики мотоцикла
                MotorcycleTag = nameof(Motorcycle);   // Тег для логирования мотоцикла

            public const double
                LogThrottle = 0.5,          // Порог газа для логирования
                EngineForceThreshold = 1.2; // Порог силы двигателя для логирования
        }

        public static class Motorcycle
        {
            public const double
                DefaultWheelBase = 75.0,    // Базовая колесная база мотоцикла
                FullRotation = 2 * Math.PI, // Полный оборот колеса в радианах
                AirDetectionThreshold = 7.5;// Порог определения нахождения в воздухе

            public const double
                WheelCircumferenceFactor = 2 * Math.PI, // Фактор расчета окружности колеса
                AirRotationFactor = 12.0,   // Скорость вращения колеса в воздухе
                GroundRotationFactor = 12.0,// Скорость вращения колеса на земле
                ThrottleRotationFactor = 6.0,// Влияние газа на вращение заднего колеса
                SlipThrottleFactor = 1.8;   // Фактор проскальзывания при высоком газе

            public static readonly Point DefaultStartPosition = new(100, 100); // Начальная позиция мотоцикла
            public static readonly Color DefaultBikeColor = Colors.Red;        // Цвет мотоцикла по умолчанию
        }

        public static class Physics
        {
            public const double
                ThrottleTransitionRate = 3.0,   // Скорость изменения газа
                BrakeTransitionRate = 4.0,      // Скорость изменения тормоза
                BrakeTransitionThreshold = 0.01,// Порог сброса тормоза
                TorqueSmoothingFactor = 2.5,    // Фактор сглаживания крутящего момента
                TorqueIdleThreshold = 0.1,      // Порог холостого хода для момента
                TorqueFadeThreshold = 0.05,     // Порог затухания крутящего момента
                SlopeTransitionRate = 2.0;      // Скорость адаптации к уклону

            public const double
                DefaultGravity = 1000.0,        // Базовая гравитация
                DefaultWheelRadius = 15.0,      // Радиус колеса по умолчанию
                GravityMultiplier = 0.1,        // Множитель гравитации
                CriticalLeanAngle = Math.PI / 1.8; // Критический угол наклона для аварии

            public const double
                DirectControlFactor = 10.0,     // Фактор прямого управления
                WheelieTransitionSpeed = 5.0,   // Скорость перехода для вилли
                AirFrictionMultiplier = 0.05,   // Множитель трения в воздухе
                DragThrottleThreshold = 0.1,    // Порог газа для сопротивления
                DragThrottleMultiplierBase = 1.0,// Базовый множитель сопротивления при газе
                DragThrottleMultiplier = 1.5,   // Множитель сопротивления при газе
                DragIdleMultiplier = 2.0,       // Множитель сопротивления на холостом ходу
                GroundToAirTransitionSpeed = 10.0,// Скорость перехода с земли в воздух
                AirToGroundTransitionSpeed = 10.0,// Скорость перехода из воздуха на землю
                LeanControlSpeed = 4.0,         // Скорость управления наклоном
                MaxAirControlForce = 5.0,       // Максимальная сила управления в воздухе
                SuspensionDampingFactor = 23.0, // Фактор демпфирования подвески
                FrictionMultiplier = 2.5;       // Множитель трения

            public const double
                MinBrakeInput = 0.1,            // Минимальный ввод тормоза
                NearStopThreshold = 30.0,       // Порог состояния "почти остановка"
                ReverseStartThreshold = 20.0,   // Порог начала движения назад
                ReverseForceBase = 1500.0,      // Базовая сила движения назад
                FrameTimeApproximation = 0.016, // Примерное время кадра
                MaxReverseSpeed = 300.0,        // Максимальная скорость назад
                ReverseControlMultiplier = 0.7,  // Множитель управления при движении назад
                BrakeEfficiencyMultiplier = 1.2;// Множитель эффективности торможения

            public const double
                FrameCollisionMinVelocity = 50.0,      // Минимальная скорость для столкновения рамы
                FrameCollisionMinPenetration = 0.3,    // Минимальное проникновение для столкновения рамы
                FrameCrashThreshold = 0.9,             // Порог аварии при столкновении рамы
                FrameCollisionReactionForce = 0.3,     // Сила реакции при столкновении рамы
                FrameStabilizingFactorBase = 1.5,      // Базовый стабилизирующий фактор при столкновении
                FrameStabilizingFactorStrong = 2.0,    // Сильный стабилизирующий фактор при столкновении
                FrameStabilizingAngleThreshold = Math.PI / 6, // Порог угла для стабилизации
                FrameCollisionMaxDeltaVelocity = 100.0,// Максимальное изменение скорости по Y при столкновении
                FrameCollisionMaxDeltaAngular = 1.0,   // Максимальное изменение угловой скорости при столкновении
                FrameCollisionLowSpeedThreshold = 30.0,// Порог низкой скорости для столкновения рамы
                FrameCollisionLowSpeedImpulse = 10.0,  // Импульс при низкой скорости для столкновения
                FrameCriticalBackwardTiltAngle = Math.PI / 2.5, // Критический угол заднего наклона
                FrameCollisionHighSpeedThreshold = 200.0, // Порог высокой скорости для столкновения
                WheelDistanceMinRatio = 0.8,           // Минимальное соотношение расстояния между колесами
                WheelDistanceMaxRatio = 1.2;           // Максимальное соотношение расстояния между колесами

            public const double
                SuspensionProgressiveFactor = 1.8,     // Прогрессивный фактор подвески
                MaxSuspensionCompression = 0.85,       // Максимальное сжатие подвески
                MinSuspensionCompression = 0.2,        // Минимальное сжатие подвески
                MaxWheelPenetration = 0.75,            // Максимальное проникновение колеса в поверхность
                BaseCompressionMultiplier = 0.5,       // Базовый множитель сжатия подвески
                ProgressiveFactorBase = 1.0,           // Базовый прогрессивный фактор
                ProgressiveFactorMultiplier = 2.0,     // Множитель прогрессивного фактора
                WheelRadiusHalfFactor = 0.5,           // Половина радиуса колеса для расчетов
                WheelieReducedSuspensionFactor = 0.3,  // Уменьшение подвески при вилли
                CompressionSmoothingFactor = 0.3,      // Фактор сглаживания сжатия подвески
                LargeSuspensionChangeThreshold = 0.2,  // Порог большого изменения подвески
                LargeSuspensionChangeSmoothingFactor = 0.4, // Фактор сглаживания при большом изменении
                CompressionRatioBase = 1.0,            // Базовое соотношение сжатия
                ProgressiveFactorForceMultiplier = 2.0,// Множитель силы прогрессивного фактора
                VelocityDampingFactor = 0.5,           // Фактор демпфирования скорости
                FrictionForceMultiplier = 0.8,         // Множитель силы трения
                ReactionForceSmoothingFactor = 0.5,    // Фактор сглаживания силы реакции
                LandingSmoothingFactor = 0.6,          // Фактор сглаживания при приземлении
                WheelieIntensityDampingMax = 0.7,      // Максимальное демпфирование интенсивности вилли
                WheelieIntensityDampingMultiplier = 0.8;// Множитель демпфирования интенсивности вилли

            public const double
                MaxAngularVelocity = Math.PI * 3.0,    // Максимальная угловая скорость
                MaxAirAngularVelocity = Math.PI,       // Максимальная угловая скорость в воздухе
                AirDampingFactor = 0.5,                // Фактор демпфирования в воздухе
                LeanProportionalCoefficient = 5.0,     // Пропорциональный коэффициент для наклона
                LeanDifferentialCoefficient = 3.0,     // Дифференциальный коэффициент для наклона
                GroundAngularVelocityFactor = 0.95;    // Фактор угловой скорости на земле

            public const double
                SlopeAngleDifferenceThreshold = 0.1,   // Порог разницы углов склона
                SpeedFactorDenominator = 300.0,        // Делитель для фактора скорости
                BlendBase = 0.5,                       // Базовое значение для смешивания углов
                BlendSpeedFactor = 0.3;                // Фактор скорости для смешивания углов

            public const double
                SurfaceReactionMultiplier = 0.85,      // Множитель реакции поверхности
                NormalForceGravityFactor = 4.9,        // Фактор гравитации для нормальной силы
                LeanFrictionMultiplier = 0.5,          // Множитель трения при наклоне
                SpeedFrictionThreshold = 1000.0,       // Порог скорости для уменьшения трения
                SpeedFrictionReductionFactor = 0.15,   // Фактор уменьшения трения при высокой скорости
                MaxForceSafetyMultiplier = 2000.0;     // Множитель безопасности для максимальной силы

            public const double
                LowSpeedBoostBase = 2.5,               // Базовый буст при низкой скорости
                LowSpeedBoostMax = 2.0,                // Максимальный буст при низкой скорости
                SpeedFactorThreshold = 250.0,          // Порог для фактора скорости
                StabilityAngleFactor = 0.4,            // Фактор угла для стабильности
                EnginePowerLimitMultiplier = 1.8;      // Множитель ограничения мощности двигателя

            public const double
                WheelieMinAngle = 0.15,                // Минимальный угол для вилли
                WheelieHeightFactor = 0.5,             // Фактор высоты для вилли
                WheelieForceMultiplier = 0.35,         // Множитель силы для вилли
                WheelieAngleBoost = 1.5,               // Усиление угла для вилли
                WheelieThrottleThreshold = 0.4;        // Порог газа для вилли

            public const double
                AirRotationMultiplier = 0.5,           // Множитель вращения в воздухе
                AirTorqueMultiplier = 8.0,             // Множитель крутящего момента в воздухе
                RotationDirectControl = 6.0,           // Прямое управление вращением
                AirDragReductor = 0.6,                 // Редуктор сопротивления в воздухе
                LiftSpeedThreshold = 250.0,            // Порог скорости для подъема
                LiftForceMultiplier = 0.2,             // Множитель силы подъема
                MaxLiftForce = 400.0;                  // Максимальная сила подъема

            public const double
                MinThrottleForFriction = 0.1,          // Минимальный газ для трения
                StrongThrottleThreshold = 0.6,         // Порог сильного газа
                FrontWheelReactionReduction = 0.4,     // Уменьшение реакции переднего колеса
                FrontWheelReductionAngle = 0.3,        // Угол для уменьшения реакции переднего колеса
                MinSlipFactor = 0.4,                   // Минимальный фактор проскальзывания
                SlipThrottleMultiplier = 0.25,         // Множитель проскальзывания при газе
                WheelSlipThreshold = 0.5,              // Порог проскальзывания колеса
                SlipFrictionRatio = 0.7,               // Соотношение трения для проскальзывания
                SlipSpeedThreshold = 400.0,            // Порог скорости для проскальзывания
                HighWheelSlipThreshold = 0.4,          // Порог высокого проскальзывания
                HighThrottleThreshold = 0.7;           // Порог высокого газа

            public const double
                AirDetectionMultiplier = 1.8,          // Множитель для определения нахождения в воздухе
                MomentOfInertiaMultiplier = 0.6;       // Множитель момента инерции

            public const double
                WheelieThrottleMinimum = 0.4,          // Минимальный газ для вилли
                WheelieThrottleMultiplier = 1.5,       // Множитель газа для вилли
                WheelieForceBase = 0.3,                // Базовая сила для вилли
                WheelieMaxForceMultiplier = 2.0,       // Множитель максимальной силы для вилли
                WheelieOptimalMinSpeed = 80.0,         // Минимальная оптимальная скорость для вилли
                WheelieOptimalMaxSpeed = 400.0,        // Максимальная оптимальная скорость для вилли
                WheelieMaxSpeed = 700.0,               // Максимальная скорость для вилли
                WheelieOptimalAngle = 0.1,             // Оптимальный угол для вилли
                WheelieBalanceAngle = 0.4,             // Угол баланса для вилли
                WheelieBalanceTolerance = 0.15,        // Допуск баланса для вилли
                WheelieBalanceStrength = 2.0,          // Сила баланса для вилли
                WheelieControlMultiplier = 5.0,        // Множитель управления для вилли
                WheelieStabilizationFactor = 1.0,      // Фактор стабилизации для вилли
                WheelieBalanceResponseFactor = 2.5,    // Фактор ответа баланса для вилли
                WheelieEasyTime = 2.0,                 // Время легкого вилли
                WheelieHardTimeDelta = 3.0,            // Дельта времени для сложного вилли
                WheelieProgressiveDifficulty = 0.4,    // Прогрессивная сложность для вилли
                WheelieLowSpeedEfficiency = 0.5,       // Эффективность на низкой скорости для вилли
                WheelieHighSpeedEfficiency = 0.1,      // Эффективность на высокой скорости для вилли
                WheelieAngleMinOffset = Math.PI / 6,   // Минимальное смещение угла для вилли
                WheelieAngleMaxOffset = Math.PI / 4,   // Максимальное смещение угла для вилли
                WheelieLowAngleEfficiency = 0.4,       // Эффективность при низком угле для вилли
                WheelieHighAngleEfficiency = 0.2,      // Эффективность при высоком угле для вилли
                WheelieBalanceMinFactor = 0.2;         // Минимальный фактор баланса для вилли

            public const double
                StoppieThresholdMinimum = 0.6,         // Минимальный порог для стоппи
                StoppieBrakeMultiplier = 1.0,          // Множитель тормоза для стоппи
                StoppieForceBase = 0.15,               // Базовая сила для стоппи
                StoppieMaxForceMultiplier = 2.0,       // Множитель максимальной силы для стоппи
                StoppieMinSpeed = 300.0,               // Минимальная скорость для стоппи
                StoppieOptimalMinSpeed = 400.0,        // Минимальная оптимальная скорость для стоппи
                StoppieOptimalMaxSpeed = 700.0,        // Максимальная оптимальная скорость для стоппи
                StoppieMaxSpeed = 900.0,               // Максимальная скорость для стоппи
                StoppieOptimalAngle = 0.05,            // Оптимальный угол для стоппи
                StoppieMinAngle = 0.2,                 // Минимальный угол для стоппи
                StoppieHeightFactor = 0.4,             // Фактор высоты для стоппи
                StoppieBalanceAngle = -0.3,            // Угол баланса для стоппи
                StoppieBalanceTolerance = 0.08,        // Допуск баланса для стоппи
                StoppieBalanceStrength = 2.0,          // Сила баланса для стоппи
                StoppieControlMultiplier = 3.0,        // Множитель управления для стоппи
                StoppieStabilizationFactor = 0.8,      // Фактор стабилизации для стоппи
                StoppieBalanceResponseFactor = 2.5,    // Фактор ответа баланса для стоппи
                StoppieEasyTime = 0.5,                 // Время легкого стоппи
                StoppieHardTimeDelta = 1.0,            // Дельта времени для сложного стоппи
                StoppieProgressiveDifficulty = 0.6,    // Прогрессивная сложность для стоппи
                StoppieLowSpeedEfficiency = 0.3,       // Эффективность на низкой скорости для стоппи
                StoppieHighSpeedEfficiency = 0.2,      // Эффективность на высокой скорости для стоппи
                StoppieAngleMinOffset = Math.PI / 4,   // Минимальное смещение угла для стоппи
                StoppieAngleMaxOffset = Math.PI / 6,   // Максимальное смещение угла для стоппи
                StoppieLowAngleEfficiency = 0.2,       // Эффективность при низком угле для стоппи
                StoppieHighAngleEfficiency = 0.4,      // Эффективность при высоком угле для стоппи
                StoppieBalanceMinFactor = 0.2;         // Минимальный фактор баланса для стоппи

            public const double
                SlopeAngleSmoothingThreshold = 0.02,   // Порог сглаживания угла склона
                LeanSpeedFactorDenominator = 300.0,    // Делитель для фактора скорости наклона
                LeanSpeedBaseMultiplier = 0.8,         // Базовый множитель скорости наклона
                LeanSpeedFactorMultiplier = 0.4,       // Множитель фактора скорости наклона
                TerrainAdaptationSpeedThreshold = 200.0,// Порог скорости адаптации к местности
                ThrottleLeanInfluence = 0.2,           // Влияние газа на наклон
                AngularVelocityDampingBase = 200.0,    // Базовое демпфирование угловой скорости
                AngularVelocityDampingFactor = 100.0,  // Фактор демпфирования угловой скорости
                LeanControlTorqueMultiplier = 1000.0,  // Множитель управляющего момента для наклона
                InputIdleThreshold = 0.1,              // Порог холостого хода для ввода
                AngularVelocityIdleThreshold = 0.1,    // Порог холостого хода угловой скорости
                StabilizationFactorBase = 1.2,         // Базовый стабилизирующий фактор
                StabilizationFactorSpeedMultiplier = 0.8, // Множитель стабилизации по скорости
                StabilizationSpeedThreshold = 400.0,   // Порог скорости для стабилизации
                StabilizationTorqueMultiplier = 100.0; // Множитель стабилизирующего момента

            public const int StatusLogInterval = 100;  // Интервал логирования состояния
        }

        public static class Validation
        {
            public const double
                MaxSafeVelocity = 3000.0,              // Максимальная безопасная скорость
                MaxSafeAngularVelocity = Math.PI * 4,  // Максимальная безопасная угловая скорость
                MaxSafeAcceleration = 6000.0,          // Максимальное безопасное ускорение
                MaxSafeHeight = 600.0,                 // Максимальная безопасная высота
                MaxSafeSpeed = 2000.0,                 // Максимальная безопасная скорость для предупреждений
                MaxSafeRotation = 6.0,                 // Максимальное безопасное вращение
                DangerLandingVelocity = 1200.0,        // Опасная скорость приземления
                MinSuspensionOffset = 0.1,             // Минимальное смещение подвески
                HighSuspensionCompressionThreshold = 0.8, // Порог высокого сжатия подвески
                CriticalSuspensionCompressionThreshold = 0.95, // Порог критического сжатия подвески
                SignificantAirTimeThreshold = 0.5,     // Порог значительного времени в воздухе
                LongAirTimeThreshold = 3.0,            // Порог длительного времени в воздухе
                SpeedWarningThresholdMultiplier = 0.8,  // Множитель порога предупреждения о скорости
                SpeedCriticalThresholdMultiplier = 1.2, // Множитель критического порога скорости
                SpeedWarningCooldown = 1.0,            // Время ожидания между предупреждениями о скорости
                ExtremeTiltAngleThreshold = Math.PI / 2.2, // Порог экстремального угла наклона
                TiltWarningCooldown = 1.0;             // Время ожидания между предупреждениями о наклоне
        }

        public static class Wheels
        {
            public static readonly WheelProperties Standard = new(
                radius: 15.0,               // Радиус стандартного колеса
                friction: 1.0,              // Коэффициент трения колеса
                suspensionStrength: 1.2,    // Множитель жесткости подвески
                suspensionDamping: 1.2      // Множитель демпфирования подвески
            );

            public static readonly WheelProperties Sport = new(
                radius: 14.0,               // Радиус спортивного колеса
                friction: 1.0,              // Коэффициент трения колеса
                suspensionStrength: 0.9,    // Множитель жесткости подвески
                suspensionDamping: 0.8      // Множитель демпфирования подвески
            );

            public static readonly WheelProperties OffRoad = new(
                radius: 18.0,               // Радиус внедорожного колеса
                friction: 1.0,              // Коэффициент трения колеса
                suspensionStrength: 1.3,    // Множитель жесткости подвески
                suspensionDamping: 1.2      // Множитель демпфирования подвески
            );
        }
    }
}