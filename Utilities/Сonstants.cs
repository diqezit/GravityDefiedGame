using GravityDefiedGame.Models;
using Microsoft.Xna.Framework;
using System;

namespace GravityDefiedGame.Utilities
{
    public enum BikeType
    {
        Standard,
        Sport,
        OffRoad
    }

    public record BikeProperties(
        float mass,
        float power,
        float brakeForce,
        float drag,
        float maxLeanAngle,
        float leanSpeed,
        float friction,
        float suspensionStrength,
        float suspensionDamping,
        float suspensionRestLength,
        float maxSuspensionAngle
    );

    public record WheelProperties(
        float radius,
        float friction,
        float suspensionStrength,
        float suspensionDamping
    );

    public static class GameConstants
    {
        public static class Bike
        {
            public static readonly BikeProperties Standard = new(
                mass: 180.0f,                // Масса стандартного мотоцикла в кг
                power: 10000.0f,             // Мощность двигателя в Н
                brakeForce: 2000.0f,         // Сила торможения в Н
                drag: 0.35f,                 // Коэффициент сопротивления воздуха
                maxLeanAngle: MathHelper.Pi / 2.5f,// Максимальный угол наклона
                leanSpeed: 6.0f,             // Скорость изменения наклона
                friction: 1.0f,              // Базовый коэффициент трения
                suspensionStrength: 8000.0f, // Жесткость подвески
                suspensionDamping: 1000.0f,  // Демпфирование подвески
                suspensionRestLength: 25.0f, // Длина подвески в покое
                maxSuspensionAngle: MathHelper.Pi / 12f // Максимальный угол отклонения подвески
            );

            public static readonly BikeProperties Sport = new(
                mass: 150.0f,                // Масса спортивного мотоцикла в кг
                power: 14000.0f,             // Мощность двигателя в Н
                brakeForce: 2400.0f,         // Сила торможения в Н
                drag: 0.25f,                 // Коэффициент сопротивления воздуха
                maxLeanAngle: MathHelper.Pi / 2f,  // Максимальный угол наклона
                leanSpeed: 7.0f,             // Скорость изменения наклона
                friction: 1.0f,              // Базовый коэффициент трения
                suspensionStrength: 4500.0f, // Жесткость подвески
                suspensionDamping: 450.0f,   // Демпфирование подвески
                suspensionRestLength: 20.0f, // Длина подвески в покое
                maxSuspensionAngle: MathHelper.Pi / 5.14f // Максимальный угол отклонения подвески
            );

            public static readonly BikeProperties OffRoad = new(
                mass: 200.0f,                // Масса внедорожного мотоцикла в кг
                power: 12000.0f,             // Мощность двигателя в Н
                brakeForce: 1800.0f,         // Сила торможения в Н
                drag: 0.45f,                 // Коэффициент сопротивления воздуха
                maxLeanAngle: MathHelper.Pi / 3f,  // Максимальный угол наклона
                leanSpeed: 5.5f,             // Скорость изменения наклона
                friction: 1.0f,              // Базовый коэффициент трения
                suspensionStrength: 5500.0f, // Жесткость подвески
                suspensionDamping: 550.0f,   // Демпфирование подвески
                suspensionRestLength: 30.0f, // Длина подвески в покое
                maxSuspensionAngle: MathHelper.Pi / 7.2f // Максимальный угол отклонения подвески
            );
        }

        public static class Debug
        {
            public const string
                BikePhysicsTag = nameof(BikePhysics), // Тег для логирования физики мотоцикла
                MotorcycleTag = nameof(Motorcycle);   // Тег для логирования мотоцикла

            public const float
                LogThrottle = 0.5f,          // Порог газа для логирования
                EngineForceThreshold = 1.2f; // Порог силы двигателя для логирования
        }

        public static class Motorcycle
        {
            public const float
                DefaultWheelBase = 75.0f,    // Базовая колесная база мотоцикла
                FullRotation = MathHelper.TwoPi, // Полный оборот колеса в радианах
                AirDetectionThreshold = 7.5f;// Порог определения нахождения в воздухе

            public const float
                WheelCircumferenceFactor = MathHelper.TwoPi, // Фактор расчета окружности колеса
                AirRotationFactor = 12.0f,   // Скорость вращения колеса в воздухе
                GroundRotationFactor = 12.0f,// Скорость вращения колеса на земле
                ThrottleRotationFactor = 6.0f,// Влияние газа на вращение заднего колеса
                SlipThrottleFactor = 1.8f;   // Фактор проскальзывания при высоком газе

            public static readonly Vector2 DefaultStartPosition = new(100, 100); // Начальная позиция мотоцикла
            public static readonly Color DefaultBikeColor = Color.Red;        // Цвет мотоцикла по умолчанию
        }

        public static class Physics
        {
            public const float
                ThrottleTransitionRate = 3.0f,   // Скорость изменения газа
                BrakeTransitionRate = 4.0f,      // Скорость изменения тормоза
                BrakeTransitionThreshold = 0.01f,// Порог сброса тормоза
                TorqueSmoothingFactor = 2.5f,    // Фактор сглаживания крутящего момента
                TorqueIdleThreshold = 0.1f,      // Порог холостого хода для момента
                TorqueFadeThreshold = 0.05f,     // Порог затухания крутящего момента
                SlopeTransitionRate = 2.0f;      // Скорость адаптации к уклону

            public const float
                DefaultGravity = 1000.0f,        // Базовая гравитация
                DefaultWheelRadius = 15.0f,      // Радиус колеса по умолчанию
                GravityMultiplier = 0.1f,        // Множитель гравитации
                CriticalLeanAngle = MathHelper.Pi / 1.8f; // Критический угол наклона для аварии

            public const float
                DirectControlFactor = 10.0f,     // Фактор прямого управления
                WheelieTransitionSpeed = 5.0f,   // Скорость перехода для вилли
                AirFrictionMultiplier = 0.05f,   // Множитель трения в воздухе
                DragThrottleThreshold = 0.1f,    // Порог газа для сопротивления
                DragThrottleMultiplierBase = 1.0f,// Базовый множитель сопротивления при газе
                DragThrottleMultiplier = 1.5f,   // Множитель сопротивления при газе
                DragIdleMultiplier = 2.0f,       // Множитель сопротивления на холостом ходу
                GroundToAirTransitionSpeed = 10.0f,// Скорость перехода с земли в воздух
                AirToGroundTransitionSpeed = 10.0f,// Скорость перехода из воздуха на землю
                LeanControlSpeed = 4.0f,         // Скорость управления наклоном
                MaxAirControlForce = 5.0f,       // Максимальная сила управления в воздухе
                SuspensionDampingFactor = 23.0f, // Фактор демпфирования подвески
                FrictionMultiplier = 2.5f;       // Множитель трения

            public const float
                MinBrakeInput = 0.1f,            // Минимальный ввод тормоза
                NearStopThreshold = 30.0f,       // Порог состояния "почти остановка"
                ReverseStartThreshold = 20.0f,   // Порог начала движения назад
                ReverseForceBase = 1500.0f,      // Базовая сила движения назад
                FrameTimeApproximation = 0.016f, // Примерное время кадра
                MaxReverseSpeed = 300.0f,        // Максимальная скорость назад
                ReverseControlMultiplier = 0.7f,  // Множитель управления при движении назад
                BrakeEfficiencyMultiplier = 1.2f;// Множитель эффективности торможения

            public const float
                FrameCollisionMinVelocity = 50.0f,      // Минимальная скорость для столкновения рамы
                FrameCollisionMinPenetration = 0.3f,    // Минимальное проникновение для столкновения рамы
                FrameCrashThreshold = 0.9f,             // Порог аварии при столкновении рамы
                FrameCollisionReactionForce = 0.3f,     // Сила реакции при столкновении рамы
                FrameStabilizingFactorBase = 1.5f,      // Базовый стабилизирующий фактор при столкновении
                FrameStabilizingFactorStrong = 2.0f,    // Сильный стабилизирующий фактор при столкновении
                FrameStabilizingAngleThreshold = MathHelper.Pi / 6f, // Порог угла для стабилизации
                FrameCollisionMaxDeltaVelocity = 100.0f,// Максимальное изменение скорости по Y при столкновении
                FrameCollisionMaxDeltaAngular = 1.0f,   // Максимальное изменение угловой скорости при столкновении
                FrameCollisionLowSpeedThreshold = 30.0f,// Порог низкой скорости для столкновения рамы
                FrameCollisionLowSpeedImpulse = 10.0f,  // Импульс при низкой скорости для столкновения
                FrameCriticalBackwardTiltAngle = MathHelper.Pi / 2.5f, // Критический угол заднего наклона
                FrameCollisionHighSpeedThreshold = 200.0f, // Порог высокой скорости для столкновения
                WheelDistanceMinRatio = 0.8f,           // Минимальное соотношение расстояния между колесами
                WheelDistanceMaxRatio = 1.2f;           // Максимальное соотношение расстояния между колесами

            public const float
                SuspensionProgressiveFactor = 1.8f,     // Прогрессивный фактор подвески
                MaxSuspensionCompression = 0.85f,       // Максимальное сжатие подвески
                MinSuspensionCompression = 0.2f,        // Минимальное сжатие подвески
                MaxWheelPenetration = 0.75f,            // Максимальное проникновение колеса в поверхность
                BaseCompressionMultiplier = 0.5f,       // Базовый множитель сжатия подвески
                ProgressiveFactorBase = 1.0f,           // Базовый прогрессивный фактор
                ProgressiveFactorMultiplier = 2.0f,     // Множитель прогрессивного фактора
                WheelRadiusHalfFactor = 0.5f,           // Половина радиуса колеса для расчетов
                WheelieReducedSuspensionFactor = 0.3f,  // Уменьшение подвески при вилли
                CompressionSmoothingFactor = 0.5f,      // Фактор сглаживания сжатия подвески
                LargeSuspensionChangeThreshold = 0.2f,  // Порог большого изменения подвески
                LargeSuspensionChangeSmoothingFactor = 0.4f, // Фактор сглаживания при большом изменении
                CompressionRatioBase = 1.0f,            // Базовое соотношение сжатия
                ProgressiveFactorForceMultiplier = 2.0f,// Множитель силы прогрессивного фактора
                VelocityDampingFactor = 1.0f,           // Фактор демпфирования скорости
                FrictionForceMultiplier = 0.8f,         // Множитель силы трения
                ReactionForceSmoothingFactor = 0.7f,    // Фактор сглаживания силы реакции
                LandingSmoothingFactor = 0.6f,          // Фактор сглаживания при приземлении
                WheelieIntensityDampingMax = 0.7f,      // Максимальное демпфирование интенсивности вилли
                WheelieIntensityDampingMultiplier = 0.8f;// Множитель демпфирования интенсивности вилли

            public const float
                MaxAngularVelocity = MathHelper.Pi * 3.0f,    // Максимальная угловая скорость
                MaxAirAngularVelocity = MathHelper.Pi,       // Максимальная угловая скорость в воздухе
                AirDampingFactor = 0.5f,                // Фактор демпфирования в воздухе
                LeanProportionalCoefficient = 5.0f,     // Пропорциональный коэффициент для наклона
                LeanDifferentialCoefficient = 3.0f,     // Дифференциальный коэффициент для наклона
                GroundAngularVelocityFactor = 0.95f;    // Фактор угловой скорости на земле

            public const float
                SlopeAngleDifferenceThreshold = 0.1f,   // Порог разницы углов склона
                SpeedFactorDenominator = 300.0f,        // Делитель для фактора скорости
                BlendBase = 0.5f,                       // Базовое значение для смешивания углов
                BlendSpeedFactor = 0.3f;                // Фактор скорости для смешивания углов

            public const float
                SurfaceReactionMultiplier = 0.85f,      // Множитель реакции поверхности
                NormalForceGravityFactor = 4.9f,        // Фактор гравитации для нормальной силы
                LeanFrictionMultiplier = 0.5f,          // Множитель трения при наклоне
                SpeedFrictionThreshold = 1000.0f,       // Порог скорости для уменьшения трения
                SpeedFrictionReductionFactor = 0.15f,   // Фактор уменьшения трения при высокой скорости
                MaxForceSafetyMultiplier = 2000.0f;     // Множитель безопасности для максимальной силы

            public const float
                LowSpeedBoostBase = 2.5f,               // Базовый буст при низкой скорости
                LowSpeedBoostMax = 2.0f,                // Максимальный буст при низкой скорости
                SpeedFactorThreshold = 250.0f,          // Порог для фактора скорости
                StabilityAngleFactor = 0.4f,            // Фактор угла для стабильности
                EnginePowerLimitMultiplier = 1.8f;      // Множитель ограничения мощности двигателя

            public const float
                WheelieMinAngle = 0.15f,                // Минимальный угол для вилли
                WheelieHeightFactor = 0.5f,             // Фактор высоты для вилли
                WheelieForceMultiplier = 0.35f,         // Множитель силы для вилли
                WheelieAngleBoost = 1.5f,               // Усиление угла для вилли
                WheelieThrottleThreshold = 0.4f;        // Порог газа для вилли

            public const float
                AirRotationMultiplier = 0.5f,           // Множитель вращения в воздухе
                AirTorqueMultiplier = 8.0f,             // Множитель крутящего момента в воздухе
                RotationDirectControl = 6.0f,           // Прямое управление вращением
                AirDragReductor = 0.6f,                 // Редуктор сопротивления в воздухе
                LiftSpeedThreshold = 250.0f,            // Порог скорости для подъема
                LiftForceMultiplier = 0.2f,             // Множитель силы подъема
                MaxLiftForce = 400.0f;                  // Максимальная сила подъема

            public const float
                MinThrottleForFriction = 0.1f,          // Минимальный газ для трения
                StrongThrottleThreshold = 0.6f,         // Порог сильного газа
                FrontWheelReactionReduction = 0.4f,     // Уменьшение реакции переднего колеса
                FrontWheelReductionAngle = 0.3f,        // Угол для уменьшения реакции переднего колеса
                MinSlipFactor = 0.4f,                   // Минимальный фактор проскальзывания
                SlipThrottleMultiplier = 0.25f,         // Множитель проскальзывания при газе
                WheelSlipThreshold = 0.5f,              // Порог проскальзывания колеса
                SlipFrictionRatio = 0.7f,               // Соотношение трения для проскальзывания
                SlipSpeedThreshold = 400.0f,            // Порог скорости для проскальзывания
                HighWheelSlipThreshold = 0.4f,          // Порог высокого проскальзывания
                HighThrottleThreshold = 0.7f;           // Порог высокого газа

            public const float
                AirDetectionMultiplier = 1.8f,          // Множитель для определения нахождения в воздухе
                MomentOfInertiaMultiplier = 0.6f;       // Множитель момента инерции

            public const float
                WheelieThrottleMinimum = 0.4f,          // Минимальный газ для вилли
                WheelieThrottleMultiplier = 1.5f,       // Множитель газа для вилли
                WheelieForceBase = 0.3f,                // Базовая сила для вилли
                WheelieMaxForceMultiplier = 2.0f,       // Множитель максимальной силы для вилли
                WheelieOptimalMinSpeed = 80.0f,         // Минимальная оптимальная скорость для вилли
                WheelieOptimalMaxSpeed = 400.0f,        // Максимальная оптимальная скорость для вилли
                WheelieMaxSpeed = 700.0f,               // Максимальная скорость для вилли
                WheelieOptimalAngle = 0.1f,             // Оптимальный угол для вилли
                WheelieBalanceAngle = 0.4f,             // Угол баланса для вилли
                WheelieBalanceTolerance = 0.15f,        // Допуск баланса для вилли
                WheelieBalanceStrength = 2.0f,          // Сила баланса для вилли
                WheelieControlMultiplier = 5.0f,        // Множитель управления для вилли
                WheelieStabilizationFactor = 1.0f,      // Фактор стабилизации для вилли
                WheelieBalanceResponseFactor = 2.5f,    // Фактор ответа баланса для вилли
                WheelieEasyTime = 2.0f,                 // Время легкого вилли
                WheelieHardTimeDelta = 3.0f,            // Дельта времени для сложного вилли
                WheelieProgressiveDifficulty = 0.4f,    // Прогрессивная сложность для вилли
                WheelieLowSpeedEfficiency = 0.5f,       // Эффективность на низкой скорости для вилли
                WheelieHighSpeedEfficiency = 0.1f,      // Эффективность на высокой скорости для вилли
                WheelieAngleMinOffset = MathHelper.Pi / 6f,   // Минимальное смещение угла для вилли
                WheelieAngleMaxOffset = MathHelper.Pi / 4f,   // Максимальное смещение угла для вилли
                WheelieLowAngleEfficiency = 0.4f,       // Эффективность при низком угле для вилли
                WheelieHighAngleEfficiency = 0.2f,      // Эффективность при высоком угле для вилли
                WheelieBalanceMinFactor = 0.2f;         // Минимальный фактор баланса для вилли

            public const float
                StoppieThresholdMinimum = 0.6f,         // Минимальный порог для стоппи
                StoppieBrakeMultiplier = 1.0f,          // Множитель тормоза для стоппи
                StoppieForceBase = 0.15f,               // Базовая сила для стоппи
                StoppieMaxForceMultiplier = 2.0f,       // Множитель максимальной силы для стоппи
                StoppieMinSpeed = 300.0f,               // Минимальная скорость для стоппи
                StoppieOptimalMinSpeed = 400.0f,        // Минимальная оптимальная скорость для стоппи
                StoppieOptimalMaxSpeed = 700.0f,        // Максимальная оптимальная скорость для стоппи
                StoppieMaxSpeed = 900.0f,               // Максимальная скорость для стоппи
                StoppieOptimalAngle = 0.05f,            // Оптимальный угол для стоппи
                StoppieMinAngle = 0.2f,                 // Минимальный угол для стоппи
                StoppieHeightFactor = 0.4f,             // Фактор высоты для стоппи
                StoppieBalanceAngle = -0.3f,            // Угол баланса для стоппи
                StoppieBalanceTolerance = 0.08f,        // Допуск баланса для стоппи
                StoppieBalanceStrength = 2.0f,          // Сила баланса для стоппи
                StoppieControlMultiplier = 3.0f,        // Множитель управления для стоппи
                StoppieStabilizationFactor = 0.8f,      // Фактор стабилизации для стоппи
                StoppieBalanceResponseFactor = 2.5f,    // Фактор ответа баланса для стоппи
                StoppieEasyTime = 0.5f,                 // Время легкого стоппи
                StoppieHardTimeDelta = 1.0f,            // Дельта времени для сложного стоппи
                StoppieProgressiveDifficulty = 0.6f,    // Прогрессивная сложность для стоппи
                StoppieLowSpeedEfficiency = 0.3f,       // Эффективность на низкой скорости для стоппи
                StoppieHighSpeedEfficiency = 0.2f,      // Эффективность на высокой скорости для стоппи
                StoppieAngleMinOffset = MathHelper.Pi / 4f,   // Минимальное смещение угла для стоппи
                StoppieAngleMaxOffset = MathHelper.Pi / 6f,   // Максимальное смещение угла для стоппи
                StoppieLowAngleEfficiency = 0.2f,       // Эффективность при низком угле для стоппи
                StoppieHighAngleEfficiency = 0.4f,      // Эффективность при высоком угле для стоппи
                StoppieBalanceMinFactor = 0.2f;         // Минимальный фактор баланса для стоппи

            public const float
                SlopeAngleSmoothingThreshold = 0.02f,   // Порог сглаживания угла склона
                LeanSpeedFactorDenominator = 300.0f,    // Делитель для фактора скорости наклона
                LeanSpeedBaseMultiplier = 0.8f,         // Базовый множитель скорости наклона
                LeanSpeedFactorMultiplier = 0.4f,       // Множитель фактора скорости наклона
                TerrainAdaptationSpeedThreshold = 200.0f,// Порог скорости адаптации к местности
                ThrottleLeanInfluence = 0.2f,           // Влияние газа на наклон
                AngularVelocityDampingBase = 200.0f,    // Базовое демпфирование угловой скорости
                AngularVelocityDampingFactor = 100.0f,  // Фактор демпфирования угловой скорости
                LeanControlTorqueMultiplier = 1000.0f,  // Множитель управляющего момента для наклона
                InputIdleThreshold = 0.1f,              // Порог холостого хода для ввода
                AngularVelocityIdleThreshold = 0.1f,    // Порог холостого хода угловой скорости
                StabilizationFactorBase = 1.2f,         // Базовый стабилизирующий фактор
                StabilizationFactorSpeedMultiplier = 0.8f, // Множитель стабилизации по скорости
                StabilizationSpeedThreshold = 400.0f,   // Порог скорости для стабилизации
                StabilizationTorqueMultiplier = 100.0f; // Множитель стабилизирующего момента

            public const int StatusLogInterval = 100;  // Интервал логирования состояния
        }

        public static class Validation
        {
            public const float
                MaxSafeVelocity = 3000.0f,              // Максимальная безопасная скорость
                MaxSafeAngularVelocity = MathHelper.Pi * 4f,  // Максимальная безопасная угловая скорость
                MaxSafeAcceleration = 6000.0f,          // Максимальное безопасное ускорение
                MaxSafeHeight = 600.0f,                 // Максимальная безопасная высота
                MaxSafeSpeed = 2000.0f,                 // Максимальная безопасная скорость для предупреждений
                MaxSafeRotation = 6.0f,                 // Максимальное безопасное вращение
                DangerLandingVelocity = 1200.0f,        // Опасная скорость приземления
                MinSuspensionOffset = 0.1f,             // Минимальное смещение подвески
                HighSuspensionCompressionThreshold = 0.8f, // Порог высокого сжатия подвески
                CriticalSuspensionCompressionThreshold = 0.95f, // Порог критического сжатия подвески
                SignificantAirTimeThreshold = 0.5f,     // Порог значительного времени в воздухе
                LongAirTimeThreshold = 3.0f,            // Порог длительного времени в воздухе
                SpeedWarningThresholdMultiplier = 0.8f,  // Множитель порога предупреждения о скорости
                SpeedCriticalThresholdMultiplier = 1.2f, // Множитель критического порога скорости
                SpeedWarningCooldown = 1.0f,            // Время ожидания между предупреждениями о скорости
                ExtremeTiltAngleThreshold = MathHelper.Pi / 2.2f, // Порог экстремального угла наклона
                TiltWarningCooldown = 1.0f;             // Время ожидания между предупреждениями о наклоне
        }

        public static class Wheels
        {
            public static readonly WheelProperties Standard = new(
                radius: 15.0f,               // Радиус стандартного колеса
                friction: 1.0f,              // Коэффициент трения колеса
                suspensionStrength: 1.2f,    // Множитель жесткости подвески
                suspensionDamping: 1.2f      // Множитель демпфирования подвески
            );

            public static readonly WheelProperties Sport = new(
                radius: 14.0f,               // Радиус спортивного колеса
                friction: 1.0f,              // Коэффициент трения колеса
                suspensionStrength: 0.9f,    // Множитель жесткости подвески
                suspensionDamping: 0.8f      // Множитель демпфирования подвески
            );

            public static readonly WheelProperties OffRoad = new(
                radius: 18.0f,               // Радиус внедорожного колеса
                friction: 1.0f,              // Коэффициент трения колеса
                suspensionStrength: 1.3f,    // Множитель жесткости подвески
                suspensionDamping: 1.2f      // Множитель демпфирования подвески
            );
        }
    }
}