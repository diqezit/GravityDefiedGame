using System;
using System.Windows;
using System.Windows.Media;
using GravityDefiedGame.Models;
using static System.Runtime.InteropServices.JavaScript.JSType;

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
        double suspensionRestLength
    );

    public record WheelProperties(
        double radius,
        double friction,
        double suspensionStrength,
        double suspensionDamping
    );

    public static class GameConstants
    {
        public static class Validation
        {
            // Значения безопасности и проверки
            public const double
                MaxSafeVelocity = 3000.0,
                MaxSafeAngularVelocity = Math.PI * 4,
                MaxSafeAcceleration = 6000.0,
                MaxSafeHeight = 600.0,
                MaxSafeSpeed = 2000.0,
                MaxSafeRotation = 6.0,
                DangerLandingVelocity = 1200.0,
                MinSuspensionOffset = 0.1,
                HighSuspensionCompressionThreshold = 0.8,
                SignificantAirTimeThreshold = 0.5,
                LongAirTimeThreshold = 3.0;
        }

        public static class Physics
        {
            // Базовые физические константы
            public const double
                DefaultGravity = 1000.0,
                DefaultWheelRadius = 15.0,
                GravityMultiplier = 0.1,
                CriticalLeanAngle = Math.PI / 1.8;

            // Новые константы для прямого управления
            public const double
                DirectControlFactor = 10.0,        // Фактор прямого управления наклоном
                WheelieTransitionSpeed = 3.0,      // Скорость перехода в wheelie
                AirFrictionMultiplier = 0.1,       // Множитель трения в воздухе
                AirBrakeReductionFactor = 0.7,     // Фактор уменьшения тормозов в воздухе
                GroundToAirTransitionSpeed = 3.0,    // Скорость перехода с земли в воздух
                AirToGroundTransitionSpeed = 1.5,    // Скорость перехода с воздуха на землю
                LandingTransitionTime = 0.3,         // Время плавного перехода при приземлении
                AirTransitionThreshold = 0.5,        // Порог для переключения логики
                AirLeanControlFactor = 3.5,          // Фактор управления наклоном в воздухе
                LeanControlSpeed = 3.0,              // Скорость изменения управления наклоном
                AirLeanControlReduction = 0.7,       // Уменьшение скорости управления в воздухе
                MaxAirControlForce = 3.0;            // Максимальная сила управления в воздухе

            // Константы для торможения и движения назад
            public const double
                MinBrakeInput = 0.1,                // Минимальное нажатие тормоза для эффекта
                NearStopThreshold = 30.0,           // Скорость, при которой считаем, что мотоцикл почти остановился
                ReverseStartThreshold = 20.0,       // Порог скорости для включения заднего хода
                ReverseForceBase = 1500.0,          // Ускорение при движении назад
                FrameTimeApproximation = 0.016,     // Приблизительное время кадра
                MaxReverseSpeed = 300.0,            // Максимальная скорость движения назад
                ReverseControlMultiplier = 0.7,     // Множитель управляемости при движении назад
                BrakeEfficiencyMultiplier = 1.2;    // Множитель эффективности тормоза

            // Константы для проверки столкновений рамы
            public const double
                FrameCollisionMinVelocity = 50.0,    // Минимальная скорость для проверки столкновений рамы
                FrameCollisionMinPenetration = 0.3,  // Минимальное проникновение для обнаружения столкновения (в радиусах колеса)
                FrameCrashThreshold = 0.9,           // Порог проникновения для аварии (в радиусах колеса)
                FrameCollisionReactionForce = 0.3,   // Множитель силы реакции при столкновении рамы
                FrameStabilizingFactorBase = 1.5,    // Базовый фактор стабилизации при столкновении рамы
                FrameStabilizingFactorStrong = 2.0,  // Усиленный фактор стабилизации при большом угле
                FrameStabilizingAngleThreshold = Math.PI / 6, // Порог угла для усиленной стабилизации
                FrameCollisionMaxDeltaVelocity = 100.0, // Максимальное изменение скорости при столкновении
                FrameCollisionMaxDeltaAngular = 1.0,    // Максимальное изменение угловой скорости при столкновении
                FrameCollisionLowSpeedThreshold = 30.0, // Порог низкой скорости для дополнительного импульса
                FrameCollisionLowSpeedImpulse = 10.0,   // Импульс при низкой скорости
                FrameCriticalBackwardTiltAngle = Math.PI / 2.5, // Критический угол наклона назад (около -72°)
                WheelDistanceMinRatio = 0.8,         // Минимальное отношение расстояния между колесами к номинальному
                WheelDistanceMaxRatio = 1.2;         // Максимальное отношение расстояния между колесами к номинальному

            // Константы подвески
            public const double
                SuspensionProgressiveFactor = 2.0,
                MaxSuspensionCompression = 0.9,
                MinSuspensionCompression = 0.2,
                SuspensionDampingFactor = 12.0,
                MaxWheelPenetration = 0.8,
                PenetrationBaseMultiplier = 1.2,
                PenetrationProgressiveFactor = 0.5,
                WheelRadiusHalfFactor = 0.5;

            // Константы вращения и наклона
            public const double
                MaxAngularVelocity = Math.PI * 2.5,
                LeanProportionalCoefficient = 4.0,
                LeanDifferentialCoefficient = 2.5,
                StabilizationFactor = 1.2,
                LeanSmoothingFactor = 0.15,
                AngularAccelerationWarningFactor = 2.0,
                GroundAngularVelocityFactor = 0.9;

            // Константы сцепления и трения
            public const double
                SurfaceReactionMultiplier = 0.85,
                NormalForceGravityFactor = 4.9,
                FrictionMultiplier = 2.0,
                LeanFrictionMultiplier = 0.4,
                SpeedFrictionThreshold = 1000.0,
                SpeedFrictionReductionFactor = 0.15,
                MaxForceSafetyMultiplier = 2000.0;

            // Константы двигателя и ускорения
            public const double
                LowSpeedBoostBase = 2.5,
                LowSpeedBoostMax = 2.0,
                SpeedFactorThreshold = 250.0,
                StabilityAngleFactor = 0.4,
                EnginePowerLimitMultiplier = 1.8;

            // Константы для wheelie
            public const double
                WheelieMinAngle = 0.25,
                WheelieHeightFactor = 0.5,
                WheelieBoostFactor = 1.8,
                WheelieForceMultiplier = 0.4,
                WheelieAngleBoost = 1.8,
                WheelieVerticalMultiplier = 1.8,
                WheelieThrottleThreshold = 0.5;

            // Константы для полета
            public const double
                AirRotationMultiplier = 0.35,
                AirTorqueMultiplier = 6.0,
                RotationDirectControl = 6.0,
                AirDragReductor = 0.6,
                LiftSpeedThreshold = 250.0,
                LiftForceMultiplier = 0.2,
                MaxLiftForce = 400.0,
                AirAngularVelocityWarningThreshold = 0.9;

            // Константы для скольжения и управления
            public const double
                MinThrottleForFriction = 0.1,
                StrongThrottleThreshold = 0.6,
                FrontWheelReactionReduction = 0.4,
                FrontWheelReductionAngle = 0.3,
                MinSlipFactor = 0.4,
                SlipThrottleMultiplier = 0.25,
                WheelSlipThreshold = 0.5,
                SlipFrictionRatio = 0.7,
                SlipSpeedThreshold = 400.0,
                HighWheelSlipThreshold = 0.4,
                HighThrottleThreshold = 0.7;

            // Прочие константы
            public const double
                AirDetectionMultiplier = 1.2,
                MomentOfInertiaMultiplier = 0.6;
        }

        public static class Motorcycle
        {
            // Основные параметры мотоцикла
            public const double
                DefaultWheelBase = 75.0,
                FullRotation = 2 * Math.PI,
                AirDetectionThreshold = 7.5;

            // Множители для вращения и движения
            public const double
                WheelCircumferenceFactor = 2 * Math.PI,
                AirRotationFactor = 12.0,
                GroundRotationFactor = 12.0,
                ThrottleRotationFactor = 6.0,
                SlipThrottleFactor = 1.8;

            // Начальные значения
            public static readonly Point DefaultStartPosition = new(100, 100);
            public static readonly Color DefaultBikeColor = Colors.Red;
        }

        public static class Bike
        {
            // Стандартный мотоцикл
            public static readonly BikeProperties Standard = new(
                mass: 180.0,
                power: 4500.0,
                brakeForce: 2000.0,
                drag: 0.4,
                maxLeanAngle: Math.PI / 2.5,
                leanSpeed: 5.0,
                friction: 1.0,
                suspensionStrength: 6000.0,
                suspensionDamping: 600.0,
                suspensionRestLength: 25.0
            );

            // Спортивный мотоцикл
            public static readonly BikeProperties Sport = new(
                mass: 150.0,
                power: 5500.0,
                brakeForce: 2400.0,
                drag: 0.3,
                maxLeanAngle: Math.PI / 2,
                leanSpeed: 6.0,
                friction: 0.9,
                suspensionStrength: 4000.0,
                suspensionDamping: 400.0,
                suspensionRestLength: 20.0
            );

            // Внедорожный мотоцикл
            public static readonly BikeProperties OffRoad = new(
                mass: 200.0,
                power: 4000.0,
                brakeForce: 1800.0,
                drag: 0.5,
                maxLeanAngle: Math.PI / 3,
                leanSpeed: 4.5,
                friction: 1.2,
                suspensionStrength: 5000.0,
                suspensionDamping: 500.0,
                suspensionRestLength: 30.0
            );
        }

        public static class Wheels
        {
            // Стандартные колеса
            public static readonly WheelProperties Standard = new(
                radius: 15.0,
                friction: 1.0,
                suspensionStrength: 1.0,
                suspensionDamping: 1.0
            );

            // Спортивные колеса
            public static readonly WheelProperties Sport = new(
                radius: 14.0,
                friction: 0.9,
                suspensionStrength: 0.8,
                suspensionDamping: 0.7
            );

            // Внедорожные колеса
            public static readonly WheelProperties OffRoad = new(
                radius: 18.0,
                friction: 1.2,
                suspensionStrength: 1.2,
                suspensionDamping: 1.1
            );
        }

        public static class Debug
        {
            public const string
                BikePhysicsTag = nameof(BikePhysics),
                MotorcycleTag = nameof(Motorcycle);

            public const double
                LogThrottle = 0.5,
                EngineForceThreshold = 1.2;
        }
    }
}