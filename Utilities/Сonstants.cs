using System;
using System.Windows;
using System.Windows.Media;
using GravityDefiedGame.Models;

namespace GravityDefiedGame.Utilities
{
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

        public static class Rendering
        {
            // Толщина линий
            public const double
                TerrainStrokeThickness = 3.0,
                BikeStrokeThickness = 3.0,
                SpokeStrokeThickness = 1.0,
                SuspensionStrokeThickness = 2.5,
                SuspensionSpringStrokeThickness = 1.5;

            // Новые константы для рамы мотоцикла
            public const double
                FrameHeightMultiplier = 1.5,    // Множитель для общей высоты рамы (был 1.5)
                LowerFrameOffsetMultiplier = 0.1,  // Множитель для смещения нижних точек рамы
                UpperFrameOffsetMultiplier = 0.5;  // Множитель для смещения верхних точек рамы

            // Настройки отображения
            public const double
                MovementSpeedThreshold = 15.0,
                MovementBlurOpacity = 0.8,
                NormalOpacity = 1.0;

            public const int InitialSkeletonLinesCount = 30;

            // Смещения для сиденья
            public const double
                SeatOffsetX1 = -15.0,
                SeatOffsetX2 = 25.0,
                SeatOffsetX3 = 30.0,
                SeatOffsetX4 = -10.0,
                SeatOffsetY1 = -15.0,
                SeatOffsetY2 = -5.0;

            // Смещения для руля
            public const double
                HandlebarOffsetX1 = -7.5,
                HandlebarOffsetX2 = 7.5,
                HandlebarOffsetX3 = 15.0,
                HandlebarOffsetX4 = 0.0,
                HandlebarOffsetY1 = -7.5,
                HandlebarOffsetY2 = 7.5;

            // Смещения для выхлопной трубы
            public const double
                ExhaustOffsetX1 = -15.0,
                ExhaustOffsetX2 = -25.0,
                ExhaustOffsetX3 = -30.0,
                ExhaustOffsetX4 = -20.0,
                ExhaustOffsetY1 = -15.0,
                ExhaustOffsetY2 = -20.0,
                ExhaustOffsetY3 = -7.5,
                ExhaustOffsetY4 = 0.0;

            // Множители для шасси
            public const double
                ChassisMultiplier1 = 1.2,
                ChassisMultiplier2 = 2.5,
                ChassisMultiplier3 = 3.5,
                ChassisMultiplier4 = 3.0,
                ChassisMultiplier5 = 4.5;
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

            public const int SpokeCount = 6;

            // Параметры визуализации подвески
            public const double
                SuspensionSpringWidth = 6.0,
                SuspensionSpringDensity = 0.2;

            public const int MinZigzagCount = 3;

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