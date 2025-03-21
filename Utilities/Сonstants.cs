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
        public static class Validation
        {
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
            public const double
                DefaultGravity = 1000.0,
                DefaultWheelRadius = 15.0,
                GravityMultiplier = 0.1,
                CriticalLeanAngle = Math.PI / 1.8;

            public const double
                DirectControlFactor = 10.0,
                WheelieTransitionSpeed = 5.0,
                AirFrictionMultiplier = 0.05,
                GroundToAirTransitionSpeed = 10.0,
                AirToGroundTransitionSpeed = 10.0,
                LeanControlSpeed = 4.0,
                MaxAirControlForce = 5.0,
                SuspensionDampingFactor = 23.0,
                FrictionMultiplier = 2.5;

            public const double
                MinBrakeInput = 0.1,
                NearStopThreshold = 30.0,
                ReverseStartThreshold = 20.0,
                ReverseForceBase = 1500.0,
                FrameTimeApproximation = 0.016,
                MaxReverseSpeed = 300.0,
                ReverseControlMultiplier = 0.7,
                BrakeEfficiencyMultiplier = 1.2;

            public const double
                FrameCollisionMinVelocity = 50.0,
                FrameCollisionMinPenetration = 0.3,
                FrameCrashThreshold = 0.9,
                FrameCollisionReactionForce = 0.3,
                FrameStabilizingFactorBase = 1.5,
                FrameStabilizingFactorStrong = 2.0,
                FrameStabilizingAngleThreshold = Math.PI / 6,
                FrameCollisionMaxDeltaVelocity = 100.0,
                FrameCollisionMaxDeltaAngular = 1.0,
                FrameCollisionLowSpeedThreshold = 30.0,
                FrameCollisionLowSpeedImpulse = 10.0,
                FrameCriticalBackwardTiltAngle = Math.PI / 2.5,
                WheelDistanceMinRatio = 0.8,
                WheelDistanceMaxRatio = 1.2;

            public const double
                SuspensionProgressiveFactor = 1.8,
                MaxSuspensionCompression = 0.85,
                MinSuspensionCompression = 0.2,
                MaxWheelPenetration = 0.75,
                PenetrationBaseMultiplier = 1.1,
                PenetrationProgressiveFactor = 0.4,
                WheelRadiusHalfFactor = 0.5,
                WheelieReducedSuspensionFactor = 0.3,
                CompressionSmoothingFactor = 0.4,
                LargeSuspensionChangeThreshold = 0.3,
                LargeSuspensionChangeSmoothingFactor = 0.3,
                VelocityDampingFactor = 0.7,
                LandingSmoothingFactor = 0.6,
                WheelieIntensityDampingMax = 0.7,
                WheelieIntensityDampingMultiplier = 0.8;

            public const double
                MaxAngularVelocity = Math.PI * 3.0,
                LeanProportionalCoefficient = 5.0,
                LeanDifferentialCoefficient = 3.0,
                GroundAngularVelocityFactor = 0.95;

            public const double
                SurfaceReactionMultiplier = 0.85,
                NormalForceGravityFactor = 4.9,
                LeanFrictionMultiplier = 0.5,
                SpeedFrictionThreshold = 1000.0,
                SpeedFrictionReductionFactor = 0.15,
                MaxForceSafetyMultiplier = 2000.0;

            public const double
                LowSpeedBoostBase = 2.5,
                LowSpeedBoostMax = 2.0,
                SpeedFactorThreshold = 250.0,
                StabilityAngleFactor = 0.4,
                EnginePowerLimitMultiplier = 1.8;

            public const double
                WheelieMinAngle = 0.15,
                WheelieHeightFactor = 0.5,
                WheelieForceMultiplier = 0.35,
                WheelieAngleBoost = 1.5,
                WheelieThrottleThreshold = 0.4;

            public const double
                AirRotationMultiplier = 0.5,
                AirTorqueMultiplier = 8.0,
                RotationDirectControl = 6.0,
                AirDragReductor = 0.6,
                LiftSpeedThreshold = 250.0,
                LiftForceMultiplier = 0.2,
                MaxLiftForce = 400.0;

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

            public const double
                AirDetectionMultiplier = 1.8,
                MomentOfInertiaMultiplier = 0.6;

            public const double
                WheelieThrottleMinimum = 0.4,
                WheelieThrottleMultiplier = 1.5,
                WheelieForceBase = 0.3,
                WheelieOptimalMinSpeed = 80.0,
                WheelieOptimalMaxSpeed = 400.0,
                WheelieMaxSpeed = 700.0,
                WheelieOptimalAngle = 0.1,
                WheelieBalanceAngle = 0.4,
                WheelieBalanceTolerance = 0.15,
                WheelieBalanceStrength = 2.0,
                WheelieControlMultiplier = 5.0,
                WheelieStabilizationFactor = 1.0,
                WheelieBalanceResponseFactor = 2.5,
                WheelieEasyTime = 2.0,
                WheelieHardTimeDelta = 3.0,
                WheelieProgressiveDifficulty = 0.4;

            public const double
                StoppieThresholdMinimum = 0.6,
                StoppieBrakeMultiplier = 1.0,
                StoppieForceBase = 0.15,
                StoppieMinSpeed = 300.0,
                StoppieOptimalMinSpeed = 400.0,
                StoppieOptimalMaxSpeed = 700.0,
                StoppieMaxSpeed = 900.0,
                StoppieOptimalAngle = 0.05,
                StoppieMinAngle = 0.2,
                StoppieHeightFactor = 0.4,
                StoppieBalanceAngle = -0.3,
                StoppieBalanceTolerance = 0.08,
                StoppieBalanceStrength = 2.0,
                StoppieControlMultiplier = 3.0,
                StoppieStabilizationFactor = 0.8,
                StoppieBalanceResponseFactor = 2.5,
                StoppieEasyTime = 0.5,
                StoppieHardTimeDelta = 1.0,
                StoppieProgressiveDifficulty = 0.6;
        }

        public static class Motorcycle
        {
            public const double
                DefaultWheelBase = 75.0,
                FullRotation = 2 * Math.PI,
                AirDetectionThreshold = 7.5;

            public const double
                WheelCircumferenceFactor = 2 * Math.PI,
                AirRotationFactor = 12.0,
                GroundRotationFactor = 12.0,
                ThrottleRotationFactor = 6.0,
                SlipThrottleFactor = 1.8;

            public static readonly Point DefaultStartPosition = new(100, 100);
            public static readonly Color DefaultBikeColor = Colors.Red;
        }

        public static class Bike
        {
            public static readonly BikeProperties Standard = new(
                mass: 180.0,
                power: 10000.0,
                brakeForce: 2000.0,
                drag: 0.35,
                maxLeanAngle: Math.PI / 2.5,
                leanSpeed: 6.0,
                friction: 1.0,
                suspensionStrength: 8000.0,
                suspensionDamping: 800.0,
                suspensionRestLength: 25.0,
                maxSuspensionAngle: Math.PI / 12
            );

            public static readonly BikeProperties Sport = new(
                mass: 150.0,
                power: 14000.0,
                brakeForce: 2400.0,
                drag: 0.25,
                maxLeanAngle: Math.PI / 2,
                leanSpeed: 7.0,
                friction: 1.0,
                suspensionStrength: 4500.0,
                suspensionDamping: 450.0,
                suspensionRestLength: 20.0,
                maxSuspensionAngle: Math.PI / 5.14
            );

            public static readonly BikeProperties OffRoad = new(
                mass: 200.0,
                power: 12000.0,
                brakeForce: 1800.0,
                drag: 0.45,
                maxLeanAngle: Math.PI / 3,
                leanSpeed: 5.5,
                friction: 1.0,
                suspensionStrength: 5500.0,
                suspensionDamping: 550.0,
                suspensionRestLength: 30.0,
                maxSuspensionAngle: Math.PI / 7.2
            );
        }

        public static class Wheels
        {
            public static readonly WheelProperties Standard = new(
                radius: 15.0,
                friction: 1.0,
                suspensionStrength: 1.2,
                suspensionDamping: 1.2
            );

            public static readonly WheelProperties Sport = new(
                radius: 14.0,
                friction: 1.0,
                suspensionStrength: 0.9,
                suspensionDamping: 0.8
            );

            public static readonly WheelProperties OffRoad = new(
                radius: 18.0,
                friction: 1.0,
                suspensionStrength: 1.3,
                suspensionDamping: 1.2
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