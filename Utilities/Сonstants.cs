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
                SuspensionProgressiveFactor = 2.0,
                MaxSuspensionCompression = 0.9,
                MinSuspensionCompression = 0.2,
                MaxWheelPenetration = 0.8,
                PenetrationBaseMultiplier = 1.2,
                PenetrationProgressiveFactor = 0.5,
                WheelRadiusHalfFactor = 0.5;

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
                WheelieMinAngle = 0.25,
                WheelieHeightFactor = 0.5,
                WheelieForceMultiplier = 0.2,
                WheelieAngleBoost = 1.0,
                WheelieThrottleThreshold = 0.5;

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
                suspensionRestLength: 25.0
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
                suspensionRestLength: 20.0
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
                suspensionRestLength: 30.0
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