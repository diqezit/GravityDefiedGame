using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Threading;
using static GravityDefiedGame.Utilities.GameConstants;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;
using static GravityDefiedGame.Utilities.GameConstants.Motorcycle;
using static GravityDefiedGame.Utilities.GameConstants.Wheels;
using static GravityDefiedGame.Utilities.GameConstants.Bike;
using static GravityDefiedGame.Utilities.LoggerCore;
using static GravityDefiedGame.Utilities.Logger;
using static System.Math;
using GravityDefiedGame.Utilities;

namespace GravityDefiedGame.Models
{
    [Flags]
    public enum BikeState
    {
        None = 0,
        InAir = 1,
        InWheelie = 2,
        MovingBackward = 4,
        Crashed = 8,
        InStoppie = 16
    }

    public class BikePropertiesProvider : PhysicsComponent
    {
        public static BikeProperties GetBikeProperties(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => Bike.Standard,
            BikeType.Sport => Bike.Sport,
            BikeType.OffRoad => Bike.OffRoad,
            _ => Bike.Standard
        };

        public static (WheelProperties Front, WheelProperties Rear) GetWheelProperties(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => Wheels.Standard,
            BikeType.Sport => Wheels.Sport,
            BikeType.OffRoad => Wheels.OffRoad,
            _ => Wheels.Standard
        };

        public static void InitializeBikeProperties(BikePhysics physics, BikeType bikeType, float wheelBase)
        {
            var props = GetBikeProperties(bikeType);
            var wheelProps = GetWheelProperties(bikeType);

            physics.Mass = props.mass;
            physics.EnginePower = props.power;
            physics.BrakeForce = props.brakeForce;
            physics.DragCoefficient = props.drag;
            physics.MaxLeanAngle = props.maxLeanAngle;
            physics.LeanSpeed = props.leanSpeed;
            physics.FrontGroundFriction = props.friction * wheelProps.Front.friction;
            physics.RearGroundFriction = props.friction * wheelProps.Rear.friction;
            physics.FrontSuspensionStrength = props.suspensionStrength * wheelProps.Front.suspensionStrength;
            physics.RearSuspensionStrength = props.suspensionStrength * wheelProps.Rear.suspensionStrength;
            physics.FrontSuspensionDamping = props.suspensionDamping * wheelProps.Front.suspensionDamping;
            physics.RearSuspensionDamping = props.suspensionDamping * wheelProps.Rear.suspensionDamping;
            physics.SuspensionRestLength = props.suspensionRestLength;
            physics.MaxSuspensionAngle = props.maxSuspensionAngle;
            physics.WheelRadius = wheelProps.Front.radius;
            physics.MomentOfInertia = physics.Mass * (float)Pow(wheelBase / 2, 2) * MomentOfInertiaMultiplier;
            physics.NominalWheelBase = wheelBase;
            physics.MinWheelDistance = physics.NominalWheelBase * WheelDistanceMinRatio;
            physics.MaxWheelDistance = physics.NominalWheelBase * WheelDistanceMaxRatio;

            physics.GroundFriction = physics.FrontGroundFriction;
            physics.SuspensionStrength = physics.FrontSuspensionStrength;
            physics.SuspensionDamping = physics.FrontSuspensionDamping;

#if DEBUG
            Info("BikePhysics", $"Initialized {bikeType} bike with mass: {physics.Mass}, power: {physics.EnginePower}, brake: {physics.BrakeForce}");
#endif
        }
    }

    public class PhysicsValidator : PhysicsComponent
    {
        private const float MaxAllowedPenetration = 5.0f;

        public static void ValidateAndSanitizeBikeState(Motorcycle bike, BikePhysics physics, ILevelPhysics level, float deltaTime, StateComponent stateComponent)
        {
            SanitizeSuspension(bike, physics.SuspensionRestLength);
            bike.Position = SanitizePosition(bike.Position, DefaultStartPosition, "Invalid position detected");
            SanitizeMotion(bike, MaxAngularVelocity, MaxSafeVelocity);
            CheckWheelPenetration(bike, level, physics.WheelRadius);
            stateComponent.UpdateAirTime(deltaTime);
            CheckSuspensionCompression(bike, physics.SuspensionRestLength);
        }

        private static void SanitizeSuspension(Motorcycle bike, float restLength)
        {
            float minOffset = MinSuspensionOffset;
            var offsets = bike.SuspensionOffsets;

            offsets.Front = GetValidOffset(offsets.Front, restLength, minOffset, "front");
            offsets.Rear = GetValidOffset(offsets.Rear, restLength, minOffset, "rear");

            bike.SuspensionOffsets = offsets;
        }

        private static float GetValidOffset(float offset, float restLength, float minOffset, string wheel)
        {
            if (float.IsNaN(offset) || float.IsInfinity(offset))
            {
                Warning("ValidationComponent", $"Invalid {wheel} suspension offset: {offset}");
                return restLength;
            }

            if (offset < minOffset)
            {
                Warning("ValidationComponent", $"{wheel} suspension offset below minimum: {offset} < {minOffset}");
                return minOffset;
            }

            if (offset > restLength)
            {
                Warning("ValidationComponent", $"{wheel} suspension offset exceeds rest length: {offset} > {restLength}");
                return restLength;
            }

            return offset;
        }

        private static void SanitizeMotion(Motorcycle bike, float maxAngularVelocity, float maxVelocity)
        {
            bike.Angle = (float)NormalizeAngle(SanitizeValue(bike.Angle, 0, "Invalid angle value"));
            bike.AngularVelocity = ClampValue(
                SanitizeValue(bike.AngularVelocity, 0, "Invalid angular velocity"),
                -maxAngularVelocity,
                maxAngularVelocity);

            bike.Velocity = SanitizeVector(bike.Velocity, new Vector2(), "Invalid velocity");

            if (bike.Velocity.Length() > maxVelocity)
            {
                Warning("BikePhysics", $"Velocity exceeds max safe value: {bike.Velocity.Length():F2} > {maxVelocity:F2}");
                bike.Velocity = bike.Velocity * (maxVelocity / bike.Velocity.Length());
            }
        }

        private static void CheckWheelPenetration(Motorcycle bike, ILevelPhysics level, float wheelRadius)
        {
            float frontGroundY = level.GetGroundYAtX(bike.WheelPositions.Front.X);
            float rearGroundY = level.GetGroundYAtX(bike.WheelPositions.Rear.X);
            float wheelThreshold = wheelRadius + MaxAllowedPenetration;

            if (bike.WheelPositions.Front.Y > frontGroundY + wheelThreshold ||
                bike.WheelPositions.Rear.Y > rearGroundY + wheelThreshold)
            {
                Warning("BikePhysics", "Wheel penetration exceeded maximum allowed value");
                float centerGroundY = level.GetGroundYAtX(bike.Position.X);
                bike.Position = new Vector2(bike.Position.X, centerGroundY - wheelRadius);
                bike.Velocity = new Vector2(bike.Velocity.X, 0);
            }
        }

        private static void CheckSuspensionCompression(Motorcycle bike, float restLength)
        {
            float threshold = HighSuspensionCompressionThreshold;
            var offsets = bike.SuspensionOffsets;

            float frontCompression = 1.0f - SafeDivide(offsets.Front, restLength);
            float rearCompression = 1.0f - SafeDivide(offsets.Rear, restLength);

            if (frontCompression > threshold)
            {
                Warning("ValidationComponent", $"High front suspension compression: {frontCompression:P0}");
            }

            if (rearCompression > threshold)
            {
                Warning("ValidationComponent", $"High rear suspension compression: {rearCompression:P0}");
            }
        }
    }

    public class FramePhysics : PhysicsComponent
    {
        public static List<Vector2> GetFramePoints(Vector2 position, float angle, float wheelBase, float frameHeight)
        {
            var (cosAngle, sinAngle) = GetTrigsFromAngle(angle);
            float halfWheelBase = wheelBase / 2;
            frameHeight *= 0.8f;

            return new List<Vector2>
            {
                new Vector2(
                    position.X + halfWheelBase * 0.7f * cosAngle - frameHeight * sinAngle,
                    position.Y + halfWheelBase * 0.7f * sinAngle + frameHeight * cosAngle
                ),
                new Vector2(
                    position.X - halfWheelBase * 0.7f * cosAngle - frameHeight * sinAngle,
                    position.Y - halfWheelBase * 0.7f * sinAngle + frameHeight * cosAngle
                )
            };
        }

        public static void CheckCrashConditions(Motorcycle bike, float minWheelDistance, float maxWheelDistance)
        {
            if (bike.IsInAir || bike.IsCrashed)
                return;

            if (Abs(bike.Angle) > CriticalLeanAngle)
            {
                bike.State |= BikeState.Crashed;
#if DEBUG
                Info("BikePhysics", $"Bike crashed due to critical lean angle: {bike.Angle:F2}");
#endif
                return;
            }

            float currentDistance = CalculateDistance(bike.WheelPositions.Front, bike.WheelPositions.Rear);
            if (currentDistance < minWheelDistance || currentDistance > maxWheelDistance)
            {
                bike.State |= BikeState.Crashed;
#if DEBUG
                Info("BikePhysics", $"Bike crashed due to invalid wheel distance: {currentDistance:F2}");
#endif
            }
        }

        public static (bool IsCollision, Vector2 CollisionPoint, float MaxPenetration) DetectFrameCollision(List<Vector2> framePoints, ILevelPhysics level)
        {
            bool frameCollision = false;
            Vector2 collisionPoint = default;
            float maxPenetration = 0;

            foreach (var point in framePoints)
            {
                float groundY = level.GetGroundYAtX(point.X);
                float penetration = point.Y - groundY;

                if (penetration > 0 && penetration > maxPenetration)
                {
                    frameCollision = true;
                    maxPenetration = penetration;
                    collisionPoint = point;
                }
            }

            return (frameCollision, collisionPoint, maxPenetration);
        }
    }

    public class LeanController : PhysicsComponent
    {
        private float _prevSlopeAngle;

        public LeanController()
        {
            _prevSlopeAngle = 0;
        }

        public void UpdateLean(Motorcycle bike, BikePhysics physics, float deltaTime, ILevelPhysics level)
        {
            if (bike.IsInAir)
            {
                UpdateAirLean(bike, deltaTime);
                return;
            }

            UpdateGroundLean(bike, physics, deltaTime, level);
        }

        private void UpdateAirLean(Motorcycle bike, float deltaTime)
        {
            float desiredAngularVelocity = bike.LeanAmount * MaxAirAngularVelocity;
            bike.AngularVelocity = MathHelper.Lerp(bike.AngularVelocity, desiredAngularVelocity, AirDampingFactor * deltaTime);
        }

        private void UpdateGroundLean(Motorcycle bike, BikePhysics physics, float deltaTime, ILevelPhysics level)
        {
            float currentSlopeAngle = level.CalculateSlopeAngle(bike.Position.X);
            float angleDifference = Math.Abs(currentSlopeAngle - _prevSlopeAngle);
            float smoothedSlopeAngle = angleDifference > SlopeAngleSmoothingThreshold
                ? MathHelper.Lerp(_prevSlopeAngle, currentSlopeAngle, SlopeTransitionRate * deltaTime)
                : currentSlopeAngle;
            _prevSlopeAngle = smoothedSlopeAngle;

            float speed = bike.Velocity.Length();
            float speedFactor = MathHelper.Min(1.0f, speed / LeanSpeedFactorDenominator);
            float adaptiveLeanSpeed = physics.LeanSpeed * (LeanSpeedBaseMultiplier + LeanSpeedFactorMultiplier * speedFactor);
            float terrainAdaptationFactor = MathHelper.Min(1.0f, speed / TerrainAdaptationSpeedThreshold);
            float slopeInfluence = smoothedSlopeAngle * terrainAdaptationFactor;

            float targetLean = bike.LeanAmount * physics.MaxLeanAngle + slopeInfluence + bike.Throttle * ThrottleLeanInfluence;
            float leanError = targetLean - bike.Angle;
            float angularVelocityDamping = AngularVelocityDampingBase + AngularVelocityDampingFactor * speedFactor;

            float controlTorque = LeanControlTorqueMultiplier * leanError * adaptiveLeanSpeed -
                               angularVelocityDamping * bike.AngularVelocity;

            if (bike.Throttle < InputIdleThreshold && bike.Brake < InputIdleThreshold &&
                Math.Abs(bike.AngularVelocity) > AngularVelocityIdleThreshold)
            {
                float stabilizationFactor = StabilizationFactorBase +
                    StabilizationFactorSpeedMultiplier * MathHelper.Min(1.0f, speed / StabilizationSpeedThreshold);
                controlTorque -= bike.AngularVelocity * stabilizationFactor * StabilizationTorqueMultiplier;
            }

            bike.AngularVelocity += controlTorque / physics.MomentOfInertia * deltaTime;
            float maxAngularVel = MaxAngularVelocity * GroundAngularVelocityFactor;
            bike.AngularVelocity = ClampValue(bike.AngularVelocity, -maxAngularVel, maxAngularVel);
        }
    }

    public class ForcesComponent : PhysicsComponent
    {
        private readonly IBikePhysicsData _bike;
        private readonly BikePhysics _physics;
        private float _prevThrottle, _prevBrake;

        public ForcesComponent(IBikePhysicsData bike, BikePhysics physics) : base() =>
            (_bike, _physics, _prevThrottle, _prevBrake) = (bike, physics, 0f, 0f);

        public Vector2 CalculateThrustForce(float deltaTime)
        {
            _prevThrottle += (_bike.Throttle - _prevThrottle) * MathHelper.Min(1.0f, ThrottleTransitionRate * deltaTime);
            float force = _prevThrottle * _physics.EnginePower;
            var (cos, sin) = _physics.GetBikeTrigs();
            return new Vector2((float)(cos * force), (float)(sin * force));
        }

        public Vector2 CalculateBrakeForce(float deltaTime)
        {
            if (_bike.Brake <= MinBrakeInput || _bike.Velocity.Length() <= 0)
            {
                _prevBrake = MathHelper.Max(0f, _prevBrake - BrakeTransitionRate * deltaTime);
                return _prevBrake <= BrakeTransitionThreshold ? new Vector2() :
                    -_bike.Velocity * (_prevBrake * _physics.BrakeForce / _bike.Velocity.Length());
            }

            _prevBrake += (_bike.Brake - _prevBrake) * MathHelper.Min(1.0f, BrakeTransitionRate * deltaTime);
            return -_bike.Velocity * (_prevBrake * _physics.BrakeForce / _bike.Velocity.Length());
        }

        public Vector2 CalculateDragForce()
        {
            float speed = _bike.Velocity.Length();
            if (speed <= 0) return new Vector2();

            float drag = _physics.DragCoefficient * (_bike.IsInAir ? AirFrictionMultiplier : 1.0f);
            if (!_bike.IsInAir && _bike.Throttle == 0 && _bike.Brake == 0)
                drag *= _prevThrottle > DragThrottleThreshold ?
                       DragThrottleMultiplierBase + _prevThrottle * DragThrottleMultiplier :
                       DragIdleMultiplier;

            return -_bike.Velocity * (drag * speed);
        }

        public Vector2 CalculateSlopeForce(ILevelPhysics level)
        {
            if (_bike.IsInAir) return new Vector2();

            float frontAngle = level.CalculateSlopeAngle(_bike.WheelPositions.Front.X);
            float rearAngle = level.CalculateSlopeAngle(_bike.WheelPositions.Rear.X);

            Vector2 frontForce = CalculateWheelSlopeForce(frontAngle);
            Vector2 rearForce = CalculateWheelSlopeForce(rearAngle);
            return (frontForce + rearForce) * 0.5f;
        }

        private Vector2 CalculateWheelSlopeForce(float slopeAngle)
        {
            float forceMagnitude = _physics.Mass * _physics.Gravity * (float)Sin(slopeAngle) * 0.5f;
            return new Vector2(
                forceMagnitude * (float)Cos(slopeAngle),
                -forceMagnitude * (float)Sin(slopeAngle)
            );
        }

        public Vector2 CalculateTotalForce(ILevelPhysics level, float deltaTime) =>
            new Vector2(0, _physics.Gravity * _physics.Mass) +
            CalculateThrustForce(deltaTime) +
            CalculateBrakeForce(deltaTime) +
            CalculateDragForce() +
            CalculateSlopeForce(level);
    }

    public class TorqueComponent : PhysicsComponent
    {
        private readonly IBikePhysicsData _bike;
        private readonly BikePhysics _physics;
        private float _prevTorque;

        private readonly record struct TrickParameters(
            bool IsWheelie,
            float InputValue,
            float MinThreshold,
            float ForceMultiplier,
            float BaseForce,
            float MaxForce,
            Func<float> SpeedEfficiencyFunc,
            Func<float> AngleEfficiencyFunc,
            Func<float> BalanceFactorFunc,
            Action<float> UpdateBalanceAction);

        public TorqueComponent(IBikePhysicsData bike, BikePhysics physics) : base() =>
            (_bike, _physics, _prevTorque) = (bike, physics, 0);

        public float CalculateBaseTorque(float deltaTime)
        {
            if (_bike.Throttle <= 0 && _prevTorque <= TorqueIdleThreshold) return 0f;

            float force = _bike.Throttle * _physics.EnginePower;
            var (cos, sin) = _physics.GetBikeTrigs();
            Vector2 thrust = new((float)(cos * force), (float)(sin * force));
            Vector2 r = _bike.WheelPositions.Rear - _bike.Position;
            float torque = r.X * thrust.Y - r.Y * thrust.X;

            float rate = MathHelper.Min(1.0f, TorqueSmoothingFactor * deltaTime);
            _prevTorque = _bike.Throttle <= TorqueFadeThreshold && _prevTorque > 0 ?
                _prevTorque * (1.0f - rate) :
                _prevTorque + (torque - _prevTorque) * rate;

            return _prevTorque;
        }

        public float CalculateWheelieTorque(float deltaTime) =>
            CalculateTrickTorque(deltaTime, new TrickParameters(
                IsWheelie: true,
                InputValue: _bike.Throttle,
                MinThreshold: WheelieThrottleMinimum,
                ForceMultiplier: WheelieThrottleMultiplier * _physics.EnginePower,
                BaseForce: WheelieForceBase,
                MaxForce: _physics.EnginePower * WheelieMaxForceMultiplier,
                SpeedEfficiencyFunc: CalculateWheelieSpeedEfficiency,
                AngleEfficiencyFunc: CalculateWheelieAngleEfficiency,
                BalanceFactorFunc: CalculateWheelieBalanceFactor,
                UpdateBalanceAction: UpdateWheelieBalance
            ));

        public float CalculateStoppieTorque(float deltaTime) =>
            CalculateTrickTorque(deltaTime, new TrickParameters(
                IsWheelie: false,
                InputValue: _bike.Brake,
                MinThreshold: StoppieThresholdMinimum,
                ForceMultiplier: StoppieBrakeMultiplier * _physics.BrakeForce,
                BaseForce: StoppieForceBase,
                MaxForce: _physics.BrakeForce * StoppieMaxForceMultiplier,
                SpeedEfficiencyFunc: CalculateStoppieSpeedEfficiency,
                AngleEfficiencyFunc: CalculateStoppieAngleEfficiency,
                BalanceFactorFunc: CalculateStoppieBalanceFactor,
                UpdateBalanceAction: UpdateStoppieBalance
            ));

        private float CalculateTrickTorque(float deltaTime, TrickParameters parameters)
        {
            if (_bike.IsInAir || parameters.InputValue <= parameters.MinThreshold) return 0f;

            string trickName = parameters.IsWheelie ? "wheelie" : "stoppie";
            bool isInTrickState = parameters.IsWheelie ? _bike.IsInWheelie : _bike.IsInStoppie;
            float torque = 0;
            float speed = _bike.Velocity.Length();

            bool validSpeed = parameters.IsWheelie ? speed > 0 : speed > StoppieMinSpeed;
            if (validSpeed)
            {
                float efficiency = parameters.SpeedEfficiencyFunc() * parameters.AngleEfficiencyFunc();
                float force = (parameters.InputValue - parameters.MinThreshold) *
                              parameters.ForceMultiplier * parameters.BaseForce * efficiency;
                force = SanitizeValue(force, 0.0f, "Invalid trick force calculated");

                if (force > parameters.MaxForce)
                {
                    Warning("TorqueComponent", $"{trickName} force exceeded max limit: {force} > {parameters.MaxForce}");
                    force = parameters.MaxForce;
                }

                if (isInTrickState) force *= parameters.BalanceFactorFunc();
                torque += parameters.IsWheelie ? force : -force;
            }

            if (isInTrickState)
            {
                parameters.UpdateBalanceAction(deltaTime);
                float controlMult = parameters.IsWheelie ? WheelieControlMultiplier : StoppieControlMultiplier;
                float balanceStrength = parameters.IsWheelie ? WheelieBalanceStrength : StoppieBalanceStrength;
                float balanceValue = parameters.IsWheelie ? _physics.WheelieBalance : _physics.StoppieBalance;

                float control = _bike.LeanAmount * controlMult;
                float balance = balanceValue * balanceStrength + (parameters.IsWheelie ? control : -control);
                balance = SanitizeValue(balance, 0.0f, "Invalid balance torque calculated");

                if (parameters.IsWheelie && _bike.Brake > 0)
                    balance += _bike.Brake * WheelieStabilizationFactor;
                else if (!parameters.IsWheelie && _bike.Throttle > 0)
                    balance -= _bike.Throttle * StoppieStabilizationFactor;

                torque += balance;
            }

            return torque;
        }

        private static float CalculateEfficiency(
            float value, float minOptimal, float maxOptimal,
            float minValue, float maxValue, float lowEff, float highEff)
        {
            if (value < minValue) return 0.0f;
            if (value < minOptimal) return MathHelper.Lerp(lowEff, 1.0f, (value - minValue) / (minOptimal - minValue));
            if (value > maxOptimal) return MathHelper.Lerp(1.0f, highEff, (value - maxOptimal) / (maxValue - maxOptimal));
            return 1.0f;
        }

        private float CalculateWheelieSpeedEfficiency() =>
            CalculateEfficiency(
                _bike.Velocity.Length(),
                WheelieOptimalMinSpeed,
                WheelieOptimalMaxSpeed,
                0,
                WheelieMaxSpeed,
                WheelieLowSpeedEfficiency,
                WheelieHighSpeedEfficiency);

        private float CalculateStoppieSpeedEfficiency() =>
            CalculateEfficiency(
                _bike.Velocity.Length(),
                StoppieOptimalMinSpeed,
                StoppieOptimalMaxSpeed,
                StoppieMinSpeed,
                StoppieMaxSpeed,
                StoppieLowSpeedEfficiency,
                StoppieHighSpeedEfficiency);

        private float CalculateWheelieAngleEfficiency() =>
            CalculateEfficiency(
                _bike.Angle,
                -WheelieOptimalAngle,
                0,
                -WheelieOptimalAngle - WheelieAngleMinOffset,
                WheelieAngleMaxOffset,
                WheelieLowAngleEfficiency,
                WheelieHighAngleEfficiency);

        private float CalculateStoppieAngleEfficiency() =>
            CalculateEfficiency(
                _bike.Angle,
                -StoppieOptimalAngle,
                0,
                -StoppieOptimalAngle - StoppieAngleMinOffset,
                StoppieAngleMaxOffset,
                StoppieLowAngleEfficiency,
                StoppieHighAngleEfficiency);

        private float CalculateWheelieBalanceFactor()
        {
            float diff = (float)Abs(_bike.Angle - WheelieBalanceAngle);
            return diff < WheelieBalanceTolerance ?
                   MathHelper.Lerp(WheelieBalanceMinFactor, 1.0f, diff / WheelieBalanceTolerance) :
                   1.0f;
        }

        private float CalculateStoppieBalanceFactor()
        {
            float diff = (float)Abs(_bike.Angle - StoppieBalanceAngle);
            return diff < StoppieBalanceTolerance ?
                   MathHelper.Lerp(StoppieBalanceMinFactor, 1.0f, diff / StoppieBalanceTolerance) :
                   1.0f;
        }

        private void UpdateWheelieBalance(float deltaTime)
        {
            float diff = _bike.Angle - WheelieBalanceAngle;
            _physics.WheelieBalance = SanitizeValue(
                -diff * WheelieBalanceResponseFactor,
                0.0f,
                "Invalid wheelie balance value calculated");

            if (_bike is Motorcycle motorcycle)
            {
                motorcycle.WheelieTime = SanitizeValue(
                    motorcycle.WheelieTime + deltaTime,
                    0.0f,
                    "Invalid wheelie time value");

                if (motorcycle.WheelieTime > WheelieEasyTime)
                {
                    float factor = MathHelper.Min(1.0f, (motorcycle.WheelieTime - WheelieEasyTime) / WheelieHardTimeDelta);
                    _physics.WheelieBalance *= (1.0f - factor * WheelieProgressiveDifficulty);
                }
            }
        }

        private void UpdateStoppieBalance(float deltaTime)
        {
            float diff = _bike.Angle - StoppieBalanceAngle;
            _physics.StoppieBalance = SanitizeValue(
                -diff * StoppieBalanceResponseFactor,
                0.0f,
                "Invalid stoppie balance value calculated");

            if (_bike is Motorcycle motorcycle)
            {
                motorcycle.StoppieTime = SanitizeValue(
                    motorcycle.StoppieTime + deltaTime,
                    0.0f,
                    "Invalid stoppie time value");

                if (motorcycle.StoppieTime > StoppieEasyTime)
                {
                    float factor = MathHelper.Min(1.0f, (motorcycle.StoppieTime - StoppieEasyTime) / StoppieHardTimeDelta);
                    _physics.StoppieBalance *= (1.0f - factor * StoppieProgressiveDifficulty);
                }
            }
        }

        public float CalculateTotalTorque(float deltaTime) =>
            CalculateBaseTorque(deltaTime) +
            CalculateWheelieTorque(deltaTime) +
            CalculateStoppieTorque(deltaTime);
    }

    public class TricksComponent : PhysicsComponent
    {
        private readonly IBikePhysicsData _bike;
        private readonly BikePhysics _physics;

        public TricksComponent(IBikePhysicsData bike, BikePhysics physics) =>
            (_bike, _physics) = (bike, physics);

        public void UpdateTrickStates(float deltaTime)
        {
            if (_bike is Motorcycle motorcycle)
            {
                bool wasInWheelie = motorcycle.IsInWheelie;
                bool wasInStoppie = motorcycle.IsInStoppie;

                motorcycle.IsInWheelie = !motorcycle.IsInAir &&
                    motorcycle.WheelPositions.Front.Y < motorcycle.WheelPositions.Rear.Y - _physics.WheelRadius * WheelieHeightFactor &&
                    motorcycle.Angle > WheelieMinAngle;

                motorcycle.IsInStoppie = !motorcycle.IsInAir &&
                    motorcycle.WheelPositions.Rear.Y < motorcycle.WheelPositions.Front.Y - _physics.WheelRadius * StoppieHeightFactor &&
                    motorcycle.Angle < -StoppieMinAngle;

                float wheelieTime = motorcycle.WheelieTime;
                float stoppieTime = motorcycle.StoppieTime;

                UpdateTrickTime(wasInWheelie, motorcycle.IsInWheelie, ref wheelieTime);
                UpdateTrickTime(wasInStoppie, motorcycle.IsInStoppie, ref stoppieTime);

                motorcycle.WheelieTime = wheelieTime;
                motorcycle.StoppieTime = stoppieTime;

#if DEBUG
                LogTrickChange(wasInWheelie, motorcycle.IsInWheelie, "Wheelie", wheelieTime);
                LogTrickChange(wasInStoppie, motorcycle.IsInStoppie, "Stoppie", stoppieTime);
#endif
            }
        }

#if DEBUG
        private void LogTrickChange(bool wasActive, bool isActive, string trickName, float duration)
        {
            if (isActive && !wasActive)
                Info("TricksComponent", $"{trickName} started");
            else if (!isActive && wasActive)
                Info("TricksComponent", $"{trickName} ended, duration: {duration:F2}s");
        }
#endif

        private static void UpdateTrickTime(bool wasActive, bool isActive, ref float trickTime)
        {
            if (!isActive && wasActive)
            {
                trickTime = 0;
            }
        }
    }

    public class SuspensionComponent : PhysicsComponent
    {
        private readonly IBikePhysicsData _bike;
        private readonly BikePhysics _physics;

        private struct SuspensionState
        {
            public float FrontPenetration;
            public float RearPenetration;
            public float FrontCompression;
            public float RearCompression;
            public Vector2 FrontReactionForce;
            public Vector2 RearReactionForce;
        }

        private SuspensionState _state = new()
        {
            FrontReactionForce = new Vector2(),
            RearReactionForce = new Vector2()
        };

        private const float SlopeGripReductionFactor = 0.5f;
        private const float MinGripThreshold = 0.1f;
        private const float MinPenetrationThreshold = 0.01f;
        private const float MinNormalY = 0.2f;

        public SuspensionComponent(IBikePhysicsData bike, BikePhysics physics) : base() =>
            (_bike, _physics) = (bike, physics);

        private Vector2 CalculateSurfaceNormal(ILevelPhysics level, float x)
        {
            float slopeAngle = level.CalculateSlopeAngle(x);
            Vector2 normal = new((float)(-Sin(slopeAngle)), (float)(Cos(slopeAngle)));

            if (normal.Y < 0)
            {
                normal = new(-normal.X, -normal.Y);
            }

            if (Abs(normal.Y) < MinNormalY)
            {
                float normalLength = normal.Length();
                float newY = MinNormalY;
                float newX = (float)(Sign(normal.X) * Sqrt(normalLength * normalLength - newY * newY));
                normal = new(newX, newY);
            }

            return normal;
        }

        private float CalculateGripFactor(float slopeAngle, float groundFriction)
        {
            float grip = groundFriction * (1.0f - (float)Abs(Sin(slopeAngle)) * SlopeGripReductionFactor);
            return MathHelper.Max(grip, MinGripThreshold);
        }

        public void HandleWheelCollisions(ILevelPhysics level, float deltaTime)
        {
            float frontGroundY = level.GetGroundYAtX(_bike.WheelPositions.Front.X);
            float rearGroundY = level.GetGroundYAtX(_bike.WheelPositions.Rear.X);

            Vector2 frontNormal = CalculateSurfaceNormal(level, _bike.WheelPositions.Front.X);
            Vector2 rearNormal = CalculateSurfaceNormal(level, _bike.WheelPositions.Rear.X);

            bool frontContact = HandleWheelCollision(true, frontGroundY, frontNormal, level, deltaTime);
            bool rearContact = HandleWheelCollision(false, rearGroundY, rearNormal, level, deltaTime);

            if (_bike is Motorcycle motorcycle)
            {
                bool wasInAir = motorcycle.IsInAir;
                motorcycle.State = (frontContact || rearContact)
                    ? motorcycle.State & ~BikeState.InAir
                    : motorcycle.State | BikeState.InAir;

#if DEBUG
                if (wasInAir && !motorcycle.IsInAir)
                    Info("SuspensionComponent", "Landed");
                else if (!wasInAir && motorcycle.IsInAir)
                    Info("SuspensionComponent", "Became airborne");
#endif
            }
        }

        public bool HandleWheelCollision(bool isFrontWheel, float groundY, Vector2 normal, ILevelPhysics level, float deltaTime)
        {
            Vector2 wheelPos = isFrontWheel ? _bike.WheelPositions.Front : _bike.WheelPositions.Rear;
            float penetration = wheelPos.Y + _physics.WheelRadius - groundY;

            if (penetration <= 0)
            {
                SetSuspensionOffset(isFrontWheel, _physics.SuspensionRestLength);
                return false;
            }

            ProcessWheelPenetration(isFrontWheel, wheelPos, penetration, normal, level, deltaTime);
            return true;
        }

        private void ProcessWheelPenetration(bool isFrontWheel, Vector2 wheelPos, float penetration, Vector2 normal, ILevelPhysics level, float deltaTime)
        {
            float suspensionStrength = isFrontWheel ? _physics.FrontSuspensionStrength : _physics.RearSuspensionStrength;
            float suspensionDamping = isFrontWheel ? _physics.FrontSuspensionDamping : _physics.RearSuspensionDamping;
            float groundFriction = isFrontWheel ? _physics.FrontGroundFriction : _physics.RearGroundFriction;

            float prevPenetration = isFrontWheel ? _state.FrontPenetration : _state.RearPenetration;
            float penetrationVelocity = (penetration - prevPenetration) / deltaTime;
            float relativeVelocity = Vector2.Dot(_bike.Velocity, normal);

            if (isFrontWheel)
                _state.FrontPenetration = penetration;
            else
                _state.RearPenetration = penetration;

            if (penetration < MinPenetrationThreshold)
            {
                SetSuspensionOffset(isFrontWheel, _physics.SuspensionRestLength);
                return;
            }

            float slopeAngle = (float)Atan2(normal.Y, normal.X) - MathHelper.Pi;
            float gripFactor = CalculateGripFactor(slopeAngle, groundFriction);

            float CalculateCompression()
            {
                if (penetration > _physics.WheelRadius * MaxWheelPenetration)
                {
                    string wheel = isFrontWheel ? "front" : "rear";
                    Warning("SuspensionComponent", $"Excessive {wheel} wheel penetration: {penetration:F2}");
                    penetration = MathHelper.Min(penetration, _physics.WheelRadius * MaxWheelPenetration);
                }

                float baseCompression = penetration * BaseCompressionMultiplier;
                float progressiveFactor = ProgressiveFactorBase +
                    (float)Pow(penetration / (_physics.WheelRadius * WheelRadiusHalfFactor), 2) * ProgressiveFactorMultiplier;
                float desiredCompression = baseCompression * progressiveFactor;

                float prevCompression = isFrontWheel ? _state.FrontCompression : _state.RearCompression;
                float smoothedCompression = MathHelper.Lerp(prevCompression, desiredCompression, CompressionSmoothingFactor);

                if (isFrontWheel)
                    _state.FrontCompression = smoothedCompression;
                else
                    _state.RearCompression = smoothedCompression;

                return smoothedCompression;
            }

            float CalculateSuspensionOffset(float compression)
            {
                float newOffset = _physics.SuspensionRestLength - compression;
                float currentOffset = isFrontWheel ? _bike.SuspensionOffsets.Front : _bike.SuspensionOffsets.Rear;

                if (Abs(newOffset - currentOffset) > _physics.SuspensionRestLength * LargeSuspensionChangeThreshold)
                    newOffset = MathHelper.Lerp(currentOffset, newOffset, LargeSuspensionChangeSmoothingFactor);

                return ClampValue(newOffset, _physics.SuspensionRestLength * MinSuspensionOffset, _physics.SuspensionRestLength);
            }

            Vector2 CalculateReactionForce(float newOffset)
            {
                float compressionRatio = CompressionRatioBase - newOffset / _physics.SuspensionRestLength;
                float progressiveFactorForce = ProgressiveFactorBase +
                    (float)Pow(compressionRatio, 2) * ProgressiveFactorForceMultiplier;
                float adjustedSuspensionStrength = suspensionStrength * progressiveFactorForce;

                float penetrationDamping = -suspensionDamping * penetrationVelocity;
                float velocityDamping = -suspensionDamping * relativeVelocity * VelocityDampingFactor;

                Vector2 suspensionForce = normal * (-adjustedSuspensionStrength * penetration);
                Vector2 dampingForce = normal * (penetrationDamping + velocityDamping);

                Vector2 tangent = new(-normal.Y, normal.X);
                float tangentialVelocity = Vector2.Dot(_bike.Velocity, tangent);
                float maxFriction = gripFactor * suspensionForce.Length();

                Vector2 frictionForce = tangent * (-tangentialVelocity * groundFriction * FrictionForceMultiplier);
                bool isSlipping = frictionForce.Length() > maxFriction;

#if DEBUG
                if (isSlipping)
                {
                    string wheel = isFrontWheel ? "Front" : "Rear";
                    Info("SuspensionComponent", $"{wheel} wheel slipping");
                }
#endif

                if (isSlipping)
                {
                    frictionForce = frictionForce * (maxFriction / frictionForce.Length());
                }

                return suspensionForce + dampingForce + frictionForce;
            }

            void ApplyReactionForce(Vector2 reactionForce)
            {
                Vector2 prevReactionForce = isFrontWheel ? _state.FrontReactionForce : _state.RearReactionForce;
                reactionForce = LerpVector(prevReactionForce, reactionForce, ReactionForceSmoothingFactor);

                if (isFrontWheel)
                    _state.FrontReactionForce = reactionForce;
                else
                    _state.RearReactionForce = reactionForce;

                Vector2 r = wheelPos - _bike.Position;
                float torque = r.X * reactionForce.Y - r.Y * reactionForce.X;

                if (_bike is Motorcycle motorcycle)
                {
                    motorcycle.Velocity += reactionForce * (deltaTime / _physics.Mass);
                    motorcycle.AngularVelocity += torque / _physics.MomentOfInertia * deltaTime;
                }
            }

            float compression = CalculateCompression();
            float newOffset = CalculateSuspensionOffset(compression);
            Vector2 reactionForce = CalculateReactionForce(newOffset);
            ApplyReactionForce(reactionForce);
            SetSuspensionOffset(isFrontWheel, newOffset);
        }

        private void SetSuspensionOffset(bool isFrontWheel, float offset)
        {
            if (_bike is Motorcycle motorcycle)
            {
                var current = motorcycle.SuspensionOffsets;
                motorcycle.SuspensionOffsets = isFrontWheel ? (offset, current.Rear) : (current.Front, offset);
            }
        }
    }

    public class CollisionComponent : PhysicsComponent
    {
        private readonly IBikePhysicsData _bike;
        private readonly BikePhysics _physics;

        public CollisionComponent(IBikePhysicsData bike, BikePhysics physics) : base() =>
            (_bike, _physics) = (bike, physics);

        public void CheckFrameCollision(ILevelPhysics level, float deltaTime)
        {
            if (_bike.IsInAir || _bike.IsCrashed || _bike.Velocity.Length() <= FrameCollisionMinVelocity)
                return;

            var framePoints = _physics.GetFramePoints();
            var collisionInfo = FramePhysics.DetectFrameCollision(framePoints, level);

            if (collisionInfo.IsCollision && collisionInfo.MaxPenetration > _physics.WheelRadius * FrameCollisionMinPenetration)
                HandleFrameCollision(collisionInfo, deltaTime);
        }

        private void HandleFrameCollision((bool IsCollision, Vector2 CollisionPoint, float MaxPenetration) collisionInfo, float deltaTime)
        {
            float crashThreshold = _physics.WheelRadius * FrameCrashThreshold;
            bool isBadAngle = Abs(_bike.Angle) > FrameCriticalBackwardTiltAngle;
            bool isHighSpeed = _bike.Velocity.Length() > FrameCollisionHighSpeedThreshold;

            if (_bike is Motorcycle motorcycle)
            {
                if (collisionInfo.MaxPenetration > crashThreshold && (isHighSpeed || isBadAngle))
                {
                    motorcycle.State |= BikeState.Crashed;
#if DEBUG
                    Info("CollisionComponent", "Bike crashed due to frame collision");
#endif
                }
                else
                {
                    ApplyFrameCollisionResponse(collisionInfo.CollisionPoint, collisionInfo.MaxPenetration, deltaTime);
                }
            }
        }

        private void ApplyFrameCollisionResponse(Vector2 _, float penetration, float deltaTime)
        {
            if (_bike is Motorcycle motorcycle)
            {
                float reactionForce = penetration * _physics.SuspensionStrength * FrameCollisionReactionForce;
                float impulse = reactionForce * deltaTime;
                float deltaVelocityY = -impulse / _physics.Mass;

                deltaVelocityY = MathHelper.Max(deltaVelocityY, -FrameCollisionMaxDeltaVelocity * deltaTime);
                motorcycle.Velocity = new(motorcycle.Velocity.X, motorcycle.Velocity.Y + deltaVelocityY);

                float stabilizingFactor = Abs(motorcycle.Angle) > FrameStabilizingAngleThreshold
                    ? FrameStabilizingFactorStrong
                    : FrameStabilizingFactorBase;

                float stabilizingTorque = -motorcycle.Angle * stabilizingFactor;
                float deltaAngularVelocity = stabilizingTorque * deltaTime / _physics.MomentOfInertia;

                deltaAngularVelocity = ClampValue(
                    deltaAngularVelocity,
                    -FrameCollisionMaxDeltaAngular * deltaTime,
                    FrameCollisionMaxDeltaAngular * deltaTime);

                motorcycle.AngularVelocity += deltaAngularVelocity;
            }
        }
    }

    public class StateComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;
        private float _airTime;

        public StateComponent(Motorcycle bike, BikePhysics physics) =>
            (_bike, _physics, _airTime) = (bike, physics, 0);

        public void Reset()
        {
            _bike.Position = DefaultStartPosition;
            _bike.Velocity = new Vector2();
            _bike.Throttle = _bike.Brake = _bike.LeanAmount = 0;
            _bike.State = BikeState.None;
            _bike.WasInAir = false;
            _bike.Angle = _bike.AngularVelocity = 0;
            _bike.WheelRotations = (0, 0);
            _bike.SuspensionOffsets = (_physics.SuspensionRestLength, _physics.SuspensionRestLength);
            _airTime = _bike.WheelieTime = _bike.StoppieTime = 0;
            _bike.FrontWheelAngularVelocity = 0;
            _bike.RearWheelAngularVelocity = 0;
        }

        public bool ShouldSkipUpdate(CancellationToken token) =>
            _bike.IsCrashed || token.IsCancellationRequested;

        public void SavePreviousState() => _bike.WasInAir = _bike.IsInAir;

        public void HandleUpdateException(Exception ex)
        {
            Error("StateComponent", $"Update exception: {ex.Message}");
            _bike.IsCrashed = true;
        }

        public void LogStateChanges()
        {
            if (_bike.IsInAir || !_bike.WasInAir || _airTime <= SignificantAirTimeThreshold)
                return;

#if DEBUG
            Info("StateComponent", $"Air time: {_airTime:F2}s");
#endif
            _airTime = 0;
        }

        public void UpdateAirTime(float deltaTime)
        {
            if (_bike.IsInAir)
                _airTime += deltaTime;
        }
    }

    public class InputComponent : PhysicsComponent
    {
        private readonly IBikePhysicsData _bike;

        public InputComponent(IBikePhysicsData bike) => _bike = bike;

        public void ApplyThrottle(float amount)
        {
            if (_bike is Motorcycle motorcycle)
            {
                motorcycle.Throttle = ClampValue(amount, 0, 1);
            }
        }

        public void ApplyBrake(float amount)
        {
            if (_bike is Motorcycle motorcycle)
            {
                motorcycle.Brake = ClampValue(amount, 0, 1);
            }
        }

        public void Lean(float direction)
        {
            if (_bike is Motorcycle motorcycle)
            {
                motorcycle.LeanAmount = ClampValue(direction, -1, 1);
            }
        }
    }

    public class KinematicsComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;
        private ILevelPhysics _level;

        public KinematicsComponent(Motorcycle bike, BikePhysics physics) =>
            (_bike, _physics) = (bike, physics);

        public void SetLevel(ILevelPhysics level) => _level = level;

        public void UpdateKinematics(float deltaTime)
        {
            UpdateAttachmentPoints();
            UpdateWheelPositions();
            UpdateWheelRotations(deltaTime);
            _bike.Position += _bike.Velocity * deltaTime;
        }

        public void UpdateAttachmentPoints()
        {
            var (cosAngle, sinAngle) = GetTrigsFromAngle(_bike.Angle);
            float halfWheelBase = _bike.WheelBase / 2;

            _bike.AttachmentPoints = (
                new Vector2(
                    _bike.Position.X + halfWheelBase * cosAngle,
                    _bike.Position.Y + halfWheelBase * sinAngle
                ),
                new Vector2(
                    _bike.Position.X - halfWheelBase * cosAngle,
                    _bike.Position.Y - halfWheelBase * sinAngle
                )
            );
        }

        public void UpdateWheelPositions()
        {
            if (_level is null)
            {
                UpdateWheelPositionsDefault();
                return;
            }

            float frontGroundY = _level.GetGroundYAtX(_bike.AttachmentPoints.Front.X);
            float rearGroundY = _level.GetGroundYAtX(_bike.AttachmentPoints.Rear.X);

            float frontAngle = _level.CalculateSlopeAngle(_bike.AttachmentPoints.Front.X);
            float rearAngle = _level.CalculateSlopeAngle(_bike.AttachmentPoints.Rear.X);

            float bikeAngle = _bike.Angle;
            float maxAngle = _physics.MaxSuspensionAngle;

            float frontSuspensionAngle = ClampValue(
                bikeAngle + frontAngle - MathHelper.PiOver2,
                bikeAngle - maxAngle,
                bikeAngle + maxAngle
            );

            float rearSuspensionAngle = ClampValue(
                bikeAngle + rearAngle - MathHelper.PiOver2,
                bikeAngle - maxAngle,
                bikeAngle + maxAngle
            );

            var (cosFrontAngle, sinFrontAngle) = GetTrigsFromAngle(frontSuspensionAngle);
            var (cosRearAngle, sinRearAngle) = GetTrigsFromAngle(rearSuspensionAngle);

            float frontSuspOffset = _bike.SuspensionOffsets.Front;
            float rearSuspOffset = _bike.SuspensionOffsets.Rear;

            ApplyTrickSuspensionAdjustments(ref frontSuspOffset, ref rearSuspOffset);

            _bike.WheelPositions = (
                new Vector2(
                    _bike.AttachmentPoints.Front.X + frontSuspOffset * cosFrontAngle,
                    frontGroundY + _physics.WheelRadius
                ),
                new Vector2(
                    _bike.AttachmentPoints.Rear.X + rearSuspOffset * cosRearAngle,
                    rearGroundY + _physics.WheelRadius
                )
            );

            AdjustBikePosition(frontGroundY, rearGroundY);
        }

        private void AdjustBikePosition(float frontGroundY, float rearGroundY)
        {
            float avgGroundY = (frontGroundY + rearGroundY) / 2f;
            float frontSuspOffset = _bike.SuspensionOffsets.Front;
            float rearSuspOffset = _bike.SuspensionOffsets.Rear;
            float avgSuspOffset = (frontSuspOffset + rearSuspOffset) / 2f;
            float desiredBikeY = avgGroundY + _physics.WheelRadius + avgSuspOffset;

            _bike.Position = new Vector2(_bike.Position.X, desiredBikeY);

            UpdateAttachmentPoints();
        }

        private void ApplyTrickSuspensionAdjustments(ref float frontSuspOffset, ref float rearSuspOffset)
        {
            if (_bike.IsInWheelie && !_bike.IsInAir)
            {
                frontSuspOffset = _physics.SuspensionRestLength -
                    (_physics.SuspensionRestLength - frontSuspOffset) *
                    (1.0f - _bike.WheelieIntensity * WheelieIntensityDampingMultiplier);
            }
            else if (_bike.IsInStoppie && !_bike.IsInAir)
            {
                rearSuspOffset = _physics.SuspensionRestLength -
                    (_physics.SuspensionRestLength - rearSuspOffset) *
                    (1.0f - _bike.StoppieIntensity * WheelieIntensityDampingMultiplier);
            }
        }

        private void UpdateWheelPositionsDefault()
        {
            float maxAngle = _physics.MaxSuspensionAngle;
            float bikeAngle = _bike.Angle;
            float verticalAngle = MathHelper.PiOver2;
            float normalAngle = bikeAngle + MathHelper.PiOver2;
            float angleDiff = normalAngle - verticalAngle;

            while (angleDiff > MathHelper.Pi) angleDiff -= MathHelper.TwoPi;
            while (angleDiff < -MathHelper.Pi) angleDiff += MathHelper.TwoPi;

            float suspensionAngle = _bike.EnforceSuspensionAngleLimits
                ? (Abs(angleDiff) <= maxAngle
                    ? verticalAngle
                    : normalAngle - (float)Sign(angleDiff) * maxAngle)
                : verticalAngle;

            float frontSuspOffset = _bike.SuspensionOffsets.Front;
            float rearSuspOffset = _bike.SuspensionOffsets.Rear;

            ApplyTrickSuspensionAdjustments(ref frontSuspOffset, ref rearSuspOffset);

            var (cosSuspAngle, sinSuspAngle) = GetTrigsFromAngle(suspensionAngle);

            _bike.WheelPositions = (
                new Vector2(
                    _bike.AttachmentPoints.Front.X + frontSuspOffset * cosSuspAngle,
                    _bike.AttachmentPoints.Front.Y + frontSuspOffset * sinSuspAngle
                ),
                new Vector2(
                    _bike.AttachmentPoints.Rear.X + rearSuspOffset * cosSuspAngle,
                    _bike.AttachmentPoints.Rear.Y + rearSuspOffset * sinSuspAngle
                )
            );
        }

        private void UpdateWheelRotations(float deltaTime)
        {
            var (cosAngle, sinAngle) = _physics.GetBikeTrigs();
            float groundSpeed = Vector2.Dot(_bike.Velocity, new Vector2((float)cosAngle, (float)sinAngle));
            float desiredOmega = groundSpeed / _physics.WheelRadius;

            if (_bike.IsInAir)
            {
                float airDeceleration = Physics.AirDeceleration * deltaTime;
                _bike.FrontWheelAngularVelocity = DecayAngularVelocity(_bike.FrontWheelAngularVelocity, airDeceleration);
                _bike.RearWheelAngularVelocity = DecayAngularVelocity(_bike.RearWheelAngularVelocity, airDeceleration);

                if (_bike.Throttle > 0)
                {
                    _bike.RearWheelAngularVelocity += _bike.Throttle * Physics.ThrottleRotationEffect * deltaTime;
                }
            }
            else
            {
                _bike.FrontWheelAngularVelocity = desiredOmega;

                float maxFrictionTorque = CalculateMaxFrictionTorque(true);
                float engineTorque = _bike.Throttle * _physics.EnginePower * _physics.WheelRadius;

                float netTorque = engineTorque - Sign(_bike.RearWheelAngularVelocity - desiredOmega) * maxFrictionTorque;

                float angularAcceleration = netTorque / (_physics.Mass * _physics.WheelRadius * _physics.WheelRadius);
                _bike.RearWheelAngularVelocity += angularAcceleration * deltaTime;

                float maxOmega = desiredOmega + (maxFrictionTorque / (_physics.Mass * _physics.WheelRadius));
                _bike.RearWheelAngularVelocity = MathHelper.Clamp(_bike.RearWheelAngularVelocity, desiredOmega - maxOmega, maxOmega);
            }

            _bike.WheelRotations = (
                (_bike.WheelRotations.Front + _bike.FrontWheelAngularVelocity * deltaTime) % FullRotation,
                (_bike.WheelRotations.Rear + _bike.RearWheelAngularVelocity * deltaTime) % FullRotation
            );
        }

        private float CalculateMaxFrictionTorque(bool isRearWheel)
        {
            float normalForce = _physics.Mass * _physics.Gravity / 2;
            float frictionCoefficient = isRearWheel ? _physics.RearGroundFriction : _physics.FrontGroundFriction;
            return normalForce * frictionCoefficient * _physics.WheelRadius;
        }

        private static float DecayAngularVelocity(float omega, float deceleration)
        {
            if (omega > 0)
            {
                return Math.Max(0, omega - deceleration);
            }
            else if (omega < 0)
            {
                return Math.Min(0, omega + deceleration);
            }
            return 0;
        }
    }

    public class BikePhysics : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly TrigCache _trigCache = new();

        private readonly ForcesComponent _forcesComponent;
        private readonly TorqueComponent _torqueComponent;
        private readonly TricksComponent _tricksComponent;
        private readonly SuspensionComponent _suspensionComponent;
        private readonly CollisionComponent _collisionComponent;
        internal readonly StateComponent _stateComponent;
        private readonly InputComponent _inputComponent;
        private readonly KinematicsComponent _kinematicsComponent;
        private readonly LeanController _leanController;

        private int _updateCounter;

        public float Mass { get; internal set; }
        public float MomentOfInertia { get; internal set; }
        public float Gravity { get; internal set; }
        public float EnginePower { get; internal set; }
        public float BrakeForce { get; internal set; }
        public float DragCoefficient { get; internal set; }
        public float FrontGroundFriction { get; internal set; }
        public float RearGroundFriction { get; internal set; }
        public float GroundFriction { get; internal set; }
        public float FrontSuspensionStrength { get; internal set; }
        public float RearSuspensionStrength { get; internal set; }
        public float FrontSuspensionDamping { get; internal set; }
        public float RearSuspensionDamping { get; internal set; }
        public float SuspensionRestLength { get; internal set; }
        public float SuspensionStrength { get; internal set; }
        public float SuspensionDamping { get; internal set; }
        public float MaxSuspensionAngle { get; internal set; }
        public float LeanSpeed { get; internal set; }
        public float MaxLeanAngle { get; internal set; }
        public float WheelRadius { get; internal set; }
        public float NominalWheelBase { get; internal set; }
        public float MinWheelDistance { get; internal set; }
        public float MaxWheelDistance { get; internal set; }
        public float WheelieBalance { get; set; }
        public float StoppieBalance { get; set; }

        public (double cos, double sin) GetBikeTrigs() => (_trigCache.Cos, _trigCache.Sin);

        public BikePhysics(Motorcycle bike) : base()
        {
            _bike = bike;
            Gravity = DefaultGravity * GravityMultiplier;
            _updateCounter = 0;
            WheelieBalance = StoppieBalance = 0.0f;

            _forcesComponent = new ForcesComponent(bike, this);
            _torqueComponent = new TorqueComponent(bike, this);
            _tricksComponent = new TricksComponent(bike, this);
            _suspensionComponent = new SuspensionComponent(bike, this);
            _collisionComponent = new CollisionComponent(bike, this);
            _stateComponent = new StateComponent(bike, this);
            _inputComponent = new InputComponent(bike);
            _kinematicsComponent = new KinematicsComponent(bike, this);
            _leanController = new LeanController();
        }

        public void InitializeProperties(BikeType bikeType)
        {
            BikePropertiesProvider.InitializeBikeProperties(this, bikeType, _bike.WheelBase);
        }

        public void Reset()
        {
            _stateComponent.Reset();
            _kinematicsComponent.UpdateAttachmentPoints();
            _kinematicsComponent.UpdateWheelPositions();
#if DEBUG
            Info("BikePhysics", "Bike reset to initial state");
#endif
        }

        public void SetPosition(Vector2 position)
        {
            _bike.Position = position;
            _kinematicsComponent.UpdateAttachmentPoints();
            _kinematicsComponent.UpdateWheelPositions();
        }

        public void SetBikeType(BikeType bikeType) => InitializeProperties(bikeType);

        public void ApplyThrottle(float amount) => _inputComponent.ApplyThrottle(amount);
        public void ApplyBrake(float amount) => _inputComponent.ApplyBrake(amount);
        public void Lean(float direction) => _inputComponent.Lean(direction);

        public List<Vector2> GetFramePoints() =>
            FramePhysics.GetFramePoints(_bike.Position, _bike.Angle, _bike.WheelBase, _bike.FrameHeight);

        public void Update(float deltaTime, ILevelPhysics level, CancellationToken cancellationToken = default)
        {
            if (_stateComponent.ShouldSkipUpdate(cancellationToken))
                return;

            _stateComponent.SavePreviousState();
            UpdateCycle(deltaTime, level, cancellationToken);
            _stateComponent.LogStateChanges();
        }

        private void UpdateCycle(float deltaTime, ILevelPhysics level, CancellationToken cancellationToken)
        {
            if (cancellationToken.IsCancellationRequested)
                return;

            UpdatePhysics(deltaTime, level, cancellationToken);
            _kinematicsComponent.UpdateKinematics(deltaTime);
            PhysicsValidator.ValidateAndSanitizeBikeState(_bike, this, level, deltaTime, _stateComponent);
        }

        public void UpdatePhysics(float deltaTime, ILevelPhysics level, CancellationToken cancellationToken = default)
        {
            if (cancellationToken.IsCancellationRequested) return;

            _trigCache.Update(_bike.Angle);
            _suspensionComponent.HandleWheelCollisions(level, deltaTime);

            Vector2 totalForce = _forcesComponent.CalculateTotalForce(level, deltaTime);
            _bike.Velocity += totalForce * (deltaTime / Mass);

            float totalTorque = _torqueComponent.CalculateTotalTorque(deltaTime);
            ValidateVectorParameter("TotalForce", totalForce, Mass * MaxSafeAcceleration, "BikePhysics");
            float maxTorque = MomentOfInertia * MaxAngularVelocity / deltaTime;
            ValidatePhysicalParameter("TotalTorque", (float)Abs(totalTorque), 0, maxTorque, "BikePhysics");

            _bike.AngularVelocity += totalTorque / MomentOfInertia * deltaTime;

            _leanController.UpdateLean(_bike, this, deltaTime, level);
            _bike.Angle = MathHelper.WrapAngle(_bike.Angle + _bike.AngularVelocity * deltaTime);

            float groundY = level.GetGroundYAtX(_bike.Position.X);
            if (_bike.Position.Y > groundY + 5.0f)
            {
                Warning("BikePhysics", $"Excessive ground penetration: {_bike.Position.Y - groundY}");
                _bike.Position = new Vector2(_bike.Position.X, groundY);
                _bike.Velocity = new Vector2(_bike.Velocity.X, 0);
            }

            _kinematicsComponent.UpdateAttachmentPoints();
            _kinematicsComponent.UpdateWheelPositions();

            _collisionComponent.CheckFrameCollision(level, deltaTime);
            FramePhysics.CheckCrashConditions(_bike, MinWheelDistance, MaxWheelDistance);
            _tricksComponent.UpdateTrickStates(deltaTime);

#if DEBUG
            LogStatusIfNeeded(_bike.Velocity.Length());
#endif
        }

#if DEBUG
        private void LogStatusIfNeeded(float speed)
        {
            if (++_updateCounter >= StatusLogInterval)
            {
                Debug("BikePhysics", $"Status: vel={speed:F2}, angle={_bike.Angle:F2}, inAir={_bike.IsInAir}");
                _updateCounter = 0;
            }
        }
#endif
    }
}