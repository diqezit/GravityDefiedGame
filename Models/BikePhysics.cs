using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Threading;
using static GravityDefiedGame.Utilities.GameConstants;
using static GravityDefiedGame.Utilities.GameConstants.Debug;
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

    public class ForcesComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;
        private float _prevThrottle, _prevBrake;

        public ForcesComponent(Motorcycle bike, BikePhysics physics) : base() =>
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

        public Vector2 CalculateSlopeForce(Level level)
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

        public Vector2 CalculateTotalForce(Level level, float deltaTime) =>
            new Vector2(0, _physics.Gravity * _physics.Mass) +
            CalculateThrustForce(deltaTime) +
            CalculateBrakeForce(deltaTime) +
            CalculateDragForce() +
            CalculateSlopeForce(level);
    }

    public class TorqueComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
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

        public TorqueComponent(Motorcycle bike, BikePhysics physics) : base() =>
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

            _bike.WheelieTime = SanitizeValue(
                _bike.WheelieTime + deltaTime,
                0.0f,
                "Invalid wheelie time value");

            if (_bike.WheelieTime > WheelieEasyTime)
            {
                float factor = MathHelper.Min(1.0f, (_bike.WheelieTime - WheelieEasyTime) / WheelieHardTimeDelta);
                _physics.WheelieBalance *= (1.0f - factor * WheelieProgressiveDifficulty);
            }
        }

        private void UpdateStoppieBalance(float deltaTime)
        {
            float diff = _bike.Angle - StoppieBalanceAngle;
            _physics.StoppieBalance = SanitizeValue(
                -diff * StoppieBalanceResponseFactor,
                0.0f,
                "Invalid stoppie balance value calculated");

            _bike.StoppieTime = SanitizeValue(
                _bike.StoppieTime + deltaTime,
                0.0f,
                "Invalid stoppie time value");

            if (_bike.StoppieTime > StoppieEasyTime)
            {
                float factor = MathHelper.Min(1.0f, (_bike.StoppieTime - StoppieEasyTime) / StoppieHardTimeDelta);
                _physics.StoppieBalance *= (1.0f - factor * StoppieProgressiveDifficulty);
            }
        }

        public float CalculateTotalTorque(float deltaTime) =>
            CalculateBaseTorque(deltaTime) +
            CalculateWheelieTorque(deltaTime) +
            CalculateStoppieTorque(deltaTime);
    }

    public class TricksComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public TricksComponent(Motorcycle bike, BikePhysics physics) =>
            (_bike, _physics) = (bike, physics);

        public void UpdateTrickStates(float deltaTime)
        {
            bool wasInWheelie = _bike.IsInWheelie;
            bool wasInStoppie = _bike.IsInStoppie;

            _bike.IsInWheelie = !_bike.IsInAir &&
                _bike.WheelPositions.Front.Y < _bike.WheelPositions.Rear.Y - _physics.WheelRadius * WheelieHeightFactor &&
                _bike.Angle > WheelieMinAngle;

            _bike.IsInStoppie = !_bike.IsInAir &&
                _bike.WheelPositions.Rear.Y < _bike.WheelPositions.Front.Y - _physics.WheelRadius * StoppieHeightFactor &&
                _bike.Angle < -StoppieMinAngle;

            float wheelieTime = _bike.WheelieTime;
            float stoppieTime = _bike.StoppieTime;

            UpdateTrickTime(wasInWheelie, _bike.IsInWheelie, ref wheelieTime);
            UpdateTrickTime(wasInStoppie, _bike.IsInStoppie, ref stoppieTime);

            _bike.WheelieTime = wheelieTime;
            _bike.StoppieTime = stoppieTime;

#if DEBUG
            LogTrickChange(wasInWheelie, _bike.IsInWheelie, "Wheelie", wheelieTime);
            LogTrickChange(wasInStoppie, _bike.IsInStoppie, "Stoppie", stoppieTime);
#endif
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
        private readonly Motorcycle _bike;
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

        // Константы для расчета сцепления с землей
        private const float SlopeGripReductionFactor = 0.5f;
        private const float MinGripThreshold = 0.1f;
        private const float MinPenetrationThreshold = 0.01f;
        private const float MinNormalY = 0.2f;

        public SuspensionComponent(Motorcycle bike, BikePhysics physics) : base() =>
            (_bike, _physics) = (bike, physics);

        private Vector2 CalculateSurfaceNormal(Level level, float x)
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

        public void HandleWheelCollisions(Level level, float deltaTime)
        {
            float frontGroundY = level.GetGroundYAtX(_bike.WheelPositions.Front.X);
            float rearGroundY = level.GetGroundYAtX(_bike.WheelPositions.Rear.X);

            Vector2 frontNormal = CalculateSurfaceNormal(level, _bike.WheelPositions.Front.X);
            Vector2 rearNormal = CalculateSurfaceNormal(level, _bike.WheelPositions.Rear.X);

            bool frontContact = HandleWheelCollision(true, frontGroundY, frontNormal, level, deltaTime);
            bool rearContact = HandleWheelCollision(false, rearGroundY, rearNormal, level, deltaTime);

            bool wasInAir = _bike.IsInAir;
            _bike.State = (frontContact || rearContact)
                ? _bike.State & ~BikeState.InAir
                : _bike.State | BikeState.InAir;

#if DEBUG
            if (wasInAir && !_bike.IsInAir)
                Info("SuspensionComponent", "Landed");
            else if (!wasInAir && _bike.IsInAir)
                Info("SuspensionComponent", "Became airborne");
#endif
        }

        public bool HandleWheelCollision(bool isFrontWheel, float groundY, Vector2 normal, Level level, float deltaTime)
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

        private void ProcessWheelPenetration(bool isFrontWheel, Vector2 wheelPos, float penetration, Vector2 normal, Level _, float deltaTime)
        {
            // Выбор параметров в зависимости от колеса
            float suspensionStrength = isFrontWheel ? _physics.FrontSuspensionStrength : _physics.RearSuspensionStrength;
            float suspensionDamping = isFrontWheel ? _physics.FrontSuspensionDamping : _physics.RearSuspensionDamping;
            float groundFriction = isFrontWheel ? _physics.FrontGroundFriction : _physics.RearGroundFriction;

            // Обновляем состояние проникновения
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

            // Расчет сцепления с учетом трения колеса
            float slopeAngle = (float)Atan2(normal.Y, normal.X) - MathHelper.Pi;
            float gripFactor = CalculateGripFactor(slopeAngle, groundFriction);

            // Функция для расчета компрессии
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

            // Функция для расчета смещения подвески
            float CalculateSuspensionOffset(float compression)
            {
                float newOffset = _physics.SuspensionRestLength - compression;
                float currentOffset = isFrontWheel ? _bike.SuspensionOffsets.Front : _bike.SuspensionOffsets.Rear;

                if (Abs(newOffset - currentOffset) > _physics.SuspensionRestLength * LargeSuspensionChangeThreshold)
                    newOffset = MathHelper.Lerp(currentOffset, newOffset, LargeSuspensionChangeSmoothingFactor);

                return ClampValue(newOffset, _physics.SuspensionRestLength * MinSuspensionOffset, _physics.SuspensionRestLength);
            }

            // Функция для расчета силы реакции
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

            // Функция для применения силы реакции
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

                _bike.Velocity += reactionForce * (deltaTime / _physics.Mass);
                _bike.AngularVelocity += torque / _physics.MomentOfInertia * deltaTime;
            }

            float compression = CalculateCompression();
            float newOffset = CalculateSuspensionOffset(compression);
            Vector2 reactionForce = CalculateReactionForce(newOffset);
            ApplyReactionForce(reactionForce);
            SetSuspensionOffset(isFrontWheel, newOffset);
        }

        private void SetSuspensionOffset(bool isFrontWheel, float offset)
        {
            var current = _bike.SuspensionOffsets;
            _bike.SuspensionOffsets = isFrontWheel ? (offset, current.Rear) : (current.Front, offset);
        }
    }

    public class CollisionComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public CollisionComponent(Motorcycle bike, BikePhysics physics) : base() =>
            (_bike, _physics) = (bike, physics);

        public void CheckFrameCollision(Level level, float deltaTime)
        {
            if (_bike is { IsInAir: true } or { IsCrashed: true } || _bike.Velocity.Length() <= FrameCollisionMinVelocity)
                return;

            var collisionInfo = DetectFrameCollision(level);
            if (collisionInfo.IsCollision && collisionInfo.MaxPenetration > _physics.WheelRadius * FrameCollisionMinPenetration)
                HandleFrameCollision(collisionInfo, deltaTime);
        }

        private void HandleFrameCollision((bool IsCollision, Vector2 CollisionPoint, float MaxPenetration) collisionInfo, float deltaTime)
        {
            float crashThreshold = _physics.WheelRadius * FrameCrashThreshold;
            bool isBadAngle = Abs(_bike.Angle) > FrameCriticalBackwardTiltAngle;
            bool isHighSpeed = _bike.Velocity.Length() > FrameCollisionHighSpeedThreshold;

            if (collisionInfo.MaxPenetration > crashThreshold && (isHighSpeed || isBadAngle))
            {
                _bike.State |= BikeState.Crashed;
#if DEBUG
                Info("CollisionComponent", "Bike crashed due to frame collision");
#endif
            }
            else
            {
                ApplyFrameCollisionResponse(collisionInfo.CollisionPoint, collisionInfo.MaxPenetration, deltaTime);
            }
        }

        private void ApplyFrameCollisionResponse(Vector2 _, float penetration, float deltaTime)
        {
            float reactionForce = penetration * _physics.SuspensionStrength * FrameCollisionReactionForce;
            float impulse = reactionForce * deltaTime;
            float deltaVelocityY = -impulse / _physics.Mass;

            deltaVelocityY = MathHelper.Max(deltaVelocityY, -FrameCollisionMaxDeltaVelocity * deltaTime);
            _bike.Velocity = new(_bike.Velocity.X, _bike.Velocity.Y + deltaVelocityY);

            float stabilizingFactor = Abs(_bike.Angle) > FrameStabilizingAngleThreshold
                ? FrameStabilizingFactorStrong
                : FrameStabilizingFactorBase;

            float stabilizingTorque = -_bike.Angle * stabilizingFactor;
            float deltaAngularVelocity = stabilizingTorque * deltaTime / _physics.MomentOfInertia;

            deltaAngularVelocity = ClampValue(
                deltaAngularVelocity,
                -FrameCollisionMaxDeltaAngular * deltaTime,
                FrameCollisionMaxDeltaAngular * deltaTime);

            _bike.AngularVelocity += deltaAngularVelocity;
        }

        private (bool IsCollision, Vector2 CollisionPoint, float MaxPenetration) DetectFrameCollision(Level level)
        {
            var framePoints = _bike.GetFramePoints();
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

    public class BikePhysics : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly ForcesComponent _forcesComponent;
        private readonly TorqueComponent _torqueComponent;
        private readonly TricksComponent _tricksComponent;
        private readonly SuspensionComponent _suspensionComponent;
        private readonly CollisionComponent _collisionComponent;
        private readonly StateComponent _stateComponent;
        private readonly InputComponent _inputComponent;
        private readonly KinematicsComponent _kinematicsComponent;
        private readonly ValidationComponent _validationComponent;
        private readonly BikeGeom _geometryComponent;
        private readonly TrigCache _trigCache = new();
        private const float MaxAllowedPenetration = 5.0f;

        private int _updateCounter;
        private float _prevSlopeAngle;

        // Свойства мотоцикла (сгруппированы по функциональности)
        // Физические параметры
        public float Mass { get; private set; }                      // Масса мотоцикла
        public float MomentOfInertia { get; private set; }           // Момент инерции
        public float Gravity { get; private set; }                   // Сила гравитации

        // Параметры двигателя и торможения
        public float EnginePower { get; private set; }               // Мощность двигателя
        public float BrakeForce { get; private set; }                // Тормозная сила
        public float DragCoefficient { get; private set; }           // Коэффициент сопротивления

        // Параметры трения
        public float FrontGroundFriction { get; private set; }       // Трение переднего колеса
        public float RearGroundFriction { get; private set; }        // Трение заднего колеса
        public float GroundFriction { get; private set; }            // Устаревшее: общее трение

        // Параметры подвески
        public float FrontSuspensionStrength { get; private set; }   // Жесткость передней подвески
        public float RearSuspensionStrength { get; private set; }    // Жесткость задней подвески
        public float FrontSuspensionDamping { get; private set; }    // Демпфирование передней подвески
        public float RearSuspensionDamping { get; private set; }     // Демпфирование задней подвески
        public float SuspensionRestLength { get; private set; }      // Длина подвески в покое
        public float SuspensionStrength { get; private set; }        // Устаревшее: общая жесткость
        public float SuspensionDamping { get; private set; }         // Устаревшее: общее демпфирование
        public float MaxSuspensionAngle { get; private set; }        // Макс. угол подвески

        // Параметры управления
        public float LeanSpeed { get; private set; }                 // Скорость наклона
        public float MaxLeanAngle { get; private set; }              // Макс. угол наклона

        // Параметры колес
        public float WheelRadius { get; private set; } = DefaultWheelRadius; // Радиус колеса
        public float NominalWheelBase { get; private set; }          // Номинальная база колес
        public float MinWheelDistance { get; private set; }          // Мин. расстояние между колесами
        public float MaxWheelDistance { get; private set; }          // Макс. расстояние между колесами

        // Параметры трюков
        public float WheelieBalance { get; set; }                    // Баланс при вилли
        public float StoppieBalance { get; set; }                    // Баланс при стоппи

        public (double cos, double sin) GetBikeTrigs() => (_trigCache.Cos, _trigCache.Sin);

        public BikePhysics(Motorcycle bike) : base()
        {
            _bike = bike;
            Gravity = DefaultGravity * GravityMultiplier;
            _updateCounter = 0;
            _prevSlopeAngle = 0;
            WheelieBalance = StoppieBalance = 0.0f;

            // Инициализация компонентов
            _forcesComponent = new ForcesComponent(bike, this);
            _torqueComponent = new TorqueComponent(bike, this);
            _tricksComponent = new TricksComponent(bike, this);
            _suspensionComponent = new SuspensionComponent(bike, this);
            _collisionComponent = new CollisionComponent(bike, this);
            _stateComponent = new StateComponent(bike, this);
            _inputComponent = new InputComponent(bike);
            _kinematicsComponent = new KinematicsComponent(bike, this);
            _validationComponent = new ValidationComponent(bike, this);
            _geometryComponent = new BikeGeom(bike);
        }

        public void InitializeProperties(BikeType bikeType)
        {
            var props = GetBikeProperties(bikeType);
            var wheelProps = GetWheelProperties(bikeType);

            // Установка всех свойств
            Mass = props.mass;
            EnginePower = props.power;
            BrakeForce = props.brakeForce;
            DragCoefficient = props.drag;
            MaxLeanAngle = props.maxLeanAngle;
            LeanSpeed = props.leanSpeed;
            FrontGroundFriction = props.friction * wheelProps.Front.friction;
            RearGroundFriction = props.friction * wheelProps.Rear.friction;
            FrontSuspensionStrength = props.suspensionStrength * wheelProps.Front.suspensionStrength;
            RearSuspensionStrength = props.suspensionStrength * wheelProps.Rear.suspensionStrength;
            FrontSuspensionDamping = props.suspensionDamping * wheelProps.Front.suspensionDamping;
            RearSuspensionDamping = props.suspensionDamping * wheelProps.Rear.suspensionDamping;
            SuspensionRestLength = props.suspensionRestLength;
            MaxSuspensionAngle = props.maxSuspensionAngle;
            WheelRadius = wheelProps.Front.radius;
            MomentOfInertia = Mass * (float)Pow(_bike.WheelBase / 2, 2) * MomentOfInertiaMultiplier;
            NominalWheelBase = _bike.WheelBase;
            MinWheelDistance = NominalWheelBase * WheelDistanceMinRatio;
            MaxWheelDistance = NominalWheelBase * WheelDistanceMaxRatio;

            // Установка устаревших свойств для совместимости
            GroundFriction = FrontGroundFriction;
            SuspensionStrength = FrontSuspensionStrength;
            SuspensionDamping = FrontSuspensionDamping;

#if DEBUG
            Info("BikePhysics", $"Initialized {bikeType} bike with mass: {Mass}, power: {EnginePower}, brake: {BrakeForce}");
#endif
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

        // Простые делегаты к компонентам
        public void ApplyThrottle(float amount) => _inputComponent.ApplyThrottle(amount);
        public void ApplyBrake(float amount) => _inputComponent.ApplyBrake(amount);
        public void Lean(float direction) => _inputComponent.Lean(direction);
        public List<Vector2> GetFramePoints() => _geometryComponent.GetFramePoints();
        public (List<BikeGeom.SkeletonPoint> Points, List<BikeGeom.SkeletonLine> Lines) GetSkeleton() =>
            _geometryComponent.GetSkeleton();

        public void Update(float deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            if (_stateComponent.ShouldSkipUpdate(cancellationToken))
                return;

            _stateComponent.SavePreviousState();
            UpdateCycle(deltaTime, level, cancellationToken);
            _stateComponent.LogStateChanges();
        }

        private void UpdateCycle(float deltaTime, Level level, CancellationToken cancellationToken)
        {
            if (cancellationToken.IsCancellationRequested)
                return;

            UpdatePhysics(deltaTime, level, cancellationToken);
            _kinematicsComponent.UpdateKinematics(deltaTime);
            _validationComponent.ValidateAndSanitizeState(deltaTime);
        }

        public void UpdatePhysics(float deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            if (cancellationToken.IsCancellationRequested) return;

            _trigCache.Update(_bike.Angle);
            _suspensionComponent.HandleWheelCollisions(level, deltaTime);

            // Расчет и применение сил
            Vector2 totalForce = _forcesComponent.CalculateTotalForce(level, deltaTime);
            _bike.Velocity += totalForce * (deltaTime / Mass);

            float totalTorque = _torqueComponent.CalculateTotalTorque(deltaTime);
            ValidateVectorParameter("TotalForce", totalForce, Mass * MaxSafeAcceleration, "BikePhysics");
            float maxTorque = MomentOfInertia * MaxAngularVelocity / deltaTime;
            ValidatePhysicalParameter("TotalTorque", (float)Abs(totalTorque), 0, maxTorque, "BikePhysics");

            _bike.AngularVelocity += totalTorque / MomentOfInertia * deltaTime;

            UpdateLean(deltaTime, level);
            _bike.Angle = MathHelper.WrapAngle(_bike.Angle + _bike.AngularVelocity * deltaTime);

            // Проверка на проникновение в землю
            float groundY = level.GetGroundYAtX(_bike.Position.X);
            if (_bike.Position.Y > groundY + MaxAllowedPenetration)
            {
                Warning("BikePhysics", $"Excessive ground penetration: {_bike.Position.Y - groundY}");
                _bike.Position = new Vector2(_bike.Position.X, groundY);
                _bike.Velocity = new Vector2(_bike.Velocity.X, 0);
            }

            _kinematicsComponent.UpdateAttachmentPoints();
            _kinematicsComponent.UpdateWheelPositions();

            // Проверка на проникновение колес
            CheckWheelPenetration(level);

            _collisionComponent.CheckFrameCollision(level, deltaTime);
            CheckCrashConditions();
            _tricksComponent.UpdateTrickStates(deltaTime);

#if DEBUG
            LogStatusIfNeeded(_bike.Velocity.Length());
#endif

            SanitizePhysicalState();
        }

        private void CheckWheelPenetration(Level level)
        {
            float frontGroundY = level.GetGroundYAtX(_bike.WheelPositions.Front.X);
            float rearGroundY = level.GetGroundYAtX(_bike.WheelPositions.Rear.X);
            float wheelThreshold = WheelRadius + MaxAllowedPenetration;

            if (_bike.WheelPositions.Front.Y > frontGroundY + wheelThreshold ||
                _bike.WheelPositions.Rear.Y > rearGroundY + wheelThreshold)
            {
                Warning("BikePhysics", "Wheel penetration exceeded maximum allowed value");
                float centerGroundY = level.GetGroundYAtX(_bike.Position.X);
                _bike.Position = new Vector2(_bike.Position.X, centerGroundY - WheelRadius);
                _bike.Velocity = new Vector2(_bike.Velocity.X, 0);
                _kinematicsComponent.UpdateAttachmentPoints();
                _kinematicsComponent.UpdateWheelPositions();
            }
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

        private void CheckCrashConditions()
        {
            if (_bike.IsInAir || _bike.IsCrashed)
                return;

            // Критический угол наклона
            if (Abs(_bike.Angle) > CriticalLeanAngle)
            {
                _bike.State |= BikeState.Crashed;
#if DEBUG
                Info("BikePhysics", $"Bike crashed due to critical lean angle: {_bike.Angle:F2}");
#endif
                return;
            }

            // Неправильное расстояние между колесами
            float currentDistance = CalculateDistance(_bike.WheelPositions.Front, _bike.WheelPositions.Rear);
            if (currentDistance < MinWheelDistance || currentDistance > MaxWheelDistance)
            {
                _bike.State |= BikeState.Crashed;
#if DEBUG
                Info("BikePhysics", $"Bike crashed due to invalid wheel distance: {currentDistance:F2}");
#endif
            }
        }

        private void UpdateLean(float deltaTime, Level level)
        {
            if (_bike.IsInAir)
            {
                // Упрощенное управление в воздухе
                float desiredAngularVelocity = _bike.LeanAmount * MaxAirAngularVelocity;
                _bike.AngularVelocity = MathHelper.Lerp(_bike.AngularVelocity, desiredAngularVelocity, AirDampingFactor * deltaTime);
                return;
            }

            // Расчет наклона поверхности
            float currentSlopeAngle = level.CalculateSlopeAngle(_bike.Position.X);
            float angleDifference = Math.Abs(currentSlopeAngle - _prevSlopeAngle);
            float smoothedSlopeAngle = angleDifference > SlopeAngleSmoothingThreshold
                ? MathHelper.Lerp(_prevSlopeAngle, currentSlopeAngle, SlopeTransitionRate * deltaTime)
                : currentSlopeAngle;
            _prevSlopeAngle = smoothedSlopeAngle;

            // Расчет факторов, влияющих на управление
            float speed = _bike.Velocity.Length();
            float speedFactor = MathHelper.Min(1.0f, speed / LeanSpeedFactorDenominator);
            float adaptiveLeanSpeed = LeanSpeed * (LeanSpeedBaseMultiplier + LeanSpeedFactorMultiplier * speedFactor);
            float terrainAdaptationFactor = MathHelper.Min(1.0f, speed / TerrainAdaptationSpeedThreshold);
            float slopeInfluence = smoothedSlopeAngle * terrainAdaptationFactor;

            // Расчет целевого угла наклона
            float targetLean = _bike.LeanAmount * MaxLeanAngle + slopeInfluence + _bike.Throttle * ThrottleLeanInfluence;
            float leanError = targetLean - _bike.Angle;
            float angularVelocityDamping = AngularVelocityDampingBase + AngularVelocityDampingFactor * speedFactor;

            // Расчет управляющего момента
            float controlTorque = LeanControlTorqueMultiplier * leanError * adaptiveLeanSpeed -
                                 angularVelocityDamping * _bike.AngularVelocity;

            // Стабилизация при отсутствии ввода
            if (_bike.Throttle < InputIdleThreshold && _bike.Brake < InputIdleThreshold &&
                Math.Abs(_bike.AngularVelocity) > AngularVelocityIdleThreshold)
            {
                float stabilizationFactor = StabilizationFactorBase +
                    StabilizationFactorSpeedMultiplier * MathHelper.Min(1.0f, speed / StabilizationSpeedThreshold);
                controlTorque -= _bike.AngularVelocity * stabilizationFactor * StabilizationTorqueMultiplier;
            }

            // Применение момента и ограничение угловой скорости
            _bike.AngularVelocity += controlTorque / MomentOfInertia * deltaTime;
            float maxAngularVel = MaxAngularVelocity * GroundAngularVelocityFactor;
            _bike.AngularVelocity = MathHelper.Clamp(_bike.AngularVelocity, -maxAngularVel, maxAngularVel);
        }

        private void SanitizePhysicalState()
        {
            _bike.Angle = (float)NormalizeAngle(SanitizeValue(_bike.Angle, 0, "Invalid angle value"));
            _bike.AngularVelocity = ClampValue(
                SanitizeValue(_bike.AngularVelocity, 0, "Invalid angular velocity"),
                -MaxAngularVelocity,
                MaxAngularVelocity);

            _bike.Velocity = SanitizeVector(_bike.Velocity, new Vector2(), "Invalid velocity");

            if (_bike.Velocity.Length() > MaxSafeVelocity)
            {
                Warning("BikePhysics",
                    $"Velocity exceeds max safe value: {_bike.Velocity.Length():F2} > {MaxSafeVelocity:F2}");
                _bike.Velocity = _bike.Velocity * (MaxSafeVelocity / _bike.Velocity.Length());
            }
        }

        private static BikeProperties GetBikeProperties(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => Bike.Standard,
            BikeType.Sport => Bike.Sport,
            BikeType.OffRoad => Bike.OffRoad,
            _ => Bike.Standard
        };

        private static (WheelProperties Front, WheelProperties Rear)
            GetWheelProperties(BikeType bikeType) => bikeType switch
            {
                BikeType.Standard => Wheels.Standard,
                BikeType.Sport => Wheels.Sport,
                BikeType.OffRoad => Wheels.OffRoad,
                _ => Wheels.Standard
            };

        /// <summary>
        /// Компонент для управления состоянием мотоцикла
        /// </summary>
        private class StateComponent
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

        /// <summary>
        /// Компонент для обработки входных данных мотоцикла
        /// </summary>
        private class InputComponent
        {
            private readonly Motorcycle _bike;

            public InputComponent(Motorcycle bike) => _bike = bike;

            public void ApplyThrottle(float amount) =>
                _bike.Throttle = ClampValue(amount, 0, 1);

            public void ApplyBrake(float amount) =>
                _bike.Brake = ClampValue(amount, 0, 1);

            public void Lean(float direction) =>
                _bike.LeanAmount = ClampValue(direction, -1, 1);
        }

        /// <summary>
        /// Компонент для расчета кинематики мотоцикла
        /// </summary>
        private class KinematicsComponent
        {
            private readonly Motorcycle _bike;
            private readonly BikePhysics _physics;
            private Level _level;

            public KinematicsComponent(Motorcycle bike, BikePhysics physics) =>
                (_bike, _physics) = (bike, physics);

            public void SetLevel(Level level) => _level = level;

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
                        _bike.Position.X + halfWheelBase * (float)cosAngle,
                        _bike.Position.Y + halfWheelBase * (float)sinAngle
                    ),
                    new Vector2(
                        _bike.Position.X - halfWheelBase * (float)cosAngle,
                        _bike.Position.Y - halfWheelBase * (float)sinAngle
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

                // Корректировка для трюков
                ApplyTrickSuspensionAdjustments(ref frontSuspOffset, ref rearSuspOffset);

                _bike.WheelPositions = (
                    new Vector2(
                        _bike.AttachmentPoints.Front.X + frontSuspOffset * (float)cosFrontAngle,
                        _bike.AttachmentPoints.Front.Y + frontSuspOffset * (float)sinFrontAngle
                    ),
                    new Vector2(
                        _bike.AttachmentPoints.Rear.X + rearSuspOffset * (float)cosRearAngle,
                        _bike.AttachmentPoints.Rear.Y + rearSuspOffset * (float)sinRearAngle
                    )
                );
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

                // Корректировка для трюков
                ApplyTrickSuspensionAdjustments(ref frontSuspOffset, ref rearSuspOffset);

                var (cosSuspAngle, sinSuspAngle) = GetTrigsFromAngle(suspensionAngle);

                _bike.WheelPositions = (
                    new Vector2(
                        _bike.AttachmentPoints.Front.X + frontSuspOffset * (float)cosSuspAngle,
                        _bike.AttachmentPoints.Front.Y + frontSuspOffset * (float)sinSuspAngle
                    ),
                    new Vector2(
                        _bike.AttachmentPoints.Rear.X + rearSuspOffset * (float)cosSuspAngle,
                        _bike.AttachmentPoints.Rear.Y + rearSuspOffset * (float)sinSuspAngle
                    )
                );
            }

            private void UpdateWheelRotations(float deltaTime)
            {
                var (cosAngle, sinAngle) = _physics.GetBikeTrigs();
                float groundSpeed = Vector2.Dot(_bike.Velocity, new Vector2((float)cosAngle, (float)sinAngle));
                float wheelCircumference = WheelCircumferenceFactor * _physics.WheelRadius;

                float frontRotation = UpdateSingleWheelRotation(
                    _bike.WheelRotations.Front, groundSpeed, deltaTime, true, wheelCircumference);
                float rearRotation = UpdateSingleWheelRotation(
                    _bike.WheelRotations.Rear, groundSpeed, deltaTime, false, wheelCircumference);

                _bike.WheelRotations = (frontRotation % FullRotation, rearRotation % FullRotation);
            }

            private float UpdateSingleWheelRotation(
                float currentRotation,
                float groundSpeed,
                float deltaTime,
                bool isFrontWheel,
                float wheelCircumference)
            {
                float rotationFactor = _bike.IsInAir ? AirRotationFactor : GroundRotationFactor;
                float rotationDelta = SafeDivide(groundSpeed, wheelCircumference);

                float slipFactor = CalculateWheelSlipFactor();

                rotationDelta = _bike.IsInAir
                    ? isFrontWheel
                        ? rotationDelta
                        : rotationDelta + SafeDivide(_bike.Throttle * ThrottleRotationFactor, wheelCircumference)
                    : isFrontWheel
                        ? rotationDelta
                        : rotationDelta * (1 + slipFactor);

                return currentRotation + rotationDelta * deltaTime * rotationFactor;
            }

            private float CalculateWheelSlipFactor()
            {
                if (_bike.Throttle <= WheelSlipThreshold)
                    return 0;

                float slipFactor = (_bike.Throttle - WheelSlipThreshold) * SlipThrottleFactor;
                slipFactor *= MathHelper.Min(1.0f, _physics.GroundFriction / SlipFrictionRatio);
                slipFactor *= MathHelper.Min(1.0f, _bike.Velocity.Length() / SlipSpeedThreshold);

                return slipFactor;
            }
        }

        /// <summary>
        /// Компонент для валидации и поддержания корректного физического состояния
        /// </summary>
        private class ValidationComponent
        {
            private readonly Motorcycle _bike;
            private readonly BikePhysics _physics;

            public ValidationComponent(Motorcycle bike, BikePhysics physics) =>
                (_bike, _physics) = (bike, physics);

            public void ValidateAndSanitizeState(float deltaTime)
            {
                SanitizePhysicalState();
                _physics._stateComponent.UpdateAirTime(deltaTime);
                ValidateState();
            }

            private void SanitizePhysicalState()
            {
                SanitizeSuspension();
                _bike.Position = SanitizePosition(_bike.Position, DefaultStartPosition, "Invalid position detected");
            }

            private void SanitizeSuspension()
            {
                float restLength = _physics.SuspensionRestLength;
                float minOffset = MinSuspensionOffset;
                var offsets = _bike.SuspensionOffsets;

                offsets.Front = GetValidOffset(offsets.Front, restLength, minOffset, "front");
                offsets.Rear = GetValidOffset(offsets.Rear, restLength, minOffset, "rear");

                _bike.SuspensionOffsets = offsets;
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

            private void ValidateState() => CheckSuspensionCompression();

            private void CheckSuspensionCompression()
            {
                float restLength = _physics.SuspensionRestLength;
                float threshold = HighSuspensionCompressionThreshold;
                var offsets = _bike.SuspensionOffsets;

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
    }
}