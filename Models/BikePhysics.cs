using GravityDefiedGame.Utilities;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Windows;
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
        private double _prevThrottle, _prevBrake;

        public ForcesComponent(Motorcycle bike, BikePhysics physics) : base() =>
            (_bike, _physics, _prevThrottle, _prevBrake) = (bike, physics, 0, 0);

        public Vector CalculateThrustForce(double deltaTime)
        {
            return Log("ForcesComponent", "calculating thrust force", () =>
            {
                _prevThrottle += (_bike.Throttle - _prevThrottle) * Min(1.0, ThrottleTransitionRate * deltaTime);
                double force = _prevThrottle * _physics.EnginePower;
                var (cos, sin) = _physics.GetBikeTrigs();
                return new Vector(cos * force, sin * force);
            }, new Vector(0, 0));
        }

        public Vector CalculateBrakeForce(double deltaTime)
        {
            return Log("ForcesComponent", "calculating brake force", () =>
            {
                if (_bike.Brake <= MinBrakeInput || _bike.Velocity.Length <= 0)
                {
                    _prevBrake = Max(0, _prevBrake - BrakeTransitionRate * deltaTime);
                    return _prevBrake <= BrakeTransitionThreshold ? new Vector(0, 0) :
                        -_bike.Velocity * (_prevBrake * _physics.BrakeForce / _bike.Velocity.Length);
                }

                _prevBrake += (_bike.Brake - _prevBrake) * Min(1.0, BrakeTransitionRate * deltaTime);
                return -_bike.Velocity * (_prevBrake * _physics.BrakeForce / _bike.Velocity.Length);
            }, new Vector(0, 0));
        }

        public Vector CalculateDragForce()
        {
            return Log("ForcesComponent", "calculating drag force", () =>
            {
                double speed = _bike.Velocity.Length;
                if (speed <= 0) return new Vector(0, 0);

                double drag = _physics.DragCoefficient * (_bike.IsInAir ? AirFrictionMultiplier : 1.0);
                if (!_bike.IsInAir && _bike.Throttle == 0 && _bike.Brake == 0)
                    drag *= _prevThrottle > DragThrottleThreshold ? DragThrottleMultiplierBase + _prevThrottle * DragThrottleMultiplier : DragIdleMultiplier;

                return -_bike.Velocity * (drag * speed);
            }, new Vector(0, 0));
        }

        public Vector CalculateSlopeForce(Level level)
        {
            return Log("ForcesComponent", "calculating slope force", () =>
            {
                if (_bike.IsInAir) return new Vector(0, 0);

                double frontAngle = level.CalculateSlopeAngle(_bike.WheelPositions.Front.X);
                double rearAngle = level.CalculateSlopeAngle(_bike.WheelPositions.Rear.X);

                Vector frontForce = CalculateWheelSlopeForce(frontAngle);
                Vector rearForce = CalculateWheelSlopeForce(rearAngle);
                return (frontForce + rearForce) * 0.5;
            }, new Vector(0, 0));
        }

        private Vector CalculateWheelSlopeForce(double slopeAngle)
        {
            return Log("ForcesComponent", "calculating wheel slope force", () =>
            {
                double forceMagnitude = _physics.Mass * _physics.Gravity * Sin(slopeAngle) * 0.5;
                return new Vector(
                    forceMagnitude * Cos(slopeAngle),
                    -forceMagnitude * Sin(slopeAngle)
                );
            }, new Vector(0, 0));
        }

        public Vector CalculateTotalForce(Level level, double deltaTime)
        {
            return Log("ForcesComponent", "calculating total force", () =>
            {
                return new Vector(0, _physics.Gravity * _physics.Mass) +
                    CalculateThrustForce(deltaTime) +
                    CalculateBrakeForce(deltaTime) +
                    CalculateDragForce() +
                    CalculateSlopeForce(level);
            }, new Vector(0, _physics.Gravity * _physics.Mass));
        }
    }

    public class TorqueComponent : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;
        private double _prevTorque;

        public TorqueComponent(Motorcycle bike, BikePhysics physics) : base() =>
            (_bike, _physics, _prevTorque) = (bike, physics, 0);

        public double CalculateBaseTorque(double deltaTime)
        {
            return Log("TorqueComponent", "calculating base torque", () =>
            {
                if (_bike.Throttle <= 0 && _prevTorque <= TorqueIdleThreshold) return 0;

                double force = _bike.Throttle * _physics.EnginePower;
                var (cos, sin) = _physics.GetBikeTrigs();
                Vector thrust = new(cos * force, sin * force);
                Vector r = _bike.WheelPositions.Rear - _bike.Position;
                double torque = r.X * thrust.Y - r.Y * thrust.X;

                double rate = Min(1.0, TorqueSmoothingFactor * deltaTime);
                _prevTorque = _bike.Throttle <= TorqueFadeThreshold && _prevTorque > 0 ?
                    _prevTorque * (1.0 - rate) :
                    _prevTorque + (torque - _prevTorque) * rate;

                return _prevTorque;
            }, 0.0);
        }

        public double CalculateWheelieTorque(double deltaTime)
        {
            return Log("TorqueComponent", "calculating wheelie torque", () =>
            {
                return CalculateTrickTorque(
                    deltaTime, true, _bike.Throttle, WheelieThrottleMinimum,
                    WheelieThrottleMultiplier * _physics.EnginePower, WheelieForceBase,
                    _physics.EnginePower * WheelieMaxForceMultiplier, CalculateWheelieSpeedEfficiency,
                    CalculateWheelieAngleEfficiency, CalculateWheelieBalanceFactor,
                    UpdateWheelieBalance);
            }, 0.0);
        }

        public double CalculateStoppieTorque(double deltaTime)
        {
            return Log("TorqueComponent", "calculating stoppie torque", () =>
            {
                return CalculateTrickTorque(
                    deltaTime, false, _bike.Brake, StoppieThresholdMinimum,
                    StoppieBrakeMultiplier * _physics.BrakeForce, StoppieForceBase,
                    _physics.BrakeForce * StoppieMaxForceMultiplier, CalculateStoppieSpeedEfficiency,
                    CalculateStoppieAngleEfficiency, CalculateStoppieBalanceFactor,
                    UpdateStoppieBalance);
            }, 0.0);
        }

        private double CalculateTrickTorque(
            double deltaTime, bool isWheelie, double inputValue, double minThreshold,
            double forceMultiplier, double baseForce, double maxForce,
            Func<double> speedFunc, Func<double> angleFunc, Func<double> balanceFunc,
            Action<double> updateBalance)
        {
            return Log("TorqueComponent", "calculating trick torque", () =>
            {
                if (_bike.IsInAir || inputValue <= minThreshold) return 0;

                string trickName = isWheelie ? "wheelie" : "stoppie";
                bool isInTrickState = isWheelie ? _bike.IsInWheelie : _bike.IsInStoppie;
                double torque = 0;
                double speed = _bike.Velocity.Length;

                bool validSpeed = isWheelie ? speed > 0 : speed > StoppieMinSpeed;
                if (validSpeed)
                {
                    double efficiency = speedFunc() * angleFunc();
                    double force = (inputValue - minThreshold) * forceMultiplier * baseForce * efficiency;
                    force = SanitizeValue(force, 0.0, "Invalid trick force calculated");

                    if (force > maxForce)
                    {
                        Warning("TorqueComponent", $"{trickName} force exceeded max limit: {force} > {maxForce}");
                        force = maxForce;
                    }

                    if (isInTrickState) force *= balanceFunc();
                    torque += isWheelie ? force : -force;
                }

                if (isInTrickState)
                {
                    updateBalance(deltaTime);
                    double controlMult = isWheelie ? WheelieControlMultiplier : StoppieControlMultiplier;
                    double balanceStrength = isWheelie ? WheelieBalanceStrength : StoppieBalanceStrength;
                    double balanceValue = isWheelie ? _physics.WheelieBalance : _physics.StoppieBalance;

                    double control = _bike.LeanAmount * controlMult;
                    double balance = balanceValue * balanceStrength + (isWheelie ? control : -control);
                    balance = SanitizeValue(balance, 0.0, "Invalid balance torque calculated");

                    if (isWheelie && _bike.Brake > 0)
                        balance += _bike.Brake * WheelieStabilizationFactor;
                    else if (!isWheelie && _bike.Throttle > 0)
                        balance -= _bike.Throttle * StoppieStabilizationFactor;

                    torque += balance;
                }

                return torque;
            }, 0.0);
        }

        private double CalculateEfficiency(
            double value, double minOptimal, double maxOptimal,
            double minValue, double maxValue, double lowEff, double highEff)
        {
            return Log("TorqueComponent", "calculating efficiency", () =>
            {
                if (value < minValue) return 0.0;
                if (value < minOptimal) return Lerp(lowEff, 1.0, (value - minValue) / (minOptimal - minValue));
                if (value > maxOptimal) return Lerp(1.0, highEff, (value - maxOptimal) / (maxValue - maxValue));
                return 1.0;
            }, 0.5);
        }

        private double CalculateWheelieSpeedEfficiency()
        {
            return Log("TorqueComponent", "calculating wheelie speed efficiency", () =>
            {
                return CalculateEfficiency(_bike.Velocity.Length, WheelieOptimalMinSpeed, WheelieOptimalMaxSpeed,
                                        0, WheelieMaxSpeed, WheelieLowSpeedEfficiency, WheelieHighSpeedEfficiency);
            }, 0.5);
        }

        private double CalculateStoppieSpeedEfficiency()
        {
            return Log("TorqueComponent", "calculating stoppie speed efficiency", () =>
            {
                return CalculateEfficiency(_bike.Velocity.Length, StoppieOptimalMinSpeed, StoppieOptimalMaxSpeed,
                                       StoppieMinSpeed, StoppieMaxSpeed, StoppieLowSpeedEfficiency, StoppieHighSpeedEfficiency);
            }, 0.5);
        }

        private double CalculateWheelieAngleEfficiency()
        {
            return Log("TorqueComponent", "calculating wheelie angle efficiency", () =>
            {
                return CalculateEfficiency(_bike.Angle, -WheelieOptimalAngle, 0,
                                       -WheelieOptimalAngle - WheelieAngleMinOffset, WheelieAngleMaxOffset,
                                       WheelieLowAngleEfficiency, WheelieHighAngleEfficiency);
            }, 0.5);
        }

        private double CalculateStoppieAngleEfficiency()
        {
            return Log("TorqueComponent", "calculating stoppie angle efficiency", () =>
            {
                return CalculateEfficiency(_bike.Angle, -StoppieOptimalAngle, 0,
                                       -StoppieOptimalAngle - StoppieAngleMinOffset, StoppieAngleMaxOffset,
                                       StoppieLowAngleEfficiency, StoppieHighAngleEfficiency);
            }, 0.5);
        }

        private double CalculateWheelieBalanceFactor()
        {
            return Log("TorqueComponent", "calculating wheelie balance factor", () =>
            {
                double diff = Abs(_bike.Angle - WheelieBalanceAngle);
                return diff < WheelieBalanceTolerance ? Lerp(WheelieBalanceMinFactor, 1.0, diff / WheelieBalanceTolerance) : 1.0;
            }, 0.5);
        }

        private double CalculateStoppieBalanceFactor()
        {
            return Log("TorqueComponent", "calculating stoppie balance factor", () =>
            {
                double diff = Abs(_bike.Angle - StoppieBalanceAngle);
                return diff < StoppieBalanceTolerance ? Lerp(StoppieBalanceMinFactor, 1.0, diff / StoppieBalanceTolerance) : 1.0;
            }, 0.5);
        }

        private void UpdateWheelieBalance(double deltaTime)
        {
            Log("TorqueComponent", "updating wheelie balance", () =>
            {
            double diff = _bike.Angle - WheelieBalanceAngle;
            _physics.WheelieBalance = SanitizeValue(-diff * WheelieBalanceResponseFactor,
                                                   0.0, "Invalid wheelie balance value calculated");
            _bike.WheelieTime = SanitizeValue(_bike.WheelieTime + deltaTime, 0.0, "Invalid wheelie time value");

                if (_bike.WheelieTime > WheelieEasyTime)
                {
                    double factor = Min(1.0, (_bike.WheelieTime - WheelieEasyTime) / WheelieHardTimeDelta);
                    _physics.WheelieBalance *= (1.0 - factor * WheelieProgressiveDifficulty);
                }
            });
        }

        private void UpdateStoppieBalance(double deltaTime)
        {
            Log("TorqueComponent", "updating stoppie balance", () =>
            {
                double diff = _bike.Angle - StoppieBalanceAngle;
                _physics.StoppieBalance = SanitizeValue(-diff * StoppieBalanceResponseFactor,
                                                       0.0, "Invalid stoppie balance value calculated");
                _bike.StoppieTime = SanitizeValue(_bike.StoppieTime + deltaTime, 0.0, "Invalid stoppie time value");

                if (_bike.StoppieTime > StoppieEasyTime)
                {
                    double factor = Min(1.0, (_bike.StoppieTime - StoppieEasyTime) / StoppieHardTimeDelta);
                    _physics.StoppieBalance *= (1.0 - factor * StoppieProgressiveDifficulty);
                }
            });
        }

        public double CalculateTotalTorque(double deltaTime)
        {
            return Log("TorqueComponent", "calculating total torque", () =>
            {
                return CalculateBaseTorque(deltaTime) +
                    CalculateWheelieTorque(deltaTime) +
                    CalculateStoppieTorque(deltaTime);
            }, 0.0);
        }
    }

    public class TricksComponent
    {
        private readonly Motorcycle _bike;
        private readonly BikePhysics _physics;

        public TricksComponent(Motorcycle bike, BikePhysics physics) => (_bike, _physics) = (bike, physics);

        public void UpdateTrickStates(double deltaTime)
        {
            Log("TricksComponent", "updating trick states", () =>
            {
                bool wasInWheelie = _bike.IsInWheelie;
                bool wasInStoppie = _bike.IsInStoppie;

                _bike.IsInWheelie = !_bike.IsInAir &&
                    _bike.WheelPositions.Front.Y < _bike.WheelPositions.Rear.Y - _physics.WheelRadius * WheelieHeightFactor &&
                    _bike.Angle > WheelieMinAngle;

                _bike.IsInStoppie = !_bike.IsInAir &&
                    _bike.WheelPositions.Rear.Y < _bike.WheelPositions.Front.Y - _physics.WheelRadius * StoppieHeightFactor &&
                    _bike.Angle < -StoppieMinAngle;

                double wheelieTime = _bike.WheelieTime;
                double stoppieTime = _bike.StoppieTime;

                UpdateTrickTime(wasInWheelie, _bike.IsInWheelie, ref wheelieTime);
                UpdateTrickTime(wasInStoppie, _bike.IsInStoppie, ref stoppieTime);

                _bike.WheelieTime = wheelieTime;
                _bike.StoppieTime = stoppieTime;

                if (_bike.IsInWheelie && !wasInWheelie)
                    Info("TricksComponent", "Wheelie started");
                else if (!_bike.IsInWheelie && wasInWheelie)
                    Info("TricksComponent", $"Wheelie ended, duration: {wheelieTime:F2}s");

                if (_bike.IsInStoppie && !wasInStoppie)
                    Info("TricksComponent", "Stoppie started");
                else if (!_bike.IsInStoppie && wasInStoppie)
                    Info("TricksComponent", $"Stoppie ended, duration: {stoppieTime:F2}s");
            });
        }

        private void UpdateTrickTime(bool wasActive, bool isActive, ref double trickTime)
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
        private double _prevFrontPenetration, _prevRearPenetration;
        private double _prevFrontCompression, _prevRearCompression;
        private Vector _prevFrontReactionForce, _prevRearReactionForce;
        private const double SlopeGripReductionFactor = 0.5;
        private const double MinGripThreshold = 0.1;
        private const double MinPenetrationThreshold = 0.01;
        private const double SurfaceNormalSampleDelta = 0.1;
        private const double MinNormalY = 0.2;
        private const double MaxAllowedPenetration = 5.0;

        public SuspensionComponent(Motorcycle bike, BikePhysics physics) : base()
        {
            _bike = bike;
            _physics = physics;
            _prevFrontReactionForce = _prevRearReactionForce = new Vector(0, 0);
        }

        private Vector CalculateSurfaceNormal(Level level, double x)
        {
            return Log("SuspensionComponent", "calculating surface normal", () =>
            {
                double slopeAngle = level.CalculateSlopeAngle(x);
                Vector normal = new Vector(-Math.Sin(slopeAngle), Math.Cos(slopeAngle));

                if (normal.Y < 0)
                {
                    normal = new Vector(-normal.X, -normal.Y);
                }

                if (Math.Abs(normal.Y) < MinNormalY)
                {
                    double normalLength = normal.Length;
                    double newY = MinNormalY;
                    double newX = Math.Sign(normal.X) * Math.Sqrt(normalLength * normalLength - newY * newY);
                    normal = new Vector(newX, newY);
                }

                return normal;
            }, new Vector(0, 1));
        }

        private double CalculateGripFactor(double slopeAngle)
        {
            return Log("SuspensionComponent", "calculating grip factor", () =>
            {
                double grip = _physics.GroundFriction * (1.0 - Abs(Sin(slopeAngle)) * SlopeGripReductionFactor);
                return Max(grip, MinGripThreshold);
            }, MinGripThreshold);
        }

        public void HandleWheelCollisions(Level level, double deltaTime)
        {
            Log("SuspensionComponent", "handling wheel collisions", () =>
            {
                double frontGroundY = level.GetGroundYAtX(_bike.WheelPositions.Front.X);
                double rearGroundY = level.GetGroundYAtX(_bike.WheelPositions.Rear.X);

                Vector frontNormal = CalculateSurfaceNormal(level, _bike.WheelPositions.Front.X);
                Vector rearNormal = CalculateSurfaceNormal(level, _bike.WheelPositions.Rear.X);

                bool frontContact = HandleWheelCollision(true, frontGroundY, frontNormal, level, deltaTime);
                bool rearContact = HandleWheelCollision(false, rearGroundY, rearNormal, level, deltaTime);

                bool wasInAir = _bike.IsInAir;
                _bike.State = (frontContact || rearContact)
                    ? _bike.State & ~BikeState.InAir
                    : _bike.State | BikeState.InAir;

                if (wasInAir && !_bike.IsInAir)
                {
                    Info("SuspensionComponent", "Landed");
                }
                else if (!wasInAir && _bike.IsInAir)
                {
                    Info("SuspensionComponent", "Became airborne");
                }
            });
        }

        public bool HandleWheelCollision(bool isFrontWheel, double groundY, Vector normal, Level level, double deltaTime)
        {
            return Log("SuspensionComponent", $"handling {(isFrontWheel ? "front" : "rear")} wheel collision", () =>
            {
                Point wheelPos = isFrontWheel ? _bike.WheelPositions.Front : _bike.WheelPositions.Rear;
                double penetration = wheelPos.Y + _physics.WheelRadius - groundY;

                if (penetration <= 0)
                {
                    SetSuspensionOffset(isFrontWheel, _physics.SuspensionRestLength);
                    return false;
                }

                ProcessWheelPenetration(isFrontWheel, wheelPos, penetration, normal, level, deltaTime);
                return true;
            }, false);
        }

        private void ProcessWheelPenetration(bool isFrontWheel, Point wheelPos, double penetration, Vector normal, Level level, double deltaTime)
        {
            Log("SuspensionComponent", $"processing {(isFrontWheel ? "front" : "rear")} wheel penetration", () =>
            {
                double prevPenetration = isFrontWheel ? _prevFrontPenetration : _prevRearPenetration;
                double penetrationVelocity = (penetration - prevPenetration) / deltaTime;
                double relativeVelocity = Vector.Multiply(_bike.Velocity, normal);

                if (isFrontWheel)
                    _prevFrontPenetration = penetration;
                else
                    _prevRearPenetration = penetration;

                if (penetration < MinPenetrationThreshold)
                {
                    SetSuspensionOffset(isFrontWheel, _physics.SuspensionRestLength);
                    return;
                }

                if (penetration > _physics.WheelRadius * MaxWheelPenetration)
                {
                    string wheel = isFrontWheel ? "front" : "rear";
                    Warning("SuspensionComponent", $"Excessive {wheel} wheel penetration: {penetration:F2}");
                    penetration = Min(penetration, _physics.WheelRadius * MaxWheelPenetration);
                }

                double baseCompression = penetration * BaseCompressionMultiplier;
                double progressiveFactor = ProgressiveFactorBase + Pow(penetration / (_physics.WheelRadius * WheelRadiusHalfFactor), 2) * ProgressiveFactorMultiplier;
                double desiredCompression = baseCompression * progressiveFactor;

                double prevCompression = isFrontWheel ? _prevFrontCompression : _prevRearCompression;
                double smoothedCompression = Lerp(prevCompression, desiredCompression, CompressionSmoothingFactor);

                if (isFrontWheel)
                    _prevFrontCompression = smoothedCompression;
                else
                    _prevRearCompression = smoothedCompression;

                double newOffset = _physics.SuspensionRestLength - smoothedCompression;
                double currentOffset = isFrontWheel ? _bike.SuspensionOffsets.Front : _bike.SuspensionOffsets.Rear;

                if (Abs(newOffset - currentOffset) > _physics.SuspensionRestLength * LargeSuspensionChangeThreshold)
                    newOffset = Lerp(currentOffset, newOffset, LargeSuspensionChangeSmoothingFactor);

                newOffset = ClampValue(newOffset, _physics.SuspensionRestLength * MinSuspensionOffset, _physics.SuspensionRestLength);

                double compressionRatio = CompressionRatioBase - newOffset / _physics.SuspensionRestLength;
                double progressiveFactorForce = ProgressiveFactorBase + Pow(compressionRatio, 2) * ProgressiveFactorForceMultiplier;
                double adjustedSuspensionStrength = _physics.SuspensionStrength * progressiveFactorForce;

                double penetrationDamping = -_physics.SuspensionDamping * penetrationVelocity;
                double velocityDamping = -_physics.SuspensionDamping * relativeVelocity * VelocityDampingFactor;

                Vector suspensionForce = normal * (-adjustedSuspensionStrength * penetration);
                Vector dampingForce = normal * (penetrationDamping + velocityDamping);

                Vector tangent = new Vector(-normal.Y, normal.X);
                double tangentialVelocity = Vector.Multiply(_bike.Velocity, tangent);
                double slopeAngle = Atan2(normal.Y, normal.X) - PI / 2;
                double gripFactor = CalculateGripFactor(slopeAngle);
                double maxFriction = gripFactor * suspensionForce.Length;

                Vector frictionForce = tangent * (-tangentialVelocity * _physics.GroundFriction * FrictionForceMultiplier);
                bool isSlipping = frictionForce.Length > maxFriction;

                if (isSlipping)
                {
                    string wheel = isFrontWheel ? "Front" : "Rear";
                    Info("SuspensionComponent", $"{wheel} wheel slipping");
                    frictionForce = frictionForce * (maxFriction / frictionForce.Length);
                }

                Vector reactionForce = suspensionForce + dampingForce + frictionForce;
                Vector prevReactionForce = isFrontWheel ? _prevFrontReactionForce : _prevRearReactionForce;
                reactionForce = LerpVector(prevReactionForce, reactionForce, ReactionForceSmoothingFactor);

                if (isFrontWheel)
                    _prevFrontReactionForce = reactionForce;
                else
                    _prevRearReactionForce = reactionForce;

                Vector r = wheelPos - _bike.Position;
                double torque = r.X * reactionForce.Y - r.Y * reactionForce.X;

                _bike.Velocity += reactionForce / _physics.Mass * deltaTime;
                _bike.AngularVelocity += torque / _physics.MomentOfInertia * deltaTime;

                SetSuspensionOffset(isFrontWheel, newOffset);
            });
        }

        private void SetSuspensionOffset(bool isFrontWheel, double offset)
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

        public void CheckFrameCollision(Level level, double deltaTime)
        {
            Log("CollisionComponent", "checking frame collision", () =>
            {
                if (_bike.IsInAir || _bike.IsCrashed || _bike.Velocity.Length <= FrameCollisionMinVelocity)
                    return;

                var collisionInfo = DetectFrameCollision(level);
                if (collisionInfo.IsCollision && collisionInfo.MaxPenetration > _physics.WheelRadius * FrameCollisionMinPenetration)
                    HandleFrameCollision(collisionInfo, deltaTime);
            });
        }

        private void HandleFrameCollision((bool IsCollision, Point CollisionPoint, double MaxPenetration) collisionInfo, double deltaTime)
        {
            Log("CollisionComponent", "handling frame collision", () =>
            {
                double crashThreshold = _physics.WheelRadius * FrameCrashThreshold;
                bool isBadAngle = Abs(_bike.Angle) > FrameCriticalBackwardTiltAngle;
                bool isHighSpeed = _bike.Velocity.Length > FrameCollisionHighSpeedThreshold;

                if (collisionInfo.MaxPenetration > crashThreshold && (isHighSpeed || isBadAngle))
                {
                    _bike.State |= BikeState.Crashed;
                    Info("CollisionComponent", "Bike crashed due to frame collision");
                }
                else
                {
                    ApplyFrameCollisionResponse(collisionInfo.CollisionPoint, collisionInfo.MaxPenetration, deltaTime);
                }
            });
        }

        private void ApplyFrameCollisionResponse(Point collisionPoint, double penetration, double deltaTime)
        {
            Log("CollisionComponent", "applying frame collision response", () =>
            {
                double reactionForce = penetration * _physics.SuspensionStrength * FrameCollisionReactionForce;
                double impulse = reactionForce * deltaTime;
                double deltaVelocityY = -impulse / _physics.Mass;

                deltaVelocityY = Max(deltaVelocityY, -FrameCollisionMaxDeltaVelocity * deltaTime);
                _bike.Velocity = new Vector(_bike.Velocity.X, _bike.Velocity.Y + deltaVelocityY);

                double stabilizingFactor = Abs(_bike.Angle) > FrameStabilizingAngleThreshold
                    ? FrameStabilizingFactorStrong
                    : FrameStabilizingFactorBase;

                double stabilizingTorque = -_bike.Angle * stabilizingFactor;
                double deltaAngularVelocity = stabilizingTorque * deltaTime / _physics.MomentOfInertia;

                deltaAngularVelocity = ClampValue(deltaAngularVelocity,
                                                -FrameCollisionMaxDeltaAngular * deltaTime,
                                                 FrameCollisionMaxDeltaAngular * deltaTime);

                _bike.AngularVelocity += deltaAngularVelocity;
            });
        }

        private (bool IsCollision, Point CollisionPoint, double MaxPenetration) DetectFrameCollision(Level level)
        {
            return Log("CollisionComponent", "detecting frame collision", () =>
            {
                var framePoints = _bike.GetFramePoints();
                bool frameCollision = false;
                Point collisionPoint = default;
                double maxPenetration = 0;

                foreach (var point in framePoints)
                {
                    double groundY = level.GetGroundYAtX(point.X);
                    double penetration = point.Y - groundY;

                    if (penetration > 0 && penetration > maxPenetration)
                    {
                        frameCollision = true;
                        maxPenetration = penetration;
                        collisionPoint = point;
                    }
                }

                return (frameCollision, collisionPoint, maxPenetration);
            }, (false, new Point(0, 0), 0));
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
        private const double MaxAllowedPenetration = 5.0;

        private double _prevAngularVelocity;
        private Vector _prevVelocity = new(0, 0);
        private int _updateCounter;
        private double _prevSlopeAngle;

        public double BrakeForce { get; private set; }
        public double DragCoefficient { get; private set; }
        public double EnginePower { get; private set; }
        public double Gravity { get; private set; }
        public double GroundFriction { get; private set; }
        public double LeanSpeed { get; private set; }
        public double Mass { get; private set; }
        public double MaxLeanAngle { get; private set; }
        public double MaxWheelDistance { get; private set; }
        public double MinWheelDistance { get; private set; }
        public double MomentOfInertia { get; private set; }
        public double NominalWheelBase { get; private set; }
        public double SuspensionDamping { get; private set; }
        public double SuspensionRestLength { get; private set; }
        public double SuspensionStrength { get; private set; }
        public double WheelRadius { get; private set; } = DefaultWheelRadius;
        public double WheelieBalance { get; set; }
        public double StoppieBalance { get; set; }
        public double MaxSuspensionAngle { get; private set; }
        public (double cos, double sin) GetBikeTrigs() => (_trigCache.Cos, _trigCache.Sin);

        public BikePhysics(Motorcycle bike) : base()
        {
            _bike = bike;
            Gravity = DefaultGravity * GravityMultiplier;
            _prevVelocity = new Vector(0, 0);
            _prevAngularVelocity = 0;
            _updateCounter = 0;
            _prevSlopeAngle = 0;
            WheelieBalance = StoppieBalance = 0.0;

            _forcesComponent = new ForcesComponent(bike, this);
            _torqueComponent = new TorqueComponent(bike, this);
            _tricksComponent = new TricksComponent(bike, this);
            _suspensionComponent = new SuspensionComponent(bike, this);
            _collisionComponent = new CollisionComponent(bike, this);

            _stateComponent = new StateComponent(_bike, this);
            _inputComponent = new InputComponent(bike);
            _kinematicsComponent = new KinematicsComponent(bike, this);
            _validationComponent = new ValidationComponent(bike, this);
            _geometryComponent = new BikeGeom(bike);
        }

        public void InitializeProperties(BikeType bikeType)
        {
            Log("BikePhysics", "initializing bike properties", () =>
            {
                var props = GetBikeProperties(bikeType);
                var wheelProps = GetWheelProperties(bikeType);

                (Mass, EnginePower, BrakeForce, DragCoefficient) = (props.mass, props.power, props.brakeForce, props.drag);
                (MaxLeanAngle, LeanSpeed) = (props.maxLeanAngle, props.leanSpeed);
                GroundFriction = props.friction * wheelProps.friction;
                SuspensionStrength = props.suspensionStrength * wheelProps.suspensionStrength;
                SuspensionDamping = props.suspensionDamping * wheelProps.suspensionDamping;
                SuspensionRestLength = props.suspensionRestLength;
                MaxSuspensionAngle = props.maxSuspensionAngle;
                WheelRadius = wheelProps.radius;
                MomentOfInertia = Mass * Pow(_bike.WheelBase / 2, 2) * MomentOfInertiaMultiplier;
                NominalWheelBase = _bike.WheelBase;
                MinWheelDistance = NominalWheelBase * WheelDistanceMinRatio;
                MaxWheelDistance = NominalWheelBase * WheelDistanceMaxRatio;

                Info("BikePhysics", $"Initialized {bikeType} bike with mass: {Mass}, power: {EnginePower}, brake: {BrakeForce}");
            });
        }

        public void Reset()
        {
            Log("BikePhysics", "resetting bike", () =>
            {
                _stateComponent.Reset();
                _kinematicsComponent.UpdateAttachmentPoints();
                _kinematicsComponent.UpdateWheelPositions();
                Info("BikePhysics", "Bike reset to initial state");
            });
        }

        public void SetPosition(Point position)
        {
            Log("BikePhysics", "setting position", () =>
            {
                _bike.Position = position;
                _kinematicsComponent.UpdateAttachmentPoints();
                _kinematicsComponent.UpdateWheelPositions();
            });
        }

        public void SetBikeType(BikeType bikeType)
        {
            Log("BikePhysics", $"setting bike type to {bikeType}", () =>
            {
                InitializeProperties(bikeType);
            });
        }

        public void ApplyThrottle(double amount) => _inputComponent.ApplyThrottle(amount);
        public void ApplyBrake(double amount) => _inputComponent.ApplyBrake(amount);
        public void Lean(double direction) => _inputComponent.Lean(direction);

        public List<Point> GetFramePoints() => _geometryComponent.GetFramePoints();
        public (List<BikeGeom.SkeletonPoint> Points, List<BikeGeom.SkeletonLine> Lines) GetSkeleton() =>
            _geometryComponent.GetSkeleton();

        public void Update(double deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            if (_stateComponent.ShouldSkipUpdate(cancellationToken))
                return;

            Log("BikePhysics", "updating", () =>
            {
                _stateComponent.SavePreviousState();
                UpdateCycle(deltaTime, level, cancellationToken);
                _stateComponent.LogStateChanges();
            });
        }

        private void UpdateCycle(double deltaTime, Level level, CancellationToken cancellationToken)
        {
            if (cancellationToken.IsCancellationRequested)
                return;

            UpdatePhysics(deltaTime, level, cancellationToken);
            _kinematicsComponent.UpdateKinematics(deltaTime);
            _validationComponent.ValidateAndSanitizeState(deltaTime);
        }

        public void UpdatePhysics(double deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            if (cancellationToken.IsCancellationRequested) return;

            Log("BikePhysics", "updating physics", () =>
            {
                _trigCache.Update(_bike.Angle);
                SavePreviousState();
                _suspensionComponent.HandleWheelCollisions(level, deltaTime);

                Vector totalForce = _forcesComponent.CalculateTotalForce(level, deltaTime);
                _bike.Velocity += totalForce / Mass * deltaTime;

                double totalTorque = _torqueComponent.CalculateTotalTorque(deltaTime);
                ValidateVectorParameter("TotalForce", totalForce, Mass * MaxSafeAcceleration, "BikePhysics");
                double maxTorque = MomentOfInertia * MaxAngularVelocity / deltaTime;
                ValidatePhysicalParameter("TotalTorque", Abs(totalTorque), 0, maxTorque, "BikePhysics");

                _bike.AngularVelocity += totalTorque / MomentOfInertia * deltaTime;

                UpdateLean(deltaTime, level);
                _bike.Angle = NormalizeAngle(_bike.Angle + _bike.AngularVelocity * deltaTime);

                double groundY = level.GetGroundYAtX(_bike.Position.X);
                if (_bike.Position.Y > groundY + MaxAllowedPenetration)
                {
                    Warning("BikePhysics", $"Excessive ground penetration: {_bike.Position.Y - groundY}");
                    _bike.Position = new Point(_bike.Position.X, groundY);
                    _bike.Velocity = new Vector(_bike.Velocity.X, 0);
                }

                _kinematicsComponent.UpdateAttachmentPoints();
                _kinematicsComponent.UpdateWheelPositions();

                double frontGroundY = level.GetGroundYAtX(_bike.WheelPositions.Front.X);
                double rearGroundY = level.GetGroundYAtX(_bike.WheelPositions.Rear.X);

                if (_bike.WheelPositions.Front.Y > frontGroundY + WheelRadius + MaxAllowedPenetration ||
                    _bike.WheelPositions.Rear.Y > rearGroundY + WheelRadius + MaxAllowedPenetration)
                {
                    Warning("BikePhysics", "Wheel penetration exceeded maximum allowed value");
                    double centerGroundY = level.GetGroundYAtX(_bike.Position.X);
                    _bike.Position = new Point(_bike.Position.X, centerGroundY - WheelRadius);
                    _bike.Velocity = new Vector(_bike.Velocity.X, 0);
                    _kinematicsComponent.UpdateAttachmentPoints();
                    _kinematicsComponent.UpdateWheelPositions();
                }

                _collisionComponent.CheckFrameCollision(level, deltaTime);
                CheckCrashConditions();
                _tricksComponent.UpdateTrickStates(deltaTime);

                if (++_updateCounter >= StatusLogInterval)
                {
                    Debug("BikePhysics", $"Status: vel={_bike.Velocity.Length:F2}, angle={_bike.Angle:F2}, inAir={_bike.IsInAir}");
                    _updateCounter = 0;
                }

                SanitizePhysicalState();
            });
        }

        private void CheckCrashConditions()
        {
            Log("BikePhysics", "checking crash conditions", () =>
            {
                if (_bike.IsInAir || _bike.IsCrashed)
                    return;

                if (Abs(_bike.Angle) > CriticalLeanAngle)
                {
                    _bike.State |= BikeState.Crashed;
                    Info("BikePhysics", $"Bike crashed due to critical lean angle: {_bike.Angle:F2}");
                    return;
                }

                double currentDistance = CalculateDistance(_bike.WheelPositions.Front, _bike.WheelPositions.Rear);
                if (currentDistance < MinWheelDistance || currentDistance > MaxWheelDistance)
                {
                    _bike.State |= BikeState.Crashed;
                    Info("BikePhysics", $"Bike crashed due to invalid wheel distance: {currentDistance:F2}");
                }
            });
        }

        private void UpdateLean(double deltaTime, Level level)
        {
            Log("BikePhysics", "updating lean", () =>
            {
            if (_bike.IsInAir)
            {
                double desiredAngularVelocity = _bike.LeanAmount * MaxAirAngularVelocity;
                _bike.AngularVelocity += (desiredAngularVelocity - _bike.AngularVelocity) * AirDampingFactor * deltaTime;
                return;
            }

                double currentSlopeAngle = level.CalculateSlopeAngle(_bike.Position.X);
                double smoothedSlopeAngle = Abs(currentSlopeAngle - _prevSlopeAngle) > SlopeAngleSmoothingThreshold
                    ? _prevSlopeAngle + (currentSlopeAngle - _prevSlopeAngle) * Min(1.0, SlopeTransitionRate * deltaTime)
                    : _prevSlopeAngle;
                _prevSlopeAngle = smoothedSlopeAngle;

                double speedFactor = Min(1.0, _bike.Velocity.Length / LeanSpeedFactorDenominator);
                double adaptiveLeanSpeed = LeanSpeed * (LeanSpeedBaseMultiplier + LeanSpeedFactorMultiplier * speedFactor);
                double terrainAdaptationFactor = Min(1.0, _bike.Velocity.Length / TerrainAdaptationSpeedThreshold);
                double slopeInfluence = smoothedSlopeAngle * terrainAdaptationFactor;

                double targetLean = _bike.LeanAmount * MaxLeanAngle + slopeInfluence + _bike.Throttle * ThrottleLeanInfluence;
                double leanError = targetLean - _bike.Angle;
                double angularVelocityDamping = AngularVelocityDampingBase + AngularVelocityDampingFactor * speedFactor;

                double controlTorque = LeanControlTorqueMultiplier * leanError * adaptiveLeanSpeed + angularVelocityDamping * (-_bike.AngularVelocity);

                if (_bike.Throttle < InputIdleThreshold && _bike.Brake < InputIdleThreshold && Abs(_bike.AngularVelocity) > AngularVelocityIdleThreshold)
                {
                    double stabilizationFactor = StabilizationFactorBase + StabilizationFactorSpeedMultiplier * Min(1.0, _bike.Velocity.Length / StabilizationSpeedThreshold);
                    controlTorque += -_bike.AngularVelocity * stabilizationFactor * StabilizationTorqueMultiplier;
                }

                _bike.AngularVelocity += controlTorque / MomentOfInertia * deltaTime;
                double maxAngularVel = MaxAngularVelocity * GroundAngularVelocityFactor;
                _bike.AngularVelocity = ClampValue(_bike.AngularVelocity, -maxAngularVel, maxAngularVel);
            });
        }

        private void SavePreviousState()
        {
            _prevVelocity = _bike.Velocity;
            _prevAngularVelocity = _bike.AngularVelocity;
        }

        private void SanitizePhysicalState()
        {
            Log("BikePhysics", "sanitizing physical state", () =>
            {
                _bike.Angle = NormalizeAngle(SanitizeValue(_bike.Angle, 0, "Invalid angle value"));
                _bike.AngularVelocity = ClampValue(SanitizeValue(_bike.AngularVelocity, 0, "Invalid angular velocity"),
                                                -MaxAngularVelocity, MaxAngularVelocity);
                _bike.Velocity = SanitizeVector(_bike.Velocity, new Vector(0, 0), "Invalid velocity");

                if (_bike.Velocity.Length > MaxSafeVelocity)
                {
                    Warning("BikePhysics", $"Velocity exceeds max safe value: {_bike.Velocity.Length:F2} > {MaxSafeVelocity:F2}");
                    _bike.Velocity = _bike.Velocity * (MaxSafeVelocity / _bike.Velocity.Length);
                }
            });
        }

        private BikeProperties GetBikeProperties(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => Bike.Standard,
            BikeType.Sport => Bike.Sport,
            BikeType.OffRoad => Bike.OffRoad,
            _ => Bike.Standard
        };

        private WheelProperties GetWheelProperties(BikeType bikeType) => bikeType switch
        {
            BikeType.Standard => Wheels.Standard,
            BikeType.Sport => Wheels.Sport,
            BikeType.OffRoad => Wheels.OffRoad,
            _ => Wheels.Standard
        };

        private class StateComponent
        {
            private readonly Motorcycle _bike;
            private readonly BikePhysics _physics;
            private bool _wasInWheelie, _wasInStoppie;
            private double _airTime, _brakeHoldTime;

            public StateComponent(Motorcycle bike, BikePhysics physics)
            {
                _bike = bike;
                _physics = physics;
                _airTime = _brakeHoldTime = 0;
            }

            public void Reset()
            {
                Log("StateComponent", "resetting state", () =>
                {
                    _bike.Position = DefaultStartPosition;
                    _bike.Velocity = new Vector(0, 0);
                    _bike.Throttle = _bike.Brake = _bike.LeanAmount = 0;
                    _bike.State = BikeState.None;
                    _bike.WasInAir = _wasInWheelie = _wasInStoppie = false;
                    _bike.Angle = _bike.AngularVelocity = 0;
                    _bike.WheelRotations = (0, 0);
                    _bike.SuspensionOffsets = (_physics.SuspensionRestLength, _physics.SuspensionRestLength);
                    _brakeHoldTime = _airTime = _bike.WheelieTime = _bike.StoppieTime = 0;
                });
            }

            public bool ShouldSkipUpdate(CancellationToken token) =>
                _bike.IsCrashed || token.IsCancellationRequested;

            public void SavePreviousState()
            {
                _bike.WasInAir = _bike.IsInAir;
                _wasInWheelie = _bike.IsInWheelie;
                _wasInStoppie = _bike.IsInStoppie;
            }

            public void HandleUpdateException(Exception ex)
            {
                Error("StateComponent", $"Update exception: {ex.Message}");
                _bike.IsCrashed = true;
            }

            public void LogStateChanges()
            {
                if (_bike.IsInAir || !_bike.WasInAir || _airTime <= SignificantAirTimeThreshold)
                    return;

                Info("StateComponent", $"Air time: {_airTime:F2}s");
                _airTime = 0;
            }

            public void UpdateAirTime(double deltaTime)
            {
                if (!_bike.IsInAir)
                    return;

                _airTime += deltaTime;
            }
        }

        private class InputComponent
        {
            private readonly Motorcycle _bike;

            public InputComponent(Motorcycle bike) => _bike = bike;

            public void ApplyThrottle(double amount) =>
                _bike.Throttle = ClampValue(amount, 0, 1);

            public void ApplyBrake(double amount) =>
                _bike.Brake = ClampValue(amount, 0, 1);

            public void Lean(double direction) =>
                _bike.LeanAmount = ClampValue(direction, -1, 1);
        }

        private class KinematicsComponent
        {
            private readonly Motorcycle _bike;
            private readonly BikePhysics _physics;
            private Level _level;

            public KinematicsComponent(Motorcycle bike, BikePhysics physics) =>
                (_bike, _physics) = (bike, physics);

            public void SetLevel(Level level) => _level = level;

            public void UpdateKinematics(double deltaTime)
            {
                Log("KinematicsComponent", "updating kinematics", () =>
                {
                    UpdateAttachmentPoints();
                    UpdateWheelPositions();
                    UpdateWheelRotations(deltaTime);
                    _bike.Position += _bike.Velocity * deltaTime;
                });
            }

            public void UpdateAttachmentPoints()
            {
                Log("KinematicsComponent", "updating attachment points", () =>
                {
                    var (cosAngle, sinAngle) = GetTrigsFromAngle(_bike.Angle);
                    double halfWheelBase = _bike.WheelBase / 2;

                    _bike.AttachmentPoints = (
                        new Point(
                            _bike.Position.X + halfWheelBase * cosAngle,
                            _bike.Position.Y + halfWheelBase * sinAngle
                        ),
                        new Point(
                            _bike.Position.X - halfWheelBase * cosAngle,
                            _bike.Position.Y - halfWheelBase * sinAngle
                        )
                    );
                });
            }

            public void UpdateWheelPositions()
            {
                Log("KinematicsComponent", "updating wheel positions", () =>
                {
                    if (_level == null)
                    {
                        UpdateWheelPositionsDefault();
                        return;
                    }

                    double frontAngle = _level.CalculateSlopeAngle(_bike.AttachmentPoints.Front.X);
                    double rearAngle = _level.CalculateSlopeAngle(_bike.AttachmentPoints.Rear.X);

                    double bikeAngle = _bike.Angle;
                    double maxAngle = _physics.MaxSuspensionAngle;

                    double frontSuspensionAngle = ClampValue(
                        bikeAngle + frontAngle - PI / 2,
                        bikeAngle - maxAngle,
                        bikeAngle + maxAngle
                    );

                    double rearSuspensionAngle = ClampValue(
                        bikeAngle + rearAngle - PI / 2,
                        bikeAngle - maxAngle,
                        bikeAngle + maxAngle
                    );

                    var (cosFrontAngle, sinFrontAngle) = GetTrigsFromAngle(frontSuspensionAngle);
                    var (cosRearAngle, sinRearAngle) = GetTrigsFromAngle(rearSuspensionAngle);

                    double frontSuspOffset = _bike.SuspensionOffsets.Front;
                    double rearSuspOffset = _bike.SuspensionOffsets.Rear;

                    if (_bike.IsInWheelie && !_bike.IsInAir)
                    {
                        frontSuspOffset = _physics.SuspensionRestLength -
                            (_physics.SuspensionRestLength - frontSuspOffset) * (1.0 - _bike.WheelieIntensity * WheelieIntensityDampingMultiplier);
                    }
                    else if (_bike.IsInStoppie && !_bike.IsInAir)
                    {
                        rearSuspOffset = _physics.SuspensionRestLength -
                            (_physics.SuspensionRestLength - rearSuspOffset) * (1.0 - _bike.StoppieIntensity * WheelieIntensityDampingMultiplier);
                    }

                    _bike.WheelPositions = (
                        new Point(
                            _bike.AttachmentPoints.Front.X + frontSuspOffset * cosFrontAngle,
                            _bike.AttachmentPoints.Front.Y + frontSuspOffset * sinFrontAngle
                        ),
                        new Point(
                            _bike.AttachmentPoints.Rear.X + rearSuspOffset * cosRearAngle,
                            _bike.AttachmentPoints.Rear.Y + rearSuspOffset * sinRearAngle
                        )
                    );
                });
            }

            private void UpdateWheelPositionsDefault()
            {
                Log("KinematicsComponent", "updating wheel positions (default)", () =>
                {
                    double maxAngle = _physics.MaxSuspensionAngle;
                    double bikeAngle = _bike.Angle;
                    double verticalAngle = PI / 2;
                    double normalAngle = bikeAngle + PI / 2;
                    double angleDiff = normalAngle - verticalAngle;

                    while (angleDiff > PI) angleDiff -= 2 * PI;
                    while (angleDiff < -PI) angleDiff += 2 * PI;

                    double suspensionAngle = _bike.EnforceSuspensionAngleLimits
                        ? (Abs(angleDiff) <= maxAngle
                            ? verticalAngle
                            : normalAngle - Sign(angleDiff) * maxAngle)
                        : verticalAngle;

                    double frontSuspOffset = _bike.SuspensionOffsets.Front;
                    double rearSuspOffset = _bike.SuspensionOffsets.Rear;

                    if (_bike.IsInWheelie && !_bike.IsInAir)
                    {
                        frontSuspOffset = _physics.SuspensionRestLength -
                            (_physics.SuspensionRestLength - frontSuspOffset) * (1.0 - _bike.WheelieIntensity * WheelieIntensityDampingMultiplier);
                    }
                    else if (_bike.IsInStoppie && !_bike.IsInAir)
                    {
                        rearSuspOffset = _physics.SuspensionRestLength -
                            (_physics.SuspensionRestLength - rearSuspOffset) * (1.0 - _bike.StoppieIntensity * WheelieIntensityDampingMultiplier);
                    }

                    var (cosSuspAngle, sinSuspAngle) = GetTrigsFromAngle(suspensionAngle);

                    _bike.WheelPositions = (
                        new Point(
                            _bike.AttachmentPoints.Front.X + frontSuspOffset * cosSuspAngle,
                            _bike.AttachmentPoints.Front.Y + frontSuspOffset * sinSuspAngle
                        ),
                        new Point(
                            _bike.AttachmentPoints.Rear.X + rearSuspOffset * cosSuspAngle,
                            _bike.AttachmentPoints.Rear.Y + rearSuspOffset * sinSuspAngle
                        )
                    );
                });
            }

            private void UpdateWheelRotations(double deltaTime)
            {
                Log("KinematicsComponent", "updating wheel rotations", () =>
                {
                    var (cosAngle, sinAngle) = _physics.GetBikeTrigs();
                    double groundSpeed = Vector.Multiply(_bike.Velocity, new Vector(cosAngle, sinAngle));
                    double wheelCircumference = WheelCircumferenceFactor * _physics.WheelRadius;

                    double frontRotation = UpdateSingleWheelRotation(_bike.WheelRotations.Front, groundSpeed, deltaTime, true, wheelCircumference);
                    double rearRotation = UpdateSingleWheelRotation(_bike.WheelRotations.Rear, groundSpeed, deltaTime, false, wheelCircumference);

                    _bike.WheelRotations = (frontRotation % FullRotation, rearRotation % FullRotation);
                });
            }

            private double UpdateSingleWheelRotation(double currentRotation, double groundSpeed, double deltaTime,
                                                 bool isFrontWheel, double wheelCircumference)
            {
                return Log("KinematicsComponent", $"updating {(isFrontWheel ? "front" : "rear")} wheel rotation", () =>
                {
                    double rotationFactor = _bike.IsInAir ? AirRotationFactor : GroundRotationFactor;
                    double rotationDelta = SafeDivide(groundSpeed, wheelCircumference);

                    double slipFactor = CalculateWheelSlipFactor();

                    rotationDelta = _bike.IsInAir
                        ? isFrontWheel
                            ? rotationDelta
                            : rotationDelta + SafeDivide(_bike.Throttle * ThrottleRotationFactor, wheelCircumference)
                        : isFrontWheel
                            ? rotationDelta
                            : rotationDelta * (1 + slipFactor);

                    return currentRotation + rotationDelta * deltaTime * rotationFactor;
                }, currentRotation);
            }

            private double CalculateWheelSlipFactor()
            {
                return Log("KinematicsComponent", "calculating wheel slip factor", () =>
                {
                    if (_bike.Throttle <= WheelSlipThreshold)
                        return 0;

                    double slipFactor = (_bike.Throttle - WheelSlipThreshold) * SlipThrottleFactor;
                    slipFactor *= Min(1.0, _physics.GroundFriction / SlipFrictionRatio);
                    slipFactor *= Min(1.0, _bike.Velocity.Length / SlipSpeedThreshold);

                    return slipFactor;
                }, 0.0);
            }
        }

        private class ValidationComponent
        {
            private readonly Motorcycle _bike;
            private readonly BikePhysics _physics;
            private double _lastHighSpeedTime, _lastExtremeTiltTime;

            public ValidationComponent(Motorcycle bike, BikePhysics physics)
            {
                _bike = bike;
                _physics = physics;
                _lastHighSpeedTime = _lastExtremeTiltTime = 0;
            }

            public void ValidateAndSanitizeState(double deltaTime)
            {
                Log("ValidationComponent", "validating and sanitizing state", () =>
                {
                    SanitizePhysicalState();
                    _physics._stateComponent.UpdateAirTime(deltaTime);
                    ValidateState(deltaTime);
                });
            }

            private void SanitizePhysicalState()
            {
                Log("ValidationComponent", "sanitizing physical state", () =>
                {
                    SanitizeSuspension();
                    _bike.Position = SanitizePosition(_bike.Position, DefaultStartPosition, "Invalid position detected");
                });
            }

            private void SanitizeSuspension()
            {
                Log("ValidationComponent", "sanitizing suspension", () =>
                {
                    double restLength = _physics.SuspensionRestLength;
                    double minOffset = MinSuspensionOffset;
                    var offsets = _bike.SuspensionOffsets;

                    offsets.Front = GetValidOffset(offsets.Front, restLength, minOffset, "front");
                    offsets.Rear = GetValidOffset(offsets.Rear, restLength, minOffset, "rear");

                    _bike.SuspensionOffsets = offsets;
                });
            }

            private double GetValidOffset(double offset, double restLength, double minOffset, string wheel)
            {
                if (double.IsNaN(offset) || double.IsInfinity(offset))
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

            private void ValidateState(double deltaTime)
            {
                Log("ValidationComponent", "validating state", () =>
                {
                    double speed = _bike.Velocity.Length;
                    double absAngle = Abs(_bike.Angle);
                    CheckSuspensionCompression();
                });
            }

            private void CheckSuspensionCompression()
            {
                Log("ValidationComponent", "checking suspension compression", () =>
                {
                    double restLength = _physics.SuspensionRestLength;
                    double threshold = HighSuspensionCompressionThreshold;
                    var offsets = _bike.SuspensionOffsets;

                    double frontCompression = 1.0 - SafeDivide(offsets.Front, restLength);
                    double rearCompression = 1.0 - SafeDivide(offsets.Rear, restLength);

                    if (frontCompression > threshold)
                    {
                        Warning("ValidationComponent", $"High front suspension compression: {frontCompression:P0}");
                    }

                    if (rearCompression > threshold)
                    {
                        Warning("ValidationComponent", $"High rear suspension compression: {rearCompression:P0}");
                    }
                });
            }
        }
    }
}