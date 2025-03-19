using System;
using System.Windows;
using System.Threading;
using System.Collections.Generic;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Debug;
using static GravityDefiedGame.Utilities.GameConstants.Validation;
using static GravityDefiedGame.Utilities.GameConstants;

namespace GravityDefiedGame.Models
{
    [Flags]
    public enum BikeState
    {
        None = 0,
        InAir = 1,
        InWheelie = 2,
        MovingBackward = 4,
        Crashed = 8
    }

    public class BikePhysics : PhysicsComponent
    {
        private readonly Motorcycle _bike;
        private double _airTransitionFactor = 0.0;
        private double _wheelieTransitionFactor = 0.0;
        private Vector _prevVelocity = new(0, 0);
        private double _prevAngularVelocity = 0;

        public double Mass { get; private set; }
        public double EnginePower { get; private set; }
        public double BrakeForce { get; private set; }
        public double Gravity { get; private set; }
        public double DragCoefficient { get; private set; }
        public double WheelRadius { get; private set; } = DefaultWheelRadius;
        public double MaxLeanAngle { get; private set; }
        public double LeanSpeed { get; private set; }
        public double GroundFriction { get; private set; }
        public double SuspensionStrength { get; private set; }
        public double SuspensionDamping { get; private set; }
        public double SuspensionRestLength { get; private set; }
        public double MomentOfInertia { get; private set; }
        public double MinWheelDistance { get; private set; }
        public double MaxWheelDistance { get; private set; }
        public double NominalWheelBase { get; private set; }

        public BikePhysics(Motorcycle bike) : base(BikePhysicsTag)
        {
            _bike = bike;
            Gravity = DefaultGravity * GravityMultiplier;
        }

        public void InitializeProperties(BikeType bikeType)
        {
            var props = GetBikeProperties(bikeType);
            var wheelProps = GetWheelProperties(bikeType);

            Mass = props.mass;
            EnginePower = props.power;
            BrakeForce = props.brakeForce;
            DragCoefficient = props.drag;
            MaxLeanAngle = props.maxLeanAngle;
            LeanSpeed = props.leanSpeed;
            GroundFriction = props.friction * wheelProps.friction;
            SuspensionStrength = props.suspensionStrength * wheelProps.suspensionStrength;
            SuspensionDamping = props.suspensionDamping * wheelProps.suspensionDamping;
            SuspensionRestLength = props.suspensionRestLength;
            WheelRadius = wheelProps.radius;

            MomentOfInertia = Mass * Math.Pow(_bike.WheelBase / 2, 2) * 0.5;

            NominalWheelBase = _bike.WheelBase;
            MinWheelDistance = NominalWheelBase * WheelDistanceMinRatio;
            MaxWheelDistance = NominalWheelBase * WheelDistanceMaxRatio;
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

        public void UpdatePhysics(double deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            if (cancellationToken.IsCancellationRequested) return;

            SavePreviousState();
            UpdateTransitionFactors();
            HandleWheelCollisions(level, deltaTime);
            ApplyForces(deltaTime, level);
            UpdateLean(deltaTime, level);
            ApplyRotation(deltaTime);
            CheckFrameCollision(level, deltaTime);
            CheckCrashConditions();
            ValidatePhysicalState();
        }

        private void SavePreviousState()
        {
            _prevVelocity = _bike.Velocity;
            _prevAngularVelocity = _bike.AngularVelocity;
        }

        private void UpdateTransitionFactors()
        {
            _airTransitionFactor = _bike.IsInAir ? 1.0 : 0.0;
            _wheelieTransitionFactor = _bike.IsInWheelie ? 1.0 : 0.0;
        }

        private void HandleWheelCollisions(Level level, double deltaTime)
        {
            double frontGroundY = level.GetGroundYAtX(_bike.WheelPositions.Front.X);
            bool frontContact = HandleWheelCollision(true, frontGroundY, deltaTime);
            double rearGroundY = level.GetGroundYAtX(_bike.WheelPositions.Rear.X);
            bool rearContact = HandleWheelCollision(false, rearGroundY, deltaTime);

            if (!frontContact && !rearContact)
            {
                _bike.State |= BikeState.InAir;
            }
            else
            {
                _bike.State &= ~BikeState.InAir;
            }
        }

        public bool HandleWheelCollision(bool isFrontWheel, double groundY, double deltaTime)
        {
            Point wheelPosition = isFrontWheel ? _bike.WheelPositions.Front : _bike.WheelPositions.Rear;
            double penetration = wheelPosition.Y + WheelRadius - groundY;

            if (penetration <= 0)
            {
                SetSuspensionOffset(isFrontWheel, SuspensionRestLength);
                return false;
            }

            ProcessWheelPenetration(isFrontWheel, penetration, deltaTime);
            return true;
        }

        private void ProcessWheelPenetration(bool isFrontWheel, double penetration, double deltaTime)
        {
            penetration = Math.Min(penetration, WheelRadius * MaxWheelPenetration);

            double baseCompression = penetration * PenetrationBaseMultiplier;
            double progressiveFactor = 1.0 + Math.Pow(SafeDivide(penetration, WheelRadius * WheelRadiusHalfFactor), 2) * PenetrationProgressiveFactor;
            double desiredCompression = baseCompression * progressiveFactor;

            double currentOffset = isFrontWheel ? _bike.SuspensionOffsets.Front : _bike.SuspensionOffsets.Rear;
            double dampingFactor = SuspensionDampingFactor * deltaTime;
            double newOffset = Lerp(currentOffset, SuspensionRestLength - desiredCompression, dampingFactor);

            newOffset = ClampValue(newOffset, SuspensionRestLength * MinSuspensionCompression, SuspensionRestLength);
            Point attachmentPoint = isFrontWheel ? _bike.AttachmentPoints.Front : _bike.AttachmentPoints.Rear;
            double compressionRatio = 1.0 - SafeDivide(newOffset, SuspensionRestLength);

            double progressiveFactorForce = 1.0 + Math.Pow(compressionRatio, 2) * SuspensionProgressiveFactor;
            double adjustedSuspensionStrength = SuspensionStrength * progressiveFactorForce;
            double normalForce = adjustedSuspensionStrength * penetration;
            double frictionCoefficient = GroundFriction * FrictionMultiplier;

            Vector reactionForce = new Vector(
                -_bike.Velocity.X * frictionCoefficient,
                -adjustedSuspensionStrength * penetration - SuspensionDamping * _bike.Velocity.Y
            );

            Vector r = attachmentPoint - _bike.Position;
            double torque = r.X * reactionForce.Y - r.Y * reactionForce.X;

            _bike.Velocity += reactionForce / Mass * deltaTime;
            _bike.AngularVelocity += torque / MomentOfInertia * deltaTime;

            SetSuspensionOffset(isFrontWheel, newOffset);
        }

        private void SetSuspensionOffset(bool isFrontWheel, double offset)
        {
            var current = _bike.SuspensionOffsets;
            _bike.SuspensionOffsets = isFrontWheel ? (offset, current.Rear) : (current.Front, offset);
        }

        private void UpdateLean(double deltaTime, Level level)
        {
            if (_airTransitionFactor < 0.5)
                UpdateGroundLean(deltaTime, level);
            else
                UpdateAirLean(deltaTime);
        }

        private void UpdateAirLean(double deltaTime)
        {
            double maxAirAngularVelocity = Math.PI;
            double desiredAngularVelocity = _bike.LeanAmount * maxAirAngularVelocity;

            double damping = 0.5;

            _bike.AngularVelocity += (desiredAngularVelocity - _bike.AngularVelocity) * damping * deltaTime;
        }

        private void UpdateGroundLean(double deltaTime, Level level)
        {
            double slopeAngle = 0;
            if (!_bike.IsInAir)
            {
                slopeAngle = level.CalculateSlopeAngle(_bike.Position.X);
            }

            double throttleEffect = _bike.Throttle * 0.2;
            double targetLean = _bike.LeanAmount * MaxLeanAngle + slopeAngle + throttleEffect;
            double leanError = targetLean - _bike.Angle;
            double leanErrorDerivative = -_bike.AngularVelocity;

            double controlTorque = 1000.0 * leanError * LeanSpeed + 200.0 * leanErrorDerivative;

            _bike.AngularVelocity += controlTorque / MomentOfInertia * deltaTime;

            double maxAngularVel = MaxAngularVelocity * GroundAngularVelocityFactor;
            _bike.AngularVelocity = ClampValue(_bike.AngularVelocity, -maxAngularVel, maxAngularVel);
        }

        private void ApplyRotation(double deltaTime)
        {
            _bike.Angle += _bike.AngularVelocity * deltaTime;
            _bike.Angle = NormalizeAngle(_bike.Angle);
        }

        private void ApplyForces(double deltaTime, Level level)
        {
            Vector totalForce = CalculateTotalForce(level);
            double torque = 0;

            if (_bike.Throttle > 0)
            {
                double engineForce = _bike.Throttle * EnginePower;
                Vector thrustForce = new Vector(Math.Cos(_bike.Angle) * engineForce, Math.Sin(_bike.Angle) * engineForce);
                Vector r_rear = _bike.WheelPositions.Rear - _bike.Position;
                torque += r_rear.X * thrustForce.Y - r_rear.Y * thrustForce.X;
            }

            _bike.Velocity += totalForce / Mass * deltaTime;
            _bike.AngularVelocity += torque / MomentOfInertia * deltaTime;
        }

        private Vector CalculateTotalForce(Level level)
        {
            Vector gravityForce = new Vector(0, Gravity * Mass);
            Vector thrustForce = CalculateThrustForce();
            Vector brakeForce = CalculateBrakeForce();
            Vector dragForce = CalculateDragForce();
            Vector slopeForce = CalculateSlopeForce(level);

            return gravityForce + thrustForce + brakeForce + dragForce + slopeForce;
        }

        private Vector CalculateThrustForce()
        {
            double engineForce = _bike.Throttle * EnginePower;
            var (cosAngle, sinAngle) = GetTrigsFromAngle(_bike.Angle);
            return new Vector(cosAngle * engineForce, sinAngle * engineForce);
        }

        private Vector CalculateBrakeForce()
        {
            if (_bike.Brake <= MinBrakeInput || _bike.Velocity.Length <= 0)
                return new Vector(0, 0);

            double effectiveBrakeForce = _bike.Brake * BrakeForce;
            Vector brakeVector = -_bike.Velocity;
            return brakeVector * (SafeDivide(effectiveBrakeForce, brakeVector.Length));
        }

        private Vector CalculateDragForce()
        {
            double speed = _bike.Velocity.Length;
            if (speed <= 0)
                return new Vector(0, 0);

            double currentDrag = DragCoefficient * (_airTransitionFactor > 0.5 ? AirFrictionMultiplier : 1.0);
            if (!_bike.IsInAir && _bike.Throttle == 0 && _bike.Brake == 0)
                currentDrag *= 2.0;

            return -_bike.Velocity * (currentDrag * speed);
        }

        private Vector CalculateSlopeForce(Level level)
        {
            if (_bike.IsInAir)
                return new Vector(0, 0);

            double slopeAngle = level.CalculateSlopeAngle(_bike.Position.X);
            double slopeForceMagnitude = Mass * Gravity * Math.Sin(slopeAngle);

            return new Vector(slopeForceMagnitude * Math.Cos(slopeAngle), -slopeForceMagnitude * Math.Sin(slopeAngle));
        }

        public void CheckFrameCollision(Level level, double deltaTime)
        {
            if (_bike.IsInAir || _bike.IsCrashed || _bike.Velocity.Length <= FrameCollisionMinVelocity)
                return;

            var collisionInfo = DetectFrameCollision(level);
            if (collisionInfo.IsCollision && collisionInfo.MaxPenetration > WheelRadius * FrameCollisionMinPenetration)
                HandleFrameCollision(collisionInfo, deltaTime);
        }

        private (bool IsCollision, Point CollisionPoint, double MaxPenetration) DetectFrameCollision(Level level)
        {
            var framePoints = _bike.GetFramePoints();
            bool frameCollision = false;
            Point collisionPoint = default;
            double maxPenetration = 0;

            foreach (var point in framePoints)
            {
                double groundY = level.GetGroundYAtX(point.X);
                double penetration = point.Y - groundY;
                if (penetration > 0)
                {
                    frameCollision = true;
                    if (penetration > maxPenetration)
                    {
                        maxPenetration = penetration;
                        collisionPoint = point;
                    }
                }
            }

            return (frameCollision, collisionPoint, maxPenetration);
        }

        private void HandleFrameCollision((bool IsCollision, Point CollisionPoint, double MaxPenetration) collisionInfo, double deltaTime)
        {
            double crashThreshold = WheelRadius * FrameCrashThreshold;
            bool isHighSpeed = _bike.Velocity.Length > 200;
            bool isBadAngle = Math.Abs(_bike.Angle) > FrameCriticalBackwardTiltAngle;

            if (collisionInfo.MaxPenetration > crashThreshold && (isHighSpeed || isBadAngle))
            {
                _bike.State |= BikeState.Crashed;
                Logger.Warning(BikePhysicsTag,
                    $"Frame collision detected at {collisionInfo.CollisionPoint}, " +
                    $"penetration: {collisionInfo.MaxPenetration:F1}, " +
                    $"speed: {_bike.Velocity.Length:F1}, " +
                    $"angle: {_bike.Angle * 180 / Math.PI:F1}°");
            }
            else
            {
                ApplyFrameCollisionResponse(collisionInfo.CollisionPoint, collisionInfo.MaxPenetration, deltaTime);
            }
        }

        private void ApplyFrameCollisionResponse(Point collisionPoint, double penetration, double deltaTime)
        {
            double reactionForce = penetration * SuspensionStrength * FrameCollisionReactionForce;
            double impulse = reactionForce * deltaTime;
            double deltaVelocityY = -impulse / Mass;

            deltaVelocityY = Math.Max(deltaVelocityY, -FrameCollisionMaxDeltaVelocity * deltaTime);
            _bike.Velocity = new Vector(_bike.Velocity.X, _bike.Velocity.Y + deltaVelocityY);

            double stabilizingFactor = Math.Abs(_bike.Angle) > FrameStabilizingAngleThreshold ? FrameStabilizingFactorStrong : FrameStabilizingFactorBase;
            double stabilizingTorque = -_bike.Angle * stabilizingFactor;
            double deltaAngularVelocity = stabilizingTorque * deltaTime / MomentOfInertia;

            deltaAngularVelocity = ClampValue(deltaAngularVelocity, -FrameCollisionMaxDeltaAngular * deltaTime, FrameCollisionMaxDeltaAngular * deltaTime);
            _bike.AngularVelocity += deltaAngularVelocity;
        }

        private void CheckCrashConditions()
        {
            if (_bike.IsInAir || _bike.IsCrashed)
                return;

            double absAngle = Math.Abs(_bike.Angle);
            if (absAngle > CriticalLeanAngle)
            {
                _bike.State |= BikeState.Crashed;
                TryLog(LogLevel.W, $"Bike crashed due to excessive tilt: {_bike.Angle * 180 / Math.PI:F1}°");
                return;
            }

            double currentDistance = CalculateWheelDistance();
            if (currentDistance < MinWheelDistance || currentDistance > MaxWheelDistance)
            {
                _bike.State |= BikeState.Crashed;
                double deformation = Math.Abs(currentDistance - NominalWheelBase) / NominalWheelBase * 100;
                TryLog(LogLevel.W, $"Bike crashed due to frame deformation: {deformation:F1}% (distance: {currentDistance:F1})");
            }
        }

        public double CalculateWheelDistance()
        {
            return CalculateDistance(_bike.WheelPositions.Front, _bike.WheelPositions.Rear);
        }

        private void ValidatePhysicalState()
        {
            _diagnostics.Clear();

            _bike.Angle = SanitizeValue(_bike.Angle, 0, "Invalid angle value");
            _bike.AngularVelocity = SanitizeValue(_bike.AngularVelocity, 0, "Invalid angular velocity");
            _bike.Velocity = SanitizeVector(_bike.Velocity, new Vector(0, 0), "Invalid velocity");

            ValidateComponents(new[]
            {
                ("Angle", _bike.Angle, -Math.PI, Math.PI),
                ("AngularVelocity", _bike.AngularVelocity, -MaxAngularVelocity, MaxAngularVelocity),
                ("RearWheelY", _bike.WheelPositions.Rear.Y, -MaxSafeHeight, MaxSafeHeight),
                ("FrontWheelY", _bike.WheelPositions.Front.Y, -MaxSafeHeight, MaxSafeHeight),
                ("FrontSuspensionOffset", _bike.SuspensionOffsets.Front, SuspensionRestLength * MinSuspensionCompression, SuspensionRestLength),
                ("RearSuspensionOffset", _bike.SuspensionOffsets.Rear, SuspensionRestLength * MinSuspensionCompression, SuspensionRestLength)
            }, "Bike");

            ValidateVectorParameter("Velocity", _bike.Velocity, MaxSafeVelocity, "Bike");

            double wheelDistance = CalculateWheelDistance();
            ValidateConnectionStrain("WheelBase", wheelDistance, NominalWheelBase, WheelDistanceMinRatio, WheelDistanceMaxRatio, "Frame");

            double maxForce = Mass * MaxSafeAcceleration;
            double engineForce = _bike.Throttle * EnginePower;
            double brakeForce = _bike.Brake * BrakeForce;

            ValidatePhysicalParameter("EngineForce", engineForce, 0, maxForce, "Forces");
            ValidatePhysicalParameter("BrakeForce", brakeForce, 0, maxForce, "Forces");

            if (_diagnostics.Count > 0 && _logThrottleTime <= 0)
            {
                Logger.Debug(BikePhysicsTag, GenerateDiagnosticReport());
            }
        }
    }
}