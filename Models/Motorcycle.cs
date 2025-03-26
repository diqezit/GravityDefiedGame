#nullable enable

using GravityDefiedGame.Utilities;
using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Threading;
using static GravityDefiedGame.Utilities.GameConstants.Motorcycle;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;
using static GravityDefiedGame.Utilities.LoggerCore;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame.Models
{
    public class Motorcycle : PhysicsComponent
    {
        private readonly BikePhysics _physics;

        // Свойства состояния мотоцикла
        public float SuspensionRealMaxAngle { get; internal set; }
        public bool EnforceSuspensionAngleLimits { get; set; } = true;
        public Vector2 Position { get; internal set; }
        public Vector2 Velocity { get; internal set; }
        public float Angle { get; internal set; }
        public float AngularVelocity { get; internal set; }
        public float FrameHeight { get; private set; }

        public Vector2 FrontWheelPosition { get; internal set; }
        public Vector2 RearWheelPosition { get; internal set; }
        public (Vector2 Front, Vector2 Rear) WheelPositions
        {
            get => (FrontWheelPosition, RearWheelPosition);
            internal set => (FrontWheelPosition, RearWheelPosition) = value;
        }

        public float WheelBase { get; internal set; } = DefaultWheelBase;
        public float FrontWheelRotation { get; internal set; }
        public float RearWheelRotation { get; internal set; }
        public (float Front, float Rear) WheelRotations
        {
            get => (FrontWheelRotation, RearWheelRotation);
            internal set => (FrontWheelRotation, RearWheelRotation) = value;
        }

        public float Throttle { get; internal set; }
        public float Brake { get; internal set; }
        public float LeanAmount { get; internal set; }
        public BikeType BikeType { get; private set; }
        public Color BikeColor { get; private set; }
        public BikeState State { get; internal set; }

        public bool IsCrashed
        {
            get => (State & BikeState.Crashed) == BikeState.Crashed;
            internal set => State = value ? State | BikeState.Crashed : State & ~BikeState.Crashed;
        }

        public bool IsInAir
        {
            get => (State & BikeState.InAir) == BikeState.InAir;
            internal set => State = value ? State | BikeState.InAir : State & ~BikeState.InAir;
        }

        public bool WasInAir { get; internal set; }

        public bool IsInWheelie
        {
            get => (State & BikeState.InWheelie) == BikeState.InWheelie;
            internal set => State = value ? State | BikeState.InWheelie : State & ~BikeState.InWheelie;
        }

        public bool IsInStoppie
        {
            get => (State & BikeState.InStoppie) == BikeState.InStoppie;
            internal set => State = value ? State | BikeState.InStoppie : State & ~BikeState.InStoppie;
        }

        public bool IsMovingBackward
        {
            get => (State & BikeState.MovingBackward) == BikeState.MovingBackward;
            internal set => State = value ? State | BikeState.MovingBackward : State & ~BikeState.MovingBackward;
        }

        public float WheelieTime { get; internal set; }
        public float StoppieTime { get; internal set; }

        public float WheelieIntensity
        {
            get => IsInWheelie ? MathHelper.Min(1.0f, WheelieTime / 1.0f) : 0;
        }

        public float StoppieIntensity
        {
            get => IsInStoppie ? MathHelper.Min(1.0f, StoppieTime / 0.5f) : 0;
        }

        public Vector2 FrontAttachmentPoint { get; set; }
        public Vector2 RearAttachmentPoint { get; set; }
        public (Vector2 Front, Vector2 Rear) AttachmentPoints
        {
            get => (FrontAttachmentPoint, RearAttachmentPoint);
            internal set => (FrontAttachmentPoint, RearAttachmentPoint) = value;
        }

        public float FrontSuspensionOffset { get; set; }
        public float RearSuspensionOffset { get; set; }
        public (float Front, float Rear) SuspensionOffsets
        {
            get => (FrontSuspensionOffset, RearSuspensionOffset);
            internal set => (FrontSuspensionOffset, RearSuspensionOffset) = value;
        }

        public int Direction { get; private set; } = 1;

        public Motorcycle(BikeType bikeType = BikeType.Standard)
        {
            _physics = new BikePhysics(this);

            Log("Motorcycle", "initializing motorcycle", () =>
            {
                BikeType = bikeType;
                InitializeBikeProperties();
                Reset();
                Info("Motorcycle", $"Created {bikeType} motorcycle");
            });
        }

        private void InitializeBikeProperties()
        {
            Log("Motorcycle", "initializing bike properties", () =>
            {
                SuspensionRealMaxAngle = _physics.MaxSuspensionAngle;
                _physics.InitializeProperties(BikeType);
                BikeColor = new Color(200, 200, 200); // Серый цвет по умолчанию
                FrameHeight = _physics.WheelRadius * 1.5f;
                Info("Motorcycle", $"Initialized {BikeType} motorcycle with wheel radius {_physics.WheelRadius:F2}");
            });
        }

        public void Reset()
        {
            Log("Motorcycle", "resetting motorcycle", () =>
            {
                _physics.Reset();
                Direction = 1; 
                Debug("Motorcycle", "Motorcycle reset to initial state");
            });
        }

        public void SetPosition(Vector2 position)
        {
            Log("Motorcycle", "setting position", () =>
            {
                _physics.SetPosition(position);
                Debug("Motorcycle", $"Position set to {position}");
            });
        }

        public void SetBikeType(BikeType bikeType)
        {
            Log("Motorcycle", $"changing bike type to {bikeType}", () =>
            {
                BikeType = bikeType;
                _physics.SetBikeType(bikeType);
                Info("Motorcycle", $"Bike type changed to {bikeType}");
            });
        }

        public void SetBikeColor(Color color)
        {
            Log("Motorcycle", "setting bike color", () =>
            {
                BikeColor = color;
                Debug("Motorcycle", $"Bike color set to {color}");
            });
        }

        public void Update(float deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            Log("Motorcycle", "updating motorcycle", () =>
            {
                _physics.Update(deltaTime, level, cancellationToken);
            });
        }

        public void ApplyThrottle(float amount)
        {
            Log("Motorcycle", $"applying throttle: {amount:F2}", () =>
            {
                Throttle = MathHelper.Clamp(amount, 0f, 1f);
                if (Direction == -1)
                {
                    IsMovingBackward = true;
                    _physics.ApplyThrottle(amount * Direction); 
                }
                else
                {
                    IsMovingBackward = false;
                    _physics.ApplyThrottle(amount); 
                }
            });
        }

        public void ApplyBrake(float amount)
        {
            Log("Motorcycle", $"applying brake: {amount:F2}", () =>
            {
                Brake = MathHelper.Clamp(amount, 0f, 1f);
                _physics.ApplyBrake(amount); 
            });
        }

        public void Lean(float direction)
        {
            Log("Motorcycle", $"leaning: {direction:F2}", () =>
            {
                _physics.Lean(direction);
            });
        }

        public void SetDirection(int direction)
        {
            Log("Motorcycle", $"setting direction: {direction}", () =>
            {
                if (direction != 1 && direction != -1)
                    throw new ArgumentException("Direction must be 1 or -1", nameof(direction));
                Direction = direction;
                Debug("Motorcycle", $"Direction set to {direction}");
            });
        }

        public float GetWheelRadius()
        {
            return _physics.WheelRadius;
        }

        public Vector2 GetVisualCenter()
        {
            Log("Motorcycle", "Calculating visual center", () =>
            {
                Vector2 center = (WheelPositions.Front + WheelPositions.Rear) / 2f;
                return center;
            }, Position);
            return (WheelPositions.Front + WheelPositions.Rear) / 2f;
        }

        public List<Vector2> GetFramePoints()
        {
            return _physics.GetFramePoints();
        }

        public (List<BikeGeom.SkeletonPoint> Points, List<BikeGeom.SkeletonLine> Lines) GetSkeleton()
        {
            return _physics.GetSkeleton();
        }
    }
}