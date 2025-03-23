using GravityDefiedGame.Utilities;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Windows;
using System.Windows.Media;
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
        public double SuspensionRealMaxAngle { get; internal set; }
        public bool EnforceSuspensionAngleLimits { get; set; } = true;
        public Point Position { get; internal set; }
        public Vector Velocity { get; internal set; }
        public double Angle { get; internal set; }
        public double AngularVelocity { get; internal set; }
        public double FrameHeight { get; private set; }

        public Point FrontWheelPosition { get; internal set; }
        public Point RearWheelPosition { get; internal set; }
        public (Point Front, Point Rear) WheelPositions
        {
            get => (FrontWheelPosition, RearWheelPosition);
            internal set => (FrontWheelPosition, RearWheelPosition) = value;
        }

        public double WheelBase { get; internal set; } = DefaultWheelBase;
        public double FrontWheelRotation { get; internal set; }
        public double RearWheelRotation { get; internal set; }
        public (double Front, double Rear) WheelRotations
        {
            get => (FrontWheelRotation, RearWheelRotation);
            internal set => (FrontWheelRotation, RearWheelRotation) = value;
        }

        public double Throttle { get; internal set; }
        public double Brake { get; internal set; }
        public double LeanAmount { get; internal set; }
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

        public double WheelieTime { get; internal set; }
        public double StoppieTime { get; internal set; }

        public double WheelieIntensity =>
            Log("Motorcycle", "calculating wheelie intensity", () =>
                IsInWheelie ? Math.Min(1.0, WheelieTime / 1.0) : 0, 0.0);

        public double StoppieIntensity =>
            Log("Motorcycle", "calculating stoppie intensity", () =>
                IsInStoppie ? Math.Min(1.0, StoppieTime / 0.5) : 0, 0.0);

        public Point FrontAttachmentPoint { get; set; }
        public Point RearAttachmentPoint { get; set; }
        public (Point Front, Point Rear) AttachmentPoints
        {
            get => (FrontAttachmentPoint, RearAttachmentPoint);
            internal set => (FrontAttachmentPoint, RearAttachmentPoint) = value;
        }

        public double FrontSuspensionOffset { get; set; }
        public double RearSuspensionOffset { get; set; }
        public (double Front, double Rear) SuspensionOffsets
        {
            get => (FrontSuspensionOffset, RearSuspensionOffset);
            internal set => (FrontSuspensionOffset, RearSuspensionOffset) = value;
        }

        public Motorcycle(BikeType bikeType = BikeType.Standard)
        {
            _physics = new BikePhysics(this);

            Log("Motorcycle", "initializing motorcycle", () =>
            {
                BikeType = bikeType;
                InitializeBikeProperties();
                Reset();
#if DEBUG
                Info("Motorcycle", $"Created {bikeType} motorcycle");
#endif
            });
        }

        private void InitializeBikeProperties()
        {
            Log("Motorcycle", "initializing bike properties", () =>
            {
                SuspensionRealMaxAngle = _physics.MaxSuspensionAngle;
                _physics.InitializeProperties(BikeType);
                BikeColor = DefaultBikeColor;
                FrameHeight = _physics.WheelRadius * 1.5;
#if DEBUG
                Info("Motorcycle", $"Initialized {BikeType} motorcycle with wheel radius {_physics.WheelRadius:F2}");
#endif
            });
        }

        // Основные методы (делегирующие вызовы к BikePhysics)
        public void Reset()
        {
            Log("Motorcycle", "resetting motorcycle", () =>
            {
                _physics.Reset();
#if DEBUG
                Debug("Motorcycle", "Motorcycle reset to initial state");
#endif
            });
        }

        public void SetPosition(Point position)
        {
            Log("Motorcycle", "setting position", () =>
            {
                _physics.SetPosition(position);
#if DEBUG
                Debug("Motorcycle", $"Position set to {position}");
#endif
            });
        }

        public void SetBikeType(BikeType bikeType)
        {
            Log("Motorcycle", $"changing bike type to {bikeType}", () =>
            {
                BikeType = bikeType;
                _physics.SetBikeType(bikeType);
#if DEBUG
                Info("Motorcycle", $"Bike type changed to {bikeType}");
#endif
            });
        }

        public void SetBikeColor(Color color)
        {
            Log("Motorcycle", "setting bike color", () =>
            {
                BikeColor = color;
#if DEBUG
                Debug("Motorcycle", $"Bike color set to {color}");
#endif
            });
        }

        public void Update(double deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            Log("Motorcycle", "updating motorcycle", () =>
            {
                _physics.Update(deltaTime, level, cancellationToken);
            });
        }

        // Методы управления
        public void ApplyThrottle(double amount)
        {
            Log("Motorcycle", $"applying throttle: {amount:F2}", () =>
            {
                _physics.ApplyThrottle(amount);
            });
        }

        public void ApplyBrake(double amount)
        {
            Log("Motorcycle", $"applying brake: {amount:F2}", () =>
            {
                _physics.ApplyBrake(amount);
            });
        }

        public void Lean(double direction)
        {
            Log("Motorcycle", $"leaning: {direction:F2}", () =>
            {
                _physics.Lean(direction);
            });
        }

        // Геометрия и визуализация
        public double GetWheelRadius() =>
            Log("Motorcycle", "getting wheel radius", () => _physics.WheelRadius, DefaultWheelRadius);

        public List<Point> GetFramePoints() =>
            Log("Motorcycle", "getting frame points", () => _physics.GetFramePoints(), new List<Point>());

        public (List<BikeGeom.SkeletonPoint> Points, List<BikeGeom.SkeletonLine> Lines) GetSkeleton() =>
            Log("Motorcycle", "getting skeleton", () => _physics.GetSkeleton(),
                (new List<BikeGeom.SkeletonPoint>(), new List<BikeGeom.SkeletonLine>()));
    }
}