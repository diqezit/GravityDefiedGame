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

        public double WheelieIntensity => IsInWheelie ? Math.Min(1.0, WheelieTime / 1.0) : 0;
        public double StoppieIntensity => IsInStoppie ? Math.Min(1.0, StoppieTime / 0.5) : 0;

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
            BikeType = bikeType;
            InitializeBikeProperties();
            Reset();
        }

        private void InitializeBikeProperties()
        {
            SuspensionRealMaxAngle = _physics.MaxSuspensionAngle;
            _physics.InitializeProperties(BikeType);
            BikeColor = DefaultBikeColor;
            FrameHeight = _physics.WheelRadius * 1.5;
            WriteLog(LogLevel.I, "Motorcycle", $"Initialized {BikeType} motorcycle");
        }

        // Основные методы (делегирующие вызовы к BikePhysics)
        public void Reset() => _physics.Reset();

        public void SetPosition(Point position) => _physics.SetPosition(position);

        public void SetBikeType(BikeType bikeType)
        {
            BikeType = bikeType;
            _physics.SetBikeType(bikeType);
            WriteLog(LogLevel.I, "Motorcycle", $"Bike type changed to {bikeType}");
        }

        public void SetBikeColor(Color color) => BikeColor = color;

        public void Update(double deltaTime, Level level, CancellationToken cancellationToken = default) =>
            _physics.Update(deltaTime, level, cancellationToken);

        // Методы управления
        public void ApplyThrottle(double amount) => _physics.ApplyThrottle(amount);
        public void ApplyBrake(double amount) => _physics.ApplyBrake(amount);
        public void Lean(double direction) => _physics.Lean(direction);

        // Геометрия и визуализация
        public double GetWheelRadius() => _physics.WheelRadius;
        public List<Point> GetFramePoints() => _physics.GetFramePoints();
        public (List<BikeGeom.SkeletonPoint> Points, List<BikeGeom.SkeletonLine> Lines) GetSkeleton() =>
            _physics.GetSkeleton();
    }
}