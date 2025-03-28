#nullable enable

using GravityDefiedGame.Utilities;
using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Threading;
using GravityDefiedGame.Views;
using static GravityDefiedGame.Utilities.GameConstants.Motorcycle;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;
using static GravityDefiedGame.Utilities.LoggerCore;
using static GravityDefiedGame.Utilities.Logger;
using static GravityDefiedGame.Models.BikeGeom;

namespace GravityDefiedGame.Models
{
    public interface IBikePhysicsData
    {
        Vector2 Position { get; }
        Vector2 Velocity { get; }
        float Angle { get; }
        float AngularVelocity { get; }
        float WheelBase { get; }
        float FrameHeight { get; }
        BikeType BikeType { get; }
        (Vector2 Front, Vector2 Rear) WheelPositions { get; }
        (Vector2 Front, Vector2 Rear) AttachmentPoints { get; }
        (float Front, float Rear) SuspensionOffsets { get; }
        (float Front, float Rear) WheelRotations { get; }
        float Throttle { get; }
        float Brake { get; }
        float LeanAmount { get; }
        int Direction { get; }
        BikeState State { get; }
        bool IsCrashed { get; }
        bool IsInAir { get; }
        bool WasInAir { get; }
        bool IsInWheelie { get; }
        bool IsInStoppie { get; }
        bool IsMovingBackward { get; }
        float WheelieTime { get; }
        float StoppieTime { get; }
        float WheelieIntensity { get; }
        float StoppieIntensity { get; }
        float GetWheelRadius();
        Vector2 GetVisualCenter();
        List<Vector2> GetFramePoints();
    }

    public interface IBikeVisualData
    {
        Color BikeColor { get; }
        BikeType BikeType { get; }
        void SetBikeColor(Color color);
        void UpdateFromTheme();
        (List<SkeletonPoint> Points, List<SkeletonLine> Lines) GetSkeleton();
    }

    public class Motorcycle : PhysicsComponent, IBikePhysicsData, IBikeVisualData
    {
        #region Private Fields
        private BikePhysics _physics;
        private int _direction = 1;
        private Color _bikeColor;
        #endregion

        #region Properties - Основные физические свойства
        public Vector2 Position { get; internal set; }
        public Vector2 Velocity { get; internal set; }
        public float Angle { get; internal set; }
        public float AngularVelocity { get; internal set; }
        public float WheelBase { get; internal set; } = DefaultWheelBase;
        public float FrameHeight { get; private set; }
        public BikeType BikeType { get; private set; }
        #endregion

        #region Properties - Колеса и подвеска
        public Vector2 FrontWheelPosition { get; internal set; }
        public Vector2 RearWheelPosition { get; internal set; }
        public (Vector2 Front, Vector2 Rear) WheelPositions
        {
            get => (FrontWheelPosition, RearWheelPosition);
            internal set => (FrontWheelPosition, RearWheelPosition) = value;
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

        public float FrontWheelRotation { get; internal set; }
        public float RearWheelRotation { get; internal set; }
        public (float Front, float Rear) WheelRotations
        {
            get => (FrontWheelRotation, RearWheelRotation);
            internal set => (FrontWheelRotation, RearWheelRotation) = value;
        }

        public float FrontWheelAngularVelocity { get; set; }
        public float RearWheelAngularVelocity { get; set; }
        #endregion

        #region Properties - Состояние управления
        public float Throttle { get; internal set; }
        public float Brake { get; internal set; }
        public float LeanAmount { get; internal set; }
        public float SuspensionRealMaxAngle { get; internal set; }
        public bool EnforceSuspensionAngleLimits { get; set; } = true;
        public int Direction
        {
            get => _direction;
            private set => _direction = value;
        }
        #endregion

        #region Properties - Визуальные аспекты
        public Color BikeColor => _bikeColor;
        #endregion

        #region Properties - Состояние мотоцикла
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

        public float WheelieIntensity => IsInWheelie ? MathHelper.Min(1.0f, WheelieTime / 1.0f) : 0;
        public float StoppieIntensity => IsInStoppie ? MathHelper.Min(1.0f, StoppieTime / 0.5f) : 0;
        #endregion

        #region Конструктор
        public Motorcycle(BikeType bikeType = BikeType.Standard)
        {
            Log("Motorcycle", "initializing motorcycle", () =>
            {
                BikeType = bikeType;
                _bikeColor = new Color(200, 200, 200);
                _physics = new BikePhysics(this);
                InitializeBikeProperties();
                Reset();
                Info("Motorcycle", $"Created {bikeType} motorcycle");
            });
        }
        #endregion

        #region Приватные методы
        private void InitializeBikeProperties()
        {
            Log("Motorcycle", "initializing bike properties", () =>
            {
                SuspensionRealMaxAngle = _physics.MaxSuspensionAngle;
                _physics.InitializeProperties(BikeType);
                FrameHeight = _physics.WheelRadius * 1.5f;
                Info("Motorcycle", $"Initialized {BikeType} motorcycle with wheel radius {_physics.WheelRadius:F2}");
            });
        }
        #endregion

        #region Публичные методы - Основные операции
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

        public void Update(float deltaTime, Level level, CancellationToken cancellationToken = default)
        {
            Log("Motorcycle", "updating motorcycle", () =>
            {
                _physics.Update(deltaTime, level, cancellationToken);
            });
        }
        #endregion

        #region Публичные методы - Конфигурация
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
                _bikeColor = color;
                Debug("Motorcycle", $"Bike color set to {color}");
            });
        }

        public void UpdateFromTheme()
        {
            Log("Motorcycle", "updating from theme", () =>
            {
                try
                {
                    _bikeColor = ThemeManager.CurrentTheme.BikeColors[SkeletonLineType.MainFrame];
                    Debug("Motorcycle", $"Bike color updated from theme to {_bikeColor}");
                }
                catch (Exception ex)
                {
                    Error("Motorcycle", $"Failed to update from theme: {ex.Message}");
                }
            });
        }
        #endregion

        #region Публичные методы - Управление
        public void ApplyThrottle(float amount)
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
                if (direction is not (1 or -1))
                    throw new ArgumentException("Direction must be 1 or -1", nameof(direction));

                Direction = direction;
                Debug("Motorcycle", $"Direction set to {direction}");
            });
        }
        #endregion

        #region Публичные методы - Получение данных
        public float GetWheelRadius() => _physics.WheelRadius;

        public Vector2 GetVisualCenter() =>
            Log("Motorcycle", "Calculating visual center", () =>
                (WheelPositions.Front + WheelPositions.Rear) / 2f,
            Position);

        public List<Vector2> GetFramePoints() => _physics.GetFramePoints();

        public (List<SkeletonPoint> Points, List<SkeletonLine> Lines) GetSkeleton()
        {
            var geom = new BikeGeom(this);
            return geom.GetSkeleton();
        }
        #endregion
    }

    public class BikeComponentFactory
    {
        public static IBikePhysicsData CreatePhysicsData(Motorcycle bike) => bike;

        public static IBikeVisualData CreateVisualData(Motorcycle bike) => bike;

        public static BikeGeom CreateGeometryComponent(IBikePhysicsData physicsData)
            => new BikeGeom(physicsData);
    }
}