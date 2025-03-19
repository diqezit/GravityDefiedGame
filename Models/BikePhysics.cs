using System;
using System.Windows;
using System.Threading;
using GravityDefiedGame.Utilities;
using static GravityDefiedGame.Utilities.GameConstants.Physics;
using static GravityDefiedGame.Utilities.GameConstants.Validation;
using static GravityDefiedGame.Utilities.GameConstants.Debug;
using static GravityDefiedGame.Utilities.GameConstants;

namespace GravityDefiedGame.Models;

public class BikePhysics : Component
{
    #region Properties and Fields
    private readonly Motorcycle _bike;
    internal readonly BikeConfig _config;
    internal readonly LeanController _leanController;
    private readonly ForceController _forceController;
    private readonly SuspensionSystem _suspensionSystem;
    private readonly CollisionHandler _collisionHandler;
    private readonly CrashDetector _crashDetector;

    public double Mass => _config.Mass;
    public double EnginePower => _config.EnginePower;
    public double BrakeForce => _config.BrakeForce;
    public double WheelRadius => _config.WheelRadius;
    public double SuspensionRestLength => _config.SuspensionRestLength;
    public double GroundFriction => _config.GroundFriction;
    #endregion

    public BikePhysics(Motorcycle bike) : base(BikePhysicsTag)
    {
        _bike = bike;
        _config = new BikeConfig(this, _bike);
        _leanController = new LeanController(this, _bike, _config);
        _forceController = new ForceController(this, _bike, _config);
        _suspensionSystem = new SuspensionSystem(this, _bike, _config);
        _collisionHandler = new CollisionHandler(this, _bike, _config);
        _crashDetector = new CrashDetector(this, _bike, _config);
    }

    public void InitializeProperties(BikeType bikeType)
    {
        _config.Initialize(bikeType);
    }

    public void UpdatePhysics(double deltaTime, CancellationToken cancellationToken = default)
    {
        if (cancellationToken.IsCancellationRequested) return;

        _leanController.SavePreviousState();
        _leanController.UpdateTransitionFactors(deltaTime);
        _leanController.UpdateLeanControl(deltaTime);
        _leanController.UpdateLean(deltaTime);
        _forceController.ApplyForces(deltaTime);
        _crashDetector.CheckCrashConditions();
        ValidatePhysicalState();
    }

    public void HandleWheelCollision(bool isFrontWheel, double groundY, double deltaTime)
    {
        _suspensionSystem.HandleWheelCollision(isFrontWheel, groundY, deltaTime);
    }

    public void CheckFrameCollision(Level level, double deltaTime)
    {
        _collisionHandler.CheckFrameCollision(level, deltaTime);
    }

    private void ValidatePhysicalState()
    {
        _bike.Angle = SanitizeValue(_bike.Angle, 0, "Invalid angle value");
        _bike.AngularVelocity = SanitizeValue(_bike.AngularVelocity, 0, "Invalid angular velocity");
        _bike.Velocity = SanitizeVector(_bike.Velocity, new Vector(0, 0), "Invalid velocity");
    }
}