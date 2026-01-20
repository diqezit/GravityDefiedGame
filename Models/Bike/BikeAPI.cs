using Vector2 = Microsoft.Xna.Framework.Vector2;

namespace GravityDefiedGame.Models.Bike;

public sealed class BikeAPI(BikeType type) : IDisposable
{
    readonly BikePhysics _p = new(type);
    bool _disposed;

    public BikeCfg Cfg => _p.Cfg;
    public BikeType Type => _p.Type;
    public Vector2 Pos => _p.Pos;
    public Vector2 Vel => _p.Vel;
    public float Ang => _p.Ang;
    public float AngVel => _p.AngVel;
    public float Spd => _p.Speed;
    public BikeState State => _p.State;
    public bool IsCrashed => _p.Crashed;
    public bool IsInAir => _p.InAir;
    public bool IsGrounded => _p.IsGrounded;
    public bool FwGnd => _p.FwGnd;
    public bool RwGnd => _p.RwGnd;
    public float FrontSusp => _p.FrontSusp;
    public float RearSusp => _p.RearSusp;
    public BikeVisual Visual => _p.ComputeVisual();
    public Vector2 FwPos => _p.FwPos;
    public Vector2 RwPos => _p.RwPos;
    public float FwRot => _p.FwRot;
    public float RwRot => _p.RwRot;

    public event BikeStateChanged? OnStateChanged
    { add => _p.OnStateChanged += value; remove => _p.OnStateChanged -= value; }

    public event BikeEvent? OnCrash
    { add => _p.OnCrash += value; remove => _p.OnCrash -= value; }

    public event LandEvent? OnHardLanding
    { add => _p.OnHardLand += value; remove => _p.OnHardLand -= value; }

    public void Update(float dt)
    {
        if (!_disposed)
            _p.Update(dt);
    }

    public void SetInput(float thr, float brk, float lean)
    {
        if (!_disposed)
            _p.SetInput(thr, brk, lean);
    }

    public void SetLevel(Level level)
    {
        if (!_disposed)
            _p.SetTerrain(level);
    }

    public void Reset(float startX)
    {
#if DEBUG
        Log.Debug("BikeAPI", $"Reset({startX}) called");
#endif
        if (!_disposed)
            _p.Reset(startX);
    }

    public void SetType(BikeType t)
    {
        if (!_disposed)
            _p.SetType(t);
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        _p.Dispose();
        GC.SuppressFinalize(this);
    }
}
