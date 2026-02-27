using static System.MathF;
using static GravityDefiedGame.Models.Bike.BikePhysics;

namespace GravityDefiedGame.Models.Bike;

public enum BikeType : byte { Standard, Sport, OffRoad }

[Flags]
public enum BikeState : byte
{
    None = 0, Grounded = 1, InAir = 2,
    Wheelie = 4, Stoppie = 8, Crashed = 16
}

public delegate void BikeStateChanged(BikeState o, BikeState n);
public delegate void BikeEvent();
public delegate void LandEvent(float impactSpeed);

public readonly record struct BikeInput(
    bool Throttle, bool Brake, float Lean);

public readonly record struct BikeCfg(
    float WheelR, float Mass,
    float SpringK, float SpringDamp,
    float MaxWheelOmega, float LeanForce,
    float BounceDamp,
    float TorqueAccel, float MaxEngineTorque,
    float InvWheelInertia)
{
    public static BikeCfg Get(BikeType t) => t switch
    {
        BikeType.Standard => new(
            1.75f, 5.5f, 169f, 3f, 12.75f, 0.225f,
            0.5f, 37.5f, 750f, 0.33f),

        BikeType.Sport => new(
            1.75f, 5.5f, 186f, 3f, 15f, 0.34f,
            0.5f, 39.75f, 862f, 0.33f),

        BikeType.OffRoad => new(
            1.75f, 5.5f, 186f, 3f, 16.5f, 0.56f,
            0.5f, 40.5f, 900f, 0.33f),

        _ => throw new ArgumentOutOfRangeException(nameof(t))
    };
}

public readonly record struct SpringCfg(
    int A, int B, float RestLen, float K, float D);

public readonly struct PhysConst
{
    public static readonly PhysConst Default = new();

    public readonly float
        Gravity, FixedDt, WheelBase, CollMargin,
        WheelieAngle, StoppieAngle, TorqueDampRate, MinAxleDist2, MaxAxleDist2,
        UpperOffsetX, UpperOffsetY, HeadOffsetY, MaxRelVel, SpringMinDist,
        MinDist, Epsilon, MaxFrameDt, HardLandVel, PlaceOffsetY, DefaultGroundY,
        MinSlice, CollPadding, TiltCenter, TiltFollow,
        CrossSpringKMul, CrossSpringDMul, CrossSpringRwDMul,
        HeadSpringKMul, TerrainScale, HeadCrashVel, LeanHeadMul,
        FreeSpinDampRate, BrakeDampRate, RelVelDampRate;

    public PhysConst()
    {
        (Gravity, FixedDt, WheelBase) = (14.06f, 0.016f, 3.5f);
        CollMargin = 0.05f;

        (WheelieAngle, StoppieAngle, TorqueDampRate) = (0.32f, 0.32f, 6.6f);
        (MinAxleDist2, MaxAxleDist2) = (15f, 70f);

        (UpperOffsetX, UpperOffsetY, HeadOffsetY) = (2.0f, 3.0f, 7.1f);

        (MaxRelVel, SpringMinDist) = (22.5f, 0.0000458f);
        (MinDist, Epsilon, MaxFrameDt) = (0.01f, 0.0001f, 0.1f);

        (HardLandVel, PlaceOffsetY, DefaultGroundY) = (2.25f, 0.5f, 13.33f);

        (MinSlice, CollPadding) = (0.001f, 1.75f);

        (TiltCenter, TiltFollow) = (0.5f, 12f);

        (CrossSpringKMul, CrossSpringDMul, CrossSpringRwDMul) = (0.2f, 1.0f, 2.0f);

        (HeadSpringKMul, TerrainScale, HeadCrashVel, LeanHeadMul) = (1.1f, 6f, 1.5f, 18f);

        (FreeSpinDampRate, BrakeDampRate, RelVelDampRate) = (0.125f, 6.6f, 8f);
    }

    public float RestWheel => WheelBase;
    public float RestUpper => Hyp(UpperOffsetX, UpperOffsetY);
    public float RestUpperSpan => UpperOffsetX * 2f;
    public float RestWheelUpper => Hyp(WheelBase - UpperOffsetX, UpperOffsetY);
    public float RestHeadUpper => Hyp(UpperOffsetX, HeadOffsetY - UpperOffsetY);
    public float RestHeadFrame => HeadOffsetY;

    public float OuterR(float baseR) => baseR + CollMargin;
    public float InnerR(float baseR) => Max(baseR - CollMargin, 0.01f);

    static float Hyp(float a, float b) => Sqrt(a * a + b * b);

    public static float WrapAngle(float a) =>
        a - Tau * Floor((a + PI) / Tau);

    public static float Dist(float dx, float dy) => Sqrt(dx * dx + dy * dy);

    public static bool SafeNorm(
        float dx, float dy, float minDist,
        out float nx, out float ny)
    {
        float d = Dist(dx, dy);
        if (d < minDist)
        { nx = 0f; ny = -1f; return false; }

        nx = dx / d;
        ny = dy / d;
        return true;
    }
}

public readonly struct WheelFriction
{
    public static readonly WheelFriction Default = new();

    public readonly float
        RotDamp, SlipFriction, RollCouple,
        BounceFront, BounceRear, TangentDamp,
        BrakeBase, BrakeP, StopThreshold, MinWheelR;

    public WheelFriction()
    {
        (RotDamp, SlipFriction, RollCouple) = (0.7f, 0.2f, 0.6f);
        (BounceFront, BounceRear, TangentDamp) = (0.5f, 0.5f, 0.5f);
        (BrakeBase, BrakeP, StopThreshold, MinWheelR) = (0.4f, 0.4f, 0.1f, 0.1f);
    }
}

public readonly struct BodyCfg
{
    public static readonly BodyCfg Default = new();

    public readonly float[] Radii;
    public readonly float FrameCollOffset;

    public BodyCfg()
    {
        Radii = [0.5f, 1.75f, 1.75f, 0.5f, 0.5f, 0.5f];
        FrameCollOffset = 0.05f;
    }
}

public readonly struct MassCfg
{
    public static readonly MassCfg Default = new();

    public readonly float[] Mass;
    public readonly float[] InvMass;

    public MassCfg()
    {
        Mass = [0.275f, 0.075f, 0.275f, 0.175f, 0.175f, 0.225f];
        InvMass = new float[Mass.Length];
        for (int i = 0; i < Mass.Length; i++)
            InvMass[i] = 1f / Mass[i];
    }
}

public readonly record struct Pose(
    Vector2 Hip, Vector2 Sh, Vector2 Elb, Vector2 Hnd,
    Vector2 Nk, Vector2 Ft, Vector2 Kn, Vector2 Hd)
{
    public static Pose Lerp(in Pose a, in Pose b, float t) => new(
        Vector2.Lerp(a.Hip, b.Hip, t),
        Vector2.Lerp(a.Sh, b.Sh, t),
        Vector2.Lerp(a.Elb, b.Elb, t),
        Vector2.Lerp(a.Hnd, b.Hnd, t),
        Vector2.Lerp(a.Nk, b.Nk, t),
        Vector2.Lerp(a.Ft, b.Ft, t),
        Vector2.Lerp(a.Kn, b.Kn, t),
        Vector2.Lerp(a.Hd, b.Hd, t));

    public Pose ToWorld(Vector2 o, Vector2 fw, Vector2 pp, float sc)
    {
        Vector2 M(Vector2 v) =>
            o + pp * (v.X * sc) + fw * (v.Y * sc);

        return new(
            M(Hip), M(Sh), M(Elb), M(Hnd),
            M(Nk), M(Ft), M(Kn), M(Hd));
    }

    public void CopyTo(Span<Vector2> dest)
    {
        (dest[0], dest[1], dest[2], dest[3]) = (Hip, Sh, Elb, Hnd);
        (dest[4], dest[5], dest[6], dest[7]) = (Nk, Ft, Kn, Hd);
    }

    public static Pose FromSpan(ReadOnlySpan<Vector2> src) => new(
        src[0], src[1], src[2], src[3],
        src[4], src[5], src[6], src[7]);
}

public readonly struct PoseCfg
{
    public static readonly PoseCfg Default = new();

    public readonly float Scale, TiltMid;
    public readonly Pose L, N, R;

    public PoseCfg()
    {
        (Scale, TiltMid) = (1f, 0.5f);

        L = new(
            new(2.9f, -1.4f), new(3.9f, -3.6f),
            new(5.1f, -1.8f), new(6f, -0.7f),
            new(4.6f, 0.1f), new(1f, -1.2f),
            new(0.2f, -1.2f), new(4.4f, 1.3f));

        N = new(
            new(2.8f, -0.6f), new(4f, -2f),
            new(6f, -1f), new(7f, -0.6f),
            new(4.5f, 0.1f), new(0.3f, -2.2f),
            new(0.2f, -1.2f), new(4.4f, 1.3f));

        R = new(
            new(2.4f, 0.2f), new(4.5f, -0.2f),
            new(5.6f, 1.6f), new(6.2f, 2.7f),
            new(5.3f, 1.1f), new(0.6f, -1.5f),
            new(0.2f, -0.8f), new(4.4f, 1.3f));
    }
}

public static class Poses
{
    static readonly PoseCfg C = PoseCfg.Default;

    public static Pose ForTilt(float z) => z < C.TiltMid
        ? Pose.Lerp(C.L, C.N, Math.Clamp(z * 2f, 0f, 1f))
        : Pose.Lerp(C.N, C.R, Math.Clamp((z - C.TiltMid) * 2f, 0f, 1f));

    public static Pose Build(Vector2 fr, Vector2 uf, Vector2 ur, float tiltZ)
    {
        Vector2 ax = uf - ur;
        float len = Max(ax.Length(), 0.01f);
        Vector2 fw = ax / len;
        Vector2 pp = new(fw.Y, -fw.X);
        return ForTilt(tiltZ).ToWorld(fr, fw, pp, C.Scale);
    }
}

public struct PtState
{
    public Vector2 Pos, Vel, Force;
    public float AngVel, Torque, Angle;
}

public struct BodyDef
{
    public float R, InvInertia;
    public bool IsWheel;
}

public sealed partial class BikePhysics : IDisposable
{
    public const int
        BFrame = 0, BFw = 1, BRw = 2,
        BUF = 3, BUR = 4, BHead = 5,
        BodyCount = 6, SpringCount = 10;

    static readonly PhysConst P = PhysConst.Default;
    static readonly BodyCfg BodC = BodyCfg.Default;
    static readonly MassCfg MasC = MassCfg.Default;

    internal readonly PtState[] Cur = new PtState[BodyCount];
    internal readonly PtState[] New = new PtState[BodyCount];
    internal readonly BodyDef[] Bods = new BodyDef[BodyCount];
    internal readonly SpringCfg[] Spr = new SpringCfg[SpringCount];
    internal readonly float[] Mass = new float[BodyCount];
    internal readonly float[] InvMass = new float[BodyCount];

    internal BikeInput Input;
    internal BikeCfg Cfg;
    internal Level? Terrain;

    internal float EngineTorque, TiltZ;
    internal bool Braking, FwGnd, RwGnd;
    internal int FrameNum;

    internal bool LeanL => !Crashed && Input.Lean < 0f;
    internal bool LeanR => !Crashed && Input.Lean > 0f;

    readonly BikeDynamics _dyn;
    readonly BikeCollider _col;
    readonly RiderRagdoll _ragdoll = new();

    bool _disposed;
    float _accumDt;

    public BikeCfg CfgPub => Cfg;
    public BikeType Type { get; private set; }
    public BikeState State { get; private set; } = BikeState.Grounded;

    public Vector2 Pos => Cur[BFrame].Pos;
    public Vector2 Vel => Cur[BFrame].Vel;

    public Vector2 FwPos => Cur[BFw].Pos;
    public Vector2 RwPos => Cur[BRw].Pos;
    public Vector2 UfPos => Cur[BUF].Pos;
    public Vector2 UrPos => Cur[BUR].Pos;
    public Vector2 HdPos => Cur[BHead].Pos;

    public float Ang { get; private set; }
    public float AngVel { get; private set; }
    public float Speed => Vel.Length();

    public float FwRot => Cur[BFw].Angle;
    public float RwRot => Cur[BRw].Angle;

    public float RawTiltZ => TiltZ;

    public bool InAir => !FwGnd && !RwGnd;
    public bool IsGrounded => FwGnd || RwGnd;
    public bool Crashed => (State & BikeState.Crashed) != 0;

    public bool RagdollActive => _ragdoll.On;
    public bool RagdollDone { get; set; }
    public Pose RiderPose => _ragdoll.On ? _ragdoll.GetPose() : GetPose();

    public event BikeStateChanged? OnStateChanged;
    public event BikeEvent? OnCrash;
    public event LandEvent? OnHardLand;

    public BikePhysics(BikeType type)
    {
        Type = type;
        Cfg = BikeCfg.Get(type);
        _dyn = new BikeDynamics(this);
        _col = new BikeCollider(this, _dyn);
        Build();
    }

    public void SetTerrain(Level level)
    {
        if (!_disposed)
            Terrain = level;
    }

    public void SetType(BikeType t)
    {
        if (_disposed)
            return;

        Type = t;
        Cfg = BikeCfg.Get(t);
        Build();
    }

    public void SetInput(BikeInput inp)
    {
        if (_disposed || Crashed)
            return;
        Input = inp;
    }

    public void Reset(float startXMeters)
    {
        if (_disposed)
            return;

        _ragdoll.Stop();
        Build();
        Place(startXMeters);
        ResetState();
    }

    public void Update(float dt)
    {
        if (_disposed || dt <= 0f)
            return;

        if (Crashed && RagdollDone)
            return;

        _accumDt += Min(dt, P.MaxFrameDt);

        while (_accumDt >= P.FixedDt)
        {
            if (_ragdoll.On)
                _ragdoll.StepFixed(P.FixedDt);

            StepFixed(P.FixedDt);
            _accumDt -= P.FixedDt;
        }
    }

    public void Dispose()
    {
        if (_disposed)
            return;

        _disposed = true;
        (OnStateChanged, OnCrash, OnHardLand) = (null, null, null);
    }

    void ResetState()
    {
        (Input, _accumDt) = (default, 0f);
        (TiltZ, FrameNum, Ang, AngVel) = (P.TiltCenter, 0, 0f, 0f);
        (FwGnd, RwGnd, Braking) = (false, false, false);
        EngineTorque = 0f;
        RagdollDone = false;
        State = BikeState.Grounded;
    }

    Pose GetPose() =>
        Poses.Build(Cur[BFrame].Pos, Cur[BUF].Pos, Cur[BUR].Pos, TiltZ);

    void Build()
    {
        for (int i = 0; i < BodyCount; i++)
        {
            Mass[i] = MasC.Mass[i];
            InvMass[i] = MasC.InvMass[i];
            Bods[i] = new()
            {
                R = BodC.Radii[i],
                IsWheel = i is BFw or BRw,
                InvInertia = i == BRw ? Cfg.InvWheelInertia : 0f
            };
        }

        _dyn.RebuildSprings();
    }

    void Place(float x)
    {
        float scale = P.TerrainScale;
        float gy = Terrain != null
            ? Terrain.GetGroundYAtX(x * scale) / scale
            : P.DefaultGroundY;

        float baseY = gy - Cfg.WheelR - P.PlaceOffsetY;
        Array.Clear(Cur);

        Cur[BFrame].Pos = new(x, baseY);
        Cur[BFw].Pos = new(x + P.WheelBase, baseY);
        Cur[BRw].Pos = new(x - P.WheelBase, baseY);
        Cur[BUF].Pos = new(x + P.UpperOffsetX, baseY - P.UpperOffsetY);
        Cur[BUR].Pos = new(x - P.UpperOffsetX, baseY - P.UpperOffsetY);
        Cur[BHead].Pos = new(x, baseY - P.HeadOffsetY);

        Array.Copy(Cur, New, BodyCount);
    }

    void StepFixed(float dt)
    {
        FrameNum++;
        float oldVelY = Cur[BFrame].Vel.Y;
        bool wasAir = InAir;

        Braking = !Crashed && Input.Brake;

        _dyn.UpdateMassAndLean();

        int res = _col.StepCollision(dt);

        if (!Crashed && res < 0)
        {
            DoCrash();
#if DEBUG
            UpdateDebugStats(dt);
#endif
            return;
        }

        UpdAngle();

        if (wasAir && !InAir && oldVelY > P.HardLandVel)
            OnHardLand?.Invoke(oldVelY);

        UpdState();

#if DEBUG
        UpdateDebugStats(dt);
#endif
    }

    void UpdAngle()
    {
        Vector2 ax = Cur[BFw].Pos - Cur[BRw].Pos;
        float newAng = Atan2(ax.Y, ax.X);
        float diff = PhysConst.WrapAngle(newAng - Ang);
        (AngVel, Ang) = (diff / P.FixedDt, newAng);
    }

    void UpdState()
    {
        if (Crashed)
            return;

        BikeState prev = State;
        BikeState next = IsGrounded ? BikeState.Grounded : BikeState.InAir;

        float normAng = PhysConst.WrapAngle(Ang);

        if (RwGnd && !FwGnd && normAng < -P.WheelieAngle)
            next |= BikeState.Wheelie;

        if (FwGnd && !RwGnd && normAng > P.StoppieAngle)
            next |= BikeState.Stoppie;

        if (State != next)
        {
            State = next;
            OnStateChanged?.Invoke(prev, next);
        }
    }

    void DoCrash()
    {
        BikeState prev = State;
        State |= BikeState.Crashed;
        Input = default;

        if (!_ragdoll.On)
            _ragdoll.Start(GetPose(), Vel, AngVel, Terrain);

        if (prev != State)
            OnStateChanged?.Invoke(prev, State);

        OnCrash?.Invoke();
    }
}

// BikePhysics handles input events crash state and public API
// BikeDynamics handles forces springs and integration math
// Split exists because handling tuning changes constantly during playtesting
// Input mapping and event wiring should remain untouched during tuning
//
// Physics must run on fixed 60fps timestep
// Every level jump and obstacle was hand-tuned against that exact tick rate
// Variable dt breaks level design
//
// Scratch arrays _tmp _k1 _k2 are pre-allocated and reused every frame
// Allocating per-tick causes GC spikes mid-race ruining frame pacing
public sealed class BikeDynamics(BikePhysics p)
{
    static readonly PhysConst P = PhysConst.Default;
    static readonly WheelFriction WF = WheelFriction.Default;

    readonly PtState[] _tmp = new PtState[BodyCount];
    readonly PtState[] _k1 = new PtState[BodyCount];
    readonly PtState[] _k2 = new PtState[BodyCount];

    public void RebuildSprings()
    {
        float k = p.Cfg.SpringK, d = p.Cfg.SpringDamp;
        float cK = k * P.CrossSpringKMul,
              cD = d * P.CrossSpringDMul,
              cDrw = d * P.CrossSpringRwDMul,
              hK = k * P.HeadSpringKMul;

        (p.Spr[0], p.Spr[1]) = (
            new(BFrame, BFw, P.RestWheel, k, d),
            new(BFrame, BRw, P.RestWheel, k, d));

        (p.Spr[2], p.Spr[3]) = (
            new(BFrame, BUF, P.RestUpper, k, d),
            new(BFrame, BUR, P.RestUpper, k, d));

        p.Spr[4] = new(BUF, BUR, P.RestUpperSpan, k, d);

        (p.Spr[5], p.Spr[6]) = (
            new(BFw, BUF, P.RestWheelUpper, cK, cD),
            new(BRw, BUR, P.RestWheelUpper, cK, cDrw));

        (p.Spr[7], p.Spr[8], p.Spr[9]) = (
            new(BHead, BUR, P.RestHeadUpper, hK, d),
            new(BHead, BUF, P.RestHeadUpper, hK, d),
            new(BHead, BFrame, P.RestHeadFrame, hK, d));
    }

    public void UpdateMassAndLean()
    {
        if (p.Crashed)
            return;

        Engine();

        float t = P.TiltCenter * (p.Input.Lean + 1f);
        p.TiltZ += (t - p.TiltZ) * (1f - Exp(-P.TiltFollow * P.FixedDt));
    }

    // Fixed 60 Hz tick matches level layout and spring tuning
    // Heun kept since bike constants were tuned for its two force passes
    //
    // Collision may split a tick into smaller slices when penetration gets too deep
    // Each slice runs integration and collision resolve then repeats until dt is consumed
    // Damping and engine torque decay use slice dt so total damping over a full tick stays time based
    // Using FixedDt would apply full tick damping per slice causing extra power loss and faster wheel spin drop on rough ground
    //
    // Wheel Angle and AngVel updated after linear blend
    // Wheel rotation drives slip and braking so moving it into the blend changes handling and tuning
    // Keeping this order avoids retuning levels and bike types
    public void Heun(float dt)
    {
        Forces(p.Cur, dt);
        Integ(p.Cur, _k1, dt);

        Combine(_tmp, p.Cur, _k1, 1.0f);

        Forces(_tmp, dt);
        Integ(_tmp, _k2, dt);

        CombineTwo(p.New, p.Cur, _k1, _k2, 0.5f);

        for (int i = BFw; i <= BRw; i++)
        {
            p.New[i].Angle = p.Cur[i].Angle + p.Cur[i].AngVel * dt;
            p.New[i].AngVel = p.Cur[i].AngVel
                + p.Bods[i].InvInertia * p.Cur[i].Torque * dt;
        }
    }

    static void Combine(PtState[] dst, PtState[] src, PtState[] k, float scale)
    {
        for (int i = 0; i < BodyCount; i++)
        {
            dst[i].Pos = src[i].Pos + k[i].Pos * scale;
            dst[i].Vel = src[i].Vel + k[i].Vel * scale;
            (dst[i].AngVel, dst[i].Angle) = (src[i].AngVel, src[i].Angle);
        }
    }

    static void CombineTwo(PtState[] dst, PtState[] src, PtState[] k1, PtState[] k2, float scale)
    {
        for (int i = 0; i < BodyCount; i++)
        {
            dst[i].Pos = src[i].Pos + (k1[i].Pos + k2[i].Pos) * scale;
            dst[i].Vel = src[i].Vel + (k1[i].Vel + k2[i].Vel) * scale;
            (dst[i].AngVel, dst[i].Angle) = (src[i].AngVel, src[i].Angle);
        }
    }

    void Forces(PtState[] st, float dt)
    {
        for (int i = 0; i < BodyCount; i++)
        {
            st[i].Force = new(0f, P.Gravity * p.Mass[i]);
            st[i].Torque = 0f;
        }

        Lean(st);

        for (int i = 0; i < SpringCount; i++)
            Spring(ref st[p.Spr[i].A], ref st[p.Spr[i].B], i);

        p.EngineTorque *= Exp(-P.TorqueDampRate * dt);

        st[BRw].Torque = p.EngineTorque;
        st[BRw].AngVel = Math.Clamp(
            st[BRw].AngVel,
            -p.Cfg.MaxWheelOmega,
            p.Cfg.MaxWheelOmega);

        float spinDamp = Exp(-P.FreeSpinDampRate * dt);
        st[BFw].AngVel *= spinDamp;
        st[BRw].AngVel *= spinDamp;

        DampRelativeVelocities(st, dt);
    }

    void Integ(PtState[] cur, PtState[] k, float dt)
    {
        for (int i = 0; i < BodyCount; i++)
        {
            k[i].Pos = cur[i].Vel * dt;
            k[i].Vel = cur[i].Force * (dt * p.InvMass[i]);
            k[i].AngVel = 0f;
        }
    }

    void Spring(ref PtState a, ref PtState b, int idx)
    {
        ref SpringCfg s = ref p.Spr[idx];

        float dx = a.Pos.X - b.Pos.X;
        float dy = a.Pos.Y - b.Pos.Y;
        float dist = PhysConst.Dist(dx, dy);
        if (dist < P.SpringMinDist)
            return;

        float nx = dx / dist, ny = dy / dist;

        float dvx = a.Vel.X - b.Vel.X;
        float dvy = a.Vel.Y - b.Vel.Y;
        float f = s.K * (dist - s.RestLen)
                + s.D * (dvx * nx + dvy * ny);

        a.Force.X -= nx * f;
        a.Force.Y -= ny * f;
        b.Force.X += nx * f;
        b.Force.Y += ny * f;
    }

    void Engine()
    {
        if (p.Braking)
        {
            p.EngineTorque = 0f;
            float damp = Exp(-P.BrakeDampRate * P.FixedDt);
            BrakeWheel(ref p.Cur[BFw].AngVel, damp);
            BrakeWheel(ref p.Cur[BRw].AngVel, damp);
            return;
        }

        if (p.Input.Throttle)
            p.EngineTorque = Min(
                p.EngineTorque + p.Cfg.TorqueAccel,
                p.Cfg.MaxEngineTorque);
    }

    static void BrakeWheel(ref float w, float d)
    {
        w *= d;
        if (Abs(w) < WF.StopThreshold)
            w = 0f;
    }

    // Lean is rider tilt control
    //
    // In original J2ME lean was a direct velocity kick on UF UR and Head each tick
    // Gravity and springs lived in a separate force accumulator
    // Lean wrote straight into the velocity layer once per tick before integration
    //
    // Here lean goes through forces so the integrator and collision sub steps handle timing and dt scaling
    // Trade off is that LeanForce numbers from J2ME do not map 1 to 1
    // because forces go through mass and dt before becoming velocity changes
    //
    // Bike axis nx ny is from rear wheel to front wheel
    // Sideways direction perp = -ny nx works the same on flat ground slopes and in the air
    //
    // UF and UR get equal and opposite sideways forces
    // This creates a torque couple on the frame without net linear push
    // Wheels are not pushed so wheelie and stoppie stay driven by contacts and springs
    //
    // Head gets a force along the bike axis scaled by LeanHeadMul
    // Multiplier is needed because forces are weaker per tick than direct velocity changes
    // LeanHeadMul is gameplay tuning not a physical parameter so it lives in PhysConst
    //
    // Head force is compensated on the frame to prevent lean self-acceleration
    // Original J2ME left this uncompensated adding net momentum to the system
    void Lean(PtState[] st)
    {
        float tilt = p.TiltZ / P.TiltCenter - 1f;
        if (Abs(tilt) <= P.Epsilon)
            return;

        float dx = st[BFw].Pos.X - st[BRw].Pos.X;
        float dy = st[BFw].Pos.Y - st[BRw].Pos.Y;
        if (!PhysConst.SafeNorm(dx, dy, P.MinDist, out float nx, out float ny))
            return;

        float f = p.Cfg.LeanForce * tilt;
        float px = -ny * f, py = nx * f;
        float hx = nx * f * P.LeanHeadMul, hy = ny * f * P.LeanHeadMul;

        st[BUF].Force.X -= px;
        st[BUF].Force.Y -= py;
        st[BUR].Force.X += px;
        st[BUR].Force.Y += py;
        st[BHead].Force.X += hx;
        st[BHead].Force.Y += hy;
        st[BFrame].Force.X -= hx;
        st[BFrame].Force.Y -= hy;
    }

    // Exponential damping on relative velocities exceeding MaxRelVel
    // Proportional to excess speed, dt-independent via rate constant
    // Prevents explosive spring forces without shockwaves from hard clamping
    static void DampRelativeVelocities(PtState[] st, float dt)
    {
        float cx = 0f, cy = 0f;
        for (int i = 0; i < BodyCount; i++)
        {
            cx += st[i].Vel.X;
            cy += st[i].Vel.Y;
        }

        float inv = 1f / BodyCount;
        (cx, cy) = (cx * inv, cy * inv);

        for (int i = 0; i < BodyCount; i++)
        {
            float vx = st[i].Vel.X - cx;
            float vy = st[i].Vel.Y - cy;
            float spd = PhysConst.Dist(vx, vy);

            if (spd > P.MaxRelVel)
            {
                float factor = Exp(-P.RelVelDampRate * (spd - P.MaxRelVel) / spd * dt);
                st[i].Vel.X = cx + vx * factor;
                st[i].Vel.Y = cy + vy * factor;
            }
        }
    }
}

// Collision detection uses adaptive time-stepping with bisection
// When penetration is too deep we halve the timestep and re-integrate
// This avoids tunneling through thin terrain segments at high speeds
//
// Three collision zones per body:
// - touch zone: sets FwGnd/RwGnd flags for gameplay state detection
// - outer zone: triggers velocity reflection (Resolve)
// - inner zone: penetration too deep, requires timestep reduction
//
// ChkColl returns CollResult containing body index and collision normal
// The while(Resolve) loop handles multiple colliding bodies sequentially
// Each Resolve flips velocity direction so same body wont trigger again
// MaxResolveIter caps iterations as safety against degenerate geometry
//
// Collision normal is normalized for consistent push and reflection
//
// Head collision crashes the bike when impact exceeds HeadCrashVel threshold
// Upper frame points BUF/BUR skip collision until crashed
// to prevent false positives during wheelies and stoppies
public sealed class BikeCollider(BikePhysics p, BikeDynamics dyn)
{
    private enum CollCode { None, Resolve, DeepPen, HeadCrash }

    private readonly record struct CollResult(CollCode Code, int Idx, float Nx, float Ny, float Pen);

    const int MaxResolveIter = 8;

    private static readonly PhysConst P = PhysConst.Default;
    private static readonly WheelFriction WF = WheelFriction.Default;
    private static readonly BodyCfg BodC = BodyCfg.Default;
    private static readonly CollResult NoHit = new(CollCode.None, -1, 0f, 0f, 0f);

    private readonly PtState[] _curBackup = new PtState[BodyCount];

    private CollResult _lastColl;

    internal int CollIdx => _lastColl.Idx;
    internal float CollNx => _lastColl.Nx;
    internal float CollNy => _lastColl.Ny;

    private static float Scale => P.TerrainScale;

    private Vector2 ScaledPos(int i) => p.New[i].Pos * Scale;
    private Vector2 ScaledVel(int i) => p.New[i].Vel * Scale;

    private CollResult Ret(CollResult r) { _lastColl = r; return r; }

    bool StepSlice(float sliceDt, out CollResult cr)
    {
        Array.Copy(p.Cur, _curBackup, BodyCount);
        float torqueBk = p.EngineTorque;
        bool fwBk = p.FwGnd;
        bool rwBk = p.RwGnd;

        Array.Copy(p.Cur, p.New, BodyCount);
        dyn.Heun(sliceDt);

        cr = ChkColl();

        if (cr.Code is CollCode.HeadCrash or not CollCode.DeepPen)
            return true;

        Array.Copy(_curBackup, p.Cur, BodyCount);
        p.EngineTorque = torqueBk;
        p.FwGnd = fwBk;
        p.RwGnd = rwBk;
        return false;
    }

    public int StepCollision(float dt)
    {
#if DEBUG
        p.ResetContactStats();
#endif
        (p.FwGnd, p.RwGnd) = (false, false);

        float proc = 0f, targ = dt;

        while (proc < dt)
        {
            float sliceDt = targ - proc;

            if (!StepSlice(sliceDt, out CollResult cr))
            {
                targ = (proc + targ) * 0.5f;
                if (targ - proc < P.MinSlice)
                    return -1;
                continue;
            }

            for (int r = 0; cr.Code == CollCode.Resolve && r < MaxResolveIter; r++)
            {
                Resolve(cr.Idx, cr.Nx, cr.Ny, cr.Pen);
                cr = ChkColl();
            }

            if (cr.Code != CollCode.None)
                return -1;

            Array.Copy(p.New, p.Cur, BodyCount);
            proc = targ;
            if (proc < dt)
                targ = dt;
        }

        float dx = p.Cur[BFw].Pos.X - p.Cur[BRw].Pos.X,
              dy = p.Cur[BFw].Pos.Y - p.Cur[BRw].Pos.Y,
              d2 = dx * dx + dy * dy;

        return d2 < P.MinAxleDist2 || d2 > P.MaxAxleDist2 ? -1 : 0;
    }

    CollResult ChkColl()
    {
        if (p.Terrain == null)
            return Ret(NoHit);

        Vector2 fwP = ScaledPos(BFw), rwP = ScaledPos(BRw), hdP = ScaledPos(BHead);

        float baseR = p.Bods[BFw].R,
              outerR = P.OuterR(baseR) * Scale,
              touchR = outerR + outerR - P.InnerR(baseR) * Scale,
              minX = Min(fwP.X, Min(rwP.X, hdP.X)),
              maxX = Max(fwP.X, Max(rwP.X, hdP.X));

        p.Terrain.UpdateVisible(minX, maxX, touchR + P.CollPadding * Scale);

        PhysConst.SafeNorm(
            fwP.X - rwP.X, fwP.Y - rwP.Y,
            P.MinDist, out float axNx, out float axNy);

        CollResult result = NoHit;
        float headVnLimit = P.HeadCrashVel * Scale;

        for (int i = 0; i < BodyCount; i++)
        {
            if ((!p.Crashed && i is BUF or BUR) || (p.Crashed && i == BHead))
                continue;

            Vector2 pos = ScaledPos(i), vel = ScaledVel(i);
            float x = pos.X, y = pos.Y;

            if (i == BFrame)
            {
                float off = BodC.FrameCollOffset * Scale;
                x -= axNy * off;
                y += axNx * off;
            }

            float outR = P.OuterR(p.Bods[i].R) * Scale,
                  inR = P.InnerR(p.Bods[i].R) * Scale;

            CollCode code = ChkPtColl(x, y, vel.X, vel.Y, outR, inR,
                out bool touch, out float nx, out float ny, out float segPen);

            if (i == BFw)
                p.FwGnd |= touch;
            if (i == BRw)
                p.RwGnd |= touch;

            if (i == BHead && !p.Crashed && code != CollCode.None)
            {
                if (Abs(vel.X * nx + vel.Y * ny) <= headVnLimit)
                    continue;
                return Ret(new(CollCode.HeadCrash, BHead, nx, ny, 0f));
            }

            switch (code)
            {
                case CollCode.DeepPen:
                    return Ret(new(CollCode.DeepPen, i, nx, ny, 0f));

                case CollCode.Resolve:
                    result = new(CollCode.Resolve, i, nx, ny, segPen / Scale);
                    break;
            }
        }

        return Ret(result);
    }

    CollCode ChkPtColl(
        float x, float y, float vx, float vy,
        float outR, float inR, out bool touch,
        out float nx, out float ny, out float pen)
    {
        touch = false;
        (nx, ny, pen) = (0f, 0f, 0f);
        (int s, int e) = p.Terrain!.GetVisibleRange();

        float touchR = outR + outR - inR,
              touchSq = touchR * touchR,
              outSq = outR * outR,
              inSq = inR * inR;

        int hits = 0;
        float nxSum = 0f, nySum = 0f, maxPen = 0f;

        for (int i = s; i < e; i++)
        {
            p.Terrain.GetSegmentPoints(i, out float sx, out float sy, out float ex, out float ey);
            if (x - touchR > ex || x + touchR < sx)
                continue;

            float dx = ex - sx, dy = ey - sy, lenSq = dx * dx + dy * dy;
            if (lenSq < P.Epsilon)
                continue;

            float t = Math.Clamp(((x - sx) * dx + (y - sy) * dy) / lenSq, 0f, 1f),
                  rx = x - sx - t * dx,
                  ry = y - sy - t * dy,
                  distSq = rx * rx + ry * ry;

            if (distSq >= touchSq)
                continue;
            touch = true;

            if (distSq >= outSq)
                continue;

            p.Terrain.GetSegmentNormal(i, out float snx, out float sny);
            if (vx * snx + vy * sny >= 0f)
                continue;

            if (distSq < inSq)
            {
                (nx, ny) = (snx, sny);
                return CollCode.DeepPen;
            }

            float segPen = outR - Sqrt(distSq);
            if (segPen > maxPen)
                maxPen = segPen;

            hits++;
            nxSum += snx;
            nySum += sny;
        }

        if (hits == 0 || vx * nxSum + vy * nySum >= 0f)
            return CollCode.None;

        float len = PhysConst.Dist(nxSum, nySum);
        if (len <= P.Epsilon)
            return CollCode.None;

        (nx, ny) = (nxSum / len, nySum / len);
        pen = maxPen;
        return CollCode.Resolve;
    }

    void Resolve(int idx, float nx, float ny, float pen)
    {
#if DEBUG
        p.RecordKeBefore(idx);
#endif
        ref PtState pt = ref p.New[idx];

        float push = pen + P.Epsilon;
        pt.Pos.X += nx * push;
        pt.Pos.Y += ny * push;

        float vn = -(pt.Vel.X * nx + pt.Vel.Y * ny);
        float vt = pt.Vel.X * ny - pt.Vel.Y * nx;

        float bd, rt;
        if (p.Bods[idx].IsWheel)
            (bd, rt) = ResolveWheel(idx, ref pt, vt);
        else
            (bd, rt) = (WF.BounceRear, WF.TangentDamp * vt);

        float rn = bd * vn;
        pt.Vel.X = rt * ny + rn * nx;
        pt.Vel.Y = -rt * nx + rn * ny;

#if DEBUG
        p.RecordResolve(idx, vn, nx, ny, in pt);
#endif
    }

    (float bounce, float tangent) ResolveWheel(int idx, ref PtState pt, float vt)
    {
        float rd = WF.RotDamp;
        float td = WF.TangentDamp;
        float bounce = idx == BFw ? WF.BounceFront : WF.BounceRear;

        if (p.Braking && Abs(pt.AngVel) < WF.StopThreshold)
        {
            float brk = WF.BrakeBase - WF.BrakeP;
            (rd, td, bounce) = (rd - WF.BrakeP, brk, brk);
        }

        float rad = Max(p.Bods[idx].R, WF.MinWheelR);
        pt.AngVel = rd * pt.AngVel - WF.SlipFriction * vt / rad;

        return (bounce, td * vt - WF.RollCouple * pt.AngVel * rad);
    }
}

public readonly struct RagdollCfg
{
    public readonly float
        K, D, Bounce, Friction,
        CollRadius, SlopeStep,
        CoreKMul, CoreDMul, CoreMinFrac, CoreStopMul,
        HeadDampMul, StretchFrac, StretchMul,
        MinLen, MinNy;

    public RagdollCfg()
    {
        (K, D) = (150f, 8f);
        (Bounce, Friction) = (0.3f, 0.8f);
        (CollRadius, SlopeStep) = (0.5f, 0.5f);
        (CoreKMul, CoreDMul, CoreMinFrac, CoreStopMul) = (2.2f, 1.4f, 0.75f, 6f);
        HeadDampMul = 2.0f;
        (StretchFrac, StretchMul) = (1.1f, 5f);
        (MinLen, MinNy) = (0.01f, 0.2f);
    }
}

// Spring-mass ragdoll used after a crash
// Starts from the current rider pose and inherits bike linear and angular velocity
// Steps on the same fixed dt as bike physics to keep springs stable
// Terrain contact uses slope normal so points do not sink into hills
// Push distance compensates for steep slopes because penetration is measured vertically
// Collision radius is tuned for line rendering so thick strokes do not clip into ground
public sealed class RiderRagdoll
{
    const int N = 8, NS = 11, Hd = 7;

    static readonly PhysConst P = PhysConst.Default;
    static readonly RagdollCfg C = new();

    static readonly (int A, int B)[] Skeleton =
    [
        (0, 1), (1, 2), (2, 3), (1, 4),
        (4, 7), (0, 6), (6, 5), (0, 4),
        (2, 4), (1, 6), (0, 7)
    ];

    readonly Vector2[] _pos = new Vector2[N];
    readonly Vector2[] _vel = new Vector2[N];
    readonly float[] _rest = new float[NS];

    Level? _terrain;

    public bool On { get; private set; }

    public Pose GetPose() => Pose.FromSpan(_pos);

    public void Stop() => (On, _terrain) = (false, null);

    public void Start(in Pose pose, Vector2 linVel, float angVel, Level? terrain)
    {
        (On, _terrain) = (true, terrain);
        pose.CopyTo(_pos);

        Vector2 c = default;
        for (int i = 0; i < N; i++)
            c += _pos[i];
        c *= 1f / N;

        for (int i = 0; i < N; i++)
        {
            Vector2 r = _pos[i] - c;
            _vel[i] = linVel + new Vector2(-angVel * r.Y, angVel * r.X);
        }

        for (int i = 0; i < NS; i++)
        {
            (int a, int b) = Skeleton[i];
            _rest[i] = Max(Vector2.Distance(_pos[a], _pos[b]), C.MinLen);
        }
    }

    public void StepFixed(float dt)
    {
        if (!On)
            return;

        ApplySprings(dt);
        Integrate(dt);
        CollideTerrain();
    }

    void ApplySprings(float dt)
    {
        for (int i = 0; i < NS; i++)
        {
            (int a, int b) = Skeleton[i];

            Vector2 d = _pos[a] - _pos[b];
            float len = d.Length();
            if (len < C.MinLen)
                continue;

            float k = C.K, damp = C.D, rest = _rest[i];

            if (IsCoreBone(a) && IsCoreBone(b))
            {
                k *= C.CoreKMul;
                damp *= C.CoreDMul;

                if (a == Hd || b == Hd)
                    damp *= C.HeadDampMul;

                if (len < rest * C.CoreMinFrac)
                    k *= 1f + (1f - len / (rest * C.CoreMinFrac)) * (C.CoreStopMul - 1f);
            }

            if (len > rest * C.StretchFrac)
                k *= 1f + (len / (rest * C.StretchFrac) - 1f) * (C.StretchMul - 1f);

            Vector2 n = d / len;
            float rel = Vector2.Dot(_vel[a] - _vel[b], n);
            float imp = ((len - rest) * k + rel * damp) * dt * 0.5f;

            _vel[a] -= n * imp;
            _vel[b] += n * imp;
        }
    }

    void Integrate(float dt)
    {
        for (int i = 0; i < N; i++)
        {
            _vel[i].Y += P.Gravity * dt;
            _pos[i] += _vel[i] * dt;
        }
    }

    void CollideTerrain()
    {
        if (_terrain == null)
            return;

        Level level = _terrain;
        float sc = P.TerrainScale;
        float Gy(float x) => level.GetGroundYAtX(x * sc) / sc;

        for (int i = 0; i < N; i++)
        {
            float x = _pos[i].X, gy = Gy(x);
            float pen = _pos[i].Y + C.CollRadius - gy;
            if (pen <= 0f)
                continue;

            PhysConst.SafeNorm(
                Gy(x + C.SlopeStep) - gy, -C.SlopeStep, C.MinLen,
                out float nx, out float ny);

            float push = pen / Max(-ny, C.MinNy);
            _pos[i].X += nx * push;
            _pos[i].Y += ny * push;

            float vx = _vel[i].X, vy = _vel[i].Y;

            float vn = vx * nx + vy * ny;
            float vt = (vx * ny - vy * nx) * C.Friction;

            if (vn < 0f)
                vn = -vn * C.Bounce;

            _vel[i].X = nx * vn + ny * vt;
            _vel[i].Y = ny * vn - nx * vt;
        }
    }

    static bool IsCoreBone(int i) => i is 0 or 1 or 4 or 7;
}
