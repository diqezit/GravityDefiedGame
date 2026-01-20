using static System.MathF;
using Vector2 = Microsoft.Xna.Framework.Vector2;

namespace GravityDefiedGame.Models.Bike;

public enum BikeType { Standard, Sport, OffRoad }

[Flags]
public enum BikeState
{
    None = 0, Grounded = 1, InAir = 2, Wheelie = 4, Stoppie = 8, Crashed = 16
}

public delegate void BikeStateChanged(BikeState o, BikeState n);
public delegate void BikeEvent();
public delegate void LandEvent(float impactSpeed);

public readonly record struct BikeInput(float Throttle, float Brake, float Lean);

public readonly record struct PhysConst(
    float Gravity, float FixedDt,
    float Scale, float InvScale, float WheelBase,
    float PushOut, float CollMargin,
    float CrashAngle, float CrashTime, float WheelieAngle, float StoppieAngle,
    float MinAxleDist2, float MaxAxleDist2, float TorqueDamp,
    float UpperOffsetX, float UpperOffsetY, float HeadOffsetY,
    float MaxRelVel, float SpringMinDist, float FrameCollOffset)
{
    public static readonly PhysConst Default = new(
        Gravity: 25f, FixedDt: 0.016f,
        Scale: 6f, InvScale: 1f / 6f, WheelBase: 3.5f,
        PushOut: 0.01f, CollMargin: 0.1f,
        CrashAngle: 2.1f, CrashTime: 0.5f, WheelieAngle: 0.32f, StoppieAngle: 0.32f,
        MinAxleDist2: 15f, MaxAxleDist2: 70f, TorqueDamp: 0.1f,
        UpperOffsetX: 2.0f, UpperOffsetY: 3.0f, HeadOffsetY: 5.0f,
        MaxRelVel: 30f, SpringMinDist: 0.0000458f, FrameCollOffset: 1.0f);

    public readonly float RestWheel => WheelBase;
    public readonly float RestUpper =>
        Sqrt(UpperOffsetX * UpperOffsetX + UpperOffsetY * UpperOffsetY);
    public readonly float RestUpperSpan => UpperOffsetX * 2f;
    public readonly float RestWheelUpper =>
        Sqrt((WheelBase - UpperOffsetX) * (WheelBase - UpperOffsetX) +
             UpperOffsetY * UpperOffsetY);
    public readonly float RestHeadUpper =>
        Sqrt(UpperOffsetX * UpperOffsetX +
             (HeadOffsetY - UpperOffsetY) * (HeadOffsetY - UpperOffsetY));
    public readonly float RestHeadFrame => HeadOffsetY;

    public readonly float OuterR(float baseR) => baseR + CollMargin;

    public readonly float InnerR(float baseR)
    {
        float v = baseR - CollMargin;
        return v < 0.01f ? 0.01f : v;
    }
}

public readonly record struct WheelFriction(
    float RotDamp, float SlipFriction, float RollCouple,
    float BounceFront, float BounceRear,
    float TangentDamp, float BrakeDamp,
    float BrakeBase, float BrakeP)
{
    public static readonly WheelFriction Default = new(
        RotDamp: 0.7f,
        SlipFriction: 0.2f,
        RollCouple: 0.6f,
        BounceFront: 0.5f,
        BounceRear: 0.5f,
        TangentDamp: 0.5f,
        BrakeDamp: 0.1f,
        BrakeBase: 0.4f,
        BrakeP: 0.4f);
}

public readonly record struct BikeCfg(
    float WheelR, float Mass,
    float SpringK, float SpringDamp,
    float MaxWheelOmega, float LeanForce, float MaxTilt, float BounceDamp,
    float TorqueAccel, float MaxEngineTorque)
{
    public static BikeCfg Get(BikeType t) => t switch
    {
        BikeType.Sport => new(1.75f, 5.5f, 330f, 4f, 20f, 0.6f, 5f, 0.5f, 53f, 1150f),
        BikeType.OffRoad => new(1.75f, 5.5f, 330f, 4f, 22f, 1.0f, 20f, 0.5f, 54f, 1200f),
        _ => new(1.75f, 5.5f, 300f, 4f, 17f, 0.4f, 5f, 0.5f, 50f, 1000f)
    };
}

public readonly record struct SpringCfg(int A, int B, float RestLen, float K, float D);

public readonly record struct BikeVisual(
    Vector2 Fr, Vector2 Fw, Vector2 Rw, Vector2 Uf, Vector2 Ur, Vector2 Hd,
    float WhlR, float FwRot, float RwRot, float TiltZ)
{
    public static BikeVisual Empty => new(
        default, default, default, default, default, default,
        10.5f, 0f, 0f, 0.5f);
}

public sealed partial class BikePhysics : IDisposable
{
    static readonly PhysConst P = PhysConst.Default;
    static readonly WheelFriction WF = WheelFriction.Default;

    static readonly float[] CollR = [1.0f, 1.75f, 1.75f, 1.0f, 1.0f, 0.5f];
    static readonly int[] CollOrder = [0, 1, 2, 5];

    static readonly float[] NormInvM = [3.64f, 13.34f, 3.64f, 5.72f, 5.72f, 4.44f];
    static readonly float[] WheelieInvM = [5.72f, 13.34f, 3.08f, 5.72f, 4.44f, 4.44f];
    static readonly float[] StoppieInvM = [5.72f, 8.0f, 3.64f, 4.44f, 5.72f, 4.44f];

    bool _disposed;
    Level? _terrain;
    BikeInput _input;
    int _frameNum;
    float _accumDt, _crashTimer, _tiltZ;
    float _engineTorque, _tiltAngle;
    bool _isFalling;

    readonly PtState[] _cur = new PtState[6];
    readonly PtState[] _new = new PtState[6];
    readonly PtState[] _k1 = new PtState[6];
    readonly PtState[] _k2 = new PtState[6];
    readonly PtState[] _tmp = new PtState[6];
    readonly PtState[] _vis = new PtState[6];

    readonly BodyDef[] _bods = new BodyDef[6];
    readonly SpringCfg[] _spr = new SpringCfg[10];
    readonly float[] _curInvM = new float[6];

    bool _leanL, _leanR, _braking;
    int _collIdx;
    float _collNx, _collNy;
    bool _headHit;

    public BikeCfg Cfg { get; private set; } = BikeCfg.Get(BikeType.Standard);
    public BikeType Type { get; private set; } = BikeType.Standard;
    public BikeState State { get; private set; } = BikeState.Grounded;

    public Vector2 Pos => _cur[0].Pos * P.Scale;
    public Vector2 Vel => _cur[0].Vel * P.Scale;
    public float Ang { get; private set; }
    public float AngVel { get; private set; }
    public float Speed => FastLen(Vel);

    public Vector2 FwPos => _cur[1].Pos * P.Scale;
    public Vector2 RwPos => _cur[2].Pos * P.Scale;
    public float FwRot { get; private set; }
    public float RwRot { get; private set; }

    public bool FwGnd { get; private set; }
    public bool RwGnd { get; private set; }
    public bool InAir => !FwGnd && !RwGnd;
    public bool IsGrounded => FwGnd || RwGnd;
    public bool Crashed => (State & BikeState.Crashed) != 0;
    public bool FinishReached { get; private set; }

    public float FrontSusp { get; private set; }
    public float RearSusp { get; private set; }

    public event BikeStateChanged? OnStateChanged;
    public event BikeEvent? OnCrash;
    public event LandEvent? OnHardLand;

    public BikePhysics(BikeType type)
    {
        Type = type;
        Cfg = BikeCfg.Get(type);
        Build();
    }

    public void SetTerrain(Level level) => _terrain = level;

    public void SetInput(float thr, float brk, float lean)
    {
        float finalThr = thr > 0f ? 1f : 0f;
        float finalBrk = brk > 0f ? 1f : 0f;

        if (finalBrk > 0f)
            finalThr = 0f;

        float finalLean = lean < 0f ? -1f : (lean > 0f ? 1f : 0f);

        _input = new BikeInput(finalThr, finalBrk, finalLean);
    }

    public void SetType(BikeType t)
    {
        Type = t;
        Cfg = BikeCfg.Get(t);
        Build();
    }

    public void Reset(float startX)
    {
        Build();
        Place(startX * P.InvScale);
        (_input, _accumDt, _crashTimer, _tiltAngle, _tiltZ, _frameNum) =
            (default, 0f, 0f, 0f, 0.5f, 0);
        (Ang, AngVel, FwRot, RwRot) = (0f, 0f, 0f, 0f);
        (FwGnd, RwGnd, FrontSusp, RearSusp) = (false, false, 0f, 0f);
        (_leanL, _leanR, _braking, _isFalling) = (false, false, false, false);
        (_collIdx, _collNx, _collNy, _headHit) = (-1, 0f, -1f, false);
        (_engineTorque, FinishReached) = (0f, false);
        State = BikeState.Grounded;
    }

    public void Update(float dt)
    {
        if (_disposed || Crashed || dt <= 0f)
            return;

        _accumDt += Min(dt, 0.1f);
        while (_accumDt >= P.FixedDt)
        {
            Step(P.FixedDt);
            _accumDt -= P.FixedDt;
            if (Crashed)
                break;
        }

        for (int i = 0; i < 6; i++)
            _vis[i] = _cur[i];
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        (OnStateChanged, OnCrash, OnHardLand) = (null, null, null);
    }
}

public sealed partial class BikePhysics
{
    struct PtState
    {
        public Vector2 Pos, Vel, Force;
        public float AngVel, Torque, Angle;
    }

    struct BodyDef
    {
        public float R, Inertia;
        public bool IsWheel, IsHead;
    }

    void Build()
    {
        float wheelI = 0.33f;

        for (int i = 0; i < 6; i++)
        {
            _curInvM[i] = NormInvM[i];
            _bods[i] = new()
            {
                R = CollR[i],
                Inertia = (i == 2) ? wheelI : 0f,
                IsWheel = i is 1 or 2,
                IsHead = i == 5
            };
        }

        RebuildSprings();
    }

    void RebuildSprings()
    {
        float k = Cfg.SpringK, d = Cfg.SpringDamp;

        _spr[0] = new(0, 1, P.RestWheel, k, d);
        _spr[1] = new(0, 2, P.RestWheel, k, d);
        _spr[2] = new(0, 3, P.RestUpper, k, d);
        _spr[3] = new(0, 4, P.RestUpper, k, d);
        _spr[4] = new(3, 4, P.RestUpperSpan, k, d);
        _spr[5] = new(1, 3, P.RestWheelUpper, k * 0.1f, d * 0.7f);
        _spr[6] = new(2, 4, P.RestWheelUpper, k * 0.1f, d);
        _spr[7] = new(5, 4, P.RestHeadUpper, k * 1.1f, d);
        _spr[8] = new(5, 3, P.RestHeadUpper, k * 1.1f, d);
        _spr[9] = new(5, 0, P.RestHeadFrame, k * 1.1f, d);
    }

    void Place(float x)
    {
        float gy = _terrain != null
            ? _terrain.GetGroundYAtX(x * P.Scale) * P.InvScale
            : 80f;
        float r = Cfg.WheelR;
        float baseY = gy - r - 0.5f;

        _cur[0] = new() { Pos = new(x, baseY) };
        _cur[1] = new() { Pos = new(x + P.WheelBase, baseY) };
        _cur[2] = new() { Pos = new(x - P.WheelBase, baseY) };
        _cur[3] = new() { Pos = new(x + P.UpperOffsetX, baseY - P.UpperOffsetY) };
        _cur[4] = new() { Pos = new(x - P.UpperOffsetX, baseY - P.UpperOffsetY) };
        _cur[5] = new() { Pos = new(x, baseY - P.HeadOffsetY) };

        for (int i = 0; i < 6; i++)
        {
            _cur[i].Vel = Vector2.Zero;
            _cur[i].AngVel = 0f;
            _cur[i].Angle = 0f;
            _new[i] = _cur[i];
            _vis[i] = _cur[i];
        }
    }
}

public sealed partial class BikePhysics
{
    void Step(float dt)
    {
        _frameNum++;
        float var1 = _cur[0].Vel.Y;
        bool var2 = InAir;

        _leanL = _input.Lean < -0.05f;
        _leanR = _input.Lean > 0.05f;
        _braking = _input.Brake > 0.01f;

#if DEBUG
        DbgClrFrame();
#endif

        UpdateInvMassAndLean();

        // IMPORTANT (change):
        // Removed terrain.UpdateVisible() from Step().
        // We update visible segment window only in ChkColl() (J2ME-style),
        // so it cannot be "shrunk" by a second call with a different margin.

        _headHit = false;
        int var6 = StepCollision(dt);

        if (_headHit || var6 < 0)
        {
            DoCrash();
#if DEBUG
            DbgLog(dt);
#endif
            return;
        }

        FwRot = _cur[1].Angle;
        RwRot = _cur[2].Angle;

        UpdAngle();
        UpdSusp();

        if (var2 && !InAir && var1 > 3f)
            OnHardLand?.Invoke(var1 * P.Scale);

        if (ChkCrash(dt))
        {
            DoCrash();
#if DEBUG
            DbgLog(dt);
#endif
            return;
        }

        UpdState();

#if DEBUG
        DbgLog(dt);
#endif
    }

    int StepCollision(float dt)
    {
        const float MinSlice = 65f / 65536f;

        float proc = 0f;
        float targ = dt;

        while (proc < dt)
        {
            for (int i = 0; i < 6; i++)
                _new[i] = _cur[i];

            HeunJ2ME(targ - proc);

            if (_terrain != null && !FinishReached)
            {
                float fwX = _new[1].Pos.X * P.Scale;
                float rwX = _new[2].Pos.X * P.Scale;

                if (fwX > _terrain.FinishPoint.X || rwX > _terrain.FinishPoint.X)
                {
                    FinishReached = true;
                    targ = (proc + targ) * 0.5f;
                    continue;
                }
            }

            int coll = ChkColl();

            if (_headHit)
                return -1;

            if (coll == 0)
            {
                targ = (proc + targ) * 0.5f;
                if (Abs(targ - proc) < MinSlice)
                    return -1;
                continue;
            }

            if (coll == 1)
            {
                while (coll == 1)
                {
                    Resolve(_collIdx);
                    coll = ChkColl();

                    if (_headHit)
                        return -1;

                    if (coll == 0)
                        return -1;
                }
            }

            for (int i = 0; i < 6; i++)
                _cur[i] = _new[i];

            proc = targ;
            targ = dt;
        }

        float dx = _cur[1].Pos.X - _cur[2].Pos.X;
        float dy = _cur[1].Pos.Y - _cur[2].Pos.Y;
        float d2 = dx * dx + dy * dy;

        return d2 < P.MinAxleDist2 || d2 > P.MaxAxleDist2 ? -1 : 0;
    }

    void HeunJ2ME(float dt)
    {
        Forces(_cur);
        Integ(_cur, _k1, dt);

        for (int var1 = 0; var1 < 6; var1++)
        {
            _tmp[var1].Pos = _cur[var1].Pos + _k1[var1].Pos * 0.5f;
            _tmp[var1].Vel = _cur[var1].Vel + _k1[var1].Vel * 0.5f;
            _tmp[var1].AngVel = _cur[var1].AngVel;
            _tmp[var1].Angle = _cur[var1].Angle;
        }

        Forces(_tmp);
        Integ(_tmp, _k2, dt * 0.5f);

        for (int var2 = 0; var2 < 6; var2++)
        {
            _new[var2].Pos = _cur[var2].Pos + _k1[var2].Pos * 0.5f + _k2[var2].Pos * 0.5f;
            _new[var2].Vel = _cur[var2].Vel + _k1[var2].Vel * 0.5f + _k2[var2].Vel * 0.5f;
            _new[var2].AngVel = _cur[var2].AngVel;
            _new[var2].Angle = _cur[var2].Angle;
        }

        for (int var3 = 1; var3 <= 2; var3++)
        {
            _new[var3].Angle = _cur[var3].Angle + _cur[var3].AngVel * dt;
            _new[var3].AngVel = _cur[var3].AngVel + _bods[var3].Inertia * _cur[var3].Torque * dt;
        }
    }

    void Integ(PtState[] var1, PtState[] var2, float var3)
    {
        for (int var4 = 0; var4 < 6; var4++)
        {
            float var5 = var3 * _curInvM[var4];
            var2[var4].Pos = var1[var4].Vel * var3;
            var2[var4].Vel = var1[var4].Force * var5;
            var2[var4].AngVel = 0f;
        }
    }

    void UpdateInvMassAndLean()
    {
        if (Crashed)
            return;

        float var1 = _cur[1].Pos.X - _cur[2].Pos.X;
        float var2 = _cur[1].Pos.Y - _cur[2].Pos.Y;
        float var3 = FastDist(var1, var2);

        if (var3 < 0.01f)
            return;

        float var4 = var1 / var3;
        float var5 = var2 / var3;

        if (_braking)
        {
            _engineTorque = 0f;

            float var6 = 1f - WF.BrakeDamp;
            _cur[1].AngVel *= var6;
            _cur[2].AngVel *= var6;

            if (_cur[1].AngVel < 0.1f)
                _cur[1].AngVel = 0f;
            if (_cur[2].AngVel < 0.1f)
                _cur[2].AngVel = 0f;
        }
        else if (_input.Throttle > 0.01f)
        {
            if (_engineTorque < Cfg.MaxEngineTorque)
            {
                _engineTorque += Cfg.TorqueAccel;
                if (_engineTorque > Cfg.MaxEngineTorque)
                    _engineTorque = Cfg.MaxEngineTorque;
            }
        }

        float[] var7 = NormInvM;
        if (_leanL)
            var7 = WheelieInvM;
        else if (_leanR)
            var7 = StoppieInvM;

        for (int var8 = 0; var8 < 6; var8++)
            _curInvM[var8] = var7[var8];

        if (_leanL || _leanR)
        {
            float var9 = -var5;

            if (_leanL && _tiltAngle > -Cfg.MaxTilt)
            {
                float var10 = 1f;
                if (_tiltAngle < 0f)
                {
                    var10 = (Cfg.MaxTilt - Abs(_tiltAngle)) / Cfg.MaxTilt;
                }

                float var11 = Cfg.LeanForce * var10;
                float var12 = var9 * var11;
                float var13 = var4 * var11;
                float var14 = var4 * var11;
                float var15 = var5 * var11;

                if (_tiltZ > 0.5f)
                    _tiltZ = _tiltZ - 0.025f >= 0f ? _tiltZ - 0.025f : 0f;
                else
                    _tiltZ = _tiltZ - 0.05f >= 0f ? _tiltZ - 0.05f : 0f;

                _cur[4].Vel.X -= var12;
                _cur[4].Vel.Y -= var13;
                _cur[3].Vel.X += var12;
                _cur[3].Vel.Y += var13;
                _cur[5].Vel.X -= var14;
                _cur[5].Vel.Y -= var15;
            }

            if (_leanR && _tiltAngle < Cfg.MaxTilt)
            {
                float var10 = 1f;
                if (_tiltAngle > 0f)
                {
                    var10 = (Cfg.MaxTilt - _tiltAngle) / Cfg.MaxTilt;
                }

                float var11 = Cfg.LeanForce * var10;
                float var12 = var9 * var11;
                float var13 = var4 * var11;
                float var14 = var4 * var11;
                float var15 = var5 * var11;

                if (_tiltZ > 0.5f)
                    _tiltZ = _tiltZ + 0.025f <= 1f ? _tiltZ + 0.025f : 1f;
                else
                    _tiltZ = _tiltZ + 0.05f <= 1f ? _tiltZ + 0.05f : 1f;

                _cur[4].Vel.X += var12;
                _cur[4].Vel.Y += var13;
                _cur[3].Vel.X -= var12;
                _cur[3].Vel.Y -= var13;
                _cur[5].Vel.X += var14;
                _cur[5].Vel.Y += var15;
            }

            float var16 = _cur[2].Pos.Y - _cur[0].Pos.Y;
            float var17 = _cur[2].Vel.Y - _cur[0].Vel.Y;
            float var18 = FastDist(_cur[2].Vel.X - _cur[0].Vel.X, var17);

            int var19 = var16 < 0f ? -1 : 1;
            int var20 = var17 < 0f ? -1 : 1;

            _tiltAngle = var19 * var20 > 0 ? var18 : -var18;
            return;
        }

        if (_tiltZ < 0.4f)
        {
            _tiltZ += 0.05f;
        }
        else if (_tiltZ > 0.6f)
        {
            _tiltZ -= 0.05f;
        }
        else
        {
            _tiltZ = 0.5f;
        }

        float var21 = _cur[2].Pos.Y - _cur[0].Pos.Y;
        float var22 = _cur[2].Vel.Y - _cur[0].Vel.Y;
        float var23 = FastDist(_cur[2].Vel.X - _cur[0].Vel.X, var22);

        int var24 = var21 < 0f ? -1 : 1;
        int var25 = var22 < 0f ? -1 : 1;

        _tiltAngle = var24 * var25 > 0 ? var23 : -var23;
    }
}

public sealed partial class BikePhysics
{
    void Forces(PtState[] st)
    {
        for (int var1 = 0; var1 < 6; var1++)
        {
            st[var1].Force = new Vector2(0f, P.Gravity / _curInvM[var1]);
            st[var1].Torque = 0f;
        }

        ApplySprings(st, false);

        _engineTorque *= (1f - P.TorqueDamp);
        st[2].Torque = _engineTorque;

        if (st[2].AngVel > Cfg.MaxWheelOmega)
            st[2].AngVel = Cfg.MaxWheelOmega;
        if (st[2].AngVel < -Cfg.MaxWheelOmega)
            st[2].AngVel = -Cfg.MaxWheelOmega;

        float var2 = 0f, var3 = 0f;
        for (int var4 = 0; var4 < 6; var4++)
        {
            var2 += st[var4].Vel.X;
            var3 += st[var4].Vel.Y;
        }
        var2 *= (1f / 6f);
        var3 *= (1f / 6f);

        for (int var5 = 0; var5 < 6; var5++)
        {
            float var6 = st[var5].Vel.X - var2;
            float var7 = st[var5].Vel.Y - var3;
            float var8 = FastDist(var6, var7);

            if (var8 > P.MaxRelVel)
            {
                float var9 = var6 / var8;
                float var10 = var7 / var8;
                st[var5].Vel.X -= var9;
                st[var5].Vel.Y -= var10;
            }
        }
    }

    void ApplySprings(PtState[] st, bool var1)
    {
        if (var1 || !Crashed)
        {
            Spring(ref st[0], ref st[2], 1, 1f);
            Spring(ref st[0], ref st[1], 0, 1f);
            Spring(ref st[2], ref st[4], 6, 2f);
            Spring(ref st[1], ref st[3], 5, 2f);
        }

        Spring(ref st[0], ref st[3], 2, 1f);
        Spring(ref st[0], ref st[4], 3, 1f);
        Spring(ref st[3], ref st[4], 4, 1f);
        Spring(ref st[5], ref st[3], 8, 1f);
        Spring(ref st[5], ref st[4], 7, 1f);
        Spring(ref st[5], ref st[0], 9, 1f);
    }

    void Spring(ref PtState var1, ref PtState var2, int var3, float var4)
    {
        SpringCfg var5 = _spr[var3];

        float var6 = var1.Pos.X - var2.Pos.X;
        float var7 = var1.Pos.Y - var2.Pos.Y;
        float var8 = FastDist(var6, var7);

        if (var8 < P.SpringMinDist)
            return;

        float var9 = var6 / var8;
        float var10 = var7 / var8;
        float var11 = var8 - var5.RestLen;

        float var12 = var1.Vel.X - var2.Vel.X;
        float var13 = var1.Vel.Y - var2.Vel.Y;
        float var14 = var12 * var9 + var13 * var10;

        float var15 = (var11 * var5.K + var14 * var5.D) * var4;

        var1.Force.X -= var9 * var15;
        var1.Force.Y -= var10 * var15;
        var2.Force.X += var9 * var15;
        var2.Force.Y += var10 * var15;
    }

}

public sealed partial class BikePhysics
{
    int _gndFrame = -1;

    int ChkColl()
    {
        if (_gndFrame != _frameNum)
        {
            _gndFrame = _frameNum;
            FwGnd = RwGnd = false;
        }

        _headHit = false;

        if (_terrain == null)
        {
            _collIdx = -1;
            return 2;
        }

        float var1 = _new[1].Pos.X * P.Scale; // fwX
        float var2 = _new[2].Pos.X * P.Scale; // rwX
        float var3 = _new[5].Pos.X * P.Scale; // headX

        float var4 = var1 < var2 ? var1 : var2; // minX
        float var5 = var1 > var2 ? var1 : var2; // maxX
        if (var3 < var4)
            var4 = var3;
        if (var3 > var5)
            var5 = var3;

        // IMPORTANT: UpdateVisible must cover touch broad-phase radius (ChkPtColl uses touchR)
        // Also add J2ME-like padding: 1.5 world (98304 in Q16)
        float var6 = CollR[1];
        float var7 = P.OuterR(var6) * P.Scale; // outerRpx
        float var8 = P.InnerR(var6) * P.Scale; // innerRpx
        float var9 = var7 + (var7 - var8);     // touchRpx (NOT IN J2ME)
        float var10 = 1.5f * P.Scale;          // J2ME padding
        _terrain.UpdateVisible(var4, var5, var9 + var10);

        float var11 = _new[1].Pos.X - _new[2].Pos.X; // dx
        float var12 = _new[1].Pos.Y - _new[2].Pos.Y; // dy
        float var13 = FastDist(var11, var12);

        float var14 = var13 > 0.01f ? var11 / var13 : 1f; // dirX
        float var15 = var13 > 0.01f ? var12 / var13 : 0f; // dirY

        // IMPORTANT: flipped perpendicular (intentional 180° frame orientation)
        float var16 = var15;
        float var17 = -var14;

        int var18 = 2;

        for (int var19 = 0; var19 < 6; var19++)
        {
            if (var19 == 3 || var19 == 4)
                continue;

            ref PtState var20 = ref _new[var19];

            float var21 = var20.Pos.X * P.Scale;
            float var22 = var20.Pos.Y * P.Scale;

            if (var19 == 0)
            {
                float var23 = P.FrameCollOffset * P.Scale;
                var21 += var16 * var23;
                var22 += var17 * var23;
            }

            float var24 = var20.Vel.X * P.Scale;
            float var25 = var20.Vel.Y * P.Scale;

            float var26 = CollR[var19];
            float var27 = P.OuterR(var26) * P.Scale;
            float var28 = P.InnerR(var26) * P.Scale;

            int var29 = ChkPtColl(var21, var22, var24, var25, var27, var28, out bool touch);

#if DEBUG
            DbgTrackTouch(var19, touch);
#endif

            // IMPORTANT: grounded by touch (geometry), not by return != 2 (depends on v·n)
            if (var19 == 1 && touch)
                FwGnd = true;
            if (var19 == 2 && touch)
                RwGnd = true;

            if (var19 == 5 && var29 != 2)
            {
                _headHit = true;
                _collIdx = 5;
#if DEBUG
                DbgSaveLastColl(5, var29);
#endif
                return -1;
            }

            if (var29 == 0)
            {
                _collIdx = var19;
#if DEBUG
                DbgSaveLastColl(var19, 0);
#endif
                return 0;
            }

            if (var29 == 1)
            {
                _collIdx = var19;
                var18 = 1;
#if DEBUG
                DbgSaveLastColl(var19, 1);
#endif
            }
        }

        if (var18 == 2)
            _collIdx = -1;

        return var18;
    }

    int ChkPtColl(float var1, float var2, float var3, float var4,
                  float var5, float var6, out bool var7)
    {
        var7 = false;

        if (_terrain == null)
            return 2;

        (int var8, int var9) = _terrain.GetVisibleRange();

        float var10 = var5 * var5; // outerR^2
        float var11 = var6 * var6; // innerR^2

        // IMPORTANT: touch hysteresis (NOT IN J2ME). Used only for stable grounded in float.
        float var12 = var5 + (var5 - var6); // touchR
        float var13 = var12 * var12;        // touchR^2

        int var14 = 0;
        float var15 = 0f, var16 = 0f;

        for (int var17 = var8; var17 < var9; var17++)
        {
            _terrain.GetSegmentPoints(var17, out float var18, out float var19, out float var20, out float var21);

            // IMPORTANT: broad-phase uses touchR, must match UpdateVisible margin.
            if (var1 - var12 > var20 || var1 + var12 < var18)
                continue;

            float var22 = var20 - var18;
            float var23 = var21 - var19;
            float var24 = var22 * var22 + var23 * var23;

            if (var24 < 0.0001f)
                continue;

            float var25 = ((var1 - var18) * var22 + (var2 - var19) * var23) / var24;
            if (var25 < 0f)
                var25 = 0f;
            else if (var25 > 1f)
                var25 = 1f;

            float var26 = var18 + var25 * var22;
            float var27 = var19 + var25 * var23;

            float var28 = var1 - var26;
            float var29 = var2 - var27;
            float var30 = var28 * var28 + var29 * var29;

            // IMPORTANT: touch flag is geometry-only (no v·n check)
            if (var30 >= var13)
                continue;
            var7 = true;

            // Touch-only zone: grounded yes, but no resolve/crash
            if (var30 >= var10)
                continue;

            _terrain.GetSegmentNormal(var17, out float var31, out float var32);

            float var33 = var3 * var31 + var4 * var32; // v·n

            if (var30 < var11)
            {
                if (var33 < 0f)
                {
                    _collNx = var31;
                    _collNy = var32;
                    return 0;
                }
                continue;
            }

            if (var33 < 0f)
            {
                var14++;
                var15 += var31;
                var16 += var32;
            }
        }

        if (var14 > 0)
        {
            if (var3 * var15 + var4 * var16 >= 0f)
                return 2;

            _collNx = var15;
            _collNy = var16;
            return 1;
        }

        return 2;
    }

    void Resolve(int var1)
    {
#if DEBUG
        float var2 = _bods[var1].IsWheel ? _new[var1].AngVel : 0f;
#endif

        if (_bods[var1].IsWheel)
            ResolveWheel(ref _new[var1], var1);
        else
            ResolveBody(ref _new[var1]);

#if DEBUG
        float var3 = _bods[var1].IsWheel ? _new[var1].AngVel : 0f;
        DbgTrackResolve(var1, var2, var3);
#endif
    }

    void ResolveWheel(ref PtState var1, int var2)
    {
        float var3 = WF.RotDamp;
        float var4 = WF.SlipFriction;
        float var5 = WF.RollCouple;
        float var6 = WF.TangentDamp;
        float var7 = (var2 == 1) ? WF.BounceFront : WF.BounceRear;

        if (_braking && (var2 == 1 || var2 == 2) && var1.AngVel < 0.1f)
        {
            float var8 = WF.BrakeBase - WF.BrakeP;
            var3 = WF.RotDamp - WF.BrakeP;
            var6 = var8;
            var7 = var8;
        }

        float var9 = P.PushOut;
        var1.Pos.X += _collNx * var9;
        var1.Pos.Y += _collNy * var9;

        float var10 = FastDist(_collNx, _collNy);
        float var11 = var10 > 0.0001f ? _collNx / var10 : 0f;
        float var12 = var10 > 0.0001f ? _collNy / var10 : -1f;

        float var13 = _bods[var2].R;
        if (var13 < 0.1f)
            var13 = 0.1f;

        float var14 = var1.Vel.X;
        float var15 = var1.Vel.Y;

        float var16 = -(var14 * var11 + var15 * var12);
        float var17 = -(var14 * (-var12) + var15 * var11);

        float var18 = var3 * var1.AngVel - var4 * (var17 / var13);
        float var19 = var6 * var17 - var5 * (var1.AngVel * var13);
        float var20 = -(var7 * var16);

        float var21 = (-var19) * (-var12);
        float var22 = (-var19) * var11;
        float var23 = (-var20) * var11;
        float var24 = (-var20) * var12;

        var1.AngVel = var18;
        var1.Vel.X = var21 + var23;
        var1.Vel.Y = var22 + var24;
    }

    void ResolveBody(ref PtState var1)
    {
        float var2 = P.PushOut;

        var1.Pos.X += _collNx * var2;
        var1.Pos.Y += _collNy * var2;

        float var3 = FastDist(_collNx, _collNy);
        float var4 = var3 > 0.0001f ? _collNx / var3 : 0f;
        float var5 = var3 > 0.0001f ? _collNy / var3 : -1f;

        float var6 = var1.Vel.X;
        float var7 = var1.Vel.Y;

        float var8 = -(var6 * var4 + var7 * var5);
        float var9 = -(var6 * (-var5) + var7 * var4);

        float var10 = WF.TangentDamp * var9;
        float var11 = -(WF.BounceRear * var8);

        float var12 = (-var10) * (-var5);
        float var13 = (-var10) * var4;
        float var14 = (-var11) * var4;
        float var15 = (-var11) * var5;

        var1.Vel.X = var12 + var14;
        var1.Vel.Y = var13 + var15;
    }
}

public sealed partial class BikePhysics
{
    void UpdAngle()
    {
        Vector2 ax = _cur[1].Pos - _cur[2].Pos;
        float newAng = Atan2(ax.Y, ax.X);
        AngVel = NormAng(newAng - Ang) / P.FixedDt;
        Ang = newAng;
    }

    void UpdSusp()
    {
        float fd = FastLen(_cur[0].Pos - _cur[1].Pos);
        float rd = FastLen(_cur[0].Pos - _cur[2].Pos);
        FrontSusp = Math.Clamp(Abs(fd - P.RestWheel) / 2f, 0f, 1f);
        RearSusp = Math.Clamp(Abs(rd - P.RestWheel) / 2f, 0f, 1f);
    }

    bool ChkCrash(float dt)
    {
        if (_headHit)
            return true;

        float absAng = Abs(NormAng(Ang));
        if (absAng > P.CrashAngle)
        {
            _crashTimer += dt;
            if (_crashTimer > P.CrashTime)
                return true;
        }
        else
            _crashTimer = Max(0f, _crashTimer - dt * 2f);

        return false;
    }

    void DoCrash()
    {
        BikeState prev = State;
        State |= BikeState.Crashed;
        if (prev != State)
            OnStateChanged?.Invoke(prev, State);
        OnCrash?.Invoke();
    }

    void UpdState()
    {
        if (Crashed)
            return;

        BikeState prev = State;
        BikeState next = IsGrounded ? BikeState.Grounded : BikeState.InAir;

        float ang = NormAng(Ang);
        if (RwGnd && !FwGnd && ang < -P.WheelieAngle)
            next |= BikeState.Wheelie;
        if (FwGnd && !RwGnd && ang > P.StoppieAngle)
            next |= BikeState.Stoppie;

        if (State != next)
        {
            State = next;
            OnStateChanged?.Invoke(prev, next);
        }
    }
}

public sealed partial class BikePhysics
{
    public BikeVisual ComputeVisual()
    {
        Vector2 fr = _vis[0].Pos * P.Scale;
        Vector2 fw = _vis[1].Pos * P.Scale;
        Vector2 rw = _vis[2].Pos * P.Scale;
        Vector2 uf = _vis[3].Pos * P.Scale;
        Vector2 ur = _vis[4].Pos * P.Scale;
        Vector2 hd = _vis[5].Pos * P.Scale;

        return new BikeVisual(
            fr, fw, rw, uf, ur, hd,
            Cfg.WheelR * P.Scale, FwRot, RwRot, _tiltZ);
    }
}

public sealed partial class BikePhysics
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float FastDist(float dx, float dy)
    {
        float ax = dx < 0f ? -dx : dx;
        float ay = dy < 0f ? -dy : dy;
        float max = ay >= ax ? ay : ax;
        float min = ay >= ax ? ax : ay;
        return 0.98339844f * max + 0.43066406f * min;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float FastLen(Vector2 v) => FastDist(v.X, v.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float NormAng(float a)
    {
        while (a > 3.14159265f)
            a -= 6.28318530f;
        while (a < -3.14159265f)
            a += 6.28318530f;
        return a;
    }
}
