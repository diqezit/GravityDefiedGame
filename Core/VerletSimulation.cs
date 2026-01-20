using System.Diagnostics;
using Vector2 = System.Numerics.Vector2;

namespace GravityDefiedGame.Core;

// An attempt to write a self-written engine using
// simulation verlets and approximate logic for implementing basic physics from
// C++ BOX2D—at least in the current mode it works—but due to the complexity
// of further implementation and time,
// I decided to use simulation logic identical to j2me.

public readonly record struct SimCfg(
    int ConstraintIter, float ContactSlop, float SleepVelSq,
    float ImpactThreshold, float CcdK, int SleepFrames)
{
    public static readonly SimCfg Default = new(8, 0.3f, 0.25f, 15f, 0.5f, 30);
}

public readonly record struct ParticleDef(
    float Friction, float Restitution, float Roll, float AngDamp,
    float PacB, float PacC, float PacD, float OptSlip)
{
    public static readonly ParticleDef Body = new(1f, 0.05f, 0f, 0.998f, 0f, 0f, 0f, 0.12f);
    public static readonly ParticleDef Wheel = new(2f, 0.05f, 0.004f, 0.998f, 10f, 1.9f, 1.5f, 0.12f);
}

public readonly record struct MuCfg(float MinK, float BlendK, float MinMu)
{
    public static readonly MuCfg Default = new(0.7f, 0.8f, 0.1f);
}

public struct VerletParticle
{
    public Vector2 Position, PrevPosition, Acc;
    public float InvMass, Radius, AngVel;
    public float Friction, Restitution, Roll;
    public float PacB, PacC, PacD, OptSlip;
    public float MaxAngVel, AngDamp;
    public byte Flags;
    public byte SleepCounter;

    public readonly bool IsStatic => InvMass <= 0f;
    public readonly bool HasRotation => (Flags & 1) != 0;
    public readonly bool IsAwake => (Flags & 2) == 0;
    public readonly float Mass => InvMass > 0f ? 1f / InvMass : 0f;

    public readonly float InvInertia => HasRotation && Radius > 0.01f
        ? 2f * InvMass / (Radius * Radius) : 0f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void Wake()
    {
        Flags &= unchecked((byte)~2);
        SleepCounter = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void Sleep() => Flags |= 2;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly float CalcMu(float slip) => CalcMu(slip, MuCfg.Default);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly float CalcMu(float slip, in MuCfg mu)
    {
        if (PacD <= 0f)
            return Friction * mu.MinK;

        float absSlip = MathF.Abs(slip);
        float pac = PacD * MathF.Sin(PacC * MathF.Atan(PacB * absSlip));
        float t = absSlip < OptSlip ? absSlip / OptSlip : 1f;
        return MathF.Max(Friction * mu.BlendK * (1f - t) + pac * t, mu.MinMu);
    }

    public static VerletParticle Create(Vector2 pos, float mass, float radius,
        bool isWheel = false, float maxAV = 0f)
    {
        ParticleDef def = isWheel ? ParticleDef.Wheel : ParticleDef.Body;
        return new()
        {
            Position = pos,
            PrevPosition = pos,
            InvMass = mass > 0f ? 1f / mass : 0f,
            Radius = radius,
            Friction = def.Friction,
            Restitution = def.Restitution,
            Roll = def.Roll,
            MaxAngVel = maxAV,
            AngDamp = def.AngDamp,
            Flags = isWheel ? (byte)1 : (byte)0,
            PacB = def.PacB,
            PacC = def.PacC,
            PacD = def.PacD,
            OptSlip = def.OptSlip,
            SleepCounter = 0
        };
    }
}

struct DistanceConstraint
{
    public int A, B;
    public float Rest, Compliance, Damping;
    public float WarmLambda;
}

public readonly struct GroundInfo(float y, float nx, float ny)
{
    public readonly float Y = y;
    public readonly Vector2 Normal = new(nx, ny);
}

public readonly struct WheelContact(float slip, float mu, bool grounded)
{
    public readonly float SlipRatio = slip;
    public readonly float Mu = mu;
    public readonly bool Grounded = grounded;
}

struct CachedContact
{
    public float NormalImpulse, TangentImpulse;
    public uint FrameId;
}

internal static class PhysMath
{
    public const float Eps = 1e-6f, EpsSq = 1e-12f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Clamp(float v, float lo, float hi) => v < lo ? lo : v > hi ? hi : v;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Clamp01(float v) => v < 0f ? 0f : v > 1f ? 1f : v;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Lerp(float a, float b, float t) => a + (b - a) * t;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2 Norm(Vector2 v)
    {
        float len2 = v.LengthSquared();
        return len2 > EpsSq ? v * (1f / MathF.Sqrt(len2)) : Vector2.Zero;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float NormAngle(float a)
    {
        if (!float.IsFinite(a))
            return 0f;
        a %= MathF.Tau;
        if (a > MathF.PI)
            a -= MathF.Tau;
        else if (a < -MathF.PI)
            a += MathF.Tau;
        return a;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float SlipRatio(float vGnd, float vWhl)
    {
        float d = MathF.Max(MathF.Abs(vGnd), 1f);
        return Clamp((vGnd - vWhl) / d, -1f, 1f);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float CombineMu(float a, float b) => MathF.Sqrt(MathF.Abs(a * b));
}

[Obsolete("Use cuctom ported logic from the GD instead", true)]
public sealed partial class VerletSimulation
{
    const float MinMass = 1e-8f;
    const float WarmDecay = 0.85f;

    readonly VerletParticle[] _p;
    readonly DistanceConstraint[] _c;
    readonly bool[] _touch;
    readonly Vector2[] _touchN;
    readonly float[] _normImp;
    readonly CachedContact[] _cache;
    readonly SimCfg _cfg;

    uint _frameId;

#if DEBUG
    bool _inStep;
    readonly WheelContact[] _whlContact;
#endif

    public Vector2 Gravity = new(0f, 980f);
    public int PCount { get; private set; }
    public int CCount { get; private set; }

    public Func<float, GroundInfo>? GetGroundInfo;
    public Func<float, float>? GetGroundMu;
    public Func<float, float, float, float, float, (bool, float, float, float, float)>? Raycast;
    public Action<int, float>? OnImpact;
    public Action<VerletSimulation, float>? BeforeIntegrate;
    public Action<VerletSimulation, float>? AfterConstraints;

    public VerletSimulation(int maxP = 8, int maxC = 8) : this(maxP, maxC, SimCfg.Default) { }

    public VerletSimulation(int maxP, int maxC, in SimCfg cfg)
    {
        _p = new VerletParticle[maxP];
        _c = new DistanceConstraint[maxC];
        _touch = new bool[maxP];
        _touchN = new Vector2[maxP];
        _normImp = new float[maxP];
        _cache = new CachedContact[maxP];
        _cfg = cfg;
#if DEBUG
        _whlContact = new WheelContact[maxP];
#endif
    }

    #region Creation API (cold path)

    public int AddParticle(in VerletParticle p)
    {
        if (PCount >= _p.Length)
            throw new InvalidOperationException("Particle buffer full");
        _p[PCount] = p;
        return PCount++;
    }

    public int AddConstraint(int a, int b, float rest, float compliance = 0f, float damping = 0f)
    {
        if (CCount >= _c.Length)
            throw new InvalidOperationException("Constraint buffer full");
        if ((uint)a >= (uint)PCount || (uint)b >= (uint)PCount)
            throw new ArgumentOutOfRangeException(nameof(a), "Invalid particle index");
        if (a == b)
            throw new ArgumentException("Self-constraint not allowed", nameof(a));

        _c[CCount] = new()
        {
            A = a,
            B = b,
            Rest = rest,
            Compliance = compliance,
            Damping = damping,
            WarmLambda = 0f
        };
        return CCount++;
    }

    public void ClearAll() => PCount = CCount = 0;

    #endregion

    #region Configuration API (cold path)

    public void SetFrictionParams(int i, float friction, float roll,
        float pacB, float pacC, float pacD, float optSlip)
    {
        if ((uint)i >= (uint)PCount)
            return;
        ref VerletParticle p = ref _p[i];
        p.Friction = friction;
        p.Roll = roll;
        p.PacB = pacB;
        p.PacC = pacC;
        p.PacD = pacD;
        p.OptSlip = optSlip;
    }

    public void SetRestLength(int ci, float rest)
    {
        if ((uint)ci >= (uint)CCount)
            return;
        ref DistanceConstraint c = ref _c[ci];
        c.Rest = rest;
        _p[c.A].Wake();
        _p[c.B].Wake();
    }

    public void Teleport(int i, Vector2 pos)
    {
        if ((uint)i >= (uint)PCount)
            return;
        ref VerletParticle p = ref _p[i];
        p.Position = pos;
        p.PrevPosition = pos;
        p.Acc = Vector2.Zero;
        p.AngVel = 0f;
        p.Wake();
    }

    #endregion

    #region Forces API — Wake (external events)

    public void AddForce(int i, Vector2 f)
    {
#if DEBUG
        Debug.Assert(!_inStep, "Use AddAcceleration inside callbacks, AddForce outside Step");
#endif
        if ((uint)i >= (uint)PCount)
            return;
        ref VerletParticle p = ref _p[i];
        if (p.IsStatic)
            return;
        p.Wake();
        p.Acc += f * p.InvMass;
    }

    public void SetAngularVelocity(int i, float w)
    {
#if DEBUG
        Debug.Assert(!_inStep, "Use AddAngularVelocity inside callbacks, SetAngularVelocity outside Step");
#endif
        if ((uint)i >= (uint)PCount)
            return;
        ref VerletParticle p = ref _p[i];
        if (!p.HasRotation)
            return;
        p.Wake();
        p.AngVel = w;
    }

    public void WakeAll()
    {
        for (int i = 0; i < PCount; i++)
            _p[i].Wake();
    }

    #endregion

    #region Forces API — No Wake (inside callbacks)

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddAcceleration(int i, Vector2 acc)
    {
        if ((uint)i >= (uint)PCount)
            return;
        ref VerletParticle p = ref _p[i];
        if (!p.IsStatic)
            p.Acc += acc;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddAngularVelocity(int i, float dw)
    {
        if ((uint)i >= (uint)PCount)
            return;
        ref VerletParticle p = ref _p[i];
        if (p.HasRotation)
            p.AngVel += dw;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void NudgePosition(int i, Vector2 delta, float prevFactor = 1f)
    {
        if ((uint)i >= (uint)PCount)
            return;
        ref VerletParticle p = ref _p[i];
        if (p.IsStatic)
            return;
        p.Position += delta;
        p.PrevPosition += delta * prevFactor;
    }

    #endregion

    #region Query API — Kinematics

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetPos(int i) => (uint)i < (uint)PCount ? _p[i].Position : Vector2.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetDisplacement(int i) =>
        (uint)i < (uint)PCount ? _p[i].Position - _p[i].PrevPosition : Vector2.Zero;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 GetVelocity(int i, float invDt)
    {
        if ((uint)i >= (uint)PCount)
            return Vector2.Zero;
        ref VerletParticle p = ref _p[i];
        return (p.Position - p.PrevPosition) * invDt;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetAngularVelocity(int i) =>
        (uint)i < (uint)PCount ? _p[i].AngVel : 0f;

    #endregion

    #region Query API — Properties

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetRadius(int i) => (uint)i < (uint)PCount ? _p[i].Radius : 0f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetMass(int i) => (uint)i < (uint)PCount ? _p[i].Mass : 0f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetInvMass(int i) => (uint)i < (uint)PCount ? _p[i].InvMass : 0f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetInvInertia(int i) => (uint)i < (uint)PCount ? _p[i].InvInertia : 0f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetFriction(int i) => (uint)i < (uint)PCount ? _p[i].Friction : 0f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetOptSlip(int i) => (uint)i < (uint)PCount ? _p[i].OptSlip : 0f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsWheel(int i) => (uint)i < (uint)PCount && _p[i].HasRotation;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsAwake(int i) => (uint)i < (uint)PCount && _p[i].IsAwake;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float CalcMu(int i, float slip) =>
        (uint)i < (uint)PCount ? _p[i].CalcMu(slip) : 0f;

    #endregion

    #region Query API — Contact

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsInContact(int i) => (uint)i < (uint)PCount && _touch[i];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2 ContactNormal(int i) =>
        (uint)i < (uint)PCount ? _touchN[i] : -Vector2.UnitY;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetNormalImpulse(int i) =>
        (uint)i < (uint)PCount ? _normImp[i] : 0f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public WheelContact GetWheelContact(int i)
    {
#if DEBUG
        return (uint)i < (uint)PCount ? _whlContact[i] : default;
#else
        return default;
#endif
    }

    #endregion

    #region Query API — Constraints

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetConstraintLength(int ci)
    {
        if ((uint)ci >= (uint)CCount)
            return 0f;
        ref DistanceConstraint c = ref _c[ci];
        return (_p[c.B].Position - _p[c.A].Position).Length();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float GetConstraintRest(int ci) =>
        (uint)ci < (uint)CCount ? _c[ci].Rest : 0f;

    #endregion

    #region Lifecycle

    public void SyncPrev()
    {
        for (int i = 0; i < PCount; i++)
            _p[i].PrevPosition = _p[i].Position;
    }

    public void Update(float dt, int sub)
    {
        if (dt <= 0f || sub <= 0)
            return;
        float sdt = dt / sub;
        for (int s = 0; s < sub; s++)
            Step(sdt);
    }

    public void PreSettle(int iter, float sdt)
    {
        for (int k = 0; k < iter; k++)
        {
            ClearContact();
            for (int it = 0; it < _cfg.ConstraintIter; it++)
            {
                SolveConstraints(sdt);
                SolveGround();
            }
            SolveGroundVel(sdt);
        }
    }

    #endregion

    #region Internal Step

    void Step(float dt)
    {
        if (dt < PhysMath.Eps)
            return;

#if DEBUG
        _inStep = true;
#endif

        _frameId++;

        ClearContact();
        ApplyGravity();
        BeforeIntegrate?.Invoke(this, dt);
        Integrate(dt);
        ApplyAngDamp();
        SolveCcd();

        for (int it = 0; it < _cfg.ConstraintIter; it++)
        {
            SolveConstraints(dt);
            SolveGround();
        }

        SolveGroundVel(dt);
        UpdateSleep();

#if DEBUG
        _inStep = false;
#endif

        AfterConstraints?.Invoke(this, dt);
    }

    void ClearContact()
    {
        Array.Clear(_touch, 0, PCount);
        Array.Clear(_normImp, 0, PCount);
#if DEBUG
        Array.Clear(_whlContact, 0, PCount);
#endif
    }

    void ApplyGravity()
    {
        for (int i = 0; i < PCount; i++)
        {
            ref VerletParticle p = ref _p[i];
            if (!p.IsStatic && p.IsAwake)
                p.Acc += Gravity;
        }
    }

    void Integrate(float dt)
    {
        float dt2 = dt * dt;
        for (int i = 0; i < PCount; i++)
        {
            ref VerletParticle p = ref _p[i];
            if (p.IsStatic)
            {
                p.Acc = Vector2.Zero;
                continue;
            }

            if (!p.IsAwake)
            {
                p.Acc = Vector2.Zero;
                continue;
            }

            Vector2 d = p.Position - p.PrevPosition;
            p.PrevPosition = p.Position;
            p.Position += d + p.Acc * dt2;
            p.Acc = Vector2.Zero;

#if DEBUG
            // NaN prot
            if (!float.IsFinite(p.Position.X) || !float.IsFinite(p.Position.Y))
            {
                Debug.Fail($"NaN detected in particle {i}, recovering to PrevPosition");
                p.Position = p.PrevPosition;
                p.AngVel = 0f;
            }
#endif
        }
    }

    void ApplyAngDamp()
    {
        for (int i = 0; i < PCount; i++)
        {
            ref VerletParticle p = ref _p[i];
            if (p.HasRotation && p.AngDamp < 1f && p.IsAwake)
                p.AngVel *= p.AngDamp;
        }
    }

    void SolveCcd()
    {
        if (Raycast == null)
            return;

        for (int i = 0; i < PCount; i++)
        {
            ref VerletParticle p = ref _p[i];
            if (p.IsStatic || !p.IsAwake)
                continue;

            Vector2 move = p.Position - p.PrevPosition;
            float thresh = p.Radius * _cfg.CcdK;
            if (move.LengthSquared() <= thresh * thresh)
                continue;

            (bool hit, float hx, float hy, float nx, float ny) = Raycast(
                p.PrevPosition.X, p.PrevPosition.Y, p.Position.X, p.Position.Y, p.Radius);
            if (!hit)
                continue;

            Vector2 n = PhysMath.Norm(new Vector2(nx, ny));
            if (n == Vector2.Zero)
                continue;

            Vector2 hitPos = new(hx, hy);
            Vector2 delta = hitPos - p.Position;
            p.Position = hitPos;
            p.PrevPosition += delta;
            _touch[i] = true;
            _touchN[i] = n;
        }
    }

    void SolveConstraints(float dt)
    {
        float invDt = 1f / dt, invDt2 = invDt * invDt;

        for (int i = 0; i < CCount; i++)
        {
            ref DistanceConstraint cn = ref _c[i];
            ref VerletParticle pa = ref _p[cn.A];
            ref VerletParticle pb = ref _p[cn.B];

            if (!pa.IsAwake && !pb.IsAwake)
                continue;

            float wSum = pa.InvMass + pb.InvMass;
            if (wSum < MinMass)
                continue;

            Vector2 d = pb.Position - pa.Position;
            float distSq = d.LengthSquared();
            if (distSq < PhysMath.EpsSq)
                continue;

            float dist = MathF.Sqrt(distSq);
            Vector2 n = d / dist;
            float C = dist - cn.Rest;

            if (MathF.Abs(cn.WarmLambda) > PhysMath.Eps)
            {
                Vector2 warmCorr = n * cn.WarmLambda * 0.5f;
                if (!pa.IsStatic)
                    pa.Position -= warmCorr * pa.InvMass;
                if (!pb.IsStatic)
                    pb.Position += warmCorr * pb.InvMass;
            }

            float alpha = cn.Compliance * invDt2;
            Vector2 relVel = (pb.Position - pb.PrevPosition - pa.Position + pa.PrevPosition) * invDt;
            float dCdt = Vector2.Dot(n, relVel);
            float denom = wSum + alpha + cn.Damping * invDt;
            if (denom < PhysMath.Eps)
                continue;

            float lambda = (-C - cn.Damping * dCdt) / denom;
            Vector2 corr = n * lambda;

            if (!pa.IsStatic)
                pa.Position -= corr * pa.InvMass;
            if (!pb.IsStatic)
                pb.Position += corr * pb.InvMass;

            cn.WarmLambda = lambda * WarmDecay;
        }
    }

    void SolveGround()
    {
        if (GetGroundInfo == null)
            return;

        for (int i = 0; i < PCount; i++)
        {
            ref VerletParticle p = ref _p[i];
            if (p.IsStatic || !p.IsAwake)
                continue;

            GroundInfo g = GetGroundInfo(p.Position.X);
            Vector2 n = PhysMath.Norm(g.Normal);
            if (n == Vector2.Zero)
                continue;

            float signedDist = Vector2.Dot(p.Position - new Vector2(p.Position.X, g.Y), n);
            float pen = p.Radius - signedDist;

            if (pen > 0f)
            {
                Vector2 corr = n * pen;
                p.Position += corr;
                p.PrevPosition += corr;
                _touch[i] = true;
                _touchN[i] = n;
            }
            else if (pen > -_cfg.ContactSlop)
            {
                _touch[i] = true;
                _touchN[i] = n;
            }
        }
    }

    void SolveGroundVel(float dt)
    {
        if (GetGroundInfo == null)
            return;
        float invDt = 1f / dt;

        for (int i = 0; i < PCount; i++)
        {
            if (!_touch[i])
                continue;
            ref VerletParticle p = ref _p[i];
            if (p.IsStatic || !p.IsAwake)
                continue;

            SolveFriction(ref p, i, _touchN[i], invDt, dt);
        }
    }

    void SolveFriction(ref VerletParticle p, int idx, Vector2 n, float invDt, float dt)
    {
        float m = p.Mass;
        if (m < MinMass)
            return;

        Vector2 v = (p.Position - p.PrevPosition) * invDt;
        Vector2 t = new(-n.Y, n.X);
        float vn = Vector2.Dot(v, n);

        ref CachedContact cache = ref _cache[idx];
        bool validCache = cache.FrameId == _frameId - 1;

        float Jn = 0f;
        if (vn < 0f)
        {
            float dvn = -(1f + p.Restitution) * vn;
            v += n * dvn;
            Jn = m * MathF.Abs(dvn);

            float impact = m * MathF.Abs(vn);
            if (impact > _cfg.ImpactThreshold)
            {
                OnImpact?.Invoke(idx, impact);
                p.Wake();
            }
        }

        float gn = MathF.Abs(Vector2.Dot(Gravity, n));
        Jn = MathF.Max(Jn, m * gn * dt);
        _normImp[idx] = Jn;

        float surfMu = GetGroundMu?.Invoke(p.Position.X) ?? 1f;
        float vGnd = Vector2.Dot(v, t);
        float vWhl = p.HasRotation ? p.AngVel * p.Radius : 0f;
        float slip = PhysMath.SlipRatio(vGnd, vWhl);

        float mu = PhysMath.CombineMu(p.CalcMu(slip), surfMu);
        float JtMax = mu * Jn;

        float rotTerm = p.HasRotation ? p.Radius * p.Radius * p.InvInertia : 0f;
        float mEff = 1f / (p.InvMass + rotTerm);
        float vSlip = vGnd - vWhl;

        float warmTangent = validCache ? cache.TangentImpulse * 0.3f : 0f;
        float Jt = PhysMath.Clamp(mEff * vSlip + warmTangent, -JtMax, JtMax);

        v -= t * (Jt * p.InvMass);
        if (p.HasRotation)
            p.AngVel += Jt * p.Radius * p.InvInertia;

        if (p.Roll > 0f && Jn > PhysMath.Eps)
        {
            float dv = Jn * p.Roll * p.InvMass;
            float vt = Vector2.Dot(v, t);

            if (MathF.Abs(vt) > dv)
                v -= t * (MathF.Sign(vt) * dv);
            else
                v -= t * vt;

            if (p.HasRotation)
            {
                float dw = Jn * p.Roll * p.Radius * p.InvInertia;
                p.AngVel = MathF.Abs(p.AngVel) > dw
                    ? p.AngVel - MathF.Sign(p.AngVel) * dw
                    : 0f;
            }
        }

        if (p.HasRotation && p.MaxAngVel > 0f)
            p.AngVel = PhysMath.Clamp(p.AngVel, -p.MaxAngVel, p.MaxAngVel);

        if (v.LengthSquared() < _cfg.SleepVelSq)
            v = Vector2.Zero;

        cache.NormalImpulse = Jn;
        cache.TangentImpulse = Jt;
        cache.FrameId = _frameId;

#if DEBUG
        _whlContact[idx] = new WheelContact(slip, mu, true);
#endif
        p.PrevPosition = p.Position - v * dt;
    }

    void UpdateSleep()
    {
        for (int i = 0; i < PCount; i++)
        {
            ref VerletParticle p = ref _p[i];
            if (p.IsStatic)
                continue;
            if (!p.IsAwake)
                continue;

            Vector2 v = p.Position - p.PrevPosition;
            float velSq = v.LengthSquared() + p.AngVel * p.AngVel * p.Radius * p.Radius;

            if (velSq < _cfg.SleepVelSq && _touch[i])
            {
                p.SleepCounter++;
                if (p.SleepCounter > _cfg.SleepFrames)
                    p.Sleep();
            }
            else
                p.SleepCounter = 0;
        }

        for (int i = 0; i < CCount; i++)
        {
            ref DistanceConstraint cn = ref _c[i];
            ref VerletParticle pa = ref _p[cn.A];
            ref VerletParticle pb = ref _p[cn.B];

            if (pa.IsAwake && !pb.IsAwake && !pb.IsStatic)
                pb.Wake();
            else if (pb.IsAwake && !pa.IsAwake && !pa.IsStatic)
                pa.Wake();
        }
    }

    #endregion
}
