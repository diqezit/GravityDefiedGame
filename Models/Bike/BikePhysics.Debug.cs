using System.Diagnostics;
using System.Text;
using static System.MathF;
using static GravityDefiedGame.Core.GamePlay;
using static GravityDefiedGame.Models.Bike.BikePhysics;
using Cam3D = GravityDefiedGame.Views.Camera3D;

#if DEBUG
using DebugMask = GravityDefiedGame.Models.Bike.BikePhysics.DebugMask;
#endif

namespace GravityDefiedGame.Models.Bike;

#if DEBUG

public readonly struct DebugCfg
{
    public readonly int WinN;
    public readonly float MinDiv, MinSprDist, WarnOmegaK, R2D;
    public readonly float Pick, VelK, LeanK, NormL, SprK;
    public readonly float SlipWarn, SlipCrit, ESpike, Desync;
    public readonly float MinRestLen, CritStrain, WarnStrain;

    public DebugCfg()
    {
        WinN = 30;
        R2D = 57.2958f;
        (MinDiv, MinSprDist, WarnOmegaK) = (0.01f, 0.0001f, 0.9f);
        (Pick, VelK, LeanK, NormL, SprK) = (22f, 0.08f, 0.05f, 2f, 0.002f);
        (SlipWarn, SlipCrit, ESpike, Desync) = (0.3f, 0.6f, 50f, 2f);
        (MinRestLen, CritStrain, WarnStrain) = (0.01f, 0.15f, 0.08f);
    }
}

public readonly struct HudCfg
{
    public readonly int Lbl, V1, V2, V3;
    public readonly float Pad, Gap, ColGap;

    public HudCfg()
    {
        (Lbl, V1, V2, V3) = (5, 8, 7, 6);
        (Pad, Gap, ColGap) = (12f, 10f, 16f);
    }

    HudCfg(float pad, float gap, float colGap)
    {
        (Lbl, V1, V2, V3) = (5, 8, 7, 6);
        (Pad, Gap, ColGap) = (pad, gap, colGap);
    }

    public static HudCfg ForRes(int w) => w switch
    {
        >= 2560 => new(24f, 14f, 18f),
        >= 1920 => new(16f, 12f, 16f),
        _ => new()
    };
}

static class DbgCol
{
    public static readonly Color
        Spring = Color.LimeGreen * 0.6f,
        Warn = Color.Orange * 0.7f,
        Crit = Color.Red * 0.8f,
        Bg = Color.Black * 0.8f,
        Tip = Color.Black * 0.85f,
        TipText = Color.LimeGreen,
        Hover = Color.Red,
        Link = Color.Red * 0.4f,
        Vel = Color.Yellow * 0.4f,
        Lean = Color.Cyan * 0.55f,
        PoseDot = Color.White * 0.5f,
        Skel = Color.White * 0.3f,
        Dsync = Color.Red,
        Node = Color.White,
        Norm = Color.Cyan * 0.7f,
        ZoneOut = Color.Yellow * 0.25f,
        ZoneIn = Color.Red * 0.25f,
        ZoneTouch = Color.Green * 0.2f,
        Eng = Color.Lime * 0.6f,
        EngCrit = Color.Red * 0.8f,
        Rag = Color.OrangeRed * 0.7f,
        Terr = Color.Green * 0.4f;

    public static readonly Color[] Nodes =
        [Color.Yellow, Color.Cyan, Color.Cyan, Color.Orange, Color.Orange, Color.Magenta];

    public static Color NodeAt(int i) =>
        (uint)i < (uint)Nodes.Length ? Nodes[i] : Node;

    public static Color Slip(float s) =>
        s > 0.6f ? Color.Red * 0.5f
        : s > 0.3f ? Color.Yellow * 0.5f
        : Color.Green * 0.5f;

    public static Color Strain(float s, in DebugCfg dc) =>
        s > dc.CritStrain ? Crit
        : s > dc.WarnStrain ? Warn
        : Spring;
}

public readonly record struct HudBlock(string Content, Color Bg, Color Fg);

public struct JitterSnap
{
    public float TiltMin, TiltMax, LeanMin, LeanMax, LeanAMin, LeanAMax;
    public float KeMin, KeMax, SprMax;

    public static JitterSnap Init() => new()
    {
        TiltMin = float.MaxValue,
        TiltMax = float.MinValue,
        LeanMin = float.MaxValue,
        LeanMax = float.MinValue,
        LeanAMin = float.MaxValue,
        LeanAMax = float.MinValue,
        KeMin = float.MaxValue,
        KeMax = float.MinValue
    };

    public void Acc(float tilt, float lean, float leanA, float ke)
    {
        (TiltMin, TiltMax) = (Min(TiltMin, tilt), Max(TiltMax, tilt));
        (LeanMin, LeanMax) = (Min(LeanMin, lean), Max(LeanMax, lean));
        (LeanAMin, LeanAMax) = (Min(LeanAMin, leanA), Max(LeanAMax, leanA));
        (KeMin, KeMax) = (Min(KeMin, ke), Max(KeMax, ke));
    }
}

public struct SpeedSnap
{
    public float MaxSpd, MaxAcc, MaxAV, MaxFwO, MaxRwO, DKE, MaxDKE;
    public int AirF, GndF, OverCnt, WarnCnt;

    public void Acc(float spd, float acc, float av, float fwO, float rwO)
    {
        MaxSpd = Max(MaxSpd, spd);
        MaxAcc = Max(MaxAcc, acc);
        MaxAV = Max(MaxAV, av);
        MaxFwO = Max(MaxFwO, fwO);
        MaxRwO = Max(MaxRwO, rwO);
    }
}

public readonly record struct DampSnap(
    float FwCrit, float RwCrit, float FwRatio, float RwRatio,
    float L1, float L2, float Rho, float Dmp, float Half, float Kick);

public readonly record struct ContactSnap(
    int Res, float VnMin, float VnMax, float DkeAvg, float VnPre, float VnPost);

public struct DebugSnap
{
    public JitterSnap Jitter;
    public SpeedSnap Speed;
    public DampSnap Damp;
    public readonly float[] BodyVx, BodyVy;
    public readonly float[] SprErr, SprRel, SprRelMax;

    public DebugSnap(int bodies, int springs)
    {
        BodyVx = new float[bodies];
        BodyVy = new float[bodies];
        SprErr = new float[springs];
        SprRel = new float[springs];
        SprRelMax = new float[springs];
        (Jitter, Speed, Damp) = (default, default, default);
    }
}

readonly ref struct DbgGfx
{
    const int WheelSegs = 20, CircleSegs = 16, ZoneSegs = 24;

    public readonly Viewport Vp;
    public readonly SpriteBatch Sb;
    readonly bool _is3D;
    readonly float _sc;
    readonly Matrix _cam, _view, _proj;

    public DbgGfx(SpriteBatch sb, Viewport vp, Matrix cam, float sc)
    {
        (Sb, Vp, _sc, _is3D) = (sb, vp, sc, false);
        (_cam, _view, _proj) = (cam, default, default);
    }

    public DbgGfx(SpriteBatch sb, Viewport vp, Matrix view, Matrix proj, float sc)
    {
        (Sb, Vp, _sc, _is3D) = (sb, vp, sc, true);
        (_cam, _view, _proj) = (default, view, proj);
    }

    public Vector2 W(Vector2 w) => _is3D ? W3(w) : Vector2.Transform(w * _sc, _cam);

    public static Vector2 Mid(Vector2 a, Vector2 b) => (a + b) * 0.5f;

    public bool Vis(Vector2 w)
    {
        if (!_is3D)
            return true;
        Vector3 p = Vp.Project(new(w.X * _sc, -w.Y * _sc, 0f), _proj, _view, Matrix.Identity);
        return p.Z is >= 0f and <= 1f;
    }

    public DbgGfx L(Vector2 a, Vector2 b, Color c, float t = 1f)
    { Sb.DrawLine(W(a), W(b), c, t); return this; }

    public DbgGfx Pt(Vector2 p, Color c, float s = 4f)
    { Sb.DrawPoint(W(p), c, s); return this; }

    public DbgGfx Bar(Vector2 pos, float h, Color c, float t = 2f)
    { Sb.DrawLine(pos, pos - new Vector2(0, h), c, t); return this; }

    public DbgGfx Ring(Vector2 ctr, float r, int seg, Color c, float t = 1f)
    {
        if (seg < 3)
            return this;
        float step = Tau / seg;

        for (int i = 0; i < seg; i++)
        {
            float a1 = i * step, a2 = (i + 1) * step;
            Sb.DrawLine(
                W(ctr + new Vector2(Cos(a1), Sin(a1)) * r),
                W(ctr + new Vector2(Cos(a2), Sin(a2)) * r), c, t);
        }
        return this;
    }

    public DbgGfx Wheel(Vector2 pos, float r, float rot, Color c, float t = 1.5f) =>
        Ring(pos, r, WheelSegs, c, t)
            .L(pos, pos + new Vector2(Cos(rot), Sin(rot)) * r, c);

    public DbgGfx Circle(Vector2 screenPos, float r, Color c, float t = 2f)
    { Sb.DrawCircle(screenPos, r, CircleSegs, c, t); return this; }

    public DbgGfx Zones(Vector2 hp, float oR, float iR) =>
        Ring(hp, oR * 2f - iR, ZoneSegs, DbgCol.ZoneTouch)
            .Ring(hp, oR, ZoneSegs, DbgCol.ZoneOut)
            .Ring(hp, iR, ZoneSegs, DbgCol.ZoneIn);

    public DbgGfx Skel(ReadOnlySpan<Vector2> pts, ReadOnlySpan<byte> idx,
        Color line, float lt, Color dot, float ds)
    {
        for (int i = 0; i < idx.Length; i += 2)
            L(pts[idx[i]], pts[idx[i + 1]], line, lt);
        foreach (Vector2 p in pts)
            Pt(p, dot, ds);
        return this;
    }

    Vector2 W3(Vector2 w)
    {
        Vector3 p = Vp.Project(new(w.X * _sc, -w.Y * _sc, 0f), _proj, _view, Matrix.Identity);
        return new(p.X, p.Y);
    }
}

public sealed partial class BikePhysics
{
    struct DebugAcc(int bodies, int springs)
    {
        public JitterSnap Jitter = JitterSnap.Init();
        public SpeedSnap Speed;
        public int N;
        public readonly float[] BodyVx = new float[bodies], BodyVy = new float[bodies];
        public readonly float[] SprErrSum = new float[springs];
        public readonly float[] SprRelSum = new float[springs], SprRelMax = new float[springs];

        public void Reset()
        {
            (Jitter, Speed, N) = (JitterSnap.Init(), default, 0);
            BodyVx.AsSpan().Clear();
            BodyVy.AsSpan().Clear();
            SprErrSum.AsSpan().Clear();
            SprRelSum.AsSpan().Clear();
            SprRelMax.AsSpan().Clear();
        }
    }

    struct ContactAcc
    {
        public float VnMin = float.MaxValue, VnMax = float.MinValue;
        public float DkeSum, VnPreSum, VnPostSum;
        public int ResMax, N;

        public ContactAcc() { }

        public void Acc(in ContactFrame f)
        {
            ResMax = (int)Max(ResMax, f.Resolves);
            if (f.Resolves <= 0)
                return;

            (VnMin, VnMax) = (Min(VnMin, f.VnMin), Max(VnMax, f.VnMax));
            DkeSum += f.DeltaKE;
            VnPreSum += f.VnPre;
            VnPostSum += f.VnPost;
            N++;
        }

        public readonly ContactSnap ToSnap() => N == 0
            ? new(ResMax, 0f, 0f, 0f, 0f, 0f)
            : new(ResMax, VnMin, VnMax, DkeSum / N, VnPreSum / N, VnPostSum / N);
    }

    struct ContactFrame
    {
        public float VnMin = float.MaxValue, VnMax = float.MinValue;
        public float DeltaKE, VnPre, VnPost, KeBefore;
        public int Resolves;

        public ContactFrame() { }

        public void Record(float vn, float nx, float ny, in PtState pt, float keAfter)
        {
            (VnMin, VnMax) = (Min(VnMin, vn), Max(VnMax, vn));
            (VnPre, VnPost) = (vn, -(pt.Vel.X * nx + pt.Vel.Y * ny));
            DeltaKE += keAfter - KeBefore;
            Resolves++;
        }
    }

    readonly ref struct FrameRec
    {
        readonly BikePhysics _bp;
        readonly float _dt, _spd, _ke, _fwO, _rwO;

        internal FrameRec(BikePhysics bp, float dt)
        {
            _bp = bp;
            _dt = dt;
            _spd = bp.Speed;
            _ke = bp.KE();
            _fwO = Abs(bp.Cur[BFw].AngVel);
            _rwO = Abs(bp.Cur[BRw].AngVel);
        }

        public readonly FrameRec Speed() { _bp.RecSpeed(_dt, _spd, _fwO, _rwO); return this; }
        public readonly FrameRec Energy() { _bp.RecEnergy(_dt, _ke); return this; }
        public readonly FrameRec Jitter() { _bp.RecJitter(_ke); return this; }
        public readonly FrameRec Springs() { _bp.RecSprings(); return this; }
        public readonly FrameRec Contact() { _bp._ctAcc.Acc(in _bp._ctFrame); return this; }

        public readonly void Commit()
        {
            if (++_bp._acc.N >= DC.WinN)
                _bp.Flush();
            if (_bp.FrameNum % 60 == 0)
                _bp.AnalyzePhys();
        }
    }

    [Flags]
    public enum DebugMask : byte
    {
        None = 0,
        Basic = 1 << 0, Springs = 1 << 1, Collision = 1 << 2,
        Integrator = 1 << 3, Ragdoll = 1 << 4, Energy = 1 << 5, Terrain = 1 << 6,
        All = Basic | Springs | Collision | Integrator | Ragdoll | Energy | Terrain
    }

    static readonly DebugCfg DC = new();
    static readonly WheelFriction DbgWF = WheelFriction.Default;

    bool _dbgInit;
    DebugRender? _dbgRender;

    DebugAcc _acc;
    DebugSnap _dbg;

    ContactAcc _ctAcc;
    ContactSnap _ct;
    ContactFrame _ctFrame;

    int _dbgIdx;
    float _dbgNx, _dbgNy;

    float _prevSpd, _prevKE;

    public bool DebugNodesEnabled { get; set; }
    public DebugMask DebugView { get; set; } = DebugMask.All;

    internal float AccumDt => _accumDt;
    internal ref readonly DebugSnap Dbg => ref _dbg;
    internal ref readonly ContactSnap Ct => ref _ct;
    internal bool HasCollN => Abs(_dbgNx) > 0.001f || Abs(_dbgNy) > 0.001f;
    internal (int idx, float nx, float ny) CollInfo => (_dbgIdx, _dbgNx, _dbgNy);

    internal float CalcLean() => RawTiltZ / P.TiltCenter - 1f;
    internal float CalcLeanAccel() => Cfg.LeanForce * P.Gravity * CalcLean();
    FrameRec Rec(float dt) => new(this, dt);

    [Conditional("DEBUG")]
    internal void ResetContactStats()
    {
        InitDbg();
        _ctFrame = new();
    }

    [Conditional("DEBUG")]
    internal void RecordKeBefore(int idx)
    {
        InitDbg();
        _ctFrame.KeBefore = BodyKE(idx);
    }

    [Conditional("DEBUG")]
    internal void RecordResolve(int idx, float vn, float nx, float ny, in PtState pt)
    {
        InitDbg();
        _ctFrame.Record(vn, nx, ny, in pt, BodyKE(idx));
    }

    public void DrawDebug(SpriteBatch sb, SpriteFontBase font, GraphicsDevice gd,
        Matrix camTr, Cam3D? cam3D)
    {
        if (!DebugNodesEnabled)
            return;
        InitDbg();
        _dbgRender ??= new(this);
        _dbgRender.Draw(sb, font, gd, camTr, cam3D);
    }

    [Conditional("DEBUG")]
    public void UpdateDebugStats(float dt)
    {
        if (!DebugNodesEnabled || dt <= 0f)
            return;
        InitDbg();
        (_dbgIdx, _dbgNx, _dbgNy) = (_col.CollIdx, _col.CollNx, _col.CollNy);
        Rec(dt).Speed().Energy().Jitter().Springs().Contact().Commit();
    }

    [Conditional("DEBUG")]
    public void ResetDebugStats()
    {
        InitDbg();
        (_prevSpd, _prevKE) = (Speed, KE());
        _acc.Reset();
        (_ctAcc, _ctFrame, _ct) = (new(), new(), default);
        (_dbgIdx, _dbgNx, _dbgNy) = (-1, 0f, 0f);
    }

    void InitDbg()
    {
        if (_dbgInit)
            return;
        _dbgInit = true;
        _acc = new(BodyCount, SpringCount);
        _dbg = new(BodyCount, SpringCount);
        _ctAcc = new();
        (_prevSpd, _prevKE) = (Speed, KE());
    }

    void RecSpeed(float dt, float spd, float fwO, float rwO)
    {
        float omega = Max(fwO, rwO), limit = Cfg.MaxWheelOmega;
        _acc.Speed.Acc(spd, Abs(spd - _prevSpd) / dt, Abs(AngVel), fwO, rwO);
        _prevSpd = spd;

        if (omega > limit)
            _acc.Speed.OverCnt++;
        if (omega > limit * DC.WarnOmegaK)
            _acc.Speed.WarnCnt++;
        if (InAir)
            _acc.Speed.AirF++;
        else
            _acc.Speed.GndF++;
    }

    void RecEnergy(float dt, float ke)
    {
        float dke = (ke - _prevKE) / dt;
        _acc.Speed.DKE = dke;
        _acc.Speed.MaxDKE = Max(_acc.Speed.MaxDKE, Abs(dke));
        _prevKE = ke;
    }

    void RecJitter(float ke)
    {
        _acc.Jitter.Acc(RawTiltZ, CalcLean(), CalcLeanAccel(), ke);

        for (int i = 0; i < BodyCount; i++)
        {
            _acc.BodyVx[i] = Max(_acc.BodyVx[i], Abs(Cur[i].Vel.X));
            _acc.BodyVy[i] = Max(_acc.BodyVy[i], Abs(Cur[i].Vel.Y));
        }

        for (int i = 0; i < SpringCount; i++)
            _acc.Jitter.SprMax = Max(_acc.Jitter.SprMax, Abs(SLen(i) - Spr[i].RestLen));
    }

    void RecSprings()
    {
        for (int i = 0; i < SpringCount; i++)
        {
            (float err, float rv) = SpringMetrics(i);
            _acc.SprErrSum[i] += err;
            _acc.SprRelSum[i] += rv;
            _acc.SprRelMax[i] = Max(_acc.SprRelMax[i], rv);
        }
    }

    (float err, float rv) SpringMetrics(int i)
    {
        ref readonly SpringCfg s = ref Spr[i];
        ref readonly PtState a = ref Cur[s.A];
        ref readonly PtState b = ref Cur[s.B];

        float dist = Vector2.Distance(a.Pos, b.Pos);
        if (dist < DC.MinSprDist)
            return (0f, 0f);

        return (
            Abs(dist - s.RestLen),
            Abs(Vector2.Dot(a.Vel - b.Vel, (a.Pos - b.Pos) / dist)));
    }

    void Flush()
    {
        float inv = 1f / DC.WinN;
        (_dbg.Jitter, _dbg.Speed) = (_acc.Jitter, _acc.Speed);
        _acc.BodyVx.AsSpan().CopyTo(_dbg.BodyVx);
        _acc.BodyVy.AsSpan().CopyTo(_dbg.BodyVy);

        for (int i = 0; i < SpringCount; i++)
        {
            _dbg.SprErr[i] = _acc.SprErrSum[i] * inv;
            _dbg.SprRel[i] = _acc.SprRelSum[i] * inv;
            _dbg.SprRelMax[i] = _acc.SprRelMax[i];
        }

        _dbg.Damp = BuildDampSnap();
        _ct = _ctAcc.ToSnap();
        _acc.Reset();
        _ctAcc = new();
    }

    DampSnap BuildDampSnap()
    {
        (float fwC, float fwR) = DampRatio(BFw, 0);
        (float rwC, float rwR) = DampRatio(BRw, 1);
        ref readonly DampSnap d = ref _dbg.Damp;
        return new(fwC, rwC, fwR, rwR, d.L1, d.L2, d.Rho, d.Dmp, d.Half, d.Kick);
    }

    void AnalyzePhys()
    {
        float r = Cfg.WheelR;
        (float l1, float l2) = Eigenvalues(r);
        float rho = Max(Abs(l1), Abs(l2));
        float dmp = (1f - rho) * 100f;
        float half = rho > DC.MinDiv ? Log(0.5f) / Log(rho) : 999f;
        float kick = DbgWF.RollCouple * r * Cfg.MaxWheelOmega;

        ref readonly DampSnap d = ref _dbg.Damp;
        _dbg.Damp = new(d.FwCrit, d.RwCrit, d.FwRatio, d.RwRatio, l1, l2, rho, dmp, half, kick);
    }

    static (float l1, float l2) Eigenvalues(float r)
    {
        float a11 = DbgWF.RotDamp, a22 = DbgWF.TangentDamp;
        float a12 = -DbgWF.SlipFriction / r, a21 = -DbgWF.RollCouple * r;
        float tr = a11 + a22, det = a11 * a22 - a12 * a21;
        float disc = tr * tr - 4f * det;

        if (disc >= 0f)
        {
            float sq = Sqrt(disc);
            return ((tr + sq) * 0.5f, (tr - sq) * 0.5f);
        }
        float mag = Sqrt(tr * tr + Abs(disc)) * 0.5f;
        return (mag, mag);
    }

    (float crit, float ratio) DampRatio(int w, int s)
    {
        float invM = InvMass[BFrame] + InvMass[w];
        float crit = 2f * Sqrt(Spr[s].K / Max(invM, DC.MinDiv));
        float ratio = Spr[s].D / Max(crit, DC.MinDiv) * 100f;
        return (crit, ratio);
    }

    float BodyKE(int i) =>
        0.5f * Mass[i] * New[i].Vel.LengthSquared();

    internal float KE()
    {
        float ke = 0f;
        for (int i = 0; i < BodyCount; i++)
            ke += BodyKE(i);
        return ke;
    }

    internal float Slip(int i)
    {
        float wR = Abs(Cur[i].AngVel) * Bods[i].R;
        float v = Cur[i].Vel.Length();
        float d = Max(wR, v);
        return d > DC.MinDiv ? Abs(wR - v) / d : 0f;
    }

    internal float SLen(int i) =>
        Vector2.Distance(Cur[Spr[i].A].Pos, Cur[Spr[i].B].Pos);

    internal float GndY(float x) =>
        Terrain!.GetGroundYAtX(x * P.TerrainScale) / P.TerrainScale;

    internal Vector2 GndN(float x)
    {
        const float step = 0.5f;
        float dy = GndY(x + step) - GndY(x);
        float len = Sqrt(step * step + dy * dy);
        return len > P.Epsilon ? new(-dy / len, step / len) : Vector2.UnitY;
    }
}

sealed class DebugOverlay(BikePhysics bp)
{
    static readonly PhysConst P = PhysConst.Default;
    static readonly DebugCfg DC = new();

    static readonly string[] NodeNames = ["Frame", "FW", "RW", "UF", "UR", "Head"];

    static ReadOnlySpan<byte> PoseBones =>
        [0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 4, 6, 6, 7];

    static ReadOnlySpan<byte> RagBones =>
        [0, 1, 1, 2, 2, 3, 1, 4, 4, 7, 0, 6, 6, 5, 0, 4, 2, 4, 1, 6, 0, 7];

    internal int PickNode(Vector2 mouse, ref DbgGfx g)
    {
        int best = -1;
        float bestSq = DC.Pick * DC.Pick;

        for (int i = 0; i < BodyCount; i++)
        {
            Vector2 s = g.W(bp.Cur[i].Pos);
            g.Sb.DrawPoint(s, DbgCol.NodeAt(i), bp.Bods[i].IsWheel ? 6f : 4f);

            float sq = (s - mouse).LengthSquared();
            if (sq < bestSq)
            { bestSq = sq; best = i; }
        }
        return best;
    }

    internal void Springs(ref DbgGfx g, bool showForce)
    {
        for (int i = 0; i < SpringCount; i++)
        {
            ref readonly SpringCfg s = ref bp.Spr[i];
            float len = bp.SLen(i);
            float strain = s.RestLen > DC.MinRestLen
                ? Abs(len - s.RestLen) / s.RestLen : 0f;

            Color col = DbgCol.Strain(strain, in DC);
            g.L(bp.Cur[s.A].Pos, bp.Cur[s.B].Pos, col, 1.5f + strain * 3f);

            if (!showForce)
                continue;
            Vector2 mid = g.W(DbgGfx.Mid(bp.Cur[s.A].Pos, bp.Cur[s.B].Pos));
            g.Bar(mid, s.K * Abs(len - s.RestLen) * DC.SprK, col);
        }
    }

    internal void Velocities(ref DbgGfx g)
    {
        for (int i = 0; i < BodyCount; i++)
            g.L(bp.Cur[i].Pos, bp.Cur[i].Pos + bp.Cur[i].Vel * DC.VelK, DbgCol.Vel);
    }

    internal void LeanForce(ref DbgGfx g)
    {
        float k = bp.CalcLeanAccel() * DC.LeanK;
        if (Abs(k) <= P.Epsilon)
            return;

        Vector2 fw = bp.Cur[BFw].Pos, rw = bp.Cur[BRw].Pos;
        g.L(fw, fw + new Vector2(0f, k), DbgCol.Lean, 2f)
         .L(rw, rw + new Vector2(0f, -k), DbgCol.Lean, 2f);
    }

    internal void PoseViz(ref DbgGfx g)
    {
        Pose p = Poses.Build(bp.Cur[BFrame].Pos, bp.Cur[BUF].Pos, bp.Cur[BUR].Pos, bp.TiltZ);
        ReadOnlySpan<Vector2> pts = [p.Ft, p.Kn, p.Hip, p.Sh, p.Elb, p.Hnd, p.Nk, p.Hd];
        g.Skel(pts, PoseBones, DbgCol.Skel, 1f, DbgCol.PoseDot, 3f);

        Vector2 phy = g.W(bp.Cur[BHead].Pos), viz = g.W(pts[7]);
        if (Vector2.Distance(phy, viz) > DC.Desync)
            g.Sb.DrawLine(phy, viz, DbgCol.Dsync, 1.5f);
    }

    internal void Wheels(ref DbgGfx g)
    {
        DrawWheel(BFw, ref g);
        DrawWheel(BRw, ref g);
    }

    internal void Collision(int hover, ref DbgGfx g)
    {
        DrawCollisionNormal(ref g);
        if (hover >= 0)
            DrawZones(hover, ref g);
    }

    internal void Energy(ref DbgGfx g)
    {
        Vector2 ctr = g.W(bp.Cur[BFrame].Pos) + new Vector2(20, 0);
        Color col = Abs(bp.Dbg.Speed.DKE) > DC.ESpike ? DbgCol.EngCrit : DbgCol.Eng;
        g.Bar(ctr, Min(bp.KE() * 0.5f, 80f), col, 3f);

        Vector2 fw = bp.Cur[BFw].Pos, rw = bp.Cur[BRw].Pos;
        float d2 = (fw - rw).LengthSquared();
        Color axle = d2 < P.MinAxleDist2 || d2 > P.MaxAxleDist2 ? DbgCol.Crit : DbgCol.Eng;
        g.L(fw, rw, axle, 1.5f);
    }

    internal void Ragdoll(ref DbgGfx g)
    {
        Pose rp = bp.RiderPose;
        ReadOnlySpan<Vector2> pts = [rp.Hip, rp.Sh, rp.Elb, rp.Hnd, rp.Nk, rp.Ft, rp.Kn, rp.Hd];
        g.Skel(pts, RagBones, DbgCol.Rag, 2f, DbgCol.Rag, 4f);
    }

    internal void Terrain(ref DbgGfx g)
    {
        Level? terrain = bp.Terrain;
        if (terrain is null)
            return;

        (int s, int e) = terrain.GetVisibleRange();
        float isc = 1f / P.TerrainScale;

        for (int i = s; i < e; i++)
        {
            terrain.GetSegmentPoints(i, out float sx, out float sy, out float ex, out float ey);
            Vector2 a = new(sx * isc, sy * isc), b = new(ex * isc, ey * isc);
            if (!g.Vis(a) || !g.Vis(b))
                continue;

            terrain.GetSegmentNormal(i, out float nx, out float ny);
            Vector2 mid = DbgGfx.Mid(a, b);
            g.L(a, b, DbgCol.Terr, 2f)
             .L(mid, mid + new Vector2(nx, ny), DbgCol.Norm * 0.4f);
        }
    }

    internal void Tooltip(SpriteFontBase font, Vector2 mouse, int i,
        StringBuilder sb, ref DbgGfx g)
    {
        BuildTooltipText(i, sb);
        DrawTooltipLinks(i, mouse, font, ref g);
        DrawTooltipBox(font, mouse, sb.ToString(), ref g);
    }

    void DrawWheel(int i, ref DbgGfx g)
    {
        Vector2 pos = bp.Cur[i].Pos;
        float r = bp.Bods[i].R, rot = bp.Cur[i].Angle;
        g.Wheel(pos, r, rot, DbgCol.Slip(bp.Slip(i)));
    }

    void DrawCollisionNormal(ref DbgGfx g)
    {
        (int idx, float nx, float ny) = bp.CollInfo;
        if (!IsValidCollision(idx))
            return;

        Vector2 pos = bp.Cur[idx].Pos;
        g.L(pos, pos + new Vector2(nx, ny) * DC.NormL, DbgCol.Norm, 2.5f)
         .Pt(pos, DbgCol.Norm, 5f);
    }

    void DrawZones(int hover, ref DbgGfx g)
    {
        Vector2 hp = bp.Cur[hover].Pos;
        float oR = P.OuterR(bp.Bods[hover].R), iR = P.InnerR(bp.Bods[hover].R);
        g.Zones(hp, oR, iR);

        if (bp.Terrain is null)
            return;
        Vector2 gp = new(hp.X, bp.GndY(hp.X));
        g.L(gp, gp + bp.GndN(hp.X) * DC.NormL, DbgCol.Norm, 2f)
         .Pt(gp, DbgCol.Norm, 3f);
    }

    void BuildTooltipText(int i, StringBuilder sb)
    {
        ref readonly PtState pt = ref bp.Cur[i];
        string name = (uint)i < (uint)NodeNames.Length ? NodeNames[i] : $"N{i}";

        sb.Clear()
          .Lbl(name, 6).Append('[').Append(i).Append(']').Ln()
          .Lbl("P", 2).Val(pt.Pos.X, 2, 7).Val(pt.Pos.Y, 2, 7).Ln()
          .Lbl("V", 2).Val(pt.Vel.X, 2, 7).Val(pt.Vel.Y, 2, 7).Ln()
          .Lbl("|V|", 4).Val(pt.Vel.Length(), 1, 6)
          .Lbl("F", 2).Val(pt.Force.Length(), 0, 5).Ln()
          .Lbl("R", 2).Val(bp.Bods[i].R, 2, 5)
          .Lbl("M", 2).Val(bp.Mass[i], 3, 5).Ln();
    }

    void DrawTooltipLinks(int i, Vector2 mouse, SpriteFontBase font, ref DbgGfx g)
    {
        Vector2 ns = g.W(bp.Cur[i].Pos);
        float lh = font.MeasureString("A").Y;
        float hitR = Max(6f, lh * 0.6f);

        g.Circle(ns, hitR, DbgCol.Hover)
         .Sb.DrawLine(ns, mouse, DbgCol.Link, 1f);

        for (int j = 0; j < SpringCount; j++)
        {
            ref readonly SpringCfg s = ref bp.Spr[j];
            if (s.A != i && s.B != i)
                continue;
            g.L(bp.Cur[s.A].Pos, bp.Cur[s.B].Pos, Color.White, 2.5f);
        }
    }

    static void DrawTooltipBox(SpriteFontBase font, Vector2 mouse, string text, ref DbgGfx g)
    {
        Vector2 sz = font.MeasureString(text);
        float lh = font.MeasureString("A").Y;
        float pad = Max(6f, lh * 0.6f), off = Max(10f, lh * 0.9f);

        Vector2 pos = new(
            Math.Clamp(mouse.X + off, pad, g.Vp.Width - sz.X - pad),
            Math.Clamp(mouse.Y + off, pad, g.Vp.Height - sz.Y - pad));

        HudDraw.Block(g.Sb, font, pos, text, DbgCol.Tip, DbgCol.TipText, pad);
    }

    bool IsValidCollision(int idx) =>
        (uint)idx < BodyCount && bp.HasCollN;
}

static file class HudDraw
{
    public static void Block(SpriteBatch sb, SpriteFontBase font, Vector2 pos,
        string text, Color bg, Color fg, float pad)
    {
        Vector2 sz = font.MeasureString(text);
        sb.FillRectangle(new Rectangle(
            (int)pos.X, (int)pos.Y,
            (int)(sz.X + pad * 2f), (int)(sz.Y + pad * 2f)), bg);
        font.DrawText(sb, text, pos + new Vector2(pad, pad), fg);
    }

    public static void Col(SpriteFontBase font, ref DbgGfx g,
        IReadOnlyList<HudBlock> blocks, float x, float w,
        ref float y, float gap, float padIn)
    {
        for (int i = 0; i < blocks.Count; i++)
        {
            HudBlock b = blocks[i];
            Vector2 sz = font.MeasureString(b.Content);
            float h = sz.Y + padIn * 2f;

            g.Sb.FillRectangle(
                new Rectangle((int)x, (int)y, (int)w, (int)h), b.Bg);
            font.DrawText(g.Sb, b.Content,
                new Vector2(x + padIn, y + padIn), b.Fg);
            y += h + gap;
        }
    }

    public static void Measure(SpriteFontBase font,
        IReadOnlyList<HudBlock> blocks, out float w, out float h)
    {
        w = h = 0f;
        for (int i = 0; i < blocks.Count; i++)
        {
            Vector2 sz = font.MeasureString(blocks[i].Content);
            if (sz.X > w)
                w = sz.X;
            h += sz.Y;
        }
    }
}

sealed class DebugHud
{
    readonly record struct Section(
        string Title, DebugMask Mask, Action Build, bool Right);

    static readonly DebugCfg DC = new();
    static readonly HudCfg H = new();

    static readonly string[] BodyLbl = ["Frm", "FW ", "RW ", "UF ", "UR ", "Hd "];
    static readonly string[] SprLbl =
        ["FrFW", "FrRW", "FrUF", "FrUR", "UFUR",
         "FWUF", "RWUR", "HdUR", "HdUF", "HdFr"];

    readonly BikePhysics _bp;
    readonly StringBuilder _sb = new(2048);
    readonly List<HudBlock> _left = [], _right = [];
    readonly Section[] _sections;
    int _logTick;

    public DebugHud(BikePhysics bp)
    {
        _bp = bp;
        _sections =
        [
            new("STATE",      DebugMask.Basic,      BuildState,   Right: false),
            new("INPUT",      DebugMask.Basic,      BuildInput,   Right: false),
            new("ENERGY",     DebugMask.Energy,     BuildEnergy,  Right: false),
            new("CONTACT",    DebugMask.Collision,  BuildContact, Right: false),

            new("MOTION",     DebugMask.Basic,      BuildMotion,  Right: true),
            new("JITTER",     DebugMask.Basic,      BuildJitter,  Right: true),
            new("INTEGRATOR", DebugMask.Integrator, BuildIntegr,  Right: true),
            new("SPRINGS",    DebugMask.Springs,    BuildSprings, Right: true),
        ];
    }

    public void Draw(SpriteFontBase font, ref DbgGfx g)
    {
        Rebuild();
        if (_left.Count + _right.Count == 0)
            return;
        Layout(font, ref g);
        Log();
    }

    void Rebuild()
    {
        _left.Clear();
        _right.Clear();
        Color bg = DbgCol.Bg, fg = Color.White;

        for (int i = 0; i < _sections.Length; i++)
        {
            ref readonly Section s = ref _sections[i];
            if ((_bp.DebugView & s.Mask) == 0)
                continue;

            _sb.Clear();
            s.Build();
            var block = new HudBlock($"[ {s.Title} ]\n{_sb}", bg, fg);
            (s.Right ? _right : _left).Add(block);
        }
    }

    void Layout(SpriteFontBase font, ref DbgGfx g)
    {
        var cfg = HudCfg.ForRes(g.Vp.Width);
        float lh = font.MeasureString("A").Y;
        float padIn = Max(6f, lh * 0.4f);

        HudDraw.Measure(font, _left, out float wL, out _);
        HudDraw.Measure(font, _right, out float wR, out _);

        float needL = wL + padIn * 2f;
        float needR = wR + padIn * 2f;
        float vpW = g.Vp.Width;

        float xL = cfg.Pad;
        float xR = vpW - needR - cfg.Pad;
        float freeGap = xR - (xL + needL);

        if (freeGap < cfg.ColGap)
        {
            float colW = Max(needL, needR);
            float y = cfg.Pad;
            HudDraw.Col(font, ref g, _left, cfg.Pad, colW, ref y, cfg.Gap, padIn);
            HudDraw.Col(font, ref g, _right, cfg.Pad, colW, ref y, cfg.Gap, padIn);
            return;
        }

        float yL = cfg.Pad, yR = cfg.Pad;
        HudDraw.Col(font, ref g, _left, xL, needL, ref yL, cfg.Gap, padIn);
        HudDraw.Col(font, ref g, _right, xR, needR, ref yR, cfg.Gap, padIn);
    }

    void BuildState()
    {
        _sb.Lbl("ST", H.Lbl);
        AppendBikeFlags();
        _sb.Ln()
           .Lbl("GND", H.Lbl).Pad(GndLbl(), H.V1)
           .Lbl("MASS", H.Lbl).Pad(MassLbl(), H.V1).Ln()
           .Lbl("FRM", H.Lbl).Append(_bp.FrameNum).Ln();
    }

    void BuildInput()
    {
        _sb.Lbl("T", 3).Val(_bp.Input.Throttle, 2, 5)
           .Lbl("B", 3).Val(_bp.Input.Brake, 2, 5)
           .Lbl("L", 3).Val(_bp.Input.Lean, 2, 5).Ln()
           .Lbl("LEAN", H.Lbl).Pad(_bp.LeanL ? "L" : _bp.LeanR ? "R" : "-", 3)
           .Lbl("BRK", H.Lbl).Pad(_bp.Braking ? "ON" : "-", 3).Ln();
    }

    void BuildMotion()
    {
        _sb.Lbl("SPD", H.Lbl).Val(_bp.Speed, 1, H.V1)
           .Lbl("ANG", H.Lbl).Val(_bp.Ang * DC.R2D, 1, H.V1).Append('°').Ln()
           .Lbl("AV", H.Lbl).Val(_bp.AngVel * DC.R2D, 1, H.V1).Append("°/s").Ln();
    }

    void BuildJitter()
    {
        ref readonly DebugSnap d = ref _bp.Dbg;

        _sb.Lbl("TILT", H.Lbl).Val(_bp.TiltZ, 2, H.V1)
           .Lbl("LEAN", H.Lbl).Val(_bp.CalcLean(), 2, H.V1).Ln()
           .Lbl("LA", H.Lbl).Val(_bp.CalcLeanAccel(), 1, H.V1).Ln();

        _sb.Ln()
           .Lbl("", H.Lbl).Lbl("min", H.V1).Lbl("max", H.V2).Ln()
           .Lbl("Tilt", H.Lbl)
              .Val(d.Jitter.TiltMin, 2, H.V1).Val(d.Jitter.TiltMax, 2, H.V2).Ln()
           .Lbl("Lean", H.Lbl)
              .Val(d.Jitter.LeanMin, 2, H.V1).Val(d.Jitter.LeanMax, 2, H.V2).Ln()
           .Lbl("LA", H.Lbl)
              .Val(d.Jitter.LeanAMin, 1, H.V1).Val(d.Jitter.LeanAMax, 1, H.V2).Ln()
           .Lbl("KE", H.Lbl)
              .Val(d.Jitter.KeMin, 1, H.V1).Val(d.Jitter.KeMax, 1, H.V2).Ln()
           .Lbl("SprE", H.Lbl).Val(d.Jitter.SprMax, 3, H.V1).Ln();

        _sb.Ln().Lbl("", 4).Lbl("Vx", H.V1).Lbl("Vy", H.V2).Ln();
        for (int i = 0; i < BodyCount; i++)
            _sb.Lbl((uint)i < (uint)BodyLbl.Length ? BodyLbl[i] : $"B{i}", 4)
               .Val(d.BodyVx[i], 3, H.V1)
               .Val(d.BodyVy[i], 3, H.V2).Ln();
    }

    void BuildEnergy()
    {
        ref readonly DebugSnap d = ref _bp.Dbg;

        _sb.Lbl("KE", H.Lbl).Val(_bp.KE(), 1, H.V1)
           .Lbl("dKE", H.Lbl).Val(d.Speed.DKE, 1, H.V1).Ln()
           .Lbl("max", H.Lbl).Val(d.Speed.MaxDKE, 1, H.V1).Ln();
    }

    void BuildContact()
    {
        ref readonly ContactSnap ct = ref _bp.Ct;
        if (ct.Res == 0)
        { _sb.Append("  No resolves").Ln(); return; }

        float eAvg = ct.VnPre > 0.01f ? Abs(ct.VnPost) / ct.VnPre : 0f;

        _sb.Lbl("Res", H.Lbl).Val(ct.Res, 0, 4)
           .Lbl("Vn", H.Lbl)
              .Val(ct.VnMin, 2, H.V2).Append("..")
              .Val(ct.VnMax, 2, H.V3).Ln()
           .Lbl("Pre", H.Lbl).Val(ct.VnPre, 2, H.V1)
           .Lbl("Post", H.Lbl).Val(ct.VnPost, 2, H.V1).Ln()
           .Lbl("eAvg", H.Lbl).Val(eAvg, 3, H.V1)
           .Lbl("dKE", H.Lbl).Val(ct.DkeAvg, 3, H.V1).Ln();
    }

    void BuildIntegr()
    {
        ref readonly DebugSnap d = ref _bp.Dbg;
        int tot = d.Speed.AirF + d.Speed.GndF;
        float air = tot > 0 ? 100f * d.Speed.AirF / tot : 0f;

        _sb.Lbl("AIR", H.Lbl).Val(air, 0, 4).Append('%')
           .Lbl("OVER", H.Lbl).Pad(d.Speed.OverCnt.ToString(), 3)
           .Lbl("WARN", H.Lbl).Pad(d.Speed.WarnCnt.ToString(), 3).Ln()
           .Lbl("vMx", H.Lbl).Val(d.Speed.MaxSpd, 1, H.V1)
           .Lbl("aMx", H.Lbl).Val(d.Speed.MaxAcc, 0, H.V2).Ln()
           .Lbl("avMx", H.Lbl).Val(d.Speed.MaxAV * DC.R2D, 0, H.V1)
              .Append("°/s").Ln()
           .Lbl("λ1", H.Lbl).Val(d.Damp.L1, 4, H.V1)
           .Lbl("λ2", H.Lbl).Val(d.Damp.L2, 4, H.V1).Ln()
           .Lbl("ρ", H.Lbl).Val(d.Damp.Rho, 4, H.V1)
           .Lbl("DMP", H.Lbl).Val(d.Damp.Dmp, 1, H.V2).Append('%').Ln()
           .Lbl("T½", H.Lbl).Val(d.Damp.Half, 1, H.V1)
           .Lbl("KICK", H.Lbl).Val(d.Damp.Kick, 1, H.V1).Ln();
    }

    void BuildSprings()
    {
        ref readonly DebugSnap d = ref _bp.Dbg;

        int t1 = 0, t2 = 1;
        for (int i = 2; i < SpringCount; i++)
        {
            if (d.SprRel[i] > d.SprRel[t1])
            { t2 = t1; t1 = i; }
            else if (d.SprRel[i] > d.SprRel[t2])
                t2 = i;
        }

        _sb.Lbl("HOT", 4)
           .Pad(SprLbl[t1], 5).Val(d.SprRel[t1], 1, 5)
           .Pad(SprLbl[t2], 5).Val(d.SprRel[t2], 1, 5).Ln()
           .Lbl("DIAG", 5)
           .Append(SprLbl[5]).Append('=').Val(_bp.Spr[5].D, 2, 5)
           .Append("  ")
           .Append(SprLbl[6]).Append('=').Val(_bp.Spr[6].D, 2, 5).Ln();

        _sb.Ln()
           .Lbl("", 5).Lbl("err", H.V2).Lbl("relV", H.V2).Lbl("max", H.V3).Ln();
        for (int i = 0; i < SpringCount; i++)
            _sb.Lbl(SprLbl[i], 5)
               .Val(d.SprErr[i], 2, H.V2)
               .Val(d.SprRel[i], 1, H.V2)
               .Val(d.SprRelMax[i], 1, H.V3).Ln();

        _sb.Lbl("WHL", 4).Append("crit")
           .Val(d.Damp.FwCrit, 1, H.V3).Append('/')
           .Val(d.Damp.RwCrit, 1, H.V3)
           .Append(" %")
           .Val(d.Damp.FwRatio, 0, 4).Append('/')
           .Val(d.Damp.RwRatio, 0, 3).Ln();
    }

    void AppendBikeFlags()
    {
        BikeState s = _bp.State;
        if (s == BikeState.None)
        { _sb.Append("---"); return; }
        if ((s & BikeState.Crashed) != 0)
            _sb.Append("DIE ");
        if ((s & BikeState.Grounded) != 0)
            _sb.Append("GND ");
        if ((s & BikeState.InAir) != 0)
            _sb.Append("AIR ");
        if ((s & BikeState.Wheelie) != 0)
            _sb.Append("WHL ");
        if ((s & BikeState.Stoppie) != 0)
            _sb.Append("STP ");
    }

    string GndLbl() => (_bp.FwGnd, _bp.RwGnd) switch
    {
        (true, true) => "BOTH",
        (true, false) => "FW  ",
        (false, true) => "RW  ",
        _ => "NONE"
    };

    string MassLbl() =>
        _bp.LeanL ? "LEAN L" : _bp.LeanR ? "LEAN R" : "CENTER";

    void Log()
    {
        if (_bp.FrameNum - _logTick < 30)
            return;
        _logTick = _bp.FrameNum;

        _sb.Clear();
        AppendLog(_left);
        if (_right.Count > 0)
        { _sb.Append("\n\n"); AppendLog(_right); }

        Serilog.Log.Information(
            "\n=== TICK {Tick} ===\n{Hud}", _logTick, _sb.ToString());
    }

    void AppendLog(IReadOnlyList<HudBlock> blocks)
    {
        for (int i = 0; i < blocks.Count; i++)
        {
            if (i != 0)
                _sb.Append("\n\n");
            _sb.Append(blocks[i].Content);
        }
    }
}

sealed class DebugRender(BikePhysics bp)
{
    readonly DebugOverlay _ov = new(bp);
    readonly DebugHud _hud = new(bp);
    readonly StringBuilder _sb = new(256);

    bool Has(DebugMask m) => (bp.DebugView & m) != 0;

    public void Draw(SpriteBatch sb, SpriteFontBase font, GraphicsDevice gd,
        Matrix camTr, Cam3D? cam3D)
    {
        MouseState ms = Mouse.GetState();
        Vector2 mouse = new(ms.X, ms.Y);

        DbgGfx g = cam3D is null
            ? new(sb, gd.Viewport, camTr, BikeScale)
            : new(sb, gd.Viewport, cam3D.View, cam3D.Proj, BikeScale);

        sb.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend,
            SamplerState.PointClamp, DepthStencilState.None, RasterizerState.CullNone);

        Render(font, mouse, ref g);
        sb.End();
    }

    void Render(SpriteFontBase font, Vector2 mouse, ref DbgGfx g)
    {
        if (Has(DebugMask.Terrain))
            _ov.Terrain(ref g);

        int hover = _ov.PickNode(mouse, ref g);

        if (Has(DebugMask.Springs))
            _ov.Springs(ref g, Has(DebugMask.Integrator));

        if (Has(DebugMask.Basic))
        {
            _ov.Velocities(ref g);
            _ov.LeanForce(ref g);
            _ov.PoseViz(ref g);
            _ov.Wheels(ref g);
            _hud.Draw(font, ref g);
        }

        if (Has(DebugMask.Collision))
            _ov.Collision(hover, ref g);
        if (Has(DebugMask.Energy))
            _ov.Energy(ref g);
        if (Has(DebugMask.Ragdoll) && bp.RagdollActive)
            _ov.Ragdoll(ref g);
        if (hover >= 0)
            _ov.Tooltip(font, mouse, hover, _sb, ref g);
    }
}

static file class SbExt
{
    public static StringBuilder Ln(this StringBuilder s) => s.Append('\n');

    public static StringBuilder Lbl(this StringBuilder s, string lbl, int w)
    {
        s.Append(lbl);
        for (int i = lbl.Length; i < w; i++)
            s.Append(' ');
        return s;
    }

    public static StringBuilder Pad(this StringBuilder s, string val, int w)
    {
        for (int i = val.Length; i < w; i++)
            s.Append(' ');
        return s.Append(val);
    }

    public static StringBuilder Val(this StringBuilder s, float v, int d, int w)
    {
        Span<char> buf = stackalloc char[24];
        if (!v.TryFormat(buf, out int n, Fmt(d),
            System.Globalization.CultureInfo.InvariantCulture))
            return s.Append('?');
        for (int i = 0; i < w - n; i++)
            s.Append(' ');
        return s.Append(buf[..n]);
    }

    public static StringBuilder Val(this StringBuilder s, int v, int _, int w)
    {
        string str = v.ToString();
        for (int i = str.Length; i < w; i++)
            s.Append(' ');
        return s.Append(str);
    }

    static ReadOnlySpan<char> Fmt(int d) => d switch
    {
        <= 0 => "F0",
        1 => "F1",
        2 => "F2",
        3 => "F3",
        _ => "F4"
    };
}

#endif
