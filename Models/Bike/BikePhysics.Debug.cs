using System.Diagnostics;
using System.Text;
using static System.MathF;
using static GravityDefiedGame.Core.GamePlay;
using static GravityDefiedGame.Models.Bike.BikePhysics;
using Cam3D = GravityDefiedGame.Views.Voxel3D.Camera3D;

namespace GravityDefiedGame.Models.Bike;

#if DEBUG

public sealed partial class BikePhysics
{
    const int WinN = 30;
    const float MinDiv = 0.01f;
    const float MinSprDist = 0.0001f;
    const float WarnOmegaK = 0.9f;

    static readonly WheelFriction DbgWF = WheelFriction.Default;

    bool _dbgInit;
    float _prevSpd, _prevKE;

    DebugAcc _acc;
    DebugSnap _dbg;
    ContactAcc _ctAcc;
    ContactSnap _ct;
    ContactFrame _ctFrame;

    DebugRender? _dbgRender;

    int _dbgIdx;
    float _dbgNx, _dbgNy;

    public bool DebugNodesEnabled { get; set; }
    public DebugMask DebugView { get; set; } = DebugMask.All;

    internal float AccumDt => _accumDt;
    internal bool HasCollN => Abs(_dbgNx) > 0.001f || Abs(_dbgNy) > 0.001f;
    internal (int idx, float nx, float ny) CollInfo => (_dbgIdx, _dbgNx, _dbgNy);

    internal ref readonly DebugSnap Dbg => ref _dbg;
    internal ref readonly ContactSnap Ct => ref _ct;

    [Conditional("DEBUG")]
    internal void ResetContactStats()
    {
        InitDbg();
        _ctFrame.Reset();
    }

    [Conditional("DEBUG")]
    internal void RecordKeBefore(int idx)
    {
        InitDbg();
        _ctFrame.KeBefore = BodyKE(idx);
    }

    [Conditional("DEBUG")]
    internal void RecordResolve(
        int idx, float vn, float nx, float ny, in PtState pt)
    {
        InitDbg();
        _ctFrame.Record(vn, nx, ny, in pt, BodyKE(idx));
    }

    public void DrawDebug(
        SpriteBatch sb, SpriteFontBase font,
        GraphicsDevice gd, Matrix camTr, Cam3D? cam3D)
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
        CollectColl();
        AccumulateFrame(dt);

        if (++_acc.N >= WinN)
            Flush();

        if (FrameNum % 60 == 0)
            AnalyzePhys();
    }

    [Conditional("DEBUG")]
    public void ResetDebugStats()
    {
        InitDbg();
        _prevSpd = Speed;
        _prevKE = KE();

        _acc.Reset();
        _ctAcc.Reset();
        _ctFrame.Reset();

        _ct = default;
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

        _prevSpd = Speed;
        _prevKE = KE();
    }

    void CollectColl() =>
        (_dbgIdx, _dbgNx, _dbgNy) = (_col.CollIdx, _col.CollNx, _col.CollNy);

    void AccumulateFrame(float dt)
    {
        float spd = Speed, ke = KE();
        float fwO = Abs(Cur[BFw].AngVel);
        float rwO = Abs(Cur[BRw].AngVel);

        AccumulateSpeed(dt, spd, fwO, rwO);
        AccumulateEnergy(dt, ke);
        AccumulateJitter(ke);
        AccumulateSprings();
        AccumulateContact();
    }

    void AccumulateSpeed(float dt, float spd, float fwO, float rwO)
    {
        float omega = Max(fwO, rwO), limit = Cfg.MaxWheelOmega;

        _acc.MaxSpd = Max(_acc.MaxSpd, spd);
        _acc.MaxAcc = Max(_acc.MaxAcc, Abs(spd - _prevSpd) / dt);
        _acc.MaxAV = Max(_acc.MaxAV, Abs(AngVel));
        _acc.MaxFwO = Max(_acc.MaxFwO, fwO);
        _acc.MaxRwO = Max(_acc.MaxRwO, rwO);

        _prevSpd = spd;

        if (omega > limit)
            _acc.OverCnt++;
        if (limit > MinDiv && omega / limit > WarnOmegaK)
            _acc.WarnCnt++;

        if (InAir)
            _acc.AirF++;
        else
            _acc.GndF++;
    }

    void AccumulateEnergy(float dt, float ke)
    {
        float dke = (ke - _prevKE) / dt;
        _acc.DKE = dke;
        _acc.MaxDKE = Max(_acc.MaxDKE, Abs(dke));
        _prevKE = ke;
    }

    void AccumulateJitter(float ke)
    {
        _acc.TiltMin = Min(_acc.TiltMin, TiltAngle);
        _acc.TiltMax = Max(_acc.TiltMax, TiltAngle);
        _acc.KeMin = Min(_acc.KeMin, ke);
        _acc.KeMax = Max(_acc.KeMax, ke);

        for (int i = 0; i < BodyCount; i++)
        {
            _acc.BodyVx[i] = Max(_acc.BodyVx[i], Abs(Cur[i].Vel.X));
            _acc.BodyVy[i] = Max(_acc.BodyVy[i], Abs(Cur[i].Vel.Y));
        }

        for (int i = 0; i < SpringCount; i++)
            _acc.SprMax = Max(_acc.SprMax, Abs(SLen(i) - Spr[i].RestLen));
    }

    void AccumulateSprings()
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
        ref readonly PtState a = ref Cur[s.A], b = ref Cur[s.B];

        float dx = a.Pos.X - b.Pos.X, dy = a.Pos.Y - b.Pos.Y;
        float dist = BikeDynamics.FastDist(dx, dy);

        if (dist < MinSprDist)
            return (0f, 0f);

        float nx = dx / dist, ny = dy / dist;
        float rv = Abs((a.Vel.X - b.Vel.X) * nx + (a.Vel.Y - b.Vel.Y) * ny);

        return (Abs(dist - s.RestLen), rv);
    }

    void AccumulateContact()
    {
        _ctAcc.ResMax = (int)Max(_ctAcc.ResMax, _ctFrame.Resolves);
        if (_ctFrame.Resolves <= 0)
            return;

        _ctAcc.VnMin = Min(_ctAcc.VnMin, _ctFrame.VnMin);
        _ctAcc.VnMax = Max(_ctAcc.VnMax, _ctFrame.VnMax);
        _ctAcc.DkeSum += _ctFrame.DeltaKE;
        _ctAcc.VnPreSum += _ctFrame.VnPre;
        _ctAcc.VnPostSum += _ctFrame.VnPost;
        _ctAcc.N++;
    }

    void Flush()
    {
        float inv = 1f / WinN;

        Copy(_acc.BodyVx, _dbg.BodyVx);
        Copy(_acc.BodyVy, _dbg.BodyVy);

        for (int i = 0; i < SpringCount; i++)
        {
            _dbg.SprErr[i] = _acc.SprErrSum[i] * inv;
            _dbg.SprRel[i] = _acc.SprRelSum[i] * inv;
            _dbg.SprRelMax[i] = _acc.SprRelMax[i];
        }

        (_dbg.WhlFwDCrit, _dbg.WhlFwDRatio) = DampRatio(BFw, 0);
        (_dbg.WhlRwDCrit, _dbg.WhlRwDRatio) = DampRatio(BRw, 1);

        _dbg.TiltMin = _acc.TiltMin;
        _dbg.TiltMax = _acc.TiltMax;
        _dbg.KeMin = _acc.KeMin;
        _dbg.KeMax = _acc.KeMax;
        _dbg.SprMax = _acc.SprMax;
        _dbg.MaxSpd = _acc.MaxSpd;
        _dbg.MaxAcc = _acc.MaxAcc;
        _dbg.MaxAV = _acc.MaxAV;
        _dbg.DKE = _acc.DKE;
        _dbg.MaxDKE = _acc.MaxDKE;

        _dbg.AirF = _acc.AirF;
        _dbg.GndF = _acc.GndF;
        _dbg.OverCnt = _acc.OverCnt;
        _dbg.WarnCnt = _acc.WarnCnt;

        FlushContact();
        _acc.Reset();
        _ctAcc.Reset();
    }

    void FlushContact()
    {
        float inv = _ctAcc.N > 0 ? 1f / _ctAcc.N : 0f;
        _ct = new()
        {
            Res = _ctAcc.ResMax,
            VnMin = _ctAcc.N > 0 ? _ctAcc.VnMin : 0f,
            VnMax = _ctAcc.N > 0 ? _ctAcc.VnMax : 0f,
            DkeAvg = _ctAcc.DkeSum * inv,
            VnPre = _ctAcc.VnPreSum * inv,
            VnPost = _ctAcc.VnPostSum * inv
        };
    }

    void AnalyzePhys()
    {
        float r = Cfg.WheelR;
        (float l1, float l2) = Eigenvalues(r);
        float rho = Max(Abs(l1), Abs(l2));

        _dbg.L1 = l1;
        _dbg.L2 = l2;
        _dbg.Rho = rho;
        _dbg.Dmp = (1f - rho) * 100f;
        _dbg.Half = rho > MinDiv ? Log(0.5f) / Log(rho) : 999f;
        _dbg.Kick = DbgWF.RollCouple * r * Cfg.MaxWheelOmega;
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

        float mag = Sqrt(tr * tr * 0.25f + Abs(disc) * 0.25f);
        return (mag, mag);
    }

    (float crit, float ratio) DampRatio(int w, int s)
    {
        float invM = InvM[BFrame] + InvM[w];
        float crit = 2f * Sqrt(Spr[s].K / Max(invM, MinDiv));
        return (crit, Spr[s].D / Max(crit, MinDiv) * 100f);
    }

    float BodyKE(int i)
    {
        ref readonly PtState pt = ref New[i];
        float m = 1f / Max(InvM[i], MinDiv);
        return 0.5f * m * (pt.Vel.X * pt.Vel.X + pt.Vel.Y * pt.Vel.Y);
    }

    internal float KE()
    {
        float ke = 0f;
        for (int i = 0; i < BodyCount; i++)
            ke += BodyKE(i);
        return ke;
    }

    internal float Slip(int i)
    {
        float wR = Abs(Cur[i].AngVel) * Bods[i].R, v = VLen(Cur[i].Vel);
        float d = Max(wR, v);
        return d > MinDiv ? Abs(wR - v) / d : 0f;
    }

    internal float SDef(int a, int b) =>
        Abs(Vector2.Distance(Cur[a].Pos, Cur[b].Pos) - P.RestWheel);
    internal float ADist() =>
        Vector2.Distance(Cur[BFw].Pos, Cur[BRw].Pos);
    internal float SLen(int i) =>
        Vector2.Distance(Cur[Spr[i].A].Pos, Cur[Spr[i].B].Pos);
    internal float GndY(float x) =>
        Terrain!.GetGroundYAtX(x * P.TerrainScale) / P.TerrainScale;

    internal Vector2 GndN(float x)
    {
        const float step = 0.5f;
        float dy = GndY(x + step) - GndY(x);
        float len = Sqrt(step * step + dy * dy);
        return len > 0.0001f ? new(-dy / len, step / len) : Vector2.UnitY;
    }

    internal static float VLen(Vector2 v) => Sqrt(v.X * v.X + v.Y * v.Y);

    internal string StStr() => State switch
    {
        BikeState.Crashed => "DIE",
        BikeState.Wheelie => "WHL",
        BikeState.Stoppie => "STP",
        BikeState.InAir => "AIR",
        BikeState.Grounded => "GND",
        BikeState.None => "---",
        _ => "?"
    };

    internal string GStr() => (FwGnd, RwGnd) switch
    {
        (true, true) => "BOTH",
        (true, false) => "FW",
        (false, true) => "RW",
        _ => "NONE"
    };

    internal string MStr() => LeanL ? "WHEELIE" : LeanR ? "STOPPIE" : "NORMAL";

    static void Copy(float[] s, float[] d) => Array.Copy(s, d, s.Length);

    [Flags]
    public enum DebugMask : byte
    {
        None = 0, Basic = 1 << 0, Springs = 1 << 1, Collision = 1 << 2,
        Integrator = 1 << 3, Ragdoll = 1 << 4, Energy = 1 << 5, Terrain = 1 << 6,
        All = Basic | Springs | Collision | Integrator | Ragdoll | Energy | Terrain
    }

    struct DebugAcc
    {
        public float TiltMin, TiltMax, KeMin, KeMax, SprMax;
        public float MaxSpd, MaxAcc, MaxAV, MaxFwO, MaxRwO, DKE, MaxDKE;
        public int AirF, GndF, OverCnt, WarnCnt, N;

        public readonly float[] BodyVx, BodyVy;
        public readonly float[] SprErrSum, SprRelSum, SprRelMax;

        public DebugAcc(int b, int s)
        {
            BodyVx = new float[b];
            BodyVy = new float[b];
            SprErrSum = new float[s];
            SprRelSum = new float[s];
            SprRelMax = new float[s];

            TiltMin = KeMin = float.MaxValue;
            TiltMax = KeMax = float.MinValue;
            SprMax = MaxSpd = MaxAcc = MaxAV = MaxFwO = MaxRwO = DKE = MaxDKE = 0f;
            AirF = GndF = OverCnt = WarnCnt = N = 0;
        }

        public void Reset()
        {
            TiltMin = KeMin = float.MaxValue;
            TiltMax = KeMax = float.MinValue;
            SprMax = MaxSpd = MaxAcc = MaxAV = MaxFwO = MaxRwO = DKE = MaxDKE = 0f;
            AirF = GndF = OverCnt = WarnCnt = N = 0;

            Array.Clear(BodyVx);
            Array.Clear(BodyVy);
            Array.Clear(SprErrSum);
            Array.Clear(SprRelSum);
            Array.Clear(SprRelMax);
        }
    }

    public struct DebugSnap
    {
        public float TiltMin, TiltMax, KeMin, KeMax, SprMax;
        public float MaxSpd, MaxAcc, MaxAV, DKE, MaxDKE;
        public float WhlFwDCrit, WhlRwDCrit, WhlFwDRatio, WhlRwDRatio;
        public float L1, L2, Rho, Dmp, Half, Kick;
        public int AirF, GndF, OverCnt, WarnCnt;

        public readonly float[] BodyVx, BodyVy;
        public readonly float[] SprErr, SprRel, SprRelMax;

        public DebugSnap(int b, int s)
        {
            BodyVx = new float[b];
            BodyVy = new float[b];
            SprErr = new float[s];
            SprRel = new float[s];
            SprRelMax = new float[s];

            TiltMin = TiltMax = KeMin = KeMax = SprMax = 0f;
            MaxSpd = MaxAcc = MaxAV = DKE = MaxDKE = 0f;
            WhlFwDCrit = WhlRwDCrit = WhlFwDRatio = WhlRwDRatio = 0f;
            L1 = L2 = Rho = Dmp = Half = Kick = 0f;
            AirF = GndF = OverCnt = WarnCnt = 0;
        }
    }

    struct ContactAcc
    {
        public float VnMin, VnMax, DkeSum, VnPreSum, VnPostSum;
        public int ResMax, N;

        public ContactAcc()
        {
            VnMin = float.MaxValue;
            VnMax = float.MinValue;
            DkeSum = VnPreSum = VnPostSum = 0f;
            ResMax = N = 0;
        }

        public void Reset()
        {
            VnMin = float.MaxValue;
            VnMax = float.MinValue;
            DkeSum = VnPreSum = VnPostSum = 0f;
            ResMax = N = 0;
        }
    }

    public struct ContactSnap
    {
        public float VnMin, VnMax, DkeAvg, VnPre, VnPost;
        public int Res;
    }

    struct ContactFrame
    {
        public int Resolves;
        public float VnMin, VnMax, DeltaKE, VnPre, VnPost, KeBefore;

        public ContactFrame()
        {
            Resolves = 0;
            DeltaKE = VnPre = VnPost = KeBefore = 0f;
            VnMin = float.MaxValue;
            VnMax = float.MinValue;
        }

        public void Reset()
        {
            Resolves = 0;
            DeltaKE = VnPre = VnPost = KeBefore = 0f;
            VnMin = float.MaxValue;
            VnMax = float.MinValue;
        }

        public void Record(float vn, float nx, float ny, in PtState pt, float keAfter)
        {
            VnMin = Min(VnMin, vn);
            VnMax = Max(VnMax, vn);
            VnPre = vn;
            VnPost = -(pt.Vel.X * nx + pt.Vel.Y * ny);
            DeltaKE += keAfter - KeBefore;
            Resolves++;
        }
    }
}

sealed class DebugRender(BikePhysics bp)
{
    static readonly PhysConst P = PhysConst.Default;
    readonly StringBuilder _sb = new(1024);
    int _logTick;

    const BikePhysics.DebugMask
        MB = BikePhysics.DebugMask.Basic,
        MS = BikePhysics.DebugMask.Springs,
        MC = BikePhysics.DebugMask.Collision,
        MI = BikePhysics.DebugMask.Integrator,
        MR = BikePhysics.DebugMask.Ragdoll,
        ME = BikePhysics.DebugMask.Energy,
        MT = BikePhysics.DebugMask.Terrain;

    static readonly string[] BodyLbl = ["Frm", "FW ", "RW ", "UF ", "UR ", "Hd "];
    static readonly string[] NodeNms = ["Frame", "FW", "RW", "UF", "UR", "Head"];

    static readonly string[] SprLbl =
    [
        "FrFW", "FrRW", "FrUF", "FrUR", "UFUR",
        "FWUF", "RWUR", "HdUR", "HdUF", "HdFr"
    ];

    static readonly Color[] NodeClrs =
    [
        Color.Yellow, Color.Cyan, Color.Cyan,
        Color.Orange, Color.Orange, Color.Magenta
    ];

    static ReadOnlySpan<byte> PoseBones =>
        [0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 4, 6, 6, 7];

    static ReadOnlySpan<byte> RagBones =>
        [0, 1, 1, 2, 2, 3, 1, 4, 4, 7, 0, 6, 6, 5, 0, 4, 2, 4, 1, 6, 0, 7];

    static class C
    {
        public const float
            Pick = 22f, VelK = 0.08f, R2D = 57.2958f,
            Desync = 2f, NormL = 2f, SprK = 0.002f,
            SlipWarn = 0.3f, SlipCrit = 0.6f, ESpike = 50f;

        public static readonly Color
            Spring = Color.LimeGreen * 0.6f,
            Warn = Color.Orange * 0.7f,
            Crit = Color.Red * 0.8f,
            Bg = Color.Black * 0.8f,
            Tip = Color.Black * 0.85f,
            Hover = Color.Red,
            Link = Color.Red * 0.4f,
            Vel = Color.Yellow * 0.4f,
            Pose = Color.White * 0.5f,
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
    }

    readonly ref struct G
    {
        public readonly Viewport Vp;
        public readonly SpriteBatch Sb;
        readonly bool _is3D;
        readonly float _sc;
        readonly Matrix _cam, _view, _proj;

        public G(SpriteBatch sb, Viewport vp, Matrix cam, float sc)
            : this(sb, vp, sc, false, cam, default, default) { }

        public G(SpriteBatch sb, Viewport vp, Matrix view, Matrix proj, float sc)
            : this(sb, vp, sc, true, default, view, proj) { }

        G(
            SpriteBatch sb, Viewport vp, float sc, bool is3D,
            Matrix cam, Matrix view, Matrix proj)
        {
            (Sb, Vp, _sc, _is3D) = (sb, vp, sc, is3D);
            (_cam, _view, _proj) = (cam, view, proj);
        }

        public Vector2 W(Vector2 w) =>
            _is3D ? W3(w) : Vector2.Transform(w * _sc, _cam);

        Vector2 W3(Vector2 w)
        {
            Vector3 p = Vp.Project(
                new(w.X * _sc, -w.Y * _sc, 0f),
                _proj, _view, Matrix.Identity);
            return new(p.X, p.Y);
        }

        public bool InView(Vector2 w)
        {
            if (!_is3D)
                return true;
            Vector3 p = Vp.Project(
                new(w.X * _sc, -w.Y * _sc, 0f),
                _proj, _view, Matrix.Identity);
            return p.Z is >= 0f and <= 1f;
        }

        public G L(Vector2 a, Vector2 b, Color c, float t)
        { Sb.DrawLine(W(a), W(b), c, t); return this; }

        public G Pt(Vector2 p, Color c, float s)
        { Sb.DrawPoint(W(p), c, s); return this; }

        public G Bar(Vector2 pos, float h, Color c, float t)
        { Sb.DrawLine(pos, pos - new Vector2(0, h), c, t); return this; }

        public G Ring(Vector2 ctr, float r, int seg, Color c, float t)
        {
            if (seg < 3)
                return this;
            float step = Tau / seg;

            for (int i = 0; i < seg; i++)
            {
                float a1 = i * step, a2 = (i + 1) * step;
                Sb.DrawLine(
                    W(new(ctr.X + Cos(a1) * r, ctr.Y + Sin(a1) * r)),
                    W(new(ctr.X + Cos(a2) * r, ctr.Y + Sin(a2) * r)),
                    c, t);
            }
            return this;
        }

        public G Skel(
            ReadOnlySpan<Vector2> pts, ReadOnlySpan<byte> idx,
            Color line, float lt, Color dot, float ds)
        {
            for (int i = 0; i < idx.Length; i += 2)
                L(pts[idx[i]], pts[idx[i + 1]], line, lt);

            foreach (Vector2 p in pts)
                Pt(p, dot, ds);

            return this;
        }

        public G Hud(
            SpriteFontBase font, Vector2 pos,
            string text, Color bg, Color fg)
        {
            Vector2 sz = font.MeasureString(text);
            Sb.FillRectangle(
                new Rectangle(
                    (int)pos.X - 6, (int)pos.Y - 4,
                    (int)sz.X + 12, (int)sz.Y + 8),
                bg);
            font.DrawText(Sb, text, pos, fg);
            return this;
        }
    }

    public void Draw(
        SpriteBatch sb, SpriteFontBase font,
        GraphicsDevice gd, Matrix camTr, Cam3D? cam3D)
    {
        MouseState ms = Mouse.GetState();
        Vector2 mouse = new(ms.X, ms.Y);

        G g = MakeCtx(sb, gd.Viewport, camTr, cam3D);

        sb.Begin(
            SpriteSortMode.Deferred, BlendState.AlphaBlend,
            SamplerState.PointClamp, DepthStencilState.None,
            RasterizerState.CullNone);

        RenderFrame(font, mouse, in g);

        sb.End();
    }

    static G MakeCtx(SpriteBatch sb, Viewport vp, Matrix cam, Cam3D? c3) =>
        c3 is null
            ? new G(sb, vp, cam, BikeScale)
            : new G(sb, vp, c3.View, c3.Proj, BikeScale);

    void RenderFrame(SpriteFontBase font, Vector2 mouse, in G g)
    {
        if (Has(MT))
            DrawTerrain(in g);

        int hover = DrawNodes(mouse, in g);

        if (Has(MS))
            DrawSprings(in g);
        if (Has(MB))
            DrawBasic(font, in g);
        if (Has(MC))
            DrawCollision(hover, in g);
        if (Has(ME))
            DrawEnergy(in g);

        if (Has(MR) && bp.RagdollActive)
            DrawRagdoll(in g);

        if (hover >= 0)
            DrawTooltip(font, mouse, hover, in g);
    }

    bool Has(BikePhysics.DebugMask m) => (bp.DebugView & m) != 0;

    int DrawNodes(Vector2 mouse, in G g)
    {
        int best = -1;
        float bestSq = C.Pick * C.Pick;

        for (int i = 0; i < BodyCount; i++)
        {
            Vector2 s = g.W(bp.Cur[i].Pos);
            g.Sb.DrawPoint(s, NodeC(i), bp.Bods[i].IsWheel ? 6f : 4f);

            float dx = s.X - mouse.X;
            float dy = s.Y - mouse.Y;
            float sq = dx * dx + dy * dy;

            if (sq < bestSq)
            {
                bestSq = sq;
                best = i;
            }
        }

        return best;
    }

    void DrawSprings(in G g)
    {
        for (int i = 0; i < SpringCount; i++)
        {
            ref readonly SpringCfg s = ref bp.Spr[i];
            float len = bp.SLen(i);

            float strain = s.RestLen > 0.01f
                ? Abs(len - s.RestLen) / s.RestLen
                : 0f;

            Color col = strain > 0.15f ? C.Crit
                      : strain > 0.08f ? C.Warn
                      : C.Spring;

            g.L(bp.Cur[s.A].Pos, bp.Cur[s.B].Pos, col, 1.5f + strain * 3f);

            if (Has(MI))
            {
                Vector2 mid = g.W((bp.Cur[s.A].Pos + bp.Cur[s.B].Pos) * 0.5f);
                float bar = s.K * Abs(len - s.RestLen) * C.SprK;
                g.Bar(mid, bar, col, 2f);
            }
        }
    }

    void DrawBasic(SpriteFontBase font, in G g)
    {
        for (int i = 0; i < BodyCount; i++)
            g.L(
                bp.Cur[i].Pos,
                bp.Cur[i].Pos + bp.Cur[i].Vel * C.VelK,
                C.Vel, 1f);

        Pose pose = Poses.Build(
            bp.Cur[BFrame].Pos, bp.Cur[BUF].Pos,
            bp.Cur[BUR].Pos, bp.TiltZ);

        ReadOnlySpan<Vector2> pts =
        [
            pose.Ft, pose.Kn, pose.Hip, pose.Sh,
            pose.Elb, pose.Hnd, pose.Nk, pose.Hd
        ];

        g.Skel(pts, PoseBones, C.Skel, 1f, C.Pose, 3f);

        Vector2 phy = g.W(bp.Cur[BHead].Pos);
        Vector2 viz = g.W(pts[7]);

        if (Vector2.Distance(phy, viz) > C.Desync)
            g.Sb.DrawLine(phy, viz, C.Dsync, 1.5f);

        DrawWheel(BFw, in g);
        DrawWheel(BRw, in g);
        DrawHud(font, in g);
    }

    void DrawWheel(int i, in G g)
    {
        Vector2 pos = bp.Cur[i].Pos;
        float r = bp.Bods[i].R, rot = bp.Cur[i].Angle;
        Color col = SlipC(bp.Slip(i));

        g.Ring(pos, r, 20, col, 1.5f)
         .L(pos, pos + new Vector2(Cos(rot), Sin(rot)) * r, col, 1f);
    }

    void DrawCollision(int hover, in G g)
    {
        (int idx, float nx, float ny) = bp.CollInfo;

        if (idx >= 0 && idx < BodyCount && bp.HasCollN)
        {
            Vector2 pos = bp.Cur[idx].Pos;
            Vector2 n = new(nx, ny);
            g.L(pos, pos + n * C.NormL, C.Norm, 2.5f)
             .Pt(pos, C.Norm, 5f);
        }

        if (hover < 0)
            return;

        Vector2 hp = bp.Cur[hover].Pos;
        float oR = P.OuterR(bp.Bods[hover].R);
        float iR = P.InnerR(bp.Bods[hover].R);

        g.Ring(hp, oR * 2f - iR, 24, C.ZoneTouch, 1f)
         .Ring(hp, oR, 24, C.ZoneOut, 1f)
         .Ring(hp, iR, 24, C.ZoneIn, 1f);

        if (bp.Terrain is null)
            return;

        float gy = bp.GndY(hp.X);
        Vector2 gn = bp.GndN(hp.X);
        Vector2 gp = new(hp.X, gy);

        g.L(gp, gp + gn * C.NormL, C.Norm, 2f)
         .Pt(gp, C.Norm, 3f);
    }

    void DrawEnergy(in G g)
    {
        Vector2 ctr = g.W(bp.Cur[BFrame].Pos) + new Vector2(20, 0);
        Color col = Abs(bp.Dbg.DKE) > C.ESpike ? C.EngCrit : C.Eng;
        g.Bar(ctr, Min(bp.KE() * 0.5f, 80f), col, 3f);

        Vector2 fw = bp.Cur[BFw].Pos, rw = bp.Cur[BRw].Pos;
        float dx = fw.X - rw.X, dy = fw.Y - rw.Y;
        float d2 = dx * dx + dy * dy;

        Color axCol = d2 < P.MinAxleDist2 || d2 > P.MaxAxleDist2 ? C.Crit : C.Eng;
        g.L(fw, rw, axCol, 1.5f);
    }

    void DrawRagdoll(in G g)
    {
        Pose rp = bp.RiderPose;
        ReadOnlySpan<Vector2> pts =
        [
            rp.Hip, rp.Sh, rp.Elb, rp.Hnd,
            rp.Nk, rp.Ft, rp.Kn, rp.Hd
        ];
        g.Skel(pts, RagBones, C.Rag, 2f, C.Rag, 4f);
    }

    void DrawTerrain(in G g)
    {
        if (bp.Terrain is null)
            return;

        (int s, int e) = bp.Terrain.GetVisibleRange();
        float isc = 1f / P.TerrainScale;

        for (int i = s; i < e; i++)
        {
            bp.Terrain.GetSegmentPoints(
                i, out float sx, out float sy, out float ex, out float ey);

            Vector2 a = new(sx * isc, sy * isc);
            Vector2 b = new(ex * isc, ey * isc);

            if (!g.InView(a) || !g.InView(b))
                continue;

            bp.Terrain.GetSegmentNormal(i, out float nx, out float ny);
            Vector2 mid = (a + b) * 0.5f;

            g.L(a, b, C.Terr, 2f)
             .L(mid, mid + new Vector2(nx, ny), C.Norm * 0.4f, 1f);
        }
    }

    void DrawHud(SpriteFontBase font, in G g)
    {
        string left = HudText(AppendHudLeft);
        string right = HudText(AppendHudRight);

        DrawHudPanel(font, in g, 10f, left);

        if (right.Length > 0)
        {
            float x = g.Vp.Width - font.MeasureString(right).X - 22f;
            DrawHudPanel(font, in g, x, right);
        }

        LogStats(left, right);
    }

    string HudText(Action build)
    {
        _sb.Clear();
        build();
        return _sb.ToString();
    }

    static void DrawHudPanel(SpriteFontBase font, in G g, float x, string text) =>
        g.Hud(font, new(x, 10f), text, C.Bg, Color.White);

    void LogStats(string left, string right)
    {
        if (bp.FrameNum - _logTick < 30)
            return;
        _logTick = bp.FrameNum;

        Serilog.Log.Information(
            "\n=== TICK {Tick} ===\n{Left}\n{Right}",
            _logTick, left, right);
    }

    void AppendHudLeft()
    {
        ref readonly DebugSnap l = ref bp.Dbg;

        _sb.Append("ST ").Append(bp.StStr())
           .Append("  GND ").Append(bp.GStr())
           .Append("  MASS ").Append(bp.MStr())
           .Append("  FRM ").Append(bp.FrameNum).Ln();

        _sb.Append("SPD").F(bp.Speed, 1, 6)
           .Append("  ANG").F(bp.Ang * C.R2D, 1, 6).Append('°')
           .Append("  AV").F(bp.AngVel * C.R2D, 1, 6).Append("°/s").Ln();

        _sb.Append("IN T=").F(bp.Input.Throttle, 0)
           .Append(" B=").F(bp.Input.Brake, 0)
           .Append(" L=").F(bp.Input.Lean, 0)
           .Append("  LEAN ").Append(bp.LeanL ? "L" : bp.LeanR ? "R" : "-")
           .Append("  BRK ").Append(bp.Braking ? "ON" : "-").Ln();

        _sb.Append("TILT").F(bp.TiltZ, 2, 6)
           .Append("  TILTA").F(bp.TiltAngle, 2, 6).Ln();

        _sb.Append("\n[ JITTER 30F ]\n")
           .Append("Tilt").F(l.TiltMin, 2, 7).Append(" ..").F(l.TiltMax, 2, 6).Ln()
           .Append("KE  ").F(l.KeMin, 1, 7).Append(" ..").F(l.KeMax, 1, 6).Ln()
           .Append("SprE").F(l.SprMax, 3, 8).Ln();

        _sb.Append("         Vx     Vy\n");
        for (int i = 0; i < BodyCount; i++)
        {
            _sb.Append(BodyLbl[i])
               .F(l.BodyVx[i], 3, 7)
               .F(l.BodyVy[i], 3, 7).Ln();
        }
    }

    void AppendHudRight()
    {
        ref readonly DebugSnap l = ref bp.Dbg;
        ref readonly ContactSnap ct = ref bp.Ct;

        if (Has(ME))
        {
            _sb.Append("KE").F(bp.KE(), 1, 7)
               .Append("  dKE").F(l.DKE, 1, 7)
               .Append("  max").F(l.MaxDKE, 1, 6).Ln();
        }

        if (Has(MC))
        {
            _sb.Append("\n[ CONTACT 30F ]\n");

            if (ct.Res == 0)
            {
                _sb.Append("No resolves").Ln();
            }
            else
            {
                float eAvg = ct.VnPre > 0.01f ? Abs(ct.VnPost) / ct.VnPre : 0f;

                _sb.Append("Res").F(ct.Res, 0, 3)
                   .Append("  Vn").F(ct.VnMin, 2, 6)
                   .Append(" ..").F(ct.VnMax, 2, 0).Ln();

                _sb.Append("Pre").F(ct.VnPre, 2, 7)
                   .Append("  Post").F(ct.VnPost, 2, 7).Ln();

                _sb.Append("eAvg").F(eAvg, 3, 6)
                   .Append("  dKE").F(ct.DkeAvg, 3, 7).Ln();
            }
        }

        if (Has(MI))
        {
            int tot = l.AirF + l.GndF;
            float air = tot > 0 ? 100f * l.AirF / tot : 0f;

            _sb.Append("\n[ INTEGRATOR ]\n")
               .Append("AIR").F(air, 0, 4).Append('%')
               .Append("  OVER ").Append(l.OverCnt)
               .Append("  WARN ").Append(l.WarnCnt).Ln();

            _sb.Append("vMx").F(l.MaxSpd, 1, 6)
               .Append("  aMx").F(l.MaxAcc, 0, 5)
               .Append("  avMx").F(l.MaxAV * C.R2D, 0, 5).Append("°/s").Ln();

            _sb.Append("λ1").F(l.L1, 4, 8)
               .Append("  λ2").F(l.L2, 4, 8).Ln();

            _sb.Append("ρ ").F(l.Rho, 4, 8)
               .Append("  DMP").F(l.Dmp, 1, 5).Append('%').Ln();

            _sb.Append("T½").F(l.Half, 1, 6)
               .Append("  KICK").F(l.Kick, 1, 6).Ln();
        }

        if (Has(MS))
        {
            _sb.Append("\n[ SPRINGS 30F ]\n");

            int t1 = 0, t2 = 1;
            for (int i = 2; i < SpringCount; i++)
            {
                if (l.SprRel[i] > l.SprRel[t1])
                { t2 = t1; t1 = i; }
                else if (l.SprRel[i] > l.SprRel[t2])
                { t2 = i; }
            }

            _sb.Append("HOT ").Append(SprLbl[t1])
               .Append(" rv").F(l.SprRel[t1], 1, 4)
               .Append("  ").Append(SprLbl[t2])
               .Append(" rv").F(l.SprRel[t2], 1, 4).Ln();

            _sb.Append("DIAG D: ")
               .Append(SprLbl[5]).Append('=').F(bp.Spr[5].D, 2, 5)
               .Append("  ")
               .Append(SprLbl[6]).Append('=').F(bp.Spr[6].D, 2, 5).Ln();

            _sb.Append("      err  relV  max\n");
            for (int i = 0; i < SpringCount; i++)
            {
                _sb.Append(SprLbl[i])
                   .F(l.SprErr[i], 2, 6)
                   .F(l.SprRel[i], 1, 6)
                   .F(l.SprRelMax[i], 1, 5).Ln();
            }

            _sb.Append("WHL crit ")
               .F(l.WhlFwDCrit, 1, 5).Append('/')
               .F(l.WhlRwDCrit, 1, 5)
               .Append("  % ").F(l.WhlFwDRatio, 0, 3).Append('/')
               .F(l.WhlRwDRatio, 0, 0).Ln();
        }
    }

    void DrawTooltip(SpriteFontBase font, Vector2 mouse, int i, in G g)
    {
        ref readonly PtState pt = ref bp.Cur[i];
        float spd = VLen(pt.Vel), frc = VLen(pt.Force);

        _sb.Clear()
           .Append(Name(i)).Append(" [").Append(i).Append(']').Ln()
           .Append('P').F(pt.Pos.X, 2, 7).Append(',').F(pt.Pos.Y, 2, 0).Ln()
           .Append('V').F(pt.Vel.X, 2, 7).Append(',').F(pt.Vel.Y, 2, 0).Ln()
           .Append("|V|").F(spd, 1, 5).Append("  F").F(frc, 0, 5)
           .Append("  R").F(bp.Bods[i].R, 2, 5).Append("  M").F(bp.InvM[i], 2, 5).Ln();

        Vector2 ns = g.W(pt.Pos);
        g.Sb.DrawCircle(ns, 10f, 16, C.Hover, 2f);
        g.Sb.DrawLine(ns, mouse, C.Link, 1f);

        if (Has(MS))
        {
            for (int j = 0; j < SpringCount; j++)
            {
                ref readonly SpringCfg s = ref bp.Spr[j];
                if (s.A != i && s.B != i)
                    continue;
                g.L(bp.Cur[s.A].Pos, bp.Cur[s.B].Pos, Color.White, 2.5f);
            }
        }

        string text = _sb.ToString();
        Vector2 sz = font.MeasureString(text);

        Vector2 pos = new(
            Math.Clamp(mouse.X + 18, 6f, g.Vp.Width - sz.X - 18f),
            Math.Clamp(mouse.Y + 18, 6f, g.Vp.Height - sz.Y - 18f));

        g.Hud(font, pos, text, C.Tip, Color.LimeGreen);
    }

    static Color SlipC(float s) => s > C.SlipCrit
        ? Color.Red * 0.5f
        : s > C.SlipWarn ? Color.Yellow * 0.5f : Color.Green * 0.5f;

    static Color NodeC(int i) => i < NodeClrs.Length ? NodeClrs[i] : C.Node;
    static string Name(int i) => i < NodeNms.Length ? NodeNms[i] : $"N{i}";
}

static file class X
{
    public static StringBuilder Ln(this StringBuilder s) => s.Append('\n');

    public static StringBuilder F(this StringBuilder s, float v, int d, int w = 0)
    {
        Span<char> buf = stackalloc char[24];
        if (!v.TryFormat(buf, out int n, Fmt(d), CultureInfo.InvariantCulture))
            return s.Append('?');

        if (w > n)
            s.Spc(w - n);

        return s.Append(buf[..n]);
    }

    static StringBuilder Spc(this StringBuilder s, int n)
    {
        for (int i = 0; i < n; i++)
            s.Append(' ');
        return s;
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
