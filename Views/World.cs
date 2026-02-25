using static System.MathF;
using static GravityDefiedGame.Views.Sky;
using static GravityDefiedGame.Views.Geo;

namespace GravityDefiedGame.Views;

public readonly struct FlagCfg
{
    public readonly float
        PoleTh, LineTh, PoleH, FlagW, FlagH,
        WaveSpd, WaveFreq, WaveAmp, CellSz;

    public readonly int CheckN;
    public readonly Color Pole, Start;

    public FlagCfg()
    {
        (PoleTh, LineTh, PoleH, FlagW, FlagH) = (2.5f, 1.25f, 55f, 22f, 16f);
        (WaveSpd, WaveFreq, WaveAmp, CellSz, CheckN) = (3.5f, 0.35f, 6f, 5f, 4);
        (Pole, Start) = (new(140, 140, 140), new(0, 200, 0));
    }
}

public readonly struct SkyCfg
{
    public readonly float SunX, SunY, StarH, AlphaLo, AlphaHi, SzLo, SzHi;
    public readonly int SunR1, SunR2, SunSeg, StarSeed;
    public readonly Color SunGlow, SunCore;

    public SkyCfg()
    {
        (SunX, SunY, StarH) = (0.35f, 0.2f, 0.7f);
        (SunR1, SunR2, SunSeg, StarSeed) = (52, 40, 16, unchecked((int)19349663u));
        (AlphaLo, AlphaHi, SzLo, SzHi) = (0.3f, 0.7f, 0.6f, 1.4f);
        (SunGlow, SunCore) = (new(255, 178, 48, 76), new(255, 218, 98));
    }
}

public readonly struct ProjCfg
{
    public readonly float HorzOfs, ProjR, MaxOfs, ZNear, ZFar;
    public readonly int DepthDiv, SkyBands, Extra;

    public ProjCfg()
    {
        (HorzOfs, ProjR, MaxOfs) = (80f, 0.0375f, 50f);
        (ZNear, ZFar) = (-15f, 15f);
        (DepthDiv, SkyBands, Extra) = (30, 12, 40);
    }
}

public readonly record struct Ctx(RenderCfg Cfg, ThemeSettings Theme, Color[] Sky)
{
    public static Ctx Load() => new(
        ThemeManager.Rendering,
        ThemeManager.Cur,
        Clr.ParsePal(ThemeManager.Cur.RenderCfg.SkyPal));
}

public delegate void LnFn(Vector2 a, Vector2 b, float th, Color c);

public readonly record struct TerrainCtx(LnFn Line);

public static class DrawExt
{
    const float Phi = 0.618033988f;

    public static SpriteBatch QuadFrame(
        this SpriteBatch sb,
        Vector2 an, Vector2 af, Vector2 bn, Vector2 bf,
        Color c, float th)
    {
        sb.DrawLine(an, bn, c, th);
        sb.DrawLine(af, bf, c, th);
        sb.DrawLine(an, af, c, th);
        sb.DrawLine(bn, bf, c, th);
        return sb;
    }

    public static SpriteBatch QuadFill(
        this SpriteBatch sb,
        Vector2 an, Vector2 af, Vector2 bn, Vector2 bf,
        Color c, float sc, int seg, int n)
    {
        for (int j = 0; j < n; j++)
        {
            int idx = seg * n + j;
            float tx = idx * Phi % 1f, ty = idx * Phi * Phi % 1f;
            sb.DrawPoint(
                Vector2.Lerp(Vector2.Lerp(an, bn, tx), Vector2.Lerp(af, bf, tx), ty),
                c, sc);
        }
        return sb;
    }
}

public static class Geo
{
    internal static readonly ProjCfg P = new();

    public static Vector2 Point(Level lv, int i) =>
        new(lv.GetPtX(i), lv.GetPtY(i));

    public static Vector2 ViewPos(Vector2 cam) =>
        new(cam.X, cam.Y - P.HorzOfs);

    public static bool Visible(Level lv, out int s, out int e)
    {
        (s, e) = lv.GetVisibleRange();
        int max = lv.PtCount - 1;
        if (max < 0)
        { s = e = 0; return false; }
        (s, e) = (Math.Clamp(s, 0, max), Math.Clamp(e, 0, max));
        return s < e;
    }

    public static (Vector2 near, Vector2 far) Project(Vector2 pt, Vector2 vp)
    {
        float dx = Math.Clamp((vp.X - pt.X) * P.ProjR, -P.MaxOfs, P.MaxOfs);
        float dy = Math.Clamp((vp.Y - pt.Y) * P.ProjR, -P.MaxOfs, P.MaxOfs);
        return (new(pt.X - dx, pt.Y - dy), new(pt.X + dx, pt.Y + dy));
    }

    public static TerrainCtx Ctx2D(SpriteBatch sb) =>
        new((a, b, th, c) => sb.DrawLine(a, b, c, th));

    public static TerrainCtx Ctx3D(Mesh m, float z) =>
        new((a, b, th, c) => m.Line(a, b, z, th, c));
}

public static class Flags
{
    static readonly FlagCfg F = new();

    public static void Draw(
        Level lv, int s, int e,
        Vector2 startPos, Vector2 finishPos,
        float t, in TerrainCtx ctx)
    {
        if (InRange(lv, s, e, lv.StartGatePos.X))
            Flag(startPos, t, in ctx, true);
        if (InRange(lv, s, e, lv.FinishGatePos.X))
            Flag(finishPos, t, in ctx, false);
    }

    static bool InRange(Level lv, int s, int e, float x) =>
        x >= lv.GetPtX(s) && x <= lv.GetPtX(e);

    static void Flag(Vector2 g, float t, in TerrainCtx ctx, bool wave)
    {
        Vector2 top = new(g.X, g.Y - F.PoleH);
        ctx.Line(g, top, F.PoleTh, F.Pole);
        if (wave)
            Wave(top, t, in ctx);
        else
            Check(top, in ctx);
    }

    static void Wave(Vector2 top, float t, in TerrainCtx ctx)
    {
        float h2 = F.FlagH * 0.5f;
        for (int y = 0; y < (int)F.FlagH; y++)
        {
            float w = F.FlagW * (1f - Abs(y / h2 - 1f));
            if (w < 0.5f)
                continue;
            float dx = Sin(t * F.WaveSpd + y * F.WaveFreq) * F.WaveAmp * (y / F.FlagH);
            ctx.Line(
                new(top.X, top.Y + y),
                new(Max(top.X + 1f, top.X + w + dx), top.Y + y),
                F.LineTh, F.Start);
        }
    }

    static void Check(Vector2 top, in TerrainCtx ctx)
    {
        for (int r = 0; r < F.CheckN; r++)
            for (int c = 0; c < F.CheckN; c++)
            {
                float x = top.X + c * F.CellSz;
                float y = top.Y + (r + 0.5f) * F.CellSz;
                ctx.Line(
                    new(x, y), new(x + F.CellSz, y), F.CellSz,
                    (r + c) % 2 == 0 ? Color.White : Color.Black);
            }
    }
}

public static class Sky
{
    static readonly SkyCfg S = new();

    public static void Grad(SpriteBatch sb, int w, int h, Color[] pal, int bands)
    {
        if (pal.Length < 2)
            return;
        int last = pal.Length - 1;
        float bh = h / (float)bands;

        for (int i = 0; i < bands; i++)
        {
            float pos = (i + 0.5f) / bands * last;
            int j = Math.Min((int)pos, last - 1);
            sb.FillRectangle(0f, i * bh, w, bh + 1f,
                Color.Lerp(pal[j], pal[j + 1], pos - j));
        }
    }

    public static void Sun(SpriteBatch sb, int w, int h)
    {
        Vector2 c = new(w * S.SunX, h * S.SunY);
        sb.DrawCircle(c, S.SunR1, S.SunSeg, S.SunGlow, 20f);
        sb.DrawCircle(c, S.SunR2, S.SunSeg, S.SunCore, 40f);
    }

    public static void Stars(SpriteBatch sb, int w, int h, int n)
    {
        Random rnd = new(S.StarSeed);
        float maxY = h * S.StarH;

        for (int i = 0; i < n; i++)
        {
            float x = rnd.NextSingle() * w;
            float y = rnd.NextSingle() * maxY;
            float a = S.AlphaLo + rnd.NextSingle() * (S.AlphaHi - S.AlphaLo);
            float sz = S.SzLo + rnd.NextSingle() * (S.SzHi - S.SzLo);
            sb.DrawPoint(x, y, new Color(255, 255, 255, (int)(a * 255f)), sz);
        }
    }
}

public abstract class WorldBase
{
    protected Ctx C = Ctx.Load();

    public void OnTheme() => C = Ctx.Load();

    public abstract void Sky(SpriteBatch sb, int w, int h);
}

public sealed class VoxWorld : WorldBase
{
    public override void Sky(SpriteBatch sb, int w, int h) =>
        Grad(sb, w, h, C.Sky, P.SkyBands);

    public static void Terrain(
        Mesh m, Level lv, TerrainColors col, float th, float t)
    {
        if (!Visible(lv, out int s, out int e))
            return;

        int end = Math.Min(e + P.Extra, lv.PtCount - 1);
        float z0 = P.ZNear, z1 = P.ZFar;

        Vector2 p = Point(lv, s);
        m.Cross(p, z0, z1, th, col.Stroke);

        for (int i = s; i < end; i++)
        {
            Vector2 q = Point(lv, i + 1);
            m.Seg(p, q, z0, z1, th, col.Fill, col.Stroke);
            p = q;
        }

        m.Cross(p, z0, z1, th, col.Stroke);

        Vector2 sp = lv.StartGatePos, fp = lv.FinishGatePos;
        Flags.Draw(lv, s, e, sp, fp, t, Ctx3D(m, z0));
        Flags.Draw(lv, s, e, sp, fp, t, Ctx3D(m, z1));
    }
}

public sealed class FlatWorld : WorldBase
{
    const float ZLo = 0.5f, ZHi = 1.5f, ZK = 1.5f;

    public override void Sky(SpriteBatch sb, int w, int h)
    {
        Grad(sb, w, h, C.Sky, C.Cfg.GradientSteps);
        ThemeRenderCfg rc = C.Theme.RenderCfg;
        if (rc.HasSun)
            Sun(sb, w, h);
        if (rc.HasStars)
            Stars(sb, w, h, C.Cfg.StarCount);
    }

    public void Terrain(
        SpriteBatch sb, Level lv, Vector2 cam,
        float zoom, TerrainColors col, float t)
    {
        if (C.Sky.Length == 0 || !Visible(lv, out int s, out int e))
            return;

        Vector2 vp = ViewPos(cam);
        float th = C.Cfg.TerrainStroke;
        float sc = Math.Clamp(zoom, ZLo, ZHi) * ZK;
        int pts = C.Cfg.FillPoints;
        (Vector2 dn, Vector2 df) = Project(Point(lv, s), vp);

        for (int i = s; i < e; i++)
        {
            (Vector2 nn, Vector2 nf) = Project(Point(lv, i + 1), vp);
            sb.QuadFill(dn, df, nn, nf, col.Fill, sc, i, pts)
              .QuadFrame(dn, df, nn, nf, col.Stroke, th);
            (dn, df) = (nn, nf);
        }

        Depth(sb, lv, s, e, vp);
        DrawFlags(sb, lv, s, e, vp, t, near: false);
    }

    public void NearFlags(SpriteBatch sb, Level lv, Vector2 cam, float t)
    {
        if (!Visible(lv, out int s, out int e))
            return;
        DrawFlags(sb, lv, s, e, ViewPos(cam), t, near: true);
    }

    static void DrawFlags(
        SpriteBatch sb, Level lv, int s, int e,
        Vector2 vp, float t, bool near)
    {
        (Vector2 sn, Vector2 sf) = Project(lv.StartGatePos, vp);
        (Vector2 fn, Vector2 ff) = Project(lv.FinishGatePos, vp);
        Flags.Draw(lv, s, e, near ? sn : sf, near ? fn : ff, t, Ctx2D(sb));
    }

    void Depth(SpriteBatch sb, Level lv, int s, int e, Vector2 vp)
    {
        int step = Math.Max(1, (e - s) / P.DepthDiv);
        int end = Math.Min(e, lv.PtCount - 1);
        Color col = C.Theme.VLineColor;
        float th = C.Cfg.VerticalStroke;

        for (int i = s; i <= end; i += step)
        {
            (Vector2 ln, Vector2 lf) = Project(Point(lv, i), vp);
            sb.DrawLine(ln, lf, col, th);
        }
    }
}
