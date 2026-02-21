using static System.MathF;
using static GravityDefiedGame.Views.Terrain.Flags;
using static GravityDefiedGame.Views.Terrain.SkyDraw;

namespace GravityDefiedGame.Views.Terrain;

public readonly struct FlagCfg
{
    public readonly float PoleTh, LineTh, PoleH, FlagW, FlagH;
    public readonly float WaveSpd, WaveFreq, WaveAmp, CellSz;
    public readonly int CheckerN;
    public readonly Color Pole, Start;

    public FlagCfg()
    {
        (PoleTh, LineTh) = (2.5f, 1.25f);
        (PoleH, FlagW, FlagH) = (55f, 22f, 16f);
        (WaveSpd, WaveFreq, WaveAmp) = (3.5f, 0.35f, 6f);
        (CellSz, CheckerN) = (5f, 4);
        (Pole, Start) = (new(140, 140, 140), new(0, 200, 0));
    }
}

public readonly struct SunCfg
{
    public readonly float PosX, PosY;
    public readonly int R1, R2, Segments;
    public readonly Color Glow, Core;

    public SunCfg()
    {
        (PosX, PosY) = (0.35f, 0.2f);
        (R1, R2, Segments) = (52, 40, 16);
        Glow = new(255, 178, 48, 76);
        Core = new(255, 218, 98);
    }
}

public readonly struct StarCfg
{
    public readonly float HeightK, AlphaLo, AlphaHi, SizeLo, SizeHi;
    public readonly int Seed;

    public StarCfg()
    {
        HeightK = 0.7f;
        (AlphaLo, AlphaHi) = (0.3f, 0.7f);
        (SizeLo, SizeHi) = (0.6f, 1.4f);
        Seed = unchecked((int)19349663u);
    }
}

public readonly struct ProjCfg
{
    public readonly float HorizonOfs, ProjR, MaxOfs, ZNear, ZFar;
    public readonly int DepthDiv, SkyBands, ExtraRange;

    public ProjCfg()
    {
        (HorizonOfs, ProjR, MaxOfs) = (80f, 30f / 400f * 0.5f, 50f);
        (ZNear, ZFar) = (-15f, 15f);
        (DepthDiv, SkyBands, ExtraRange) = (30, 12, 40);
    }
}

public readonly record struct RenderCtx(RenderCfg Cfg, ThemeSettings Theme, Color[] Sky)
{
    public static RenderCtx Load() => new(
        ThemeManager.Rendering, ThemeManager.Cur,
        ThemeManager.ParsePal(ThemeManager.Cur.RenderCfg.SkyPal));
}

public delegate void LineFn(Vector2 a, Vector2 b, float th, Color col);

public static class Flags
{
    static readonly FlagCfg F = new();

    public static bool TryVisRange(Level lv, out int s, out int e)
    {
        (s, e) = lv.GetVisibleRange();
        int max = lv.PtCount - 1;

        if (max < 0)
        { s = e = 0; return false; }

        s = Math.Clamp(s, 0, max);
        e = Math.Clamp(e, 0, max);
        return s < e;
    }

    public static Vector2 Pt(Level lv, int i) => new(lv.GetPtX(i), lv.GetPtY(i));

    public static void Draw2D(
        SpriteBatch sb, Level lv, int s, int e,
        Vector2 sp, Vector2 fp, float time) =>
        DrawBoth(lv, s, e, sp, fp, time, (a, b, th, col) => sb.DrawLine(a, b, col, th));

    public static void Draw3D(
        VoxMesh m, float z, Level lv, int s, int e,
        Vector2 sp, Vector2 fp, float time) =>
        DrawBoth(lv, s, e, sp, fp, time, (a, b, th, col) => m.Line(a, b, z, th, col));

    static void DrawBoth(
        Level lv, int s, int e,
        Vector2 sp, Vector2 fp, float time, LineFn line)
    {
        if (Vis(lv, s, e, lv.StartGatePos.X))
            DrawFlag(sp, time, false, line);

        if (Vis(lv, s, e, lv.FinishGatePos.X))
            DrawFlag(fp, time, true, line);
    }

    static bool Vis(Level lv, int s, int e, float x) =>
        x >= lv.GetPtX(s) && x <= lv.GetPtX(e);

    static void DrawFlag(Vector2 g, float t, bool finish, LineFn line)
    {
        Vector2 top = new(g.X, g.Y - F.PoleH);
        line(g, top, F.PoleTh, F.Pole);

        if (finish)
            DrawChecker(top, line);
        else
            DrawWave(top, t, line);
    }

    static void DrawWave(Vector2 top, float t, LineFn line)
    {
        float halfH = F.FlagH * 0.5f;

        for (int y = 0; y < (int)F.FlagH; y++)
        {
            float w = F.FlagW * (1f - Abs(y / halfH - 1f));
            if (w < 0.5f)
                continue;

            float wave = Sin(t * F.WaveSpd + y * F.WaveFreq) * F.WaveAmp * y / F.FlagH;
            float x1 = Max(top.X + 1f, top.X + w + wave);
            line(new(top.X, top.Y + y), new(x1, top.Y + y), F.LineTh, F.Start);
        }
    }

    static void DrawChecker(Vector2 top, LineFn line)
    {
        for (int r = 0; r < F.CheckerN; r++)
            for (int c = 0; c < F.CheckerN; c++)
            {
                float x0 = top.X + c * F.CellSz;
                float y0 = top.Y + r * F.CellSz + F.CellSz * 0.5f;
                Color col = (r + c) % 2 == 0 ? Color.White : Color.Black;
                line(new(x0, y0), new(x0 + F.CellSz, y0), F.CellSz, col);
            }
    }
}

public static class SkyDraw
{
    static readonly SunCfg Sun = new();
    static readonly StarCfg Star = new();

    public static void Gradient(SpriteBatch sb, int w, int h, Color[] pal, int bands)
    {
        if (pal.Length < 2)
            return;

        int last = pal.Length - 1;
        float bandH = h / (float)bands;

        for (int i = 0; i < bands; i++)
        {
            float t = (i + 0.5f) / bands, pos = t * last;
            int j = Math.Min((int)pos, last - 1);
            sb.FillRectangle(0f, i * bandH, w, bandH + 1f,
                Color.Lerp(pal[j], pal[j + 1], pos - j));
        }
    }

    public static void DrawSun(SpriteBatch sb, int w, int h)
    {
        Vector2 c = new(w * Sun.PosX, h * Sun.PosY);
        sb.DrawCircle(c, Sun.R1, Sun.Segments, Sun.Glow, 20f);
        sb.DrawCircle(c, Sun.R2, Sun.Segments, Sun.Core, 40f);
    }

    public static void DrawStars(SpriteBatch sb, int w, int h, int count)
    {
        Random rnd = new(Star.Seed);
        float maxY = h * Star.HeightK;
        float dA = Star.AlphaHi - Star.AlphaLo, dS = Star.SizeHi - Star.SizeLo;

        for (int i = 0; i < count; i++)
        {
            float x = rnd.NextSingle() * w, y = rnd.NextSingle() * maxY;
            float a = Star.AlphaLo + rnd.NextSingle() * dA;
            float sz = Star.SizeLo + rnd.NextSingle() * dS;
            sb.DrawPoint(x, y, new Color(255, 255, 255, (int)(a * 255f)), sz);
        }
    }
}

public static class TerrainExt
{
    public static SpriteBatch Quad(
        this SpriteBatch sb, Vector2 an, Vector2 af,
        Vector2 bn, Vector2 bf, Color col, float th)
    {
        sb.DrawLine(an, bn, col, th);
        sb.DrawLine(af, bf, col, th);
        sb.DrawLine(an, af, col, th);
        sb.DrawLine(bn, bf, col, th);
        return sb;
    }
}

public sealed class VoxelWorldRenderer
{
    static readonly ProjCfg P = new();

    RenderCtx _ctx;

    public void Init() => _ctx = RenderCtx.Load();
    public void OnTheme() => _ctx = RenderCtx.Load();

    public static void Sky(SpriteBatch sb, int w, int h, Color[] sky) =>
        Gradient(sb, w, h, sky, P.SkyBands);

    public void Sky(SpriteBatch sb, int w, int h) =>
        Sky(sb, w, h, _ctx.Sky);

    public void Terrain(VoxMesh m, Level lv, TerrainColors col, float th, float time) =>
        Terrain(m, lv, col, th, time, P.ExtraRange);

    public void Terrain(
        VoxMesh m, Level lv, TerrainColors col, float th, float time, int extra)
    {
        if (!TryVisRange(lv, out int s, out int e))
            return;

        e = Math.Min(e + extra, lv.PtCount - 1);
        Vector2 p = Pt(lv, s);
        ReadOnlySpan<float> layers = [P.ZNear, P.ZFar];

        for (int i = s; i < e; i++)
        {
            Vector2 q = Pt(lv, i + 1);
            m.CrossZ(p, P.ZNear, P.ZFar, th, col.Stroke);
            m.FillZ(p, q, P.ZNear, P.ZFar, col.Fill);

            foreach (float z in layers)
                m.Line(p, q, z, th, col.Stroke);

            p = q;
        }

        m.CrossZ(p, P.ZNear, P.ZFar, th, col.Stroke);

        foreach (float z in layers)
            Draw3D(m, z, lv, s, e, lv.StartGatePos, lv.FinishGatePos, time);
    }
}

public sealed class WorldRenderer
{
    static readonly ProjCfg P = new();
    const float Phi = 0.618033988f;

    RenderCtx _ctx;

    public void Init() => _ctx = RenderCtx.Load();
    public void OnTheme() => _ctx = RenderCtx.Load();

    public void Sky(SpriteBatch sb, int w, int h)
    {
        Gradient(sb, w, h, _ctx.Sky, _ctx.Cfg.GradientSteps);

        ThemeRenderCfg rc = _ctx.Theme.RenderCfg;
        if (rc.HasSun)
            DrawSun(sb, w, h);
        if (rc.HasStars)
            DrawStars(sb, w, h, _ctx.Cfg.StarCount);
    }

    public void Terrain(
        SpriteBatch sb, Level lv, Vector2 cam,
        float zoom, TerrainColors col, float time)
    {
        if (_ctx.Sky.Length == 0 || !TryVisRange(lv, out int s, out int e))
            return;

        Vector2 vp = Vp(cam);
        float th = _ctx.Cfg.TerrainStroke;
        float sc = Math.Clamp(zoom, 0.5f, 1.5f) * 1.5f;
        int pts = _ctx.Cfg.FillPoints;

        (Vector2 dn, Vector2 df) = Proj(Pt(lv, s), vp);

        for (int i = s; i < e; i++)
        {
            (Vector2 nn, Vector2 nf) = Proj(Pt(lv, i + 1), vp);
            Fill(sb, dn, df, nn, nf, col.Fill, sc, i, pts);
            sb.Quad(dn, df, nn, nf, col.Stroke, th);
            (dn, df) = (nn, nf);
        }

        DrawDepth(sb, lv, s, e, vp);
        DrawFlags(sb, lv, s, e, vp, time, near: false);
    }

    public void DrawNearFlags(SpriteBatch sb, Level lv, Vector2 cam, float time)
    {
        if (!TryVisRange(lv, out int s, out int e))
            return;
        DrawFlags(sb, lv, s, e, Vp(cam), time, near: true);
    }

    static void DrawFlags(
        SpriteBatch sb, Level lv, int s, int e,
        Vector2 vp, float time, bool near)
    {
        (Vector2 sn, Vector2 sf) = Proj(lv.StartGatePos, vp);
        (Vector2 fn, Vector2 ff) = Proj(lv.FinishGatePos, vp);
        Draw2D(sb, lv, s, e, near ? sn : sf, near ? fn : ff, time);
    }

    void DrawDepth(SpriteBatch sb, Level lv, int s, int e, Vector2 vp)
    {
        int step = Math.Max(1, (e - s) / P.DepthDiv);
        int end = Math.Min(e, lv.PtCount - 1);
        Color col = _ctx.Theme.VLineColor;
        float th = _ctx.Cfg.VerticalStroke;

        for (int i = s; i <= end; i += step)
        {
            (Vector2 ln, Vector2 lf) = Proj(Pt(lv, i), vp);
            sb.DrawLine(ln, lf, col, th);
        }
    }

    static void Fill(
        SpriteBatch sb, Vector2 an, Vector2 af,
        Vector2 bn, Vector2 bf, Color col, float sc, int seg, int n)
    {
        for (int j = 0; j < n; j++)
        {
            int idx = seg * n + j;
            float tx = (idx * Phi) % 1f;
            float ty = (idx * Phi * Phi) % 1f;

            var top = Vector2.Lerp(an, bn, tx);
            var bot = Vector2.Lerp(af, bf, tx);
            sb.DrawPoint(Vector2.Lerp(top, bot, ty), col, sc);
        }
    }

    static Vector2 Vp(Vector2 cam) => new(cam.X, cam.Y - P.HorizonOfs);

    static (Vector2 near, Vector2 far) Proj(Vector2 pt, Vector2 vp)
    {
        float dx = Math.Clamp((vp.X - pt.X) * P.ProjR, -P.MaxOfs, P.MaxOfs);
        float dy = Math.Clamp((vp.Y - pt.Y) * P.ProjR, -P.MaxOfs, P.MaxOfs);
        return (new(pt.X - dx, pt.Y - dy), new(pt.X + dx, pt.Y + dy));
    }
}
