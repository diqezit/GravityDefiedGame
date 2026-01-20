namespace GravityDefiedGame.Views.Terrain;

public readonly struct SunCfg
{
    public readonly float Radius, GlowK, ThickK, PosX, PosY;
    public readonly int Segments;
    public readonly Color Glow, Core;

    public SunCfg()
    {
        Radius = 40f;
        GlowK = 1.3f;
        ThickK = 0.5f;
        PosX = 0.35f;
        PosY = 0.2f;
        Segments = 16;
        Glow = new(255, 178, 48, 76);
        Core = new(255, 218, 98);
    }
}

public readonly struct StarCfg
{
    public readonly float HeightK, AlphaLo, AlphaHi,
        SizeLo, SizeHi;

    public StarCfg()
    {
        HeightK = 0.7f;
        AlphaLo = 0.3f;
        AlphaHi = 1f;
        SizeLo = 0.6f;
        SizeHi = 2f;
    }
}

public readonly struct FillCfg
{
    public readonly float PointSize, ZoomLo, ZoomHi;
    public readonly uint SeedK;

    public FillCfg()
    {
        PointSize = 1.5f;
        ZoomLo = 0.5f;
        ZoomHi = 1f;
        SeedK = 1234567u;
    }
}

public readonly struct GradientCfg
{
    public readonly int MinPalette;
    public readonly float BandPad;

    public GradientCfg()
    {
        MinPalette = 2;
        BandPad = 1f;
    }
}

public readonly struct HashCfg
{
    public readonly uint SeedK;
    public readonly uint A, B, C;

    public HashCfg()
    {
        SeedK = 19349663u;
        A = 0xB5297A4D;
        B = 0x68E31DA4;
        C = 0x1B56C4E9;
    }
}

public readonly struct ProjCfg
{
    public readonly float HalfK;
    public readonly int DepthDiv;

    public ProjCfg()
    {
        HalfK = 0.5f;
        DepthDiv = 30;
    }
}

public readonly record struct PerspectiveCfg(
    float Depth, float FocalLen,
    float HorizonOff, float MaxOffset)
{
    public float Ratio => Depth / FocalLen;

    public static readonly PerspectiveCfg Default = new(
        Depth: 30f, FocalLen: 400f,
        HorizonOff: -80f, MaxOffset: 50f);
}

public sealed partial class WorldRenderer
{
    static readonly PerspectiveCfg Persp = PerspectiveCfg.Default;
    static readonly SunCfg Sun = new();
    static readonly StarCfg Star = new();
    static readonly FillCfg Fill = new();
    static readonly GradientCfg Grad = new();
    static readonly HashCfg Hash = new();
    static readonly ProjCfg Proj = new();

    RenderCfg _cfg = ThemeManager.Rendering;
    ThemeSettings _theme = ThemeManager.Cur;
    Color[] _sky = [];

    public void Init() => Reload();
    public void OnTheme() => Reload();

    void Reload()
    {
        (_cfg, _theme) =
            (ThemeManager.Rendering, ThemeManager.Cur);
        _sky = ThemeManager.ParsePal(
            _theme.RenderCfg.SkyPal);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static bool Fin(Vector2 v) =>
        float.IsFinite(v.X) && float.IsFinite(v.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static uint Seed(int i) =>
        unchecked((uint)i * Hash.SeedK);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Rng(uint s)
    {
        s *= Hash.A;
        s ^= s >> 8;
        s += Hash.B;
        s ^= s << 8;
        s *= Hash.C;
        s ^= s >> 8;
        return s * (1f / 4294967296f);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Rng(uint s, float lo, float hi) =>
        lo + Rng(s) * (hi - lo);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ProjectSpan(
        float x, float y, float vx, float vy,
        out float nx, out float ny,
        out float fx, out float fy)
    {
        float r = Persp.Ratio * Proj.HalfK;
        float dx = Math.Clamp(
            (vx - x) * r,
            -Persp.MaxOffset, Persp.MaxOffset);
        float dy = Math.Clamp(
            (vy - y) * r,
            -Persp.MaxOffset, Persp.MaxOffset);
        (nx, ny) = (x - dx, y - dy);
        (fx, fy) = (x + dx, y + dy);
    }
}

public sealed partial class WorldRenderer
{
    public void Sky(SpriteBatch sb)
    {
        Viewport vp = sb.GraphicsDevice.Viewport;
        float w = vp.Width, h = vp.Height;
        DrawGradient(sb, w, h);
        if (_theme.RenderCfg.HasSun)
            DrawSun(sb, w, h);
        if (_theme.RenderCfg.HasStars)
            DrawStars(sb, w, h);
    }

    void DrawGradient(SpriteBatch sb, float w, float h)
    {
        if (_sky.Length < Grad.MinPalette
            || _cfg.GradientSteps <= 0)
            return;

        float bandH = h / _cfg.GradientSteps + Grad.BandPad;
        int last = _sky.Length - 1;

        for (int i = 0; i < _cfg.GradientSteps; i++)
        {
            float k = (float)i / _cfg.GradientSteps;
            float pos = k * last;
            int idx = Math.Min((int)pos, last - 1);
            sb.FillRectangle(0, h * k, w, bandH,
                Color.Lerp(_sky[idx], _sky[idx + 1],
                    pos - idx));
        }
    }

    static void DrawSun(SpriteBatch sb, float w, float h)
    {
        float sx = w * Sun.PosX, sy = h * Sun.PosY;
        sb.DrawCircle(new(sx, sy),
            Sun.Radius * Sun.GlowK, Sun.Segments,
            Sun.Glow, Sun.Radius * Sun.ThickK);
        sb.DrawCircle(new(sx, sy),
            Sun.Radius, Sun.Segments,
            Sun.Core, Sun.Radius);
    }

    void DrawStars(SpriteBatch sb, float w, float h)
    {
        int cnt = _cfg.StarCount;
        if (cnt <= 0)
            return;

        for (int i = 0; i < cnt; i++)
        {
            uint s = Seed(i);
            byte a = (byte)(Rng(s + 3,
                Star.AlphaLo, Star.AlphaHi) * 255f);
            Color col = new(
                (byte)255, (byte)255, (byte)255, a);
            sb.DrawPoint(
                Rng(s) * w,
                Rng(s + 1) * h * Star.HeightK,
                col,
                Rng(s + 2, Star.SizeLo, Star.SizeHi));
        }
    }
}

public sealed partial class WorldRenderer
{
    public void Terrain(
        SpriteBatch sb, Level lv, Vector2 cam,
        float zoom, TerrainColors col)
    {
        (int s, int e) = lv.GetVisibleRange();
        int last = lv.PtCount - 1;
        s = Math.Clamp(s, 0, last);
        e = Math.Clamp(e, 0, last);
        if (s >= e)
            return;

        float vx = cam.X;
        float vy = cam.Y + Persp.HorizonOff;
        float th = _cfg.TerrainStroke;

        for (int i = s; i < e; i++)
        {
            float x0 = lv.GetPtX(i), y0 = lv.GetPtY(i);
            float x1 = lv.GetPtX(i + 1),
                  y1 = lv.GetPtY(i + 1);

            ProjectSpan(x0, y0, vx, vy,
                out float nx0, out float ny0,
                out float fx0, out float fy0);
            ProjectSpan(x1, y1, vx, vy,
                out float nx1, out float ny1,
                out float fx1, out float fy1);

            Vector2 n0 = new(nx0, ny0),
                    n1 = new(nx1, ny1),
                    f0 = new(fx0, fy0),
                    f1 = new(fx1, fy1);

            if (!Fin(n0) || !Fin(n1)
                || !Fin(f0) || !Fin(f1))
                continue;

            DrawFill(sb, n0, n1, f0, f1, col.Fill, zoom);
            sb.DrawLine(n0, n1, col.Stroke, th);
            sb.DrawLine(f0, f1, col.Stroke, th);
            sb.DrawLine(n0, f0, col.Stroke, th);
            sb.DrawLine(n1, f1, col.Stroke, th);
        }

        DrawDepthLines(sb, lv, s, e, vx, vy);
    }

    void DrawDepthLines(
        SpriteBatch sb, Level lv,
        int s, int e, float vx, float vy)
    {
        float th = _cfg.VerticalStroke;
        Color col = _theme.VLineColor;
        int end = Math.Min(e, lv.PtCount - 1);
        int step = Math.Max(1, (e - s) / Proj.DepthDiv);

        for (int i = s; i <= end; i += step)
        {
            float x = lv.GetPtX(i), y = lv.GetPtY(i);
            if (!float.IsFinite(x)
                || !float.IsFinite(y))
                continue;

            ProjectSpan(x, y, vx, vy,
                out float nx, out float ny,
                out float fx, out float fy);
            sb.DrawLine(nx, ny, fx, fy, col, th);
        }
    }

    void DrawFill(
        SpriteBatch sb, Vector2 a0, Vector2 a1,
        Vector2 b0, Vector2 b1, Color col, float zoom)
    {
        int pts = _cfg.FillPoints;
        if (pts <= 0)
            return;
        float sc = Math.Clamp(
            zoom, Fill.ZoomLo, Fill.ZoomHi);

        for (int i = 0; i < pts; i++)
        {
            uint seed = (uint)i * Fill.SeedK;
            float tx = Rng(seed), ty = Rng(seed + 1);
            var p = Vector2.Lerp(
                Vector2.Lerp(a0, a1, tx),
                Vector2.Lerp(b0, b1, tx), ty);
            sb.DrawPoint(p, col, Fill.PointSize * sc);
        }
    }
}
