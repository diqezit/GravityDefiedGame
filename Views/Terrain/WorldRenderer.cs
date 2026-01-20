namespace GravityDefiedGame.Views.Terrain;

public readonly record struct PerspectiveCfg(
    float Depth,
    float FocalLen,
    float HorizonOff,
    float MaxOffset)
{
    public float Ratio => Depth / FocalLen;

    public static readonly PerspectiveCfg Default = new(
        Depth: 30f,
        FocalLen: 400f,
        HorizonOff: -80f,
        MaxOffset: 50f);
}

public sealed partial class WorldRenderer
{
    static readonly PerspectiveCfg Persp = PerspectiveCfg.Default;

    RenderCfg _cfg = ThemeManager.Rendering;
    ThemeSettings _theme = ThemeManager.Cur;
    Color[] _sky = [];

    public void Init() => Reload();
    public void OnTheme() => Reload();

    void Reload()
    {
        (_cfg, _theme) = (ThemeManager.Rendering, ThemeManager.Cur);
        _sky = ThemeManager.ParsePal(_theme.RenderCfg.SkyPal);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Color Col(byte r, byte g, byte b, byte a = 255) => new(r, g, b, a);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static bool Fin(Vector2 v) => float.IsFinite(v.X) && float.IsFinite(v.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static uint Seed(int i) => unchecked((uint)i * 19349663u);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Rng(uint s)
    {
        s *= 0xB5297A4D;
        s ^= s >> 8;
        s += 0x68E31DA4;
        s ^= s << 8;
        s *= 0x1B56C4E9;
        s ^= s >> 8;
        return s * (1f / 4294967296f);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Rng(uint s, float lo, float hi) => lo + Rng(s) * (hi - lo);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static void ProjectSpan(float x, float y, float vanishX, float vanishY,
                            out float nx, out float ny, out float fx, out float fy)
    {
        float r = Persp.Ratio * 0.5f;

        float dx = (vanishX - x) * r;
        float dy = (vanishY - y) * r;

        dx = Math.Clamp(dx, -Persp.MaxOffset, Persp.MaxOffset);
        dy = Math.Clamp(dy, -Persp.MaxOffset, Persp.MaxOffset);

        nx = x - dx;
        ny = y - dy;

        fx = x + dx;
        fy = y + dy;
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
        if (_sky.Length < 2 || _cfg.GradientSteps <= 0)
            return;

        float bandH = h / _cfg.GradientSteps + 1f;
        int last = _sky.Length - 1;

        for (int i = 0; i < _cfg.GradientSteps; i++)
        {
            float k = (float)i / _cfg.GradientSteps;
            float pos = k * last;
            int idx = Math.Min((int)pos, last - 1);

            sb.FillRectangle(0, h * k, w, bandH,
                Color.Lerp(_sky[idx], _sky[idx + 1], pos - idx));
        }
    }

    static void DrawSun(SpriteBatch sb, float w, float h)
    {
        const float sz = 40f;
        float sx = w * 0.35f, sy = h * 0.2f;

        sb.DrawCircle(new(sx, sy), sz * 1.3f, 16, Col(255, 178, 48, 76), sz * 0.5f);
        sb.DrawCircle(new(sx, sy), sz, 16, Col(255, 218, 98), sz);
    }

    void DrawStars(SpriteBatch sb, float w, float h)
    {
        int cnt = _cfg.StarCount;
        if (cnt <= 0)
            return;

        for (int i = 0; i < cnt; i++)
        {
            uint s = Seed(i);

            sb.DrawPoint(
                Rng(s) * w,
                Rng(s + 1) * h * 0.7f,
                Col(255, 255, 255, (byte)(Rng(s + 3, 0.3f, 1f) * 255f)),
                Rng(s + 2, 0.6f, 2f));
        }
    }
}

public sealed partial class WorldRenderer
{
    public void Terrain(SpriteBatch sb, Level lv, Vector2 cam, float zoom, TerrainColors col)
    {
        (int s, int e) = lv.GetVisibleRange();
        int last = lv.PtCount - 1;

        s = Math.Clamp(s, 0, last);
        e = Math.Clamp(e, 0, last);

        if (s >= e)
            return;

        float vanishX = cam.X;
        float vanishY = cam.Y + Persp.HorizonOff;
        float th = _cfg.TerrainStroke;

        for (int i = s; i < e; i++)
        {
            float x0 = lv.GetPtX(i), y0 = lv.GetPtY(i);
            float x1 = lv.GetPtX(i + 1), y1 = lv.GetPtY(i + 1);

            ProjectSpan(x0, y0, vanishX, vanishY, out float nx0, out float ny0, out float fx0, out float fy0);
            ProjectSpan(x1, y1, vanishX, vanishY, out float nx1, out float ny1, out float fx1, out float fy1);

            Vector2 n0 = new(nx0, ny0);
            Vector2 n1 = new(nx1, ny1);
            Vector2 f0 = new(fx0, fy0);
            Vector2 f1 = new(fx1, fy1);

            if (!Fin(n0) || !Fin(n1) || !Fin(f0) || !Fin(f1))
                continue;

            DrawFill(sb, n0, n1, f0, f1, col.Fill, zoom);

            sb.DrawLine(n0, n1, col.Stroke, th);
            sb.DrawLine(f0, f1, col.Stroke, th);
            sb.DrawLine(n0, f0, col.Stroke, th);
            sb.DrawLine(n1, f1, col.Stroke, th);
        }

        DrawDepthLines(sb, lv, s, e, vanishX, vanishY);
    }

    void DrawDepthLines(SpriteBatch sb, Level lv, int s, int e, float vanishX, float vanishY)
    {
        float th = _cfg.VerticalStroke;
        Color col = _theme.VLineColor;

        int end = Math.Min(e, lv.PtCount - 1);
        int step = Math.Max(1, (e - s) / 30);

        for (int i = s; i <= end; i += step)
        {
            float x = lv.GetPtX(i), y = lv.GetPtY(i);

            if (!float.IsFinite(x) || !float.IsFinite(y))
                continue;

            ProjectSpan(x, y, vanishX, vanishY, out float nx, out float ny, out float fx, out float fy);
            sb.DrawLine(nx, ny, fx, fy, col, th);
        }
    }

    void DrawFill(SpriteBatch sb, Vector2 a0, Vector2 a1, Vector2 b0, Vector2 b1, Color col, float zoom)
    {
        int pts = _cfg.FillPoints;
        if (pts <= 0)
            return;

        float sc = Math.Clamp(zoom, 0.5f, 1f);

        for (int i = 0; i < pts; i++)
        {
            uint seed = (uint)i * 1234567u;
            float tx = Rng(seed);
            float ty = Rng(seed + 1);

            var p = Vector2.Lerp(
                Vector2.Lerp(a0, a1, tx),
                Vector2.Lerp(b0, b1, tx), ty);

            sb.DrawPoint(p, col, 1.5f * sc);
        }
    }
}
