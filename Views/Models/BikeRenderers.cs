namespace GravityDefiedGame.Views.Models;

public readonly record struct BikeColors(
    Color Frame, Color Wheel, Color Fork, Color Shock,
    Color Rider, Color Leg, Color Body, Color Fist, float Thickness)
{
    public static BikeColors From(Dictionary<LineType, Color> d, float th = 2.5f)
    {
        d ??= [];
        return new(
            d.GetValueOrDefault(LineType.Frame, Color.Gray),
            d.GetValueOrDefault(LineType.Wheel, Color.Black),
            d.GetValueOrDefault(LineType.Fork, Color.Gray),
            d.GetValueOrDefault(LineType.Shock, Color.Gray),
            d.GetValueOrDefault(LineType.Rider, Color.Black),
            d.GetValueOrDefault(LineType.Leg, Color.Black),
            d.GetValueOrDefault(LineType.Body, new(0, 0, 128)),
            d.GetValueOrDefault(LineType.Fist, new(156, 0, 0)),
            th);
    }
}

public static class BikeSpecs
{
    public const float Eps = 0.01f, Sc = 6f, HelmTilt = 0.31f;
    public const float FistR = 1.33f, WhlIn = 0.82f, SpkS = 0.2f;
    public const float WhlThRimK = 0.65f, WhlThSpkK = 0.45f, TiltMid = 0.5f, RimMarkK = 1.2f;
    public const int WhlN = 16, SpkN = 5, HdN = 8;

    // Sprite indices
    public const int ArmBrace = 0, Head = 1, Body = 2, LegFull = 3,
        ArmUp = 4, Helmet = 5, Engine = 6, Fender = 7;

    // Render mask
    public const int MaskBike = 1, MaskRider = 2, MaskAll = 3;

    // Pose: [0]=Hip [1]=Sh [2]=Elb [3]=Hnd [4]=Nk [5]=Ft [6]=Lg [7]=Hd
    public static readonly Vector2[] PoseL =
    [
        new(2.90f, -1.40f), new(3.90f, -3.60f), new(5.10f, -1.80f), new(6.00f, -0.70f),
        new(4.60f,  0.10f), new(1.00f, -1.20f), new(0.20f, -1.20f), new(4.40f,  1.30f)
    ];

    public static readonly Vector2[] PoseN =
    [
        new(2.80f, -0.60f), new(4.00f, -2.00f), new(6.00f, -1.00f), new(7.00f, -0.60f),
        new(4.50f,  0.10f), new(0.30f, -2.20f), new(0.20f, -1.20f), new(4.40f,  1.30f)
    ];

    public static readonly Vector2[] PoseR =
    [
        new(2.40f,  0.20f), new(4.50f, -0.20f), new(5.60f,  1.60f), new(6.20f,  2.70f),
        new(5.30f,  1.10f), new(0.60f, -1.50f), new(0.20f, -0.80f), new(4.40f,  1.30f)
    ];
}

public sealed partial class BikeRenderer : IDisposable
{
    readonly record struct Spr(Rectangle R, Vector2 P0, Vector2 P1, float AspectK)
    {
        public bool IsVert => MathF.Abs(P1.Y - P0.Y) > MathF.Abs(P1.X - P0.X);
    }

    readonly record struct Rider(
        Vector2 Hip, Vector2 Sh, Vector2 Elb, Vector2 Hnd,
        Vector2 Nk, Vector2 Ft, Vector2 Lg, Vector2 Hd);

    static readonly Spr[] _spr =
    [
        new(new(773, 132, 274, 92),   new(5.73f, 45.23f),    new(269.04f, 45.23f),  1.00f), // ArmBrace
        new(new(1155, 132, 239, 92),  new(13.5f, 48f),       new(223.5f,  46f),     1.00f), // Head
        new(new(60, 496, 247, 92),    new(1.66f, 46f),       new(246.11f, 43.69f),  0.85f), // Body
        new(new(421, 496, 250, 92),   new(20f, 46f),         new(250f,    46f),     0.85f), // LegFull
        new(new(79, 41, 206, 285),    new(113.51f, 279.13f), new(79.35f,  11.13f),  0.80f), // ArmUp
        new(new(409, 41, 274, 263),   new(140.09f, 95.91f),  new(249f,   -40.5f),   0.90f), // Helmet
        new(new(739, 417, 331, 239),  new(253.5f, 139.5f),   new(529.5f, 115.5f),   1.00f), // Engine
        new(new(1126, 485, 302, 103), new(277f, 161.5f),     new(481f,   183.5f),   1.00f), // Fender
    ];

    Texture2D? _tex;
    bool _disposed;

    public bool SpritesLoaded => _tex is not null;

    public void LoadSprites(GraphicsDevice gd, string dir = "Content/Assets")
    {
        if (_disposed || gd is null)
            return;
        _tex?.Dispose();
        _tex = LoadTex(gd, Path.Combine(dir, "sprites.png"));
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        _tex?.Dispose();
        _tex = null;
    }

    public static void Render(SpriteBatch sb, in BikeVisual v, in BikeColors c)
    {
        if (sb is null)
            return;

        float th = c.Thickness;
        DrawBikeFrame(sb, in v, in c, th, drawFrameLines: true);

        Rider r = Build(in v);
        sb.DrawLine(r.Ft, r.Hip, c.Leg, th);
        sb.DrawLine(r.Hip, r.Sh, c.Leg, th);
        sb.DrawLine(r.Sh, r.Elb, c.Body, th);
        sb.DrawLine(r.Elb, r.Nk, c.Body, th);
        sb.DrawLine(r.Nk, r.Hd, c.Body, th);
        DrawRing(sb, r.Hnd, BikeSpecs.FistR, BikeSpecs.HdN, 0f, c.Fist, th);
    }

    public void RenderSprites(SpriteBatch sb, in BikeVisual v, in BikeColors c, int mask = BikeSpecs.MaskAll)
    {
        Texture2D? tex = _tex;
        if (_disposed || tex is null || sb is null)
            return;

        float th = c.Thickness;

        // In sprite mode, we do not draw frame lines—only wheels, fork, and shock absorber
        DrawBikeFrame(sb, in v, in c, th, drawFrameLines: false);

        float bikeAng = Ang(v.Ur, v.Uf);
        float upLen = SafeLen(v.Uf.X - v.Ur.X, v.Uf.Y - v.Ur.Y);

        if ((mask & BikeSpecs.MaskBike) != 0)
        {
            Vector2 eng = Mid(v.Fr, v.Uf), fnd = Mid(v.Fr, v.Ur);
            DrawAtAng(sb, tex, BikeSpecs.Engine, eng, bikeAng, upLen, c.Frame);
            DrawAtAng(sb, tex, BikeSpecs.Fender, fnd, bikeAng, upLen, c.Frame);
        }

        if ((mask & BikeSpecs.MaskRider) == 0)
            return;

        Rider r = Build(in v);
        float tilt = v.TiltZ > BikeSpecs.TiltMid ? BikeSpecs.HelmTilt : 0f;

        DrawSeg(sb, tex, BikeSpecs.ArmBrace, r.Nk, r.Elb, c.Body);
        DrawSeg(sb, tex, BikeSpecs.Head, r.Nk, r.Hd, c.Body);
        DrawSeg(sb, tex, BikeSpecs.Body, r.Hip, r.Sh, c.Body);
        DrawSeg(sb, tex, BikeSpecs.LegFull, r.Hip, r.Ft, c.Leg);
        DrawSeg(sb, tex, BikeSpecs.ArmUp, r.Sh, r.Elb, c.Body);

        float helmLen = SafeLen(r.Hnd.X - r.Elb.X, r.Hnd.Y - r.Elb.Y);
        float armAng = Ang(r.Elb, r.Hnd);
        DrawAtAng(sb, tex, BikeSpecs.Helmet, r.Hnd, armAng + tilt, helmLen, c.Rider);
    }

    static void DrawBikeFrame(SpriteBatch sb, in BikeVisual v, in BikeColors c, float th, bool drawFrameLines)
    {
        DrawWheel(sb, v.Fw, v.WhlR, v.FwRot, c.Wheel, th);
        DrawWheel(sb, v.Rw, v.WhlR, v.RwRot, c.Wheel, th);
        sb.DrawLine(v.Uf, v.Fw, c.Fork, th);
        sb.DrawLine(v.Ur, v.Rw, c.Shock, th);

        if (drawFrameLines)
        {
            sb.DrawLine(v.Fr, v.Uf, c.Frame, th);
            sb.DrawLine(v.Fr, v.Ur, c.Frame, th);
            sb.DrawLine(v.Uf, v.Ur, c.Frame, th);
        }
    }

    static Rider Build(in BikeVisual v)
    {
        Vector2 ax = v.Uf - v.Ur;
        float len = MathF.Max(ax.Length(), BikeSpecs.Eps);
        Vector2 fwd = ax / len, perp = new(fwd.Y, -fwd.X), fr = v.Fr;

        float z = v.TiltZ;
        Vector2[] a, b;
        float t;

        if (z < BikeSpecs.TiltMid)
        {
            a = BikeSpecs.PoseL;
            b = BikeSpecs.PoseN;
            t = z * 2f;
        }
        else
        {
            a = BikeSpecs.PoseN;
            b = BikeSpecs.PoseR;
            t = (z - BikeSpecs.TiltMid) * 2f;
        }
        t = Math.Clamp(t, 0f, 1f);

        Vector2 Pt(int i)
        {
            float ox = Lp(a[i].X, b[i].X, t) * BikeSpecs.Sc;
            float oy = Lp(a[i].Y, b[i].Y, t) * BikeSpecs.Sc;
            return fr + perp * ox + fwd * oy;
        }

        return new(Pt(0), Pt(1), Pt(2), Pt(3), Pt(4), Pt(5), Pt(6), Pt(7));
    }

    static void DrawSeg(SpriteBatch sb, Texture2D tex, int idx, Vector2 a, Vector2 b, Color col)
    {
        if ((uint)idx >= (uint)_spr.Length)
            return;
        ref readonly Spr s = ref _spr[idx];
        DrawPair(sb, tex, in s, a, b, col);
    }

    static void DrawAtAng(SpriteBatch sb, Texture2D tex, int idx, Vector2 p, float ang, float len, Color col)
    {
        if ((uint)idx >= (uint)_spr.Length)
            return;
        ref readonly Spr s = ref _spr[idx];
        Vector2 b = new(p.X + MathF.Cos(ang) * len, p.Y + MathF.Sin(ang) * len);
        DrawPair(sb, tex, in s, p, b, col);
    }

    static void DrawPair(SpriteBatch sb, Texture2D tex, in Spr s, Vector2 a, Vector2 b, Color col)
    {
        float wx = b.X - a.X, wy = b.Y - a.Y;
        float wl = SafeLen(wx, wy);
        float lx = s.P1.X - s.P0.X, ly = s.P1.Y - s.P0.Y;
        float ll = SafeLen(lx, ly);
        float rot = MathF.Atan2(wy, wx) - MathF.Atan2(ly, lx);

        float scLen = wl / ll;
        float scWid = scLen * s.AspectK;

        Vector2 scale = s.IsVert ? new(scWid, scLen) : new(scLen, scWid);

        sb.Draw(tex, a, s.R, col, rot, s.P0, scale, SpriteEffects.None, 0f);
    }

    static void DrawWheel(SpriteBatch sb, Vector2 c, float r, float rot, Color col, float th)
    {
        if (r < BikeSpecs.Eps)
            return;

        float thR = MathF.Max(th * BikeSpecs.WhlThRimK, 1f);
        float thS = MathF.Max(th * BikeSpecs.WhlThSpkK, 1f);
        float radO = MathF.Max(r - thR * 0.5f, BikeSpecs.Eps);
        float radI = MathF.Max(r * BikeSpecs.WhlIn - thR * 0.5f, BikeSpecs.Eps);

        DrawRing(sb, c, radO, BikeSpecs.WhlN, rot, col, thR);
        DrawRing(sb, c, radI, BikeSpecs.WhlN, rot, col, thR);

        float step = MathF.Tau / BikeSpecs.SpkN;
        float sp0 = MathF.Max(r * BikeSpecs.SpkS, BikeSpecs.Eps);

        for (int i = 0; i < BikeSpecs.SpkN; i++)
        {
            float ang = rot + i * step;
            Vector2 d = new(MathF.Cos(ang), MathF.Sin(ang));
            sb.DrawLine(c + d * sp0, c + d * radI, col, thS);
        }

        float mk0 = MathF.Max(radO - thR * BikeSpecs.RimMarkK, BikeSpecs.Eps);
        Vector2 m = new(MathF.Cos(rot), MathF.Sin(rot));
        sb.DrawLine(c + m * mk0, c + m * radO, col, thS);
    }

    static void DrawRing(SpriteBatch sb, Vector2 cen, float rad, int n, float rot, Color col, float th)
    {
        if (n < 3)
            return;

        float step = MathF.Tau / n;
        Vector2 p0 = cen + new Vector2(MathF.Cos(rot), MathF.Sin(rot)) * rad;

        for (int i = 1; i <= n; i++)
        {
            float a1 = rot + step * i;
            Vector2 p1 = cen + new Vector2(MathF.Cos(a1), MathF.Sin(a1)) * rad;
            sb.DrawLine(p0, p1, col, th);
            p0 = p1;
        }
    }

    static Texture2D? LoadTex(GraphicsDevice gd, string path)
    {
        try
        {
            using Stream s = TitleContainer.OpenStream(path);
            return Texture2D.FromStream(gd, s);
        }
        catch { return null; }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Ang(Vector2 a, Vector2 b) => MathF.Atan2(b.Y - a.Y, b.X - a.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Lp(float a, float b, float t) => a + (b - a) * t;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector2 Mid(Vector2 a, Vector2 b) => (a + b) * 0.5f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float SafeLen(float x, float y) => MathF.Max(MathF.Sqrt(x * x + y * y), BikeSpecs.Eps);
}
