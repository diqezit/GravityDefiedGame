namespace GravityDefiedGame.Views.Models;

[Flags]
public enum Layer { None = 0, Bike = 1, Rider = 2, All = 3 }

public enum Slot : byte
{
    ArmBrace, Head, Body, LegFull,
    ArmUp, Helmet, Engine, Fender
}

public static class Spec
{
    public const float Eps = 0.01f, MinPixTh = 1f;
    public const int WhlSeg = 16, SpkN = 5, FistSeg = 8;
    public const float WhlIn = 0.82f, SpkSt = 0.2f,
        RimK = 0.65f, SpkK = 0.45f, MrkK = 1.2f;
    public const float FistR = 1.33f, HelmTilt = 0.31f,
        PoseScale = 6f, TiltMid = 0.5f, TiltK = 2f;
    public const float DefThick = 2.5f;
    public const string AssetDir = "Content/Assets",
        SpriteFile = "sprites.png";
    public static readonly Color
        DefFrame = Color.Gray, DefWheel = Color.Black,
        DefFork = Color.Gray, DefShock = Color.Gray,
        DefRider = Color.Black, DefLeg = Color.Black,
        DefBody = new(0, 0, 128), DefFist = new(156, 0, 0);
}

public readonly record struct BikeColors(
    Color Frame, Color Wheel, Color Fork, Color Shock,
    Color Rider, Color Leg, Color Body, Color Fist, float Thick)
{
    public static BikeColors From(
        Dictionary<LineType, Color>? d,
        float th = Spec.DefThick)
    {
        d ??= [];
        return new(
            d.GetValueOrDefault(LineType.Frame, Spec.DefFrame),
            d.GetValueOrDefault(LineType.Wheel, Spec.DefWheel),
            d.GetValueOrDefault(LineType.Fork, Spec.DefFork),
            d.GetValueOrDefault(LineType.Shock, Spec.DefShock),
            d.GetValueOrDefault(LineType.Rider, Spec.DefRider),
            d.GetValueOrDefault(LineType.Leg, Spec.DefLeg),
            d.GetValueOrDefault(LineType.Body, Spec.DefBody),
            d.GetValueOrDefault(LineType.Fist, Spec.DefFist),
            th);
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

    public Pose ToWorld(
        Vector2 o, Vector2 fw, Vector2 pp, float sc)
    {
        Vector2 M(Vector2 v) =>
            o + pp * (v.X * sc) + fw * (v.Y * sc);
        return new(
            M(Hip), M(Sh), M(Elb), M(Hnd),
            M(Nk), M(Ft), M(Kn), M(Hd));
    }
}

public static class Poses
{
    public static readonly Pose L = new(
        new(2.90f, -1.40f), new(3.90f, -3.60f),
        new(5.10f, -1.80f), new(6.00f, -0.70f),
        new(4.60f, 0.10f), new(1.00f, -1.20f),
        new(0.20f, -1.20f), new(4.40f, 1.30f));

    public static readonly Pose N = new(
        new(2.80f, -0.60f), new(4.00f, -2.00f),
        new(6.00f, -1.00f), new(7.00f, -0.60f),
        new(4.50f, 0.10f), new(0.30f, -2.20f),
        new(0.20f, -1.20f), new(4.40f, 1.30f));

    public static readonly Pose R = new(
        new(2.40f, 0.20f), new(4.50f, -0.20f),
        new(5.60f, 1.60f), new(6.20f, 2.70f),
        new(5.30f, 1.10f), new(0.60f, -1.50f),
        new(0.20f, -0.80f), new(4.40f, 1.30f));

    public static Pose ForTilt(float z) => z < Spec.TiltMid
        ? Pose.Lerp(L, N,
            Math.Clamp(z * Spec.TiltK, 0f, 1f))
        : Pose.Lerp(N, R,
            Math.Clamp((z - Spec.TiltMid) * Spec.TiltK, 0f, 1f));

    public static Pose Build(in BikeVisual v)
    {
        Vector2 ax = v.Uf - v.Ur;
        float len = MathF.Max(ax.Length(), Spec.Eps);
        Vector2 fw = ax / len, pp = new(fw.Y, -fw.X);
        return ForTilt(v.TiltZ)
            .ToWorld(v.Fr, fw, pp, Spec.PoseScale);
    }
}

public sealed partial class BikeRenderer : IDisposable
{
    readonly record struct Spr(
        Rectangle Src, Vector2 A, Vector2 B, float Asp)
    {
        public bool Vert =>
            MathF.Abs(B.Y - A.Y) > MathF.Abs(B.X - A.X);
        public float Len =>
            MathF.Max(Vector2.Distance(A, B), Spec.Eps);
        public float Ang =>
            MathF.Atan2(B.Y - A.Y, B.X - A.X);
    }

    static readonly Spr[] _spr =
    [
        new(new(773, 132, 274, 92),
            new(5.73f, 45.23f), new(269.04f, 45.23f), 1.00f),
        new(new(1155, 132, 239, 92),
            new(13.5f, 48f), new(223.5f, 46f), 1.00f),
        new(new(60, 496, 247, 92),
            new(1.66f, 46f), new(246.11f, 43.69f), 0.85f),
        new(new(421, 496, 250, 92),
            new(20f, 46f), new(250f, 46f), 0.85f),
        new(new(79, 41, 206, 285),
            new(113.51f, 279.13f), new(79.35f, 11.13f), 0.80f),
        new(new(409, 41, 274, 263),
            new(140.09f, 95.91f), new(249f, -40.5f), 0.90f),
        new(new(739, 417, 331, 239),
            new(253.5f, 139.5f), new(529.5f, 115.5f), 1.00f),
        new(new(1126, 485, 302, 103),
            new(277f, 161.5f), new(481f, 183.5f), 1.00f),
    ];

    Texture2D? _tex;
    bool _disposed;

    public bool SpritesLoaded => _tex is not null;

    public void LoadSprites(
        GraphicsDevice gd, string dir = Spec.AssetDir)
    {
        if (_disposed || gd is null)
            return;
        _tex?.Dispose();
        _tex = LoadTex(gd,
            Path.Combine(dir, Spec.SpriteFile));
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        _tex?.Dispose();
        _tex = null;
    }

    public static void Render(
        SpriteBatch sb, in BikeVisual v, in BikeColors c)
    {
        if (sb is null)
            return;
        float th = c.Thick;
        DrawFrame(sb, in v, in c, th, lines: true);
        Pose r = Poses.Build(in v);
        sb.DrawLine(r.Ft, r.Hip, c.Leg, th);
        sb.DrawLine(r.Hip, r.Sh, c.Leg, th);
        sb.DrawLine(r.Sh, r.Elb, c.Body, th);
        sb.DrawLine(r.Elb, r.Nk, c.Body, th);
        sb.DrawLine(r.Nk, r.Hd, c.Body, th);
        DrawRing(sb, r.Hnd,
            Spec.FistR, Spec.FistSeg, 0f, c.Fist, th);
    }

    public void RenderSprites(
        SpriteBatch sb, in BikeVisual v,
        in BikeColors c, Layer mask = Layer.All)
    {
        if (_disposed || _tex is null || sb is null)
            return;
        float th = c.Thick;
        DrawFrame(sb, in v, in c, th, lines: false);

        if (mask.HasFlag(Layer.Bike))
        {
            float ang = Ang(v.Ur, v.Uf);
            float len = Dist(v.Ur, v.Uf);
            DrawAt(sb, _tex, Slot.Engine,
                Mid(v.Fr, v.Uf), ang, len, c.Frame);
            DrawAt(sb, _tex, Slot.Fender,
                Mid(v.Fr, v.Ur), ang, len, c.Frame);
        }

        if (!mask.HasFlag(Layer.Rider))
            return;
        Pose r = Poses.Build(in v);
        float tilt = v.TiltZ > Spec.TiltMid
            ? Spec.HelmTilt : 0f;
        DrawSeg(sb, _tex, Slot.ArmBrace,
            r.Nk, r.Elb, c.Body);
        DrawSeg(sb, _tex, Slot.Head,
            r.Nk, r.Hd, c.Body);
        DrawSeg(sb, _tex, Slot.Body,
            r.Hip, r.Sh, c.Body);
        DrawSeg(sb, _tex, Slot.LegFull,
            r.Hip, r.Ft, c.Leg);
        DrawSeg(sb, _tex, Slot.ArmUp,
            r.Sh, r.Elb, c.Body);
        DrawAt(sb, _tex, Slot.Helmet, r.Hnd,
            Ang(r.Elb, r.Hnd) + tilt,
            Dist(r.Elb, r.Hnd), c.Rider);
    }

    static void DrawFrame(
        SpriteBatch sb, in BikeVisual v,
        in BikeColors c, float th, bool lines)
    {
        DrawWheel(sb, v.Fw, v.WhlR, v.FwRot, c.Wheel, th);
        DrawWheel(sb, v.Rw, v.WhlR, v.RwRot, c.Wheel, th);
        sb.DrawLine(v.Uf, v.Fw, c.Fork, th);
        sb.DrawLine(v.Ur, v.Rw, c.Shock, th);
        if (!lines)
            return;
        sb.DrawLine(v.Fr, v.Uf, c.Frame, th);
        sb.DrawLine(v.Fr, v.Ur, c.Frame, th);
        sb.DrawLine(v.Uf, v.Ur, c.Frame, th);
    }

    static void DrawSeg(
        SpriteBatch sb, Texture2D tex,
        Slot slot, Vector2 a, Vector2 b, Color col) =>
        DrawPair(sb, tex, _spr[(int)slot], a, b, col);

    static void DrawAt(
        SpriteBatch sb, Texture2D tex, Slot slot,
        Vector2 p, float ang, float len, Color col)
    {
        Vector2 b = p
            + new Vector2(MathF.Cos(ang), MathF.Sin(ang)) * len;
        DrawPair(sb, tex, _spr[(int)slot], p, b, col);
    }

    static void DrawPair(
        SpriteBatch sb, Texture2D tex,
        in Spr s, Vector2 a, Vector2 b, Color col)
    {
        float wx = b.X - a.X, wy = b.Y - a.Y;
        float wl = MathF.Max(
            MathF.Sqrt(wx * wx + wy * wy), Spec.Eps);
        float rot = MathF.Atan2(wy, wx) - s.Ang;
        float sc = wl / s.Len, sw = sc * s.Asp;
        Vector2 scale = s.Vert
            ? new(sw, sc) : new(sc, sw);
        sb.Draw(tex, a, s.Src, col, rot,
            s.A, scale, SpriteEffects.None, 0f);
    }

    static void DrawWheel(
        SpriteBatch sb, Vector2 c, float r,
        float rot, Color col, float th)
    {
        if (r < Spec.Eps)
            return;
        float tR = MathF.Max(th * Spec.RimK, Spec.MinPixTh);
        float tS = MathF.Max(th * Spec.SpkK, Spec.MinPixTh);
        float rO = MathF.Max(r - tR * 0.5f, Spec.Eps);
        float rI = MathF.Max(
            r * Spec.WhlIn - tR * 0.5f, Spec.Eps);
        DrawRing(sb, c, rO, Spec.WhlSeg, rot, col, tR);
        DrawRing(sb, c, rI, Spec.WhlSeg, rot, col, tR);
        float step = MathF.Tau / Spec.SpkN;
        float sp0 = MathF.Max(r * Spec.SpkSt, Spec.Eps);
        for (int i = 0; i < Spec.SpkN; i++)
        {
            float a = rot + i * step;
            Vector2 d = new(MathF.Cos(a), MathF.Sin(a));
            sb.DrawLine(c + d * sp0, c + d * rI, col, tS);
        }
        float mk = MathF.Max(
            rO - tR * Spec.MrkK, Spec.Eps);
        Vector2 m = new(MathF.Cos(rot), MathF.Sin(rot));
        sb.DrawLine(c + m * mk, c + m * rO, col, tS);
    }

    static void DrawRing(
        SpriteBatch sb, Vector2 cen, float rad,
        int n, float rot, Color col, float th)
    {
        if (n < 3)
            return;
        float step = MathF.Tau / n;
        Vector2 p0 = cen
            + new Vector2(MathF.Cos(rot), MathF.Sin(rot)) * rad;
        for (int i = 1; i <= n; i++)
        {
            float a = rot + step * i;
            Vector2 p1 = cen
                + new Vector2(MathF.Cos(a), MathF.Sin(a)) * rad;
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
    static float Ang(Vector2 a, Vector2 b) =>
        MathF.Atan2(b.Y - a.Y, b.X - a.X);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static Vector2 Mid(Vector2 a, Vector2 b) =>
        (a + b) * 0.5f;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Dist(Vector2 a, Vector2 b) =>
        MathF.Max(Vector2.Distance(a, b), Spec.Eps);
}
