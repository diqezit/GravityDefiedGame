using BikePhysics = GravityDefiedGame.Models.Bike.BikePhysics;
using PhysPose = GravityDefiedGame.Models.Bike.Pose;
using static System.MathF;
using static GravityDefiedGame.Views.Models.Gfx;
using static GravityDefiedGame.Views.Models.BikeDrawCore;

namespace GravityDefiedGame.Views.Models;

[Flags]
public enum Layer : byte { None = 0, Bike = 1 << 0, Rider = 1 << 1, All = Bike | Rider }

public delegate void LineFn(Vector2 a, Vector2 b, float th, Color col);
public delegate void RingFn(Vector2 c, float r, int seg, float rot, Color col, float th);

public readonly record struct SpriteData(Rectangle Src, Vector2 A, Vector2 B, float Asp);

public readonly record struct BikeVisual(
    Vector2 Fr, Vector2 Fw, Vector2 Rw, Vector2 Uf, Vector2 Ur, Vector2 Hd,
    float WhlR, float FwRot, float RwRot, float TiltZ)
{
    public static BikeVisual Create(BikePhysics p, float sc) => new(
        p.Pos * sc, p.FwPos * sc, p.RwPos * sc,
        p.UfPos * sc, p.UrPos * sc, p.HdPos * sc,
        p.CfgPub.WheelR * sc, p.FwRot, p.RwRot, p.RawTiltZ);
}

public readonly record struct Pose(
    Vector2 Hip, Vector2 Sh, Vector2 Elb, Vector2 Hnd,
    Vector2 Nk, Vector2 Ft, Vector2 Kn, Vector2 Hd)
{
    public static Pose From(in PhysPose p, float sc) => new(
        p.Hip * sc, p.Sh * sc, p.Elb * sc, p.Hnd * sc,
        p.Nk * sc, p.Ft * sc, p.Kn * sc, p.Hd * sc);

    public static Pose Lerp(in Pose a, in Pose b, float t) => new(
        Vector2.Lerp(a.Hip, b.Hip, t), Vector2.Lerp(a.Sh, b.Sh, t),
        Vector2.Lerp(a.Elb, b.Elb, t), Vector2.Lerp(a.Hnd, b.Hnd, t),
        Vector2.Lerp(a.Nk, b.Nk, t), Vector2.Lerp(a.Ft, b.Ft, t),
        Vector2.Lerp(a.Kn, b.Kn, t), Vector2.Lerp(a.Hd, b.Hd, t));

    public Pose ToWorld(Vector2 o, Vector2 fw, Vector2 pp, float sc)
    {
        Vector2 M(Vector2 v) => o + pp * (v.X * sc) + fw * (v.Y * sc);
        return new(M(Hip), M(Sh), M(Elb), M(Hnd), M(Nk), M(Ft), M(Kn), M(Hd));
    }
}

public readonly record struct BikeColors(
    Color Frame, Color Wheel, Color Fork, Color Shock,
    Color Rider, Color Leg, Color Body, Color Fist, float Thick)
{
    static readonly Dictionary<LineType, Color> Empty = [];
    static readonly Color DefBody = new(0, 0, 128), DefFist = new(156, 0, 0);

    static Color Get(Dictionary<LineType, Color> d, LineType t, Color def) =>
        d.GetValueOrDefault(t, def);

    public static BikeColors From(Dictionary<LineType, Color>? d, float th = 2.5f)
    {
        d ??= Empty;
        return new(
            Get(d, LineType.Frame, Color.Gray), Get(d, LineType.Wheel, Color.Black),
            Get(d, LineType.Fork, Color.Gray), Get(d, LineType.Shock, Color.Gray),
            Get(d, LineType.Rider, Color.Black), Get(d, LineType.Leg, Color.Black),
            Get(d, LineType.Body, DefBody), Get(d, LineType.Fist, DefFist), th);
    }
}

public readonly record struct ThickCfg(
    float Wheel, float Fork, float Frame, float Limb, float Body, float Head)
{
    public static ThickCfg Uniform(float th) => new(th, th, th, th, th, th);

    public static ThickCfg Voxel(float th, in VoxelBikeCfg c) => new(
        th * c.WheelK, th * c.ForkK, th * c.FrameK,
        th * c.LimbK, th * c.BodyK, th * c.HeadK);
}

public readonly record struct DrawCtx(LineFn Line, RingFn Ring);

public readonly struct BikeBaseCfg
{
    public readonly float RimK, SpkK, InnerK, SpkStart, MrkK, FistR;
    public readonly int RimSeg, SpkN, FistSeg;

    public BikeBaseCfg()
    {
        RimK = 0.65f;
        SpkK = 0.45f;
        InnerK = 0.82f;
        SpkStart = 0.2f;
        MrkK = 1.2f;
        FistR = 1.33f;
        RimSeg = 16;
        SpkN = 5;
        FistSeg = 8;
    }
}

public readonly struct SpriteCfg
{
    public readonly float HelmTilt, TiltMid;
    public readonly string SpriteFile;
    public readonly SpriteData[] Sprites;

    public SpriteCfg()
    {
        HelmTilt = 0.31f;
        TiltMid = 0.5f;
        SpriteFile = "sprites.png";
        Sprites =
        [
            new(new(773, 132, 274, 92),   new(5.73f, 45.23f),    new(269.04f, 45.23f),  1f),
            new(new(1155, 132, 239, 92),  new(13.5f, 48f),       new(223.5f, 46f),      1f),
            new(new(60, 496, 247, 92),    new(1.66f, 46f),       new(246.11f, 43.69f),  0.85f),
            new(new(421, 496, 250, 92),   new(20f, 46f),         new(250f, 46f),        0.85f),
            new(new(79, 41, 206, 285),    new(113.51f, 279.13f), new(79.35f, 11.13f),   0.8f),
            new(new(409, 41, 274, 263),   new(140.09f, 95.91f),  new(249f, -40.5f),     0.9f),
            new(new(739, 417, 331, 239),  new(253.5f, 139.5f),   new(529.5f, 115.5f),   1f),
            new(new(1126, 485, 302, 103), new(277f, 161.5f),     new(481f, 183.5f),     1f)
        ];
    }
}

public readonly struct VoxelBikeCfg
{
    public readonly float WheelK, ForkK, FrameK, LimbK, BodyK, HeadK;

    public VoxelBikeCfg()
    {
        WheelK = 0.9f;
        ForkK = 0.95f;
        FrameK = 1.15f;
        LimbK = 1.3f;
        BodyK = 1.4f;
        HeadK = 1.5f;
    }
}

public static class Gfx
{
    public const float Eps = 0.01f;

    public static float Len(float v) => Max(v, Eps);
    public static Vector2 Dir(float a) => new(Cos(a), Sin(a));
    public static float Ang(Vector2 a, Vector2 b) => Atan2(b.Y - a.Y, b.X - a.X);
    public static Vector2 Mid(Vector2 a, Vector2 b) => (a + b) * 0.5f;
    public static float Dist(Vector2 a, Vector2 b) => Len(Vector2.Distance(a, b));

    public static void Ring2D(
        SpriteBatch sb, Vector2 c, float r, int seg, float rot, Color col, float th)
    {
        if (seg < 3)
            return;
        float step = Tau / seg;
        for (int i = 0; i < seg; i++)
        {
            float a = rot + i * step;
            sb.DrawLine(c + Dir(a) * r, c + Dir(a + step) * r, col, th);
        }
    }

    public static DrawCtx Ctx(SpriteBatch sb) => new(
        (a, b, th, col) => sb.DrawLine(a, b, col, th),
        (c, r, seg, rot, col, th) => Ring2D(sb, c, r, seg, rot, col, th));

    public static DrawCtx Ctx(VoxMesh m) => new(
        (a, b, th, col) => m.Line(a, b, 0f, th, col),
        (c, r, seg, rot, col, th) => m.Ring(c, r, seg, -rot, col, th));
}

public static class BikeDrawCore
{
    static readonly BikeBaseCfg B = new();

    public static void Wheel(
        Vector2 c, float r, float rot, Color col, float th, in DrawCtx d)
    {
        if (r < Eps)
            return;

        float rimTh = Max(th * B.RimK, 1f), spkTh = Max(th * B.SpkK, 1f);
        float outR = Len(r - rimTh * 0.5f), inR = Len(r * B.InnerK - rimTh * 0.5f);
        float spkSt = Len(r * B.SpkStart), mrkSt = Len(outR - rimTh * B.MrkK);
        float step = Tau / B.SpkN;

        d.Ring(c, outR, B.RimSeg, rot, col, rimTh);
        d.Ring(c, inR, B.RimSeg, rot, col, rimTh);

        for (int i = 0; i < B.SpkN; i++)
        {
            Vector2 dir = Dir(rot + i * step);
            d.Line(c + dir * spkSt, c + dir * inR, spkTh, col);
        }

        Vector2 mrkDir = Dir(rot);
        d.Line(c + mrkDir * mrkSt, c + mrkDir * outR, spkTh, col);
    }

    public static void Forks(in BikeVisual v, in BikeColors c, float th, in DrawCtx d)
    {
        d.Line(v.Uf, v.Fw, th, c.Fork);
        d.Line(v.Ur, v.Rw, th, c.Shock);
    }

    public static void Frame(in BikeVisual v, in BikeColors c, float th, in DrawCtx d)
    {
        d.Line(v.Fr, v.Uf, th, c.Frame);
        d.Line(v.Fr, v.Ur, th, c.Frame);
        d.Line(v.Uf, v.Ur, th, c.Frame);
    }

    public static void Rider(in Pose p, in BikeColors c, in ThickCfg th, in DrawCtx d)
    {
        d.Line(p.Ft, p.Hip, th.Limb, c.Leg);
        d.Line(p.Hip, p.Sh, th.Limb, c.Leg);
        d.Line(p.Sh, p.Elb, th.Body, c.Body);
        d.Line(p.Elb, p.Nk, th.Body, c.Body);
        d.Line(p.Nk, p.Hd, th.Head, c.Body);
        d.Ring(p.Hnd, B.FistR, B.FistSeg, 0f, c.Fist, th.Head);
    }

    public static void Wheels(in BikeVisual v, in BikeColors c, float th, in DrawCtx d)
    {
        Wheel(v.Fw, v.WhlR, v.FwRot, c.Wheel, th, in d);
        Wheel(v.Rw, v.WhlR, v.RwRot, c.Wheel, th, in d);
    }

    public static void All(
        in BikeVisual v, in Pose p, in BikeColors c, in ThickCfg th, in DrawCtx d)
    {
        Wheels(in v, in c, th.Wheel, in d);
        Forks(in v, in c, th.Fork, in d);
        Frame(in v, in c, th.Frame, in d);
        Rider(in p, in c, in th, in d);
    }
}

public sealed class BikeRenderer : IDisposable
{
    static readonly SpriteCfg Cfg = new();

    Texture2D? _tex;
    bool _disposed;
    (SpriteBatch sb, DrawCtx d)? _cache;

    public bool SpritesLoaded => _tex is not null;

    public void LoadSprites(GraphicsDevice gd, string dir)
    {
        if (_disposed)
            return;
        _tex?.Dispose();
        _tex = null;

        try
        {
            string file = Cfg.SpriteFile;
            using Stream s = TitleContainer.OpenStream(Path.Combine(dir, file));
            _tex = Texture2D.FromStream(gd, s);
        }
        catch { _tex = null; }
    }

    public static void Render(
        SpriteBatch sb, in BikeVisual v, in Pose p, in BikeColors c)
    {
        if (sb is null)
            return;
        All(in v, in p, in c, ThickCfg.Uniform(c.Thick), Ctx(sb));
    }

    public void RenderSprites(
        SpriteBatch sb, in BikeVisual v, in Pose p, in BikeColors c,
        Layer mask = Layer.All)
    {
        if (_disposed || _tex is null || sb is null)
            return;

        DrawCtx d = GetCtx(sb);
        float th = c.Thick;

        Wheels(in v, in c, th, in d);
        Forks(in v, in c, th, in d);

        if ((mask & Layer.Bike) != 0)
            DrawBikeSprites(sb, in v, c.Frame);

        if ((mask & Layer.Rider) != 0)
            DrawRiderSprites(sb, in v, in p, in c);
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        _tex?.Dispose();
        (_tex, _cache) = (null, null);
    }

    DrawCtx GetCtx(SpriteBatch sb)
    {
        if (_cache is { } c && ReferenceEquals(c.sb, sb))
            return c.d;
        DrawCtx d = Ctx(sb);
        _cache = (sb, d);
        return d;
    }

    void DrawBikeSprites(SpriteBatch sb, in BikeVisual v, Color col)
    {
        float ang = Ang(v.Ur, v.Uf), len = Dist(v.Ur, v.Uf);
        Sprite(sb, 6, Mid(v.Fr, v.Uf), ang, len, col);
        Sprite(sb, 7, Mid(v.Fr, v.Ur), ang, len, col);
    }

    void DrawRiderSprites(SpriteBatch sb, in BikeVisual v, in Pose p, in BikeColors c)
    {
        float tilt = v.TiltZ > Cfg.TiltMid ? Cfg.HelmTilt : 0f;

        Sprite(sb, 0, p.Nk, p.Elb, c.Body);
        Sprite(sb, 1, p.Nk, p.Hd, c.Body);
        Sprite(sb, 2, p.Hip, p.Sh, c.Body);
        Sprite(sb, 3, p.Hip, p.Ft, c.Leg);
        Sprite(sb, 4, p.Sh, p.Elb, c.Body);
        Sprite(sb, 5, p.Hnd, Ang(p.Elb, p.Hnd) + tilt, Dist(p.Elb, p.Hnd), c.Rider);
    }

    void Sprite(SpriteBatch sb, int idx, Vector2 a, Vector2 b, Color col) =>
        Sprite(sb, idx, a, Ang(a, b), Dist(a, b), col);

    void Sprite(SpriteBatch sb, int idx, Vector2 pos, float ang, float len, Color col)
    {
        ref readonly SpriteData s = ref Cfg.Sprites[idx];

        float sLen = Len(Vector2.Distance(s.A, s.B));
        float sAng = Atan2(s.B.Y - s.A.Y, s.B.X - s.A.X);
        float rot = ang - sAng, sc = len / sLen, sw = sc * s.Asp;

        bool vert = Abs(s.B.Y - s.A.Y) > Abs(s.B.X - s.A.X);
        Vector2 scale = vert ? new(sw, sc) : new(sc, sw);

        sb.Draw(_tex!, pos, s.Src, col, rot, s.A, scale, SpriteEffects.None, 0f);
    }
}

public static class VoxelBikeRenderer
{
    static readonly VoxelBikeCfg Cfg = new();

    public static void Render(VoxMesh m, in BikeVisual v, in Pose p, in BikeColors c) =>
        All(in v, in p, in c, ThickCfg.Voxel(c.Thick, in Cfg), Ctx(m));
}
