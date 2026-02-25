using BikePhysics = GravityDefiedGame.Models.Bike.BikePhysics;
using PhysPose = GravityDefiedGame.Models.Bike.Pose;
using static System.MathF;
using static GravityDefiedGame.Views.Gfx;
using static GravityDefiedGame.Views.BikeDraw;

namespace GravityDefiedGame.Views;

[Flags]
public enum Layer : byte
{
    None = 0, Bike = 1 << 0, Rider = 1 << 1, All = Bike | Rider
}

public readonly record struct WheelData(Vector2 Pos, float Rot);

public readonly record struct SpriteData(
    Rectangle Src, Vector2 A, Vector2 B, float Asp);

public readonly record struct BikeVisual(
    Vector2 Frame, WheelData Front, WheelData Rear,
    Vector2 UpperFront, Vector2 UpperRear, Vector2 Head,
    float WheelRadius, float TiltZ)
{
    public static BikeVisual Create(BikePhysics p, float sc) => new(
        p.Pos * sc,
        new(p.FwPos * sc, p.FwRot),
        new(p.RwPos * sc, p.RwRot),
        p.UfPos * sc, p.UrPos * sc, p.HdPos * sc,
        p.CfgPub.WheelR * sc, p.RawTiltZ);
}

public readonly record struct Pose(
    Vector2 Hip, Vector2 Shoulder, Vector2 Elbow, Vector2 Hand,
    Vector2 Neck, Vector2 Foot, Vector2 Knee, Vector2 Head)
{
    public static Pose From(in PhysPose p, float sc) => new(
        p.Hip * sc, p.Sh * sc, p.Elb * sc, p.Hnd * sc,
        p.Nk * sc, p.Ft * sc, p.Kn * sc, p.Hd * sc);

}

public readonly record struct BikeColors(
    Color Frame, Color Wheel, Color Fork, Color Shock,
    Color Rider, Color Leg, Color Body, Color Fist, float Thick)
{
    static readonly Color DefBody = new(0, 0, 128);
    static readonly Color DefFist = new(156, 0, 0);

    public BikeColors(Dictionary<LineType, Color>? d, float th = 2.5f)
        : this(
            G(d, LineType.Frame, Color.Gray),
            G(d, LineType.Wheel, Color.Black),
            G(d, LineType.Fork, Color.Gray),
            G(d, LineType.Shock, Color.Gray),
            G(d, LineType.Rider, Color.Black),
            G(d, LineType.Leg, Color.Black),
            G(d, LineType.Body, DefBody),
            G(d, LineType.Fist, DefFist),
            th)
    { }

    static Color G(
        Dictionary<LineType, Color>? d, LineType t, Color def) =>
        d?.GetValueOrDefault(t, def) ?? def;
}

public readonly record struct ThickCfg(
    float Wheel, float Fork, float Frame,
    float Limb, float Body, float Head)
{
    public ThickCfg(float th) : this(th, th, th, th, th, th) { }

    public ThickCfg(float th, in VoxelBikeCfg c) : this(
        th * c.WheelK, th * c.ForkK, th * c.FrameK,
        th * c.LimbK, th * c.BodyK, th * c.HeadK)
    { }
}

public readonly struct DrawCtx
{
    readonly Action<Vector2, Vector2, float, Color> _line;
    readonly Action<Vector2, float, int, float, Color, float> _ring;

    public DrawCtx(
        Action<Vector2, Vector2, float, Color> line,
        Action<Vector2, float, int, float, Color, float> ring) =>
        (_line, _ring) = (line, ring);

    public void Line(Vector2 a, Vector2 b, float th, Color col) =>
        _line(a, b, th, col);

    public void Ring(
        Vector2 c, float r, int seg,
        float rot, Color col, float th) =>
        _ring(c, r, seg, rot, col, th);
}

public readonly struct BikeBaseCfg
{
    public readonly float RimK, SpkK, InnerK, SpkStart, MrkK, FistR;
    public readonly int RimSeg, SpkN, FistSeg;

    public BikeBaseCfg()
    {
        (RimK, SpkK, InnerK) = (0.65f, 0.45f, 0.82f);
        (SpkStart, MrkK, FistR) = (0.2f, 1.2f, 1.33f);
        (RimSeg, SpkN, FistSeg) = (16, 5, 8);
    }
}

public readonly struct SpriteCfg
{
    public readonly float HelmTilt, TiltMid;
    public readonly string SpriteFile;
    public readonly SpriteData[] Sprites;

    public SpriteCfg()
    {
        (HelmTilt, TiltMid) = (0.31f, 0.5f);
        SpriteFile = "sprites.png";
        Sprites =
        [
            new(new(773, 132, 274, 92),  new(5.73f, 45.23f),    new(269.04f, 45.23f), 1f),
            new(new(1155, 132, 239, 92), new(13.5f, 48f),       new(223.5f, 46f), 1f),
            new(new(60, 496, 247, 92),   new(1.66f, 46f),       new(246.11f, 43.69f), 0.85f),
            new(new(421, 496, 250, 92),  new(20f, 46f),         new(250f, 46f), 0.85f),
            new(new(79, 41, 206, 285),   new(113.51f, 279.13f), new(79.35f, 11.13f), 0.8f),
            new(new(409, 41, 274, 263),  new(140.09f, 95.91f),  new(249f, -40.5f), 0.9f),
            new(new(739, 417, 331, 239), new(253.5f, 139.5f),   new(529.5f, 115.5f), 1f),
            new(new(1126, 485, 302, 103),new(277f, 161.5f),     new(481f, 183.5f), 1f)
        ];
    }
}

public readonly struct VoxelBikeCfg
{
    public readonly float WheelK, ForkK, FrameK, LimbK, BodyK, HeadK;

    public VoxelBikeCfg()
    {
        (WheelK, ForkK, FrameK) = (0.9f, 0.95f, 1.15f);
        (LimbK, BodyK, HeadK) = (1.3f, 1.4f, 1.5f);
    }
}

public static class Gfx
{
    public const float Eps = 0.01f;

    public static float Len(float v) => Max(v, Eps);
    public static Vector2 Dir(float a) => new(Cos(a), Sin(a));
    public static Vector2 Mid(Vector2 a, Vector2 b) => (a + b) * 0.5f;

    public static float Ang(Vector2 a, Vector2 b) =>
        Atan2(b.Y - a.Y, b.X - a.X);

    public static float Dist(Vector2 a, Vector2 b) =>
        Len(Vector2.Distance(a, b));

    public static void Ring2D(
        SpriteBatch sb, Vector2 c, float r,
        int seg, float rot, Color col, float th)
    {
        if (seg < 3)
            return;
        float step = Tau / seg;
        for (int i = 0; i < seg; i++)
        {
            float a = rot + i * step;
            sb.DrawLine(
                c + Dir(a) * r,
                c + Dir(a + step) * r, col, th);
        }
    }

    public static DrawCtx Ctx(SpriteBatch sb) => new(
        (a, b, th, col) => sb.DrawLine(a, b, col, th),
        (c, r, seg, rot, col, th) =>
            Ring2D(sb, c, r, seg, rot, col, th));

    public static DrawCtx Ctx(Mesh m) => new(
        (a, b, th, col) => m.Line(a, b, 0f, th, col),
        (c, r, seg, rot, col, th) =>
            m.Ring(c, r, seg, -rot, col, th));
}

public static class BikeDraw
{
    static readonly BikeBaseCfg B = new();

    public static void Wheel(
        Vector2 c, float r, float rot,
        Color col, float th, in DrawCtx d)
    {
        if (r < Eps)
            return;

        float rimTh = Max(th * B.RimK, 1f);
        float spkTh = Max(th * B.SpkK, 1f);
        float outR = Len(r - rimTh * 0.5f);
        float inR = Len(r * B.InnerK - rimTh * 0.5f);
        float spkSt = Len(r * B.SpkStart);
        float mrkSt = Len(outR - rimTh * B.MrkK);
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

    public static void Wheels(
        in BikeVisual v, in BikeColors c,
        float th, in DrawCtx d)
    {
        Wheel(v.Front.Pos, v.WheelRadius, v.Front.Rot,
            c.Wheel, th, in d);
        Wheel(v.Rear.Pos, v.WheelRadius, v.Rear.Rot,
            c.Wheel, th, in d);
    }

    public static void Forks(
        in BikeVisual v, in BikeColors c,
        float th, in DrawCtx d)
    {
        d.Line(v.UpperFront, v.Front.Pos, th, c.Fork);
        d.Line(v.UpperRear, v.Rear.Pos, th, c.Shock);
    }

    public static void Frame(
        in BikeVisual v, in BikeColors c,
        float th, in DrawCtx d)
    {
        d.Line(v.Frame, v.UpperFront, th, c.Frame);
        d.Line(v.Frame, v.UpperRear, th, c.Frame);
        d.Line(v.UpperFront, v.UpperRear, th, c.Frame);
    }

    public static void Rider(
        in Pose p, in BikeColors c,
        in ThickCfg th, in DrawCtx d)
    {
        d.Line(p.Foot, p.Hip, th.Limb, c.Leg);
        d.Line(p.Hip, p.Shoulder, th.Limb, c.Leg);
        d.Line(p.Shoulder, p.Elbow, th.Body, c.Body);
        d.Line(p.Elbow, p.Neck, th.Body, c.Body);
        d.Line(p.Neck, p.Head, th.Head, c.Body);
        d.Ring(p.Hand, B.FistR, B.FistSeg, 0f, c.Fist, th.Head);
    }

    public static void All(
        in BikeVisual v, in Pose p,
        in BikeColors c, in ThickCfg th, in DrawCtx d)
    {
        Wheels(in v, in c, th.Wheel, in d);
        Forks(in v, in c, th.Fork, in d);
        Frame(in v, in c, th.Frame, in d);
        Rider(in p, in c, in th, in d);
    }
}

public sealed class BikeRenderer : IDisposable
{
    static readonly SpriteCfg Spr = new();
    static readonly VoxelBikeCfg Vox = new();

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
            using Stream s = TitleContainer.OpenStream(
                Path.Combine(dir, Spr.SpriteFile));
            _tex = Texture2D.FromStream(gd, s);
        }
        catch { _tex = null; }
    }

    public static void Render(
        SpriteBatch sb, in BikeVisual v,
        in Pose p, in BikeColors c)
    {
        if (sb is null)
            return;
        All(in v, in p, in c, new ThickCfg(c.Thick), Ctx(sb));
    }

    public static void Render(
        Mesh m, in BikeVisual v,
        in Pose p, in BikeColors c) =>
        All(in v, in p, in c,
            new ThickCfg(c.Thick, in Vox), Ctx(m));

    public void RenderSprites(
        SpriteBatch sb, in BikeVisual v, in Pose p,
        in BikeColors c, Layer mask = Layer.All)
    {
        if (_disposed || _tex is null || sb is null)
            return;

        DrawCtx d = GetCtx(sb);
        Wheels(in v, in c, c.Thick, in d);
        Forks(in v, in c, c.Thick, in d);

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

    void DrawBikeSprites(
        SpriteBatch sb, in BikeVisual v, Color col)
    {
        float ang = Ang(v.UpperRear, v.UpperFront);
        float len = Dist(v.UpperRear, v.UpperFront);
        Sprite(sb, 6,
            Mid(v.Frame, v.UpperFront), ang, len, col);
        Sprite(sb, 7,
            Mid(v.Frame, v.UpperRear), ang, len, col);
    }

    void DrawRiderSprites(
        SpriteBatch sb, in BikeVisual v,
        in Pose p, in BikeColors c)
    {
        float tilt = v.TiltZ > Spr.TiltMid
            ? Spr.HelmTilt : 0f;

        Sprite(sb, 0, p.Neck, p.Elbow, c.Body);
        Sprite(sb, 1, p.Neck, p.Head, c.Body);
        Sprite(sb, 2, p.Hip, p.Shoulder, c.Body);
        Sprite(sb, 3, p.Hip, p.Foot, c.Leg);
        Sprite(sb, 4, p.Shoulder, p.Elbow, c.Body);
        Sprite(sb, 5, p.Hand,
            Ang(p.Elbow, p.Hand) + tilt,
            Dist(p.Elbow, p.Hand), c.Rider);
    }

    void Sprite(
        SpriteBatch sb, int idx,
        Vector2 from, Vector2 to, Color col) =>
        Sprite(sb, idx, from,
            Ang(from, to), Dist(from, to), col);

    void Sprite(
        SpriteBatch sb, int idx, Vector2 pos,
        float ang, float len, Color col)
    {
        ref readonly SpriteData s = ref Spr.Sprites[idx];

        float sLen = Dist(s.A, s.B);
        float sAng = Atan2(s.B.Y - s.A.Y, s.B.X - s.A.X);
        float rot = ang - sAng;
        float sc = len / sLen, sw = sc * s.Asp;

        bool vert = Abs(s.B.Y - s.A.Y) > Abs(s.B.X - s.A.X);
        Vector2 scale = vert ? new(sw, sc) : new(sc, sw);

        sb.Draw(
            _tex!, pos, s.Src, col, rot, s.A,
            scale, SpriteEffects.None, 0f);
    }
}
