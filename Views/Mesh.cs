namespace GravityDefiedGame.Views;

public readonly struct CamCfg
{
    public readonly float
        Sens, ZoomK, Fov, Near, Far,
        PitchDef, PitchMin, PitchMax,
        DistDef, DistMin, DistMax;

    public CamCfg()
    {
        (Sens, ZoomK) = (0.01f, 10f);
        (Fov, Near, Far) = (MathF.PI / 4f, 1f, 5000f);
        (PitchDef, PitchMin, PitchMax) = (0.3f, -0.5f, 1f);
        (DistDef, DistMin, DistMax) = (500f, 200f, 1500f);
    }
}

public readonly struct MeshCfg
{
    public readonly int Cap;
    public readonly float Eps, NormMin, Ambient, Diffuse;
    public readonly Vector3 Light;

    public MeshCfg()
    {
        (Cap, Eps, NormMin) = (24576, 0.01f, 1e-6f);
        (Ambient, Diffuse) = (0.5f, 0.5f);
        Light = Vector3.Normalize(new(-0.3f, 0.8f, 0.6f));
    }
}

public sealed class Camera3D
{
    static readonly CamCfg C = new();

    float _yaw, _pitch = C.PitchDef, _dist = C.DistDef;
    int _w, _h;
    Vector3 _tgt;

    public Matrix View { get; private set; }
    public Matrix Proj { get; private set; }
    public Vector3 Eye { get; private set; }

    public Camera3D(int w, int h) => (_w, _h) = (w, h);

    public void Target(Vector2 p) =>
        _tgt = new(p.X, -p.Y, 0);

    public void Resize(int w, int h) =>
        (_w, _h) = (w, h);

    public void Zoom(float d) =>
        _dist = Math.Clamp(_dist - d * C.ZoomK, C.DistMin, C.DistMax);

    public void Rotate(float dx, float dy)
    {
        _yaw += dx * C.Sens;
        _pitch = Math.Clamp(_pitch + dy * C.Sens, C.PitchMin, C.PitchMax);
    }

    public void Update()
    {
        float cp = MathF.Cos(_pitch), sp = MathF.Sin(_pitch);
        Vector3 dir = new(cp * MathF.Sin(_yaw), sp, cp * MathF.Cos(_yaw));
        Eye = _tgt + dir * _dist;
        View = Matrix.CreateLookAt(Eye, _tgt, Vector3.Up);
        Proj = Matrix.CreatePerspectiveFieldOfView(C.Fov, (float)_w / _h, C.Near, C.Far);
    }
}

public sealed class Mesh : IDisposable
{
    static readonly MeshCfg C = new();

    readonly VertexPositionColor[] _buf = new VertexPositionColor[C.Cap];
    BasicEffect? _fx;
    int _n;
    bool _dead;

    public void Init(GraphicsDevice gd) => _fx ??= new(gd) { VertexColorEnabled = true };

    public void Begin() => _n = 0;

    public void Flush(GraphicsDevice gd, Camera3D cam)
    {
        if (_fx is null || _n < 3)
            return;

        _fx.View = cam.View;
        _fx.Projection = cam.Proj;
        _fx.World = Matrix.Identity;

        gd.DepthStencilState = DepthStencilState.Default;
        gd.RasterizerState = RasterizerState.CullNone;
        gd.BlendState = BlendState.Opaque;

        foreach (EffectPass? p in _fx.CurrentTechnique.Passes)
        {
            p.Apply();
            gd.DrawUserPrimitives(PrimitiveType.TriangleList, _buf, 0, _n / 3);
        }
    }

    public void Seg(Vector2 a, Vector2 b, float z0, float z1, float th, Color fill, Color stroke)
    {
        Quad(V(a, z0), V(b, z0), V(b, z1), V(a, z1), fill);
        Line(a, b, z0, th, stroke);
        Line(a, b, z1, th, stroke);
    }

    public void Cross(Vector2 p, float z0, float z1, float th, Color c)
    {
        float h = th * 0.5f;
        Quad(V(p, z0, h), V(p, z0, -h), V(p, z1, -h), V(p, z1, h), c);
    }

    public void Line(Vector2 a, Vector2 b, float z, float th, Color c)
    {
        float dx = b.X - a.X, dy = a.Y - b.Y;
        float len = MathF.Sqrt(dx * dx + dy * dy);
        if (len < C.Eps)
            return;

        float r = th * 0.5f, s = r / len;
        Vector3 n = new(-dy * s, dx * s, 0), t = new(dx * s, dy * s, 0), h = new(0, 0, r);
        Vector3 az = V(a, z), bz = V(b, z);

        ReadOnlySpan<Vector3> p = [az - t, az + n, bz + n, bz + t, bz - n, az - n];

        Quad(p[0] + h, p[1] + h, p[2] + h, p[3] + h, c);
        Quad(p[0] + h, p[3] + h, p[4] + h, p[5] + h, c);
        Quad(p[0] - h, p[3] - h, p[2] - h, p[1] - h, c);
        Quad(p[0] - h, p[5] - h, p[4] - h, p[3] - h, c);

        for (int i = 0; i < 6; i++)
        {
            int j = (i + 1) % 6;
            Quad(p[i] + h, p[i] - h, p[j] - h, p[j] + h, c);
        }
    }

    public void Ring(Vector2 c, float r, int seg, float rot, Color col, float th)
    {
        float step = MathF.Tau / seg;
        for (int i = 0; i < seg; i++)
        {
            float a1 = rot + i * step, a2 = a1 + step;
            Line(c + Dir(a1) * r, c + Dir(a2) * r, 0, th, col);
        }
    }

    public void Dispose()
    {
        if (_dead)
            return;
        _dead = true;
        _fx?.Dispose();
    }

    static Vector2 Dir(float a) => new(MathF.Cos(a), -MathF.Sin(a));
    static Vector3 V(Vector2 p, float z, float y = 0) => new(p.X, -p.Y + y, z);

    static Color Shade(Color c, Vector3 n)
    {
        float k = C.Ambient + C.Diffuse * MathF.Max(0, -Vector3.Dot(n, C.Light));
        return new((byte)(c.R * k), (byte)(c.G * k), (byte)(c.B * k), c.A);
    }

    void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color col)
    {
        if (_n + 6 > C.Cap)
            return;

        Vector3 nm = Vector3.Cross(b - a, c - a);
        if (nm.LengthSquared() < C.NormMin)
            return;

        Color lit = Shade(col, Vector3.Normalize(nm));
        _buf[_n++] = new(a, lit);
        _buf[_n++] = new(b, lit);
        _buf[_n++] = new(c, lit);
        _buf[_n++] = new(a, lit);
        _buf[_n++] = new(c, lit);
        _buf[_n++] = new(d, lit);
    }
}
