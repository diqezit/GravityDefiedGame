namespace GravityDefiedGame.Views.Voxel3D;

public sealed class Camera3D
{
    const float Sens = 0.01f, ZoomK = 10f;
    const float PitchDef = 0.3f, PitchMin = -0.5f, PitchMax = 1f;
    const float DistDef = 500f, DistMin = 200f, DistMax = 1500f;
    const float Fov = MathF.PI / 4f, Near = 1f, Far = 5000f;

    float _yaw, _pitch = PitchDef, _dist = DistDef;
    int _w, _h;
    Vector3 _tgt;

    public Camera3D(int w, int h) => (_w, _h) = (w, h);
    public Matrix View { get; private set; }
    public Matrix Proj { get; private set; }
    public Vector3 Eye { get; private set; }

    public void Target(Vector2 p) => _tgt = new(p.X, -p.Y, 0);
    public void Resize(int w, int h) => (_w, _h) = (w, h);

    public void Rotate(float dx, float dy)
    {
        _yaw += dx * Sens;
        _pitch = Math.Clamp(_pitch + dy * Sens, PitchMin, PitchMax);
    }

    public void Zoom(float d) =>
        _dist = Math.Clamp(_dist - d * ZoomK, DistMin, DistMax);

    public void Update()
    {
        float cp = MathF.Cos(_pitch), sp = MathF.Sin(_pitch);
        Vector3 dir = new(cp * MathF.Sin(_yaw), sp, cp * MathF.Cos(_yaw));
        Eye = _tgt + dir * _dist;
        View = Matrix.CreateLookAt(Eye, _tgt, Vector3.Up);
        Proj = Matrix.CreatePerspectiveFieldOfView(
            Fov, (float)_w / _h, Near, Far);
    }
}

public sealed class VoxMesh : IDisposable
{
    const int Cap = 24576;
    const float Eps = 0.01f, FillOfs = 0.02f, NormMin = 1e-6f;
    const float Ambient = 0.5f, Diffuse = 0.5f;

    static readonly Vector3 LightDir =
        Vector3.Normalize(new(-0.3f, 0.8f, 0.6f));

    readonly VertexPositionColor[] _buf = new VertexPositionColor[Cap];
    int _n;
    BasicEffect? _fx;
    bool _disposed;

    public void Init(GraphicsDevice gd) =>
        _fx ??= new(gd) { VertexColorEnabled = true };

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

        foreach (EffectPass p in _fx.CurrentTechnique.Passes)
        {
            p.Apply();
            gd.DrawUserPrimitives(
                PrimitiveType.TriangleList, _buf, 0, _n / 3);
        }
    }

    public void Line(Vector2 a, Vector2 b, float z, float th, Color col)
    {
        float dx = b.X - a.X, dy = a.Y - b.Y;
        float len = MathF.Sqrt(dx * dx + dy * dy);
        if (len < Eps)
            return;

        float s = th * 0.5f / len, hz = th * 0.5f;
        Vector3 a3 = V3(a, z), b3 = V3(b, z);
        Vector3 n = new(-dy * s, dx * s, 0);
        Vector3 h = new(0, 0, hz);
        Box(a3 + n, a3 - n, b3 + n, b3 - n, h, col);
    }

    public void CrossZ(Vector2 p, float z0, float z1, float th, Color c)
    {
        float h = th * 0.5f;
        Quad(V3(p, z0, h), V3(p, z0, -h),
             V3(p, z1, -h), V3(p, z1, h), c);
    }

    public void FillZ(Vector2 a, Vector2 b, float z0, float z1, Color c)
    {
        Quad(V3(a, z0, -FillOfs), V3(b, z0, -FillOfs),
             V3(b, z1, -FillOfs), V3(a, z1, -FillOfs), c);
    }

    public void Ring(Vector2 c, float r, int seg, float rot,
        Color col, float th)
    {
        float step = MathF.Tau / seg;
        for (int i = 0; i < seg; i++)
        {
            float a1 = rot + i * step, a2 = a1 + step;
            Line(c + DirNeg(a1) * r, c + DirNeg(a2) * r,
                 0, th, col);
        }
    }

    static Vector2 DirNeg(float a) =>
        new(MathF.Cos(a), -MathF.Sin(a));

    static Vector3 V3(Vector2 p, float z, float yOfs = 0) =>
        new(p.X, -p.Y + yOfs, z);

    void Box(Vector3 a0, Vector3 a1, Vector3 b0, Vector3 b1,
        Vector3 h, Color c)
    {
        Vector3 v0 = a0 + h, v1 = a1 + h, v2 = b0 + h, v3 = b1 + h;
        Vector3 v4 = a0 - h, v5 = a1 - h, v6 = b0 - h, v7 = b1 - h;
        Quad(v0, v2, v3, v1, c);
        Quad(v4, v5, v7, v6, c);
        Quad(v0, v4, v6, v2, c);
        Quad(v1, v3, v7, v5, c);
        Quad(v0, v1, v5, v4, c);
        Quad(v2, v6, v7, v3, c);
    }

    void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color col)
    {
        if (_n + 6 > Cap)
            return;

        var nm = Vector3.Cross(b - a, c - a);
        if (nm.LengthSquared() < NormMin)
            return;

        float dot = -Vector3.Dot(Vector3.Normalize(nm), LightDir);
        float k = Ambient + Diffuse * MathF.Max(0, dot);
        Color lit = new(
            (byte)(col.R * k), (byte)(col.G * k),
            (byte)(col.B * k), col.A);

        _buf[_n++] = new(a, lit);
        _buf[_n++] = new(b, lit);
        _buf[_n++] = new(c, lit);
        _buf[_n++] = new(a, lit);
        _buf[_n++] = new(c, lit);
        _buf[_n++] = new(d, lit);
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        _fx?.Dispose();
    }
}
