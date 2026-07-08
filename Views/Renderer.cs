using static GravityDefiedGame.Views.GhostVisCfg;
using static GravityDefiedGame.Core.GamePlay;

namespace GravityDefiedGame.Views;

public enum RenderMode : byte { Flat, Voxel }

public readonly record struct TerrainColors(Color Fill, Color Stroke)
{
    public static TerrainColors From(ThemeSettings t) => new(
        t.TerrainColor,
        Color.Lerp(t.TerrainColor, Color.Black, 0.3f));
}

public readonly record struct GameRefs(GamePlay GP, Camera Cam)
{
    public BikePhysics? Bike => GP.Bike;
    public Level? Level => GP.Level;
    public Matrix Tr => Cam.Transform;
    public bool HasLevel => Level is { PtCount: > 1 };
}

public sealed class Renderer : IDisposable
{
    public const string AssetDir = "Content/Assets";

    readonly SpriteBatch _sb;
    readonly GameRefs _refs;
    readonly FlatWorld _flatWorld = new();
    readonly VoxWorld _voxWorld = new();
    readonly BikeRenderer _bikeRnd = new();
    readonly Mesh _mesh = new();
    readonly Camera3D _voxCam;

    TerrainColors _terrain;
    BikeColors _bike;
    float _time;
    bool _spr, _disposed;

    public RenderMode Mode { get; private set; }
    public Camera3D? VoxCam => IsVox ? _voxCam : null;

    bool UseSpr => _spr && _bikeRnd.SpritesLoaded && !IsVox;
    bool IsVox => Mode == RenderMode.Voxel;

    public Renderer(SpriteBatch sb, GameRefs refs, int w, int h)
    {
        (_sb, _refs, _voxCam) = (sb, refs, new(w, h));
        UpdateTheme();
        ThemeManager.Changed += OnTheme;
    }

    public void SetMode(RenderMode m) => Mode = m;
    public void CycleMode() => SetMode(IsVox ? RenderMode.Flat : RenderMode.Voxel);
    public void Resize(int w, int h) => _voxCam.Resize(w, h);
    public void Update(float dt) => _time += dt;
    public void ToggleSprites() => _spr = !_spr;

    public void LoadSprites(GraphicsDevice gd) =>
        _bikeRnd.LoadSprites(gd, AssetDir);

    public void Draw(BikePhysics bike, Ghost ghost)
    {
        if (IsVox)
            DrawVoxel(bike, ghost);
        else
            DrawFlat(bike, ghost);
    }

    public void Input(in InputState inp)
    {
        if (!IsVox) return;

        MouseInput m = inp.Mouse;
        if (m.Right) _voxCam.Rotate(m.Delta.X, m.Delta.Y);
        if (m.Scroll != 0) _voxCam.Zoom(m.Scroll * 0.01f);
    }

    public void Background()
    {
        Viewport vp = _sb.GraphicsDevice.Viewport;

        _sb.Begin();
        if (IsVox) _voxWorld.Sky(_sb, vp.Width, vp.Height);
        else _flatWorld.Sky(_sb, vp.Width, vp.Height);
        _sb.End();
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        ThemeManager.Changed -= OnTheme;
        _bikeRnd.Dispose();
        _mesh.Dispose();
    }

    static (BikeVisual Vis, Pose Pose) GetPose(BikePhysics bike) =>
        (BikeVisual.Create(bike, BikeScale),
         Pose.From(bike.RiderPose, BikeScale));

    void DrawGhost(BikePhysics bike, GhostFrame frame, bool isVox)
    {
        float alpha = isVox ? AlphaVox : Alpha2D;

        BikeVisual vis = BikeVisual.Create(frame, bike, BikeScale);
        Pose pose = Pose.From(frame, BikeScale);
        BikeColors col = _bike.Fade(alpha);

        if (isVox)
            BikeRenderer.Render(_mesh, vis, pose, col, VoxOffsetZ);
        else
            BikeRenderer.Render(_sb, vis, pose, col);
    }

    void DrawFlat(BikePhysics bike, Ghost ghost)
    {
        bool spr = UseSpr;
        var (vis, pose) = GetPose(bike);
        var gf = ghost.CurrentFrame;

        _sb.Begin(SpriteSortMode.Deferred, null, null, null, null, null, _refs.Tr);

        if (_refs.HasLevel)
            _flatWorld.Terrain(
                _sb, _refs.Level!, _refs.Cam.Pos,
                _refs.Cam.Zoom, _terrain, _time);

        if (!spr)
            BikeRenderer.Render(_sb, vis, pose, _bike);

        if (ghost.Enabled && gf is GhostFrame frame)
            DrawGhost(bike, frame, false);

        _sb.End();

        if (spr)
        {
            _sb.Begin(
                SpriteSortMode.Deferred,
                BlendState.NonPremultiplied,
                SamplerState.PointClamp,
                DepthStencilState.None,
                RasterizerState.CullNone, null, _refs.Tr);
            _bikeRnd.RenderSprites(_sb, vis, pose, _bike, Layer.All);
            _sb.End();
        }

        if (_refs.HasLevel)
        {
            _sb.Begin(SpriteSortMode.Deferred, null, null, null, null, null, _refs.Tr);
            _flatWorld.NearFlags(
                _sb, _refs.Level!, _refs.Cam.Pos, _time);
            _sb.End();
        }
    }

    void DrawVoxel(BikePhysics bike, Ghost ghost)
    {
        GraphicsDevice gd = _sb.GraphicsDevice;
        _mesh.Init(gd);

        _voxCam.Target(bike.Pos * BikeScale);

        _voxCam.Update();
        _mesh.Begin();

        if (_refs.HasLevel)
            VoxWorld.Terrain(_mesh, _refs.Level!, _terrain, _bike.Thick, _time);

        var (vis, pose) = GetPose(bike);

        BikeRenderer.Render(_mesh, vis, pose, _bike);

        var gf = ghost.CurrentFrame;
        if (ghost.Enabled && gf is GhostFrame frame)
            DrawGhost(bike, frame, true);

        gd.Clear(ClearOptions.DepthBuffer, Color.Transparent, 1f, 0);
        _mesh.Flush(gd, _voxCam);
    }

    void UpdateTheme()
    {
        ThemeSettings t = ThemeManager.Cur;
        _terrain = TerrainColors.From(t);
        _bike = new BikeColors(t.BikeColors, ThemeManager.Rendering.TerrainStroke);
    }

    void OnTheme()
    {
        UpdateTheme();
        _flatWorld.OnTheme();
        _voxWorld.OnTheme();
    }
}
