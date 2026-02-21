using BikePhysics = GravityDefiedGame.Models.Bike.BikePhysics;
using Pose2D = GravityDefiedGame.Views.Models.Pose;

namespace GravityDefiedGame.Views;

public enum RenderMode : byte { Flat, Voxel }

public readonly record struct TerrainColors(Color Fill, Color Stroke)
{
    public static TerrainColors From(ThemeSettings t) => new(
        t.TerrainColor,
        Color.Lerp(t.TerrainColor, Color.Black, 0.3f));
}

public readonly record struct RenderState(TerrainColors Terrain, BikeColors Bike)
{
    public static RenderState From(ThemeSettings t) => new(
        TerrainColors.From(t),
        BikeColors.From(t.BikeColors, ThemeManager.Rendering.TerrainStroke));
}

public readonly record struct GameRefs(GamePlay GP, Camera Cam)
{
    public BikePhysics? Bike => GP.Bike;
    public Level? Level => GP.Level;
    public float Zoom => Cam.Zoom;
    public Vector2 Camera => Cam.Pos;
    public Matrix Transform => Cam.Transform;
}

public sealed class Renderer : IDisposable
{
    public const string AssetDir = "Content/Assets";

    readonly SpriteBatch _sb;
    readonly GameRefs _refs;
    readonly WorldRenderer _world = new();
    readonly VoxelWorldRenderer _voxWorld = new();
    readonly BikeRenderer _bike = new();
    readonly Camera3D _cam;
    readonly VoxMesh _mesh = new();

    RenderState _st;
    float _time;
    bool _sprites, _disposed;

    public Layer SpriteMask { get; set; } = Layer.All;
    public RenderMode Mode { get; private set; }

#if DEBUG
    public Camera3D VoxCam => _cam;
#endif

    bool UseSprites => _sprites && _bike.SpritesLoaded && Mode == RenderMode.Flat;

    public Renderer(SpriteBatch sb, GameRefs refs, int w, int h)
    {
        _sb = sb;
        _refs = refs;
        _st = RenderState.From(ThemeManager.Cur);
        _cam = new(w, h);

        _world.Init();
        _voxWorld.Init();
        ThemeManager.Changed += OnTheme;
    }

    public void SetMode(RenderMode m) => Mode = m;

    public void CycleMode() =>
        Mode = Mode == RenderMode.Flat ? RenderMode.Voxel : RenderMode.Flat;

    public void ResizeCamera(int w, int h) => _cam.Resize(w, h);
    public void Update(float dt) => _time += dt;

    public void HandleInput(InputState inp)
    {
        if (Mode != RenderMode.Voxel)
            return;

        if (inp.Dragging)
            _cam.Rotate(inp.MouseDelta.X, inp.MouseDelta.Y);
        if (inp.ScrollDelta != 0)
            _cam.Zoom(inp.ScrollDelta * 0.01f);
    }

    public void LoadBikeSprites(GraphicsDevice gd)
    {
        if (!_disposed)
            _bike.LoadSprites(gd, AssetDir);
    }

    public void ToggleBikeRenderMode() => _sprites = !_sprites;

    public void RenderBackground()
    {
        if (_disposed)
            return;

        Viewport vp = _sb.GraphicsDevice.Viewport;

        _sb.Begin();
        if (Mode == RenderMode.Voxel)
            _voxWorld.Sky(_sb, vp.Width, vp.Height);
        else
            _world.Sky(_sb, vp.Width, vp.Height);
        _sb.End();
    }

    public void RenderGame()
    {
        if (_disposed)
            return;

        if (Mode == RenderMode.Voxel)
            DrawVox();
        else
            DrawFlat();
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        ThemeManager.Changed -= OnTheme;
        _bike.Dispose();
        _mesh.Dispose();
    }

    void DrawFlat()
    {
        BikePhysics? b = _refs.Bike;
        Level? lv = _refs.Level;
        Matrix tr = _refs.Transform;
        Vector2 cam = _refs.Camera;

        (BikeVisual v, Pose2D p) = CreateBikePose(b);

        _sb.Begin(transformMatrix: tr);

        if (lv is { PtCount: > 1 })
            _world.Terrain(_sb, lv, cam, _refs.Zoom, _st.Terrain, _time);

        if (b is not null && !UseSprites)
            BikeRenderer.Render(_sb, v, p, _st.Bike);

        _sb.End();

        if (b is not null && UseSprites)
            DrawSprites(v, p);

        if (lv is { PtCount: > 1 })
        {
            _sb.Begin(transformMatrix: tr);
            _world.DrawNearFlags(_sb, lv, cam, _time);
            _sb.End();
        }
    }

    void DrawVox()
    {
        GraphicsDevice gd = _sb.GraphicsDevice;
        _mesh.Init(gd);

        BikePhysics? b = _refs.Bike;
        Level? lv = _refs.Level;

        if (b is not null)
            _cam.Target(b.Pos * GamePlay.BikeScale);

        _cam.Update();
        _mesh.Begin();

        if (lv is { PtCount: > 1 })
            _voxWorld.Terrain(_mesh, lv, _st.Terrain, _st.Bike.Thick, _time);

        if (b is not null)
        {
            (BikeVisual v, Pose2D p) = CreateBikePose(b);
            VoxelBikeRenderer.Render(_mesh, v, p, _st.Bike);
        }

        gd.Clear(ClearOptions.DepthBuffer, Color.Transparent, 1f, 0);
        _mesh.Flush(gd, _cam);
    }

    void DrawSprites(in BikeVisual v, in Pose2D p)
    {
        _sb.Begin(
            SpriteSortMode.Deferred, BlendState.NonPremultiplied,
            SamplerState.PointClamp, DepthStencilState.None,
            RasterizerState.CullNone, null, _refs.Transform);

        _bike.RenderSprites(_sb, v, p, _st.Bike, SpriteMask);
        _sb.End();
    }

    static (BikeVisual v, Pose2D p) CreateBikePose(BikePhysics? b)
    {
        if (b is null)
            return (default, default);

        return (
            BikeVisual.Create(b, GamePlay.BikeScale),
            Pose2D.From(b.RiderPose, GamePlay.BikeScale));
    }

    void OnTheme()
    {
        if (_disposed)
            return;
        _st = RenderState.From(ThemeManager.Cur);
        _world.OnTheme();
        _voxWorld.OnTheme();
    }
}
