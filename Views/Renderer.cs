namespace GravityDefiedGame.Views;

public readonly record struct TerrainColors(Color Fill, Color Stroke)
{
    public static TerrainColors From(ThemeSettings t) => new(
        t.TerrainColor, Color.Lerp(t.TerrainColor, Color.Black, 0.3f));
}

public readonly record struct RenderState(TerrainColors Terrain, BikeColors Bike)
{
    public static RenderState From(ThemeSettings t) => new(
        TerrainColors.From(t),
        BikeColors.From(t.BikeColors, ThemeManager.Rendering.TerrainStroke));
}

public readonly record struct GameRefs(
    Func<BikeAPI?> Bike, Func<Level?> Level,
    Func<float> Zoom, Func<Vector2> Camera, Func<Matrix> Transform);

public sealed class Renderer : IDisposable
{
    readonly SpriteBatch _sb;
    readonly GameRefs _refs;
    readonly WorldRenderer _world = new();
    readonly BikeRenderer _bike = new();

    RenderState _state;
    bool _useSprites, _disposed;

    public int SkinIndex { get; set; }
    public int SpriteMask { get; set; } = 3;

    public Renderer(SpriteBatch sb, GameRefs refs)
    {
        (_sb, _refs) = (sb, refs);
        _state = RenderState.From(ThemeManager.Cur);
        _world.Init();
        ThemeManager.Changed += OnTheme;
    }

    public void LoadBikeSprites(GraphicsDevice gd, string dir = "Content/Assets")
    {
        if (_disposed)
            return;
        _bike.LoadSprites(gd, dir);
    }

    public void ToggleBikeRenderMode() => _useSprites = !_useSprites;

    public void RenderBackground()
    {
        if (_disposed)
            return;

        _sb.Begin();
        _world.Sky(_sb);
        _sb.End();
    }

    public void RenderGame()
    {
        if (_disposed)
            return;

        Bike? bike = _refs.Bike();
        Level? lv = _refs.Level();

        _sb.Begin(transformMatrix: _refs.Transform());

        if (lv is { PtCount: > 1 })
            _world.Terrain(_sb, lv, _refs.Camera(), _refs.Zoom(), _state.Terrain);

        if (bike is not null && !ShouldUseSprites)
            BikeRenderer.Render(_sb, bike.Visual, _state.Bike);

        _sb.End();

        if (bike is not null && ShouldUseSprites)
            RenderBikeSprites(bike.Visual);
    }

    bool ShouldUseSprites => _useSprites && _bike.SpritesLoaded;

    void RenderBikeSprites(in BikeVisual v)
    {
        _sb.Begin(
            SpriteSortMode.Deferred, BlendState.NonPremultiplied,
            SamplerState.PointClamp, DepthStencilState.None,
            RasterizerState.CullNone, null, _refs.Transform());

        _bike.RenderSprites(_sb, v, _state.Bike, SpriteMask);

        _sb.End();
    }

    public void Dispose()
    {
        if (_disposed)
            return;

        _disposed = true;
        ThemeManager.Changed -= OnTheme;
        _bike.Dispose();
    }

    void OnTheme()
    {
        if (_disposed)
            return;

        _state = RenderState.From(ThemeManager.Cur);
        _world.OnTheme();
    }
}
