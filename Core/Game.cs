using BikeInput = GravityDefiedGame.Models.Bike.BikeInput;
using Vector2 = Microsoft.Xna.Framework.Vector2;

namespace GravityDefiedGame.Core;

public readonly record struct InputState(
    Point MousePos, Point MouseDelta,
    bool LMB, bool LMBWas, bool RMB,
    KeyboardState Kb, KeyboardState PrevKb,
    int ScrollDelta, BikeInput Bike,
    bool ToggleSprites, bool CycleRender,
    bool ToggleDebugNodes)
{
    public bool Dragging => RMB;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool JustPressed(Keys k) =>
        Kb.IsKeyDown(k) && PrevKb.IsKeyUp(k);
}

public static class InputReader
{
    static MouseState _pm;
    static KeyboardState _pk;

    public static InputState Read()
    {
        (MouseState pm, MouseState m) = (_pm, Mouse.GetState());
        (KeyboardState pk, KeyboardState k) = (_pk, Keyboard.GetState());
        (_pm, _pk) = (m, k);

        return new(
            new(m.X, m.Y),
            new(m.X - pm.X, m.Y - pm.Y),
            m.LeftButton == ButtonState.Pressed,
            pm.LeftButton == ButtonState.Pressed,
            m.RightButton == ButtonState.Pressed,
            k, pk,
            m.ScrollWheelValue - pm.ScrollWheelValue,
            ReadBike(k),
            k.IsKeyDown(Keys.F1) && pk.IsKeyUp(Keys.F1),
            k.IsKeyDown(Keys.F2) && pk.IsKeyUp(Keys.F2),
            k.IsKeyDown(Keys.F4) && pk.IsKeyUp(Keys.F4));
    }

    static BikeInput ReadBike(KeyboardState k) => new(
        k.IsKeyDown(Keys.W) ? 1f : 0f,
        k.IsKeyDown(Keys.S) ? 1f : 0f,
        k.IsKeyDown(Keys.A) ? -1f
            : k.IsKeyDown(Keys.D) ? 1f : 0f);
}

public sealed class Display(
    GraphicsDeviceManager gfx, int w, int h,
    Action<int, int>? onResize = null)
{
    public int W { get; private set; } = w;
    public int H { get; private set; } = h;

    public void Set(int nw, int nh)
    {
        if (nw <= 0 || nh <= 0 || (nw == W && nh == H))
            return;

        (W, H) = (nw, nh);
        gfx.PreferredBackBufferWidth = nw;
        gfx.PreferredBackBufferHeight = nh;
        gfx.ApplyChanges();
        onResize?.Invoke(nw, nh);
    }
}

public sealed class Camera
{
    const float
        ZMin = 0.5f, ZMax = 5f, ZDef = 3f,
        LerpK = 0.1f, ScrollK = 0.001f, YOff = -60f;

    int _w, _h;

    public Camera(int w, int h) => (_w, _h) = (w, h);

    public float Zoom { get; private set; } = ZDef;
    public Vector2 Pos { get; private set; }
    public float HalfViewWidth => _w / (2f * Zoom);

    public Matrix Transform =>
        Matrix.CreateTranslation(-Pos.X, -Pos.Y, 0f) *
        Matrix.CreateScale(Zoom) *
        Matrix.CreateTranslation(_w / 2f, _h / 2f, 0f);

    public void Follow(Vector2 t) =>
        Pos = Vector2.Lerp(Pos, new(t.X, t.Y + YOff), LerpK);

    public void Scroll(int d)
    {
        if (d != 0)
            Zoom = MathHelper.Clamp(Zoom + d * ScrollK, ZMin, ZMax);
    }

    public void Resize(int w, int h) => (_w, _h) = (w, h);
}

public sealed class GameSettings
{
    const string FilePath = "settings.json";
    const int WMin = 640, WMax = 3840, HMin = 480, HMax = 2160;

    static readonly (int W, int H)[] Res =
    [
        (800, 600), (1024, 768), (1280, 720),
        (1366, 768), (1600, 900), (1920, 1080),
    ];

    static readonly string[] RndNames = ["2D", "3D Voxel"];

    static readonly JsonSerializerOptions Jso = new()
    {
        WriteIndented = true
    };

    public int W { get; set; } = 800;
    public int H { get; set; } = 600;
    public int ThemeIdx { get; set; }
    public int BikeIdx { get; set; }
    public int RenderIdx { get; set; }

    public string ResStr => $"{W} x {H}";
    public string RenderStr => RndNames[RenderIdx];

    public int ResIdx
    {
        get => Math.Max(0,
            Array.FindIndex(Res, r => r.W == W && r.H == H));
        set
        {
            if ((uint)value < (uint)Res.Length)
                (W, H) = Res[value];
        }
    }

    public static GameSettings Load()
    {
        try
        {
            if (!File.Exists(FilePath))
                return new();

            GameSettings s = JsonSerializer.Deserialize<GameSettings>(
                File.ReadAllText(FilePath)) ?? new();

            s.W = Math.Clamp(s.W, WMin, WMax);
            s.H = Math.Clamp(s.H, HMin, HMax);
            s.ThemeIdx = Math.Max(0, s.ThemeIdx);
            s.BikeIdx = Math.Clamp(s.BikeIdx, 0, 2);
            s.RenderIdx = Math.Clamp(s.RenderIdx, 0, 1);
            return s;
        }
        catch { return new(); }
    }

    public void Save()
    {
        try
        { File.WriteAllText(FilePath, JsonSerializer.Serialize(this, Jso)); }
        catch { }
    }

    public void NextRes() =>
        ResIdx = ThemeManager.Cycle(ResIdx, Res.Length, 1);

    public void PrevRes() =>
        ResIdx = ThemeManager.Cycle(ResIdx, Res.Length, -1);

    public void NextRender() =>
        RenderIdx = ThemeManager.Cycle(RenderIdx, RndNames.Length, 1);

    public void PrevRender() =>
        RenderIdx = ThemeManager.Cycle(RenderIdx, RndNames.Length, -1);
}

public sealed class Game : Microsoft.Xna.Framework.Game
{
    const float MaxDt = 0.1f, Margin = 200f;
    const int Fps = 60;

    readonly GraphicsDeviceManager _gfx;
    readonly GameSettings _cfg;
    readonly GamePlay _gp = new();

    SpriteBatch _sb = null!;
    Display _dsp = null!;
    Camera _cam = null!;
    Renderer _rnd = null!;
    ScreenMachine _ui = null!;

    public Game()
    {
        _cfg = GameSettings.Load();
        _gfx = new(this)
        {
            PreferredBackBufferWidth = _cfg.W,
            PreferredBackBufferHeight = _cfg.H
        };

        Content.RootDirectory = "Content";
        IsMouseVisible = true;
        TargetElapsedTime = TimeSpan.FromSeconds(1.0 / Fps);
    }

    protected override void Initialize()
    {
        _cam = new(_cfg.W, _cfg.H);
        _dsp = new(_gfx, _cfg.W, _cfg.H, OnResize);
        ThemeManager.Set(_cfg.ThemeIdx);
        base.Initialize();
    }

    protected override void LoadContent()
    {
        _sb = new(GraphicsDevice);
        _rnd = new(_sb, new(_gp, _cam), _cfg.W, _cfg.H);
        _rnd.LoadBikeSprites(GraphicsDevice);
        _rnd.SetMode((RenderMode)_cfg.RenderIdx);

        MyraEnvironment.Game = this;
        RebuildUI();
        _gp.LoadLevels();
    }

    protected override void Update(GameTime gt)
    {
        float dt = MathF.Min(
            (float)gt.ElapsedGameTime.TotalSeconds, MaxDt);
        InputState inp = InputReader.Read();

        if (inp.ToggleSprites)
            _rnd.ToggleBikeRenderMode();

        if (inp.CycleRender)
        {
            _rnd.CycleMode();
            _cfg.RenderIdx = (int)_rnd.Mode;
            _cfg.Save();
        }

#if DEBUG
        if (inp.ToggleDebugNodes && _gp.Bike != null)
            _gp.Bike.DebugNodesEnabled = !_gp.Bike.DebugNodesEnabled;
#endif

        _ui.Update(inp);

        if (_gp.State == GameState.Playing)
        {
            _gp.HandleInput(inp.Bike);
            _gp.Update(dt);
            UpdateCam(inp.ScrollDelta);
            _rnd.HandleInput(inp);
        }

        _rnd.Update(dt);
        base.Update(gt);
    }

    protected override void Draw(GameTime gt)
    {
        GraphicsDevice.Clear(ThemeManager.Cur.BgColor);
        _rnd.RenderBackground();

        if (_gp.Level is not null)
            _rnd.RenderGame();

        _ui.Draw();

#if DEBUG
        _gp.Bike?.DrawDebug(_sb, _ui.DebugFont, GraphicsDevice,
            _cam.Transform, _rnd.Mode == RenderMode.Voxel ? _rnd.VoxCam : null);
#endif

        base.Draw(gt);
    }

    public void SetResolution(int w, int h) => _dsp.Set(w, h);

    public void ApplyRenderMode() =>
        _rnd.SetMode((RenderMode)_cfg.RenderIdx);

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _ui?.Dispose();
            _sb?.Dispose();
            _rnd?.Dispose();
            _gp?.Dispose();
        }
        base.Dispose(disposing);
    }

    void UpdateCam(int scroll)
    {
        if (_gp.Bike is not { } b)
            return;

        Vector2 tgt = (b.RagdollActive ? b.RiderPose.Hd : b.Pos)
            * GamePlay.BikeScale;

        if (float.IsFinite(tgt.X) && float.IsFinite(tgt.Y))
        {
            _cam.Follow(tgt);

            float bx = b.Pos.X * GamePlay.BikeScale;
            if (float.IsFinite(bx))
                _gp.Level?.UpdateVisible(
                    bx - _cam.HalfViewWidth,
                    bx + _cam.HalfViewWidth,
                    Margin);
        }

        _cam.Scroll(scroll);
    }

    void RebuildUI() => _ui = new(_gp, this, _cfg);

    void OnResize(int w, int h)
    {
        _cam.Resize(w, h);
        _rnd.ResizeCamera(w, h);
        (_cfg.W, _cfg.H) = (w, h);
        _cfg.Save();
        RebuildUI();
    }
}
