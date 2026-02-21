using BikeInput = GravityDefiedGame.Models.Bike.BikeInput;
using Vector2 = Microsoft.Xna.Framework.Vector2;

namespace GravityDefiedGame.Core;

public readonly struct CamCfg
{
    public readonly float MinZoom, MaxZoom, DefZoom,
        LerpK, ScrollK, YOff;

    public CamCfg()
    {
        MinZoom = 0.5f;
        MaxZoom = 5f;
        DefZoom = 3f;
        LerpK = 0.1f;
        ScrollK = 0.001f;
        YOff = -60f;
    }
}

public readonly struct SettingsCfg
{
    public readonly string Path;
    public readonly int MinW, MaxW, MinH, MaxH, MaxBikeIdx;
    public readonly int DefW, DefH;
    public readonly (int W, int H)[] Resolutions;

    public SettingsCfg()
    {
        Path = "settings.json";
        MinW = 640;
        MaxW = 3840;
        MinH = 480;
        MaxH = 2160;
        MaxBikeIdx = 2;
        DefW = 800;
        DefH = 600;
        Resolutions =
        [
            (800, 600), (1024, 768), (1280, 720),
            (1366, 768), (1600, 900), (1920, 1080)
        ];
    }
}

public readonly struct AppCfg
{
    public readonly int Fps, FontSm, FontLg;
    public readonly float MaxDt, VisMargin;
    public readonly string FontPath, AssetDir;

    public AppCfg()
    {
        Fps = 60;
        MaxDt = 0.1f;
        VisMargin = 200f;
        FontSm = 20;
        FontLg = 40;
        FontPath = "Content/Fonts/PixelOperator8.ttf";
        AssetDir = Spec.AssetDir;
    }

    public TimeSpan FrameTime =>
        TimeSpan.FromSeconds(1.0 / Fps);
}

public readonly struct KeyBinds
{
    public readonly Keys Throttle, Brake, LeanL, LeanR,
        ToggleSprites;

    public KeyBinds()
    {
        Throttle = Keys.W;
        Brake = Keys.S;
        LeanL = Keys.A;
        LeanR = Keys.D;
        ToggleSprites = Keys.F1;
    }

    public BikeInput Read(KeyboardState k)
    {
        float thr = k.IsKeyDown(Throttle) ? 1f : 0f;
        float brk = k.IsKeyDown(Brake) ? 1f : 0f;
        float lean = k.IsKeyDown(LeanL) ? -1f
            : k.IsKeyDown(LeanR) ? 1f : 0f;
        return new(thr, brk, lean);
    }
}

public readonly record struct InputState(
    Point MousePos, bool MouseDown, bool MouseWasDown,
    KeyboardState Keys, KeyboardState PrevKeys,
    int ScrollDelta, BikeInput Bike,
    bool ToggleSprites)
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool JustPressed(Keys k) =>
        Keys.IsKeyDown(k) && PrevKeys.IsKeyUp(k);
}

public static class InputReader
{
    static readonly KeyBinds Binds = new();
    static MouseState _prevM;
    static KeyboardState _prevK;

    public static InputState Read()
    {
        MouseState pm = _prevM, m = Mouse.GetState();
        KeyboardState pk = _prevK, k = Keyboard.GetState();
        (_prevM, _prevK) = (m, k);

        bool toggle = k.IsKeyDown(Binds.ToggleSprites)
            && pk.IsKeyUp(Binds.ToggleSprites);

        return new(
            new(m.X, m.Y),
            m.LeftButton == ButtonState.Pressed,
            pm.LeftButton == ButtonState.Pressed,
            k, pk,
            m.ScrollWheelValue - pm.ScrollWheelValue,
            Binds.Read(k),
            toggle);
    }
}

public sealed class Font(string path) : IDisposable
{
    readonly FontSystem _sys = Init(path);

    static FontSystem Init(string p)
    {
        FontSystem sys = new();
        using Stream s = TitleContainer.OpenStream(p);
        sys.AddFont(s);
        return sys;
    }

    public DynamicSpriteFont Get(int size) =>
        _sys.GetFont(size);

    public void Dispose() => _sys.Dispose();
}

public sealed class Display(
    GraphicsDeviceManager gfx, int w, int h,
    Action<int, int>? onResize = null)
{
    public int W { get; private set; } = w;
    public int H { get; private set; } = h;

    public void Set(int nw, int nh)
    {
        if (nw <= 0 || nh <= 0 || nw == W && nh == H)
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
    static readonly CamCfg Cfg = new();
    int _w, _h;

    public Camera(int w, int h) => (_w, _h) = (w, h);

    public float Zoom { get; private set; } = Cfg.DefZoom;
    public Vector2 Pos { get; private set; }
    public float HalfViewWidth => _w / (2f * Zoom);

    public Matrix Transform =>
        Matrix.CreateTranslation(-Pos.X, -Pos.Y, 0)
        * Matrix.CreateScale(Zoom)
        * Matrix.CreateTranslation(_w / 2f, _h / 2f, 0);

    public void Follow(Vector2 t) =>
        Pos = Vector2.Lerp(
            Pos, new(t.X, t.Y + Cfg.YOff), Cfg.LerpK);

    public void Scroll(int d)
    {
        if (d != 0)
            Zoom = MathHelper.Clamp(
                Zoom + d * Cfg.ScrollK,
                Cfg.MinZoom, Cfg.MaxZoom);
    }

    public void Resize(int nw, int nh) =>
        (_w, _h) = (nw, nh);
}

public sealed class GameSettings
{
    static readonly SettingsCfg Cfg = new();
    static readonly JsonSerializerOptions Opt = new()
    {
        WriteIndented = true
    };

    public int W { get; set; } = Cfg.DefW;
    public int H { get; set; } = Cfg.DefH;
    public int ThemeIdx { get; set; }
    public int BikeIdx { get; set; }

    public string ResStr => $"{W} x {H}";

    public int ResIdx
    {
        get => Math.Max(0, Array.FindIndex(
            Cfg.Resolutions,
            r => r.W == W && r.H == H));
        set
        {
            if ((uint)value < (uint)Cfg.Resolutions.Length)
                (W, H) = Cfg.Resolutions[value];
        }
    }

    public static GameSettings Load()
    {
        try
        {
            if (!File.Exists(Cfg.Path))
                return new();

            GameSettings s = JsonSerializer.Deserialize
                <GameSettings>(
                    File.ReadAllText(Cfg.Path)) ?? new();

            s.W = Math.Clamp(s.W, Cfg.MinW, Cfg.MaxW);
            s.H = Math.Clamp(s.H, Cfg.MinH, Cfg.MaxH);
            s.ThemeIdx = Math.Max(0, s.ThemeIdx);
            s.BikeIdx = Math.Clamp(
                s.BikeIdx, 0, Cfg.MaxBikeIdx);
            return s;
        }
        catch { return new(); }
    }

    public void Save()
    {
        try
        {
            File.WriteAllText(Cfg.Path,
                JsonSerializer.Serialize(this, Opt));
        }
        catch { }
    }

    public void NextRes() =>
        ResIdx = (ResIdx + 1) % Cfg.Resolutions.Length;

    public void PrevRes() =>
        ResIdx = (ResIdx - 1 + Cfg.Resolutions.Length)
            % Cfg.Resolutions.Length;
}

public sealed class Game : Microsoft.Xna.Framework.Game
{
    static readonly AppCfg App = new();

    readonly GraphicsDeviceManager _gfx;
    readonly GameSettings _cfg;
    readonly GamePlay _ctrl = new();

    SpriteBatch _sb = null!;
    Font _font = null!;
    Display _disp = null!;
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
        TargetElapsedTime = App.FrameTime;
    }

    protected override void Initialize()
    {
        _cam = new(_cfg.W, _cfg.H);
        _disp = new(_gfx, _cfg.W, _cfg.H, OnResize);
        ThemeManager.Set(_cfg.ThemeIdx);
        base.Initialize();
    }

    protected override void LoadContent()
    {
        _sb = new(GraphicsDevice);
        _font = new(App.FontPath);
        _rnd = new(_sb, new(
            () => _ctrl.Bike, () => _ctrl.Level,
            () => _cam.Zoom, () => _cam.Pos,
            () => _cam.Transform));
        _rnd.LoadBikeSprites(GraphicsDevice, App.AssetDir);
        RebuildUI();
        _ctrl.LoadLevels();
    }

    protected override void Update(GameTime time)
    {
        float dt = MathF.Min(
            (float)time.ElapsedGameTime.TotalSeconds,
            App.MaxDt);
        InputState inp = InputReader.Read();

        if (inp.ToggleSprites)
            _rnd.ToggleBikeRenderMode();

        _ui.Update(inp);

        if (_ctrl.State == GameState.Playing)
        {
            _ctrl.HandleInput(inp.Bike);
            _ctrl.Update(dt);

            if (_ctrl.Bike?.Pos is { } p
                && float.IsFinite(p.X)
                && float.IsFinite(p.Y))
            {
                _cam.Follow(p);
                _ctrl.Level?.UpdateVisible(
                    p.X - _cam.HalfViewWidth,
                    p.X + _cam.HalfViewWidth,
                    App.VisMargin);
            }

            _cam.Scroll(inp.ScrollDelta);
        }

        base.Update(time);
    }

    protected override void Draw(GameTime time)
    {
        GraphicsDevice.Clear(ThemeManager.Cur.BgColor);
        _rnd.RenderBackground();

        if (_ctrl.Level is not null)
            _rnd.RenderGame();

        _sb.Begin(
            SpriteSortMode.Deferred,
            BlendState.AlphaBlend);
        _ui.Draw();
        _sb.End();

        base.Draw(time);
    }

    public void SetResolution(int w, int h) =>
        _disp.Set(w, h);

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _font?.Dispose();
            _sb?.Dispose();
            _rnd?.Dispose();
            _ctrl?.Dispose();
        }
        base.Dispose(disposing);
    }

    void RebuildUI() =>
        _ui = new(
            new(_sb, _font.Get(App.FontSm),
                _font.Get(App.FontLg),
                _disp.W, _disp.H),
            _ctrl, this, _cfg);

    void OnResize(int w, int h)
    {
        _cam.Resize(w, h);
        (_cfg.W, _cfg.H) = (w, h);
        _cfg.Save();
        RebuildUI();
    }
}
