using BikeInput = GravityDefiedGame.Models.Bike.BikeInput;
using Vector2 = Microsoft.Xna.Framework.Vector2;

namespace GravityDefiedGame.Core;

public readonly record struct InputState(
    Point MousePos, bool MouseDown, bool MouseWasDown,
    KeyboardState Keys, KeyboardState PrevKeys,
    int ScrollDelta, BikeInput Bike)
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool JustPressed(Keys k) => Keys.IsKeyDown(k) && PrevKeys.IsKeyUp(k);
}

public static class InputReader
{
    static MouseState _prevM;
    static KeyboardState _prevK;

    public static InputState Read()
    {
        MouseState pm = _prevM, m = Mouse.GetState();
        KeyboardState pk = _prevK, k = Keyboard.GetState();
        (_prevM, _prevK) = (m, k);

        bool w = k.IsKeyDown(Keys.W);
        bool s = k.IsKeyDown(Keys.S);
        bool a = k.IsKeyDown(Keys.A);
        bool d = k.IsKeyDown(Keys.D);

        float thr = w ? 1f : 0f;
        float brk = s ? 1f : 0f;
        float lean = a ? -1f : d ? 1f : 0f;

        return new(
            new(m.X, m.Y),
            m.LeftButton == ButtonState.Pressed,
            pm.LeftButton == ButtonState.Pressed,
            k, pk,
            m.ScrollWheelValue - pm.ScrollWheelValue,
            new(thr, brk, lean));
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

    public DynamicSpriteFont Get(int size) => _sys.GetFont(size);
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

public sealed class Camera(int w, int h)
{
    const float MinZ = 0.5f, MaxZ = 5f, DefZ = 3f;
    const float LerpK = 0.1f, ScrollK = 0.001f, YOff = -60f;

    int _w = w, _h = h;

    public float Zoom { get; private set; } = DefZ;
    public Vector2 Pos { get; private set; }
    public float HalfViewWidth => _w / (2f * Zoom);

    public Matrix Transform =>
        Matrix.CreateTranslation(-Pos.X, -Pos.Y, 0) *
        Matrix.CreateScale(Zoom) *
        Matrix.CreateTranslation(_w / 2f, _h / 2f, 0);

    public void Follow(Vector2 t) =>
        Pos = Vector2.Lerp(Pos, new(t.X, t.Y + YOff), LerpK);

    public void Scroll(int d)
    {
        if (d != 0)
            Zoom = MathHelper.Clamp(Zoom + d * ScrollK, MinZ, MaxZ);
    }

    public void Resize(int nw, int nh) => (_w, _h) = (nw, nh);
}

public sealed class GameSettings
{
    const string Path = "settings.json";

    static readonly (int W, int H)[] Res =
    [
        (800, 600), (1024, 768), (1280, 720),
        (1366, 768), (1600, 900), (1920, 1080)
    ];

    static readonly JsonSerializerOptions Opt = new() { WriteIndented = true };

    public int W { get; set; } = 800;
    public int H { get; set; } = 600;
    public int ThemeIdx { get; set; }
    public int BikeIdx { get; set; }

    public string ResStr => $"{W} x {H}";

    public int ResIdx
    {
        get => Math.Max(0, Array.FindIndex(Res, r => r.W == W && r.H == H));
        set { if ((uint)value < (uint)Res.Length) (W, H) = Res[value]; }
    }

    public static GameSettings Load()
    {
        try
        {
            if (!File.Exists(Path))
                return new();

            GameSettings s = JsonSerializer.Deserialize<GameSettings>(
                File.ReadAllText(Path)) ?? new();

            (s.W, s.H) = (Math.Clamp(s.W, 640, 3840), Math.Clamp(s.H, 480, 2160));
            (s.ThemeIdx, s.BikeIdx) = (Math.Max(0, s.ThemeIdx), Math.Clamp(s.BikeIdx, 0, 2));
            return s;
        }
        catch { return new(); }
    }

    public void Save()
    {
        try
        { File.WriteAllText(Path, JsonSerializer.Serialize(this, Opt)); }
        catch { }
    }

    public void NextRes() => ResIdx = (ResIdx + 1) % Res.Length;
    public void PrevRes() => ResIdx = (ResIdx - 1 + Res.Length) % Res.Length;
}

public enum GameState { MainMenu, Playing, Paused, GameOver, LevelComplete }

public sealed class GamePlay : IDisposable
{
    const int LvlCnt = 10;
    const float MaxFall = 2000f;

    bool _disposed;
    int _curLevelId = -1;

    public Bike? Bike { get; private set; } = new(BikeType.Standard);
    public Level? Level { get; private set; }
    public List<Level> Levels { get; } = [];
    public IReadOnlyList<BikeType> AvailBikes { get; } =
        [BikeType.Standard, BikeType.Sport, BikeType.OffRoad];
    public BikeType BikeType { get; private set; } = BikeType.Standard;
    public GameState State { get; private set; } = GameState.MainMenu;
    public TimeSpan Time { get; private set; }

    public void LoadLevels()
    {
        Levels.Clear();
        for (int i = 1; i <= LvlCnt; i++)
        {
            int diff = (i - 1) / 5 + 1;
            Levels.Add(Level.Create(i, $"Level {i}", seed: null, diff: diff));
        }
    }

    public void StartLevel(int id)
    {
        if (_curLevelId == id && State == GameState.Playing)
            return;

        Level? lv = Levels.FirstOrDefault(x => x.Id == id);
        if (lv == null)
            return;

        (_curLevelId, Level, Time) = (id, lv, TimeSpan.Zero);

        if (Bike != null)
        {
            Bike.SetInput(0f, 0f, 0f);
            Bike.SetType(BikeType);
            Bike.SetLevel(lv);
            Bike.Reset(lv.StartX);
        }

        State = GameState.Playing;
    }

    public void Update(float dt)
    {
        if (_disposed || State != GameState.Playing || Level == null || Bike == null)
            return;

        Time += TimeSpan.FromSeconds(dt);
        Bike.Update(dt);

        if (Level.IsFinish(Bike.Pos))
            State = GameState.LevelComplete;
        else if (Bike.Pos.Y > MaxFall || Bike.IsCrashed)
            State = GameState.GameOver;
    }

    public void HandleInput(BikeInput inp) =>
        Bike?.SetInput(inp.Throttle, inp.Brake, inp.Lean);

    public void Pause()
    {
        if (State == GameState.Playing)
            State = GameState.Paused;
    }

    public void Resume()
    {
        if (State == GameState.Paused)
            State = GameState.Playing;
    }

    public void ResetState()
    {
        _curLevelId = -1;
        State = GameState.MainMenu;
    }

    public void SetBike(BikeType t)
    {
        BikeType = t;
        Bike?.SetType(t);
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        Bike?.Dispose();
        Bike = null;
    }
}

public sealed class Game : Microsoft.Xna.Framework.Game
{
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
        TargetElapsedTime = TimeSpan.FromSeconds(1.0 / 60.0);
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
        _font = new("Content/Fonts/PixelOperator8.ttf");
        _rnd = new(_sb, new(
            () => _ctrl.Bike, () => _ctrl.Level,
            () => _cam.Zoom, () => _cam.Pos, () => _cam.Transform));
        _rnd.LoadBikeSprites(GraphicsDevice, "Content/Assets");
        RebuildUI();
        _ctrl.LoadLevels();
    }

    protected override void Update(GameTime time)
    {
        float dt = MathF.Min((float)time.ElapsedGameTime.TotalSeconds, 0.1f);
        InputState inp = InputReader.Read();

        if (inp.JustPressed(Keys.F1))
            _rnd.ToggleBikeRenderMode();

        _ui.Update(inp);

        if (_ctrl.State == GameState.Playing)
        {
            _ctrl.HandleInput(inp.Bike);
            _ctrl.Update(dt);

            if (_ctrl.Bike?.Pos is { } p &&
                float.IsFinite(p.X) && float.IsFinite(p.Y))
            {
                _cam.Follow(p);
                _ctrl.Level?.UpdateVisible(
                    p.X - _cam.HalfViewWidth,
                    p.X + _cam.HalfViewWidth, 200f);
            }

            _cam.Scroll(inp.ScrollDelta);
        }

        base.Update(time);
    }

    protected override void Draw(GameTime time)
    {
        GraphicsDevice.Clear(ThemeManager.Cur.BgColor);
        _rnd.RenderBackground();

        if (_ctrl.Level != null)
            _rnd.RenderGame();

        _sb.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);
        _ui.Draw();
        _sb.End();

        base.Draw(time);
    }

    public void SetResolution(int w, int h) => _disp.Set(w, h);

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
        _ui = new(new(_sb, _font.Get(20), _font.Get(40), _disp.W, _disp.H),
            _ctrl, this, _cfg);

    void OnResize(int w, int h)
    {
        _cam.Resize(w, h);
        (_cfg.W, _cfg.H) = (w, h);
        _cfg.Save();
        RebuildUI();
    }
}
