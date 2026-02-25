using BikeInput = GravityDefiedGame.Models.Bike.BikeInput;
using Vector2 = Microsoft.Xna.Framework.Vector2;
using static GravityDefiedGame.Views.UI;
using static GravityDefiedGame.Views.C;
using static GravityDefiedGame.Core.ThemeManager;
using static GravityDefiedGame.Core.GamePlay;

namespace GravityDefiedGame.Core;

[Flags]
public enum Hotkey : byte { None = 0, F1 = 1 << 0, F2 = 1 << 1, F4 = 1 << 2 }

public enum GameState : byte { MainMenu, Playing, Paused, GameOver, LevelComplete }

public readonly record struct MouseInput(
    Point Pos, Point Delta, bool Left, bool LeftWas, bool Right, int Scroll);

public readonly record struct InputState(
    MouseInput Mouse, KeyboardState Kb, KeyboardState Pk, BikeInput Bike, Hotkey Hot)
{
    public bool Has(Hotkey h) => (Hot & h) != 0;
}

public static class Input
{
    static MouseState _pm;
    static KeyboardState _pk;

    static float Axis(KeyboardState k, Keys key) => k.IsKeyDown(key) ? 1f : 0f;
    static bool Pressed(KeyboardState k, KeyboardState pk, Keys key) => k.IsKeyDown(key) && pk.IsKeyUp(key);

    static Hotkey ReadHotkeys(KeyboardState k, KeyboardState pk)
    {
        Hotkey hot = Hotkey.None;
        if (Pressed(k, pk, Keys.F1)) hot |= Hotkey.F1;
        if (Pressed(k, pk, Keys.F2)) hot |= Hotkey.F2;
        if (Pressed(k, pk, Keys.F4)) hot |= Hotkey.F4;
        return hot;
    }

    public static InputState Read()
    {
        var pm = _pm;
        var m = _pm = Mouse.GetState();

        var pk = _pk;
        var k = _pk = Keyboard.GetState();

        var mouse = new MouseInput(
            new(m.X, m.Y),
            new(m.X - pm.X, m.Y - pm.Y),
            m.LeftButton == ButtonState.Pressed,
            pm.LeftButton == ButtonState.Pressed,
            m.RightButton == ButtonState.Pressed,
            m.ScrollWheelValue - pm.ScrollWheelValue);

        var bike = new BikeInput(
            Axis(k, Keys.W),
            Axis(k, Keys.S),
            Axis(k, Keys.D) - Axis(k, Keys.A));

        return new InputState(mouse, k, pk, bike, ReadHotkeys(k, pk));
    }

    public static InputState ReadApply(GamePlay gp, Renderer rnd, GameSettings cfg)
    {
        var inp = Read();

        if (inp.Has(Hotkey.F1))
            rnd.ToggleSprites();

        if (inp.Has(Hotkey.F2))
        {
            rnd.CycleMode();
            cfg.RenderIdx = (int)rnd.Mode;
            cfg.Save();
        }

#if DEBUG
        if (inp.Has(Hotkey.F4))
            gp.Bike.DebugNodesEnabled = !gp.Bike.DebugNodesEnabled;
#endif

        if (gp.State == GameState.Playing)
            gp.HandleInput(inp.Bike);

        return inp;
    }
}

public sealed class GamePlay : IDisposable
{
    public const float BikeScale = 6f;
    public const int LvlCnt = 10;

    const float FallLimit = 2000f;
    const float CrashTimeout = 3f;

    static readonly BikeType[] BikeTypes = [BikeType.Standard, BikeType.Sport, BikeType.OffRoad];

    bool _disposed, _timing;
    int _bikeIdx;
    float _crashTimer;

    public Bike Bike { get; } = new(BikeType.Standard);
    public Level? Level { get; private set; }
    public GameState State { get; private set; } = GameState.MainMenu;
    public TimeSpan Time { get; private set; }
    public int LevelIdx { get; private set; }

    public string TimeText => $"{Time:mm\\:ss}";

    public static int BikeCnt => BikeTypes.Length;
    public static BikeType GetBike(int i) => BikeTypes[Math.Clamp(i, 0, BikeCnt - 1)];

    static string LvlName(int i) => $"Level {i + 1}";

    public bool TryStart(int bikeIdx, int levelIdx)
    {
        if ((uint)levelIdx >= LvlCnt) return false;

        _bikeIdx = Math.Clamp(bikeIdx, 0, BikeCnt - 1);
        LevelIdx = levelIdx;

        Level = Level?.Id == levelIdx + 1
            ? Level
            : Level.Create(levelIdx + 1, LvlName(levelIdx), null, 1);

        ResetRun();
        State = GameState.Playing;

        var lvl = Level!;
        Bike.SetType(GetBike(_bikeIdx));
        Bike.SetTerrain(lvl);
        ResetBike(lvl.StartX / BikeScale);

        return true;
    }

    public void Restart()
    {
        if (Level != null)
            TryStart(_bikeIdx, LevelIdx);
    }

    public bool TryNext()
    {
        if (Level == null || LevelIdx + 1 >= LvlCnt) return false;
        return TryStart(_bikeIdx, LevelIdx + 1);
    }

    public bool PauseIfPossible()
    {
        if (State != GameState.Playing || Bike.Crashed) return false;
        State = GameState.Paused;
        return true;
    }

    public void Resume()
    {
        if (State == GameState.Paused)
            State = GameState.Playing;
    }

    public void Update(float dt)
    {
        if (_disposed || Level == null) return;

        if (State == GameState.Playing) UpdatePlaying(dt);
        else if (State == GameState.GameOver) Bike.Update(dt);
    }

    void UpdatePlaying(float dt)
    {
        float px = Bike.Pos.X * BikeScale;
        Bike.Update(dt);
        float cx = Bike.Pos.X * BikeScale;

        if (Bike.Pos.Y * BikeScale > FallLimit)
        {
            _timing = false;
            State = GameState.GameOver;
            return;
        }

        if (Bike.Crashed)
        {
            UpdateCrash(dt);
            return;
        }

        var lvl = Level!;
        _timing |= lvl.CrossedStartGate(px, cx);
        if (!_timing) return;

        Time += TimeSpan.FromSeconds(dt);
        if (lvl.CrossedFinishGate(px, cx))
            State = GameState.LevelComplete;
    }

    void UpdateCrash(float dt)
    {
        _timing = false;
        _crashTimer += dt;
        if (Bike.RagdollDone || _crashTimer >= CrashTimeout)
            State = GameState.GameOver;
    }

    public void HandleInput(BikeInput inp)
    {
        if (!Bike.Crashed)
            Bike.SetInput(inp.Throttle, inp.Brake, inp.Lean);
    }

    public void Reset()
    {
        Level = null;
        LevelIdx = 0;
        _bikeIdx = 0;
        State = GameState.MainMenu;
        ResetRun();
        ResetBike(0f);
    }

    void ResetRun()
    {
        Time = TimeSpan.Zero;
        _timing = false;
        _crashTimer = 0f;
    }

    void ResetBike(float x)
    {
        Bike.SetInput(0f, 0f, 0f);
        Bike.Reset(x);
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        Bike.Dispose();
    }
}

public sealed class Camera
{
    const float DefZoom = 3f, MinZoom = 0.5f, MaxZoom = 5f;
    const float FollowK = 0.1f, ScrollK = 0.001f, YOff = -60f;

    int _w, _h;

    public Camera(int w, int h) => (_w, _h) = (w, h);

    public float Zoom { get; private set; } = DefZoom;
    public Vector2 Pos { get; private set; }
    public float HalfW => _w * 0.5f / Zoom;

    public Matrix Transform =>
        Matrix.CreateTranslation(-Pos.X, -Pos.Y, 0f) *
        Matrix.CreateScale(Zoom) *
        Matrix.CreateTranslation(_w * 0.5f, _h * 0.5f, 0f);

    public void Follow(Vector2 target) =>
        Pos = Vector2.Lerp(Pos, new(target.X, target.Y + YOff), FollowK);

    public void Scroll(int delta) =>
        Zoom = MathHelper.Clamp(Zoom + delta * ScrollK, MinZoom, MaxZoom);

    public void Track(Bike bike, Level lvl, int scroll, float scale, float margin)
    {
        float bx = bike.Pos.X * scale;
        Vector2 target = (bike.RagdollActive ? bike.RiderPose.Hd : bike.Pos) * scale;
        Follow(target);
        Scroll(scroll);
        lvl.UpdateVisible(bx - HalfW, bx + HalfW, margin);
    }

    public void Resize(int w, int h) => (_w, _h) = (w, h);
}

public sealed class GameSettings
{
    const string Path = "settings.json";

    static readonly (int W, int H)[] Res =
    [
        (800, 600), (1024, 768), (1280, 720),
        (1366, 768), (1600, 900), (1920, 1080)
    ];

    static readonly JsonSerializerOptions JsonOpt = new() { WriteIndented = true };

    public static int ResCount => Res.Length;

    public int W { get; set; } = 800;
    public int H { get; set; } = 600;
    public int ThemeIdx { get; set; }
    public int RenderIdx { get; set; }

    public string ResStr => $"{W} x {H}";
    public string RenderName => (RenderMode)RenderIdx == RenderMode.Flat ? "2D" : "VOXEL";

    public int ResIdx
    {
        get => Math.Max(0, Array.FindIndex(Res, r => r.W == W && r.H == H));
        set
        {
            if ((uint)value < (uint)Res.Length)
                (W, H) = Res[value];
        }
    }

    public void NextResolution() => ResIdx = (ResIdx + 1) % ResCount;
    public void NextRenderMode() => RenderIdx = (RenderIdx + 1) % 2;

    public void ApplyTheme(int idx)
    {
        ThemeIdx = Math.Max(0, idx);
        Save();
    }

    public static GameSettings Load()
    {
        if (!File.Exists(Path)) return new();
        string json = File.ReadAllText(Path);
        return JsonSerializer.Deserialize<GameSettings>(json) ?? new();
    }

    public void Save()
    {
        string json = JsonSerializer.Serialize(this, JsonOpt);
        File.WriteAllText(Path, json);
    }
}

sealed class Screens
{
    enum Scr : byte { Main, Bike, Level, Theme, Settings, Play, Pause, Over, Done }

    readonly GamePlay _gp;
    readonly GameSettings _cfg;
    readonly Action<int, int> _resize;
    readonly Dictionary<(Scr, Keys), Action> _keys;

    Scr _scr;
    int _bike, _level, _theme;
    Label? _time, _spd;

    public Screens(GamePlay gp, GameSettings cfg, Action<int, int> resize)
    {
        _gp = gp;
        _cfg = cfg;
        _resize = resize;
        _theme = Idx;
        _keys = BuildKeyMap();
        Go(Scr.Main);
    }

    public void Update(KeyboardState kb, KeyboardState pk)
    {
        if (GetPressed(kb, pk) is Keys k)
            HandleKey(k);

        if (_scr != Scr.Play) return;

        if (_gp.State == GameState.GameOver) Go(Scr.Over);
        else if (_gp.State == GameState.LevelComplete) Go(Scr.Done);
        else UpdateHud();
    }

    public void Rebuild() => Go(_scr);

    Dictionary<(Scr, Keys), Action> BuildKeyMap()
    {
        var map = new Dictionary<(Scr, Keys), Action>
        {
            [(Scr.Main, Keys.Enter)]      = () => Go(Scr.Bike),
            [(Scr.Main, Keys.Escape)]     = Exit,

            [(Scr.Settings, Keys.Escape)] = SaveAndBack,
            [(Scr.Play, Keys.Escape)]     = Pause,

            [(Scr.Pause, Keys.Enter)]     = Resume,
            [(Scr.Pause, Keys.Escape)]    = Resume,
            [(Scr.Pause, Keys.R)]         = Restart,

            [(Scr.Over, Keys.R)]          = Restart,
            [(Scr.Over, Keys.Enter)]      = Restart,
            [(Scr.Over, Keys.Escape)]     = ToMenu,

            [(Scr.Done, Keys.R)]          = Restart,
            [(Scr.Done, Keys.Enter)]      = Next,
            [(Scr.Done, Keys.Escape)]     = ToMenu
        };

        foreach (Scr s in (Scr[])[Scr.Bike, Scr.Level, Scr.Theme])
        {
            map[(s, Keys.Left)]   = () => Nav(-1);
            map[(s, Keys.Right)]  = () => Nav(1);
            map[(s, Keys.Enter)]  = Ok;
            map[(s, Keys.Escape)] = Back;
        }

        return map;
    }

    void HandleKey(Keys k)
    {
        if (_keys.TryGetValue((_scr, k), out var act))
            act();
    }

    void Go(Scr s)
    {
        _scr = s;
        UI.Desktop.Root = Build(s);
    }

    Widget? Build(Scr s) => s switch
    {
        Scr.Main => Menu("GRAVITY DEFIED", "BIKE PHYSICS GAME",
            ("PLAY", () => Go(Scr.Bike)),
            ("SETTINGS", () => Go(Scr.Settings)),
            ("EXIT", Exit)),

        Scr.Bike  => BuildPicker(BikePicker(_bike)),
        Scr.Level => BuildPicker(LvlPicker(_level)),
        Scr.Theme => BuildPicker(ThemePicker(_theme)),

        Scr.Settings => BuildSettings(),
        Scr.Play     => BuildHud(),

        Scr.Pause => Overlay("PAUSED", null,
            ("RESUME", Resume),
            ("RESTART", Restart),
            ("MENU", ToMenu)),

        Scr.Over => Overlay("GAME OVER", "YOU CRASHED!",
            ("RESTART", Restart),
            ("MENU", ToMenu)),

        Scr.Done => Overlay("COMPLETE!", $"TIME: {_gp.TimeText}",
            ("NEXT", Next),
            ("RESTART", Restart),
            ("MENU", ToMenu)),

        _ => null
    };

    Panel BuildPicker(PickerInfo p) =>
        Pick(p.Title, p.Value, p.Counter, p.Desc, Nav, Ok, Back);

    Panel BuildSettings() =>
        Col()
            .Add("SETTINGS".Title())
            .Add(Gap(15))
            .Add($"RES: {_cfg.ResStr}".Btn().On(CycleRes))
            .Add($"THEME: {ThemeName(_theme).ToUpper()}".Btn().On(() => Go(Scr.Theme)))
            .Add($"RENDER: {_cfg.RenderName}".Btn().On(CycleRender))
            .Add(Gap(15))
            .Add("BACK".Btn().On(SaveAndBack))
            .Frame();

    Panel BuildHud()
    {
        _time = "00:00".Text().Yellow();
        _spd = "SPD: 0".Text();
        string lv = _gp.Level?.Name.ToUpper() ?? "";

        return new Panel()
            .Bg(Bg).Pad(15, 8).Mar(10, 0, 10, 10)
            .Bottom().Stretch()
            .Add(Grd(3)
                .Cell($"LV: {lv}".Text().Left(), 0)
                .Cell(_time.Center(), 1)
                .Cell(_spd.Right(), 2));
    }

    void UpdateHud()
    {
        _time?.Set(_gp.TimeText);
        _spd?.Set($"SPD: {_gp.Bike.Speed:F0}");
    }

    void Nav(int dir)
    {
        switch (_scr)
        {
            case Scr.Bike:  _bike  = Step(_bike, BikeCnt, dir, true);  break;
            case Scr.Level: _level = Step(_level, LvlCnt, dir, false); break;
            case Scr.Theme: _theme = NextTheme(_theme, dir);           break;
            default: return;
        }

        Go(_scr);
    }

    void Ok()
    {
        if (_scr == Scr.Bike) Go(Scr.Level);
        else if (_scr == Scr.Level) StartGame();
        else if (_scr == Scr.Theme) Go(Scr.Settings);
    }

    void Back()
    {
        if (_scr == Scr.Bike) Go(Scr.Main);
        else if (_scr == Scr.Level) Go(Scr.Bike);
        else if (_scr == Scr.Theme) Go(Scr.Settings);
    }

    void StartGame()
    {
        if (_gp.TryStart(_bike, _level))
            Go(Scr.Play);
    }

    void Pause()
    {
        if (_gp.PauseIfPossible())
            Go(Scr.Pause);
    }

    void Resume()  => GoAfter(_gp.Resume, Scr.Play);
    void Restart() => GoAfter(_gp.Restart, Scr.Play);
    void ToMenu()  => GoAfter(_gp.Reset, Scr.Main);

    void Next()
    {
        if (_gp.TryNext())
        {
            _level = _gp.LevelIdx;
            Go(Scr.Play);
            return;
        }

        ToMenu();
    }

    void CycleRes()
    {
        _cfg.NextResolution();
        _resize(_cfg.W, _cfg.H);
        Go(Scr.Settings);
    }

    void CycleRender()
    {
        _cfg.NextRenderMode();
        Go(Scr.Settings);
    }

    void SaveAndBack()
    {
        _cfg.ApplyTheme(_theme);
        Go(Scr.Main);
    }

    void GoAfter(Action act, Scr next)
    {
        act();
        Go(next);
    }

    static PickerInfo BikePicker(int i) =>
        new("SELECT BIKE", GetBike(i).ToString().ToUpper(), Fmt(i, BikeCnt));

    static PickerInfo LvlPicker(int i) =>
        new("SELECT LEVEL", $"LEVEL {i + 1}", Fmt(i, LvlCnt));

    static Keys? GetPressed(KeyboardState kb, KeyboardState pk)
    {
        foreach (Keys k in kb.GetPressedKeys())
            if (pk.IsKeyUp(k))
                return k;
        return null;
    }

    static void Exit() => Environment.Exit(0);
}

public sealed class Game : Microsoft.Xna.Framework.Game
{
    const float MaxDt = 0.1f;
    const float CamMargin = 200f;

    readonly GraphicsDeviceManager _gfx;
    readonly GameSettings _cfg;
    readonly GamePlay _gp = new();

    SpriteBatch _sb = null!;
    Camera _cam = null!;
    Renderer _rnd = null!;
    SimpleUI _ui = null!;
    Screens _scr = null!;

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
        TargetElapsedTime = TimeSpan.FromSeconds(1d / 60d);
    }

    protected override void Initialize()
    {
        Set(_cfg.ThemeIdx);
        base.Initialize();
    }

    protected override void LoadContent()
    {
        _sb = new SpriteBatch(GraphicsDevice);
        _cam = new Camera(_cfg.W, _cfg.H);

        _rnd = new Renderer(_sb, new(_gp, _cam), _cfg.W, _cfg.H);
        _rnd.LoadSprites(GraphicsDevice);
        _rnd.SetMode((RenderMode)_cfg.RenderIdx);

        MyraEnvironment.Game = this;
        _ui = new SimpleUI("Content/Fonts/PixelOperator8.ttf");
        _scr = new Screens(_gp, _cfg, ApplyRes);

        Window.ClientSizeChanged += OnWindowResize;
    }

    protected override void Update(GameTime gt)
    {
        float dt = MathF.Min((float)gt.ElapsedGameTime.TotalSeconds, MaxDt);
        InputState inp = Input.ReadApply(_gp, _rnd, _cfg);

        _scr.Update(inp.Kb, inp.Pk);
        _gp.Update(dt);

        if (_gp.Level is Level lvl)
        {
            _rnd.Input(in inp);
            _cam.Track(_gp.Bike, lvl, inp.Mouse.Scroll, BikeScale, CamMargin);
        }

        _rnd.Update(dt);
        base.Update(gt);
    }

    protected override void Draw(GameTime gt)
    {
        GraphicsDevice.Clear(Cur.BgColor);
        _rnd.Background();

        if (_gp.Level != null)
            _rnd.Draw();

        UI.Desktop.Render();

#if DEBUG
        _gp.Bike.DrawDebug(
            _sb, Font, GraphicsDevice, _cam.Transform,
            _rnd.Mode == RenderMode.Voxel ? _rnd.VoxCam : null);
#endif

        base.Draw(gt);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            Window.ClientSizeChanged -= OnWindowResize;
            _ui?.Dispose();
            _rnd?.Dispose();
            _sb?.Dispose();
            _gp.Dispose();
        }

        base.Dispose(disposing);
    }

    void ApplyRes(int w, int h)
    {
        _gfx.PreferredBackBufferWidth = w;
        _gfx.PreferredBackBufferHeight = h;
        _gfx.ApplyChanges();
        ApplySize(w, h);
    }

    void OnWindowResize(object? sender, EventArgs e) =>
        ApplySize(Window.ClientBounds.Width, Window.ClientBounds.Height);

    void ApplySize(int w, int h)
    {
        _cam.Resize(w, h);
        _rnd.Resize(w, h);
        (_cfg.W, _cfg.H) = (w, h);
        _cfg.Save();
        _scr.Rebuild();
    }
}
