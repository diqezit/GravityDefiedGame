using static GravityDefiedGame.Core.BikeDefs;
using static GravityDefiedGame.Core.ThemeManager;
using Game = GravityDefiedGame.Core.Game;
using Thickness = Myra.Graphics2D.Thickness;
using MColor = Microsoft.Xna.Framework.Color;

namespace GravityDefiedGame.Views.UI;

public enum Screen : byte
{
    Main, Bike, Level, Theme, Settings, Play, Pause, Over, Done
}

public readonly struct Theme
{
    public readonly MColor Title, Sub, Accent, Bg, Border;

    public Theme()
    {
        Title = MColor.White;
        Sub = MColor.LightGray;
        Accent = MColor.Yellow;
        Bg = new(30, 30, 30, 240);
        Border = new(140, 140, 140);
    }
}

public sealed class ScreenMachine : IDisposable
{

#if DEBUG
    public SpriteFontBase DebugFont => _fSm;
#endif

    readonly record struct SelectorCfg(
        string Title, string OkText,
        int Idx, int Count, bool Wrap,
        Func<int, string> GetName, Func<int, string?>? GetDesc,
        Action<int> OnSelect, Action OnOk, Action OnBack);

    const string FontPath = "Content/Fonts/PixelOperator8.ttf";
    const int FontSm = 18, FontLg = 28, BtnW = 180;

    static readonly Theme T = new();

    readonly Dictionary<(Screen, Keys), Action> _keys;
    readonly Desktop _desktop = new();
    readonly FontSystem _fs;
    readonly SpriteFontBase _fSm, _fLg;
    readonly GamePlay _gp;
    readonly Game _game;
    readonly GameSettings _cfg;

    Screen _scr = (Screen)255;
    int _bike, _level, _theme;
    Label? _lblTime, _lblSpd;

    public ScreenMachine(GamePlay gp, Game game, GameSettings cfg)
    {
        (_gp, _game, _cfg, _theme) = (gp, game, cfg, Idx);

        _fs = new FontSystem();
        _fs.AddFont(File.ReadAllBytes(FontPath));
        (_fSm, _fLg) = (_fs.GetFont(FontSm), _fs.GetFont(FontLg));

        _keys = BuildKeyMap();
        Go(Screen.Main);
    }

    Dictionary<(Screen, Keys), Action> BuildKeyMap() => new()
    {
        [(Screen.Main, Keys.Enter)] = GoBike,
        [(Screen.Main, Keys.Escape)] = Exit,

        [(Screen.Bike, Keys.Left)] = () => NavBike(-1),
        [(Screen.Bike, Keys.Right)] = () => NavBike(1),
        [(Screen.Bike, Keys.Enter)] = GoLevel,
        [(Screen.Bike, Keys.Escape)] = GoMain,

        [(Screen.Level, Keys.Left)] = () => NavLevel(-1),
        [(Screen.Level, Keys.Right)] = () => NavLevel(1),
        [(Screen.Level, Keys.Enter)] = StartCurrentLevel,
        [(Screen.Level, Keys.Escape)] = GoBike,

        [(Screen.Theme, Keys.Left)] = () => NavTheme(-1),
        [(Screen.Theme, Keys.Right)] = () => NavTheme(1),
        [(Screen.Theme, Keys.Enter)] = ApplyTheme,
        [(Screen.Theme, Keys.Escape)] = GoMain,

        [(Screen.Settings, Keys.Escape)] = GoMain,

        [(Screen.Play, Keys.Escape)] = Pause,

        [(Screen.Pause, Keys.Enter)] = Resume,
        [(Screen.Pause, Keys.Escape)] = Resume,
        [(Screen.Pause, Keys.R)] = Restart,

        [(Screen.Over, Keys.Enter)] = Restart,
        [(Screen.Over, Keys.R)] = Restart,
        [(Screen.Over, Keys.Escape)] = ToMenu,

        [(Screen.Done, Keys.Enter)] = NextLevel,
        [(Screen.Done, Keys.R)] = Restart,
        [(Screen.Done, Keys.Escape)] = ToMenu,
    };

    public void Go(Screen s)
    {
        _scr = s;
        _desktop.Root = s switch
        {
            Screen.Main => BuildMain(),
            Screen.Bike or Screen.Level or Screen.Theme => Selector(SelCfg(s)),
            Screen.Settings => BuildSettings(),
            Screen.Play => BuildHud(),
            Screen.Pause => Dialog("PAUSED", null,
                ("RESUME", Resume), ("RESTART", Restart), ("MENU", ToMenu)),
            Screen.Over => Dialog("GAME OVER", "YOU CRASHED!",
                ("RESTART", Restart), ("MENU", ToMenu)),
            Screen.Done => Dialog("COMPLETE!", $"TIME: {_gp.Time:mm\\:ss}",
                ("NEXT", NextLevel), ("RESTART", Restart), ("MENU", ToMenu)),
            _ => null
        };
    }

    public void Update(InputState inp)
    {
        if (GetKey(inp) is { } k && _keys.TryGetValue((_scr, k), out Action? act))
            act();

        CheckState();

        if (_scr == Screen.Play)
            UpdateHud();
    }

    public void Draw() => _desktop.Render();

    public void Dispose() => _fs.Dispose();

    SelectorCfg SelCfg(Screen s) => s switch
    {
        Screen.Bike => new(
            "SELECT BIKE", "SELECT",
            _bike, Count, true,
            GetBikeName, null,
            SelectBike, GoLevel, GoMain),

        Screen.Level => new(
            "SELECT LEVEL", "START",
            _level, _gp.Levels.Count, false,
            GetLevelName, null,
            SelectLevel, StartCurrentLevel, GoBike),

        Screen.Theme => new(
            "SELECT THEME", "APPLY",
            _theme, Cnt, true,
            NameUpper, DescUpper,
            SelectTheme, ApplyTheme, GoMain),

        _ => throw new ArgumentOutOfRangeException(nameof(s))
    };

    void CheckState()
    {
        if (_gp.State == GameState.MainMenu)
            return;

        bool gameOver = _gp.Bike?.Crashed == true
            || _gp.State == GameState.GameOver;

        if (gameOver && _scr != Screen.Over)
            Go(Screen.Over);
        else if (_gp.State == GameState.LevelComplete && _scr != Screen.Done)
            Go(Screen.Done);
    }

    void UpdateHud()
    {
        if (_lblTime != null)
            _lblTime.Text = $"{_gp.Time:mm\\:ss}";

        if (_lblSpd != null)
            _lblSpd.Text = $"SPD: {_gp.Bike?.Speed ?? 0:F0}";
    }

    static Keys? GetKey(InputState inp)
    {
        ReadOnlySpan<Keys> keys =
            [Keys.Enter, Keys.Escape, Keys.Left, Keys.Right, Keys.R];

        foreach (Keys k in keys)
            if (inp.JustPressed(k))
                return k;

        return null;
    }

    Panel BuildMain() =>
        Col(20)
            .Add(Title("GRAVITY DEFIED"))
            .Add(Lbl("BIKE PHYSICS GAME").Color(T.Sub))
            .Add(Gap(30))
            .Add(Btn("PLAY", GoBike))
            .Add(Btn("SETTINGS", GoSettings))
            .Add(Btn("THEMES", GoTheme))
            .Add(Btn("EXIT", Exit))
            .Frame(T.Bg, T.Border);

    Panel Selector(SelectorCfg cfg)
    {
        int cur = cfg.Count > 0 ? Math.Clamp(cfg.Idx, 0, cfg.Count - 1) : 0;
        cfg.OnSelect(cur);

        HorizontalStackPanel arrows = Row(100)
            .Add(Btn("<", () => Step(cur, -1, cfg.Count, cfg.Wrap, cfg.OnSelect)))
            .Add(Btn(">", () => Step(cur, 1, cfg.Count, cfg.Wrap, cfg.OnSelect)))
            .Center();

        VerticalStackPanel stack = Col(12)
            .Add(Title(cfg.Title))
            .Add(Gap(15))
            .Add(arrows)
            .Add(Lbl(cfg.GetName(cur)).Color(T.Accent))
            .Add(Lbl(FormatCounter(cur, cfg.Count)));

        string? desc = cfg.GetDesc?.Invoke(cur);
        if (desc != null)
            stack.Add(Gap(8)).Add(Lbl(desc).Color(T.Sub));

        return stack
            .Add(Gap(20))
            .Add(Btn(cfg.OkText, cfg.OnOk))
            .Add(Btn("BACK", cfg.OnBack))
            .Frame(T.Bg, T.Border);
    }

    Panel BuildSettings() =>
        Col(15)
            .Add(Title("SETTINGS"))
            .Add(Gap(20))
            .Add(SettingsRow("RES:", _cfg.ResStr, _cfg.PrevRes, _cfg.NextRes))
            .Add(SettingsRow("RENDER:", _cfg.RenderStr, _cfg.PrevRender, _cfg.NextRender))
            .Add(Gap(20))
            .Add(Btn("APPLY", ApplySettings))
            .Add(Btn("BACK", GoMain))
            .Frame(T.Bg, T.Border);

    Panel BuildHud()
    {
        string level = _gp.Level?.Name.ToUpperInvariant() ?? "";

        _lblTime = Lbl("00:00").Color(T.Accent);
        _lblSpd = Lbl("SPD: 0");

        return new Panel()
            .Bg(T.Bg)
            .Pad(15, 8)
            .Mar(10, 0, 10, 10)
            .Bottom()
            .Stretch()
            .Add(new Grid { ColumnSpacing = 8 }
                .Cols(3)
                .Cell(Lbl($"LV: {level}").Left(), 0)
                .Cell(_lblTime.Center(), 1)
                .Cell(_lblSpd.Right(), 2));
    }

    HorizontalStackPanel SettingsRow(string label, string value, Action onPrev, Action onNext) =>
        Row(10)
            .Add(Lbl(label))
            .Add(Btn("<", Rebuild(onPrev)))
            .Add(Lbl(value.ToUpperInvariant()).Color(T.Accent).Width(150))
            .Add(Btn(">", Rebuild(onNext)))
            .Center();

    Panel Dialog(string title, string? sub, params (string text, Action action)[] buttons)
    {
        VerticalStackPanel stack = Col(12).Add(Title(title));

        if (sub != null)
            stack.Add(Lbl(sub).Color(T.Accent));

        stack.Add(Gap(20));

        foreach ((string text, Action action) in buttons)
            stack.Add(Btn(text, action));

        return new Panel()
            .Bg(MColor.Black * 0.6f)
            .Stretch()
            .StretchV()
            .Add(stack.Frame(T.Bg, T.Border));
    }

    static VerticalStackPanel Col(int spacing) =>
        new VerticalStackPanel { Spacing = spacing }
            .Center()
            .CenterV();

    static HorizontalStackPanel Row(int spacing) =>
        new HorizontalStackPanel { Spacing = spacing }
            .CenterV();

    Label Title(string text) =>
        new Label { Text = text, Font = _fLg, TextColor = T.Title }
            .Center();

    Label Lbl(string text) =>
        new Label { Text = text, Font = _fSm, TextColor = T.Title }
            .Center();

    static Panel Gap(int height) => new() { Height = height };

    Button Btn(string text, Action onClick) =>
        new Button
        {
            Content = new Label { Text = text, Font = _fSm }.Center(),
            Width = BtnW
        }
        .Center()
        .OnClick(onClick);

    static string FormatCounter(int cur, int count) =>
        count > 0 ? $"{cur + 1}/{count}" : "0/0";

    static string GetBikeName(int i) => Name(i).ToUpperInvariant();

    static string GetLevelName(int i) => $"LEVEL {i + 1}";

    void Step(int cur, int delta, int count, bool wrap, Action<int> onSelect)
    {
        int next = count > 0
            ? wrap ? Cycle(cur, count, delta)
                   : Math.Clamp(cur + delta, 0, count - 1)
            : 0;
        onSelect(next);
        Go(_scr);
    }

    void SelectBike(int i)
    {
        _bike = i;
        _gp.SetBike(Types[i]);
    }

    void SelectLevel(int i) => _level = i;

    void SelectTheme(int i) => _theme = i;

    void NavBike(int d) => Step(_bike, d, Count, true, SelectBike);

    void NavLevel(int d) => Step(_level, d, _gp.Levels.Count, false, SelectLevel);

    void NavTheme(int d) => Step(_theme, d, Cnt, true, SelectTheme);

    Action Rebuild(Action change) => () =>
    {
        change();
        Go(Screen.Settings);
    };

    void GoMain() => Go(Screen.Main);
    void GoBike() => Go(Screen.Bike);
    void GoLevel() => Go(Screen.Level);
    void GoTheme() => Go(Screen.Theme);
    void GoSettings() => Go(Screen.Settings);

    void StartCurrentLevel() => StartLevel(_level + 1);

    void StartLevel(int id)
    {
        _gp.StartLevel(id);
        Go(Screen.Play);
    }

    void Pause()
    {
        _gp.Pause();
        Go(Screen.Pause);
    }

    void Resume()
    {
        _gp.Resume();
        Go(Screen.Play);
    }

    void Restart()
    {
        if (_gp.Level is { } lv)
            StartLevel(lv.Id);
    }

    void ToMenu()
    {
        _gp.ResetState();
        Go(Screen.Main);
    }

    void NextLevel()
    {
        if (_gp.Level is { } lv && lv.Id < _gp.Levels.Count)
        {
            _level = lv.Id;
            StartLevel(lv.Id + 1);
        }
        else
            ToMenu();
    }

    void ApplyTheme()
    {
        Apply(_theme);
        Go(Screen.Main);
    }

    void ApplySettings()
    {
        _game.SetResolution(_cfg.W, _cfg.H);
        _game.ApplyRenderMode();
        _cfg.Save();
    }

    static void Exit() => Environment.Exit(0);
}

public static class FluentApi
{
    public static T Center<T>(this T w) where T : Widget
    {
        w.HorizontalAlignment = HorizontalAlignment.Center;
        return w;
    }

    public static T CenterV<T>(this T w) where T : Widget
    {
        w.VerticalAlignment = VerticalAlignment.Center;
        return w;
    }

    public static T Left<T>(this T w) where T : Widget
    {
        w.HorizontalAlignment = HorizontalAlignment.Left;
        return w;
    }

    public static T Right<T>(this T w) where T : Widget
    {
        w.HorizontalAlignment = HorizontalAlignment.Right;
        return w;
    }

    public static T Bottom<T>(this T w) where T : Widget
    {
        w.VerticalAlignment = VerticalAlignment.Bottom;
        return w;
    }

    public static T Stretch<T>(this T w) where T : Widget
    {
        w.HorizontalAlignment = HorizontalAlignment.Stretch;
        return w;
    }

    public static T StretchV<T>(this T w) where T : Widget
    {
        w.VerticalAlignment = VerticalAlignment.Stretch;
        return w;
    }

    public static T Width<T>(this T w, int width) where T : Widget
    {
        w.Width = width;
        return w;
    }

    public static Label Color(this Label lbl, MColor color)
    {
        lbl.TextColor = color;
        return lbl;
    }

    public static T Bg<T>(this T w, MColor color) where T : Widget
    {
        w.Background = new SolidBrush(color);
        return w;
    }

    public static T Pad<T>(this T w, int h, int v) where T : Widget
    {
        w.Padding = new Thickness(h, v);
        return w;
    }

    public static T Mar<T>(this T w, int l, int t, int r, int b) where T : Widget
    {
        w.Margin = new Thickness(l, t, r, b);
        return w;
    }

    public static T Add<T>(this T c, params Widget[] ws) where T : Container
    {
        foreach (Widget w in ws)
            c.Widgets.Add(w);
        return c;
    }

    public static Grid Cols(this Grid g, int count)
    {
        for (int i = 0; i < count; i++)
            g.ColumnsProportions.Add(new(ProportionType.Part, 1));
        return g;
    }

    public static Grid Cell(this Grid g, Widget w, int col)
    {
        Grid.SetColumn(w, col);
        g.Widgets.Add(w);
        return g;
    }

    public static Panel Frame(this Widget content, MColor bg, MColor border)
    {
        Panel p = new Panel()
            .Bg(bg)
            .Pad(40, 30)
            .Center()
            .CenterV();

        p.Border = new SolidBrush(border);
        p.BorderThickness = new Thickness(2);
        p.Widgets.Add(content);
        return p;
    }

    public static Button OnClick(this Button btn, Action action)
    {
        btn.Click += (_, _) => action();
        return btn;
    }
}
