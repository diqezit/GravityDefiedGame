using Game = GravityDefiedGame.Core.Game;

namespace GravityDefiedGame.Views.UI;

public enum Screen
{
    MainMenu, BikeSelect, LevelSelect, ThemeSelect,
    Gameplay, Paused, GameOver, LevelComplete, Settings
}

public readonly struct Strings
{
    public readonly struct Titles
    {
        public const string
            Main = "GRAVITY DEFIED",
            Bike = "SELECT BIKE",
            Level = "SELECT LEVEL",
            Theme = "SELECT THEME",
            Paused = "PAUSED",
            Over = "GAME OVER",
            Complete = "LEVEL COMPLETE!",
            Settings = "SETTINGS";
    }

    public readonly struct Labels
    {
        public const string
            Play = "Play", Settings = "Settings",
            Themes = "Themes", Exit = "Exit",
            Back = "Back", Select = "Select",
            Start = "Start", Apply = "Apply",
            Resume = "Resume", Restart = "Restart",
            Menu = "Menu", NextLevel = "Next Level",
            Pause = "Pause", Prev = "←", Next = "→";

        public const string
            Sub = "Bike Physics Game",
            Crashed = "You crashed!",
            ResLabel = "Resolution:",
            ResHint = "←/→ change, Apply to confirm";
    }

    public static string Counter(int cur, int total) =>
        $"{cur + 1}/{total}";

    public static string LevelN(int idx) =>
        $"Level {idx + 1}";

    public static string TimeFmt(TimeSpan t) =>
        $"Time: {t:mm\\:ss}";

    public static string SpeedFmt(float spd) =>
        $"Speed: {spd:F1}";

    public static string LevelLabel(string name) =>
        $"Level: {name}";

    public static string ResFmt(string res) =>
        $"← {res} →";
}

public readonly struct Lay
{
    public readonly struct Title
    {
        public const int Y = 80, SubGap = 80;
    }

    public readonly struct Menu
    {
        public const int SubY = 160,
            Btn0 = -80, Step = 60;
    }

    public readonly struct Carousel
    {
        public const int ArrowX = 300, ArrowOffX = 220,
            ArrowW = 80, ArrowH = 50,
            NameY = -30, CounterY = 20,
            DescY = 60, SelectY = 100;
    }

    public readonly struct BackBtn
    {
        public const int X = 20, Y = 20, W = 120, H = 40;
    }

    public readonly struct Dialog
    {
        public const int TitleOff = -120, SubOff = -50,
            Btn0 = 0, Btn1 = 20, Step = 60;
    }

    public readonly struct Hud
    {
        public const int OffBottom = 120, W = 200, H = 100,
            Pad = 10, LineH = 30, X = 20,
            PauseX = 140, PauseY = 20,
            PauseW = 120, PauseH = 40;
    }

    public readonly struct Cfg
    {
        public const int TitleY = 60, ApplyOffBottom = 80,
            LabelY = 0, ValueY = 30, HintOff = 130;
    }

    public readonly struct Ovr
    {
        public const float Default = 0.5f;
    }
}

internal sealed class ScreenCfg
{
    public string? Title;
    public Color TitleColor = Color.White;
    public int TitleY = Lay.Title.Y;
    public string? Sub;
    public Color SubColor = Color.White;
    public int SubY;
    public float Overlay;
    public List<(Func<Rectangle> Bounds, string Text,
        Color Color, Action Click)> Btns = [];
    public Dictionary<Keys, Action> KeyMap = [];
    public Action<SimpleUI, GamePlay>? Content;
}

internal sealed class ScreenBuilder(SimpleUI ui)
{
    readonly ScreenCfg _c = new();

    public ScreenBuilder Title(
        string t, Color c, int? y = null)
    {
        (_c.Title, _c.TitleColor) = (t, c);
        if (y.HasValue)
            _c.TitleY = y.Value;
        return this;
    }

    public ScreenBuilder Sub(
        string t, Color? c = null, int? y = null)
    {
        (_c.Sub, _c.SubColor, _c.SubY) =
            (t, c ?? Color.White,
             y ?? _c.TitleY + Lay.Title.SubGap);
        return this;
    }

    public ScreenBuilder Ovr(float a = Lay.Ovr.Default)
    {
        _c.Overlay = a;
        return this;
    }

    public ScreenBuilder BtnC(
        string t, Color c, Action a, int yOff = 0)
    {
        _c.Btns.Add((
            () => new(ui.C.X - Pal.BtnW / 2,
                ui.C.Y + yOff, Pal.BtnW, Pal.BtnH),
            t, c, a));
        return this;
    }

    public ScreenBuilder Btn(
        string t, Color c, Action a,
        int x, int y, int w, int h)
    {
        _c.Btns.Add((() => new(x, y, w, h), t, c, a));
        return this;
    }

    public ScreenBuilder BtnAt(
        string t, Color c, Action a,
        Func<SimpleUI, Rectangle> b)
    {
        _c.Btns.Add((() => b(ui), t, c, a));
        return this;
    }

    public ScreenBuilder Back(Action a) =>
        Btn(Labels.Back, Pal.Red, a,
            Lay.BackBtn.X, Lay.BackBtn.Y,
            Lay.BackBtn.W, Lay.BackBtn.H);

    public ScreenBuilder Carousel(Action prev, Action next)
    {
        BtnAt(Labels.Prev, Pal.Blue, prev,
            u => new(u.C.X - Lay.Carousel.ArrowX,
                u.C.Y, Lay.Carousel.ArrowW,
                Lay.Carousel.ArrowH));
        BtnAt(Labels.Next, Pal.Blue, next,
            u => new(u.C.X + Lay.Carousel.ArrowOffX,
                u.C.Y, Lay.Carousel.ArrowW,
                Lay.Carousel.ArrowH));
        Key(Keys.Left, prev);
        Key(Keys.Right, next);
        return this;
    }

    public ScreenBuilder CarouselDraw(
        Func<string> name, Func<string> counter,
        Func<string>? desc = null) =>
        Draw((ui, _) =>
        {
            ui.TxtC(name(), ui.C.Y + Lay.Carousel.NameY,
                Color.Yellow, true);
            ui.TxtC(counter(), ui.C.Y + Lay.Carousel.CounterY,
                Color.Gray);
            if (desc is not null)
                ui.TxtC(desc(),
                    ui.C.Y + Lay.Carousel.DescY,
                    Color.LightGray);
        });

    public ScreenBuilder Key(Keys k, Action a)
    {
        _c.KeyMap[k] = a;
        return this;
    }

    public ScreenBuilder Draw(Action<SimpleUI, GamePlay> d)
    {
        _c.Content = d;
        return this;
    }

    public ScreenCfg Build() => _c;

    static class Labels
    {
        public const string
            Back = Strings.Labels.Back,
            Prev = Strings.Labels.Prev,
            Next = Strings.Labels.Next;
    }
}

internal sealed class ScreenMachine
{
    readonly SimpleUI _ui;
    readonly GamePlay _ctrl;
    readonly Game _game;
    readonly GameSettings _cfg;
    readonly Dictionary<Screen, Func<ScreenCfg>> _builders = [];
    readonly ButtonGroup _btns = new();

    Screen _cur = Screen.MainMenu;
    ScreenCfg? _scr;
    int _bikeIdx, _lvlIdx, _themeIdx;

    public ScreenMachine(
        SimpleUI ui, GamePlay ctrl,
        Game game, GameSettings cfg)
    {
        (_ui, _ctrl, _game, _cfg) = (ui, ctrl, game, cfg);
        _themeIdx = ThemeManager.Idx;
        Register();
        Rebuild();
    }

    public void Update(InputState inp)
    {
        _btns.Update(
            inp.MousePos, inp.MouseDown, inp.MouseWasDown);

        if (_scr is not null)
            foreach (KeyValuePair<Keys, Action> kv
                in _scr.KeyMap)
                if (inp.JustPressed(kv.Key))
                {
                    kv.Value();
                    break;
                }

        if (_ctrl.State == GameState.GameOver
            && _cur != Screen.GameOver)
            Go(Screen.GameOver);
        else if (_ctrl.State == GameState.LevelComplete
            && _cur != Screen.LevelComplete)
            Go(Screen.LevelComplete);
    }

    public void Draw()
    {
        if (_scr is null)
            return;
        if (_scr.Overlay > 0)
            _ui.Ovr(_scr.Overlay);
        if (_scr.Title is not null)
            _ui.TxtC(_scr.Title, _scr.TitleY,
                _scr.TitleColor, true);
        if (_scr.Sub is not null)
            _ui.TxtC(_scr.Sub, _scr.SubY, _scr.SubColor);
        _scr.Content?.Invoke(_ui, _ctrl);
        _btns.Draw(_ui);
    }

    void Go(Screen s) { _cur = s; Rebuild(); }

    void Rebuild()
    {
        _scr = _builders[_cur]();
        _btns.Clr();
        foreach ((Func<Rectangle> bounds, string text,
            Color color, Action click) in _scr.Btns)
            _btns.Add(bounds(), text, color, click);
    }

    ScreenBuilder B() => new(_ui);

    void Register()
    {
        _builders[Screen.MainMenu] = BuildMainMenu;
        _builders[Screen.BikeSelect] = BuildBikeSelect;
        _builders[Screen.LevelSelect] = BuildLevelSelect;
        _builders[Screen.ThemeSelect] = BuildThemeSelect;
        _builders[Screen.Gameplay] = BuildGameplay;
        _builders[Screen.Paused] = BuildPaused;
        _builders[Screen.GameOver] = BuildGameOver;
        _builders[Screen.LevelComplete] = BuildLevelComplete;
        _builders[Screen.Settings] = BuildSettings;
    }

    ScreenCfg BuildMainMenu() => B()
        .Title(Strings.Titles.Main, Color.Gold)
        .Sub(Strings.Labels.Sub, Color.White, Lay.Menu.SubY)
        .BtnC(Strings.Labels.Play, Pal.Green,
            () => Go(Screen.BikeSelect), Lay.Menu.Btn0)
        .BtnC(Strings.Labels.Settings, Pal.Blue,
            () => Go(Screen.Settings),
            Lay.Menu.Btn0 + Lay.Menu.Step)
        .BtnC(Strings.Labels.Themes, Pal.Purple,
            () => Go(Screen.ThemeSelect),
            Lay.Menu.Btn0 + Lay.Menu.Step * 2)
        .BtnC(Strings.Labels.Exit, Pal.Red,
            () => Environment.Exit(0),
            Lay.Menu.Btn0 + Lay.Menu.Step * 3)
        .Key(Keys.Enter, () => Go(Screen.BikeSelect))
        .Key(Keys.T, () => Go(Screen.ThemeSelect))
        .Key(Keys.Escape, () => Environment.Exit(0))
        .Build();

    ScreenCfg BuildBikeSelect() => B()
        .Title(Strings.Titles.Bike, Color.Cyan)
        .Back(() => Go(Screen.MainMenu))
        .Carousel(() => CycleBike(-1), () => CycleBike(1))
        .BtnC(Strings.Labels.Select, Pal.Green,
            () => Go(Screen.LevelSelect),
            Lay.Carousel.SelectY)
        .Key(Keys.Enter, () => Go(Screen.LevelSelect))
        .Key(Keys.Escape, () => Go(Screen.MainMenu))
        .CarouselDraw(
            () => BikeDefs.Names[_bikeIdx],
            () => Strings.Counter(
                _bikeIdx, BikeDefs.Count))
        .Build();

    ScreenCfg BuildLevelSelect() => B()
        .Title(Strings.Titles.Level, Color.Magenta)
        .Back(() => Go(Screen.BikeSelect))
        .Carousel(() => StepLvl(-1), () => StepLvl(1))
        .BtnC(Strings.Labels.Start, Pal.Green,
            StartLvl, Lay.Carousel.SelectY)
        .Key(Keys.Enter, StartLvl)
        .Key(Keys.Escape, () => Go(Screen.BikeSelect))
        .CarouselDraw(
            () => Strings.LevelN(_lvlIdx),
            () => Strings.Counter(
                _lvlIdx, _ctrl.Levels.Count))
        .Build();

    ScreenCfg BuildThemeSelect() => B()
        .Title(Strings.Titles.Theme, Pal.Purple)
        .Back(() => Go(Screen.MainMenu))
        .Carousel(
            () => CycleTheme(-1), () => CycleTheme(1))
        .BtnC(Strings.Labels.Apply, Pal.Green,
            ApplyTheme, Lay.Carousel.SelectY)
        .Key(Keys.Enter, ApplyTheme)
        .Key(Keys.Escape, () => Go(Screen.MainMenu))
        .CarouselDraw(
            () => ThemeManager.GetName(_themeIdx),
            () => Strings.Counter(
                _themeIdx, ThemeManager.Cnt),
            () => ThemeManager.GetDesc(_themeIdx))
        .Build();

    ScreenCfg BuildGameplay() => B()
        .BtnAt(Strings.Labels.Pause, Pal.Blue, Pause,
            u => new(u.W - Lay.Hud.PauseX, Lay.Hud.PauseY,
                Lay.Hud.PauseW, Lay.Hud.PauseH))
        .Key(Keys.Escape, Pause)
        .Draw((ui, c) =>
        {
            if (c.Level is not { } lv)
                return;
            Rectangle panel = new(
                Lay.Hud.X, ui.H - Lay.Hud.OffBottom,
                Lay.Hud.W, Lay.Hud.H);
            ui.Panel(panel, Pal.PanelBg, Pal.PanelBrd);
            int tx = panel.X + Lay.Hud.Pad;
            int ty = panel.Y + Lay.Hud.Pad;
            ui.Txt(Strings.LevelLabel(lv.Name),
                new(tx, ty), Color.White);
            ui.Txt(Strings.TimeFmt(c.Time),
                new(tx, ty + Lay.Hud.LineH), Color.LightGreen);
            ui.Txt(Strings.SpeedFmt(c.Bike?.Speed ?? 0),
                new(tx, ty + Lay.Hud.LineH * 2), Color.Cyan);
        })
        .Build();

    ScreenCfg BuildPaused() => B()
        .Ovr()
        .Title(Strings.Titles.Paused, Color.White,
            _ui.C.Y + Lay.Dialog.TitleOff)
        .BtnC(Strings.Labels.Resume, Pal.Green,
            Resume, Lay.Dialog.Btn0)
        .BtnC(Strings.Labels.Restart, Pal.Yellow,
            Restart, Lay.Dialog.Btn0 + Lay.Dialog.Step)
        .BtnC(Strings.Labels.Menu, Pal.Red,
            GoMenu, Lay.Dialog.Btn0 + Lay.Dialog.Step * 2)
        .Key(Keys.Enter, Resume)
        .Key(Keys.Escape, Resume)
        .Key(Keys.R, Restart)
        .Build();

    ScreenCfg BuildGameOver() => B()
        .Ovr()
        .Title(Strings.Titles.Over, Color.Red,
            _ui.C.Y + Lay.Dialog.TitleOff)
        .Sub(Strings.Labels.Crashed, Color.White,
            _ui.C.Y + Lay.Dialog.SubOff)
        .BtnC(Strings.Labels.Restart, Pal.Yellow,
            Restart, Lay.Dialog.Btn1)
        .BtnC(Strings.Labels.Menu, Pal.Red,
            GoMenu, Lay.Dialog.Btn1 + Lay.Dialog.Step)
        .Key(Keys.R, Restart)
        .Key(Keys.Enter, Restart)
        .Key(Keys.Escape, GoMenu)
        .Build();

    ScreenCfg BuildLevelComplete() => B()
        .Ovr()
        .Title(Strings.Titles.Complete, Color.LimeGreen,
            _ui.C.Y + Lay.Dialog.TitleOff)
        .BtnC(Strings.Labels.NextLevel, Pal.Green,
            NextLvl, Lay.Dialog.Btn1)
        .BtnC(Strings.Labels.Restart, Pal.Yellow,
            Restart, Lay.Dialog.Btn1 + Lay.Dialog.Step)
        .BtnC(Strings.Labels.Menu, Pal.Blue,
            GoMenu, Lay.Dialog.Btn1 + Lay.Dialog.Step * 2)
        .Key(Keys.Enter, NextLvl)
        .Key(Keys.R, Restart)
        .Key(Keys.Escape, GoMenu)
        .Draw((ui, c) => ui.TxtC(
            Strings.TimeFmt(c.Time),
            ui.C.Y + Lay.Dialog.SubOff, Color.White))
        .Build();

    ScreenCfg BuildSettings() => B()
        .Title(Strings.Titles.Settings, Color.RoyalBlue,
            Lay.Cfg.TitleY)
        .Back(GoMenu)
        .BtnAt(Strings.Labels.Apply, Pal.Green,
            ApplySettings,
            u => new(u.C.X - Pal.BtnW / 2,
                u.H - Lay.Cfg.ApplyOffBottom,
                Pal.BtnW, Pal.BtnH))
        .Key(Keys.Left, _cfg.PrevRes)
        .Key(Keys.Right, _cfg.NextRes)
        .Key(Keys.Escape, GoMenu)
        .Draw((ui, _) =>
        {
            ui.TxtC(Strings.Labels.ResLabel,
                ui.C.Y + Lay.Cfg.LabelY, Color.White);
            ui.TxtC(Strings.ResFmt(_cfg.ResStr),
                ui.C.Y + Lay.Cfg.ValueY, Color.Yellow);
            ui.TxtC(Strings.Labels.ResHint,
                ui.H - Lay.Cfg.HintOff, Color.Gray);
        })
        .Build();

    void GoMenu()
    {
        _ctrl.ResetState();
        Go(Screen.MainMenu);
    }

    void CycleBike(int d)
    {
        _bikeIdx = (_bikeIdx + d + BikeDefs.Count)
            % BikeDefs.Count;
        _ctrl.SetBike(BikeDefs.Types[_bikeIdx]);
    }

    void StepLvl(int d) =>
        _lvlIdx = Math.Clamp(
            _lvlIdx + d, 0, _ctrl.Levels.Count - 1);

    void CycleTheme(int d) =>
        _themeIdx = (_themeIdx + d + ThemeManager.Cnt)
            % ThemeManager.Cnt;

    void ApplyTheme()
    {
        ThemeManager.Set(_themeIdx);
        ThemeManager.SaveToSettings();
    }

    void StartLvl()
    {
        _ctrl.StartLevel(_lvlIdx + 1);
        Go(Screen.Gameplay);
    }

    void Pause()
    {
        _ctrl.Pause();
        Go(Screen.Paused);
    }

    void Resume()
    {
        _ctrl.Resume();
        Go(Screen.Gameplay);
    }

    void Restart()
    {
        if (_ctrl.Level is not { } lv)
            return;
        _ctrl.StartLevel(lv.Id);
        Go(Screen.Gameplay);
    }

    void NextLvl()
    {
        if (_ctrl.Level is not { } lv
            || lv.Id >= _ctrl.Levels.Count)
        {
            GoMenu();
            return;
        }
        _lvlIdx = lv.Id;
        _ctrl.StartLevel(lv.Id + 1);
        Go(Screen.Gameplay);
    }

    void ApplySettings()
    {
        _game.SetResolution(_cfg.W, _cfg.H);
        _cfg.Save();
    }
}
