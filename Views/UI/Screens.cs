using Game = GravityDefiedGame.Core.Game;

namespace GravityDefiedGame.Views.UI;

public enum Screen
{
    MainMenu, BikeSelect, LevelSelect, ThemeSelect,
    Gameplay, Paused, GameOver, LevelComplete, Settings
}

internal sealed class ScreenCfg
{
    public string? Title;
    public Color TitleColor = Color.White;
    public int TitleY = 80;
    public string? Sub;
    public Color SubColor = Color.White;
    public int SubY;
    public float Overlay;
    public List<(Func<Rectangle> Bounds, string Text, Color Color, Action Click)> Btns = [];
    public Dictionary<Keys, Action> KeyMap = [];
    public Action<SimpleUI, GamePlay>? Content;
}

internal sealed class ScreenBuilder(SimpleUI ui)
{
    readonly ScreenCfg _c = new();

    public ScreenBuilder Title(string t, Color c, int? y = null)
    {
        (_c.Title, _c.TitleColor) = (t, c);
        if (y.HasValue)
            _c.TitleY = y.Value;
        return this;
    }

    public ScreenBuilder Sub(string t, Color? c = null, int? y = null)
    {
        (_c.Sub, _c.SubColor, _c.SubY) = (t, c ?? Color.White, y ?? _c.TitleY + 80);
        return this;
    }

    public ScreenBuilder Ovr(float a = 0.5f) { _c.Overlay = a; return this; }

    public ScreenBuilder BtnC(string t, Color c, Action a, int yOff = 0)
    {
        _c.Btns.Add((() => new(ui.C.X - UI.BtnW / 2, ui.C.Y + yOff, UI.BtnW, UI.BtnH), t, c, a));
        return this;
    }

    public ScreenBuilder Btn(string t, Color c, Action a, int x, int y, int w, int h)
    {
        _c.Btns.Add((() => new(x, y, w, h), t, c, a));
        return this;
    }

    public ScreenBuilder BtnAt(string t, Color c, Action a, Func<SimpleUI, Rectangle> b)
    {
        _c.Btns.Add((() => b(ui), t, c, a));
        return this;
    }

    public ScreenBuilder Key(Keys k, Action a) { _c.KeyMap[k] = a; return this; }

    public ScreenBuilder Draw(Action<SimpleUI, GamePlay> d) { _c.Content = d; return this; }

    public ScreenCfg Build() => _c;
}

internal sealed class ScreenMachine
{
    static readonly string[] BikeNames = ["Standard", "Sport", "OffRoad"];

    readonly SimpleUI _ui;
    readonly GamePlay _ctrl;
    readonly Game _game;
    readonly GameSettings _cfg;
    readonly Dictionary<Screen, Func<ScreenCfg>> _builders = [];
    readonly ButtonGroup _btns = new();

    Screen _cur = Screen.MainMenu;
    ScreenCfg? _scr;
    int _bikeIdx, _lvlIdx, _themeIdx;

    public ScreenMachine(SimpleUI ui, GamePlay ctrl, Game game, GameSettings cfg)
    {
        (_ui, _ctrl, _game, _cfg) = (ui, ctrl, game, cfg);
        _themeIdx = ThemeManager.Idx;
        Register();
        Rebuild();
    }

    public void Update(InputState inp)
    {
        _btns.Update(inp.MousePos, inp.MouseDown, inp.MouseWasDown);

        if (_scr != null)
            foreach (KeyValuePair<Keys, Action> kv in _scr.KeyMap)
                if (inp.JustPressed(kv.Key))
                {
                    kv.Value();
                    break;
                }

        if (_ctrl.State == GameState.GameOver && _cur != Screen.GameOver)
            Go(Screen.GameOver);
        else if (_ctrl.State == GameState.LevelComplete && _cur != Screen.LevelComplete)
            Go(Screen.LevelComplete);
    }

    public void Draw()
    {
        if (_scr == null)
            return;

        if (_scr.Overlay > 0)
            _ui.Ovr(_scr.Overlay);
        if (_scr.Title != null)
            _ui.TxtC(_scr.Title, _scr.TitleY, _scr.TitleColor, true);
        if (_scr.Sub != null)
            _ui.TxtC(_scr.Sub, _scr.SubY, _scr.SubColor);

        _scr.Content?.Invoke(_ui, _ctrl);
        _btns.Draw(_ui);
    }

    void Go(Screen s) { _cur = s; Rebuild(); }

    void Rebuild()
    {
        _scr = _builders[_cur]();
        _btns.Clr();
        foreach ((Func<Rectangle> bounds, string text, Color color, Action click) in _scr.Btns)
            _btns.Add(bounds(), text, color, click);
    }

    ScreenBuilder B() => new(_ui);

    void Register()
    {
        _builders[Screen.MainMenu] = () => B()
            .Title("GRAVITY DEFIED", Color.Gold)
            .Sub("Bike Physics Game", Color.White, 160)
            .BtnC("Play", UI.Green, () => Go(Screen.BikeSelect), -80)
            .BtnC("Settings", UI.Blue, () => Go(Screen.Settings), -20)
            .BtnC("Themes", UI.Purple, () => Go(Screen.ThemeSelect), 40)
            .BtnC("Exit", UI.Red, () => Environment.Exit(0), 100)
            .Key(Keys.Enter, () => Go(Screen.BikeSelect))
            .Key(Keys.T, () => Go(Screen.ThemeSelect))
            .Key(Keys.Escape, () => Environment.Exit(0))
            .Build();

        _builders[Screen.BikeSelect] = () => B()
            .Title("SELECT BIKE", Color.Cyan)
            .Btn("Back", UI.Red, () => Go(Screen.MainMenu), 20, 20, 120, 40)
            .BtnAt("←", UI.Blue, () => CycleBike(-1), u => new(u.C.X - 300, u.C.Y, 80, 50))
            .BtnAt("→", UI.Blue, () => CycleBike(1), u => new(u.C.X + 220, u.C.Y, 80, 50))
            .BtnC("Select", UI.Green, () => Go(Screen.LevelSelect), 100)
            .Key(Keys.Left, () => CycleBike(-1))
            .Key(Keys.Right, () => CycleBike(1))
            .Key(Keys.Enter, () => Go(Screen.LevelSelect))
            .Key(Keys.Escape, () => Go(Screen.MainMenu))
            .Draw((ui, _) =>
            {
                ui.TxtC(BikeNames[_bikeIdx], ui.C.Y - 30, Color.Yellow, true);
                ui.TxtC($"{_bikeIdx + 1}/{BikeNames.Length}", ui.C.Y + 20, Color.Gray);
            })
            .Build();

        _builders[Screen.LevelSelect] = () => B()
            .Title("SELECT LEVEL", Color.Magenta)
            .Btn("Back", UI.Red, () => Go(Screen.BikeSelect), 20, 20, 120, 40)
            .BtnAt("←", UI.Blue, () => StepLvl(-1), u => new(u.C.X - 300, u.C.Y, 80, 50))
            .BtnAt("→", UI.Blue, () => StepLvl(1), u => new(u.C.X + 220, u.C.Y, 80, 50))
            .BtnC("Start", UI.Green, StartLvl, 100)
            .Key(Keys.Left, () => StepLvl(-1))
            .Key(Keys.Right, () => StepLvl(1))
            .Key(Keys.Enter, StartLvl)
            .Key(Keys.Escape, () => Go(Screen.BikeSelect))
            .Draw((ui, c) =>
            {
                ui.TxtC($"Level {_lvlIdx + 1}", ui.C.Y - 30, Color.Yellow, true);
                ui.TxtC($"{_lvlIdx + 1}/{c.Levels.Count}", ui.C.Y + 20, Color.Gray);
            })
            .Build();

        _builders[Screen.ThemeSelect] = () => B()
            .Title("SELECT THEME", UI.Purple)
            .Btn("Back", UI.Red, () => Go(Screen.MainMenu), 20, 20, 120, 40)
            .BtnAt("←", UI.Blue, () => CycleTheme(-1), u => new(u.C.X - 300, u.C.Y, 80, 50))
            .BtnAt("→", UI.Blue, () => CycleTheme(1), u => new(u.C.X + 220, u.C.Y, 80, 50))
            .BtnC("Apply", UI.Green, ApplyTheme, 100)
            .Key(Keys.Left, () => CycleTheme(-1))
            .Key(Keys.Right, () => CycleTheme(1))
            .Key(Keys.Enter, ApplyTheme)
            .Key(Keys.Escape, () => Go(Screen.MainMenu))
            .Draw((ui, _) =>
            {
                ui.TxtC(ThemeManager.GetName(_themeIdx), ui.C.Y - 30, Color.Yellow, true);
                ui.TxtC($"{_themeIdx + 1}/{ThemeManager.Cnt}", ui.C.Y + 20, Color.Gray);
                ui.TxtC(ThemeManager.GetDesc(_themeIdx), ui.C.Y + 60, Color.LightGray);
            })
            .Build();

        _builders[Screen.Gameplay] = () => B()
            .BtnAt("Pause", UI.Blue, Pause, u => new(u.W - 140, 20, 120, 40))
            .Key(Keys.Escape, Pause)
            .Draw((ui, c) =>
            {
                if (c.Level is not { } lv)
                    return;
                ui.Panel(new(20, ui.H - 120, 200, 100), UI.PanelBg, UI.PanelBrd);
                ui.Txt($"Level: {lv.Name}", new(30, ui.H - 110), Color.White);
                ui.Txt($"Time: {c.Time:mm\\:ss}", new(30, ui.H - 80), Color.LightGreen);
                ui.Txt($"Speed: {c.Bike?.Spd ?? 0:F1}", new(30, ui.H - 50), Color.Cyan);
            })
            .Build();

        _builders[Screen.Paused] = () => B()
            .Ovr()
            .Title("PAUSED", Color.White, _ui.C.Y - 120)
            .BtnC("Resume", UI.Green, Resume, 0)
            .BtnC("Restart", UI.Yellow, Restart, 60)
            .BtnC("Menu", UI.Red, GoMenu, 120)
            .Key(Keys.Enter, Resume)
            .Key(Keys.Escape, Resume)
            .Key(Keys.R, Restart)
            .Build();

        _builders[Screen.GameOver] = () => B()
            .Ovr()
            .Title("GAME OVER", Color.Red, _ui.C.Y - 120)
            .Sub("You crashed!", Color.White, _ui.C.Y - 50)
            .BtnC("Restart", UI.Yellow, Restart, 20)
            .BtnC("Menu", UI.Red, GoMenu, 80)
            .Key(Keys.R, Restart)
            .Key(Keys.Enter, Restart)
            .Key(Keys.Escape, GoMenu)
            .Build();

        _builders[Screen.LevelComplete] = () => B()
            .Ovr()
            .Title("LEVEL COMPLETE!", Color.LimeGreen, _ui.C.Y - 120)
            .BtnC("Next Level", UI.Green, NextLvl, 20)
            .BtnC("Restart", UI.Yellow, Restart, 80)
            .BtnC("Menu", UI.Blue, GoMenu, 140)
            .Key(Keys.Enter, NextLvl)
            .Key(Keys.R, Restart)
            .Key(Keys.Escape, GoMenu)
            .Draw((ui, c) => ui.TxtC($"Time: {c.Time:mm\\:ss}", ui.C.Y - 50, Color.White))
            .Build();

        _builders[Screen.Settings] = () => B()
            .Title("SETTINGS", Color.RoyalBlue, 60)
            .Btn("Back", UI.Red, GoMenu, 20, 20, 120, 40)
            .BtnAt("Apply", UI.Green, ApplySettings, u => new(u.C.X - UI.BtnW / 2, u.H - 80, UI.BtnW, UI.BtnH))
            .Key(Keys.Left, _cfg.PrevRes)
            .Key(Keys.Right, _cfg.NextRes)
            .Key(Keys.Escape, GoMenu)
            .Draw((ui, _) =>
            {
                ui.TxtC("Resolution:", ui.C.Y, Color.White);
                ui.TxtC($"← {_cfg.ResStr} →", ui.C.Y + 30, Color.Yellow);
                ui.TxtC("←/→ change, Apply to confirm", ui.H - 130, Color.Gray);
            })
            .Build();
    }

    void GoMenu() { _ctrl.ResetState(); Go(Screen.MainMenu); }
    void CycleBike(int d) { _bikeIdx = (_bikeIdx + d + BikeNames.Length) % BikeNames.Length; _ctrl.SetBike(_ctrl.AvailBikes[_bikeIdx]); }
    void StepLvl(int d) => _lvlIdx = Math.Clamp(_lvlIdx + d, 0, _ctrl.Levels.Count - 1);
    void CycleTheme(int d) => _themeIdx = (_themeIdx + d + ThemeManager.Cnt) % ThemeManager.Cnt;
    void ApplyTheme() { ThemeManager.Set(_themeIdx); ThemeManager.SaveToSettings(); }
    void StartLvl() { _ctrl.StartLevel(_lvlIdx + 1); Go(Screen.Gameplay); }
    void Pause() { _ctrl.Pause(); Go(Screen.Paused); }
    void Resume() { _ctrl.Resume(); Go(Screen.Gameplay); }

    void Restart()
    {
        if (_ctrl.Level is not { } lv)
            return;
        _ctrl.StartLevel(lv.Id);
        Go(Screen.Gameplay);
    }

    void NextLvl()
    {
        if (_ctrl.Level is not { } lv || lv.Id >= _ctrl.Levels.Count)
        { GoMenu(); return; }
        _lvlIdx = lv.Id;
        _ctrl.StartLevel(lv.Id + 1);
        Go(Screen.Gameplay);
    }

    void ApplySettings() { _game.SetResolution(_cfg.W, _cfg.H); _cfg.Save(); }
}
