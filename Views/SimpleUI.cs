using MColor = Microsoft.Xna.Framework.Color;
using Thickness = Myra.Graphics2D.Thickness;

namespace GravityDefiedGame.Views;

public sealed class SimpleUI : IDisposable
{
    readonly FontSystem _fs;

    public SimpleUI(string fontPath, int sm = 18, int lg = 28)
    {
        _fs = new FontSystem();
        _fs.AddFont(File.ReadAllBytes(fontPath));
        UI.Init(_fs.GetFont(sm), _fs.GetFont(lg));
    }

    public void Dispose() => _fs.Dispose();
}

public static class C
{
    public static readonly MColor
        Gray = MColor.LightGray,
        Yellow = MColor.Yellow,
        Bg = new(30, 30, 30, 240),
        Border = new(140, 140, 140),
        DimBg = MColor.Black * 0.6f;
}

public static class UI
{
    public static Desktop Desktop { get; } = new();
    public static SpriteFontBase Font { get; private set; } = null!;
    public static SpriteFontBase FontLg { get; private set; } = null!;

    public static void Init(SpriteFontBase sm, SpriteFontBase lg)
    {
        Font = sm;
        FontLg = lg;
    }

    public static Label Title(this string s) =>
        new Label { Text = s, Font = FontLg, TextColor = MColor.White }.Center();

    public static Label Text(this string s) =>
        new Label { Text = s, Font = Font, TextColor = MColor.White }.Center();

    public static Button Btn(this string s) =>
        new Button { Content = s.Text() }.W(180).Center();

    public static Panel Gap(int h) => new() { Height = h };

    public static VerticalStackPanel Col(int sp = 12) =>
        new VerticalStackPanel { Spacing = sp }.Center().CenterV();

    public static Grid Grd(int cols, int sp = 8)
    {
        Grid g = new() { ColumnSpacing = sp };
        for (int i = 0; i < cols; i++)
            g.ColumnsProportions.Add(new(ProportionType.Part, 1));
        return g;
    }

    public static HorizontalStackPanel NavRow(Action<int> nav) =>
        new HorizontalStackPanel { Spacing = 8 }
            .Center()
            .Add("<".Btn().W(50).On(() => nav(-1)))
            .Add(">".Btn().W(50).On(() => nav(1)));

    public static Panel Pick(
        string title, string value, string counter, string? desc,
        Action<int> nav, Action ok, Action back) =>
        Col()
            .Add(title.Title())
            .Add(Gap(10))
            .Add(NavRow(nav))
            .Add(value.Text().Yellow())
            .Add(counter.Text())
            .AddIf(desc, d => d.Text().Gray())
            .Add(Gap(15))
            .Add("OK".Btn().On(ok))
            .Add("BACK".Btn().On(back))
            .Frame();

    public static Panel Menu(
        string title, string? sub, params (string Label, Action Click)[] btns) =>
        Dialog(title, sub, C.Gray, false, btns);

    public static Panel Overlay(
        string title, string? sub, params (string Label, Action Click)[] btns) =>
        Dialog(title, sub, C.Yellow, true, btns);

    static Panel Dialog(
        string title, string? sub, MColor subColor, bool dim,
        (string Label, Action Click)[] btns)
    {
        VerticalStackPanel col = Col()
            .Add(title.Title())
            .AddIf(sub, s => s.Text().Color(subColor))
            .Add(Gap(dim ? 15 : 20));

        foreach ((string label, Action click) in btns)
            col.Add(label.Btn().On(click));

        return dim ? col.Frame().Dim() : col.Frame();
    }

    public static T Center<T>(this T w) where T : Widget
    { w.HorizontalAlignment = HorizontalAlignment.Center; return w; }

    public static T Left<T>(this T w) where T : Widget
    { w.HorizontalAlignment = HorizontalAlignment.Left; return w; }

    public static T Right<T>(this T w) where T : Widget
    { w.HorizontalAlignment = HorizontalAlignment.Right; return w; }

    public static T CenterV<T>(this T w) where T : Widget
    { w.VerticalAlignment = VerticalAlignment.Center; return w; }

    public static T Bottom<T>(this T w) where T : Widget
    { w.VerticalAlignment = VerticalAlignment.Bottom; return w; }

    public static T Stretch<T>(this T w) where T : Widget
    { w.HorizontalAlignment = HorizontalAlignment.Stretch; return w; }

    public static T StretchV<T>(this T w) where T : Widget
    { w.VerticalAlignment = VerticalAlignment.Stretch; return w; }

    public static T W<T>(this T w, int width) where T : Widget
    { w.Width = width; return w; }

    public static Label Gray(this Label l)
    { l.TextColor = C.Gray; return l; }

    public static Label Yellow(this Label l)
    { l.TextColor = C.Yellow; return l; }

    public static Label Color(this Label l, MColor c)
    { l.TextColor = c; return l; }

    public static T Bg<T>(this T w, MColor c) where T : Widget
    { w.Background = new SolidBrush(c); return w; }

    public static T Pad<T>(this T w, int h, int v) where T : Widget
    { w.Padding = new Thickness(h, v); return w; }

    public static T Mar<T>(this T w, int l, int t, int r, int b) where T : Widget
    { w.Margin = new Thickness(l, t, r, b); return w; }

    public static T Add<T>(this T c, Widget w) where T : Container
    { c.Widgets.Add(w); return c; }

    public static T AddIf<T>(this T c, string? val, Func<string, Widget> f) where T : Container
    { if (val != null) c.Widgets.Add(f(val)); return c; }

    public static Grid Cell(this Grid g, Widget w, int col)
    { Grid.SetColumn(w, col); g.Widgets.Add(w); return g; }

    public static Button On(this Button b, Action click)
    { b.Click += (_, _) => click(); return b; }

    public static void Set(this Label l, string s) => l.Text = s;

    public static Panel Frame(this Widget w) => w.Frame(C.Bg, C.Border);

    public static Panel Frame(this Widget w, MColor bg, MColor border)
    {
        Panel p = new Panel().Bg(bg).Pad(40, 30).Center().CenterV();
        p.Border = new SolidBrush(border);
        p.BorderThickness = new Thickness(2);
        p.Widgets.Add(w);
        return p;
    }

    public static Panel Dim(this Widget w) =>
        new Panel().Bg(C.DimBg).Stretch().StretchV().Add(w);
}
