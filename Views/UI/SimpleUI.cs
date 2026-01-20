
namespace GravityDefiedGame.Views.UI;

public sealed class SimpleUI(
    SpriteBatch sb,
    DynamicSpriteFont font,
    DynamicSpriteFont titleFont,
    int w,
    int h)
{
    public int W { get; } = w;
    public int H { get; } = h;
    public Point C => new(W / 2, H / 2);

    public void Txt(string s, Vector2 p, Color c, bool title = false) =>
        (title ? titleFont : font).DrawText(sb, s, p, c);

    public void TxtC(string s, int y, Color c, bool title = false)
    {
        // center text horizontally
        // measure the string width and place it at (W - width) / 2
        DynamicSpriteFont f = title ? titleFont : font;
        Vector2 sz = f.MeasureString(s);
        f.DrawText(sb, s, new((W - sz.X) * 0.5f, y), c);
    }

    public void Fill(Rectangle r, Color c) => sb.FillRectangle(r, c);

    public void Border(Rectangle r, Color c, int t = 2)
    {
        // draw border as 4 filled rectangles:
        // cheaper and simpler than per-pixel or line primitives for UI
        Fill(new(r.X, r.Y, r.Width, t), c);
        Fill(new(r.X, r.Y + r.Height - t, r.Width, t), c);
        Fill(new(r.X, r.Y, t, r.Height), c);
        Fill(new(r.X + r.Width - t, r.Y, t, r.Height), c);
    }

    public void Panel(Rectangle r, Color bg, Color brd)
    {
        Fill(r, bg);
        Border(r, brd, 2);
    }

    public void Btn(Button b)
    {
        if (!b.Vis)
            return;

        // simple button state shading:
        // pressed -> darker, hovered -> brighter, else base color
        Color bg = b.Prs ? b.Col * 0.7f
            : b.Hov ? b.Col * 1.2f
            : b.Col;

        Panel(b.R, bg, Color.White);

        // center label inside button rect
        Vector2 sz = font.MeasureString(b.Txt);
        Txt(
            b.Txt,
            new(
                b.R.X + (b.R.Width - sz.X) * 0.5f,
                b.R.Y + (b.R.Height - sz.Y) * 0.5f),
            Color.White);
    }

    public void Ovr(float a) =>
        Fill(new(0, 0, W, H), Color.Black * a);
}

public sealed class Button(Rectangle r, string txt, Color col, Action? clk = null)
{
    public Rectangle R { get; } = r;
    public string Txt { get; set; } = txt;
    public Color Col { get; } = col;
    public Action? Clk { get; } = clk;

    public bool Vis { get; set; } = true;
    public bool Hov { get; set; }
    public bool Prs { get; set; }
}

public sealed class ButtonGroup
{
    private readonly List<Button> _list = [];
    private Action? _pending;

    public void Clr() => _list.Clear();

    public Button Add(Rectangle r, string txt, Color col, Action? clk = null)
    {
        var b = new Button(r, txt, col, clk);
        _list.Add(b);
        return b;
    }

    public void Update(Point m, bool dn, bool was)
    {
        // defer click invocation to avoid - сollection was modified - issues
        // click handlers may change screens -> rebuild UI -> clear/add buttons
        // so we run the action on the next Update() call (next frame)
        if (_pending != null)
        {
            Action act = _pending;
            _pending = null;
            act();
            return;
        }

        bool justDn = dn && !was;
        bool justUp = !dn && was;

        foreach (Button b in _list)
        {
            if (!b.Vis)
            {
                b.Prs = false;
                continue;
            }

            b.Hov = b.R.Contains(m);

            if (!b.Hov)
            {
                b.Prs = false;
                continue;
            }

            // press starts on mouse-down while hovered
            if (justDn)
            {
                b.Prs = true;
                continue;
            }

            // click fires on mouse-up if the button was pressed and still hovered
            if (justUp && b.Prs)
            {
                b.Prs = false;
                _pending = b.Clk;
            }
        }
    }

    public void Draw(SimpleUI ui)
    {
        foreach (Button b in _list)
            ui.Btn(b);
    }
}

public static class UI
{
    public const int BtnW = 200;
    public const int BtnH = 50;

    public static readonly Color Green = new(50, 120, 50);
    public static readonly Color Blue = new(50, 80, 150);
    public static readonly Color Red = new(150, 50, 50);
    public static readonly Color Yellow = new(150, 120, 30);
    public static readonly Color Purple = new(180, 100, 220);

    public static readonly Color PanelBg = new(20, 20, 40, 200);
    public static readonly Color PanelBrd = new(100, 100, 150);
}
