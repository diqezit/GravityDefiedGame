using static System.Array;
using static System.Globalization.NumberStyles;
using static System.IO.File;
using static System.Math;
using static System.Text.Json.JsonSerializer;
using static System.Text.Json.JsonSerializerDefaults;

namespace GravityDefiedGame.Core;

public enum LineType : byte { Frame, Wheel, Shock, Fork, Rider, Leg, Body, Fist }

public readonly record struct PickerInfo(
    string Title, string Value, string Counter, string? Desc = null);

public sealed record WorldData(List<ThemeJson>? Themes, RenderCfg? Rendering);

public sealed record ThemeJson
{
    public string? Name { get; init; }
    public string? Description { get; init; }
    public string? Background { get; init; }
    public string? Terrain { get; init; }
    public string? VerticalLine { get; init; }
    public BikeJson? Bike { get; init; }
    public SkyJson? Sky { get; init; }
}

public sealed record BikeJson
{
    public string? Frame { get; init; }
    public string? Wheel { get; init; }
    public string? Shock { get; init; }
    public string? Fork { get; init; }
    public string? Rider { get; init; }
    public string? Leg { get; init; }
    public string? Body { get; init; }
    public string? Fist { get; init; }

    public static readonly BikeJson Empty = new();
}

public sealed record SkyJson
{
    public string[]? Palette { get; init; }
    public bool HasStars { get; init; }
    public bool HasSun { get; init; }

    public static readonly SkyJson Empty = new() { Palette = [] };
}

public sealed record RenderCfg
{
    public float TerrainStroke { get; init; } = 3f;
    public float VerticalStroke { get; init; } = 0.5f;
    public int FillPoints { get; init; } = 20;
    public int StarCount { get; init; } = 200;
    public int GradientSteps { get; init; } = 30;
}

public readonly record struct ThemeRenderCfg(string[] SkyPal, bool HasStars, bool HasSun);

public sealed record ThemeSettings(
    string Name,
    string Desc,
    Color BgColor,
    Color TerrainColor,
    Color VLineColor,
    Dictionary<LineType, Color> BikeColors,
    ThemeRenderCfg RenderCfg);

public static class Clr
{
    static readonly Color Bad = Color.Magenta;

    public static Color Hex(string? s)
    {
        if (string.IsNullOrWhiteSpace(s))
            return Bad;

        ReadOnlySpan<char> h = s.AsSpan();
        if (h[0] == '#')
            h = h[1..];
        if (h.Length < 6)
            return Bad;

        if (!TryHex(h, 0, out byte r) ||
            !TryHex(h, 2, out byte g) ||
            !TryHex(h, 4, out byte b))
            return Bad;

        byte a = h.Length >= 8 && TryHex(h, 6, out byte av) ? av : (byte)255;
        return new(r, g, b, a);
    }

    public static Color HexOr(string? s, Color def) =>
        string.IsNullOrWhiteSpace(s) ? def : Hex(s);

    public static Color[] ParsePal(string[] arr) =>
        ConvertAll(arr, Hex);

    static bool TryHex(ReadOnlySpan<char> h, int i, out byte v) =>
        byte.TryParse(h.Slice(i, 2), HexNumber, null, out v);
}

public static class ThemeManager
{
    const string WorldPath = "Content/Data/World.json";

    static readonly System.Text.Json.JsonSerializerOptions Opt = new(Web);
    static readonly List<ThemeSettings> _themes = [];

    static readonly Color
        BgDef = new(0, 0, 0),
        TerrainDef = new(128, 128, 128),
        VLineDef = new(64, 64, 64),

        FrameDef = new(128, 128, 128),
        WheelDef = new(64, 64, 64),
        ShockDef = new(96, 96, 96),
        ForkDef = new(96, 96, 96),

        RiderDef = new(0, 0, 0),
        LegDef = new(0, 0, 0),
        BodyDef = new(0, 0, 128),
        FistDef = new(156, 0, 0);

    static readonly (LineType Type, Func<BikeJson, string?> Pick, Color Def)[] BikeColorMap =
    [
        (LineType.Frame, b => b.Frame, FrameDef),
        (LineType.Wheel, b => b.Wheel, WheelDef),
        (LineType.Shock, b => b.Shock, ShockDef),
        (LineType.Fork, b => b.Fork, ForkDef),
        (LineType.Rider, b => b.Rider, RiderDef),
        (LineType.Leg, b => b.Leg, LegDef),
        (LineType.Body, b => b.Body, BodyDef),
        (LineType.Fist, b => b.Fist, FistDef),
    ];

    public static RenderCfg Rendering { get; private set; } = new();
    public static ThemeSettings Cur => _themes[Idx];
    public static int Idx { get; private set; }
    public static int Cnt => _themes.Count;

    public static event Action? Changed;

    static ThemeManager() => Load(WorldPath);

    public static PickerInfo ThemePicker(int i) =>
        new("SELECT THEME", ThemeName(i).ToUpper(), Fmt(i, Cnt), ThemeDesc(i));

    public static string ThemeName(int i) =>
        (uint)i < (uint)Cnt ? _themes[i].Name : "—";

    public static string ThemeDesc(int i) =>
        (uint)i < (uint)Cnt ? _themes[i].Desc : "";

    public static string Fmt(int i, int n) => n > 0 ? $"{i + 1}/{n}" : "—";

    public static void Set(int i)
    {
        int next = Cnt <= 0 ? 0 : Clamp(i, 0, Cnt - 1);
        if (next == Idx)
            return;
        Idx = next;
        Changed?.Invoke();
    }

    public static int NextTheme(int current, int dir = 1)
    {
        int next = Step(current, Cnt, dir, wrap: true);
        Set(next);
        return next;
    }

    public static int Step(int cur, int cnt, int dir, bool wrap) =>
        cnt <= 0 ? 0 :
        wrap ? (cur + dir + cnt) % cnt :
        Clamp(cur + dir, 0, cnt - 1);

    static void Load(string path)
    {
        WorldData data = Deserialize<WorldData>(ReadAllText(path), Opt)
            ?? throw new InvalidDataException("World.json broken");

        List<ThemeJson> src = data.Themes ?? [];
        if (src.Count == 0)
            throw new InvalidDataException("World.json: no themes");

        Rendering = data.Rendering ?? new();

        _themes.Clear();
        foreach (ThemeJson t in src)
            _themes.Add(ToTheme(t));

        Idx = 0;
        Changed?.Invoke();
    }

    static ThemeSettings ToTheme(ThemeJson t)
    {
        BikeJson bike = t.Bike ?? BikeJson.Empty;
        SkyJson sky = t.Sky ?? SkyJson.Empty;

        return new(
            t.Name ?? "Unnamed",
            t.Description ?? "",
            Clr.HexOr(t.Background, BgDef),
            Clr.HexOr(t.Terrain, TerrainDef),
            Clr.HexOr(t.VerticalLine, VLineDef),
            BuildBikeColors(bike),
            new(sky.Palette ?? [], sky.HasStars, sky.HasSun));
    }

    static Dictionary<LineType, Color> BuildBikeColors(BikeJson bike)
    {
        var colors = new Dictionary<LineType, Color>(BikeColorMap.Length);
        foreach (var x in BikeColorMap)
            colors[x.Type] = Clr.HexOr(x.Pick(bike), x.Def);
        return colors;
    }
}
