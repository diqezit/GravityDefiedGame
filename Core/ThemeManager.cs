namespace GravityDefiedGame.Core;

public enum LineType : byte
{
    Frame, Wheel, Shock, Fork,
    Rider, Leg, Body, Fist
}

public readonly struct BikeDefaults
{
    public const string
        Frame = "#808080", Wheel = "#404040",
        Shock = "#606060", Fork = "#606060",
        Rider = "#000000", Leg = "#000000",
        Body = "#000080", Fist = "#9C0000";

    public const int MinHexLen = 6;

    public static readonly Color Fallback = Color.Magenta;

    static readonly (LineType Type,
        Func<BikeJson, string?> Get, string Def)[] Map =
    [
        (LineType.Frame, b => b.Frame, Frame),
        (LineType.Wheel, b => b.Wheel, Wheel),
        (LineType.Shock, b => b.Shock, Shock),
        (LineType.Fork,  b => b.Fork,  Fork),
        (LineType.Rider, b => b.Rider, Rider),
        (LineType.Leg,   b => b.Leg,   Leg),
        (LineType.Body,  b => b.Body,  Body),
        (LineType.Fist,  b => b.Fist,  Fist),
    ];

    public static Dictionary<LineType, Color> Parse(BikeJson b)
    {
        Dictionary<LineType, Color> d = new(Map.Length);
        foreach ((LineType type,
            Func<BikeJson, string?> get, string def) in Map)
            d[type] = ThemeState.Hex(get(b) ?? def);
        return d;
    }
}

public sealed record WorldData(
    List<ThemeJson>? Themes, RenderCfg? Rendering);

public sealed record ThemeJson
{
    public string? Name { get; init; }
    public string? Description { get; init; }
    public ColorsJson? Colors { get; init; }
    public BikeJson? Bike { get; init; }
    public SkyJson? Sky { get; init; }
}

public sealed record ColorsJson
{
    public string? Background { get; init; }
    public string? Terrain { get; init; }
    public string? VerticalLine { get; init; }

    public static readonly ColorsJson Empty = new()
    {
        Background = "#000000",
        Terrain = "#808080",
        VerticalLine = "#404040"
    };
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

    public static readonly SkyJson Empty = new()
    {
        Palette = []
    };
}

public sealed record RenderCfg
{
    public float TerrainStroke { get; init; } = 3f;
    public float VerticalStroke { get; init; } = 0.5f;
    public int FillPoints { get; init; } = 20;
    public int StarCount { get; init; } = 200;
    public int GradientSteps { get; init; } = 30;
}

public readonly record struct ThemeRenderCfg(
    string[] SkyPal, bool HasStars, bool HasSun);

public sealed record ThemeSettings(
    string Name, string Desc,
    Color BgColor, Color TerrainColor, Color VLineColor,
    Dictionary<LineType, Color> BikeColors,
    ThemeRenderCfg RenderCfg);

public sealed class ThemeState
{
    static readonly JsonSerializerOptions Opt = new(
        JsonSerializerDefaults.Web);

    readonly List<ThemeSettings> _themes = [];

    public ThemeSettings Cur => _themes[Idx];
    public int Idx { get; private set; }
    public int Cnt => _themes.Count;

    public event Action? Changed;

    public void Load(string path)
    {
        WorldData data = JsonSerializer.Deserialize<WorldData>(
            File.ReadAllText(path), Opt)
            ?? throw new InvalidDataException(
                "World.json broken");

        ThemeManager.Rendering = data.Rendering ?? new();

        List<ThemeJson> themes = data.Themes ?? [];
        if (themes.Count == 0)
            throw new InvalidDataException(
                "World.json: no themes");

        _themes.Clear();
        _themes.AddRange(themes.Select(ToTheme));

        Idx = 0;
        Changed?.Invoke();
    }

    public void Set(int i)
    {
        int next = Math.Clamp(i, 0, _themes.Count - 1);
        if (next == Idx)
            return;
        Idx = next;
        Changed?.Invoke();
    }

    public void Next() =>
        Set((Idx + 1) % _themes.Count);

    public void Prev() =>
        Set((Idx - 1 + _themes.Count) % _themes.Count);

    public string GetName(int i) => _themes[i].Name;
    public string GetDesc(int i) => _themes[i].Desc;

    static ThemeSettings ToTheme(ThemeJson t)
    {
        ColorsJson c = t.Colors ?? ColorsJson.Empty;
        BikeJson b = t.Bike ?? BikeJson.Empty;
        SkyJson s = t.Sky ?? SkyJson.Empty;

        return new(
            t.Name ?? "Unnamed",
            t.Description ?? "",
            Hex(c.Background ?? "#000000"),
            Hex(c.Terrain ?? "#808080"),
            Hex(c.VerticalLine ?? "#404040"),
            BikeDefaults.Parse(b),
            new(s.Palette ?? [], s.HasStars, s.HasSun));
    }

    internal static Color Hex(string? s)
    {
        if (string.IsNullOrEmpty(s))
            return BikeDefaults.Fallback;

        ReadOnlySpan<char> hex = s.AsSpan();
        if (hex[0] == '#')
            hex = hex[1..];

        if (hex.Length < BikeDefaults.MinHexLen)
            return BikeDefaults.Fallback;

        if (!TryParseHexByte(hex[..2], out byte r)
            || !TryParseHexByte(hex[2..4], out byte g)
            || !TryParseHexByte(hex[4..6], out byte b))
            return BikeDefaults.Fallback;

        byte a = hex.Length >= 8
            && TryParseHexByte(hex[6..8], out byte av)
            ? av : (byte)255;

        return new(r, g, b, a);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static bool TryParseHexByte(
        ReadOnlySpan<char> h, out byte result) =>
        byte.TryParse(h, NumberStyles.HexNumber,
            null, out result);

    internal static Color[] ParsePal(string[] arr) =>
        [.. arr.Select(Hex)];
}

public static class ThemeManager
{
    const string WorldPath = "Content/Data/World.json";

    public static RenderCfg Rendering
    {
        get; internal set;
    } = new();

    public static ThemeState State { get; } = new();

    public static ThemeSettings Cur => State.Cur;
    public static int Idx => State.Idx;
    public static int Cnt => State.Cnt;

    public static event Action? Changed
    {
        add => State.Changed += value;
        remove => State.Changed -= value;
    }

    static ThemeManager() => State.Load(WorldPath);

    public static void Set(int i) => State.Set(i);
    public static void Next() => State.Next();
    public static void Prev() => State.Prev();

    public static string GetName(int i) =>
        State.GetName(i);

    public static string GetDesc(int i) =>
        State.GetDesc(i);

    public static Color[] ParsePal(string[] pal) =>
        ThemeState.ParsePal(pal);

    public static void SaveToSettings()
    {
        var s = GameSettings.Load();
        s.ThemeIdx = Idx;
        s.Save();
    }
}
