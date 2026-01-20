namespace GravityDefiedGame.Core;

public enum LineType { Frame, Wheel, Shock, Fork, Rider, Leg, Body, Fist }

public sealed record WorldData(List<ThemeJson>? Themes, RenderCfg? Rendering);

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
}

public sealed record SkyJson
{
    public string[]? Palette { get; init; }
    public bool HasStars { get; init; }
    public bool HasSun { get; init; }
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
    string Name, string Desc, Color BgColor, Color TerrainColor, Color VLineColor,
    Dictionary<LineType, Color> BikeColors, ThemeRenderCfg RenderCfg);

public sealed class ThemeState
{
    static readonly JsonSerializerOptions Opt = new() { PropertyNameCaseInsensitive = true };

    readonly List<ThemeSettings> _themes = [];

    public ThemeSettings Cur => _themes[Idx];
    public int Idx { get; private set; }
    public int Cnt => _themes.Count;

    public event Action? Changed;

    public void Load(string path)
    {
        WorldData data = JsonSerializer.Deserialize<WorldData>(
            File.ReadAllText(path), Opt)
            ?? throw new InvalidDataException("World.json broken");

        ThemeManager.Rendering = data.Rendering ?? new();

        _themes.Clear();
        _themes.AddRange(data.Themes!.Select(ToTheme));

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

    public void Next() => Set((Idx + 1) % _themes.Count);
    public void Prev() => Set((Idx - 1 + _themes.Count) % _themes.Count);

    public string GetName(int i) => _themes[i].Name;
    public string GetDesc(int i) => _themes[i].Desc;

    static ThemeSettings ToTheme(ThemeJson t)
    {
        ColorsJson c = t.Colors!;
        BikeJson b = t.Bike!;
        SkyJson s = t.Sky!;

        return new(
            t.Name!, t.Description!,
            Hex(c.Background!), Hex(c.Terrain!), Hex(c.VerticalLine!),
            ParseBike(b),
            new(s.Palette ?? [], s.HasStars, s.HasSun));
    }

    static Dictionary<LineType, Color> ParseBike(BikeJson b) => new()
    {
        [LineType.Frame] = Hex(b.Frame ?? "#808080"),
        [LineType.Wheel] = Hex(b.Wheel ?? "#404040"),
        [LineType.Shock] = Hex(b.Shock ?? "#606060"),
        [LineType.Fork] = Hex(b.Fork ?? "#606060"),
        [LineType.Rider] = Hex(b.Rider ?? "#000000"),
        [LineType.Leg] = Hex(b.Leg ?? "#000000"),
        [LineType.Body] = Hex(b.Body ?? "#000080"),
        [LineType.Fist] = Hex(b.Fist ?? "#9C0000")
    };

    internal static Color Hex(string s)
    {
        ReadOnlySpan<char> hex = s.AsSpan();
        if (hex[0] == '#')
            hex = hex[1..];

        byte r = ToByte(hex[..2]);
        byte g = ToByte(hex[2..4]);
        byte b = ToByte(hex[4..6]);
        byte a = hex.Length >= 8 ? ToByte(hex[6..8]) : (byte)255;

        return new(r, g, b, a);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static byte ToByte(ReadOnlySpan<char> h) => (byte)Convert.ToInt32(h.ToString(), 16);

    internal static Color[] Pal(string[] arr) => [.. arr.Select(Hex)];
}

public static class ThemeManager
{
    const string WorldPath = "Content/Data/World.json";

    public static RenderCfg Rendering { get; internal set; } = new();
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

    public static string GetName(int i) => State.GetName(i);
    public static string GetDesc(int i) => State.GetDesc(i);

    public static Color[] ParsePal(string[] pal) => ThemeState.Pal(pal);

    public static void SaveToSettings()
    {
        var s = GameSettings.Load();
        s.ThemeIdx = Idx;
        s.Save();
    }
}
