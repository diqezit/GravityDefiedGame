namespace GravityDefiedGame.Core;

public enum LineType : byte { Frame, Wheel, Shock, Fork, Rider, Leg, Body, Fist }

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

public readonly record struct ThemeRenderCfg(
    string[] SkyPal, bool HasStars, bool HasSun);

public sealed record ThemeSettings(
    string Name, string Desc,
    Color BgColor, Color TerrainColor, Color VLineColor,
    Dictionary<LineType, Color> BikeColors,
    ThemeRenderCfg RenderCfg);

public static class ThemeManager
{
    const string WorldPath = "Content/Data/World.json";
    const int MinHexLen = 6;

    static readonly Color
        DefBg = new(0, 0, 0),
        DefTerrain = new(128, 128, 128),
        DefVLine = new(64, 64, 64),
        Fallback = Color.Magenta;

    static readonly JsonSerializerOptions Opt = new(JsonSerializerDefaults.Web);
    static readonly List<ThemeSettings> Themes = [];

    public static RenderCfg Rendering { get; private set; } = new();
    public static ThemeSettings Cur => Themes[Idx];
    public static int Idx { get; private set; }
    public static int Cnt => Themes.Count;

    public static event Action? Changed;

    static ThemeManager() => Load(WorldPath);

    public static void Set(int i)
    {
        int next = ClampTheme(i);
        if (next == Idx)
            return;
        Idx = next;
        Changed?.Invoke();
    }

    public static string NameUpper(int i) =>
        Themes[ClampTheme(i)].Name.ToUpperInvariant();

    public static string? DescUpper(int i) =>
        Themes[ClampTheme(i)].Desc.ToUpperInvariant();

    public static void Apply(int i)
    {
        Set(i);
        var s = GameSettings.Load();
        s.ThemeIdx = Idx;
        s.Save();
    }

    public static Color[] ParsePal(string[] arr) =>
        Array.ConvertAll(arr, Hex);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int ClampTheme(int i) =>
        Cnt > 0 ? Math.Clamp(i, 0, Cnt - 1) : 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Cycle(int idx, int count, int dir) =>
        (idx + dir + count) % count;

    static Color HexOr(string? s, Color def) =>
        s != null ? Hex(s) : def;

    internal static Color Hex(string? s)
    {
        if (string.IsNullOrEmpty(s))
            return Fallback;

        ReadOnlySpan<char> hex = s.AsSpan();
        if (hex[0] == '#')
            hex = hex[1..];
        if (hex.Length < MinHexLen)
            return Fallback;

        if (!HexByte(hex[..2], out byte r) ||
            !HexByte(hex[2..4], out byte g) ||
            !HexByte(hex[4..6], out byte b))
            return Fallback;

        byte a = hex.Length >= 8 && HexByte(hex[6..8], out byte av)
            ? av : (byte)255;
        return new(r, g, b, a);
    }

    static void Load(string path)
    {
        WorldData data = JsonSerializer.Deserialize<WorldData>(
            File.ReadAllText(path), Opt)
            ?? throw new InvalidDataException("World.json broken");

        Rendering = data.Rendering ?? new();

        List<ThemeJson> list = data.Themes ?? [];
        if (list.Count == 0)
            throw new InvalidDataException("World.json: no themes");

        Themes.Clear();
        foreach (ThemeJson t in list)
            Themes.Add(ToTheme(t));

        Idx = 0;
        Changed?.Invoke();
    }

    static ThemeSettings ToTheme(ThemeJson t)
    {
        SkyJson s = t.Sky ?? SkyJson.Empty;

        return new(
            t.Name ?? "Unnamed",
            t.Description ?? "",
            HexOr(t.Background, DefBg),
            HexOr(t.Terrain, DefTerrain),
            HexOr(t.VerticalLine, DefVLine),
            ParseBike(t.Bike ?? BikeJson.Empty),
            new(s.Palette ?? [], s.HasStars, s.HasSun));
    }

    static Dictionary<LineType, Color> ParseBike(BikeJson b) => new(8)
    {
        [LineType.Frame] = HexOr(b.Frame, new(128, 128, 128)),
        [LineType.Wheel] = HexOr(b.Wheel, new(64, 64, 64)),
        [LineType.Shock] = HexOr(b.Shock, new(96, 96, 96)),
        [LineType.Fork] = HexOr(b.Fork, new(96, 96, 96)),
        [LineType.Rider] = HexOr(b.Rider, new(0, 0, 0)),
        [LineType.Leg] = HexOr(b.Leg, new(0, 0, 0)),
        [LineType.Body] = HexOr(b.Body, new(0, 0, 128)),
        [LineType.Fist] = HexOr(b.Fist, new(156, 0, 0)),
    };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static bool HexByte(ReadOnlySpan<char> h, out byte result) =>
        byte.TryParse(h, NumberStyles.HexNumber, null, out result);
}
