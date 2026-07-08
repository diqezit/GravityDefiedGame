// Ghost records bike telemetry during a timed run and replays it later
// No physics re-simulated — only raw positions stored and played back in lockstep
// Best time per level kept in ghosts.json

namespace GravityDefiedGame.Core;

public readonly record struct GhostFrame(
    float Fx, float Fy, float Fwx, float Fwy, float FwRot,
    float Rwx, float Rwy, float RwRot, float Ufx, float Ufy,
    float Urx, float Ury, float Hdx, float Hdy, float TiltZ);

public sealed class GhostData
{
    public float Time { get; set; }
    public List<GhostFrame> Frames { get; set; } = [];
}

public static class GhostStore
{
    const string Path = "ghosts.json";

    static Dictionary<int, GhostData> _data = [];

    public static void Load() =>
        _data = File.Exists(Path)
            ? JsonSerializer.Deserialize<Dictionary<int, GhostData>>(File.ReadAllText(Path)) ?? []
            : [];

    public static void Save() =>
        File.WriteAllText(Path, JsonSerializer.Serialize(_data));

    public static GhostData? Get(int levelId) =>
        _data.TryGetValue(levelId, out GhostData? g) ? g : null;

    public static void SaveIfBetter(int levelId, GhostData run)
    {
        if (_data.TryGetValue(levelId, out GhostData? best) && run.Time >= best.Time)
            return;

        _data[levelId] = run;
        Save();
    }

    public static void Reset()
    {
        _data.Clear();
        if (File.Exists(Path))
            File.Delete(Path);
    }
}

public sealed class Ghost
{
    public bool Enabled { get; set; } = true;

    public GhostFrame? CurrentFrame =>
        _playing != null && _idx < _playing.Frames.Count
            ? _playing.Frames[_idx]
            : null;

    readonly List<GhostFrame> _recording = [];
    GhostData? _playing;
    int _idx;
    int _levelId;

    public void Start(int levelId, GhostData? best)
    {
        _levelId = levelId;
        _playing = best;
        _idx = 0;
        _recording.Clear();
    }

    public void Tick(BikePhysics bike)
    {
        _recording.Add(new(
            bike.Pos.X, bike.Pos.Y,
            bike.FwPos.X, bike.FwPos.Y, bike.FwRot,
            bike.RwPos.X, bike.RwPos.Y, bike.RwRot,
            bike.UfPos.X, bike.UfPos.Y,
            bike.UrPos.X, bike.UrPos.Y,
            bike.HdPos.X, bike.HdPos.Y,
            bike.RawTiltZ));

        if (_playing != null && _idx < _playing.Frames.Count - 1)
            _idx++;
    }

    public void Finish(float time) =>
        GhostStore.SaveIfBetter(_levelId, new() { Time = time, Frames = [.. _recording] });
}
