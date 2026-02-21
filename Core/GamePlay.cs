namespace GravityDefiedGame.Core;

public readonly struct PlayCfg
{
    public readonly int LevelCount, DiffStep;
    public readonly float MaxFall;

    public PlayCfg()
    {
        LevelCount = 10;
        DiffStep = 5;
        MaxFall = 2000f;
    }

    public int DiffFor(int i) => (i - 1) / DiffStep + 1;
}

public readonly struct BikeDefs
{
    public static readonly BikeType[] Types =
        [BikeType.Standard, BikeType.Sport, BikeType.OffRoad];

    public static readonly string[] Names =
        ["Standard", "Sport", "OffRoad"];

    public static int Count => Types.Length;
}

public enum GameState : byte
{
    MainMenu, Playing, Paused, GameOver, LevelComplete
}

public sealed class GamePlay : IDisposable
{
    static readonly PlayCfg Cfg = new();

    bool _disposed;
    int _curLevelId = -1;

    public Bike? Bike { get; private set; } =
        new(BikeType.Standard);
    public Level? Level { get; private set; }
    public List<Level> Levels { get; } = [];
    public IReadOnlyList<BikeType> AvailBikes =>
        BikeDefs.Types;
    public BikeType BikeType { get; private set; } =
        BikeType.Standard;
    public GameState State { get; private set; } =
        GameState.MainMenu;
    public TimeSpan Time { get; private set; }

    public void LoadLevels()
    {
        Levels.Clear();
        for (int i = 1; i <= Cfg.LevelCount; i++)
            Levels.Add(Level.Create(
                i, Strings.LevelN(i - 1),
                seed: null, diff: Cfg.DiffFor(i)));
    }

    public void StartLevel(int id)
    {
        if (_curLevelId == id
            && State == GameState.Playing)
            return;

        if ((uint)(id - 1) >= (uint)Levels.Count)
            return;
        Level lv = Levels[id - 1];

        (_curLevelId, Level, Time) =
            (id, lv, TimeSpan.Zero);

        if (Bike is not null)
        {
            Bike.SetInput(0f, 0f, 0f);
            Bike.SetType(BikeType);
            Bike.SetTerrain(lv);
            Bike.Reset(lv.StartX);
        }

        State = GameState.Playing;
    }

    public void Update(float dt)
    {
        if (_disposed || State != GameState.Playing
            || Level is null || Bike is null)
            return;

        Time += TimeSpan.FromSeconds(dt);
        Bike.Update(dt);

        if (Level.IsFinish(Bike.Pos))
            State = GameState.LevelComplete;
        else if (Bike.Pos.Y > Cfg.MaxFall || Bike.Crashed)
            State = GameState.GameOver;
    }

    public void HandleInput(BikeInput inp) =>
        Bike?.SetInput(inp.Throttle, inp.Brake, inp.Lean);

    public void Pause()
    {
        if (State == GameState.Playing)
            State = GameState.Paused;
    }

    public void Resume()
    {
        if (State == GameState.Paused)
            State = GameState.Playing;
    }

    public void ResetState()
    {
        _curLevelId = -1;
        State = GameState.MainMenu;
    }

    public void SetBike(BikeType t)
    {
        BikeType = t;
        Bike?.SetType(t);
    }

    public void Dispose()
    {
        if (_disposed)
            return;
        _disposed = true;
        Bike?.Dispose();
        Bike = null;
    }
}
