namespace GravityDefiedGame.Core;

public static class BikeDefs
{
    public static readonly BikeType[] Types = [BikeType.Standard, BikeType.Sport, BikeType.OffRoad];
    public static int Count => Types.Length;
    public static string Name(int i) => Types[ClampBike(i)].ToString();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int ClampBike(int i) => Count > 0 ? Math.Clamp(i, 0, Count - 1) : 0;
}

public enum GameState : byte { MainMenu, Playing, Paused, GameOver, LevelComplete }

public sealed class GamePlay : IDisposable
{
    public const float BikeScale = 6f;

    const int LevelCount = 10, DiffStep = 5;
    const float MaxYPx = 2000f;

    bool _dis, _tr;

    public BikePhysics? Bike { get; private set; } = new(BikeType.Standard);
    public Level? Level { get; private set; }
    public List<Level> Levels { get; } = [];
    public BikeType BikeType { get; private set; } = BikeType.Standard;
    public GameState State { get; private set; } = GameState.MainMenu;
    public TimeSpan Time { get; private set; }

    public void LoadLevels()
    {
        Levels.Clear();
        for (int i = 1; i <= LevelCount; i++)
            Levels.Add(Level.Create(i, $"Level {i}", null, (i - 1) / DiffStep + 1));
    }

    public void StartLevel(int id)
    {
        if ((uint)(id - 1) >= (uint)Levels.Count)
            return;

        Level = Levels[id - 1];
        (Time, _tr, State) = (TimeSpan.Zero, false, GameState.Playing);

        if (Bike is not { } b)
            return;

        b.SetInput(0f, 0f, 0f);
        b.SetType(BikeType);
        b.SetTerrain(Level);
        b.Reset(Level.StartX / BikeScale);
    }

    public void Update(float dt)
    {
        if (_dis || State != GameState.Playing || Level is null || Bike is null)
            return;

        float px = Bike.Pos.X * BikeScale;
        Bike.Update(dt);
        float cx = Bike.Pos.X * BikeScale;
        float yPx = Bike.Pos.Y * BikeScale;

        if (yPx > MaxYPx)
        {
            _tr = false;
            State = GameState.GameOver;
            return;
        }

        if (Bike.Crashed)
        {
            _tr = false;
            if (Bike.RagdollDone)
                State = GameState.GameOver;
            return;
        }

        _tr |= Level.CrossedStartGate(px, cx);
        if (!_tr)
            return;

        Time += TimeSpan.FromSeconds(dt);
        if (Level.CrossedFinishGate(px, cx))
            State = GameState.LevelComplete;
    }

    public void HandleInput(BikeInput inp)
    {
        if (Bike is { Crashed: false })
            Bike.SetInput(inp.Throttle, inp.Brake, inp.Lean);
    }

    public void Pause()
    {
        if (State == GameState.Playing && Bike is { Crashed: false })
            State = GameState.Paused;
    }

    public void Resume()
    {
        if (State == GameState.Paused)
            State = GameState.Playing;
    }

    public void ResetState()
    {
        (Level, Time, _tr, State) = (null, TimeSpan.Zero, false, GameState.MainMenu);

        if (Bike is not { } b)
            return;

        b.SetInput(0f, 0f, 0f);
        b.Reset(0f);
    }

    public void SetBike(BikeType t)
    {
        BikeType = t;
        Bike?.SetType(t);
    }

    public void Dispose()
    {
        if (_dis)
            return;
        _dis = true;
        Bike?.Dispose();
        Bike = null;
    }
}
