using Serilog;

namespace GravityDefiedGame.Core;

public static class Program
{
    const string LogFile = "physics_debug.txt";

    [STAThread]
    static void Main()
    {
        if (File.Exists(LogFile))
            File.Delete(LogFile);

        using Serilog.Core.Logger logger = new LoggerConfiguration()
            .MinimumLevel.Debug()
            .WriteTo.File(LogFile)
            .CreateLogger();

        Log.Logger = logger;

        using Game game = new();
        game.Run();
    }
}
