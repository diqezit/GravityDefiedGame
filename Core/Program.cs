using Serilog;

namespace GravityDefiedGame.Core;

public static class Program
{
    private const string LogFileName = "game.log";

    private const string Tpl =
        "[{Timestamp:HH:mm:ss}][{Level:u1}][{SourceContext}] {Message:lj}{NewLine}";

    [STAThread]
    private static void Main()
    {
        string logPath = Path.Combine(
            AppDomain.CurrentDomain.BaseDirectory,
            LogFileName);

        TryWriteHeader(logPath);
        ConfigureSerilog(logPath);

        Log.Info("App", "=== Starting ===");

        try
        {
            using Game game = new();
            game.Run();
        }
        catch (Exception ex)
        {
            Log.Error("App", $"Unhandled: {ex}");
            throw;
        }
        finally
        {
            Log.Info("App", "=== Shutdown ===");
            Serilog.Log.CloseAndFlush();
        }
    }

    private static void TryWriteHeader(string path)
    {
        try
        {
            File.WriteAllText(
                path,
                $"=== Game Log {DateTime.Now:dd.MM.yyyy HH:mm:ss} ==={Environment.NewLine}");
        }
        catch { }
    }

    private static void ConfigureSerilog(string logPath)
    {
        try
        {
            Serilog.Log.Logger = new LoggerConfiguration()
                .MinimumLevel.Verbose()
                .Enrich.FromLogContext()
                .WriteTo.Debug(outputTemplate: Tpl)
                .WriteTo.File(
                    logPath,
                    outputTemplate: Tpl,
                    shared: true,
                    buffered: false,
                    flushToDiskInterval: TimeSpan.FromMilliseconds(25))
                .CreateLogger();
        }
        catch
        {
            Serilog.Log.Logger = new LoggerConfiguration()
                .MinimumLevel.Verbose()
                .Enrich.FromLogContext()
                .WriteTo.Debug(outputTemplate: Tpl)
                .CreateLogger();
        }
    }
}

public static class Log
{
    public static bool On { get; set; } = true;

    private static ILogger Ctx(string src) =>
        Serilog.Log.ForContext("SourceContext", src);

    public static void Verbose(string src, string msg)
    {
        if (!On)
            return;
        Ctx(src).Verbose(msg);
    }

    public static void Debug(string src, string msg)
    {
        if (!On)
            return;
        Ctx(src).Debug(msg);
    }

    public static void Info(string src, string msg)
    {
        if (!On)
            return;
        Ctx(src).Information(msg);
    }

    public static void Info(string src, string[] lines)
    {
        if (!On)
            return;

        ILogger ctx = Ctx(src);
        foreach (string ln in lines)
            ctx.Information(ln);
    }

    public static void Warning(string src, string msg)
    {
        if (!On)
            return;
        Ctx(src).Warning(msg);
    }

    public static void Error(string src, string msg)
    {
        if (!On)
            return;
        Ctx(src).Error(msg);
    }

    public static void Fatal(string src, string msg)
    {
        if (!On)
            return;
        Ctx(src).Fatal(msg);
    }

    public static void Flush() => Serilog.Log.CloseAndFlush();
}
