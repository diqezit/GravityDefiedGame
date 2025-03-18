using System;
using System.IO;
using System.Diagnostics;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using System.Reflection;

namespace GravityDefiedGame.Utilities
{
    public enum LogLevel { D, I, W, E, F }

    public static class LoggerImpl
    {
        private static readonly string LogFilePath = Path.Combine(
            Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location) ?? AppDomain.CurrentDomain.BaseDirectory,
            "logger.log");

        private static readonly ConcurrentQueue<string> _logQueue = new();
        private static bool _isProcessingQueue;
        private static readonly object _lockObject = new();

        public static LogLevel MinLogLevel { get; set; } = LogLevel.D;
        public static bool IsLoggingEnabled { get; set; } = true;

        static LoggerImpl()
        {
            try
            {
                string directory = Path.GetDirectoryName(LogFilePath);
                if (!Directory.Exists(directory)) Directory.CreateDirectory(directory);
                if (File.Exists(LogFilePath)) File.Delete(LogFilePath);

                using StreamWriter writer = File.CreateText(LogFilePath);
                writer.WriteLine($"=== Gravity Defied Log - {DateTime.Now:dd.MM.yyyy HH:mm:ss} ===");

                LogMessage(LogLevel.I, "Log", $"Started: {LogFilePath}");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Logger init failed: {ex.Message}");
                IsLoggingEnabled = false;
            }
        }

        public static void LogMessage(LogLevel level, string source, string message)
        {
            if (!IsLoggingEnabled) return;

            string logMessage = $"[{DateTime.Now:HH:mm:ss}][{level}][{source}] {message}";
            Debug.WriteLine(logMessage);
            _logQueue.Enqueue(logMessage);
            StartQueueProcessing();
        }

        private static void StartQueueProcessing()
        {
            lock (_lockObject)
            {
                if (_isProcessingQueue) return;
                _isProcessingQueue = true;
                Task.Run(ProcessLogQueue);
            }
        }

        private static async Task ProcessLogQueue()
        {
            try
            {
                while (_logQueue.TryDequeue(out string logMessage))
                {
                    try
                    {
                        await File.AppendAllTextAsync(LogFilePath, logMessage + Environment.NewLine);
                    }
                    catch (Exception ex) { Debug.WriteLine($"Log write error: {ex.Message}"); }
                    await Task.Delay(5);
                }
            }
            finally
            {
                lock (_lockObject)
                {
                    _isProcessingQueue = false;
                    if (!_logQueue.IsEmpty) StartQueueProcessing();
                }
            }
        }

        public static void LogDebug(string source, string message) => LogMessage(LogLevel.D, source, message);
        public static void LogInfo(string source, string message) => LogMessage(LogLevel.I, source, message);
        public static void LogWarning(string source, string message) => LogMessage(LogLevel.W, source, message);
        public static void LogError(string source, string message) => LogMessage(LogLevel.E, source, message);
        public static void LogFatal(string source, string message) => LogMessage(LogLevel.F, source, message);

        public static void LogException(string source, Exception ex)
        {
            LogMessage(LogLevel.E, source, $"Ex: {ex.Message}");
            if (ex.StackTrace != null)
            {
                var stack = ex.StackTrace.Split(Environment.NewLine, StringSplitOptions.RemoveEmptyEntries);
                if (stack.Length > 0) LogMessage(LogLevel.D, source, $"Stack: {stack[0]}");
            }
            if (ex.InnerException != null)
                LogMessage(LogLevel.D, source, $"Inner: {ex.InnerException.Message}");
        }
    }

    public static class Logger
    {
        public static void Debug(string source, string message) => LoggerImpl.LogDebug(source, message);
        public static void Info(string source, string message) => LoggerImpl.LogInfo(source, message);
        public static void Warning(string source, string message) => LoggerImpl.LogWarning(source, message);
        public static void Error(string source, string message) => LoggerImpl.LogError(source, message);
        public static void Fatal(string source, string message) => LoggerImpl.LogFatal(source, message);
        public static void Exception(string source, Exception ex) => LoggerImpl.LogException(source, ex);
    }
}