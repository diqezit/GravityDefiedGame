#nullable enable

using System;
using System.IO;
using System.Diagnostics;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using System.Reflection;

namespace GravityDefiedGame.Utilities
{
    public enum LogLevel { D, I, W, E, F }

    public static class LoggerCore
    {
        private static readonly string LogFilePath = Path.Combine(
            Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location) ?? AppDomain.CurrentDomain.BaseDirectory,
            "logger.log");

        private static readonly ConcurrentQueue<string> _logQueue = new();
        private static bool _isProcessingQueue;
        private static readonly object _lockObject = new();

        public static LogLevel MinLogLevel { get; set; } = LogLevel.D;
        public static bool IsLoggingEnabled { get; set; } = true;

        static LoggerCore()
        {
            try
            {
                string? directory = Path.GetDirectoryName(LogFilePath);
                if (directory != null && !Directory.Exists(directory))
                {
                    Directory.CreateDirectory(directory);
                }
                if (File.Exists(LogFilePath)) File.Delete(LogFilePath);

                using StreamWriter writer = File.CreateText(LogFilePath);
                writer.WriteLine($"=== Gravity Defied Log - {DateTime.Now:dd.MM.yyyy HH:mm:ss} ===");

                WriteLog(LogLevel.I, "Log", $"Started: {LogFilePath}");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Logger init failed: {ex.Message}");
                IsLoggingEnabled = false;
            }
        }

        public static void WriteLog(LogLevel level, string source, string message)
        {
            if (!IsLoggingEnabled) return;

            string logMessage = $"[{DateTime.Now:HH:mm:ss}][{level}][{source}] {message}";
            Debug.WriteLine(logMessage);
            _logQueue.Enqueue(logMessage);

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
                while (_logQueue.TryDequeue(out string? logMessage))
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
                    if (!_logQueue.IsEmpty)
                    {
                        _isProcessingQueue = true;
                        Task.Run(ProcessLogQueue);
                    }
                }
            }
        }
    }

    public static class Logger
    {
        public static void Debug(string source, string message) => LoggerCore.WriteLog(LogLevel.D, source, message);
        public static void Info(string source, string message) => LoggerCore.WriteLog(LogLevel.I, source, message);
        public static void Warning(string source, string message) => LoggerCore.WriteLog(LogLevel.W, source, message);
        public static void Error(string source, string message) => LoggerCore.WriteLog(LogLevel.E, source, message);
        public static void Fatal(string source, string message) => LoggerCore.WriteLog(LogLevel.F, source, message);

        public static void Exception(string source, Exception ex)
        {
            LoggerCore.WriteLog(LogLevel.E, source, $"Ex: {ex.Message}");
            if (ex.StackTrace != null)
            {
                var stack = ex.StackTrace.Split(Environment.NewLine, StringSplitOptions.RemoveEmptyEntries);
                if (stack.Length > 0) LoggerCore.WriteLog(LogLevel.D, source, $"Stack: {stack[0]}");
            }
            if (ex.InnerException != null)
                LoggerCore.WriteLog(LogLevel.D, source, $"Inner: {ex.InnerException.Message}");
        }

        public static T Log<T>(string source, string operation, Func<T> action, T? defaultValue)
        {
            try
            {
                return action();
            }
            catch (Exception ex)
            {
                Error(source, $"Error in {operation}: {ex.Message}");
                return defaultValue!;
            }
        }

        public static void Log(string source, string operation, Action action)
        {
            try
            {
                action();
            }
            catch (Exception ex)
            {
                Error(source, $"Error in {operation}: {ex.Message}");
            }
        }
    }
}