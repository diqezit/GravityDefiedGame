using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using GravityDefiedGame.Models;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame.Views
{
    public record ThemeSettings(
        string Name,
        Color BackgroundColor,
        Color TerrainColor,
        Color SafeZoneColor,
        Color VerticalLineColor,
        Dictionary<BikeGeom.SkeletonLineType, Color> BikeColors,
        Color WheelFill,
        Color WheelStroke,
        Color SpokeStroke,
        Color ShadowColor,
        string Description);

    public static class ThemeManager
    {
        private static int _currentThemeIndex;
        private static readonly List<ThemeSettings> _themes = new();

        public static event Action? ThemeChanged;

        public static ThemeSettings CurrentTheme => _themes[_currentThemeIndex];
        public static List<ThemeSettings> AvailableThemes => _themes;

        static ThemeManager() => InitializeThemes();

        private static void InitializeThemes()
        {
            _themes.Clear();
            _themes.AddRange(new[]
            {
                CreateDesertTheme(),
                CreateMountainTheme(),
                CreateArcticTheme(),
                CreateVolcanoTheme(),
                CreateNeonTheme()
            });

            Info("ThemeManager", $"Initialized {_themes.Count} themes");
        }

        public static void SetTheme(int index)
        {
            if (index < 0 || index >= _themes.Count)
            {
                Warning("ThemeManager", $"Invalid theme index: {index}");
                index = 0;
            }

            if (index != _currentThemeIndex)
            {
                _currentThemeIndex = index;
                Debug("ThemeManager", $"Theme changed to {CurrentTheme.Name}");
                ThemeChanged?.Invoke();
            }
        }

        public static void SetNextTheme() =>
            SetTheme((_currentThemeIndex + 1) % _themes.Count);

        public static void SetPreviousTheme() =>
            SetTheme((_currentThemeIndex + _themes.Count - 1) % _themes.Count);

        #region Theme Definitions
        private static ThemeSettings CreateDesertTheme() => new(
            "Desert",
            new Color(255, 204, 102),
            new Color(204, 153, 102),
            new Color(152, 251, 152),
            new Color(204, 153, 102),
            new Dictionary<BikeGeom.SkeletonLineType, Color>
            {
                [BikeGeom.SkeletonLineType.MainFrame] = new Color(150, 150, 150),
                [BikeGeom.SkeletonLineType.Suspension] = Color.Black,
                [BikeGeom.SkeletonLineType.Wheel] = Color.Black,
                [BikeGeom.SkeletonLineType.Seat] = new Color(200, 200, 200),
                [BikeGeom.SkeletonLineType.Handlebar] = new Color(150, 150, 150),
                [BikeGeom.SkeletonLineType.Exhaust] = new Color(100, 100, 100)
            },
            new Color(200, 200, 200),
            Color.Black,
            Color.Black,
            new Color(0, 0, 0, 128),
            "Warm sand colors for desert riding"
        );

        private static ThemeSettings CreateMountainTheme() => new(
            "Mountain",
            new Color(155, 175, 200),
            new Color(90, 110, 90),
            new Color(150, 200, 150),
            new Color(70, 90, 70),
            new Dictionary<BikeGeom.SkeletonLineType, Color>
            {
                [BikeGeom.SkeletonLineType.MainFrame] = new Color(70, 70, 90),
                [BikeGeom.SkeletonLineType.Suspension] = new Color(50, 50, 70),
                [BikeGeom.SkeletonLineType.Wheel] = Color.Black,
                [BikeGeom.SkeletonLineType.Seat] = new Color(100, 100, 120),
                [BikeGeom.SkeletonLineType.Handlebar] = new Color(70, 70, 90),
                [BikeGeom.SkeletonLineType.Exhaust] = new Color(60, 60, 60)
            },
            new Color(150, 150, 150),
            Color.Black,
            new Color(70, 70, 70),
            new Color(0, 0, 0, 128),
            "Cool mountain tones with forest terrain"
        );

        private static ThemeSettings CreateArcticTheme() => new(
            "Arctic",
            new Color(220, 235, 255),
            new Color(200, 220, 240),
            new Color(180, 200, 255),
            new Color(190, 210, 230),
            new Dictionary<BikeGeom.SkeletonLineType, Color>
            {
                [BikeGeom.SkeletonLineType.MainFrame] = new Color(100, 130, 160),
                [BikeGeom.SkeletonLineType.Suspension] = new Color(80, 100, 130),
                [BikeGeom.SkeletonLineType.Wheel] = new Color(40, 40, 40),
                [BikeGeom.SkeletonLineType.Seat] = new Color(70, 90, 120),
                [BikeGeom.SkeletonLineType.Handlebar] = new Color(100, 130, 160),
                [BikeGeom.SkeletonLineType.Exhaust] = new Color(50, 70, 90)
            },
            new Color(150, 170, 190),
            new Color(40, 40, 40),
            new Color(70, 90, 110),
            new Color(70, 100, 130, 128),
            "Icy blues for winter riding"
        );

        private static ThemeSettings CreateVolcanoTheme() => new(
            "Volcano",
            new Color(50, 20, 20),
            new Color(80, 40, 30),
            new Color(255, 200, 50),
            new Color(120, 40, 30),
            new Dictionary<BikeGeom.SkeletonLineType, Color>
            {
                [BikeGeom.SkeletonLineType.MainFrame] = new Color(200, 50, 20),
                [BikeGeom.SkeletonLineType.Suspension] = new Color(30, 30, 30),
                [BikeGeom.SkeletonLineType.Wheel] = Color.Black,
                [BikeGeom.SkeletonLineType.Seat] = new Color(150, 40, 20),
                [BikeGeom.SkeletonLineType.Handlebar] = new Color(200, 50, 20),
                [BikeGeom.SkeletonLineType.Exhaust] = new Color(60, 60, 60)
            },
            new Color(100, 100, 100),
            Color.Black,
            new Color(70, 70, 70),
            new Color(90, 20, 10, 128),
            "Hot volcanic landscape with lava accents"
        );

        private static ThemeSettings CreateNeonTheme() => new(
            "Neon",
            new Color(20, 10, 30),
            new Color(20, 20, 40),
            new Color(0, 255, 200),
            new Color(100, 0, 200),
            new Dictionary<BikeGeom.SkeletonLineType, Color>
            {
                [BikeGeom.SkeletonLineType.MainFrame] = new Color(0, 230, 255),
                [BikeGeom.SkeletonLineType.Suspension] = new Color(255, 50, 150),
                [BikeGeom.SkeletonLineType.Wheel] = new Color(200, 0, 255),
                [BikeGeom.SkeletonLineType.Seat] = new Color(255, 230, 0),
                [BikeGeom.SkeletonLineType.Handlebar] = new Color(0, 230, 255),
                [BikeGeom.SkeletonLineType.Exhaust] = new Color(255, 50, 150)
            },
            new Color(200, 0, 255),
            new Color(255, 255, 255),
            new Color(0, 230, 255),
            new Color(0, 200, 255, 128),
            "Vibrant neon colors for night riding"
        );
        #endregion
    }
}