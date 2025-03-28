#nullable enable
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using System;
using System.Collections.Generic;
using System.Linq;
using FontStashSharp;
using static GravityDefiedGame.Views.UI;
using static GravityDefiedGame.Models.BikeGeom;
using static GravityDefiedGame.Views.ThemeManager;
using GravityDefiedGame.Utilities;
using GravityDefiedGame.Controllers;
using static GravityDefiedGame.Utilities.Logger;

namespace GravityDefiedGame.Views
{
    public class UIDrawer
    {
        private readonly SpriteBatch _spriteBatch;
        private readonly (
            DynamicSpriteFont standard,
            DynamicSpriteFont title,
            DynamicSpriteFont unicode
        ) _fonts;
        private readonly (
            Texture2D pixel,
            Texture2D gradient,
            Texture2D? logo
        ) _textures;
        private readonly (int width, int height) _screenSize;

        private List<UISmoke> _smokeParticles = [];
        private Random _random = new();
        private UIPerlinNoise _perlinNoise = new();
        private float _smokeTimer = 0f;

        public UIDrawer(
            SpriteBatch spriteBatch,
            (DynamicSpriteFont standard, DynamicSpriteFont title, DynamicSpriteFont unicode) fonts,
            (Texture2D pixel, Texture2D gradient, Texture2D? logo) textures,
            (int width, int height) screenSize)
        {
            _spriteBatch = spriteBatch;
            _fonts = fonts;
            _textures = textures;
            _screenSize = screenSize;
        }

        public void InitializeSmokeParticles()
        {
            _smokeParticles.Clear();

            for (int i = 0; i < Visual.SmokeLowerLayerCount; i++)
                _smokeParticles.Add(new UISmoke(_random, _screenSize.width, _screenSize.height, SmokeLayer.Lower));

            for (int i = 0; i < Visual.SmokeMiddleLayerCount; i++)
                _smokeParticles.Add(new UISmoke(_random, _screenSize.width, _screenSize.height, SmokeLayer.Middle));

            for (int i = 0; i < Visual.SmokeUpperLayerCount; i++)
                _smokeParticles.Add(new UISmoke(_random, _screenSize.width, _screenSize.height, SmokeLayer.Upper));

            foreach (var particle in _smokeParticles)
                particle.LifeTime = _random.Next(100) / 100f * particle.MaxLifeTime;

            Info("UIDrawer", "Smoke particles initialized");
        }

        public void UpdateSmokeParticles(float elapsedTime)
        {
            _smokeTimer += elapsedTime;

            for (int i = _smokeParticles.Count - 1; i >= 0; i--)
            {
                _smokeParticles[i].Update(elapsedTime, _perlinNoise, _screenSize.width);

                if (_smokeParticles[i].IsExpired())
                    _smokeParticles.RemoveAt(i);
            }

            while (_smokeParticles.Count < Visual.SmokeParticleCount)
            {
                var (lowerCount, middleCount, upperCount) = (
                    _smokeParticles.Count(p => p.Layer == SmokeLayer.Lower),
                    _smokeParticles.Count(p => p.Layer == SmokeLayer.Middle),
                    _smokeParticles.Count(p => p.Layer == SmokeLayer.Upper)
                );

                SmokeLayer layer = lowerCount < Visual.SmokeLowerLayerCount
                    ? SmokeLayer.Lower
                    : middleCount < Visual.SmokeMiddleLayerCount
                        ? SmokeLayer.Middle
                        : upperCount < Visual.SmokeUpperLayerCount
                            ? SmokeLayer.Upper
                            : SmokeLayer.Middle;

                _smokeParticles.Add(new UISmoke(_random, _screenSize.width, _screenSize.height, layer));
            }
        }

        public void DrawSmokeEffect() =>
            _smokeParticles.OrderBy(p => p.Layer)
                          .ToList()
                          .ForEach(p => p.Draw(_spriteBatch, _textures.pixel));

        public void DrawSplashScreen(float fadeAlpha, float loadingProgress, float transitionTimer, bool introComplete)
        {
            Color bgColor1 = Color.Black;
            Color bgColor2 = new Color(10, 10, 30);
            float bgPulse = (float)Math.Sin(transitionTimer * 0.5f) * 0.5f + 0.5f;
            Color bgColor = Color.Lerp(bgColor1, bgColor2, bgPulse);

            _spriteBatch.Draw(_textures.gradient, new Rectangle(0, 0, _screenSize.width, _screenSize.height), bgColor);

            for (int i = 0; i < 50; i++)
            {
                int x = (int)(Math.Sin(i * 0.573f + transitionTimer) * _screenSize.width * 0.5f + _screenSize.width * 0.5f);
                int y = (int)(Math.Cos(i * 0.573f + transitionTimer * 0.7f) * _screenSize.height * 0.5f + _screenSize.height * 0.5f);
                float starBrightness = (float)Math.Sin(i * 0.39f + transitionTimer * 2.0f) * 0.5f + 0.5f;

                if (x > 0 && x < _screenSize.width && y > 0 && y < _screenSize.height)
                    _spriteBatch.Draw(_textures.pixel,
                        new Rectangle(x, y, 2, 2),
                        Color.White * (0.3f * starBrightness));
            }

            float logoScale = 1.0f + (float)Math.Sin(transitionTimer) * 0.03f;

            if (_textures.logo != null)
            {
                int logoWidth = Math.Min(_screenSize.width - 100, _textures.logo.Width);
                int logoHeight = (int)(logoWidth * ((float)_textures.logo.Height / _textures.logo.Width));

                logoWidth = (int)(logoWidth * logoScale);
                logoHeight = (int)(logoHeight * logoScale);

                Rectangle logoRect = new Rectangle(
                    (_screenSize.width - logoWidth) / 2,
                    (_screenSize.height - logoHeight) / 2 - 50,
                    logoWidth,
                    logoHeight);

                _spriteBatch.Draw(_textures.logo, logoRect, Color.White);
            }
            else
            {
                string gameTitle = "GRAVITY DEFIED";
                Vector2 titleSize = _fonts.title.MeasureString(gameTitle);
                float titleX = (_screenSize.width - titleSize.X) / 2;
                float titleY = _screenSize.height * 0.4f;

                _fonts.title.DrawText(_spriteBatch, gameTitle,
                    new Vector2(titleX + Visual.ShadowOffset, titleY + Visual.ShadowOffset),
                    Color.Black * Visual.ShadowOpacity);
                _fonts.title.DrawText(_spriteBatch, gameTitle,
                    new Vector2(titleX, titleY),
                    Color.Goldenrod);
            }

            string messageText = introComplete ? "Press any key to continue" : "Loading game...";
            Vector2 messageTextSize = _fonts.standard.MeasureString(messageText);

            float messageAlpha = introComplete
                ? 0.7f + 0.3f * (float)Math.Sin(transitionTimer * 2.5f)
                : 1.0f;

            Color messageColor = introComplete
                ? Color.Lerp(Color.White, Color.Goldenrod, (float)Math.Sin(transitionTimer) * 0.5f + 0.5f)
                : Color.White;

            Vector2 scaledPosition = new Vector2(
                (_screenSize.width - messageTextSize.X) / 2,
                _screenSize.height * 0.7f);

            _fonts.standard.DrawText(_spriteBatch, messageText,
                scaledPosition,
                messageColor * messageAlpha);

            if (!introComplete)
            {
                int barWidth = Visual.LoadingBarWidth;
                int barHeight = Visual.LoadingBarHeight;
                int barX = (_screenSize.width - barWidth) / 2;
                int barY = (int)(_screenSize.height * 0.75f);

                _spriteBatch.Draw(_textures.pixel,
                    new Rectangle(barX, barY, barWidth, barHeight),
                    new Color(40, 40, 40, 200));

                Rectangle progressRect = new Rectangle(barX, barY, (int)(barWidth * loadingProgress), barHeight);
                _spriteBatch.Draw(_textures.gradient,
                    progressRect,
                    new Rectangle(0, 0, (int)(256 * loadingProgress), 256),
                    Color.LightGreen);

                DrawBorder(new Rectangle(barX - 1, barY - 1, barWidth + 2, barHeight + 2),
                    Color.White, 1);
            }
        }

        public void DrawLoadingScreen(int levelId, string levelName, float loadingProgress)
        {
            _spriteBatch.Draw(_textures.gradient,
                new Rectangle(0, 0, _screenSize.width, _screenSize.height),
                new Color(0, 0, 0, 200));

            DrawSmokeEffect();

            string levelText = $"Loading Level {levelId}: {levelName}";
            Vector2 levelTextSize = _fonts.title.MeasureString(levelText);

            _fonts.title.DrawText(_spriteBatch, levelText,
                new Vector2((_screenSize.width - levelTextSize.X) / 2 + Visual.ShadowOffset,
                           _screenSize.height * 0.4f + Visual.ShadowOffset),
                Color.Black * Visual.ShadowOpacity);

            _fonts.title.DrawText(_spriteBatch, levelText,
                new Vector2((_screenSize.width - levelTextSize.X) / 2, _screenSize.height * 0.4f),
                Color.White);

            int barWidth = Visual.LoadingBarWidth;
            int barHeight = Visual.LoadingBarHeight * 2;
            int barX = (_screenSize.width - barWidth) / 2;
            int barY = (int)(_screenSize.height * 0.6f);

            _spriteBatch.Draw(_textures.pixel,
                new Rectangle(barX, barY, barWidth, barHeight),
                new Color(40, 40, 40, 200));

            Rectangle progressRect = new Rectangle(barX, barY, (int)(barWidth * loadingProgress), barHeight);
            _spriteBatch.Draw(_textures.gradient,
                progressRect,
                new Rectangle(0, 0, (int)(256 * loadingProgress), 256),
                Color.LightBlue);

            DrawBorder(new Rectangle(barX - 1, barY - 1, barWidth + 2, barHeight + 2),
                    Color.White, 1);

            string percentText = $"{(int)(loadingProgress * 100)}%";
            Vector2 percentSize = _fonts.standard.MeasureString(percentText);
            _fonts.standard.DrawText(_spriteBatch, percentText,
                new Vector2((_screenSize.width - percentSize.X) / 2, barY + barHeight + 10),
                Color.White);

            string loadingDetails = GetLoadingStageText(loadingProgress);
            Vector2 detailsSize = _fonts.standard.MeasureString(loadingDetails);
            _fonts.standard.DrawText(_spriteBatch, loadingDetails,
                new Vector2((_screenSize.width - detailsSize.X) / 2, barY + barHeight + 40),
                Color.LightGray);
        }

        private string GetLoadingStageText(float progress) => progress switch
        {
            < 0.25f => "Initializing level...",
            < 0.5f => "Generating terrain...",
            < 0.75f => "Preparing physics...",
            _ => "Almost ready..."
        };

        public void DrawTransitionOverlay(float fadeAlpha, TransitionState transitionState, GameState nextGameState, bool isSplashScreenActive, float transitionTimer)
        {
            _spriteBatch.Draw(_textures.pixel,
                new Rectangle(0, 0, _screenSize.width, _screenSize.height),
                new Color(0, 0, 0, fadeAlpha));

            if (transitionState == TransitionState.FadeIn &&
                nextGameState == GameState.Playing &&
                !isSplashScreenActive && fadeAlpha < 0.95f && fadeAlpha > 0.05f)
            {
                int centerX = _screenSize.width / 2;
                int centerY = _screenSize.height / 2;
                float pulseSize = (1.0f - fadeAlpha) * 200.0f;

                for (int i = 0; i < 3; i++)
                {
                    float ringAlpha = fadeAlpha * 0.3f * (1.0f - (float)i / 3);
                    float ringSize = pulseSize * (1.0f + i * 0.5f);

                    if (ringSize > 0)
                        DrawCircle(centerX, centerY, (int)ringSize, Color.White * ringAlpha, 3);
                }
            }
        }

        private void DrawCircle(int centerX, int centerY, int radius, Color color, int thickness)
        {
            const int segments = 32;
            Vector2[] points = new Vector2[segments + 1];

            for (int i = 0; i <= segments; i++)
            {
                float angle = MathHelper.TwoPi * i / segments;
                points[i] = new Vector2(
                    centerX + (float)Math.Cos(angle) * radius,
                    centerY + (float)Math.Sin(angle) * radius);
            }

            for (int i = 0; i < segments; i++)
                DrawLine(points[i], points[i + 1], color, thickness);
        }

        private void DrawLine(Vector2 start, Vector2 end, Color color, int thickness)
        {
            Vector2 delta = end - start;
            float angle = (float)Math.Atan2(delta.Y, delta.X);

            _spriteBatch.Draw(
                _textures.pixel,
                new Rectangle(
                    (int)start.X,
                    (int)start.Y,
                    (int)delta.Length(),
                    thickness),
                null,
                color,
                angle,
                Vector2.Zero,
                SpriteEffects.None,
                0);
        }

        public void DrawAnimatedBackground(float backgroundOffset, Color backgroundColor)
        {
            _spriteBatch.Draw(_textures.gradient, new Rectangle(0, 0, _screenSize.width, _screenSize.height), backgroundColor);
            _spriteBatch.Draw(_textures.pixel, new Rectangle(0, 0, _screenSize.width, _screenSize.height), new Color(0, 0, 0, 100));

            for (int i = 0; i < 5; i++)
            {
                float x = (backgroundOffset + i * _screenSize.width / 5) % _screenSize.width;
                _spriteBatch.Draw(_textures.pixel,
                    new Rectangle((int)x, _screenSize.height / 4 + i * 20, 100, 20),
                    Color.White * 0.2f);
            }
        }

        public void DrawTooltip(string text, Vector2 position)
        {
            Vector2 tooltipSize = _fonts.standard.MeasureString(text);
            Rectangle tooltipRect = new Rectangle(
                (int)position.X,
                (int)position.Y,
                (int)tooltipSize.X + Visual.Padding * 2,
                (int)tooltipSize.Y + Visual.Padding);

            _spriteBatch.Draw(_textures.pixel, tooltipRect, Color.Black * 0.8f);
            _fonts.standard.DrawText(_spriteBatch, text,
                position + new Vector2(Visual.Padding, Visual.Padding / 2),
                Color.White);
        }

        public void DrawButton(UIButton button)
        {
            if (!button.IsVisible)
                return;

            Color buttonColor = button.IsHovered ? button.HoverColor : button.BaseColor;

            Rectangle scaledBounds = new Rectangle(
                (int)(button.Bounds.X - button.Bounds.Width * (button.Scale - 1) / 2),
                (int)(button.Bounds.Y - button.Bounds.Height * (button.Scale - 1) / 2),
                (int)(button.Bounds.Width * button.Scale),
                (int)(button.Bounds.Height * button.Scale)
            );

            _spriteBatch.Draw(_textures.pixel,
                new Rectangle(scaledBounds.X + 4, scaledBounds.Y + 4, scaledBounds.Width, scaledBounds.Height),
                Color.Black * 0.3f);

            _spriteBatch.Draw(_textures.pixel, scaledBounds, buttonColor);
            DrawBorder(scaledBounds, Color.White, Visual.BorderThin);

            if (!string.IsNullOrEmpty(button.Text))
            {
                Vector2 textSize = _fonts.standard.MeasureString(button.Text);
                Vector2 textPosition = new Vector2(
                    scaledBounds.X + (scaledBounds.Width - textSize.X) / 2,
                    scaledBounds.Y + (scaledBounds.Height - textSize.Y) / 2
                );

                _fonts.standard.DrawText(_spriteBatch, button.Text,
                    textPosition + new Vector2(1, 1),
                    Color.Black * 0.5f);

                _fonts.standard.DrawText(_spriteBatch, button.Text, textPosition, Color.White);
            }
        }

        public void DrawBorder(Rectangle rectangle, Color color, int thickness)
        {
            _spriteBatch.Draw(_textures.pixel, new Rectangle(rectangle.X, rectangle.Y, rectangle.Width, thickness), color);
            _spriteBatch.Draw(_textures.pixel, new Rectangle(rectangle.X, rectangle.Y + rectangle.Height - thickness, rectangle.Width, thickness), color);
            _spriteBatch.Draw(_textures.pixel, new Rectangle(rectangle.X, rectangle.Y, thickness, rectangle.Height), color);
            _spriteBatch.Draw(_textures.pixel, new Rectangle(rectangle.X + rectangle.Width - thickness, rectangle.Y, thickness, rectangle.Height), color);
        }

        private Rectangle CreateMenuRect(float widthRatio = Layout.MenuWidthStandard,
                                         float heightRatio = Layout.MenuHeightStandard) =>
            new(
                _screenSize.width / 2 - (int)(_screenSize.width * widthRatio / 2),
                _screenSize.height / 2 - (int)(_screenSize.height * heightRatio / 2),
                (int)(_screenSize.width * widthRatio),
                (int)(_screenSize.height * heightRatio));

        private void DrawTitle(string title, int yPosition, Color color)
        {
            Vector2 titleSize = _fonts.title.MeasureString(title);
            float x = (_screenSize.width - titleSize.X) / 2;

            _fonts.title.DrawText(_spriteBatch, title,
                new Vector2(x + Visual.ShadowOffset, yPosition + Visual.ShadowOffset),
                Color.Black * Visual.ShadowOpacity);
            _fonts.title.DrawText(_spriteBatch, title, new Vector2(x, yPosition), color);
        }

        private void DrawMenuBase(
            string title,
            Color borderColor,
            Action<Rectangle>? drawContent = null,
            float widthRatio = Layout.MenuWidthStandard,
            float heightRatio = Layout.MenuHeightStandard)
        {
            Rectangle menuRect = CreateMenuRect(widthRatio, heightRatio);

            _spriteBatch.Draw(_textures.pixel, menuRect, new Color(20, 20, 20, 220));
            DrawBorder(menuRect, borderColor, Visual.BorderThick);

            DrawTitle(title, menuRect.Y + Visual.TitleOffset, borderColor);

            drawContent?.Invoke(menuRect);
        }

        public void DrawMainMenu() =>
            DrawMenuBase("GRAVITY DEFIED", Color.Goldenrod, menuRect =>
            {
                string subtitle = "A Motorcycle Physics Adventure";
                Vector2 subtitleSize = _fonts.standard.MeasureString(subtitle);
                float x = (_screenSize.width - subtitleSize.X) / 2;

                _fonts.standard.DrawText(_spriteBatch, subtitle,
                    new Vector2(x + 1, menuRect.Y + Visual.SubtitleOffset + 1),
                    Color.Black * Visual.ShadowOpacity);
                _fonts.standard.DrawText(_spriteBatch, subtitle,
                    new Vector2(x, menuRect.Y + Visual.SubtitleOffset),
                    Color.LightGoldenrodYellow);

                string themeText = $"Current Theme: {CurrentTheme.Name}";
                Vector2 themeSize = _fonts.standard.MeasureString(themeText);
                _fonts.standard.DrawText(_spriteBatch, themeText,
                    new Vector2((_screenSize.width - themeSize.X) / 2, menuRect.Y + 160),
                    Color.LightGoldenrodYellow);
            });

        public void DrawBikeSelectionMenu(int bikeIndex, int colorIndex, (Color color, string name)[] bikeColors) =>
            DrawMenuBase("SELECT YOUR BIKE", Color.Cyan, menuRect =>
            {
                var bikes = Enum.GetValues<BikeType>();
                string bikeTypeText = bikes[bikeIndex].ToString();
                Vector2 bikeTypeSize = _fonts.standard.MeasureString(bikeTypeText);
                float x = (_screenSize.width - bikeTypeSize.X) / 2;

                _fonts.standard.DrawText(_spriteBatch, bikeTypeText,
                    new Vector2(x + 1, menuRect.Y + Visual.MainTextOffset + 1),
                    Color.Black * Visual.ShadowOpacity);
                _fonts.standard.DrawText(_spriteBatch, bikeTypeText,
                    new Vector2(x, menuRect.Y + Visual.MainTextOffset),
                    bikeColors[colorIndex].color);

                string colorText = $"Color: {bikeColors[colorIndex].name}";
                Vector2 colorTextSize = _fonts.standard.MeasureString(colorText);
                _fonts.standard.DrawText(_spriteBatch, colorText,
                    new Vector2((_screenSize.width - colorTextSize.X) / 2, menuRect.Y + Visual.MainTextOffset + 40),
                    bikeColors[colorIndex].color);

                DrawBikePreview(menuRect, bikeIndex);
            }, Layout.MenuWidthWide);

        private void DrawBikePreview(Rectangle menuRect, int bikeIndex)
        {
            int previewWidth = (int)(menuRect.Width * 0.4f);
            int previewHeight = (int)(menuRect.Height * 0.3f);
            int previewX = menuRect.X + (menuRect.Width - previewWidth) / 2;
            int previewY = menuRect.Y + (int)(menuRect.Height * 0.3f);

            Rectangle previewRect = new Rectangle(previewX, previewY, previewWidth, previewHeight);
            _spriteBatch.Draw(_textures.pixel, previewRect, new Color(30, 30, 30, 200));
            DrawBorder(previewRect, Color.LightGray, Visual.BorderThin);

            string previewText = "Bike Preview";
            Vector2 previewTextSize = _fonts.standard.MeasureString(previewText);
            _fonts.standard.DrawText(_spriteBatch, previewText,
                new Vector2(previewX + (previewWidth - previewTextSize.X) / 2, previewY + 10),
                Color.LightGray);

            var bikeType = Enum.GetValues<BikeType>()[bikeIndex];
            DrawBikeStats(previewX + 20, previewY + 40, bikeType);
        }

        private void DrawBikeStats(int x, int y, BikeType bikeType)
        {
            (string label, int value)[] stats = bikeType switch
            {
                BikeType.Standard => new[] { ("Speed", 3), ("Handling", 3), ("Stability", 3) },
                BikeType.Sport => new[] { ("Speed", 5), ("Handling", 2), ("Stability", 2) },
                BikeType.OffRoad => new[] { ("Speed", 2), ("Handling", 4), ("Stability", 5) },
                _ => new[] { ("Speed", 3), ("Handling", 3), ("Stability", 3) }
            };

            for (int i = 0; i < stats.Length; i++)
            {
                var (label, value) = stats[i];
                string statText = $"{label}: {new string('|', value)}{new string('.', 5 - value)}";
                _fonts.standard.DrawText(_spriteBatch, statText, new Vector2(x, y + i * 30), Color.Yellow);
            }
        }

        public void DrawLevelSelectionMenu(int levelIndex, List<Models.Level> levels) =>
            DrawMenuBase("SELECT LEVEL", Color.Magenta, menuRect =>
            {
                string levelText = $"Level {levelIndex + 1}: {levels[levelIndex].Name}";
                Vector2 levelSize = _fonts.standard.MeasureString(levelText);
                float x = (_screenSize.width - levelSize.X) / 2;

                _fonts.standard.DrawText(_spriteBatch, levelText,
                    new Vector2(x + 1, menuRect.Y + Visual.MainTextOffset + 1),
                    Color.Black * Visual.ShadowOpacity);
                _fonts.standard.DrawText(_spriteBatch, levelText,
                    new Vector2(x, menuRect.Y + Visual.MainTextOffset),
                    Color.Yellow);

                string difficultyText = $"Difficulty: {GetLevelDifficulty(levelIndex + 1)}";
                Vector2 difficultyPos = new Vector2(
                    (_screenSize.width - _fonts.standard.MeasureString(difficultyText).X) / 2,
                    menuRect.Y + 150);
                _fonts.standard.DrawText(_spriteBatch, difficultyText, difficultyPos, Color.Yellow);
            }, Layout.MenuWidthWide);

        private static string GetLevelDifficulty(int levelId) => levelId switch
        {
            <= 5 => "[|..]",
            <= 15 => "[||.]",
            _ => "[|||]"
        };

        public void DrawThemeSelectionMenu(int themeIndex, List<ThemeSettings> themes) =>
            DrawMenuBase("SELECT THEME", Color.Purple, menuRect =>
            {
                string themeName = themes[themeIndex].Name;
                Vector2 themeNameSize = _fonts.standard.MeasureString(themeName);

                _fonts.standard.DrawText(_spriteBatch, themeName,
                    new Vector2((_screenSize.width - themeNameSize.X) / 2 + 1, menuRect.Y + 100 + 1),
                    Color.Black * Visual.ShadowOpacity);
                _fonts.standard.DrawText(_spriteBatch, themeName,
                    new Vector2((_screenSize.width - themeNameSize.X) / 2, menuRect.Y + 100),
                    Color.White);

                string themeDesc = themes[themeIndex].Description;
                Vector2 descSize = _fonts.standard.MeasureString(themeDesc);
                _fonts.standard.DrawText(_spriteBatch, themeDesc,
                    new Vector2((_screenSize.width - descSize.X) / 2, menuRect.Y + 140),
                    Color.LightGray);

                DrawThemePreview(menuRect, themes[themeIndex]);
            });

        private void DrawThemePreview(Rectangle menuRect, ThemeSettings theme)
        {
            int previewY = menuRect.Y + 180;
            int swatchSize = 30;
            int margin = 10;
            int totalWidth = 6 * (swatchSize + margin);
            int startX = (_screenSize.width - totalWidth) / 2;

            DrawColorSwatch(startX, previewY, swatchSize, theme.BackgroundColor, "BG");
            DrawColorSwatch(startX + swatchSize + margin, previewY, swatchSize, theme.TerrainColor, "Terrain");
            DrawColorSwatch(startX + 2 * (swatchSize + margin), previewY, swatchSize, theme.SafeZoneColor, "Safe");
            DrawColorSwatch(startX + 3 * (swatchSize + margin), previewY, swatchSize, theme.BikeColors[SkeletonLineType.MainFrame], "Bike");
            DrawColorSwatch(startX + 4 * (swatchSize + margin), previewY, swatchSize, theme.WheelFill, "Wheel");
            DrawColorSwatch(startX + 5 * (swatchSize + margin), previewY, swatchSize, theme.ShadowColor, "Shadow");
        }

        private void DrawColorSwatch(int x, int y, int size, Color color, string label)
        {
            _spriteBatch.Draw(_textures.pixel, new Rectangle(x, y, size, size), color);
            _fonts.standard.DrawText(_spriteBatch, label, new Vector2(x, y + size + 2), Color.LightGray);
        }

        public void DrawInfoPanel(string? levelName, TimeSpan gameTime, int direction)
        {
            string levelNameText = levelName ?? "Level";
            string timeText = $"Time: {gameTime.Minutes:00}:{gameTime.Seconds:00}";
            string directionText = direction == 1 ? "Forward" : "Backward";

            Rectangle infoPanelRect = new Rectangle(10, 10, (int)(_screenSize.width * 0.25f), (int)(_screenSize.height * 0.15f));
            _spriteBatch.Draw(_textures.pixel, infoPanelRect, new Color(0, 0, 0, 150));
            DrawBorder(infoPanelRect, Color.LightBlue, Visual.BorderThin);

            _fonts.standard.DrawText(_spriteBatch, $"Level: {levelNameText}", new Vector2(20, 20), Color.White);
            _fonts.standard.DrawText(_spriteBatch, timeText, new Vector2(20, 45), Color.White);
            _fonts.standard.DrawText(_spriteBatch, $"Direction: {directionText}", new Vector2(20, 70), Color.White);

            string controlsText = "W: Forward | S: Backward | Space: Brake | A/D: Lean | ESC: Pause";
            Vector2 controlsSize = _fonts.standard.MeasureString(controlsText);
            _fonts.standard.DrawText(_spriteBatch, controlsText,
                new Vector2((_screenSize.width - controlsSize.X) / 2, _screenSize.height - 40),
                Color.LightBlue);
        }

        public void DrawPauseMenu(string? levelName, TimeSpan gameTime) =>
            DrawMenuBase("GAME PAUSED", Color.LightBlue, menuRect =>
            {
                string resumeText = "Press ESC to Resume";
                Vector2 resumeSize = _fonts.standard.MeasureString(resumeText);
                _fonts.standard.DrawText(_spriteBatch, resumeText,
                    new Vector2((_screenSize.width - resumeSize.X) / 2, menuRect.Y + 120),
                    Color.White);

                string levelText = $"Level: {levelName ?? "Unknown"}";
                string timeText = $"Time: {gameTime.Minutes:00}:{gameTime.Seconds:00}";
                _fonts.standard.DrawText(_spriteBatch, levelText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.6f)),
                    Color.LightGray);
                _fonts.standard.DrawText(_spriteBatch, timeText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.6f) + 25),
                    Color.LightGray);
            });

        public void DrawGameOverMenu(string? levelName, TimeSpan gameTime) =>
            DrawMenuBase("GAME OVER", Color.Red, menuRect =>
            {
                string levelText = $"Level: {levelName ?? "Unknown"}";
                string timeText = $"Time: {gameTime.Minutes:00}:{gameTime.Seconds:00}";
                _fonts.standard.DrawText(_spriteBatch, levelText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.4f)),
                    Color.LightGray);
                _fonts.standard.DrawText(_spriteBatch, timeText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.4f) + 25),
                    Color.LightGray);
            });

        public void DrawLevelCompleteMenu(string? levelName, TimeSpan gameTime) =>
            DrawMenuBase("LEVEL COMPLETE!", Color.LimeGreen, menuRect =>
            {
                string timeText = $"Time: {gameTime.Minutes:00}:{gameTime.Seconds:00}";
                Vector2 timeSize = _fonts.standard.MeasureString(timeText);
                _fonts.standard.DrawText(_spriteBatch, timeText,
                    new Vector2((_screenSize.width - timeSize.X) / 2, menuRect.Y + 100),
                    Color.White);

                string levelText = $"Level: {levelName ?? "Unknown"}";
                _fonts.standard.DrawText(_spriteBatch, levelText,
                    new Vector2(menuRect.X + 30, menuRect.Y + (int)(menuRect.Height * 0.6f)),
                    Color.LightGray);

                string nextLevelText = "Press Continue to proceed to the next level";
                Vector2 nextLevelSize = _fonts.standard.MeasureString(nextLevelText);
                _fonts.standard.DrawText(_spriteBatch, nextLevelText,
                    new Vector2((_screenSize.width - nextLevelSize.X) / 2, menuRect.Y + (int)(menuRect.Height * 0.7f)),
                    Color.LightGray);
            });
    }

    public class UISmoke
    {
        public Vector2 Position { get; set; }
        public Vector2 Velocity { get; set; }
        public float Size { get; set; }
        public float Opacity { get; set; }
        public float MaxOpacity { get; set; }
        public float LifeTime { get; set; }
        public float MaxLifeTime { get; set; }
        public Color BaseColor { get; set; }
        public int SeedValue { get; set; }
        public float NoiseOffset { get; set; }
        public SmokeLayer Layer { get; set; }
        public List<Vector2> CloudNodes { get; set; } = [];
        public float Turbulence { get; set; }
        public float WaveFrequency { get; set; }
        public float DensityVariation { get; set; }

        public UISmoke(Random random, int screenWidth, int screenHeight, SmokeLayer layer)
        {
            Layer = layer;
            SeedValue = random.Next(1000);
            NoiseOffset = (float)random.NextDouble() * 10f;

            (Position, Size, MaxOpacity, BaseColor, Velocity, Turbulence, WaveFrequency, DensityVariation) = layer switch
            {
                SmokeLayer.Lower => (
                    new Vector2(random.Next(-100, screenWidth + 100), random.Next(screenHeight - 150, screenHeight + 50)),
                    random.Next(30, 60),
                    0.6f + (float)random.NextDouble() * 0.2f,
                    new Color(224, 240, 255),
                    new Vector2(((float)random.NextDouble() - 0.3f) * 10f, -((float)random.NextDouble() * 3f + 1f)),
                    0.8f + (float)random.NextDouble() * 0.4f,
                    0.5f + (float)random.NextDouble() * 0.3f,
                    0.7f + (float)random.NextDouble() * 0.3f
                ),
                SmokeLayer.Middle => (
                    new Vector2(random.Next(-100, screenWidth + 100), random.Next(screenHeight / 2 - 100, screenHeight / 2 + 100)),
                    random.Next(20, 45),
                    0.5f + (float)random.NextDouble() * 0.2f,
                    new Color(232, 248, 255),
                    new Vector2(((float)random.NextDouble() + 0.1f) * 15f, ((float)random.NextDouble() - 0.5f) * 2f),
                    0.5f + (float)random.NextDouble() * 0.3f,
                    0.8f + (float)random.NextDouble() * 0.4f,
                    0.5f + (float)random.NextDouble() * 0.3f
                ),
                _ => (
                    new Vector2(random.Next(-100, screenWidth + 100), random.Next(0, screenHeight / 3)),
                    random.Next(15, 40),
                    0.4f + (float)random.NextDouble() * 0.2f,
                    new Color(240, 224, 255),
                    new Vector2(((float)random.NextDouble() - 0.5f) * 8f, ((float)random.NextDouble() - 0.5f) * 1.5f),
                    0.3f + (float)random.NextDouble() * 0.2f,
                    0.3f + (float)random.NextDouble() * 0.3f,
                    0.3f + (float)random.NextDouble() * 0.2f
                )
            };

            GenerateCloudNodes(random);

            Opacity = 0;
            MaxLifeTime = Animation.SmokeParticleLifespan + (float)random.NextDouble() * 2f;
            LifeTime = (float)random.NextDouble() * MaxLifeTime;
        }

        private void GenerateCloudNodes(Random random)
        {
            CloudNodes.Clear();

            int nodeCount = Layer switch
            {
                SmokeLayer.Lower => random.Next(6, 10),
                SmokeLayer.Middle => random.Next(4, 7),
                _ => random.Next(3, 5)
            };

            float baseRadius = Size * 0.5f;

            for (int i = 0; i < nodeCount; i++)
            {
                if (Layer == SmokeLayer.Middle)
                {
                    float angle = (float)i / nodeCount * MathHelper.TwoPi;
                    float distance = baseRadius * (0.6f + (float)random.NextDouble() * 0.7f);
                    float x = (float)Math.Cos(angle) * distance * (1.3f + (float)random.NextDouble() * 0.4f);
                    float y = (float)Math.Sin(angle) * distance * (0.7f + (float)random.NextDouble() * 0.3f);
                    CloudNodes.Add(new Vector2(x, y));
                }
                else
                {
                    float angle = (float)i / nodeCount * MathHelper.TwoPi +
                                  (float)(random.NextDouble() * (Layer == SmokeLayer.Lower ? 0.5f : 0.3f) -
                                         (Layer == SmokeLayer.Lower ? 0.25f : 0.15f));
                    float distance = baseRadius * (Layer == SmokeLayer.Lower ?
                                                 (0.5f + (float)random.NextDouble() * 0.8f) :
                                                 (0.7f + (float)random.NextDouble() * 0.6f));
                    CloudNodes.Add(new Vector2((float)Math.Cos(angle) * distance, (float)Math.Sin(angle) * distance));
                }
            }

            int extraNodes = Layer switch
            {
                SmokeLayer.Lower => random.Next(3, 6),
                SmokeLayer.Middle => random.Next(2, 4),
                _ => random.Next(1, 3)
            };

            for (int i = 0; i < extraNodes; i++)
            {
                float angle = (float)random.NextDouble() * MathHelper.TwoPi;
                float distance = baseRadius * (0.3f + (float)random.NextDouble() * 0.5f);
                CloudNodes.Add(new Vector2((float)Math.Cos(angle) * distance, (float)Math.Sin(angle) * distance));
            }
        }

        public void Update(float elapsedTime, UIPerlinNoise noise, int screenWidth)
        {
            LifeTime += elapsedTime;

            Opacity = LifeTime < MaxLifeTime * 0.3f
                ? MathHelper.Lerp(0, MaxOpacity, LifeTime / (MaxLifeTime * 0.3f))
                : LifeTime > MaxLifeTime * 0.7f
                    ? MathHelper.Lerp(MaxOpacity, 0, (LifeTime - MaxLifeTime * 0.7f) / (MaxLifeTime * 0.3f))
                    : MaxOpacity;

            float time = LifeTime * 0.5f + NoiseOffset;
            float noiseX = (float)noise.Noise(Position.X * 0.01f, Position.Y * 0.01f, time) * 2f;
            float noiseY = (float)noise.Noise(Position.X * 0.01f + 100, Position.Y * 0.01f + 100, time) * 2f;
            Vector2 noiseVelocity = new(noiseX, noiseY);

            float multiplier = Layer switch
            {
                SmokeLayer.Lower => 3f,
                SmokeLayer.Middle => 2f,
                _ => 1.5f
            };

            Position += Velocity * elapsedTime + noiseVelocity * multiplier * elapsedTime * Turbulence;

            if (Position.X < -Size * 2)
                Position = new(screenWidth + Size, Position.Y);
            else if (Position.X > screenWidth + Size * 2)
                Position = new(-Size, Position.Y);

            if (Layer == SmokeLayer.Middle)
                Position = new(Position.X, Position.Y + (float)Math.Sin(time * WaveFrequency) * 0.5f);
        }

        public bool IsExpired() => LifeTime >= MaxLifeTime;

        public void Draw(SpriteBatch spriteBatch, Texture2D pixelTexture)
        {
            if (Opacity <= 0.01f)
                return;

            int pixelSize = 3;
            int gridSize = (int)(Size / pixelSize);
            float halfGrid = gridSize / 2f;

            for (int y = 0; y < gridSize; y += 2)
            {
                for (int x = 0; x < gridSize; x += 2)
                {
                    float nx = (x - halfGrid) / halfGrid;
                    float ny = (y - halfGrid) / halfGrid;
                    float distFromCenter = Vector2.Distance(new Vector2(nx, ny), Vector2.Zero);

                    if (distFromCenter > 1.2f)
                        continue;

                    float influence = 0;
                    foreach (var node in CloudNodes)
                    {
                        float nodeDist = Vector2.Distance(new Vector2(nx, ny), node / (Size * 0.5f));
                        influence += Math.Max(0, 1 - nodeDist * 1.2f);
                    }

                    influence = MathHelper.Clamp(influence * 0.7f, 0, 1);
                    if (influence < 0.1f)
                        continue;

                    float noiseValue = SimplexNoise(nx * 3f + SeedValue, ny * 3f + SeedValue, LifeTime * 0.1f);
                    float density = influence * (0.7f + noiseValue * 0.3f * DensityVariation);
                    float alpha = density * Opacity;

                    if (alpha < 0.05f)
                        continue;

                    Color pixelColor = new(
                        (byte)MathHelper.Clamp(BaseColor.R + (noiseValue * 15), 220, 255),
                        (byte)MathHelper.Clamp(BaseColor.G + (noiseValue * 15), 225, 255),
                        (byte)MathHelper.Clamp(BaseColor.B + (noiseValue * 15), 235, 255),
                        (byte)(alpha * 255));

                    spriteBatch.Draw(
                        pixelTexture,
                        new Rectangle(
                            (int)(Position.X - Size / 2 + x * pixelSize),
                            (int)(Position.Y - Size / 2 + y * pixelSize),
                            pixelSize * 2,
                            pixelSize * 2),
                        pixelColor);
                }
            }
        }

        private float SimplexNoise(float x, float y, float z) =>
            (float)((Math.Sin(x * 12.9898f + y * 78.233f + z * 37.719f) * 43758.5453f) % 1) * 2 - 1;
    }

    public class UIPerlinNoise
    {
        private const int GradientSizeTable = 256;
        private readonly Random _random;
        private readonly double[] _gradients = new double[GradientSizeTable * 3];
        private readonly byte[] _perm = new byte[GradientSizeTable * 2];

        public UIPerlinNoise(int seed = 0)
        {
            _random = new Random(seed);
            InitGradients();
        }

        private void InitGradients()
        {
            for (int i = 0; i < GradientSizeTable; i++)
            {
                double z = 1f - 2f * _random.NextDouble();
                double r = Math.Sqrt(1f - z * z);
                double theta = 2 * Math.PI * _random.NextDouble();
                _gradients[i * 3] = r * Math.Cos(theta);
                _gradients[i * 3 + 1] = r * Math.Sin(theta);
                _gradients[i * 3 + 2] = z;
            }

            for (int i = 0; i < GradientSizeTable; i++)
                _perm[i] = (byte)i;

            for (int i = 0; i < GradientSizeTable; i++)
            {
                int j = _random.Next() & (GradientSizeTable - 1);
                (_perm[i], _perm[j]) = (_perm[j], _perm[i]);
            }

            for (int i = 0; i < GradientSizeTable; i++)
                _perm[GradientSizeTable + i] = _perm[i];
        }

        private static double Lerp(double a, double b, double t) =>
            a + t * (b - a);

        private static double Fade(double t) =>
            t * t * t * (t * (t * 6 - 15) + 10);

        private double Grad(int hash, double x, double y, double z)
        {
            int h = hash & 15;
            double u = h < 8 ? x : y;
            double v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
            return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
        }

        public double Noise(double x, double y, double z)
        {
            int ix = (int)Math.Floor(x) & (GradientSizeTable - 1);
            int iy = (int)Math.Floor(y) & (GradientSizeTable - 1);
            int iz = (int)Math.Floor(z) & (GradientSizeTable - 1);

            double fx = x - Math.Floor(x);
            double fy = y - Math.Floor(y);
            double fz = z - Math.Floor(z);

            double u = Fade(fx);
            double v = Fade(fy);
            double w = Fade(fz);

            int a = _perm[ix] + iy;
            int aa = _perm[a] + iz;
            int ab = _perm[a + 1] + iz;
            int b = _perm[ix + 1] + iy;
            int ba = _perm[b] + iz;
            int bb = _perm[b + 1] + iz;

            double x1 = Lerp(
                            Grad(_perm[aa], fx, fy, fz),
                            Grad(_perm[ba], fx - 1, fy, fz),
                            u);
            double x2 = Lerp(
                Grad(_perm[ab], fx, fy - 1, fz),
                Grad(_perm[bb], fx - 1, fy - 1, fz),
                u);
            double y1 = Lerp(x1, x2, v);

            x1 = Lerp(
                Grad(_perm[aa + 1], fx, fy, fz - 1),
                Grad(_perm[ba + 1], fx - 1, fy, fz - 1),
                u);
            x2 = Lerp(
                Grad(_perm[ab + 1], fx, fy - 1, fz - 1),
                Grad(_perm[bb + 1], fx - 1, fy - 1, fz - 1),
                u);
            double y2 = Lerp(x1, x2, v);

            return (Lerp(y1, y2, w) + 1) / 2;
        }
    }
}