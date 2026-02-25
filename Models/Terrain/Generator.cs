using static System.MathF;
using Vec2 = Microsoft.Xna.Framework.Vector2;
using NL = FastNoiseLite;

namespace GravityDefiedGame.Models.Terrain;

public static class Generator
{
    public static Level Generate(
        int id,
        string name,
        int? seed = null,
        int? diff = null)
    {
        int d = Math.Max(diff ?? id, 1);
        Cfg c = new(d);
        Noise n = new(d, seed ?? Random.Shared.Next());

        float[] px = new float[c.Pts],
                py = new float[c.Pts],
                nx = new float[c.Segs],
                ny = new float[c.Segs];

        FillHeights(px, py, c, n);
        SmoothSlopes(py, c.MaxDy);
        ClampBounds(py, c.MinY, c.MaxY);
        FillNormals(px, py, nx, ny);

        return new Level(
            id, name, d, c.Len,
            px, py, nx, ny,
            Cfg.StartX,
            new Vec2(c.FinishX, Cfg.FinishY),
            c.StartIdx, c.EndIdx);
    }

    static void FillHeights(float[] px, float[] py, in Cfg c, Noise n)
    {
        for (int i = 0; i < px.Length; i++)
        {
            float x = Min(i * c.Dx, c.Len);
            px[i] = x;
            py[i] = Cfg.H + n.Sample(x) * Fade(x, c);
        }
    }

    static void FillNormals(float[] px, float[] py, float[] nx, float[] ny)
    {
        for (int i = 0; i < nx.Length; i++)
        {
            float dx = px[i + 1] - px[i],
                  dy = py[i + 1] - py[i],
                  inv = InvLen(dx, dy) * 2f;

            (nx[i], ny[i]) = (-dy * inv, dx * inv);

            if (ny[i] > 0)
                (nx[i], ny[i]) = (-nx[i], -ny[i]);
        }
    }

    static void SmoothSlopes(float[] y, float maxDy)
    {
        for (int i = 1; i < y.Length; i++)
        {
            float dy = y[i] - y[i - 1];
            if (Abs(dy) > maxDy)
                y[i] = y[i - 1] + Sign(dy) * maxDy;
        }

        for (int i = y.Length - 2; i >= 0; i--)
        {
            float dy = y[i] - y[i + 1];
            if (Abs(dy) > maxDy)
                y[i] = y[i + 1] + Sign(dy) * maxDy;
        }
    }

    static void ClampBounds(float[] y, float lo, float hi)
    {
        for (int i = 0; i < y.Length; i++)
        {
            if (y[i] < lo)
                y[i] = lo;
            else if (y[i] > hi)
                y[i] = hi;
        }
    }

    static float Fade(float x, in Cfg c) =>
        Smooth((x - Cfg.Safe) / Cfg.FadeW) *
        Smooth((c.Len - Cfg.Safe - x) / Cfg.FadeW);

    static float Smooth(float t) =>
        t <= 0f ? 0f :
        t >= 1f ? 1f :
        t * t * (3f - 2f * t);

    static float InvLen(float x, float y) =>
        Sqrt(x * x + y * y) is float l and > 1e-6f ? 1f / l : 0f;

    readonly record struct Cfg
    {
        static readonly LevelConst L = LevelConst.Default;

        public static float H => L.TerrainH;
        public static float Safe => L.SafeZone;
        public static float FadeW => Safe * 0.5f;
        public static float StartX => L.StartX;
        public static float FinishY => H + L.EndY;

        public float Len { get; }
        public int Segs { get; }
        public float Dx { get; }
        public float Range { get; }
        public float MaxDy { get; }

        public int Pts => Segs + 1;
        public float MinY => H - Range;
        public float MaxY => H + Range;
        public float FinishX => Len + L.EndX;
        public int StartIdx => Math.Clamp((int)(StartX / Dx), 0, Segs - 1);
        public int EndIdx => Math.Clamp((int)(FinishX / Dx), 0, Segs - 1);

        public Cfg(int d)
        {
            Len = Min(L.BaseLen + L.LenPerLvl * d, L.MaxLen);
            Segs = Math.Max((int)Ceiling(Len / 15f), 2);
            Dx = Len / Segs;
            Range = Min(340f + 24f * d, 520f);
            MaxDy = Min(1.2f + 0.02f * d, 1.4f) * Dx;
        }
    }

    sealed class Noise(int d, int seed)
    {
        readonly float _macroAmp = Min(180f + 30f * d, 400f);
        readonly float _detailAmp = Min(30f + 8f * d, 90f);

        readonly NL _macro = Cellular(seed,
            freq: 0.001f + 0.0001f * d,
            ret: NL.CellularReturnType.Distance2Div,
            dist: NL.CellularDistanceFunction.EuclideanSq,
            jitter: 1.0f + 0.05f * d,
            oct: 3);

        readonly NL _detail = Ridged(seed + 1,
            freq: 0.008f + 0.001f * d,
            oct: 4, lac: 2.2f,
            gain: 0.45f, weight: 0.3f);

        readonly NL _warp = Warp(seed + 2,
            freq: 0.003f,
            amp: Min(25f + 4f * d, 60f),
            oct: 2);

        public float Sample(float x)
        {
            float y = 0f;
            _warp.DomainWarp(ref x, ref y);

            float macro = _macro.GetNoise(x, 0f) * 2f + 1f;
            float detail = _detail.GetNoise(x, 0f);

            return macro * _macroAmp + detail * _detailAmp;
        }

        static NL Base(int seed, NL.NoiseType type, float freq)
        {
            NL n = new(seed);
            n.SetNoiseType(type);
            n.SetFrequency(freq);
            return n;
        }

        static NL Cellular(
            int seed, float freq,
            NL.CellularReturnType ret,
            NL.CellularDistanceFunction dist,
            float jitter, int oct)
        {
            NL n = Base(seed, NL.NoiseType.Cellular, freq);
            n.SetCellularReturnType(ret);
            n.SetCellularDistanceFunction(dist);
            n.SetCellularJitter(jitter);
            n.SetFractalType(NL.FractalType.FBm);
            n.SetFractalOctaves(oct);
            return n;
        }

        static NL Ridged(
            int seed, float freq, int oct,
            float lac, float gain, float weight)
        {
            NL n = Base(seed, NL.NoiseType.OpenSimplex2S, freq);
            n.SetFractalType(NL.FractalType.Ridged);
            n.SetFractalOctaves(oct);
            n.SetFractalLacunarity(lac);
            n.SetFractalGain(gain);
            n.SetFractalWeightedStrength(weight);
            return n;
        }

        static NL Warp(
            int seed, float freq,
            float amp, int oct)
        {
            NL n = Base(seed, NL.NoiseType.OpenSimplex2S, freq);
            n.SetDomainWarpType(NL.DomainWarpType.OpenSimplex2Reduced);
            n.SetDomainWarpAmp(amp);
            n.SetFractalType(NL.FractalType.DomainWarpProgressive);
            n.SetFractalOctaves(oct);
            return n;
        }
    }
}
