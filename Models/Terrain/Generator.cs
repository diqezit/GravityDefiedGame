using static System.MathF;
using XVector2 = Microsoft.Xna.Framework.Vector2;

namespace GravityDefiedGame.Models.Terrain;

// Procedural terrain generator
// Idea: build a smooth "macro skeleton" first, then add medium + micro detail,
// and finally enforce hard gameplay/physics constraints so BikePhysics never explodes
public static class Generator
{
    static readonly LevelConst L = LevelConst.Default;

    // Bike safety constraints
    // WheelSpan: how far the wheels are apart in pixels
    // MaxWheelDy: max allowed height difference between those wheels
    // FadeK: how quickly terrain returns to flat near start/end safe zones
    const float WheelSpan = 42f, MaxWheelDy = 27f, FadeK = 0.45f;

    // Segment count limits (controls memory + CPU)
    const int MinSegs = 96, MaxSegs = 2600;

    // Control points buffer cap for the macro skeleton
    // We keep it small because typical levels need only ~20-40 points
    const int MaxCtrlPts = 64;

    // Rest mask remapping thresholds
    // Noise is remapped into a "difficulty intensity" in 0..1
    // Lower than RestLo -> mostly rest
    // Higher than RestHi -> full fight
    const float RestLo = 0.35f, RestHi = 0.62f;

    // Multipliers used in rest zones
    // These values intentionally do NOT go to zero to keep some motion
    const float MacroK = 0.20f, MedK = 0.08f, DetailK = 0.05f;

    // Control points slope smoothing
    // Inertia makes the macro skeleton less jittery between control points
    const float Inertia = 0.32f, InertiaBoost = 0.22f;

    // Noise layers pack
    // We keep them grouped so we don't pass 5 params everywhere
    readonly record struct Noises(
        FastNoiseLite Macro, // large-scale terrain direction (used in control points)
        FastNoiseLite Rest,  // mask: 0 rest, 1 fight
        FastNoiseLite Warp,  // domain warp: breaks periodicity and obvious patterns
        FastNoiseLite Med,   // medium bumps
        FastNoiseLite Mic);  // micro roughness

    // Difficulty parameters derived from diff
    // All "base + diff*scale then clamp" values are collected here
    readonly record struct Params(
        float YRange,    // soft max excursion from base height
        float MedAmp,    // medium amplitude
        float MicAmp,    // micro amplitude
        float MaxSlope,  // per-segment slope clamp (dy per dx)
        float StepMin,   // control points step min
        float StepMax,   // control points step max
        float Deadzone,  // minimal slope magnitude to avoid long flats
        float SlopeMax,  // max macro slope magnitude
        float CtrlRange);// soft clamp for control points Y

    // Public API
    // seed: optional deterministic generation
    // diff: if not provided, uses id as difficulty
    public static Level Generate(int id, string name, int? seed = null, int? diff = null)
    {
        int d = Math.Max(diff ?? id, 1);

        // Simple seed rule
        // We avoid extra "magic seed mixing" to keep code easy to reason about
        int s = seed ?? (Environment.TickCount ^ id);

        float len = Math.Clamp(L.BaseLen + L.LenPerLvl * d, 1f, L.MaxLen);

        return Build(id, name, s, d, len);
    }

    static Level Build(int id, string name, int seed, int diff, float len)
    {
        Params p = GetParams(diff);
        Noises n = MakeNoises(seed, diff);

        float dx = GetDx(diff, len, out int segs);
        float invDx = segs / len;

        float[] xs = new float[segs + 1];
        float[] ys = new float[segs + 1];
        float[] nx = new float[segs];
        float[] ny = new float[segs];

        FillX(xs, dx, len);

        // RNG is only used to build control points spacing
        // Terrain itself is driven by FastNoiseLite for reproducibility and "natural" look
        uint rng = Rng((uint)seed);

        MakeCtrl(len, p, ref rng, n, out float[] cx, out float[] cy);

        Synth(xs, ys, cx, cy, n, p, len);

        // Post constraints are critical
        // They are the difference between "nice terrain" and "playable terrain"
        Clamp(xs, ys, segs, len, p, dx);

        Normals(segs, xs, ys, nx, ny);

        int startI = Math.Clamp((int)(L.StartX * invDx), 0, segs - 1);
        int endI = Math.Clamp((int)((len + L.EndX) * invDx), 0, segs - 1);

        return new Level(id, name, diff, len, xs, ys, nx, ny,
            L.StartX, new XVector2(len + L.EndX, L.TerrainH + L.EndY), startI, endI);
    }

    // Converts difficulty (diff) into concrete numeric parameters
    // Kept as one method so tuning remains centralized and readable
    static Params GetParams(int diff)
    {
        float yRange = Min(340f + 24f * diff, 520f);
        float medAmp = Min(22f + 5f * diff, 78f);
        float micAmp = Min(6f + 1f * diff, 14f);
        float maxSlope = Min(1.12f + 0.02f * diff, 1.35f);

        float stepMin = Max(110f - 2.5f * diff, 85f);
        float stepMax = Max(230f - 3.5f * diff, 140f);

        // Ensures control point spacing isn't almost constant
        // Without this, macro skeleton can become too regular
        if (stepMax < stepMin * 1.1f)
            stepMax = stepMin * 1.1f;

        float deadzone = Min(0.10f + 0.01f * diff, 0.18f);
        float slopeMax = Min(0.78f + 0.03f * diff, 0.98f);
        float ctrlRange = Min(260f + 36f * diff, 470f);

        return new Params(yRange, medAmp, micAmp, maxSlope, stepMin, stepMax, deadzone, slopeMax, ctrlRange);
    }

    // Chooses sampling resolution (dx)
    // Smaller dx -> more points -> smoother + more CPU/memory
    static float GetDx(int diff, float len, out int segs)
    {
        float step = Clamp(18f - 0.8f * diff, 10f, 18f);
        segs = Math.Clamp((int)Ceiling(len / step), MinSegs, MaxSegs);
        return len / segs;
    }

    // Fills X coordinates with an uniform grid
    // Last point is forced to be exactly len to avoid accumulated float drift
    static void FillX(float[] xs, float dx, float len)
    {
        for (int i = 0; i < xs.Length; i++)
            xs[i] = i * dx;
        xs[^1] = len;
    }

    // Main height synthesis
    // Uses a local function for readability without spreading parameters
    static void Synth(float[] xs, float[] ys, float[] cx, float[] cy, Noises n, Params p, float len)
    {
        float safe = L.SafeZone;
        float fadeLen = safe * FadeK;

        // Control points X are sorted and xs are increasing
        // So we can evaluate macro with a moving index instead of binary search each time
        int hint = 0;

        for (int i = 0; i < xs.Length; i++)
        {
            float x = xs[i];
            ys[i] = Point(x);
        }

        float Point(float x)
        {
            // Flat safe zone gives predictable start and finish
            if (x <= safe || x >= len - safe)
                return L.TerrainH;

            // Fade prevents sudden cliffs right after safe zone boundary
            float fade = Fade(x, safe, fadeLen, len);

            // rest in 0..1, where 0 = calm, 1 = full difficulty
            float rest = Rest(n.Rest, x);

            // Macro comes from control points skeleton
            float macro = Spline(cx, cy, x, ref hint);

            // In rest zones we reduce macro contribution but keep some shape
            macro = L.TerrainH + (macro - L.TerrainH) * Mix(MacroK, rest);

            // Warp breaks "too straight" features and repetitiveness
            // In rest zones warp is almost disabled via DetailK
            float wx = x, wy = 0f;
            n.Warp.DomainWarp(ref wx, ref wy);
            float sx = x + (wx - x) * Mix(DetailK, rest);

            // Medium + micro detail are sampled on warped X
            float med = n.Med.GetNoise(sx, 0f) * p.MedAmp * Mix(MedK, rest) * fade;
            float mic = n.Mic.GetNoise(sx, 0f) * p.MicAmp * Mix(DetailK, rest) * fade;

            float y = macro + med + mic;

            // Soft clamp is used instead of hard clamp to avoid ugly shelves
            return L.TerrainH + Soft(y - L.TerrainH, p.YRange);
        }
    }

    // Applies constraints after synthesis
    // Order matters: slope clamp -> wheelbase clamp -> absolute Y clamp
    static void Clamp(float[] xs, float[] ys, int segs, float len, Params p, float dx)
    {
        float safe = L.SafeZone;

        // Keeps terrain locally reasonable for gameplay
        ClampSlope(segs, xs, ys, safe, len, p.MaxSlope);

        // Critical for BikePhysics stability
        // It enforces axle constraint: |y(x+wheelSpan)-y(x)| <= MaxWheelDy
        ClampWheel(segs, xs, ys, safe, len, dx);

        // Final absolute guardrail
        ClampY(ys, L.TerrainH - p.YRange, L.TerrainH + p.YRange);
    }

    // Noise initialization
    // We intentionally use seed+N instead of extra seed hashing
    // It is easy to understand and still gives independent layers
    static Noises MakeNoises(int seed, int diff)
    {
        return new Noises(
            Macro(seed, diff),
            RestN(seed + 1, diff),
            Warp(seed + 2, diff),
            Med(seed + 3, diff),
            Mic(seed + 4, diff));
    }

    // Macro noise: cellular creates large "cells" that work well as macro directions
    static FastNoiseLite Macro(int seed, int diff)
    {
        FastNoiseLite n = new(seed);
        n.SetNoiseType(FastNoiseLite.NoiseType.Cellular);
        n.SetCellularReturnType(FastNoiseLite.CellularReturnType.Distance2Div);
        n.SetCellularJitter(0.95f);
        n.SetFractalType(FastNoiseLite.FractalType.FBm);
        n.SetFractalOctaves(3);
        n.SetFractalGain(0.55f);
        n.SetFractalWeightedStrength(0.20f);
        n.SetFrequency(0.00115f + diff * 0.00006f);
        return n;
    }

    // Rest mask: slow simplex to create long calm / hard areas
    static FastNoiseLite RestN(int seed, int diff)
    {
        FastNoiseLite n = new(seed);
        n.SetNoiseType(FastNoiseLite.NoiseType.OpenSimplex2S);
        n.SetFractalType(FastNoiseLite.FractalType.FBm);
        n.SetFractalOctaves(2);
        n.SetFractalGain(0.55f);
        n.SetFrequency(0.00075f + diff * 0.00002f);
        return n;
    }

    // Warp: domain warp adds organic irregularity without brute amplitude
    static FastNoiseLite Warp(int seed, int diff)
    {
        FastNoiseLite n = new(seed);
        n.SetDomainWarpType(FastNoiseLite.DomainWarpType.OpenSimplex2Reduced);
        n.SetFractalType(FastNoiseLite.FractalType.DomainWarpProgressive);
        n.SetFractalOctaves(2);
        n.SetFractalGain(0.50f);
        n.SetFractalLacunarity(2f);
        n.SetFrequency(0.0028f + diff * 0.00012f);
        n.SetDomainWarpAmp(Min(26f + diff * 4.5f, 64f));
        return n;
    }

    // Medium detail: ridged gives sharp bumps rather than smooth hills
    static FastNoiseLite Med(int seed, int diff)
    {
        FastNoiseLite n = new(seed);
        n.SetNoiseType(FastNoiseLite.NoiseType.OpenSimplex2S);
        n.SetFractalType(FastNoiseLite.FractalType.Ridged);
        n.SetFractalOctaves(4);
        n.SetFractalGain(0.55f);
        n.SetFractalWeightedStrength(0.55f);
        n.SetFrequency(0.0105f + diff * 0.00055f);
        return n;
    }

    // Micro detail: smaller scale ridged roughness
    static FastNoiseLite Mic(int seed, int diff)
    {
        FastNoiseLite n = new(seed);
        n.SetNoiseType(FastNoiseLite.NoiseType.OpenSimplex2S);
        n.SetFractalType(FastNoiseLite.FractalType.Ridged);
        n.SetFractalOctaves(2);
        n.SetFractalGain(0.62f);
        n.SetFractalWeightedStrength(0.72f);
        n.SetFrequency(0.034f + diff * 0.0011f);
        return n;
    }

    // Builds control points skeleton
    // This skeleton is the main "track flow"
    // Later details are added on top, but skeleton defines overall ride feeling
    static void MakeCtrl(float len, Params p, ref uint rng, Noises n, out float[] cx, out float[] cy)
    {
        float safe = L.SafeZone;
        float end = len - safe;

        float[] tx = new float[MaxCtrlPts];
        float[] ty = new float[MaxCtrlPts];
        int cnt = 0;

        void Add(float x, float y)
        {
            tx[cnt] = x;
            ty[cnt] = y;
            cnt++;
        }

        // Start flat and safe
        Add(0f, L.TerrainH);
        Add(safe, L.TerrainH);

        float x = safe, y = L.TerrainH, slope = 0f;

        while (x < end - 1f && cnt < MaxCtrlPts - 2)
        {
            // Triangular distribution makes "average" step more frequent
            // It avoids too many tiny steps or too many huge steps
            float step = p.StepMin + (p.StepMax - p.StepMin) * Tri(ref rng);

            float nx = Min(x + step, end);
            step = nx - x;

            if (step < 1f)
                break;

            float rest = Rest(n.Rest, nx);

            // In rest zones we want calmer macro:
            // smaller max slope and more inertia
            float dz = p.Deadzone * (0.18f + 0.82f * rest);
            float sm = p.SlopeMax * (0.30f + 0.70f * rest);
            float inert = Inertia + (1f - rest) * InertiaBoost;

            float target = n.Macro.GetNoise(nx, 0f) * sm;
            slope += (target - slope) * inert;

            // Deadzone prevents almost-flat macro, which feels boring
            if (Abs(slope) < dz)
                slope = Sgn(target) * dz;

            slope = Clamp(slope, -sm, sm);

            // Integrate slope into height
            y += slope * step;

            // Soft clamp keeps it inside a band without hard clipping artifacts
            y = L.TerrainH + Soft(y - L.TerrainH, p.CtrlRange);

            x = nx;
            Add(x, y);
        }

        // End flat and safe
        Add(end, L.TerrainH);
        Add(len, L.TerrainH);

        cx = new float[cnt];
        cy = new float[cnt];
        Array.Copy(tx, cx, cnt);
        Array.Copy(ty, cy, cnt);
    }

    // Limits local slope so terrain is not too steep segment-to-segment
    static void ClampSlope(int cnt, float[] xs, float[] ys, float safe, float len, float max)
    {
        for (int i = 1; i <= cnt; i++)
        {
            float x = xs[i];
            if (x <= safe || x >= len - safe)
                continue;

            float dx = Max(xs[i] - xs[i - 1], 1e-6f);
            float dy = ys[i] - ys[i - 1];
            float lim = max * dx;

            if (Abs(dy) > lim)
                ys[i] = ys[i - 1] + Sgn(dy) * lim;
        }
    }

    // Limits height difference across wheel base
    // Two passes help propagation because windows overlap
    // One clamp can create a new violation right next to it, second pass reduces that chance
    static void ClampWheel(int cnt, float[] xs, float[] ys, float safe, float len, float dx)
    {
        int span = Math.Clamp((int)Round(WheelSpan / Max(dx, 1f)), 2, cnt - 1);

        for (int pass = 0; pass < 2; pass++)
        {
            for (int i = 0; i + span <= cnt; i++)
            {
                if (xs[i + span] <= safe || xs[i] >= len - safe)
                    continue;

                float dy = Abs(ys[i + span] - ys[i]);
                if (dy <= MaxWheelDy)
                    continue;

                // Compress deviations inside this span
                float k = MaxWheelDy / dy;
                float y0 = ys[i];

                for (int j = 1; j <= span; j++)
                    ys[i + j] = y0 + (ys[i + j] - y0) * k;
            }
        }
    }

    // Final absolute clamp to keep terrain inside visible/camera band
    // This is a "safety net", not the main shaping tool
    static void ClampY(float[] ys, float min, float max)
    {
        for (int i = 0; i < ys.Length; i++)
            ys[i] = Clamp(ys[i], min, max);
    }

    // Computes normals per segment
    // Normals are scaled by *2 because some collision/physics code expects that
    // Also we force normals to point "up" (negative Y in screen coordinates)
    static void Normals(int cnt, float[] xs, float[] ys, float[] nx, float[] ny)
    {
        for (int i = 0; i < cnt; i++)
        {
            float dx = xs[i + 1] - xs[i];
            float dy = ys[i + 1] - ys[i];

            float l = Sqrt(dx * dx + dy * dy);
            float inv = l > 1e-6f ? 2f / l : 0f;

            float nxi = -dy * inv;
            float nyi = dx * inv;

            if (nyi > 0f)
            {
                nxi = -nxi;
                nyi = -nyi;
            }

            nx[i] = nxi;
            ny[i] = nyi;
        }
    }

    // Macro evaluation on the control points
    // Uses a moving hint index because x is increasing
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Spline(float[] cx, float[] cy, float x, ref int i)
    {
        while (i < cx.Length - 2 && cx[i + 1] <= x)
            i++;

        float t = (x - cx[i]) / (cx[i + 1] - cx[i]);
        return cy[i] + (cy[i + 1] - cy[i]) * t;
    }

    // Converts Rest noise (-1..1) into a clean 0..1 intensity
    // SmoothStep gives stable transitions instead of hard thresholds
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Rest(FastNoiseLite n, float x)
    {
        float v = (n.GetNoise(x, 0f) + 1f) * 0.5f;
        return Smooth(RestLo, RestHi, v);
    }

    // Fades terrain in/out near safe zones
    // This avoids sharp terrain start and improves first seconds of gameplay
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Fade(float x, float safe, float fadeLen, float len)
    {
        float l = Smooth(safe, safe + fadeLen, x);
        float r = Smooth(len - safe, len - safe - fadeLen, x);
        return l * r;
    }

    // Soft clamp keeps shape but prevents runaway peaks
    // Formula: v * r / (r + |v|)
    // As |v| grows, result approaches +/-r smoothly
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Soft(float v, float r) => v * (r / (r + Abs(v)));

    // Rest->Fight interpolation
    // When t=0 returns a (rest multiplier), when t=1 returns 1 (fight)
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Mix(float a, float t) => a + (1f - a) * t;

    // SmoothStep with manual clamp
    // Used for rest mask and fade transitions
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Smooth(float a, float b, float x)
    {
        float t = Clamp((x - a) / (b - a), 0f, 1f);
        return t * t * (3f - 2f * t);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Clamp(float v, float lo, float hi) => v < lo ? lo : v > hi ? hi : v;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Sgn(float v) => v >= 0f ? 1f : -1f;

    // Triangular distribution in [0..1]
    // Helps avoid extreme step values too often
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Tri(ref uint s) => 0.5f * (Rnd(ref s) + Rnd(ref s));

    // Xorshift RNG
    // Fast and allocation-free, quality is enough for procedural stepping
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float Rnd(ref uint s)
    {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return s * 2.3283064e-10f; // 1 / 2^32
    }

    // Small seed mix to reduce obvious patterns when seed is small or similar
    // This is not "crypto", just a cheap scramble
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static uint Rng(uint s)
    {
        s ^= s >> 16;
        s *= 0x7feb352d;
        s ^= s >> 15;
        return s;
    }
}
