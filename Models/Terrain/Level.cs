using static System.MathF;
using Vec2 = Microsoft.Xna.Framework.Vector2;

namespace GravityDefiedGame.Models.Terrain;

// Core level parameters shared between data and generator
public readonly record struct LevelConst(
    float GroundY, float SafeZone, float TerrainH,
    float BaseLen, float LenPerLvl, float MaxLen,
    float StartX, float StartGateOffX,
    float EndX, float EndY, float FinishDist, float FinishGateOffX,
    int BasePts, int PtsPerLvl, int MaxPts,
    float Epsilon, float DefX, float DefY,
    float DefNormX, float DefNormY)
{
    public static readonly LevelConst Default = new(
        GroundY: 500f, SafeZone: 300f, TerrainH: 500f,
        BaseLen: 3000f, LenPerLvl: 500f, MaxLen: 10000f,
        StartX: 100f, StartGateOffX: 65f,
        EndX: -100f, EndY: 0f,
        FinishDist: 50f, FinishGateOffX: -65f,
        BasePts: 60, PtsPerLvl: 5, MaxPts: 200,
        Epsilon: 1e-6f, DefX: 0f, DefY: 500f,
        DefNormX: 0f, DefNormY: -1f);
}

// Level stores terrain points and segment normals
// Data-only class with query API for physics and rendering
public sealed class Level
{
    public static readonly LevelConst Cfg = LevelConst.Default;

    readonly float[] _ptsX, _ptsY, _normX, _normY;

    int _visStart, _visEnd;
    float _lastLeftX, _lastRightX;

    public int Id { get; }
    public string Name { get; }
    public int Diff { get; }
    public float Len { get; }
    public int PtCount { get; }
    public float StartX { get; }
    public Vec2 FinishPoint { get; }
    public int StartIdx { get; }
    public int FinishIdx { get; }

    public float StartGateX => StartX + Cfg.StartGateOffX;
    public float FinishGateX => FinishPoint.X + Cfg.FinishGateOffX;

    public Vec2 StartGatePos =>
        new(StartGateX, GetGroundYAtX(StartGateX));

    public Vec2 FinishGatePos =>
        new(FinishGateX, GetGroundYAtX(FinishGateX));

    public Level(
        int id, string name, int diff, float len,
        float[] ptsX, float[] ptsY,
        float[] normX, float[] normY,
        float startX, Vec2 finish,
        int startIdx, int finishIdx)
    {
        (Id, Name, Diff, Len) = (id, name, diff, len);
        (_ptsX, _ptsY, _normX, _normY) =
            (ptsX, ptsY, normX, normY);
        (StartX, FinishPoint) = (startX, finish);
        (StartIdx, FinishIdx) = (startIdx, finishIdx);
        PtCount = ptsX.Length;
        (_visStart, _visEnd) = (0, PtCount - 1);
        (_lastLeftX, _lastRightX) = (Cfg.DefX, len);
    }

    // Factory delegates to generator — each run produces different terrain
    public static Level Create(
        int id, string name,
        int? seed = null, int? diff = null) =>
        Generator.Generate(id, name, seed, diff);

    public float GetPtX(int i) =>
        (uint)i < (uint)PtCount ? _ptsX[i] : Cfg.DefX;

    public float GetPtY(int i) =>
        (uint)i < (uint)PtCount ? _ptsY[i] : Cfg.DefY;

    public (int Start, int End) GetVisibleRange() =>
        (_visStart, _visEnd);

    // Hot path for camera and collision
    // Binary search avoids linear scan on long tracks and keeps frame time stable
    public float GetGroundYAtX(float x)
    {
        int n = PtCount;

        if (n <= 0)
            return Cfg.DefY;
        if (n == 1)
            return _ptsY[0];

        float[] xs = _ptsX, ys = _ptsY;
        int last = n - 1;

        if (x <= xs[0])
            return ys[0];
        if (x >= xs[last])
            return ys[last];

        // Find lo so xs[lo] <= x < xs[lo + 1]
        int lo = 0, hi = last - 1;

        while (lo < hi)
        {
            // Bias to the right to avoid stalling when lo and hi are neighbors
            int mid = (lo + hi + 1) >> 1;

            if (xs[mid] <= x)
                lo = mid;
            else
                hi = mid - 1;
        }

        float x0 = xs[lo], y0 = ys[lo];
        float x1 = xs[lo + 1], y1 = ys[lo + 1];
        float dx = x1 - x0;

        // Guard for bad data where two points share same X
        return Abs(dx) < Cfg.Epsilon
            ? y0
            : y0 + (y1 - y0) * ((x - x0) / dx);
    }

    public void GetSegmentPoints(
        int idx,
        out float ax, out float ay,
        out float bx, out float by)
    {
        if ((uint)idx >= (uint)(PtCount - 1))
        {
            (ax, ay, bx, by) =
                (Cfg.DefX, Cfg.DefY, Cfg.DefX, Cfg.DefY);
            return;
        }

        (ax, ay) = (_ptsX[idx], _ptsY[idx]);
        (bx, by) = (_ptsX[idx + 1], _ptsY[idx + 1]);
    }

    public void GetSegmentNormal(
        int idx, out float nx, out float ny)
    {
        if ((uint)idx >= (uint)_normX.Length)
        {
            (nx, ny) = (Cfg.DefNormX, Cfg.DefNormY);
            return;
        }

        (nx, ny) = (_normX[idx], _normY[idx]);
    }

    public bool CrossedStartGate(float x0, float x1) =>
        x0 < StartGateX && x1 >= StartGateX;

    public bool CrossedFinishGate(float x0, float x1) =>
        x0 < FinishGateX && x1 >= FinishGateX;

    // Physics and render should not walk full arrays each frame
    // Visible range acts like a moving window around the player
    public void UpdateVisible(
        float leftX, float rightX, float margin)
    {
        if (PtCount < 2)
            return;

        leftX -= margin;
        rightX += margin;

        if (rightX > _lastRightX)
            while (_visEnd < PtCount - 1
                && rightX > _ptsX[_visEnd])
                _visEnd++;

        if (leftX < _lastLeftX)
            while (_visStart > 0
                && leftX < _ptsX[_visStart])
                _visStart--;

        if (leftX > _lastLeftX)
            while (_visStart < PtCount - 1
                && leftX > _ptsX[_visStart + 1])
                _visStart++;

        if (rightX < _lastRightX)
            while (_visEnd > 0
                && rightX < _ptsX[_visEnd - 1])
                _visEnd--;

        (_lastLeftX, _lastRightX) = (leftX, rightX);
    }
}
