using static System.MathF;
using Log = GravityDefiedGame.Core.Log;

namespace GravityDefiedGame.Models.Bike;

#if DEBUG
public sealed partial class BikePhysics
{
    static class Dbg
    {
        public const int LogInt = 4, StatsInt = 240, PhysInt = 60;
        public const int IdxFrame = 0, IdxFw = 1, IdxRw = 2;
        public const float R2D = 57.2957795f, FlipWarn = 100f, SuspWarn = 0.92f, OmegaWarn = 1.2f;
        public const float Lambda1 = 0.96f, Lambda2 = 0.24f;
        public const char T = '\t', G = '|';
    }

    readonly System.Text.StringBuilder _dbgSb = new(768);

    bool _dbgHdr;

    BikeState _dbgPrevSt;
    bool _dbgPrevFw, _dbgPrevRw, _dbgPrevFin;
    int _dbgPrevCollIdx = int.MinValue, _dbgPrevCollType = int.MinValue;

    float _dbgPrevSpd, _dbgMaxSpd, _dbgMaxAcc, _dbgMaxAV, _dbgMaxFwO, _dbgMaxRwO;
    int _dbgAirF, _dbgGndF, _dbgTotRes, _dbgMaxRes, _dbgOverCnt, _dbgWarnCnt;

    int _dbgFrameRes;
    float _dbgFrameMaxO;

    int _dbgCOn, _dbgCIdx, _dbgCType;

    int _dbgCTouch;

    float _dbgCGap, _dbgCNx, _dbgCNy, _dbgCAng, _dbgCVn, _dbgCVt;

    int _dbgLastCollType = 2, _dbgLastCollIdx = -1;
    float _dbgLastCollNx, _dbgLastCollNy;
    bool _dbgHadColl;

    int _dbgROn, _dbgRIter, _dbgRIdx;
    float _dbgRBefore, _dbgRAfter, _dbgRDelta, _dbgROverP;

    int _dbgPOn, _dbgPDrift;
    float _dbgPL1, _dbgPL2, _dbgPRho, _dbgPDmp, _dbgPHalf, _dbgPCov, _dbgPCvo, _dbgPKick;

    int _dbgSOn;

    BikeInput _dbgPrevInput;
    int _dbgInputSeq;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void DbgTrackTouch(int idx, bool touch)
    {
        if (!touch)
            return;

        if (idx == Dbg.IdxFw)
            _dbgCTouch |= 1;
        else if (idx == Dbg.IdxRw)
            _dbgCTouch |= 2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void DbgSaveLastColl(int idx, int type)
    {
        _dbgLastCollIdx = idx;
        _dbgLastCollType = type;
        _dbgLastCollNx = _collNx;
        _dbgLastCollNy = _collNy;
        _dbgHadColl = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void DbgSaveLastColl(int idx) => DbgSaveLastColl(idx, 1);

    void DbgHdr()
    {
        if (_dbgHdr)
            return;
        _dbgHdr = true;

        char t = Dbg.T, g = Dbg.G;

        _dbgSb.Clear();
        _dbgSb.Append("HDR");
        _dbgSb.Append(t);
        _dbgSb.Append('f');

        _dbgSb.Append(g);
        _dbgSb.Append("BIKE");
        _dbgSb.Append(t);
        _dbgSb.Append("st\twarn\tfin\tv\ta\tang\tav\tgnd\tsf\tsr");

        _dbgSb.Append(g);
        _dbgSb.Append("IN");
        _dbgSb.Append(t);
        _dbgSb.Append("thr\tbrk\tlean\tm\ttau");

        _dbgSb.Append(g);
        _dbgSb.Append("WHL");
        _dbgSb.Append(t);
        _dbgSb.Append("oFW\toRW\toMx\tmx\tlim\tr%\tover\tres\tpk");

        _dbgSb.Append(g);
        _dbgSb.Append("COL");
        _dbgSb.Append(t);
        _dbgSb.Append("on\tidx\ttype\ttch\tgap\tnx\tny\tang\tvN\tvT");

        _dbgSb.Append(g);
        _dbgSb.Append("RES");
        _dbgSb.Append(t);
        _dbgSb.Append("on\titer\tidx\tbefore\tafter\tdelta\tover%");

        _dbgSb.Append(g);
        _dbgSb.Append("PHY");
        _dbgSb.Append(t);
        _dbgSb.Append("on\tl1\tl2\trho\tstab\tdmp%\thalf\tcOV\tcVO\tkick\tdrift");

        _dbgSb.Append(g);
        _dbgSb.Append("STS");
        _dbgSb.Append(t);
        _dbgSb.Append("on\tair%\tvMax\taMax\tavMax\toFWMax\toRWMax\tlim\toverCnt\twarnCnt\tresT\tresMax\tresAvg");

        Log.Debug("H", _dbgSb.ToString());
    }

    void DbgLogInput()
    {
        bool changed = Abs(_input.Throttle - _dbgPrevInput.Throttle) > 0.01f ||
                       Abs(_input.Brake - _dbgPrevInput.Brake) > 0.01f ||
                       Abs(_input.Lean - _dbgPrevInput.Lean) > 0.01f;

        if (!changed)
            return;

        int ch = 0;
        if (Abs(_input.Throttle - _dbgPrevInput.Throttle) > 0.01f)
            ch |= 1;
        if (Abs(_input.Brake - _dbgPrevInput.Brake) > 0.01f)
            ch |= 2;
        if (Abs(_input.Lean - _dbgPrevInput.Lean) > 0.01f)
            ch |= 4;

        _dbgSb.Clear();
        _dbgSb.Append("INP\tseq\t");
        _dbgSb.Append(_dbgInputSeq++);
        _dbgSb.Append("\tch\t");
        _dbgSb.Append(ch);
        _dbgSb.Append("\tthr\t");
        DbgApF(_input.Throttle, 2);
        _dbgSb.Append("\tbrk\t");
        DbgApF(_input.Brake, 2);
        _dbgSb.Append("\tlean\t");
        DbgApF(_input.Lean, 2);

        Log.Debug("Input", _dbgSb.ToString());
        _dbgPrevInput = _input;
    }

    void DbgLog(float dt)
    {
        if (dt <= 0f)
            return;

        DbgHdr();
        DbgLogInput();

        float spd = Speed, acc = (spd - _dbgPrevSpd) / dt;
        float angD = Ang * Dbg.R2D, avD = AngVel * Dbg.R2D;

        float fwO = Abs(_cur[Dbg.IdxFw].AngVel), rwO = Abs(_cur[Dbg.IdxRw].AngVel);
        float oMx = fwO > rwO ? fwO : rwO, lim = Cfg.MaxWheelOmega;
        int mx = fwO > rwO ? 1 : rwO > fwO ? 2 : 0;

        float rPct = lim > 0.0001f ? (oMx / lim) * 100f : 0f;
        float over = oMx > lim ? oMx - lim : 0f;

        _dbgPrevSpd = spd;

        if (spd > _dbgMaxSpd)
            _dbgMaxSpd = spd;
        if (Abs(acc) > _dbgMaxAcc)
            _dbgMaxAcc = Abs(acc);
        if (Abs(AngVel) > _dbgMaxAV)
            _dbgMaxAV = Abs(AngVel);
        if (fwO > _dbgMaxFwO)
            _dbgMaxFwO = fwO;
        if (rwO > _dbgMaxRwO)
            _dbgMaxRwO = rwO;

        if (over > 0f)
            _dbgOverCnt++;
        if (lim > 0.0001f && oMx / lim > Dbg.OmegaWarn)
            _dbgWarnCnt++;

        if (InAir)
            _dbgAirF++;
        else
            _dbgGndF++;

        if (_dbgFrameRes > _dbgMaxRes)
            _dbgMaxRes = _dbgFrameRes;
        _dbgTotRes += _dbgFrameRes;

        bool stChg = State != _dbgPrevSt;
        bool gChg = _dbgPrevFw != FwGnd || _dbgPrevRw != RwGnd;
        bool finChg = _dbgPrevFin != FinishReached;

        bool collChg = _dbgHadColl &&
                       (_dbgLastCollIdx != _dbgPrevCollIdx || _dbgLastCollType != _dbgPrevCollType);

        int warn = 0;
        if (Abs(angD) > Dbg.FlipWarn)
            warn |= 1;
        if (FrontSusp > Dbg.SuspWarn || RearSusp > Dbg.SuspWarn)
            warn |= 2;
        if (lim > 0.0001f && oMx / lim > Dbg.OmegaWarn)
            warn |= 8;
        if (over > 0f)
            warn |= 16;

        bool isStat = _frameNum % Dbg.StatsInt == 0 && _frameNum > 0;
        bool isPhys = _frameNum % Dbg.PhysInt == 0;
        bool isTick = _frameNum % Dbg.LogInt == 0;

        bool log = isStat || stChg || gChg || finChg ||
                   (isTick && (collChg || warn != 0 || _dbgFrameRes > 0));

        if (!log && !isPhys)
            return;

        _dbgPrevSt = State;
        (_dbgPrevFw, _dbgPrevRw, _dbgPrevFin) = (FwGnd, RwGnd, FinishReached);

        if (_dbgHadColl)
            (_dbgPrevCollIdx, _dbgPrevCollType) = (_dbgLastCollIdx, _dbgLastCollType);

        if (isPhys)
            DbgUpdPhys();
        _dbgSOn = isStat ? 1 : 0;

        DbgUpdColl();
        DbgFrameLine(spd, acc, angD, avD, warn, fwO, rwO, oMx, mx, lim, rPct, over);
    }

    void DbgFrameLine(float spd, float acc, float angD, float avD, int warn,
                      float fwO, float rwO, float oMx, int mx, float lim, float rPct, float over)
    {
        char t = Dbg.T, g = Dbg.G;

        int fin = FinishReached ? 1 : 0;
        int gnd = (FwGnd ? 1 : 0) | (RwGnd ? 2 : 0);
        int im = (_braking ? 1 : 0) | (_leanL ? 2 : 0) | (_leanR ? 4 : 0);

        ReadOnlySpan<char> st = State switch
        {
            var s when (s & BikeState.Crashed) != 0 => "DIE",
            var s when (s & BikeState.Wheelie) != 0 => "WHL",
            var s when (s & BikeState.Stoppie) != 0 => "STP",
            BikeState.InAir => "AIR",
            BikeState.Grounded => "GND",
            _ => "---"
        };

        _dbgSb.Clear();
        _dbgSb.Append("FRM");
        _dbgSb.Append(t);
        DbgApI(_frameNum, 0);

        _dbgSb.Append(g);
        _dbgSb.Append("BIKE");
        _dbgSb.Append(t);
        _dbgSb.Append(st);
        _dbgSb.Append(t);
        DbgApI(warn, 0);
        _dbgSb.Append(t);
        DbgApI(fin, 0);
        _dbgSb.Append(t);
        DbgApF(spd, 1);
        _dbgSb.Append(t);
        DbgApF(acc, 0);
        _dbgSb.Append(t);
        DbgApF(angD, 1);
        _dbgSb.Append(t);
        DbgApF(avD, 1);
        _dbgSb.Append(t);
        DbgApI(gnd, 0);
        _dbgSb.Append(t);
        DbgApF(FrontSusp, 2);
        _dbgSb.Append(t);
        DbgApF(RearSusp, 2);

        _dbgSb.Append(g);
        _dbgSb.Append("IN");
        _dbgSb.Append(t);
        DbgApF(_input.Throttle, 2);
        _dbgSb.Append(t);
        DbgApF(_input.Brake, 2);
        _dbgSb.Append(t);
        DbgApF(_input.Lean, 2);
        _dbgSb.Append(t);
        DbgApI(im, 0);
        _dbgSb.Append(t);
        DbgApF(_engineTorque, 0);

        _dbgSb.Append(g);
        _dbgSb.Append("WHL");
        _dbgSb.Append(t);
        DbgApF(fwO, 2);
        _dbgSb.Append(t);
        DbgApF(rwO, 2);
        _dbgSb.Append(t);
        DbgApF(oMx, 2);
        _dbgSb.Append(t);
        DbgApI(mx, 0);
        _dbgSb.Append(t);
        DbgApF(lim, 1);
        _dbgSb.Append(t);
        DbgApF(rPct, 0);
        _dbgSb.Append(t);
        DbgApF(over, 2);
        _dbgSb.Append(t);
        DbgApI(_dbgFrameRes, 0);
        _dbgSb.Append(t);
        DbgApF(_dbgFrameMaxO, 2);

        _dbgSb.Append(g);
        _dbgSb.Append("COL");
        _dbgSb.Append(t);
        DbgApI(_dbgCOn, 0);
        _dbgSb.Append(t);
        DbgApI(_dbgCIdx, 0);
        _dbgSb.Append(t);
        DbgApI(_dbgCType, 0);
        _dbgSb.Append(t);
        DbgApI(_dbgCTouch, 0);
        _dbgSb.Append(t);
        DbgApF(_dbgCGap, 2);
        _dbgSb.Append(t);
        DbgApF(_dbgCNx, 3);
        _dbgSb.Append(t);
        DbgApF(_dbgCNy, 3);
        _dbgSb.Append(t);
        DbgApF(_dbgCAng, 1);
        _dbgSb.Append(t);
        DbgApF(_dbgCVn, 1);
        _dbgSb.Append(t);
        DbgApF(_dbgCVt, 1);

        _dbgSb.Append(g);
        _dbgSb.Append("RES");
        _dbgSb.Append(t);
        DbgApI(_dbgROn, 0);
        if (_dbgROn != 0)
        {
            _dbgSb.Append(t);
            DbgApI(_dbgRIter, 0);
            _dbgSb.Append(t);
            DbgApI(_dbgRIdx, 0);
            _dbgSb.Append(t);
            DbgApF(_dbgRBefore, 2);
            _dbgSb.Append(t);
            DbgApF(_dbgRAfter, 2);
            _dbgSb.Append(t);
            DbgApF(_dbgRDelta, 2);
            _dbgSb.Append(t);
            DbgApF(_dbgROverP, 0);
        }
        else
            _dbgSb.Append("\t\t\t\t\t\t");

        _dbgSb.Append(g);
        _dbgSb.Append("PHY");
        _dbgSb.Append(t);
        DbgApI(_dbgPOn, 0);
        if (_dbgPOn != 0)
        {
            int stab = _dbgPRho < 1f ? 1 : 0;
            _dbgSb.Append(t);
            DbgApF(_dbgPL1, 4);
            _dbgSb.Append(t);
            DbgApF(_dbgPL2, 4);
            _dbgSb.Append(t);
            DbgApF(_dbgPRho, 4);
            _dbgSb.Append(t);
            DbgApI(stab, 0);
            _dbgSb.Append(t);
            DbgApF(_dbgPDmp, 1);
            _dbgSb.Append(t);
            DbgApF(_dbgPHalf, 1);
            _dbgSb.Append(t);
            DbgApF(_dbgPCov, 3);
            _dbgSb.Append(t);
            DbgApF(_dbgPCvo, 3);
            _dbgSb.Append(t);
            DbgApF(_dbgPKick, 1);
            _dbgSb.Append(t);
            DbgApI(_dbgPDrift, 0);
        }
        else
            _dbgSb.Append("\t\t\t\t\t\t\t\t\t\t");

        _dbgSb.Append(g);
        _dbgSb.Append("STS");
        _dbgSb.Append(t);
        DbgApI(_dbgSOn, 0);
        if (_dbgSOn != 0)
        {
            int tot = _dbgAirF + _dbgGndF;
            if (tot == 0)
                tot = 1;

            float airP = 100f * _dbgAirF / tot;
            float avgRes = _dbgTotRes / (float)(_frameNum > 0 ? _frameNum : 1);

            _dbgSb.Append(t);
            DbgApF(airP, 0);
            _dbgSb.Append(t);
            DbgApF(_dbgMaxSpd, 1);
            _dbgSb.Append(t);
            DbgApF(_dbgMaxAcc, 0);
            _dbgSb.Append(t);
            DbgApF(_dbgMaxAV * Dbg.R2D, 0);
            _dbgSb.Append(t);
            DbgApF(_dbgMaxFwO, 2);
            _dbgSb.Append(t);
            DbgApF(_dbgMaxRwO, 2);
            _dbgSb.Append(t);
            DbgApF(Cfg.MaxWheelOmega, 1);
            _dbgSb.Append(t);
            DbgApI(_dbgOverCnt, 0);
            _dbgSb.Append(t);
            DbgApI(_dbgWarnCnt, 0);
            _dbgSb.Append(t);
            DbgApI(_dbgTotRes, 0);
            _dbgSb.Append(t);
            DbgApI(_dbgMaxRes, 0);
            _dbgSb.Append(t);
            DbgApF(avgRes, 2);
        }
        else
            _dbgSb.Append("\t\t\t\t\t\t\t\t\t\t\t\t");

        Log.Debug("F", _dbgSb.ToString());
    }

    void DbgUpdColl()
    {
        _dbgCOn = _dbgHadColl ? 1 : 0;
        _dbgCIdx = _dbgHadColl ? _dbgLastCollIdx : -1;
        _dbgCType = _dbgHadColl ? _dbgLastCollType : 2;
        (_dbgCGap, _dbgCNx, _dbgCNy, _dbgCAng, _dbgCVn, _dbgCVt) = (0f, 0f, 0f, 0f, 0f, 0f);

        if (!_dbgHadColl)
            return;

        int idx = _dbgLastCollIdx;
        if ((uint)idx >= 6u)
            return;

        float nx = _dbgLastCollNx, ny = _dbgLastCollNy;
        float nLen = FastDist(nx, ny);
        float nxN = nLen > 0.001f ? nx / nLen : 0f;
        float nyN = nLen > 0.001f ? ny / nLen : -1f;

        float ang = Atan2(nyN, nxN) * Dbg.R2D;

        float vx = _cur[idx].Vel.X * P.Scale;
        float vy = _cur[idx].Vel.Y * P.Scale;

        float vN = -(vx * nxN + vy * nyN);
        float vT = -(vx * (-nyN) + vy * nxN);

        _dbgCGap = 0f;
        _dbgCNx = nxN;
        _dbgCNy = nyN;
        _dbgCAng = ang;
        _dbgCVn = vN;
        _dbgCVt = vT;
    }

    void DbgUpdPhys()
    {
        float r = Cfg.WheelR;
        float a11 = WF.RotDamp, a12 = -WF.SlipFriction / r;
        float a21 = -WF.RollCouple * r, a22 = WF.TangentDamp;

        float tr = a11 + a22, det = a11 * a22 - a12 * a21;
        float disc = tr * tr - 4f * det;

        float l1, l2;
        if (disc >= 0f)
        {
            float sq = Sqrt(disc);
            l1 = (tr + sq) * 0.5f;
            l2 = (tr - sq) * 0.5f;
        }
        else
        {
            float re = tr * 0.5f, im = Sqrt(-disc) * 0.5f;
            l1 = l2 = Sqrt(re * re + im * im);
        }

        float rho = Abs(l1) > Abs(l2) ? Abs(l1) : Abs(l2);
        float dmp = (1f - rho) * 100f;
        float half = rho > 0.01f ? Log(0.5f) / Log(rho) : 999f;
        float kick = WF.RollCouple * r * Cfg.MaxWheelOmega;
        int drift = Abs(l1 - Dbg.Lambda1) > 0.05f || Abs(l2 - Dbg.Lambda2) > 0.05f ? 1 : 0;

        _dbgPOn = 1;
        (_dbgPL1, _dbgPL2, _dbgPRho, _dbgPDmp, _dbgPHalf) = (l1, l2, rho, dmp, half);
        (_dbgPCov, _dbgPCvo, _dbgPKick, _dbgPDrift) = (Abs(a21), Abs(a12), kick, drift);
    }

    void DbgClrFrame()
    {
        (_dbgPOn, _dbgSOn) = (0, 0);

        (_dbgCOn, _dbgCIdx, _dbgCType) = (0, -1, 2);
        _dbgCTouch = 0;
        (_dbgCGap, _dbgCNx, _dbgCNy, _dbgCAng, _dbgCVn, _dbgCVt) = (0f, 0f, 0f, 0f, 0f, 0f);

        (_dbgROn, _dbgRIter, _dbgRIdx) = (0, 0, -1);
        (_dbgRBefore, _dbgRAfter, _dbgRDelta, _dbgROverP) = (0f, 0f, 0f, 0f);

        (_dbgFrameRes, _dbgFrameMaxO) = (0, 0f);

        _dbgLastCollIdx = -1;
        _dbgLastCollType = 2;
        _dbgLastCollNx = 0f;
        _dbgLastCollNy = 0f;
        _dbgHadColl = false;
    }

    void DbgTrackResolve(int idx, float oBefore, float oAfter)
    {
        _dbgFrameRes++;

        float absO = Abs(oAfter);
        if (absO > _dbgFrameMaxO)
            _dbgFrameMaxO = absO;

        float d = Abs(oAfter - oBefore);
        float lim = Cfg.MaxWheelOmega;
        float overP = absO > lim && lim > 0.0001f ? (absO / lim - 1f) * 100f : 0f;

        (_dbgROn, _dbgRIter, _dbgRIdx) = (1, _dbgFrameRes, idx);
        (_dbgRBefore, _dbgRAfter, _dbgRDelta, _dbgROverP) = (oBefore, oAfter, d, overP);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void DbgApF(float v, int d)
    {
        Span<char> buf = stackalloc char[24];
        ReadOnlySpan<char> fmt = d switch
        {
            0 => "F0",
            2 => "F2",
            3 => "F3",
            4 => "F4",
            _ => "F1"
        };
        if (v.TryFormat(buf, out int n, fmt, System.Globalization.CultureInfo.InvariantCulture))
            _dbgSb.Append(buf[..n]);
        else
            _dbgSb.Append('?');
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    void DbgApI(int v, int w)
    {
        Span<char> buf = stackalloc char[12];
        ReadOnlySpan<char> fmt = w == 5 ? "D5" : "D0";
        if (v.TryFormat(buf, out int n, fmt, System.Globalization.CultureInfo.InvariantCulture))
            _dbgSb.Append(buf[..n]);
        else
            _dbgSb.Append('?');
    }
}
#endif
