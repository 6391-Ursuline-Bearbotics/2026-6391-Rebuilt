#!/usr/bin/env python3
"""
Swerve Wheel Slip / Free-Spin Analyzer — Team 6391
Detects drivetrain modules that are spinning freely without gripping carpet
by analyzing the relationship between motor velocity and current draw.

THEORY:
  For a healthy module, current scales roughly linearly with velocity — the
  motor fights both back-EMF and carpet rolling resistance.  A slipping or
  mechanically damaged wheel breaks that relationship: the wheel spins fast
  but draws less current because it is not loaded against the carpet.

  Key metric — Current-Per-Rad/s (CPR):
    CPR = StatorCurrentAmps / |VelocityRadPerSec|  (when wheel is moving)
    Low CPR relative to the other modules = wheel under-loaded = suspect

  Velocity Signature Table:
    Median current in each velocity bin (0-10, 10-20, ... rad/s).
    A damaged module will show a consistently lower row vs healthy modules.

Usage:
    python slip_analyzer.py [log_directory] [options]

Options:
    -o / --output FILE    Write report to a file in addition to stdout
    --no-practice         Exclude practice match logs
    --match LABEL         Analyze only one match (e.g. "Qual 5")
    --module N            Highlight a specific module index (0=FL 1=FR 2=BR 3=BL)
    --vel-thresh FLOAT    Min rad/s to consider "moving" for CPR (default: 5.0)
    --slip-vel FLOAT      rad/s threshold for slip event detection (default: 40.0)
    --slip-amp FLOAT      Max amps at slip-vel to count as a slip event (default: 15.0)

Interpreting results:
    CPR ratio (module vs avg of others) < 0.94  = low,  flag for inspection
    CPR ratio                           < 0.88  = critically low, likely mechanical issue
    Consistent drop across matches = progressive wear/damage
    Single-match spike in P95 current = possible debris event or hard impact
"""

import argparse
import math
import os
import re
import sys
from wpiutil.log import DataLogReader

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
MODULE_NAMES     = {0: "FL", 1: "FR", 2: "BR", 3: "BL"}
DEFAULT_VEL_MIN  = 5.0    # rad/s — below this, CPR calculation skipped
DEFAULT_SLIP_VEL = 40.0   # rad/s — ~4 m/s with 2in wheel
DEFAULT_SLIP_AMP = 15.0   # amps  — current below this at high speed = slip event
VEL_BINS         = [(0,10),(10,20),(20,30),(30,40),(40,50),(50,60),(60,70),(70,80)]

# Thresholds for flagging
CPR_LOW_WARN = 0.94   # module CPR / avg-of-others  below this = warn
CPR_LOW_CRIT = 0.88   # below this = critical

WANTED_KEYS = set(
    [f"/Drive/Module{i}/DriveCurrentAmps"       for i in range(4)] +
    [f"/Drive/Module{i}/DriveVelocityRadPerSec"  for i in range(4)] +
    ["/DriverStation/Enabled"]
)


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------
def _build_enabled_intervals(enabled_series):
    """Return list of (start_sec, end_sec) tuples when robot was enabled."""
    intervals, last_start, last_state, last_ts = [], None, False, 0.0
    for ts, val in enabled_series:
        last_ts = ts
        if val and not last_state:
            last_start = ts
        elif not val and last_state and last_start is not None:
            intervals.append((last_start, ts))
        last_state = val
    if last_state and last_start is not None:
        intervals.append((last_start, last_ts))
    return intervals


def _is_enabled(t, intervals):
    return any(s <= t <= e for s, e in intervals)


def _stats(vals):
    if not vals:
        return None
    s = sorted(vals)
    n = len(s)
    return {
        "mean":   sum(s) / n,
        "median": s[n // 2],
        "p25":    s[n // 4],
        "p75":    s[3 * n // 4],
        "p95":    s[int(0.95 * n)],
        "max":    s[-1],
        "n":      n,
    }


def _pearson(xs, ys):
    """Pearson correlation coefficient between two lists."""
    n = len(xs)
    if n < 2:
        return 0.0
    mx, my = sum(xs) / n, sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx  = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy  = math.sqrt(sum((y - my) ** 2 for y in ys))
    return num / (dx * dy) if dx * dy > 0 else 0.0


def _nearest_amp(amp_ts_sorted, t, window=0.025):
    """Binary-search the nearest amplitude timestamp within `window` seconds."""
    lo, hi = 0, len(amp_ts_sorted) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if amp_ts_sorted[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    best = amp_ts_sorted[lo]
    return best if abs(best - t) <= window else None


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------
def parse_log(filepath, vel_min, slip_vel, slip_amp):
    """
    Parse one .wpilog file and return per-module velocity/current pair data.

    Returns:
        dict: {module_index: [(vel_radps, amps), ...]}  (enabled periods only)
    """
    reader    = DataLogReader(filepath)
    wanted_ids = {}
    raw        = {}

    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            if sd.name in WANTED_KEYS:
                wanted_ids[sd.entry] = sd.name
        elif record.isControl():
            continue
        else:
            eid = record.getEntry()
            if eid not in wanted_ids:
                continue
            name = wanted_ids[eid]
            ts   = record.getTimestamp() / 1e6
            try:
                val = record.getBoolean() if name == "/DriverStation/Enabled" \
                      else record.getDouble()
                raw.setdefault(name, []).append((ts, val))
            except Exception:
                pass

    intervals = _build_enabled_intervals(raw.get("/DriverStation/Enabled", []))

    module_pairs = {}
    for i in range(4):
        vk = f"/Drive/Module{i}/DriveVelocityRadPerSec"
        ak = f"/Drive/Module{i}/DriveCurrentAmps"

        vel_en = [(t, abs(v)) for t, v in raw.get(vk, []) if _is_enabled(t, intervals)]
        amp_map = {t: v for t, v in raw.get(ak, []) if _is_enabled(t, intervals)}
        amp_ts  = sorted(amp_map)

        pairs = []
        for t, v in vel_en:
            if not amp_ts:
                break
            best = _nearest_amp(amp_ts, t)
            if best is not None:
                pairs.append((v, amp_map[best]))
        module_pairs[i] = pairs

    return module_pairs


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyze(module_pairs, match_name, vel_min, slip_vel, slip_amp):
    """Compute CPR, correlation, slip events, and velocity signature per module."""
    results = {"match": match_name}

    for i in range(4):
        pairs = module_pairs[i]
        if not pairs:
            results[i] = None
            continue

        vels = [v for v, a in pairs]
        amps = [a for v, a in pairs]

        # Current-per-rad/s (only when meaningfully moving)
        moving   = [(v, a) for v, a in pairs if v > vel_min]
        cpr_vals = [a / v for v, a in moving] if moving else []

        # Pearson correlation: strong positive = healthy load coupling
        corr = _pearson(vels, amps) if len(vels) >= 10 else None

        # Slip events: fast wheel + low current
        high_vel   = [(v, a) for v, a in pairs if v > slip_vel]
        slip_evts  = [(v, a) for v, a in high_vel if a < slip_amp]
        slip_rate  = len(slip_evts) / len(high_vel) if high_vel else 0.0

        # Velocity signature: median current in each speed bin
        bin_medians = {}
        for lo, hi in VEL_BINS:
            bin_amps = sorted(a for v, a in pairs if lo <= v < hi)
            if bin_amps:
                bin_medians[(lo, hi)] = bin_amps[len(bin_amps) // 2]

        results[i] = {
            "cpr":        _stats(cpr_vals),
            "corr":       corr,
            "slip_rate":  slip_rate,
            "slip_events": len(slip_evts),
            "slip_total":  len(high_vel),
            "bin_medians": bin_medians,
        }

    return results


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
class Report:
    def __init__(self, output_path=None):
        self._out = open(output_path, "w", encoding="utf-8") if output_path else None

    def line(self, text=""):
        print(text)
        if self._out:
            self._out.write(text + "\n")

    def sep(self, char="=", width=92):
        self.line(char * width)

    def close(self):
        if self._out:
            self._out.close()


def _cpr_flag(ratio):
    if ratio is None:
        return ""
    if ratio < CPR_LOW_CRIT:
        return "  [CRIT — likely mechanical issue]"
    if ratio < CPR_LOW_WARN:
        return "  [LOW]"
    return ""


def print_report(all_results, rpt, slip_vel, slip_amp, focus_module):
    SEP = "=" * 92

    # ── CPR median table ──────────────────────────────────────────────────────
    rpt.sep()
    rpt.line("CURRENT-PER-VELOCITY (CPR) MEDIAN  (amps / rad/s, when moving)")
    rpt.line("Lower value = spinning fast for fewer amps = possible slip or free-wheel")
    rpt.line(f'{"Match":<15} {"FL":>11} {"FR":>11} {"BR":>11} {"BL":>11}')
    rpt.line("-" * 58)
    for r in all_results:
        vals = []
        for i in range(4):
            d = r[i]
            vals.append(f'{d["cpr"]["median"]:11.4f}' if d and d["cpr"] else "        N/A")
        rpt.line(f'{r["match"]:<15} {" ".join(vals)}')

    # ── CPR summary: focus module vs others ──────────────────────────────────
    rpt.line()
    rpt.sep()
    mn = MODULE_NAMES[focus_module]
    rpt.line(f"CPR RATIO: {mn} vs AVERAGE OF OTHER 3 MODULES")
    rpt.line(f"  Ratio < {CPR_LOW_WARN} = low (inspect), < {CPR_LOW_CRIT} = critical (likely damage)")
    rpt.line(f'{"Match":<15} {"FL":>9} {"FR":>9} {"BR":>9} {"BL":>9} | {mn+"/Avg3":>9}')
    rpt.line("-" * 72)
    for r in all_results:
        cprs = [r[i]["cpr"]["median"] if (r[i] and r[i]["cpr"]) else None for i in range(4)]
        others  = [v for j, v in enumerate(cprs) if j != focus_module and v is not None]
        avg3    = sum(others) / len(others) if others else None
        focused = cprs[focus_module]
        ratio   = focused / avg3 if (focused and avg3) else None
        strs    = [f'{v:9.4f}' if v else '      N/A' for v in cprs]
        ratio_s = f'{ratio:9.3f}' if ratio else '      N/A'
        rpt.line(f'{r["match"]:<15} {" ".join(strs)} | {ratio_s}{_cpr_flag(ratio)}')

    # ── Pearson correlation ───────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line("VELOCITY-CURRENT PEARSON CORRELATION")
    rpt.line("  Measures how tightly current tracks velocity.  Healthy: ~0.25-0.40 (FRC swerve)")
    rpt.line("  Very low = weak load coupling = wheel may not be engaging carpet")
    rpt.line(f'{"Match":<15} {"FL r":>8} {"FR r":>8} {"BR r":>8} {"BL r":>8}')
    rpt.line("-" * 50)
    for r in all_results:
        vals = []
        for i in range(4):
            d = r[i]
            vals.append(f'{d["corr"]:8.3f}' if (d and d["corr"] is not None) else "     N/A")
        rpt.line(f'{r["match"]:<15} {" ".join(vals)}')

    # ── Slip events ───────────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line(f"SLIP EVENTS  (|vel| > {slip_vel:.0f} rad/s  AND  current < {slip_amp:.0f}A)")
    rpt.line(f'{"Match":<15} {"FL%":>9} {"FR%":>9} {"BR%":>9} {"BL%":>9}')
    rpt.line("-" * 54)
    for r in all_results:
        vals = []
        for i in range(4):
            d = r[i]
            vals.append(f'{d["slip_rate"]*100:9.1f}%' if d else "      N/A")
        rpt.line(f'{r["match"]:<15} {" ".join(vals)}')

    # ── Velocity signature ────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line("VELOCITY SIGNATURE — MEDIAN CURRENT (A) AT EACH SPEED BIN")
    rpt.line("  A module consistently lower than others at the same speed = under-loaded")
    rpt.line()
    for r in all_results:
        rpt.line(f"  --- {r['match']} ---")
        header = f'  {"Bin (rad/s)":<14}'
        for i in range(4):
            header += f' {MODULE_NAMES[i]:>8}'
        rpt.line(header)
        for lo, hi in VEL_BINS:
            row = f'  {lo:>3}-{hi:<9}'
            any_data = False
            for i in range(4):
                d = r[i]
                if d and (lo, hi) in d["bin_medians"]:
                    row += f' {d["bin_medians"][(lo,hi)]:8.1f}'
                    any_data = True
                else:
                    row += f' {"---":>8}'
            if any_data:
                rpt.line(row)
        rpt.line()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Swerve wheel slip/free-spin analyzer — Team 6391"
    )
    parser.add_argument("directory", nargs="?", default=".",
                        help="Directory containing .wpilog files")
    parser.add_argument("-o", "--output", metavar="FILE",
                        help="Write report to this file")
    parser.add_argument("--no-practice", action="store_true",
                        help="Exclude practice match logs")
    parser.add_argument("--match", metavar="LABEL",
                        help='Analyze only this match, e.g. "Qual 5"')
    parser.add_argument("--module", type=int, default=3, metavar="N",
                        help="Module to highlight in CPR ratio summary (0=FL 1=FR 2=BR 3=BL, default: 3=BL)")
    parser.add_argument("--vel-thresh", type=float, default=DEFAULT_VEL_MIN,
                        metavar="RADPS", help=f"Min vel for CPR calc (default {DEFAULT_VEL_MIN})")
    parser.add_argument("--slip-vel", type=float, default=DEFAULT_SLIP_VEL,
                        metavar="RADPS", help=f"Vel threshold for slip events (default {DEFAULT_SLIP_VEL})")
    parser.add_argument("--slip-amp", type=float, default=DEFAULT_SLIP_AMP,
                        metavar="AMPS", help=f"Max amps at slip-vel to flag (default {DEFAULT_SLIP_AMP})")
    args = parser.parse_args()

    log_dir = os.path.abspath(args.directory)
    if not os.path.isdir(log_dir):
        print(f"ERROR: {log_dir} is not a directory", file=sys.stderr)
        sys.exit(1)

    if args.module not in MODULE_NAMES:
        print(f"ERROR: --module must be 0-3", file=sys.stderr)
        sys.exit(1)

    # Discover logs
    PHASE_MAP = {"p": "Practice", "q": "Qual", "e": "Elim"}
    log_entries = []
    for fn in sorted(os.listdir(log_dir)):
        if not fn.endswith(".wpilog"):
            continue
        m = re.search(r"_(p|q|e)(\d+)\.wpilog$", fn)
        phase_key = m.group(1) if m else "?"
        match_name = f"{PHASE_MAP.get(phase_key, '?')} {m.group(2)}" if m \
                     else fn.replace(".wpilog", "")
        if args.no_practice and phase_key == "p":
            continue
        if args.match and match_name != args.match:
            continue
        log_entries.append((fn, match_name))

    if not log_entries:
        print(f"No matching .wpilog files found in {log_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Parsing {len(log_entries)} log(s) from {log_dir} ...", file=sys.stderr)
    all_results = []
    for fn, match_name in log_entries:
        try:
            pairs = parse_log(os.path.join(log_dir, fn),
                              args.vel_thresh, args.slip_vel, args.slip_amp)
            r = analyze(pairs, match_name, args.vel_thresh, args.slip_vel, args.slip_amp)
            all_results.append(r)
            print(f"  OK  {fn}", file=sys.stderr)
        except Exception as ex:
            print(f"  ERR {fn}: {ex}", file=sys.stderr)

    if not all_results:
        print("No logs parsed successfully.", file=sys.stderr)
        sys.exit(1)

    rpt = Report(output_path=args.output)
    rpt.sep()
    rpt.line(f"SWERVE SLIP ANALYSIS — {len(all_results)} match(es) from {log_dir}")
    rpt.line(f"Focus module: {MODULE_NAMES[args.module]} (index {args.module})  |  "
             f"Slip threshold: vel>{args.slip_vel:.0f} rad/s & amps<{args.slip_amp:.0f}A")
    rpt.sep()
    print_report(all_results, rpt, args.slip_vel, args.slip_amp, args.module)
    rpt.close()

    if args.output:
        print(f"\nReport written to: {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
