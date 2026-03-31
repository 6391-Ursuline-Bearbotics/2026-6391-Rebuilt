#!/usr/bin/env python3
"""
FRC Log Analyzer — Team 6391
Analyzes AdvantageKit .wpilog files for swerve drive health, speed/acceleration
trends, battery performance, and loop timing across a competition.

Usage:
    python log_analyzer.py [log_directory] [options]

Options:
    -o / --output FILE   Write report to a text file in addition to stdout
    -w / --wheel RADIUS  Wheel radius in inches (default: 2.0)
    --no-practice        Exclude practice match logs from analysis
    --match LABEL        Analyze only this match (e.g. "Qual 5")
"""

import argparse
import os
import re
import sys
from wpiutil.log import DataLogReader

# ---------------------------------------------------------------------------
# Configuration / thresholds
# ---------------------------------------------------------------------------
MODULE_NAMES = {0: "FL", 1: "FR", 2: "BR", 3: "BL"}
DEFAULT_WHEEL_RADIUS_IN = 2.0   # inches; SDS MK4i L2 = ~2.0"

THRESHOLDS = {
    "current_mean_diff_warn":  3.0,   # A  — inter-module mean difference
    "current_mean_diff_crit":  6.0,   # A
    "current_p95_diff_warn":   8.0,   # A
    "current_p95_diff_crit":  20.0,   # A
    "drive_temp_warn":        50.0,   # °C
    "drive_temp_crit":        60.0,   # °C
    "turn_temp_warn":         50.0,   # °C
    "turn_temp_crit":         60.0,   # °C
    "battery_min_warn":        8.0,   # V
    "battery_min_crit":        7.0,   # V
    "battery_mean_warn":      11.0,   # V  — low avg suggests weak battery
    "loop_mean_warn":         12.0,   # ms
    "loop_max_warn":          20.0,   # ms — 1 loop period
    "accel_drop_pct":         20.0,   # % drop vs match-1 to flag regression
    "short_match_thresh":     60.0,   # s  — flag as abnormally short
}

WANTED_DOUBLE_KEYS = set(
    [f"/Drive/Module{i}/DriveCurrentAmps" for i in range(4)] +
    [f"/Drive/Module{i}/TurnCurrentAmps"  for i in range(4)] +
    [f"/Drive/Module{i}/DriveTempCelsius" for i in range(4)] +
    [f"/Drive/Module{i}/TurnTempCelsius"  for i in range(4)] +
    [f"/Drive/Module{i}/DriveVelocityRadPerSec" for i in range(4)] +
    [
        "/SystemStats/BatteryVoltage",
        "/SystemStats/BatteryCurrent",
        "/RealOutputs/LoggedRobot/UserCodeMS",
    ]
)
WANTED_BOOL_KEYS = {"/DriverStation/Enabled", "/DriverStation/Autonomous"}


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------
def _stats(vals):
    if not vals:
        return None
    s = sorted(vals)
    n = len(s)
    return {
        "mean": sum(s) / n,
        "max":  s[-1],
        "min":  s[0],
        "p95":  s[int(0.95 * n)],
        "n":    n,
    }


def parse_log(filepath, match_name, wheel_radius_m):
    reader = DataLogReader(filepath)
    wanted_ids = {}
    raw = {}          # key -> [(ts_sec, value)]
    raw_all = {}      # key -> [(ts_sec, value)]  — unfiltered (for temps)

    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            if sd.name in WANTED_DOUBLE_KEYS or sd.name in WANTED_BOOL_KEYS:
                wanted_ids[sd.entry] = sd.name
        elif record.isControl():
            continue
        else:
            eid = record.getEntry()
            if eid not in wanted_ids:
                continue
            name = wanted_ids[eid]
            ts = record.getTimestamp() / 1e6
            try:
                if name in WANTED_BOOL_KEYS:
                    val = record.getBoolean()
                else:
                    val = record.getDouble()
                raw_all.setdefault(name, []).append((ts, val))
            except Exception:
                pass

    # --- Build enabled intervals ---
    enabled_times = []
    last_start = None
    last_state = False
    last_ts = 0.0
    for ts, val in raw_all.get("/DriverStation/Enabled", []):
        last_ts = ts
        if val and not last_state:
            last_start = ts
        elif not val and last_state and last_start is not None:
            enabled_times.append((last_start, ts))
        last_state = val
    if last_state and last_start is not None:
        enabled_times.append((last_start, last_ts))

    def is_enabled(t):
        for s, e in enabled_times:
            if s <= t <= e:
                return True
        return False

    # --- Filter raw to enabled periods ---
    for key, series in raw_all.items():
        raw[key] = [(t, v) for t, v in series if is_enabled(t)]

    enabled_duration = sum(e - s for s, e in enabled_times)

    # --- Per-module stats (current, speed — enabled only) ---
    module_stats = {}
    for i in range(4):
        name = MODULE_NAMES[i]
        drive_a = [v for _, v in raw.get(f"/Drive/Module{i}/DriveCurrentAmps", [])]
        turn_a  = [v for _, v in raw.get(f"/Drive/Module{i}/TurnCurrentAmps",  [])]
        speed   = [abs(v) * wheel_radius_m for _, v in raw.get(f"/Drive/Module{i}/DriveVelocityRadPerSec", [])]
        # temperature: use all records (not filtered), take max across full log
        drive_t = [v for _, v in raw_all.get(f"/Drive/Module{i}/DriveTempCelsius", [])]
        turn_t  = [v for _, v in raw_all.get(f"/Drive/Module{i}/TurnTempCelsius",  [])]
        module_stats[i] = {
            "name":      name,
            "drive_a":   _stats(drive_a),
            "turn_a":    _stats(turn_a),
            "speed":     _stats(speed),
            "drive_temp_max": max(drive_t) if drive_t else None,
            "turn_temp_max":  max(turn_t)  if turn_t  else None,
        }

    # --- Battery ---
    batt_v = [v for _, v in raw.get("/SystemStats/BatteryVoltage", [])]
    batt_a = [v for _, v in raw.get("/SystemStats/BatteryCurrent", [])]

    # --- Loop time ---
    loop_ms = [v for _, v in raw.get("/RealOutputs/LoggedRobot/UserCodeMS", [])]

    # --- Acceleration from avg speed time series ---
    speed_by_ts = {}
    for i in range(4):
        for ts, v in raw.get(f"/Drive/Module{i}/DriveVelocityRadPerSec", []):
            speed_by_ts.setdefault(ts, []).append(abs(v) * wheel_radius_m)
    ts_speed = sorted(
        (ts, sum(vs) / len(vs))
        for ts, vs in speed_by_ts.items()
        if len(vs) >= 2
    )
    accels = []
    for i in range(1, len(ts_speed)):
        t0, s0 = ts_speed[i - 1]
        t1, s1 = ts_speed[i]
        dt = t1 - t0
        if 0 < dt <= 0.5:
            a = (s1 - s0) / dt
            # Clamp implausible spikes (odometry noise / init jumps)
            if abs(a) < 30.0:
                accels.append(a)

    pos_accels = [a for a in accels if a > 0]
    neg_accels = [abs(a) for a in accels if a < 0]

    return {
        "match":            match_name,
        "enabled_duration": enabled_duration,
        "modules":          module_stats,
        "battery_v":        _stats(batt_v),
        "battery_a":        _stats(batt_a),
        "loop":             _stats(loop_ms),
        "accel":            _stats(pos_accels),
        "decel":            _stats(neg_accels),
        "accel_all_abs":    _stats([abs(a) for a in accels]),
    }


# ---------------------------------------------------------------------------
# Report helpers
# ---------------------------------------------------------------------------
def _fmt(val, fmt=".2f", none="   N/A"):
    return format(val, fmt) if val is not None else none


class Report:
    def __init__(self, output_path=None):
        self._lines = []
        self._out = open(output_path, "w", encoding="utf-8") if output_path else None

    def line(self, text=""):
        print(text)
        self._lines.append(text)
        if self._out:
            self._out.write(text + "\n")

    def sep(self, char="=", width=90):
        self.line(char * width)

    def close(self):
        if self._out:
            self._out.close()


def _diff_flag(diff, warn, crit, warn_label="WARN", crit_label="CRIT"):
    if diff >= crit:
        return f"  [{crit_label}]"
    if diff >= warn:
        return f"  [{warn_label}]"
    return ""


# ---------------------------------------------------------------------------
# Main report sections
# ---------------------------------------------------------------------------
def report_drive_current(results, rpt):
    T = THRESHOLDS
    rpt.sep()
    rpt.line("SWERVE DRIVE CURRENT — MEAN (amps, enabled periods only)")
    rpt.line(f'{"Match":<15} {"FL":>8} {"FR":>8} {"BR":>8} {"BL":>8} | {"Spread":>7}')
    rpt.line("-" * 65)
    for r in results:
        means = [r["modules"][i]["drive_a"]["mean"] if r["modules"][i]["drive_a"] else None for i in range(4)]
        valid = [v for v in means if v is not None]
        spread = max(valid) - min(valid) if len(valid) > 1 else 0
        strs = [f'{v:8.2f}' if v is not None else "     N/A" for v in means]
        flag = _diff_flag(spread, T["current_mean_diff_warn"], T["current_mean_diff_crit"])
        rpt.line(f'{r["match"]:<15} {" ".join(strs)} | {spread:7.2f}{flag}')

    rpt.line()
    rpt.sep()
    rpt.line("SWERVE DRIVE CURRENT — 95th PERCENTILE (amps)")
    rpt.line(f'{"Match":<15} {"FL":>8} {"FR":>8} {"BR":>8} {"BL":>8} | {"Spread":>7}')
    rpt.line("-" * 65)
    for r in results:
        p95s = [r["modules"][i]["drive_a"]["p95"] if r["modules"][i]["drive_a"] else None for i in range(4)]
        valid = [v for v in p95s if v is not None]
        spread = max(valid) - min(valid) if len(valid) > 1 else 0
        strs = [f'{v:8.2f}' if v is not None else "     N/A" for v in p95s]
        flag = _diff_flag(spread, T["current_p95_diff_warn"], T["current_p95_diff_crit"])
        rpt.line(f'{r["match"]:<15} {" ".join(strs)} | {spread:7.2f}{flag}')

    rpt.line()
    rpt.sep()
    rpt.line("SWERVE DRIVE CURRENT — PEAK (amps)")
    rpt.line(f'{"Match":<15} {"FL":>8} {"FR":>8} {"BR":>8} {"BL":>8}')
    rpt.line("-" * 55)
    for r in results:
        strs = [f'{r["modules"][i]["drive_a"]["max"]:8.1f}' if r["modules"][i]["drive_a"] else "     N/A" for i in range(4)]
        rpt.line(f'{r["match"]:<15} {" ".join(strs)}')


def report_temperatures(results, rpt):
    T = THRESHOLDS
    rpt.line()
    rpt.sep()
    rpt.line(f'DRIVE MOTOR TEMPERATURE — PEAK (°C)   [warn >{T["drive_temp_warn"]:.0f}, crit >{T["drive_temp_crit"]:.0f}]')
    rpt.line(f'{"Match":<15} {"FL":>8} {"FR":>8} {"BR":>8} {"BL":>8}')
    rpt.line("-" * 55)
    for r in results:
        strs = []
        flags = []
        for i in range(4):
            t = r["modules"][i]["drive_temp_max"]
            if t is not None:
                strs.append(f'{t:8.1f}')
                if t >= T["drive_temp_crit"]:
                    flags.append(f'{MODULE_NAMES[i]}:CRIT')
                elif t >= T["drive_temp_warn"]:
                    flags.append(f'{MODULE_NAMES[i]}:WARM')
            else:
                strs.append("     N/A")
        flag_str = "  [" + ", ".join(flags) + "]" if flags else ""
        rpt.line(f'{r["match"]:<15} {" ".join(strs)}{flag_str}')

    rpt.line()
    rpt.sep()
    rpt.line(f'TURN MOTOR TEMPERATURE — PEAK (°C)    [warn >{T["turn_temp_warn"]:.0f}, crit >{T["turn_temp_crit"]:.0f}]')
    rpt.line(f'{"Match":<15} {"FL":>8} {"FR":>8} {"BR":>8} {"BL":>8}')
    rpt.line("-" * 55)
    for r in results:
        strs = []
        flags = []
        for i in range(4):
            t = r["modules"][i]["turn_temp_max"]
            if t is not None:
                strs.append(f'{t:8.1f}')
                if t >= T["turn_temp_crit"]:
                    flags.append(f'{MODULE_NAMES[i]}:CRIT')
                elif t >= T["turn_temp_warn"]:
                    flags.append(f'{MODULE_NAMES[i]}:WARM')
            else:
                strs.append("     N/A")
        flag_str = "  [" + ", ".join(flags) + "]" if flags else ""
        rpt.line(f'{r["match"]:<15} {" ".join(strs)}{flag_str}')


def report_speed(results, rpt):
    rpt.line()
    rpt.sep()
    rpt.line("WHEEL SPEED — MEAN m/s (enabled, absolute value)")
    rpt.line(f'{"Match":<15} {"FL":>7} {"FR":>7} {"BR":>7} {"BL":>7} | {"Avg":>7} {"Peak":>7}')
    rpt.line("-" * 70)
    for r in results:
        means = [r["modules"][i]["speed"]["mean"] if r["modules"][i]["speed"] else None for i in range(4)]
        peaks = [r["modules"][i]["speed"]["max"]  if r["modules"][i]["speed"] else None for i in range(4)]
        valid_m = [v for v in means if v is not None]
        valid_p = [v for v in peaks if v is not None]
        avg  = sum(valid_m) / len(valid_m) if valid_m else 0
        peak = max(valid_p) if valid_p else 0
        strs = [f'{v:7.2f}' if v is not None else "    N/A" for v in means]
        rpt.line(f'{r["match"]:<15} {" ".join(strs)} | {avg:7.2f} {peak:7.2f}')


def report_acceleration(results, rpt):
    T = THRESHOLDS
    rpt.line()
    rpt.sep()
    rpt.line("ACCELERATION — MEAN m/s² (enabled, clamped at ±30 to remove noise)")
    rpt.line(f'{"Match":<15} {"MnAccel":>9} {"P95Acl":>9} {"MnDecel":>9} {"P95Dcl":>9}')
    rpt.line("-" * 58)
    prev_accel = None
    for r in results:
        a  = r["accel"]
        d  = r["decel"]
        a_mean  = a["mean"] if a else None
        a_p95   = a["p95"]  if a else None
        d_mean  = d["mean"] if d else None
        d_p95   = d["p95"]  if d else None
        flag = ""
        if a_mean is not None and prev_accel is not None:
            drop_pct = (prev_accel - a_mean) / prev_accel * 100 if prev_accel > 0 else 0
            if drop_pct >= T["accel_drop_pct"]:
                flag = f"  [DROP {drop_pct:.0f}% vs prev]"
        prev_accel = a_mean
        rpt.line(
            f'{r["match"]:<15} {_fmt(a_mean):>9} {_fmt(a_p95):>9} {_fmt(d_mean):>9} {_fmt(d_p95):>9}{flag}'
        )

    rpt.line()
    rpt.line("ACCELERATION TREND (mean m/s², bar chart):")
    rpt.line("-" * 60)
    max_accel = max((r["accel"]["mean"] for r in results if r["accel"]), default=1)
    for r in results:
        a = r["accel"]
        if a:
            bar_len = int(a["mean"] / max_accel * 40)
            bar = "#" * bar_len
            rpt.line(f'{r["match"]:<15} {a["mean"]:5.2f} |{bar}')
        else:
            rpt.line(f'{r["match"]:<15}   N/A')


def report_battery(results, rpt):
    T = THRESHOLDS
    rpt.line()
    rpt.sep()
    rpt.line("BATTERY VOLTAGE (enabled period)")
    rpt.line(f'{"Match":<15} {"MeanV":>8} {"MinV":>8} {"MaxV":>8}')
    rpt.line("-" * 45)
    for r in results:
        bv = r["battery_v"]
        if bv:
            flag = ""
            if bv["min"] < T["battery_min_crit"]:
                flag = "  [CRIT: deep sag]"
            elif bv["min"] < T["battery_min_warn"]:
                flag = "  [WARN: sag <8V]"
            if bv["mean"] < T["battery_mean_warn"] and not flag:
                flag = "  [WARN: low avg — weak battery?]"
            rpt.line(f'{r["match"]:<15} {bv["mean"]:8.2f} {bv["min"]:8.2f} {bv["max"]:8.2f}{flag}')
        else:
            rpt.line(f'{r["match"]:<15}      N/A')


def report_loop_time(results, rpt):
    T = THRESHOLDS
    rpt.line()
    rpt.sep()
    rpt.line("ROBORIO LOOP TIME — UserCodeMS (enabled periods)")
    rpt.line(f'{"Match":<15} {"Mean ms":>9} {"Max ms":>9} {"P95 ms":>9}')
    rpt.line("-" * 48)
    for r in results:
        lt = r["loop"]
        if lt:
            flag = ""
            if lt["max"] > T["loop_max_warn"]:
                flag = f'  [OVERRUN: max {lt["max"]:.0f}ms]'
            elif lt["mean"] > T["loop_mean_warn"]:
                flag = f'  [WARN: high mean]'
            rpt.line(f'{r["match"]:<15} {lt["mean"]:9.2f} {lt["max"]:9.2f} {lt["p95"]:9.2f}{flag}')
        else:
            rpt.line(f'{r["match"]:<15}       N/A       N/A       N/A')


def report_duration(results, rpt):
    T = THRESHOLDS
    rpt.line()
    rpt.sep()
    rpt.line("ENABLED DURATION")
    rpt.line("-" * 35)
    for r in results:
        dur = r["enabled_duration"]
        flag = "  [SHORT — match issue?]" if dur < T["short_match_thresh"] else ""
        rpt.line(f'{r["match"]:<15} {dur:6.1f}s{flag}')


def report_anomaly_summary(results, rpt):
    """Collect and print all flagged anomalies in one place."""
    T = THRESHOLDS
    anomalies = []  # list of (match, severity, description)

    for r in results:
        match = r["match"]
        dur   = r["enabled_duration"]

        if dur < T["short_match_thresh"]:
            anomalies.append((match, "WARN", f"Only {dur:.0f}s enabled — match did not complete normally"))

        # Module current spread
        means = [r["modules"][i]["drive_a"]["mean"] if r["modules"][i]["drive_a"] else None for i in range(4)]
        p95s  = [r["modules"][i]["drive_a"]["p95"]  if r["modules"][i]["drive_a"] else None for i in range(4)]
        valid_m = [v for v in means if v is not None]
        valid_p = [v for v in p95s  if v is not None]
        if len(valid_m) > 1:
            spread_m = max(valid_m) - min(valid_m)
            if spread_m >= T["current_mean_diff_crit"]:
                low_mod = MODULE_NAMES[means.index(min(valid_m))]
                high_mod = MODULE_NAMES[means.index(max(valid_m))]
                anomalies.append((match, "CRIT", f"Drive current spread {spread_m:.1f}A (mean) — {high_mod} high, {low_mod} low"))
            elif spread_m >= T["current_mean_diff_warn"]:
                anomalies.append((match, "WARN", f"Drive current spread {spread_m:.1f}A (mean) between modules"))
        if len(valid_p) > 1:
            spread_p = max(valid_p) - min(valid_p)
            if spread_p >= T["current_p95_diff_crit"]:
                low_mod  = MODULE_NAMES[p95s.index(min(valid_p))]
                high_mod = MODULE_NAMES[p95s.index(max(valid_p))]
                anomalies.append((match, "CRIT", f"Drive current spread {spread_p:.1f}A (p95) — {high_mod} high, {low_mod} low — likely mechanical issue"))
            elif spread_p >= T["current_p95_diff_warn"]:
                low_mod  = MODULE_NAMES[p95s.index(min(valid_p))]
                high_mod = MODULE_NAMES[p95s.index(max(valid_p))]
                anomalies.append((match, "WARN", f"Drive current spread {spread_p:.1f}A (p95) — {high_mod} vs {low_mod}"))

        # Temperatures
        for i in range(4):
            dt = r["modules"][i]["drive_temp_max"]
            tt = r["modules"][i]["turn_temp_max"]
            if dt is not None:
                if dt >= T["drive_temp_crit"]:
                    anomalies.append((match, "CRIT", f"{MODULE_NAMES[i]} drive motor temp {dt:.0f}°C (critical)"))
                elif dt >= T["drive_temp_warn"]:
                    anomalies.append((match, "WARN", f"{MODULE_NAMES[i]} drive motor temp {dt:.0f}°C"))
            if tt is not None:
                if tt >= T["turn_temp_crit"]:
                    anomalies.append((match, "CRIT", f"{MODULE_NAMES[i]} turn motor temp {tt:.0f}°C (critical)"))
                elif tt >= T["turn_temp_warn"]:
                    anomalies.append((match, "WARN", f"{MODULE_NAMES[i]} turn motor temp {tt:.0f}°C"))

        # Battery
        bv = r["battery_v"]
        if bv:
            if bv["min"] < T["battery_min_crit"]:
                anomalies.append((match, "CRIT", f"Battery sagged to {bv['min']:.2f}V — risk of brownout"))
            elif bv["min"] < T["battery_min_warn"]:
                anomalies.append((match, "WARN", f"Battery sagged to {bv['min']:.2f}V"))
            if bv["mean"] < T["battery_mean_warn"]:
                anomalies.append((match, "WARN", f"Battery mean voltage only {bv['mean']:.2f}V — consider swapping battery"))

        # Loop time
        lt = r["loop"]
        if lt and lt["max"] > T["loop_max_warn"]:
            anomalies.append((match, "WARN", f"Loop overrun: max UserCodeMS = {lt['max']:.0f}ms"))

    rpt.line()
    rpt.sep("*")
    rpt.line("ANOMALY SUMMARY")
    rpt.sep("*")
    if not anomalies:
        rpt.line("  No anomalies detected.")
    else:
        crits = [(m, sev, desc) for m, sev, desc in anomalies if sev == "CRIT"]
        warns = [(m, sev, desc) for m, sev, desc in anomalies if sev == "WARN"]
        if crits:
            rpt.line(f"\n  CRITICAL ({len(crits)} issues):")
            for match, _, desc in crits:
                rpt.line(f"    [{match}] {desc}")
        if warns:
            rpt.line(f"\n  WARNINGS ({len(warns)} issues):")
            for match, _, desc in warns:
                rpt.line(f"    [{match}] {desc}")
    rpt.sep("*")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="FRC AdvantageKit log analyzer — Team 6391")
    parser.add_argument(
        "directory",
        nargs="?",
        default=".",
        help="Directory containing .wpilog files (default: current directory)"
    )
    parser.add_argument("-o", "--output", metavar="FILE", help="Write report to this file")
    parser.add_argument(
        "-w", "--wheel", metavar="RADIUS_IN", type=float, default=DEFAULT_WHEEL_RADIUS_IN,
        help=f"Wheel radius in inches (default: {DEFAULT_WHEEL_RADIUS_IN})"
    )
    parser.add_argument("--no-practice", action="store_true", help="Exclude practice match logs")
    parser.add_argument("--match", metavar="LABEL", help='Analyze only this match, e.g. "Qual 5"')
    args = parser.parse_args()

    log_dir = os.path.abspath(args.directory)
    if not os.path.isdir(log_dir):
        print(f"ERROR: {log_dir} is not a directory", file=sys.stderr)
        sys.exit(1)

    wheel_radius_m = args.wheel * 0.0254  # inches -> meters

    # --- Discover log files ---
    PHASE_MAP = {"p": "Practice", "q": "Qual", "e": "Elim"}
    log_entries = []
    for fn in sorted(os.listdir(log_dir)):
        if not fn.endswith(".wpilog"):
            continue
        m = re.search(r"_(p|q|e)(\d+)\.wpilog$", fn)
        if m:
            phase_key = m.group(1)
            num = int(m.group(2))
            phase = PHASE_MAP[phase_key]
            match_name = f"{phase} {num}"
        else:
            # Unknown naming — include anyway
            phase_key = "?"
            match_name = fn.replace(".wpilog", "")
        if args.no_practice and phase_key == "p":
            continue
        if args.match and match_name != args.match:
            continue
        log_entries.append((fn, match_name))

    if not log_entries:
        print(f"No matching .wpilog files found in {log_dir}", file=sys.stderr)
        sys.exit(1)

    # --- Parse ---
    print(f"Parsing {len(log_entries)} log file(s) from {log_dir} ...", file=sys.stderr)
    results = []
    for fn, match_name in log_entries:
        fp = os.path.join(log_dir, fn)
        try:
            r = parse_log(fp, match_name, wheel_radius_m)
            results.append(r)
            print(f"  OK  {fn}", file=sys.stderr)
        except Exception as ex:
            print(f"  ERR {fn}: {ex}", file=sys.stderr)

    if not results:
        print("No logs parsed successfully.", file=sys.stderr)
        sys.exit(1)

    # --- Report ---
    rpt = Report(output_path=args.output)
    rpt.sep()
    rpt.line(f"FRC LOG ANALYSIS — {len(results)} match(es) from {log_dir}")
    rpt.line(f"Wheel radius: {args.wheel:.2f} in  |  Matches: {', '.join(r['match'] for r in results)}")
    rpt.sep()

    report_drive_current(results, rpt)
    report_temperatures(results, rpt)
    report_speed(results, rpt)
    report_acceleration(results, rpt)
    report_battery(results, rpt)
    report_loop_time(results, rpt)
    report_duration(results, rpt)
    report_anomaly_summary(results, rpt)

    rpt.close()
    if args.output:
        print(f"\nReport written to: {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
