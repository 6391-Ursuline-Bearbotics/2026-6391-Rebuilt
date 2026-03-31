#!/usr/bin/env python3
"""
Mechanism Motor Analyzer — Team 6391
Analyzes all non-drive subsystem motors from AdvantageKit .wpilog files.

Subsystems covered:
  - Shooter     : Left/Right flywheels — current balance, CPR, velocity imbalance, temps
  - Indexer Belt: Belt motor — current, jam events (high current + low velocity)
  - Indexer Kicker: Kicker motor — current trends
  - Indexer Spinners: Left/Right — current balance
  - Intake Deploy: Deploy motor — current, hard-stop spike count
  - Intake Roller: Roller motor — current, jams, motor disconnections
  - PDH         : Total robot current and power
  - Alerts      : All active WPILib Alert messages during enabled periods

JAM DETECTION:
  Belt / Roller: StatorCurrent > threshold AND |velocity| < threshold
  These are samples where the motor is being commanded but the mechanism
  is stalled — game piece stuck, chain skipped, etc.

HARD STOP (Deploy):
  StatorCurrent > 30A indicates the deploy arm has hit its physical hard stop.
  High counts mean the arm is being commanded past its travel frequently.

DISCONNECTION:
  A falling edge (True→False) on a Connected signal during an enabled period.
  Even one disconnect per match is worth investigating at championship.

Usage:
    python mechanism_analyzer.py [log_directory] [options]

Options:
    -o / --output FILE    Write report to a file in addition to stdout
    --no-practice         Exclude practice match logs
    --match LABEL         Analyze only one match (e.g. "Qual 5")

Thresholds (edit at top of file if needed):
    BELT_JAM_AMP    = 25A  — indexer belt stall current
    BELT_JAM_VEL    = 5    — rad/s below which belt is "stalled"
    ROLLER_JAM_AMP  = 25A  — intake roller stall current
    ROLLER_JAM_VEL  = 5    — rad/s
    DEPLOY_SPIKE_AMP= 30A  — intake deploy hard-stop current
    SHOOTER_HIGH_VEL= 50   — rad/s minimum to compute shooter CPR
    TEMP_WARN       = 55°C — motor temperature warning threshold
    SPINNER_SPREAD  = 2A   — current spread between spinner L/R to flag
    SHOOTER_SPREAD  = 3A   — current spread between shooter L/R to flag
"""

import argparse
import math
import os
import re
import sys
from wpiutil.log import DataLogReader

# ---------------------------------------------------------------------------
# Thresholds — tune these if the robot changes
# ---------------------------------------------------------------------------
BELT_JAM_AMP      = 25.0   # A    — belt jam detection: current above this...
BELT_JAM_VEL      = 5.0    # rad/s — ...while velocity is below this
ROLLER_JAM_AMP    = 25.0   # A
ROLLER_JAM_VEL    = 5.0    # rad/s
DEPLOY_SPIKE_AMP  = 30.0   # A    — intake deploy hard-stop stall current
SHOOTER_HIGH_VEL  = 50.0   # rad/s — minimum speed to compute shooter CPR
SHOOTER_SPIN_VEL  = 50.0   # rad/s — minimum speed for velocity imbalance calc
TEMP_WARN         = 55.0   # °C
SPINNER_SPREAD    = 2.0    # A    — L/R spinner current spread to flag
SHOOTER_SPREAD    = 3.0    # A    — L/R shooter mean current spread to flag
SHOOTER_CPR_RATIO = 1.08   # CPR L/R ratio above this = one side free-spinning

# ---------------------------------------------------------------------------
# Log key definitions
# ---------------------------------------------------------------------------
DOUBLE_KEYS = {
    # Shooter
    "sh_l_vel":  "/Shooter/LeftVelocityRadPerSec",
    "sh_r_vel":  "/Shooter/RightVelocityRadPerSec",
    "sh_l_cur":  "/Shooter/LeftStatorCurrentAmps",
    "sh_r_cur":  "/Shooter/RightStatorCurrentAmps",
    "sh_l_tmp":  "/Shooter/LeftTempCelsius",
    "sh_r_tmp":  "/Shooter/RightTempCelsius",
    # Indexer belt
    "ix_b_vel":  "/Indexer/Belt/VelocityRadPerSec",
    "ix_b_cur":  "/Indexer/Belt/StatorCurrentAmps",
    "ix_b_tmp":  "/Indexer/Belt/TempCelsius",
    # Indexer kicker
    "ix_k_vel":  "/Indexer/Kicker/VelocityRadPerSec",
    "ix_k_cur":  "/Indexer/Kicker/StatorCurrentAmps",
    "ix_k_tmp":  "/Indexer/Kicker/TempCelsius",
    # Indexer spinners
    "ix_sl_vel": "/Indexer/Spinners/LeftVelocityRPM",
    "ix_sr_vel": "/Indexer/Spinners/RightVelocityRPM",
    "ix_sl_cur": "/Indexer/Spinners/LeftCurrentAmps",
    "ix_sr_cur": "/Indexer/Spinners/RightCurrentAmps",
    # Intake deploy
    "in_d_vel":  "/Intake/Deploy/VelocityRadPerSec",
    "in_d_cur":  "/Intake/Deploy/StatorCurrentAmps",
    "in_d_tmp":  "/Intake/Deploy/TempCelsius",
    "in_d_pos":  "/Intake/Deploy/PositionRad",
    # Intake roller
    "in_r_vel":  "/Intake/Roller/VelocityRadPerSec",
    "in_r_cur":  "/Intake/Roller/StatorCurrentAmps",
    "in_r_tmp":  "/Intake/Roller/TempCelsius",
    # PDH
    "pdh_cur":   "/PowerDistribution/TotalCurrent",
    "pdh_pwr":   "/PowerDistribution/TotalPower",
    "pdh_vlt":   "/PowerDistribution/Voltage",
}

BOOL_KEYS = {
    "enabled":    "/DriverStation/Enabled",
    "sh_l_conn":  "/Shooter/LeftConnected",
    "sh_r_conn":  "/Shooter/RightConnected",
    "ix_b_conn":  "/Indexer/Belt/Connected",
    "ix_k_conn":  "/Indexer/Kicker/Connected",
    "ix_sl_conn": "/Indexer/Spinners/LeftConnected",
    "ix_sr_conn": "/Indexer/Spinners/RightConnected",
    "in_d_conn":  "/Intake/Deploy/Connected",
    "in_r_conn":  "/Intake/Roller/Connected",
}

STR_ARRAY_KEYS = {
    "al_err":  "/RealOutputs/Alerts/errors",
    "al_warn": "/RealOutputs/Alerts/warnings",
}

# Build a single path→short_key lookup
_ALL_KEY_MAP = {}
for _short, _path in {**DOUBLE_KEYS, **BOOL_KEYS, **STR_ARRAY_KEYS}.items():
    _ALL_KEY_MAP[_path] = _short


# ---------------------------------------------------------------------------
# Helpers
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
        "p5":   s[int(0.05 * n)],
        "n":    n,
    }


def _build_enabled(enabled_series):
    """Return (is_enabled_fn, total_enabled_seconds) from a raw boolean series."""
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

    def check(t):
        return any(s <= t <= e for s, e in intervals)

    return check, sum(e - s for s, e in intervals)


def _disc_count(series, is_en):
    """Count falling edges (True→False) during enabled periods."""
    en_series = [(t, v) for t, v in series if is_en(t)]
    return sum(
        1 for i in range(1, len(en_series))
        if en_series[i - 1][1] and not en_series[i][1]
    )


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------
def parse_log(filepath):
    """
    Parse one .wpilog and return a results dict with all mechanism stats.
    Temperatures are taken from the full log (not filtered to enabled) because
    TalonFX reports temperature at low frequency and may not update during
    short enabled windows.
    """
    reader    = DataLogReader(filepath)
    want_ids  = {}
    raw       = {}   # short_key -> [(ts_sec, value)]

    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            if sd.name in _ALL_KEY_MAP:
                want_ids[sd.entry] = sd.name
        elif record.isControl():
            continue
        else:
            eid = record.getEntry()
            if eid not in want_ids:
                continue
            name  = want_ids[eid]
            short = _ALL_KEY_MAP[name]
            ts    = record.getTimestamp() / 1e6
            try:
                if short in BOOL_KEYS:
                    val = record.getBoolean()
                elif name in STR_ARRAY_KEYS.values():
                    val = record.getStringArray()
                else:
                    val = record.getDouble()
                raw.setdefault(short, []).append((ts, val))
            except Exception:
                pass

    is_en, en_dur = _build_enabled(raw.get("enabled", []))

    # Helpers to filter by enabled state
    def en(key):    return [v for t, v in raw.get(key, []) if is_en(t)]
    def all_(key):  return [v for t, v in raw.get(key, [])]

    # ── Shooter ──────────────────────────────────────────────────────────────
    sh_l_vel = [abs(v) for v in en("sh_l_vel")]
    sh_r_vel = [abs(v) for v in en("sh_r_vel")]
    sh_l_cur = en("sh_l_cur")
    sh_r_cur = en("sh_r_cur")

    # CPR when both wheels are spinning above threshold
    sh_l_cpr = [a / v for v, a in zip(sh_l_vel, sh_l_cur) if v > SHOOTER_HIGH_VEL and a > 0]
    sh_r_cpr = [a / v for v, a in zip(sh_r_vel, sh_r_cur) if v > SHOOTER_HIGH_VEL and a > 0]

    # Velocity imbalance between L and R when both are spinning
    sh_pairs      = [(l, r) for l, r in zip(sh_l_vel, sh_r_vel)
                     if l > SHOOTER_SPIN_VEL and r > SHOOTER_SPIN_VEL]
    sh_vel_imbal  = [abs(l - r) / ((l + r) / 2) * 100 for l, r in sh_pairs]

    # Shot event proxy: samples where stator current spikes (ball being fed)
    sh_shot_events = sum(1 for v in sh_l_cur if v > 20)

    # ── Indexer belt ─────────────────────────────────────────────────────────
    ix_b_vel   = [abs(v) for v in en("ix_b_vel")]
    ix_b_cur   = en("ix_b_cur")
    # Jam = high current while belt is stalled
    ix_b_jams  = sum(1 for v, a in zip(ix_b_vel, ix_b_cur)
                     if a > BELT_JAM_AMP and v < BELT_JAM_VEL)

    # ── Indexer kicker ───────────────────────────────────────────────────────
    ix_k_cur = en("ix_k_cur")

    # ── Indexer spinners ─────────────────────────────────────────────────────
    ix_sl_vel = [abs(v) for v in en("ix_sl_vel")]
    ix_sr_vel = [abs(v) for v in en("ix_sr_vel")]
    ix_sl_cur = en("ix_sl_cur")
    ix_sr_cur = en("ix_sr_cur")

    sp_pairs = [(l, r) for l, r in zip(ix_sl_vel, ix_sr_vel) if l > 50 and r > 50]
    sp_imbal = [abs(l - r) / ((l + r) / 2) * 100 for l, r in sp_pairs]

    # ── Intake deploy ─────────────────────────────────────────────────────────
    in_d_cur    = en("in_d_cur")
    # Hard-stop events = motor stalling against physical stop
    in_d_spikes = sum(1 for v in in_d_cur if v > DEPLOY_SPIKE_AMP)

    # ── Intake roller ─────────────────────────────────────────────────────────
    in_r_vel  = [abs(v) for v in en("in_r_vel")]
    in_r_cur  = en("in_r_cur")
    in_r_jams = sum(1 for v, a in zip(in_r_vel, in_r_cur)
                    if a > ROLLER_JAM_AMP and v < ROLLER_JAM_VEL)

    # ── Alerts during enabled ─────────────────────────────────────────────────
    alert_errors, alert_warns = set(), set()
    for ts, msgs in raw.get("al_err", []):
        if is_en(ts):
            alert_errors.update(m for m in msgs if m)
    for ts, msgs in raw.get("al_warn", []):
        if is_en(ts):
            alert_warns.update(m for m in msgs if m)

    # ── Motor temperatures (full log max) ─────────────────────────────────────
    def tmp(key): return max(all_(key)) if all_(key) else None

    return {
        "en_dur":           en_dur,
        # Shooter
        "sh_l_cur":         _stats(sh_l_cur),
        "sh_r_cur":         _stats(sh_r_cur),
        "sh_l_cpr":         _stats(sh_l_cpr),
        "sh_r_cpr":         _stats(sh_r_cpr),
        "sh_vel_imbal":     _stats(sh_vel_imbal),
        "sh_shot_events":   sh_shot_events,
        "sh_l_tmp":         tmp("sh_l_tmp"),
        "sh_r_tmp":         tmp("sh_r_tmp"),
        "sh_l_disconnects": _disc_count(raw.get("sh_l_conn", []), is_en),
        "sh_r_disconnects": _disc_count(raw.get("sh_r_conn", []), is_en),
        # Indexer belt
        "ix_b_cur":         _stats(ix_b_cur),
        "ix_b_jams":        ix_b_jams,
        "ix_b_tmp":         tmp("ix_b_tmp"),
        # Indexer kicker
        "ix_k_cur":         _stats(ix_k_cur),
        "ix_k_tmp":         tmp("ix_k_tmp"),
        # Indexer spinners
        "ix_sl_cur":        _stats(ix_sl_cur),
        "ix_sr_cur":        _stats(ix_sr_cur),
        "sp_imbal":         _stats(sp_imbal),
        # Intake deploy
        "in_d_cur":         _stats(in_d_cur),
        "in_d_spikes":      in_d_spikes,
        "in_d_tmp":         tmp("in_d_tmp"),
        "in_d_disconnects": _disc_count(raw.get("in_d_conn", []), is_en),
        # Intake roller
        "in_r_cur":         _stats(in_r_cur),
        "in_r_jams":        in_r_jams,
        "in_r_tmp":         tmp("in_r_tmp"),
        "in_r_disconnects": _disc_count(raw.get("in_r_conn", []), is_en),
        # PDH
        "pdh_cur":          _stats(en("pdh_cur")),
        "pdh_pwr":          _stats(en("pdh_pwr")),
        # Alerts
        "alert_errors":     alert_errors,
        "alert_warns":      alert_warns,
    }


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

    def sep(self, char="=", width=90):
        self.line(char * width)

    def close(self):
        if self._out:
            self._out.close()


def _fmt(val, fmt=".2f"):
    return format(val, fmt) if val is not None else "N/A"


def print_report(results, rpt):

    # ── Shooter current balance ───────────────────────────────────────────────
    rpt.sep()
    rpt.line("SHOOTER — LEFT vs RIGHT CURRENT (stator amps, enabled periods)")
    rpt.line(f'{"Match":<15} {"L mean":>8} {"R mean":>8} {"L p95":>8} {"R p95":>8} | {"L max":>7} {"R max":>7}')
    rpt.line("-" * 70)
    for r in results:
        lc, rc = r["sh_l_cur"], r["sh_r_cur"]
        if not lc or not rc:
            rpt.line(f'{r["match"]:<15}  (no shooter data)')
            continue
        diff = abs(lc["mean"] - rc["mean"])
        flag = f"  [spread {diff:.1f}A]" if diff > SHOOTER_SPREAD else ""
        rpt.line(f'{r["match"]:<15} {lc["mean"]:8.2f} {rc["mean"]:8.2f} '
                 f'{lc["p95"]:8.1f} {rc["p95"]:8.1f} | {lc["max"]:7.1f} {rc["max"]:7.1f}{flag}')

    # ── Shooter CPR ───────────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line(f"SHOOTER — CURRENT-PER-VELOCITY  (amps/rad/s, when spinning >{SHOOTER_HIGH_VEL:.0f} rad/s)")
    rpt.line("  Lower side = spinning faster per amp = possible free-wheel or worn surface")
    rpt.line(f'{"Match":<15} {"L cpr":>9} {"R cpr":>9} | {"L/R":>8}')
    rpt.line("-" * 48)
    for r in results:
        lc, rc = r["sh_l_cpr"], r["sh_r_cpr"]
        if not lc or not rc:
            rpt.line(f'{r["match"]:<15}  N/A')
            continue
        ratio = lc["mean"] / rc["mean"] if rc["mean"] else 0
        flag  = "  [LEFT LOW]"  if ratio < (1 / SHOOTER_CPR_RATIO) else \
                "  [RIGHT LOW]" if ratio > SHOOTER_CPR_RATIO else ""
        rpt.line(f'{r["match"]:<15} {lc["mean"]:9.4f} {rc["mean"]:9.4f} | {ratio:8.3f}{flag}')

    # ── Shooter velocity imbalance ────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line(f"SHOOTER — FLYWHEEL VELOCITY IMBALANCE % (both wheels >{SHOOTER_SPIN_VEL:.0f} rad/s)")
    rpt.line("  High % = wheels spinning at different speeds = ball not centered or surface worn")
    rpt.line(f'{"Match":<15} {"Mean %":>9} {"P95 %":>9} {"Max %":>9}')
    rpt.line("-" * 48)
    for r in results:
        si = r["sh_vel_imbal"]
        if not si:
            rpt.line(f'{r["match"]:<15}  N/A')
            continue
        flag = "  [WARN — high imbalance]" if si["p95"] > 15 else ""
        rpt.line(f'{r["match"]:<15} {si["mean"]:9.1f} {si["p95"]:9.1f} {si["max"]:9.1f}{flag}')

    # ── Shooter temperatures ──────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line(f"SHOOTER — TEMPERATURE PEAK (whole log, warn >{TEMP_WARN:.0f}C)")
    rpt.line(f'{"Match":<15} {"Left":>8} {"Right":>8}')
    rpt.line("-" * 35)
    for r in results:
        lt, rt = r["sh_l_tmp"], r["sh_r_tmp"]
        flags = []
        if lt and lt > TEMP_WARN: flags.append("L WARM")
        if rt and rt > TEMP_WARN: flags.append("R WARM")
        flag_str = "  [" + ", ".join(flags) + "]" if flags else ""
        lstr = f'{lt:8.1f}' if lt else '     N/A'
        rstr = f'{rt:8.1f}' if rt else '     N/A'
        rpt.line(f'{r["match"]:<15} {lstr} {rstr}{flag_str}')

    # ── Shooter shot event count ──────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line("SHOOTER — ESTIMATED SHOT EVENTS  (samples with left stator > 20A)")
    rpt.line(f'{"Match":<15} {"Events":>9} {"L mean A":>10}')
    rpt.line("-" * 38)
    for r in results:
        lc = r["sh_l_cur"]
        events = r["sh_shot_events"]
        mean_s = f'{lc["mean"]:10.2f}' if lc else '       N/A'
        rpt.line(f'{r["match"]:<15} {events:9d} {mean_s}')

    # ── Indexer belt ──────────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line(f"INDEXER BELT — CURRENT & JAM EVENTS  (jam = >{BELT_JAM_AMP:.0f}A & vel<{BELT_JAM_VEL:.0f} rad/s)")
    rpt.line(f'{"Match":<15} {"Mean A":>8} {"P95 A":>8} {"Max A":>8} | {"Jams":>7}')
    rpt.line("-" * 58)
    for r in results:
        s = r["ix_b_cur"]
        if not s:
            rpt.line(f'{r["match"]:<15}  N/A')
            continue
        flag = "  [JAMS]" if r["ix_b_jams"] > 20 else ""
        rpt.line(f'{r["match"]:<15} {s["mean"]:8.2f} {s["p95"]:8.1f} '
                 f'{s["max"]:8.1f} | {r["ix_b_jams"]:7d}{flag}')

    # ── Indexer kicker ────────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line("INDEXER KICKER — CURRENT")
    rpt.line(f'{"Match":<15} {"Mean A":>8} {"P95 A":>8} {"Max A":>8}')
    rpt.line("-" * 45)
    for r in results:
        s = r["ix_k_cur"]
        if s:
            rpt.line(f'{r["match"]:<15} {s["mean"]:8.2f} {s["p95"]:8.1f} {s["max"]:8.1f}')
        else:
            rpt.line(f'{r["match"]:<15}  N/A')

    # ── Indexer spinners ──────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line(f"INDEXER SPINNERS — LEFT vs RIGHT CURRENT  (flag spread >{SPINNER_SPREAD:.0f}A)")
    rpt.line(f'{"Match":<15} {"L mean":>8} {"R mean":>8} {"L p95":>8} {"R p95":>8} | {"Spread":>7}')
    rpt.line("-" * 65)
    for r in results:
        lc, rc = r["ix_sl_cur"], r["ix_sr_cur"]
        if not lc or not rc:
            rpt.line(f'{r["match"]:<15}  N/A')
            continue
        spread = abs(lc["mean"] - rc["mean"])
        flag   = f"  [spread {spread:.1f}A]" if spread > SPINNER_SPREAD else ""
        rpt.line(f'{r["match"]:<15} {lc["mean"]:8.2f} {rc["mean"]:8.2f} '
                 f'{lc["p95"]:8.1f} {rc["p95"]:8.1f} | {spread:7.2f}{flag}')

    # ── Intake deploy ─────────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line(f"INTAKE DEPLOY — CURRENT & HARD-STOP EVENTS  (spike = stator >{DEPLOY_SPIKE_AMP:.0f}A)")
    rpt.line(f'{"Match":<15} {"Mean A":>8} {"P95 A":>8} {"Max A":>8} | {"Spikes":>8}')
    rpt.line("-" * 58)
    for r in results:
        s = r["in_d_cur"]
        if not s:
            rpt.line(f'{r["match"]:<15}  N/A')
            continue
        flag = "  [FREQUENT]" if r["in_d_spikes"] > 500 else ""
        rpt.line(f'{r["match"]:<15} {s["mean"]:8.2f} {s["p95"]:8.1f} '
                 f'{s["max"]:8.1f} | {r["in_d_spikes"]:8d}{flag}')

    # ── Intake roller ─────────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line(f"INTAKE ROLLER — CURRENT, JAMS & DISCONNECTS")
    rpt.line(f"  Jam = stator >{ROLLER_JAM_AMP:.0f}A & vel<{ROLLER_JAM_VEL:.0f} rad/s")
    rpt.line(f'{"Match":<15} {"Mean A":>8} {"P95 A":>8} {"Max A":>8} | {"Jams":>7} {"Disc":>6}')
    rpt.line("-" * 65)
    for r in results:
        s = r["in_r_cur"]
        if not s:
            rpt.line(f'{r["match"]:<15}  N/A')
            continue
        flags = []
        if r["in_r_disconnects"] > 0: flags.append(f'DISC x{r["in_r_disconnects"]}')
        if r["in_r_jams"] > 20:       flags.append("JAMS")
        flag_str = "  [" + ", ".join(flags) + "]" if flags else ""
        rpt.line(f'{r["match"]:<15} {s["mean"]:8.2f} {s["p95"]:8.1f} {s["max"]:8.1f} | '
                 f'{r["in_r_jams"]:7d} {r["in_r_disconnects"]:6d}{flag_str}')

    # ── Temperatures summary ──────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line(f"ALL MOTOR TEMPERATURES — PEAK (whole log, deg C)  [warn >{TEMP_WARN:.0f}C]")
    rpt.line(f'{"Match":<15} {"Sh L":>7} {"Sh R":>7} {"IxBlt":>7} {"IxKck":>7} {"InDpl":>7} {"InRol":>7}')
    rpt.line("-" * 60)
    TEMP_COLS = [("sh_l_tmp","ShL"),("sh_r_tmp","ShR"),("ix_b_tmp","IxBelt"),
                 ("ix_k_tmp","IxKck"),("in_d_tmp","InDpl"),("in_r_tmp","InRol")]
    for r in results:
        def tf(k): t = r[k]; return f'{t:7.1f}' if t else '    N/A'
        warm = [lbl for k, lbl in TEMP_COLS if r[k] and r[k] > TEMP_WARN]
        flag_str = "  [WARM: " + ", ".join(warm) + "]" if warm else ""
        rpt.line(f'{r["match"]:<15} '
                 + " ".join(tf(k) for k, _ in TEMP_COLS)
                 + flag_str)

    # ── PDH ──────────────────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line("PDH — TOTAL ROBOT CURRENT & POWER (enabled periods)")
    rpt.line(f'{"Match":<15} {"Mean A":>8} {"Max A":>8} {"Mean W":>9} {"Max W":>9}')
    rpt.line("-" * 55)
    for r in results:
        pc, pp = r["pdh_cur"], r["pdh_pwr"]
        if pc and pp:
            rpt.line(f'{r["match"]:<15} {pc["mean"]:8.1f} {pc["max"]:8.1f} '
                     f'{pp["mean"]:9.0f} {pp["max"]:9.0f}')
        elif pc:
            rpt.line(f'{r["match"]:<15} {pc["mean"]:8.1f} {pc["max"]:8.1f}')
        else:
            rpt.line(f'{r["match"]:<15}  N/A')

    # ── Alerts ────────────────────────────────────────────────────────────────
    rpt.line()
    rpt.sep()
    rpt.line("ACTIVE ALERTS DURING ENABLED PERIODS")
    rpt.line("-" * 60)
    any_alerts = False
    for r in results:
        for msg in sorted(r["alert_errors"]):
            rpt.line(f'  {r["match"]:<15} [ERROR] {msg}')
            any_alerts = True
        for msg in sorted(r["alert_warns"]):
            rpt.line(f'  {r["match"]:<15} [WARN]  {msg}')
            any_alerts = True
    if not any_alerts:
        rpt.line("  (none)")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Mechanism motor analyzer — Team 6391"
    )
    parser.add_argument("directory", nargs="?", default=".",
                        help="Directory containing .wpilog files")
    parser.add_argument("-o", "--output", metavar="FILE",
                        help="Write report to this file")
    parser.add_argument("--no-practice", action="store_true",
                        help="Exclude practice match logs")
    parser.add_argument("--match", metavar="LABEL",
                        help='Analyze only this match, e.g. "Qual 5"')
    args = parser.parse_args()

    log_dir = os.path.abspath(args.directory)
    if not os.path.isdir(log_dir):
        print(f"ERROR: {log_dir} is not a directory", file=sys.stderr)
        sys.exit(1)

    PHASE_MAP = {"p": "Practice", "q": "Qual", "e": "Elim"}
    log_entries = []
    for fn in sorted(os.listdir(log_dir)):
        if not fn.endswith(".wpilog"):
            continue
        m = re.search(r"_(p|q|e)(\d+)\.wpilog$", fn)
        phase_key  = m.group(1) if m else "?"
        match_name = f"{PHASE_MAP.get(phase_key,'?')} {m.group(2)}" if m \
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
    results = []
    for fn, match_name in log_entries:
        try:
            r = parse_log(os.path.join(log_dir, fn))
            r["match"] = match_name
            results.append(r)
            print(f"  OK  {fn}", file=sys.stderr)
        except Exception as ex:
            print(f"  ERR {fn}: {ex}", file=sys.stderr)

    if not results:
        print("No logs parsed successfully.", file=sys.stderr)
        sys.exit(1)

    rpt = Report(output_path=args.output)
    rpt.sep()
    rpt.line(f"MECHANISM ANALYSIS — {len(results)} match(es) from {log_dir}")
    rpt.line(f"Matches: {', '.join(r['match'] for r in results)}")
    rpt.sep()
    print_report(results, rpt)
    rpt.close()

    if args.output:
        print(f"\nReport written to: {args.output}", file=sys.stderr)


if __name__ == "__main__":
    main()
