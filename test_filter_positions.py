"""
Filter wheel position accuracy & timing test.

Connects to the PD Stepper at 192.168.4.1 (join "PD Stepper" WiFi AP first).
Runs a sweep through all 6 filter positions, measures:
  - Actual encoder angle vs. expected target angle
  - Settling time (time until angle stops changing)
  - Repeatability over multiple passes
"""

import time
import requests
import sys

BASE = "http://192.168.4.1"
NUM_POSITIONS = 6
SLOT_DEG = 360.0 / NUM_POSITIONS          # 60° per slot
SETTLE_WINDOW_DEG = 1.0                    # consider settled when <1° change per poll
SETTLE_TIMEOUT_S = 5.0
POLL_INTERVAL_S = 0.1


def get(path: str, timeout=3.0):
    r = requests.get(BASE + path, timeout=timeout)
    r.raise_for_status()
    return r.text.strip()


def post(path: str, data: dict, timeout=3.0):
    r = requests.post(BASE + path, data=data, timeout=timeout)
    r.raise_for_status()


def enable_motor():
    """Enable motor via /save endpoint."""
    post("/save", {
        "enabled1": "on",
        "setvoltage": "12",
        "microsteps": "32",
        "current": "30",
        "stall_threshold": "10",
        "standstill_mode": "NORMAL",
    })
    time.sleep(0.3)


def read_angle() -> float:
    return float(get("/filter/angle"))


def shortest_delta(target: float, current: float) -> float:
    d = (target - current) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def goto_position(pos: int) -> dict:
    """Command goto_pos, poll until settled, return result dict."""
    target_deg = (pos * SLOT_DEG) % 360.0
    t_start = time.time()
    post("/filter", {"action": "goto_pos", "pos": str(pos)})

    prev_angle = read_angle()
    settled_at = None
    last_angles = []

    while True:
        time.sleep(POLL_INTERVAL_S)
        angle = read_angle()
        last_angles.append(angle)
        elapsed = time.time() - t_start

        delta_since_last = abs(shortest_delta(angle, prev_angle))
        if delta_since_last < SETTLE_WINDOW_DEG and settled_at is None:
            settled_at = elapsed
        elif delta_since_last >= SETTLE_WINDOW_DEG:
            settled_at = None  # still moving

        prev_angle = angle

        if elapsed > SETTLE_TIMEOUT_S or (settled_at is not None and elapsed - settled_at > 0.3):
            break

    final_angle = read_angle()
    error = abs(shortest_delta(target_deg, final_angle))

    return {
        "pos": pos,
        "target_deg": target_deg,
        "final_deg": round(final_angle, 1),
        "error_deg": round(error, 1),
        "settle_s": round(settled_at, 2) if settled_at else None,
        "timeout": settled_at is None,
    }


def run_sweep(passes: int = 3):
    print(f"\n{'='*60}")
    print(f"Filter Wheel Position Test  —  {passes} passes")
    print(f"{'='*60}")

    # Make sure motor is enabled
    try:
        enable_motor()
    except Exception as e:
        print(f"WARN: could not enable motor: {e}")

    all_results = []

    for pass_n in range(passes):
        print(f"\n--- Pass {pass_n + 1}/{passes} ---")
        # Alternate sweep direction to test both approach directions
        positions = list(range(NUM_POSITIONS)) if pass_n % 2 == 0 else list(reversed(range(NUM_POSITIONS)))
        for pos in positions:
            r = goto_position(pos)
            all_results.append(r)
            status = "TIMEOUT" if r["timeout"] else f"{r['settle_s']:.2f}s"
            print(f"  Pos {r['pos']}  target={r['target_deg']:5.1f}°  "
                  f"actual={r['final_deg']:5.1f}°  err={r['error_deg']:4.1f}°  "
                  f"settle={status}")
        time.sleep(0.5)

    # ── Summary ──────────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("Summary per slot (across all passes):")
    print(f"{'Slot':>5} {'Target':>8} {'AvgErr':>8} {'MaxErr':>8} {'AvgSettle':>10} {'Timeouts':>8}")
    print("-" * 60)
    for pos in range(NUM_POSITIONS):
        slot_results = [r for r in all_results if r["pos"] == pos]
        avg_err = sum(r["error_deg"] for r in slot_results) / len(slot_results)
        max_err = max(r["error_deg"] for r in slot_results)
        good = [r for r in slot_results if r["settle_s"] is not None]
        avg_settle = sum(r["settle_s"] for r in good) / len(good) if good else float("nan")
        timeouts = sum(1 for r in slot_results if r["timeout"])
        print(f"{pos:>5} {pos * SLOT_DEG:>7.1f}°  {avg_err:>7.1f}°  {max_err:>7.1f}°  "
              f"{avg_settle:>9.2f}s  {timeouts:>8}")

    ok = [r for r in all_results if not r["timeout"]]
    if ok:
        overall_avg_err = sum(r["error_deg"] for r in ok) / len(ok)
        overall_max_err = max(r["error_deg"] for r in ok)
        overall_avg_settle = sum(r["settle_s"] for r in ok) / len(ok)
        timeouts_total = sum(1 for r in all_results if r["timeout"])
        print(f"\nOverall: avg_err={overall_avg_err:.1f}°  max_err={overall_max_err:.1f}°  "
              f"avg_settle={overall_avg_settle:.2f}s  timeouts={timeouts_total}/{len(all_results)}")
    return all_results


def run_random_test(num_moves: int = 12, seed: int | None = None):
    """Move to randomly chosen positions and verify encoder accuracy."""
    import random
    rng = random.Random(seed)

    print(f"\n{'='*60}")
    print(f"Random position test  —  {num_moves} moves")
    print(f"{'='*60}")
    try:
        enable_motor()
    except Exception:
        pass

    errors = []
    prev_pos = -1
    for i in range(num_moves):
        # Pick a position different from the last one to ensure real motion
        choices = [p for p in range(NUM_POSITIONS) if p != prev_pos]
        pos = rng.choice(choices)
        prev_pos = pos

        r = goto_position(pos)
        status = "TIMEOUT" if r["timeout"] else f"err={r['error_deg']:.2f}°  settle={r['settle_s']:.2f}s"
        print(f"  {i+1:2d}. pos={pos}  target={r['target_deg']:5.1f}°  "
              f"actual={r['final_deg']:5.1f}°  {status}")
        if not r["timeout"]:
            errors.append(r["error_deg"])

    print()
    if errors:
        avg = sum(errors) / len(errors)
        print(f"  avg_err={avg:.2f}°  max_err={max(errors):.2f}°  "
              f"min_err={min(errors):.2f}°  timeouts={num_moves-len(errors)}/{num_moves}")


def run_single_position_test(pos: int, repeats: int = 5):
    """Repeatedly command one position to check repeatability."""
    print(f"\n--- Repeatability test: position {pos} ({pos * SLOT_DEG:.0f}°), {repeats} reps ---")
    try:
        enable_motor()
    except Exception:
        pass

    errors = []
    for i in range(repeats):
        # First move away, then come back
        away = (pos + 3) % NUM_POSITIONS
        goto_position(away)
        time.sleep(0.2)
        r = goto_position(pos)
        errors.append(r["error_deg"])
        print(f"  Rep {i+1}: final={r['final_deg']:.1f}°  err={r['error_deg']:.1f}°  "
              f"settle={r['settle_s']:.2f}s" if r["settle_s"] else
              f"  Rep {i+1}: TIMEOUT  final={r['final_deg']:.1f}°")

    if errors:
        print(f"  Repeatability: avg={sum(errors)/len(errors):.1f}°  "
              f"max={max(errors):.1f}°  min={min(errors):.1f}°")


if __name__ == "__main__":
    try:
        # Quick connectivity check
        health = get("/health")
        print(f"Device: {health}")
        voltage = get("/voltage")
        print(f"Voltage: {voltage}")
        pg = get("/powergood")
        print(f"Power: {pg}")
    except Exception as e:
        print(f"Cannot reach device at {BASE}: {e}")
        sys.exit(1)

    # Random position test (primary check)
    run_random_test(num_moves=12)

    print("\nDone.")
