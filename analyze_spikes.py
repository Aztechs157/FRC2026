"""Look at what other channels logged near the top FullCycleMS spike timestamps,
to identify the source of the overruns."""

import struct
from pathlib import Path
from collections import defaultdict

LOG_PATH = Path(r"D:\logs\akit_26-05-27_22-43-13.wpilog")

# Spike timestamps (in microseconds) — from the previous analysis, excluding the
# 15s startup outlier so we look at the real steady-state spikes.
SPIKE_TS_US = [
    int(15.23 * 1_000_000),
    int(16.05 * 1_000_000),
    int(16.41 * 1_000_000),
    int(23.17 * 1_000_000),
    int(35.15 * 1_000_000),
    int(20.27 * 1_000_000),
    int(17.86 * 1_000_000),
    int(26.33 * 1_000_000),
    int(68.21 * 1_000_000),
]
# Window around each spike to look at (us). Wider window means more context.
WIN_US = 100_000  # 100ms


def read_var(buf, off, length):
    val = 0
    for i in range(length):
        val |= buf[off + i] << (8 * i)
    return val, off + length


def parse():
    data = LOG_PATH.read_bytes()
    off = 12 + struct.unpack_from("<I", data, 8)[0]

    entries = {}  # id -> (name, type)
    # Per-entry sample count
    sample_count = defaultdict(int)
    # For each spike window, count how many records each entry contributed
    window_counts = [defaultdict(int) for _ in SPIKE_TS_US]
    # And track the values of FullCycleMS, LogPeriodicMS in each window
    spike_details = [[] for _ in SPIKE_TS_US]
    fullcycle_id = None
    logperiodic_id = None

    while off < len(data):
        header = data[off]
        off += 1
        eid_len = (header & 0x03) + 1
        sz_len = ((header >> 2) & 0x03) + 1
        ts_len = ((header >> 4) & 0x0F) + 1
        entry_id, off = read_var(data, off, eid_len)
        payload_size, off = read_var(data, off, sz_len)
        ts, off = read_var(data, off, ts_len)
        payload_start = off
        off += payload_size

        if entry_id == 0:
            if payload_size > 0 and data[payload_start] == 0:
                # Start record
                ent_id = struct.unpack_from("<I", data, payload_start + 1)[0]
                name_len = struct.unpack_from("<I", data, payload_start + 5)[0]
                name = data[payload_start + 9 : payload_start + 9 + name_len].decode("utf-8", errors="replace")
                p2 = payload_start + 9 + name_len
                type_len = struct.unpack_from("<I", data, p2)[0]
                tn = data[p2 + 4 : p2 + 4 + type_len].decode("utf-8", errors="replace")
                entries[ent_id] = (name, tn)
                if name == "/RealOutputs/LoggedRobot/FullCycleMS":
                    fullcycle_id = ent_id
                elif name == "/RealOutputs/LoggedRobot/LogPeriodicMS":
                    logperiodic_id = ent_id
            continue

        sample_count[entry_id] += 1

        # Check if this record falls in any spike window
        for i, spike_ts in enumerate(SPIKE_TS_US):
            if abs(ts - spike_ts) <= WIN_US:
                window_counts[i][entry_id] += 1
                if entry_id == fullcycle_id or entry_id == logperiodic_id:
                    val = struct.unpack("<d", data[payload_start : payload_start + 8])[0]
                    spike_details[i].append((ts, entries.get(entry_id, ("?", "?"))[0], val))

    print("=== All 333 entries by sample count ===")
    by_count = sorted(sample_count.items(), key=lambda x: -x[1])
    print(f"{'samples':>10}  {'name'}")
    for ent_id, count in by_count[:60]:
        name, tn = entries.get(ent_id, ("?", "?"))
        print(f"{count:>10}  {name} ({tn})")

    print("\n=== Records logged near each spike (top 15 by count) ===")
    for i, spike_ts in enumerate(SPIKE_TS_US):
        print(f"\n--- Spike @ t={spike_ts/1_000_000:.2f}s (window ±{WIN_US/1000:.0f}ms) ---")
        for ts, name, val in sorted(spike_details[i]):
            print(f"  {name} = {val:.2f}ms  (t={ts/1_000_000:.3f}s)")
        wc = window_counts[i]
        top = sorted(wc.items(), key=lambda x: -x[1])[:15]
        for ent_id, count in top:
            name, tn = entries.get(ent_id, ("?", "?"))
            print(f"  {count:>5}  {name}")


if __name__ == "__main__":
    parse()
