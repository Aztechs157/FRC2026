"""At each FullCycleMS spike, pull the values of every Logger/*MS breakdown channel
and any other timing channels to see which step actually ate the time."""

import struct
from pathlib import Path
from collections import defaultdict

LOG_PATH = Path(r"C:\Users\aztec\Desktop\akit_26-05-27_22-43-13.wpilog")
SPIKE_THRESHOLD_MS = 50.0  # only investigate frames slower than this
MAX_SPIKES_TO_SHOW = 20


def read_var(buf, off, length):
    val = 0
    for i in range(length):
        val |= buf[off + i] << (8 * i)
    return val, off + length


def main():
    data = LOG_PATH.read_bytes()
    off = 12 + struct.unpack_from("<I", data, 8)[0]

    entries = {}  # id -> (name, type)
    fullcycle_id = None
    # Track sequential samples per entry as we go (timestamp_us -> value)
    breakdown_samples = defaultdict(list)  # name -> list of (ts_us, value)
    fullcycle_samples = []  # list of (ts_us, value)
    # Channel names we care about: any /Logger/*MS, plus a couple of others
    target_suffixes = ("MS",)  # match LogPeriodicMS, ConduitSaveMS, etc.
    target_subs = (
        "/RealOutputs/Logger/",
        "/RealOutputs/LoggedRobot/",
        "/SystemStats/",
    )

    target_ids = set()

    while off < len(data):
        header = data[off]
        off += 1
        eid_len = (header & 0x03) + 1
        sz_len = ((header >> 2) & 0x03) + 1
        ts_len = ((header >> 4) & 0x0F) + 1
        entry_id, off = read_var(data, off, eid_len)
        payload_size, off = read_var(data, off, sz_len)
        ts, off = read_var(data, off, ts_len)
        ps = off
        off += payload_size

        if entry_id == 0:
            if payload_size > 0 and data[ps] == 0:
                ent_id = struct.unpack_from("<I", data, ps + 1)[0]
                name_len = struct.unpack_from("<I", data, ps + 5)[0]
                name = data[ps + 9 : ps + 9 + name_len].decode("utf-8", "replace")
                p2 = ps + 9 + name_len
                type_len = struct.unpack_from("<I", data, p2)[0]
                tn = data[p2 + 4 : p2 + 4 + type_len].decode("utf-8", "replace")
                entries[ent_id] = (name, tn)
                if name == "/RealOutputs/LoggedRobot/FullCycleMS":
                    fullcycle_id = ent_id
                # Match the broader set of timing channels
                is_targeted = (
                    any(name.startswith(sub) for sub in target_subs)
                    and tn in ("double", "int64", "float")
                )
                if is_targeted:
                    target_ids.add(ent_id)
            continue

        if entry_id == fullcycle_id:
            val = struct.unpack("<d", data[ps : ps + 8])[0]
            fullcycle_samples.append((ts, val))
        elif entry_id in target_ids:
            name, tn = entries[entry_id]
            if tn == "double":
                val = struct.unpack("<d", data[ps : ps + 8])[0]
            elif tn == "float":
                val = struct.unpack("<f", data[ps : ps + 4])[0]
            else:  # int64
                val = struct.unpack("<q", data[ps : ps + 8])[0]
            breakdown_samples[name].append((ts, val))

    print(f"Parsed {len(entries)} entries, {len(fullcycle_samples)} FullCycle samples")
    print(f"Tracking {len(target_ids)} timing/breakdown channels")

    # Build a per-timestamp lookup for each breakdown channel.
    # For each spike, find the breakdown sample with timestamp closest to the spike.
    def nearest(samples, ts_us):
        if not samples:
            return None
        # Binary search would be faster, but list isn't huge.
        best = None
        best_dt = float("inf")
        for sts, val in samples:
            dt = abs(sts - ts_us)
            if dt < best_dt:
                best_dt = dt
                best = (sts, val)
            elif sts > ts_us + 50_000:  # samples are time-ordered
                break
        return best if best_dt < 30_000 else None  # within 30ms

    # Find spikes, skip the very early boot ones (< 5s)
    spikes = [
        (ts, v) for ts, v in fullcycle_samples if v > SPIKE_THRESHOLD_MS and ts > 5_000_000
    ]
    spikes.sort(key=lambda x: -x[1])
    print(f"\nFound {len(spikes)} spikes >{SPIKE_THRESHOLD_MS}ms after t=5s")

    # For each top spike, print the breakdown
    print(f"\n=== Top {MAX_SPIKES_TO_SHOW} spikes with timing breakdown ===")
    breakdown_names = sorted(
        [n for n in breakdown_samples.keys() if n.endswith("MS") or "MS" in n or "Cycles" in n]
    )
    for spike_ts, spike_val in spikes[:MAX_SPIKES_TO_SHOW]:
        print(f"\n--- t={spike_ts/1_000_000:.3f}s  FullCycle={spike_val:.2f}ms ---")
        # Gather all breakdown values near this timestamp
        rows = []
        for name in breakdown_names:
            nb = nearest(breakdown_samples[name], spike_ts)
            if nb:
                short = name.replace("/RealOutputs/", "").replace("/SystemStats/", "Sys/")
                rows.append((nb[1], short))
        # Sort by value descending
        rows.sort(key=lambda x: -x[0] if isinstance(x[0], (int, float)) else 0)
        for v, n in rows[:12]:
            if isinstance(v, float):
                print(f"  {v:>8.2f}  {n}")
            else:
                print(f"  {v:>8}  {n}")

    # Aggregate stats per breakdown channel for "spike frames" vs "normal frames"
    print("\n=== Aggregate: median value of each breakdown channel ===")
    print("(during spike frames vs normal frames, to see which channels are usually small but huge during spikes)")
    spike_ts_set = set(s[0] for s in spikes)
    normal_ts_set = set(ts for ts, v in fullcycle_samples if v < 20)
    for name in breakdown_names:
        # Find values nearest to each spike vs each normal frame
        spike_vals = []
        normal_vals = []
        samples = breakdown_samples[name]
        if not samples:
            continue
        # Spike values
        for sts, sval in spikes:
            nb = nearest(samples, sts)
            if nb and isinstance(nb[1], (int, float)):
                spike_vals.append(nb[1])
        # Normal values - sample 200 random normals
        normal_frames = [ts for ts, v in fullcycle_samples if 10 < v < 18]
        import random
        random.seed(42)
        for nts in random.sample(normal_frames, min(200, len(normal_frames))):
            nb = nearest(samples, nts)
            if nb and isinstance(nb[1], (int, float)):
                normal_vals.append(nb[1])
        if not spike_vals or not normal_vals:
            continue
        spike_vals.sort()
        normal_vals.sort()
        sm = spike_vals[len(spike_vals) // 2]
        nm = normal_vals[len(normal_vals) // 2]
        if sm > 5 * nm and sm > 3:  # only report channels that are ≥5x bigger during spikes
            short = name.replace("/RealOutputs/", "").replace("/SystemStats/", "Sys/")
            print(f"  spike_med={sm:>8.2f}  normal_med={nm:>6.2f}  ratio={sm/max(nm,0.001):>6.1f}x  {short}")


if __name__ == "__main__":
    main()
