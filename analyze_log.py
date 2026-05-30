"""Quick-and-dirty wpilog parser to find loop-time spikes in an AdvantageKit log.

Single pass through the file. Memory usage is O(num samples for the target channel),
which is fine for the loop-time channel alone.
"""

import struct
import sys
from pathlib import Path

LOG_PATH = Path(r"D:\logs\akit_26-05-27_22-43-13.wpilog")

# Channels we care about. Add more keywords to widen the net.
TARGET_KEYWORDS = ("FullCycleMS", "PeriodicMS", "Watchdog", "Epoch")


def read_var(buf: bytes, off: int, length: int) -> tuple[int, int]:
    """Read a variable-length little-endian unsigned int."""
    val = 0
    for i in range(length):
        val |= buf[off + i] << (8 * i)
    return val, off + length


def parse(path: Path) -> None:
    data = path.read_bytes()
    print(f"Loaded {len(data):,} bytes from {path.name}")

    # File header: "WPILOG" + u16 version + u32 extra header length + extra header
    assert data[0:6] == b"WPILOG", "not a WPILOG file"
    version = struct.unpack_from("<H", data, 6)[0]
    extra_len = struct.unpack_from("<I", data, 8)[0]
    print(f"WPILOG version 0x{version:04x}, extra header {extra_len} bytes")
    off = 12 + extra_len

    # entry_id -> (name, type, samples=[(timestamp_us, payload_bytes)])
    entries: dict[int, dict] = {}
    target_ids: set[int] = set()
    all_entry_names: list[str] = []

    record_count = 0
    while off < len(data):
        header = data[off]
        off += 1
        entry_id_len = (header & 0x03) + 1
        size_len = ((header >> 2) & 0x03) + 1
        ts_len = ((header >> 4) & 0x0F) + 1

        entry_id, off = read_var(data, off, entry_id_len)
        payload_size, off = read_var(data, off, size_len)
        timestamp_us, off = read_var(data, off, ts_len)
        payload = data[off : off + payload_size]
        off += payload_size
        record_count += 1

        if entry_id == 0:
            # Control record
            ctrl_type = payload[0]
            if ctrl_type == 0:  # Start
                ent_id = struct.unpack_from("<I", payload, 1)[0]
                name_len = struct.unpack_from("<I", payload, 5)[0]
                name = payload[9 : 9 + name_len].decode("utf-8", errors="replace")
                p2 = 9 + name_len
                type_len = struct.unpack_from("<I", payload, p2)[0]
                type_name = payload[p2 + 4 : p2 + 4 + type_len].decode("utf-8", errors="replace")
                entries[ent_id] = {"name": name, "type": type_name, "samples": []}
                all_entry_names.append(name)
                if any(k in name for k in TARGET_KEYWORDS):
                    target_ids.add(ent_id)
                    print(f"  target channel found: id={ent_id} type={type_name} name={name}")
        elif entry_id in target_ids:
            entries[entry_id]["samples"].append((timestamp_us, payload))

    print(f"\nParsed {record_count:,} records, {len(entries)} distinct entries")
    print(f"Total entry names: {len(all_entry_names)}")

    # Look for any name containing FullCycle (case insensitive) to be safe
    print("\nAll entries matching 'cycle', 'loop', 'epoch', 'periodic' (case-insensitive):")
    for name in all_entry_names:
        low = name.lower()
        if any(k in low for k in ("cycle", "loop", "epoch", "periodic", "watchdog")):
            print(f"  {name}")

    print("\n--- Stats for target channels ---")
    for ent_id, ent in entries.items():
        if ent_id not in target_ids:
            continue
        samples = ent["samples"]
        if not samples:
            print(f"\n{ent['name']}: no samples")
            continue
        if ent["type"] == "double":
            vals = [struct.unpack("<d", s[1])[0] for s in samples]
        elif ent["type"] == "float":
            vals = [struct.unpack("<f", s[1])[0] for s in samples]
        elif ent["type"] == "int64":
            vals = [struct.unpack("<q", s[1])[0] for s in samples]
        else:
            print(f"\n{ent['name']}: type {ent['type']} not handled ({len(samples)} samples)")
            continue

        n = len(vals)
        s = sorted(vals)
        median = s[n // 2]
        p95 = s[int(n * 0.95)]
        p99 = s[int(n * 0.99)]
        mx = s[-1]
        # treat as ms if name contains MS, else as seconds (then convert)
        unit = "ms" if "MS" in ent["name"] else "s"
        spikes_25 = sum(1 for v in vals if (v if unit == "ms" else v * 1000) > 25)
        spikes_50 = sum(1 for v in vals if (v if unit == "ms" else v * 1000) > 50)
        spikes_100 = sum(1 for v in vals if (v if unit == "ms" else v * 1000) > 100)
        print(f"\n{ent['name']} ({ent['type']}, {n:,} samples, unit:{unit})")
        print(f"  median {median:.3f} | p95 {p95:.3f} | p99 {p99:.3f} | max {mx:.3f}")
        print(f"  >25  units: {spikes_25:,}  ({100*spikes_25/n:.2f}%)")
        print(f"  >50  units: {spikes_50:,}  ({100*spikes_50/n:.2f}%)")
        print(f"  >100 units: {spikes_100:,}  ({100*spikes_100/n:.2f}%)")

        # Find top 10 spike events with their timestamps
        ts_and_vals = list(zip([t for t, _ in samples], vals))
        ts_and_vals.sort(key=lambda x: -x[1])
        print(f"  top 10 spikes (timestamp_us, value):")
        for t, v in ts_and_vals[:10]:
            print(f"    t={t/1_000_000:.2f}s  v={v:.3f}{unit}")


if __name__ == "__main__":
    parse(LOG_PATH)
