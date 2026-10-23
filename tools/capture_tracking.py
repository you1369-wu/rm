"""Record the project's AA BB 48 + 18 float32 protocol; never sends motor commands."""
import argparse
import csv
import math
import struct
import time
from pathlib import Path

COLUMNS = {
  0: 'yaw_ref yaw_actual pitch_ref pitch_actual yaw_error pitch_error b_error c_relative tau_a tau_b tau_c rms_yaw rms_pitch state fault phase_ms saturation imu_pitch'.split(),
  1: 'yaw_speed_ref yaw_speed_actual pitch_speed_ref pitch_speed_actual pid_a ff_a tau_a pid_c ff_c tau_c accel_a accel_c tau_b b_error state fault gap_ms saturation'.split(),
}

def decode(buffer):
    """Incremental parser. Caller owns buffer; handles partial frames and leading noise."""
    while len(buffer) >= 3:
        pos = buffer.find(b'\xaa\xbb')
        if pos < 0:
            buffer[:] = buffer[-1:] if buffer[-1] == 0xaa else b''
            return
        del buffer[:pos]
        if len(buffer) < 3:
            return
        if buffer[2] != 72:
            del buffer[0]
            continue
        if len(buffer) < 75:
            return
        values = struct.unpack_from('<18f', buffer, 3)
        if not all(math.isfinite(v) for v in values):
            del buffer[0]
            continue
        del buffer[:75]
        yield values

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument('--port', help='e.g. COM5, 115200 baud 8N1')
    source.add_argument('--input-bin', type=Path, help='decode an already recorded binary stream')
    ap.add_argument('--csv', type=Path, required=True)
    ap.add_argument('--page', type=int, choices=(0, 1), default=0, help='must match kPlotPage')
    ap.add_argument('--seconds', type=float, default=30)
    args = ap.parse_args()
    if args.seconds <= 0:
        ap.error('--seconds must be positive')
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    buffer = bytearray()
    count = 0
    if args.port:
        try:
            import serial
        except ImportError:
            ap.error('Serial recording requires: python -m pip install pyserial')
        source_stream = serial.Serial(args.port, 115200, timeout=.1)
    else:
        source_stream = args.input_bin.open('rb')
    with source_stream as stream, args.csv.open('w', newline='', encoding='utf-8') as out:
        writer = csv.writer(out)
        writer.writerow(['host_s'] + COLUMNS[args.page])
        start = time.monotonic()
        try:
            while True:
                if args.port and time.monotonic() - start >= args.seconds:
                    break
                chunk = stream.read(max(1, min(stream.in_waiting, 4096))) if args.port else stream.read(4096)
                if not chunk and not args.port:
                    break
                buffer.extend(chunk)
                for values in decode(buffer):
                    # Offline binary files have no acquisition timestamp: leave host_s empty.
                    writer.writerow([time.monotonic() - start if args.port else ''] + list(values))
                    count += 1
                out.flush()
        except KeyboardInterrupt:
            pass
    print(f'Saved {count} frames to {args.csv}. Page={args.page}. Protocol has no CRC; inspect data quality.')

if __name__ == '__main__':
    main()
