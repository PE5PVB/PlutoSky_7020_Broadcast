#!/usr/bin/env python3
"""Generate a sine LUT (.mem, hex) for skypluto_sincos.

  sin[k] = round( (2^(width-1)-1) * sin(2*pi * k / depth) )

Two's complement, 'width' bits per entry, 'depth' entries (depth = 2^ADDR_W).
Output: 1 hex word per line, suitable for $readmemh.

Example:
    python3 gen_sine_lut.py --depth 4096 --width 16 \
            --out ../hdl/library/skypluto_wfm/data/sine_lut.mem
"""
import argparse
import math
import os


def main():
    ap = argparse.ArgumentParser(description="Sinus-LUT generator (.mem hex)")
    ap.add_argument("--depth", type=int, default=4096,
                    help="aantal entries (= 2^ADDR_W), default 4096")
    ap.add_argument("--width", type=int, default=16,
                    help="bits per entry (signed), default 16")
    ap.add_argument("--out", default="sine_lut.mem", help="uitvoerbestand")
    ap.add_argument("--full-scale", type=float, default=None,
                    help="max amplitude (default 2^(width-1)-1)")
    args = ap.parse_args()

    if args.depth & (args.depth - 1):
        ap.error("--depth moet een macht van 2 zijn")

    amp = args.full_scale if args.full_scale is not None else (2 ** (args.width - 1) - 1)
    mask = (1 << args.width) - 1
    hexdigits = (args.width + 3) // 4

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        f.write(f"// sine LUT: depth={args.depth} width={args.width} amp={amp:.3f}\n")
        for k in range(args.depth):
            val = int(round(amp * math.sin(2.0 * math.pi * k / args.depth)))
            # clamp to the signed range
            hi = (1 << (args.width - 1)) - 1
            lo = -(1 << (args.width - 1))
            val = max(lo, min(hi, val))
            f.write(f"{val & mask:0{hexdigits}X}\n")

    print(f"geschreven: {args.out}  ({args.depth} entries, {args.width}-bit)")


if __name__ == "__main__":
    main()
