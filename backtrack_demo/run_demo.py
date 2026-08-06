#!/usr/bin/env python3
"""Run the Link-Loss Backtrack demo and write the blackbox logs + plot.

Usage:
    python3 run_demo.py [--heading fusion|odometry] [--seed N] [--outdir DIR]
"""

import argparse
import os

from backtrack.logio import (write_breadcrumbs_csv, write_route_svg,
                             write_timeseries_csv)
from backtrack.simulation import SimConfig, run_simulation


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--heading", choices=["fusion", "odometry"], default="fusion")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(__file__), "data"))
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    cfg = SimConfig(heading_source=args.heading, seed=args.seed)
    result = run_simulation(cfg)

    ts_path = os.path.join(args.outdir, "session_timeseries.csv")
    bc_path = os.path.join(args.outdir, "breadcrumbs.csv")
    svg_path = os.path.join(args.outdir, "route.svg")
    write_timeseries_csv(ts_path, result.records)
    write_breadcrumbs_csv(bc_path, result.breadcrumbs)
    write_route_svg(svg_path, result)

    s = result.summary
    print("=== Link-Loss Backtrack demo ===")
    print(f"heading source      : {s['heading_source']}")
    print(f"outbound distance   : {s['outbound_distance_m']} m")
    print(f"ticks logged        : {s['ticks']}")
    print(f"breadcrumbs         : {s['breadcrumbs']}")
    print(f"est outbound error  : {s['est_outbound_error_m']} m")
    print(f"backtrack completed : {s['backtrack_completed']}")
    print(f"final return error  : {s['final_return_error_m']} m "
          f"({s['final_error_pct']}% of route)")
    print()
    print(f"wrote {ts_path}")
    print(f"wrote {bc_path}")
    print(f"wrote {svg_path}")


if __name__ == "__main__":
    main()
