"""Output helpers: write the blackbox CSVs and a dependency-free SVG plot."""

import csv

SENSOR_FIELDS = [
    "t", "mode", "link_ok",
    "erpm_left", "erpm_right", "motor_current",
    "gyro_z", "compass_heading",
    "est_x", "est_y", "est_heading",
    "gt_x", "gt_y", "gt_heading",
]


def write_timeseries_csv(path, records):
    with open(path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=SENSOR_FIELDS)
        writer.writeheader()
        writer.writerows(records)


def write_breadcrumbs_csv(path, breadcrumbs):
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["idx", "x", "y", "heading", "cumulative_dist"])
        for i, b in enumerate(breadcrumbs):
            writer.writerow([i, round(b.x, 4), round(b.y, 4),
                             round(b.heading, 5), round(b.cumulative_dist, 3)])


def _polyline(points, transform, color, width=2.0, dash=None):
    pts = " ".join(f"{transform(x, y)[0]:.1f},{transform(x, y)[1]:.1f}"
                   for x, y in points)
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return (f'<polyline fill="none" stroke="{color}" stroke-width="{width}"'
            f'{d} points="{pts}" />')


def write_route_svg(path, result, size=900, margin=50):
    """Overlay true trajectory, dead-reckoned estimate, and breadcrumb trail."""
    records = result.records
    gt = [(r["gt_x"], r["gt_y"]) for r in records]
    est = [(r["est_x"], r["est_y"]) for r in records]
    crumbs = [(b.x, b.y) for b in result.breadcrumbs]
    all_pts = gt + est + crumbs

    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span = max(max_x - min_x, max_y - min_y, 1.0)
    scale = (size - 2 * margin) / span

    def tf(x, y):
        px = margin + (x - min_x) * scale
        # SVG y grows downward; flip so north is up.
        py = size - (margin + (y - min_y) * scale)
        return px, py

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" '
        f'height="{size}" viewBox="0 0 {size} {size}">',
        f'<rect width="{size}" height="{size}" fill="#0f1117" />',
        _polyline(gt, tf, "#39d353", width=3.0),          # true path
        _polyline(est, tf, "#f2a900", width=1.6, dash="6,4"),  # estimate
    ]

    sx, sy = tf(0.0, 0.0)
    parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="7" fill="#58a6ff" />')
    fx, fy = tf(result.summary["gt_final"][0], result.summary["gt_final"][1])
    parts.append(f'<circle cx="{fx:.1f}" cy="{fy:.1f}" r="7" fill="#ff5555" />')

    err = result.summary["final_return_error_m"]
    legend = [
        ("#39d353", "true trajectory"),
        ("#f2a900", "dead-reckoned estimate"),
        ("#58a6ff", "true start (home)"),
        ("#ff5555", f"final position  (err {err} m)"),
    ]
    for i, (color, label) in enumerate(legend):
        y = 30 + i * 22
        parts.append(f'<rect x="20" y="{y - 11}" width="16" height="10" '
                     f'fill="{color}" />')
        parts.append(f'<text x="44" y="{y}" fill="#c9d1d9" '
                     f'font-family="monospace" font-size="14">{label}</text>')

    parts.append('</svg>')
    with open(path, "w") as fh:
        fh.write("\n".join(parts))
