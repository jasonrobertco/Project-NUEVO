from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np


package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from robot.path_planner import LeashedAPFPlanner


OUTPUT_PATH = Path(__file__).resolve().parent / "output" / "lapf_virtual_target_scenarios.svg"
LEASH_LENGTH_MM = 400.0
OBSTACLE_RADIUS_MM = 75.0
INFLATION_MARGIN_MM = 200.0
VIRTUAL_REPULSION_RANGE_MM = 700.0


def simulate_virtual_target(
    *,
    planner: LeashedAPFPlanner,
    pose: tuple[float, float, float],
    goal: tuple[float, float],
    obstacles: np.ndarray,
    steps: int,
    dt: float,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for _ in range(steps):
        pt = planner.update_virtual_target(pose, goal, obstacles, dt)
        points.append(pt)
    return points


def scenario_definitions():
    return [
        {
            "title": "No Obstacles",
            "pose": (0.0, 0.0, 0.0),
            "goal": (1000.0, 0.0),
            "obstacles": np.empty((0, 3)),
            "steps": 16,
            "dt": 0.1,
            "planner": LeashedAPFPlanner(
                max_linear=150.0,
                max_angular=1.0,
                target_speed=200.0,
                repulsion_gain=800.0,
                repulsion_range=VIRTUAL_REPULSION_RANGE_MM,
                goal_tolerance=20.0,
                leash_length_mm=LEASH_LENGTH_MM,
                leash_half_angle_deg=60.0,
                inflation_margin_mm=INFLATION_MARGIN_MM,
            ),
        },
        {
            "title": "Leash Clip +60°",
            "pose": (0.0, 0.0, 0.0),
            "goal": (500.0, 1200.0),
            "obstacles": np.empty((0, 3)),
            "steps": 18,
            "dt": 0.1,
            "planner": LeashedAPFPlanner(
                leash_length_mm=LEASH_LENGTH_MM,
                leash_half_angle_deg=60.0,
                target_speed=220.0,
                repulsion_range=VIRTUAL_REPULSION_RANGE_MM,
                inflation_margin_mm=INFLATION_MARGIN_MM,
            ),
        },
        {
            "title": "Leash Clip -60°",
            "pose": (0.0, 0.0, 0.0),
            "goal": (500.0, -1200.0),
            "obstacles": np.empty((0, 3)),
            "steps": 18,
            "dt": 0.1,
            "planner": LeashedAPFPlanner(
                leash_length_mm=LEASH_LENGTH_MM,
                leash_half_angle_deg=60.0,
                target_speed=220.0,
                repulsion_range=VIRTUAL_REPULSION_RANGE_MM,
                inflation_margin_mm=INFLATION_MARGIN_MM,
            ),
        },
        {
            "title": "Centered Obstacle",
            "pose": (0.0, 0.0, 0.0),
            "goal": (1500.0, 0.0),
            "obstacles": np.array([[800.0, 0.0, OBSTACLE_RADIUS_MM]], dtype=float),
            "steps": 32,
            "dt": 0.1,
            "planner": LeashedAPFPlanner(
                target_speed=200.0,
                repulsion_gain=1200.0,
                repulsion_range=VIRTUAL_REPULSION_RANGE_MM,
                goal_tolerance=20.0,
                leash_length_mm=LEASH_LENGTH_MM,
                leash_half_angle_deg=60.0,
                inflation_margin_mm=INFLATION_MARGIN_MM,
            ),
        },
        {
            "title": "Offset Front Obstacle",
            "pose": (0.0, 0.0, 0.0),
            "goal": (1500.0, 0.0),
            "obstacles": np.array([[800.0, 180.0, OBSTACLE_RADIUS_MM]], dtype=float),
            "steps": 32,
            "dt": 0.1,
            "planner": LeashedAPFPlanner(
                target_speed=200.0,
                repulsion_gain=1200.0,
                repulsion_range=VIRTUAL_REPULSION_RANGE_MM,
                goal_tolerance=20.0,
                leash_length_mm=LEASH_LENGTH_MM,
                leash_half_angle_deg=60.0,
                inflation_margin_mm=INFLATION_MARGIN_MM,
            ),
        },
    ]


def to_panel_coords(
    x_mm: float,
    y_mm: float,
    *,
    panel_left: float,
    panel_top: float,
    panel_width: float,
    panel_height: float,
    bounds: tuple[float, float, float, float],
) -> tuple[float, float]:
    min_x, max_x, min_y, max_y = bounds
    range_x = max(max_x - min_x, 1.0)
    range_y = max(max_y - min_y, 1.0)
    scale = min(panel_width / range_x, panel_height / range_y)
    ox = panel_left + (panel_width - range_x * scale) / 2.0
    oy = panel_top + (panel_height - range_y * scale) / 2.0
    px = ox + (x_mm - min_x) * scale
    py = oy + range_y * scale - (y_mm - min_y) * scale
    return px, py


def scenario_bounds(
    pose: tuple[float, float, float],
    goal: tuple[float, float],
    points: list[tuple[float, float]],
    obstacles: np.ndarray,
    leash_length_mm: float,
    inflation_margin_mm: float,
) -> tuple[float, float, float, float]:
    xs = [pose[0], goal[0], *(p[0] for p in points)]
    ys = [pose[1], goal[1], *(p[1] for p in points)]
    if obstacles.size:
        for ox, oy, radius in obstacles:
            inflated = float(radius) + inflation_margin_mm
            xs.extend([float(ox) - inflated, float(ox) + inflated])
            ys.extend([float(oy) - inflated, float(oy) + inflated])
    pad = max(100.0, leash_length_mm * 0.2)
    return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad


def svg_escape(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def render_svg(output_path: Path) -> None:
    scenarios = []
    for spec in scenario_definitions():
        planner: LeashedAPFPlanner = spec["planner"]
        points = simulate_virtual_target(
            planner=planner,
            pose=spec["pose"],
            goal=spec["goal"],
            obstacles=spec["obstacles"],
            steps=spec["steps"],
            dt=spec["dt"],
        )
        scenarios.append({**spec, "points": points})

    panel_width = 360.0
    panel_height = 320.0
    margin = 24.0
    gap = 20.0
    width = margin * 2 + panel_width * len(scenarios) + gap * (len(scenarios) - 1)
    height = 420.0

    parts: list[str] = []
    parts.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" viewBox="0 0 {width:.0f} {height:.0f}">')
    parts.append('<rect width="100%" height="100%" fill="#0f172a"/>')
    parts.append('<style>')
    parts.append('text { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }')
    parts.append('.title { fill: #e2e8f0; font-size: 18px; font-weight: 700; }')
    parts.append('.subtitle { fill: #94a3b8; font-size: 12px; }')
    parts.append('.panel-title { fill: #e2e8f0; font-size: 14px; font-weight: 700; }')
    parts.append('.panel-box { fill: rgba(255,255,255,0.04); stroke: rgba(255,255,255,0.18); stroke-width: 1; }')
    parts.append('</style>')
    parts.append('<text x="24" y="32" class="title">Leashed APF Virtual Target Scenarios</text>')
    parts.append('<text x="24" y="52" class="subtitle">cyan = virtual target path, green = goal, white = robot origin, red = tracked obstacle, orange dashed = inflated obstacle</text>')

    for idx, spec in enumerate(scenarios):
        left = margin + idx * (panel_width + gap)
        top = 72.0
        parts.append(f'<rect x="{left:.1f}" y="{top:.1f}" width="{panel_width:.1f}" height="{panel_height:.1f}" rx="10" class="panel-box"/>')
        parts.append(f'<text x="{left + 12:.1f}" y="{top + 22:.1f}" class="panel-title">{svg_escape(spec["title"])}</text>')

        planner: LeashedAPFPlanner = spec["planner"]
        leash_length = planner._leash_length  # deliberate local debug use
        inflation_margin = planner._inflation_margin
        bounds = scenario_bounds(
            spec["pose"],
            spec["goal"],
            spec["points"],
            spec["obstacles"],
            leash_length,
            inflation_margin,
        )

        plot_left = left + 12.0
        plot_top = top + 36.0
        plot_width = panel_width - 24.0
        plot_height = panel_height - 48.0
        parts.append(f'<rect x="{plot_left:.1f}" y="{plot_top:.1f}" width="{plot_width:.1f}" height="{plot_height:.1f}" fill="#111827" stroke="rgba(255,255,255,0.08)" stroke-width="1"/>')

        pose = spec["pose"]
        goal = spec["goal"]
        origin_px, origin_py = to_panel_coords(
            pose[0], pose[1],
            panel_left=plot_left,
            panel_top=plot_top,
            panel_width=plot_width,
            panel_height=plot_height,
            bounds=bounds,
        )

        # Leash cone
        min_x, max_x, min_y, max_y = bounds
        scale = min(plot_width / max(max_x - min_x, 1.0), plot_height / max(max_y - min_y, 1.0))
        leash_r_px = leash_length * scale
        half_angle = planner._leash_half_angle
        for sign in (-1.0, 1.0):
            ax = origin_px + math.cos(sign * half_angle) * leash_r_px
            ay = origin_py - math.sin(sign * half_angle) * leash_r_px
            parts.append(f'<line x1="{origin_px:.1f}" y1="{origin_py:.1f}" x2="{ax:.1f}" y2="{ay:.1f}" stroke="rgba(148,163,184,0.45)" stroke-width="1"/>')
        parts.append(f'<circle cx="{origin_px:.1f}" cy="{origin_py:.1f}" r="{leash_r_px:.1f}" fill="none" stroke="rgba(148,163,184,0.18)" stroke-width="1"/>')

        # Obstacles
        if spec["obstacles"].size:
            for ox, oy, radius in spec["obstacles"]:
                cx, cy = to_panel_coords(
                    float(ox), float(oy),
                    panel_left=plot_left,
                    panel_top=plot_top,
                    panel_width=plot_width,
                    panel_height=plot_height,
                    bounds=bounds,
                )
                r_px = float(radius) * scale
                inflated_px = (float(radius) + inflation_margin) * scale
                parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{inflated_px:.1f}" fill="none" stroke="#fb923c" stroke-dasharray="6 4" stroke-width="1.5"/>')
                parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r_px:.1f}" fill="rgba(248,113,113,0.20)" stroke="#f87171" stroke-width="1.5"/>')

        # Goal
        goal_px, goal_py = to_panel_coords(
            goal[0], goal[1],
            panel_left=plot_left,
            panel_top=plot_top,
            panel_width=plot_width,
            panel_height=plot_height,
            bounds=bounds,
        )
        parts.append(f'<line x1="{goal_px - 5:.1f}" y1="{goal_py - 5:.1f}" x2="{goal_px + 5:.1f}" y2="{goal_py + 5:.1f}" stroke="#4ade80" stroke-width="2"/>')
        parts.append(f'<line x1="{goal_px - 5:.1f}" y1="{goal_py + 5:.1f}" x2="{goal_px + 5:.1f}" y2="{goal_py - 5:.1f}" stroke="#4ade80" stroke-width="2"/>')

        # Origin
        parts.append(f'<circle cx="{origin_px:.1f}" cy="{origin_py:.1f}" r="4" fill="#e5e7eb"/>')

        # Path
        if spec["points"]:
            path_d = []
            for i, (x_mm, y_mm) in enumerate(spec["points"]):
                px, py = to_panel_coords(
                    x_mm, y_mm,
                    panel_left=plot_left,
                    panel_top=plot_top,
                    panel_width=plot_width,
                    panel_height=plot_height,
                    bounds=bounds,
                )
                cmd = "M" if i == 0 else "L"
                path_d.append(f"{cmd} {px:.1f} {py:.1f}")
            parts.append(f'<path d="{" ".join(path_d)}" fill="none" stroke="#22d3ee" stroke-width="2.5"/>')
            end_x, end_y = to_panel_coords(
                spec["points"][-1][0], spec["points"][-1][1],
                panel_left=plot_left,
                panel_top=plot_top,
                panel_width=plot_width,
                panel_height=plot_height,
                bounds=bounds,
            )
            parts.append(f'<circle cx="{end_x:.1f}" cy="{end_y:.1f}" r="4" fill="#22d3ee"/>')

        parts.append(f'<text x="{left + 12:.1f}" y="{top + panel_height - 8:.1f}" class="subtitle">leash={leash_length:.0f} mm, infl={inflation_margin:.0f} mm</text>')

    parts.append('</svg>')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(parts), encoding="utf-8")


def main() -> int:
    render_svg(OUTPUT_PATH)
    print(OUTPUT_PATH)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
