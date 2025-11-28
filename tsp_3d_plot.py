#!/usr/bin/env python3
"""
3D TSP Jump Planner + Visualizer – WITH DIAGNOSTICS
- Auto-fixes backend issues
- Always saves PNG
- Debugs why interactive plot might fail
"""

import csv
import json
import os
import math
import time
from typing import List, Tuple, Dict, Optional

# --- Matplotlib Diagnostics & Fix ---
import matplotlib
print(f"Matplotlib v{matplotlib.__version__} | Backend: {matplotlib.get_backend()}")

# Try to set interactive backend if possible
try:
    matplotlib.use('Qt5Agg')  # Or 'TkAgg' if Qt fails
    print("Switched to Qt5Agg backend for interactive plots.")
except:
    try:
        matplotlib.use('TkAgg')
        print("Switched to TkAgg backend.")
    except:
        print("Warning: Stuck with non-interactive backend (PNG only). Install: pip install PyQt5")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

CONFIG_FILE = "tsp_config.json"
DEFAULT_CONFIG = {
    "max_jump_range": 65.0,
    "filename": "sys_coor.csv",
    "loop_back": "No",
    "final_is_last": "Yes"
}

class System:
    def __init__(self, name: str, x: float, y: float, z: float):
        self.name = name.strip('"')
        self.x, self.y, self.z = x, y, z

    def distance_to(self, other: 'System') -> float:
        return math.hypot(self.x - other.x, self.y - other.y, self.z - other.z)

    def jumps_to(self, other: 'System', max_range: float) -> int:
        d = self.distance_to(other)
        return max(1, math.ceil(d / max_range)) if d > 1e-6 else 0

def load_systems(filename: str) -> List[System]:
    if not os.path.exists(filename):
        print(f"File not found: {filename}")
        return []
    systems = []
    with open(filename, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) >= 4:
                try:
                    name, x, y, z = row[0], *map(float, row[1:4])
                    systems.append(System(name, x, y, z))
                except ValueError:
                    continue
    return systems

def load_config() -> Dict:
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE) as f:
                return json.load(f)
        except:
            pass
    return DEFAULT_CONFIG.copy()

def save_config(cfg: Dict):
    with open(CONFIG_FILE, 'w') as f:
        json.dump(cfg, f, indent=2)

def get_user_input():
    cfg = load_config()
    print("\n" + "═"*62)
    print("  3D TSP JUMP PLANNER + VISUALIZER  ".center(62))
    print("═"*62 + "\n")

    max_jump = float(input(f"Max jump range (LY) [default {cfg['max_jump_range']}]: ") or cfg['max_jump_range'])
    filename = input(f"CSV file [default {cfg['filename']}]: ").strip() or cfg['filename']
    loop_back = (input(f"Loop back to start? (yes/No) [default {cfg['loop_back']}]: ").strip().lower() or cfg['loop_back'].lower()) == "yes"
    final_is_last = True
    if not loop_back:
        final_is_last = (input(f"End at last system in file? (Yes/No) [default {cfg['final_is_last']}]: ").strip().lower() or cfg['final_is_last'].lower()) != "no"

    cfg.update({"max_jump_range": max_jump, "filename": filename,
                "loop_back": "Yes" if loop_back else "No",
                "final_is_last": "Yes" if final_is_last else "No"})
    save_config(cfg)
    return max_jump, filename, loop_back, final_is_last

def total_cost(path, rng, loop):
    if len(path) < 2: return 0, [], 0.0, []
    jumps, dists = [], []
    tj = tly = 0
    for a, b in zip(path[:-1], path[1:]):
        d = a.distance_to(b); j = a.jumps_to(b, rng)
        jumps.append(j); dists.append(d); tj += j; tly += d
    if loop:
        d = path[-1].distance_to(path[0]); j = path[-1].jumps_to(path[0], rng)
        jumps.append(j); dists.append(d); tj += j; tly += d
    return tj, jumps, tly, dists

def nearest_neighbor(systems, start, fixed_end, rng):
    unvisited = set(systems) - {start}
    path = [start]
    cur = start
    while unvisited:
        nxt = min(unvisited, key=lambda s: cur.jumps_to(s, rng))
        path.append(nxt); unvisited.remove(nxt); cur = nxt
    if fixed_end and path[-1] != fixed_end:
        if fixed_end in path: path.remove(fixed_end)
        path.append(fixed_end)
    return path

def two_opt(path, rng, loop):
    best = path[:]
    best_j, _, best_ly, _ = total_cost(best, rng, loop)
    n = len(best)
    improved = True
    while improved:
        improved = False
        for i in range(1, n-2):
            for k in range(i+1, n):
                if loop and (i == 1 or k == n-1): continue
                new = best[:i] + best[i:k+1][::-1] + best[k+1:]
                nj, _, nly, _ = total_cost(new, rng, loop)
                if nj < best_j or (nj == best_j and nly < best_ly):
                    best = new; best_j = nj; best_ly = nly; improved = True
            if improved: break
    j, jumps, ly, dists = total_cost(best, rng, loop)
    return best, jumps, j, ly, dists

def solve(systems, max_jump, loop_back, final_is_last):
    if len(systems) < 2: return systems, [], 0, 0.0, []
    start = systems[0]
    fixed_end = systems[-1] if not loop_back and final_is_last else None
    pool = [s for s in systems if s != fixed_end]
    path = nearest_neighbor(pool, start, fixed_end, max_jump)
    path, jumps, tj, tly, dists = two_opt(path, max_jump, loop_back)
    if loop_back and path[0] != path[-1]:
        path.append(path[0])
        d = path[-2].distance_to(path[0])
        j = path[-2].jumps_to(path[0], max_jump)
        jumps.append(j); dists.append(d)
    return path, jumps, tj, tly, dists

def plot_3d_route(path: List[System], total_ly: float, filename: str):
    print("Creating 3D plot...")
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    xs, ys, zs = [s.x for s in path], [s.y for s in path], [s.z for s in path]
    ax.plot(xs, ys, zs, color='cyan', linewidth=2.5)
    ax.scatter(xs, ys, zs, c='white', s=70, edgecolors='black')

    for s in path:
        ax.text(s.x, s.y, s.z + 4, s.name, color='yellow', fontsize=9, ha='center')

    ax.scatter([path[0].x], [path[0].y], [path[0].z], c='lime', s=300, marker='*', edgecolors='black', label='Start')
    if path[0] == path[-1]:
        ax.scatter([path[-1].x], [path[-1].y], [path[-1].z], c='red', s=250, marker='X', edgecolors='black', label='Loop')
    else:
        ax.scatter([path[-1].x], [path[-1].y], [path[-1].z], c='orange', s=200, marker='D', edgecolors='black', label='End')

    ax.set_title(f"3D Jump Route – {total_ly:.1f} LY total", color='white', fontsize=16)
    ax.set_xlabel('X (ly)'); ax.set_ylabel('Y (ly)'); ax.set_zlabel('Z (ly)')

    ax.xaxis.pane.set_facecolor((0,0,0,0.6))
    ax.yaxis.pane.set_facecolor((0,0,0,0.6))
    ax.zaxis.pane.set_facecolor((0,0,0,0.6))
    ax.grid(True, alpha=0.3)
    ax.set_facecolor('black')
    fig.patch.set_facecolor('black')
    ax.tick_params(colors='white')
    ax.legend(facecolor='black', edgecolor='cyan', labelcolor='white')

    plt.tight_layout()
    plt.savefig(filename, dpi=200, facecolor='black', bbox_inches='tight')
    print(f"PNG saved → {filename} (open in any image viewer)")

    # Try interactive show
    try:
        print("Attempting interactive window...")
        plt.show(block=False)  # Non-blocking
        input("Interactive 3D window open? Press Enter to close...")
        plt.close()
        print("Interactive plot closed.")
    except Exception as e:
        print(f"Interactive failed (normal on headless): {e}")
        print("Use the PNG file instead!")

def print_route(path, jumps, total_jumps, total_ly):
    print("\n" + "═"*92)
    print("FINAL ROUTE".center(92))
    print("═"*92)
    cum = 0.0
    for i, (s, j) in enumerate(zip(path, jumps), 1):
        if i > 1: cum += path[i-2].distance_to(path[i-1])
        marker = "START" if i == 1 else "END/LOOP" if i == len(path) else ""
        print(f"{i:3d} │ {s.name:<38} │ {j:>3} jumps │ {cum:7.1f} LY  {marker}")
    print("═"*92)
    systems = len(path) - (1 if path and path[0] == path[-1] else 0)
    print(f"Systems visited: {systems} │ Total jumps: {total_jumps} │ Total distance: {total_ly:.1f} LY")
    print("═"*92)

def save_csv(path, jumps, dists, total_jumps, total_ly):
    with open("route_output.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["Step","System","Jumps","Leg_LY","Total_Jumps","Total_LY","X","Y","Z"])
        tj = tly = 0
        for i, (s, j, d) in enumerate(zip(path, jumps, dists), 1):
            tj += j; tly += d
            w.writerow([i, s.name, j, round(d,3), tj, round(tly,3), s.x, s.y, s.z])
    print("CSV saved → route_output.csv")

def main():
    max_jump, filename, loop_back, final_is_last = get_user_input()
    systems = load_systems(filename)
    if not systems:
        print("No systems loaded – exiting.")
        return

    print(f"\nLoaded {len(systems)} systems – calculating best route...")
    path, jumps, total_jumps, total_ly, dists = solve(systems, max_jump, loop_back, final_is_last)
    print_route(path, jumps, total_jumps, total_ly)

    # CSV
    ans = input("\nSave route to CSV? (Y/n): ").strip().lower()
    if ans != "n":
        save_csv(path, jumps, dists, total_jumps, total_ly)

    # 3D Plot – default YES
    ans = input("\nGenerate 3D visualization? (Y/n): ").strip().lower()
    if ans != "n":
        png = input("PNG filename [Enter = route_3d.png]: ").strip() or "route_3d.png"
        if not png.lower().endswith(('.png','.jpg','.jpeg')):
            png += ".png"
        plot_3d_route(path, total_ly, png)

    print("\nAll done – o7 CMDR!")

if __name__ == "__main__":
    main()
