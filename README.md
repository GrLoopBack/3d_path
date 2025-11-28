# 3D TSP Jump Planner + Visualizer (Elite Dangerous / Star Systems)

A fast Python tool that finds the **shortest + fastest** route through a list of star systems in 3D space — optimized for **fewest jumps** and **total light-years**.  
Includes a stunning **interactive 3D plot** of your route!

Perfect for planning expeditions, trade runs, or neutron highway trips. 
You can get the coordinates from spansh.co.uk search. 

---

### Features
- Heuristic TSP solver (Nearest Neighbor + 2-Opt) → works instantly even with 100+ systems
- Uses real 3D coordinates (X, Y, Z)
- Respects your ship’s **maximum jump range**
- Optional: loop back to start or end at last system in list
- Shows **total jumps** and **total distance in LY**
- Saves detailed CSV
- Generates beautiful **3D route visualization** (PNG + interactive view)

---

## Quick Start


1. Clone or download this script
2. Create and activate a virtual environment (recommended)
```bash
python3 -m venv venv
source venv/bin/activate        # Linux/macOS
# or
venv\Scripts\activate           # Windows
```

### 3. Install required packages
```
pip install matplotlib numpy
```

### 4. Create your systems file: sys_coor.csv

Format (one system per line):

```
"Tsche",1.46875,-66.875,133.28125
"HIP 97927",-18.75,-60.65625,117.25
"Iksvatan",-12.6875,-60.125,116.46875
```
... add as many as you want

### 5. Run the planner
```
python tsp_3d_plot.py
```
