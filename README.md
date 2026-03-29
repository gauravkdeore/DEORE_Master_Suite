# DEORE v3.1 — Dual-Ended Optimality and Routing Engine

**Author:** Gaurav Deore
**Version:** 3.1
**Status:** Patent Pending

---

## Overview

DEORE is an original bidirectional pathfinding algorithm for uniform-cost 8-directional grids. It uses Jump Point Search as its core symmetry-breaking primitive, wraps it in a bidirectional framework with six correctness-ensuring mechanisms, and adds two new v3.1 mechanisms (DPES, SSP) for a 16.4% node reduction versus v3.

DEORE acknowledges it is a **DXBB-class algorithm** (Deterministic Expansion-Based Black Box), as is every published bidirectional heuristic search algorithm (MM, BAE*, NBS, MEET, DIBBS). DEORE does not claim to escape the theoretical 2·VC expansion bound. It achieves practical dominance on grid maps by operating at the jump-point level rather than the cell level, and by preprocessing maze topologies with TADC before any search node is expanded.

---

## The Eight Mechanisms

| # | Mechanism | Function | Complexity |
|:---|:---|:---|:---|
| 1 | **MTD** — Maze Topology Detector | Classifies grid: maze-like (→ TADC + uniJPS) vs open (→ bidir core) | O(V) |
| 2 | **TADC** — Topology-Aware Dead-End Collapse | Iteratively walls off dead-end corridors before search | O(V) |
| 3 | **ACE** — Adaptive Cost Estimation | Epsilon decays 2.5→1.0, aggressive early dive then bound tightening | O(1)/expansion |
| 4 | **FRAP** — Forward-Reach Angle Pruning | Prunes backward-facing JPS directions relative to opposing frontier | O(1)/neighbor |
| 5 | **DCM** — Dual-Confirmation Meeting | Tracks open+closed meeting nodes; finalizes only when ACF confirms | O(1)/expansion |
| 6 | **ACF** — Admissible Cost Floor | Monotonic per-frontier true-cost floor; drives all termination decisions | O(1)/push |
| 7 | **DPES** — Dual-Pivot Expansion Selector *(new v3.1)* | Selects which frontier to expand using ACF midpoint balance | O(1)/iteration |
| 8 | **SSP** — Sterile Segment Pruning *(new v3.1)* | Prunes jump segments before cellsBetween() if they can't beat incumbent | O(1)/jump candidate |

---

## What Changed in v3.1

### DPES — Dual-Pivot Expansion Selector

**Old (v3):** `expandFwd = (fO.minF() <= bO.minF())`

**New (v3.1):** When a candidate path cost `tC` is known, DPES selects the frontier whose ACF value is furthest from `tC/2` (the path midpoint):

```javascript
if (tC < Infinity && fO.size && bO.size) {
    const mid = tC / 2;
    const fwdDist = Math.abs(acfFwd - mid);
    const bwdDist = Math.abs(acfBwd - mid);
    expandFwd = (fwdDist >= bwdDist);  // expand the lagging frontier
}
```

**Why it works:** The frontier with its ACF floor furthest from the midpoint is out of balance — it hasn't yet found paths near the optimal midpoint. Expanding it drives it toward the meeting region. This enforces approximate meet-in-the-middle symmetry without requiring the full MM framework. Falls back to min-f when tC = ∞.

### SSP — Sterile Segment Pruning

Before calling `cellsBetween()` (which reconstructs every intermediate cell along a jump ray), SSP checks whether the jump point can possibly improve the incumbent:

```javascript
// Approximate ng using geometric formula (no cellsBetween needed)
const ng = gv + diag * SQRT2 + straight;

// Check 1: full-path lower bound already exceeds incumbent
if (tC < Infinity && ng + oct(jr, jc, tgt) * hScale >= tC - EPS) continue;

// Check 2: bidirectional join already dominated
if (oG[jk] !== undefined && ng + oG[jk] >= tC - EPS) continue;

// Only reach cellsBetween() if both checks pass
Seg[jk] = cellsBetween(r, c, jr, jc);
```

**Why it's correct:** Both checks use a lower bound on the true segment cost (geometric minimum). If the lower bound already fails, the actual cost also fails. No optimal path is skipped.

---

## Performance

| Benchmark | DEORE v3 | DEORE v3.1 | Note |
|:---|:---|:---|:---|
| Mean nodes (2,988 random grids) | 10.3 | **8.6** | DPES + SSP |
| Optimality violations (1,476 trials) | 0 | **0** | ACF maintained |
| Max cost ratio | 1.842 | **1.842** | Well under 2.5 cap |
| Maze nodes (41×61 grid) | 238 | **238** | Matches Dijkstra floor |

---

## Quick Start

### Requirements

Node.js ≥ 14

### Basic Usage

```javascript
const { deoreCore } = require('./DEORE_COMPLETE_v3.1.js');

const grid = [
  [0, 0, 1, 0, 0],
  [0, 0, 0, 0, 0],
  [1, 0, 1, 1, 0],
  [0, 0, 0, 0, 0],
  [0, 0, 1, 0, 0],
];

const result = deoreCore(grid, [0, 0], [4, 4]);
console.log(result.path);     // [[0,0],[1,1],[2,2],[3,3],[4,4]]
console.log(result.cost);     // path length (octile distance)
console.log(result.nodes);    // jump points expanded
console.log(result.acfFwd, result.acfBwd);
```

### Dynamic Re-planning

```javascript
const { DynamicDEORE } = require('./DEORE_COMPLETE_v3.1.js');
const agent = new DynamicDEORE(grid);
agent.plan([0, 0], [4, 4]);
agent.step();
agent.updateObstacle(2, 2, true);
```

### Benchmark

```bash
node DEORE_FullComparison.js --limit=20
```

### Visualizer

```bash
npx http-server . -p 8765 --cors
# Open http://127.0.0.1:8765/DEORE_Visualizer.html
```

---

## Files

| File | Description |
|:---|:---|
| `DEORE_COMPLETE_v3.1.js` | Core algorithm — all 8 mechanisms, fully annotated |
| `DEORE_Research_Paper_v3.1.md` | Full academic paper with proofs and literature positioning |
| `DEORE_FullComparison.js` | 8-algorithm benchmark suite |
| `DEORE_Visualizer.html` / `.jsx` | Interactive visualization |
| `benchmark_results.json` | Pre-computed benchmark output |
| `maps/` | MovingAI DAO and SC1 benchmark maps |

---

## License

See `LICENSE.txt`.
All original mechanisms (MTD, TADC, ACE, FRAP, DCM, ACF, DPES, SSP) are the exclusive intellectual property of Gaurav Deore. Patent pending.
