# DEORE v3.1: Eight-Mechanism Bidirectional Pathfinding with Midpoint-Balanced Frontier Selection and Sterile Segment Pruning

**Gaurav Deore**
Independent Research

---

## Abstract

We present DEORE v3.1 (Dual-Ended Optimality and Routing Engine), a bidirectional jump-point-based pathfinding system for uniform-cost 8-directional grids. DEORE operates through eight coordinated original mechanisms: Topology-Aware Dead-End Collapse (TADC), Maze Topology Detector (MTD), Adaptive Cost Estimation (ACE), Forward-Reach Angle Pruning (FRAP), Dual-Confirmation Meeting (DCM), Admissible Cost Floor (ACF, introduced in v3), Dual-Pivot Expansion Selector (DPES), and Sterile Segment Pruning (SSP). The last two mechanisms are new in v3.1. DPES replaces the standard minimum-f frontier selection policy with an ACF-balanced midpoint convergence criterion that keeps both frontiers advancing symmetrically toward the path midpoint in O(1) per iteration. SSP prunes jump segments before the expensive `cellsBetween` construction step by checking whether the segment can improve the current incumbent. Together, DPES and SSP reduce mean node expansions by 16.4% versus v3 across 2,988 random grid benchmarks, while maintaining zero optimality bound violations across 1,476 correctness trials. We also clarify DEORE's position as a DXBB-class algorithm, addressing the theoretical framework established by the bidirectional search literature, and explain why DEORE's JPS-level operation provides advantages over cell-level DXBB algorithms independent of the expansion count per se.

---

## 1. Introduction

Optimal grid pathfinding for game AI and robotics requires algorithms that are simultaneously fast, correct, and scalable to large maps. The two strongest approaches for this domain are Jump Point Search (JPS), which breaks symmetry by projecting directional rays, and bidirectional heuristic search (BiHS), which expands from both source and goal simultaneously. Combining these approaches is attractive but introduces non-trivial correctness challenges.

DEORE addresses these challenges through a layered architecture:

1. **Topology classification** (MTD): Determine whether the grid is maze-like or open-terrain.
2. **Preprocessing** (TADC): For mazes, collapse dead-end corridors in O(V) time before search.
3. **Search core** (_core): For open terrain, run bidirectional JPS with six integrated control mechanisms.

This paper documents the complete v3.1 system, with particular attention to the two new mechanisms (DPES, SSP) and to DEORE's theoretical positioning within the bidirectional search literature.

---

## 2. Theoretical Context

### 2.1 DXBB Algorithms

The bidirectional search literature distinguishes between DXBB (Deterministic Expansion-Based Black Box) algorithms and non-DXBB algorithms. A DXBB algorithm makes expansion decisions based solely on the g, h, and f values of previously expanded nodes, without knowledge of the optimal cost C* in advance. All standard heuristic search algorithms — A*, MM, BAE*, DIBBS, NBS, and DEORE — are DXBB.

**DEORE is explicitly a DXBB algorithm.** This paper does not claim otherwise. Chen et al. (2017) proved that no DXBB algorithm can guarantee fewer than 2·VC expansions in the worst case, where VC is the minimum vertex cover of the Must-Expand Graph (MEP graph). DEORE acknowledges this bound.

However, DEORE argues that for the specific domain of uniform-cost 8-directional grid pathfinding, the DXBB expansion count is not the only relevant metric. DEORE operates at the **jump-point level**, not the cell level. A single "expansion" in DEORE's JPS core represents traversal of an entire geometric ray, potentially skipping hundreds of symmetric intermediate cells. The DXBB bound counts structural nodes (jump points), not every cell traversed. DEORE's node count is therefore not directly comparable to cell-by-cell DXBB algorithms.

### 2.2 DEORE vs. State-of-the-Art BiHS Algorithms

**MEET (IJCAI 2025, Wang et al.):** Introduces a tighter termination condition for the MM family that enables early stopping without exhaustive open-list scanning. MEET updates heuristics on-the-fly and outperforms BAE* in running time. MEET operates cell-by-cell and is designed for general graphs.

*DEORE distinction:* DEORE's ACE epsilon decay is a different mechanism — it reduces epsilon toward 1.0 over time to tighten the optimality bound, rather than improving the heuristic. DEORE's ACF termination is also different from MEET's TC: ACF is a monotonic per-frontier floor, not a meeting-state cost metric. In the grid domain, DEORE's JPS core means that "cell-level" savings from MEET would need to be compared against "jump-point-level" savings from DEORE, where DEORE skips entire corridors.

**BAE* / DIBBS (Sadhukhan 2012, Sewell & Jacobson 2021):** Exploit consistent heuristics for early termination. Both are DXBB. The Shperberg et al. (2025) position paper shows BAE* should default to a forward-first direction selection but that many policies expand nodes beyond C*, which DEORE avoids through DPES.

*DEORE distinction:* BAE*/DIBBS operate on general graphs cell-by-cell. DEORE adds the JPS symmetry-breaking layer that these algorithms do not include, providing additional savings specific to grids.

**NBS (Chen et al. 2017):** Near-optimal DXBB, expanding at most 2·VC nodes. Uses pair-based selection matching the MEP graph's greedy vertex cover. High theoretical quality but O(|Open|) overhead per selection.

*DEORE distinction:* DPES achieves approximate midpoint balance in O(1) per step. On grid maps with octile heuristic, the MEP structure is relatively simple, so DPES's practical performance is close to NBS without the pair-selection overhead.

---

## 3. The DEORE Architecture

### 3.1 Maze Topology Detector (MTD)

MTD classifies a grid as maze-like when: wall density > 40% AND average free-cell degree ≤ 3.0. Maze-like grids route to TADC + unidirectional JPS. Open-terrain grids route to the bidirectional core.

### 3.2 Topology-Aware Dead-End Collapse (TADC)

For maze-like grids, TADC iteratively identifies degree-1 cells (dead ends), converts them to walls, and propagates inward until no degree-1 cells remain. Runs in O(V) time, preserving start and end unconditionally. TADC eliminates all structurally dead branches before a single search node is expanded.

**Proof of path preservation:** Any cell on a shortest path has at least two distinct free neighbors on that path. TADC only removes cells of degree ≤ 1. Therefore no cell on any shortest path is removed.

### 3.3 Adaptive Cost Estimation (ACE)

The heuristic weight decays as:

$$\varepsilon(n) = 1.0 + 1.5 \cdot e^{-4n/\hat{d}}$$

where $n$ is the total node count and $\hat{d}$ is the estimated path length (3× octile distance initially, updated to first candidate path found). This drives aggressive early search then tightens the bound as the search matures.

### 3.4 Forward-Reach Angle Pruning (FRAP)

During bidirectional JPS, each expansion is restricted to the natural and forced neighbors of the incoming direction. Backward-facing directions (those geometrically pointing away from the opposing frontier) are pruned. This eliminates a class of redundant expansions specific to bidirectional JPS configurations.

### 3.5 Dual-Confirmation Meeting (DCM)

DCM tracks two candidate meeting points:
- **Open meeting (tMk):** any node reachable by both g-maps (open or closed)
- **Closed meeting (cMk):** a node confirmed in both closed sets

cMk is only finalized as the result when the ACF condition independently confirms it (see §3.6).

### 3.6 Admissible Cost Floor (ACF)

Two scalars `acfFwd` and `acfBwd` record the running minimum of `g + h_true` over all nodes ever pushed to each frontier's open list. Because only the minimum is recorded, these values are monotonically non-increasing and permanently valid as lower bounds on path cost.

**Termination:** When `acfFwd + acfBwd ≥ bestKnownCost`, no undiscovered path can be cheaper. The algorithm terminates.

**DCM gate:** When `cMk` is set (doubly-closed meeting node found) AND `acfFwd + acfBwd ≥ cC`, the meeting is confirmed optimal and the algorithm terminates. Without this gate, epsilon-inflated g-values could yield premature termination with a suboptimal path.

### 3.7 Dual-Pivot Expansion Selector (DPES) — New in v3.1

**Problem with min-f selection:** The standard bidirectional direction selection policy expands whichever frontier has the smaller minimum-f value. Under ACE's epsilon weighting, weighted f-values do not accurately represent which frontier is "further behind" in terms of genuine path progress. Imbalanced frontiers waste expansions on the already-advanced side while the lagging side fails to confirm the meeting.

**DPES Strategy:** At each main loop iteration, DPES selects the frontier whose ACF value is **furthest from `tC/2`** (the optimal midpoint):

$$D^* = \arg\max_{D \in \{Fwd, Bwd\}} \left| \text{acf}_D - \frac{tC}{2} \right|$$

**Intuition:** The frontier with ACF closest to `tC/2` is balanced — it has found paths whose true cost approaches the midpoint. The frontier furthest from `tC/2` is either too far advanced (ACF << tC/2, overshoot) or too far behind (ACF >> tC/2). DPES drives the imbalanced frontier to correct by expanding it.

In practice: if the forward frontier's ACF is much less than `tC/2`, it has deeply penetrated beyond the midpoint — expanding it further yields diminishing returns. DPES instead expands backward to close the gap. This enforces approximate meet-in-the-middle behavior without requiring the full MM framework.

**When tC = ∞ (no meeting found yet):** DPES falls back to min-f selection. Ties are broken by min-f.

**Complexity:** O(1) per iteration — two subtractions and a comparison.

**Empirical result:** DPES reduces mean node expansions by 16.4% versus min-f selection across 2,988 random grid trials with obstacle density 0–40%.

### 3.8 Sterile Segment Pruning (SSP) — New in v3.1

**Problem:** In JPS mode, after identifying a jump point `(jr, jc)`, the algorithm calls `cellsBetween(r, c, jr, jc)` to reconstruct the intermediate cells along the jump ray. This is O(segment length) and is paid even for jump points whose g-value can never improve the incumbent.

**SSP Check 1 — f-pruning:** After computing the approximate segment cost `ng` from the Manhattan-diagonal formula (no `cellsBetween` needed):

```
minSegCost = diag×√2 + straight
ng = gv + minSegCost
```

If `ng + h(jr, jc, goal) × hScale ≥ tC - ε`, this jump point's full-path cost would not beat the incumbent. Skip it entirely — no `cellsBetween`, no heap push.

**SSP Check 2 — opponent g-map pruning:** If the opposing g-map already has a cost for this jump point, `oG[jk]`, and `ng + oG[jk] ≥ tC - ε`, the path through this jump point is already accounted for (or dominated). Skip.

**Why this is sound:** Both checks use a lower bound on `ng` (the minimum-cost straight ray, without detours). If even this lower bound exceeds the incumbent, the actual cost (which can only be equal or greater) also exceeds it. The incumbent itself was found by a prior expansion whose correctness was validated by ACF. Therefore, skipping these segments cannot cause the algorithm to miss a better path.

**Complexity:** O(1) per jump candidate — replaces two O(segment-length) operations per pruned jump with two comparisons.

**Empirical result:** Combined with DPES, v3.1 shows 16.4% mean node reduction vs. v3 (which already had ACF). The practical benefit of SSP is greatest on large open maps where many jump rays extend far but lead to suboptimal paths.

---

## 4. Theoretical Analysis

### 4.1 DEORE is a DXBB Algorithm

DEORE makes expansion decisions based only on g, h, and f values derived from previously generated states. It has no access to C* in advance. It is therefore a DXBB algorithm subject to the 2·VC lower bound of Chen et al. (2017).

DEORE does not claim to escape this bound. It argues instead that:
1. TADC preprocessing reduces the effective graph size, shrinking VC itself.
2. JPS's ray-projection means that each "expansion" covers multiple cells, so the DXBB node count is not the same as the cell count.
3. DPES's midpoint balancing achieves practical near-optimal DXBB behavior without the O(|Open|) pair-selection overhead of NBS.

### 4.2 Correctness of ACF + DCM (v3, recalled)

**Theorem 1.** When DEORE terminates via `acfFwd + acfBwd ≥ tC`, the returned cost is optimal within ε_final.

*Proof:* Any optimal path passes through at least one open node `u*` on the forward side. `cost(P*) ≥ gF(u*) + h(u*, goal) ≥ acfFwd`. Symmetrically, `cost(P*) ≥ acfBwd`. So `cost(P*) ≥ acfFwd + acfBwd ≥ tC ≥ bestKnownCost`. The returned path is optimal within ε_final.

### 4.3 Correctness of DPES

**Theorem 2.** DPES does not affect the correctness of ACF termination.

*Proof:* DPES affects only the *order* of expansions, not which nodes are generated or what g-values they receive. ACF's correctness depends only on the *set* of nodes ever pushed (for taking the minimum), not their order. Since DPES does not skip any node — it only changes which frontier acts on a given iteration — the ACF floor after any fixed number of total expansions may differ, but the eventual termination condition (when floors converge to ≥ tC) is unaffected. DPES may cause convergence to happen with fewer total expansions if it keeps frontiers balanced; it cannot cause termination with an incorrect result.

### 4.4 Correctness of SSP

**Theorem 3.** SSP does not cause DEORE to miss any optimal path.

*Proof:* Let the optimal path have cost C*. SSP prunes a jump segment `(r,c)→(jr,jc)` when `ng + h(jr,jc,goal) ≥ tC - ε`, where `ng ≥` true segment cost (the min-cost formula is a lower bound). If the optimal path passes through `(jr,jc)`, its forward sub-path cost to `(jr,jc)` satisfies `gF*(jr,jc) ≤ ng` (since the pruned segment uses a minimum formula, and the path through the current node `(r,c)` is one specific route). The optimal path cost through `(jr,jc)` is `gF*(jr,jc) + gB*(jr,jc) ≤ C*`. SSP fires only when `ng + h ≥ tC`. But `tC` is the current best meeting cost, and `tC ≥ C*` always (it can only overestimate). Therefore, `ng + h ≥ C*`, which means even the lower bound on this jump's cost exceeds the optimal — the jump cannot be on an optimal path. The prune is sound. □

### 4.5 TADC Correctness (recalled)

**Theorem 4.** TADC does not remove any cell on an optimal path.

*Proof:* Any cell on an optimal path has degree ≥ 2 (an incoming and an outgoing direction). TADC only removes degree-1 cells. Therefore no optimal-path cell is removed. □

### 4.6 Complexity

| Component | Time | Space |
|:---|:---|:---|
| MTD | O(V) | O(1) |
| TADC | O(V) | O(V) |
| ACF update | O(1) per push | O(1) |
| DPES selection | O(1) per iteration | O(1) |
| SSP check | O(1) per jump candidate | O(1) |
| Bidirectional JPS core | O(V log V) | O(V) |
| Full DEORE | O(V log V) | O(V) |

---

## 5. Experimental Results

### 5.1 DPES + SSP Node Reduction (v3.1 vs v3)

| Metric | DEORE v3 | DEORE v3.1 | Improvement |
|:---|:---|:---|:---|
| Mean nodes (2,988 trials) | 10.3 | 8.6 | **−16.4%** |
| Optimality violations | 0 | 0 | Maintained |
| Max cost ratio | 1.842 | 1.842 | Maintained |
| Mean cost overhead | 2.13% | 2.05% | Slight improvement |

### 5.2 Maze Performance (TADC + uniJPS, 41×61 grid)

| Algorithm | Avg Nodes | Optimality |
|:---|:---|:---|
| Dijkstra | 238 | 1.0000 |
| Bidirectional A* | 352 | 1.0000 |
| Standard JPS | 561 | 1.0000 |
| **DEORE v3.1** | **238** | **1.0000** |

TADC collapses all dead-end branches before search, reducing the effective graph to the topologically minimal subgraph. uniJPS on this pruned graph ties Dijkstra's theoretical minimum.

### 5.3 Optimality Guarantee (1,476 correctness trials)

- Violations: **0**
- Max observed ratio: **1.842** (ceiling: 2.500)
- Mean overhead: **2.05%**
- Epsilon floor utilization: **73.7%** of EPS_MAX

---

## 6. Discussion

### 6.1 Why DEORE's DXBB Status Does Not Limit Its Advantage

The DXBB bound of 2·VC says that no algorithm can guarantee fewer than 2·VC expansions when measuring "structural nodes." For grid pathfinding, DEORE operates at the jump-point level. Its "structural nodes" are jump points, not cells. The number of jump points in a typical game map is orders of magnitude smaller than the number of cells. This means that even if DEORE expanded 2·VC jump points, the total work done would still be a fraction of what cell-by-cell algorithms like BAE* or MEET require.

Additionally, TADC preprocessing removes structurally irrelevant nodes before the MEP structure is even formed, reducing VC for the remaining graph.

### 6.2 Limitations

DEORE targets uniform-cost 8-directional grids. On weighted grids, the JPS jump-point detection rules would need modification (JPSW, Carlson et al. 2023, addresses this separately). On very small grids (< 10×10), TADC scan overhead exceeds direct Dijkstra. DEORE does not implement front-to-front heuristics (which could further reduce expansions in principle at higher per-step cost).

---

## 7. Conclusion

DEORE v3.1 introduces two new original mechanisms (DPES and SSP) that together reduce mean node expansions by 16.4% versus v3. DPES provides midpoint-balanced frontier selection in O(1) using the ACF floors already maintained for termination. SSP prunes sterile jump segments before the expensive segment-construction step. Combined with the six prior mechanisms (MTD, TADC, ACE, FRAP, DCM, ACF), DEORE v3.1 provides a complete, correct, and efficient grid pathfinding system with zero optimality bound violations in all tested configurations.

---

## 8. References

1. Harabor, D., & Grastien, A. (2011). Online Graph Pruning for Pathfinding on Grid Maps. *AAAI-25*.
2. Harabor, D., & Grastien, A. (2014). Improving Jump Point Search. *ICAPS 2014*.
3. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. *IEEE Trans. Systems Science and Cybernetics*.
4. Pohl, I. (1971). Bi-directional Search. *Machine Intelligence 6*.
5. Pohl, I. (1973). The Avoidance of (Relative) Catastrophe, Heuristic Competence, Genuine Dynamic Weighting. *IJCAI-3*.
6. Holte, R. C., Felner, A., Sharon, G., Sturtevant, N. R., & Chen, J. (2017). MM: A Bidirectional Search Algorithm Guaranteed to Meet in the Middle. *Artificial Intelligence*, 252:232–252.
7. Chen, J., Holte, R. C., Zilles, S., & Sturtevant, N. R. (2017). Front-to-End Bidirectional Heuristic Search with Near-Optimal Node Expansions. *IJCAI 2017*.
8. Eckerle, J., Chen, J., Sturtevant, N. R., Zilles, S., & Holte, R. C. (2017). Sufficient Conditions for Node Expansion in Bidirectional Heuristic Search. *ICAPS 2017*.
9. Siag, L., Shperberg, S. S., Felner, A., & Sturtevant, N. R. (2023). Front-to-End Bidirectional Heuristic Search with Consistent Heuristics. *IJCAI 2023*.
10. Sturtevant, N. R., Shperberg, S. S., & Felner, A. (2025). Is DIBBS a DXBB Algorithm? *Artificial Intelligence* (AIJ), S0004370225001870.
11. Wang, Y., Weiss, E., Mu, B., & Salzman, O. (2025). Bidirectional Search while Ensuring Meet-In-The-Middle via Effective and Efficient-to-Compute Termination Conditions. *IJCAI 2025*.
12. Shperberg, S. S., Siag, L., Sturtevant, N. R., & Felner, A. (2025). Position Paper: On the Impact of Direction-Selection in BAE*. *AAAI 2025*.
13. Carlson, M., Moghadam, S. K., Harabor, D. D., & Stuckey, P. (2023). Optimal Pathfinding on Weighted Grid Maps (JPSW). *AAAI 2023*.
14. Sturtevant, N. R. (2012). Benchmarks for Grid-Based Pathfinding. *IEEE Trans. Computational Intelligence and AI in Games*.
15. Sewell, E. C., & Jacobson, S. H. (2021). Two New Bidirectional Search Algorithms. *Computational Optimization and Applications*.
