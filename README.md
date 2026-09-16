# 🗺️ Route & Network Schematization

A schematization algorithm for street networks aimed at **facilitating the readability of routes**. It uses **Integer Linear Programming (ILP)** to produce schematic maps that preserve the topology of the network while optimizing route layout criteria.

[![Paper](https://img.shields.io/badge/Paper-CaGIS%202023-blue)](https://doi.org/10.1080/15230406.2022.2125077)
[![OSF](https://img.shields.io/badge/OSF-Project-orange)](https://osf.io/vc3kw/overview)
[![Java](https://img.shields.io/badge/Java-CPLEX-red)]()

---

## 📖 Overview

This is a schematization algorithm for street networks aimed at facilitating the **readability of routes**. It uses **ILP** to transform a geographic route and its surrounding street network into a schematic layout that is easier to read, while preserving the topological and qualitative characteristics of the original network.

The algorithm optimizes the route layout criteria first, and afterward adds the surrounding street network, adapting it to the schematic route's distortions. The resulting **schematic 'route + network' maps** aim to satisfy three requirements:

- **(i)** better readability of the route with respect to its decision points,
- **(ii)** preserving the qualitative characteristics of the surrounding street network while adapting it to route distortions,
- **(iii)** better visibility of alternative routes within the street network.

---

## 🧠 The ILP Model

An ILP schematization process gets as input a graph **G = (V, E)**, where:

- **V** is the set of nodes, each of which has x and y Cartesian coordinates, represented here as **x′(v)** and **y′(v)** for all **v ∈ V**;
- **E** is the set of edges **e = uv** composed by a pair of distinct nodes, where **u** and **v ∈ V** and where there is a direct link between **u** and **v**.

The expected output are the **schematized coordinates x(v) and y(v)** for all **v ∈ V**, such that the layout **hard constraints** are satisfied and the layout **soft constraints** are optimized.

### Hard Constraints

The hard constraints are:

- **Octilinearity** — every edge lies on one of the eight octilinear directions (0°, 45°, …, 315°).
- **Best turn at DP** — the turn at each decision point is snapped to the best direction according to the chosen direction model.
- **Stub length** — dead-end edges (stubs) are given a fixed minimum length.
- **Circular order** — the cyclic order of edges around each intersection node is preserved.
- **Planarity** — no edge crossings are introduced in the output.

The last two — **circular order** and **planarity** — guarantee the **topologically correct output**.

### Soft Constraints

The soft constraints are:

- **Bend minimization** — reduces the number of bends and penalizes sharp turns; bends at intersections receive a higher weight.
- **Edge orientation** — keeps each edge as close as possible to its original orientation.
- **Node position** — minimizes the displacement of nodes from their original position.
- **Route sections proportion** — preserves the relative proportion of route sections, both at decision points and across all points.

The constraints **best turn at DP**, **stub length**, **node position**, and **route sections proportion** were newly implemented in this work.

---

## 🎯 Direction Models

The route layout can be optimized under four direction models, selected via the `directionModel` parameter:

| Model | ID | Behavior |
|-------|-----|----------|
| **None** | `0` | Free — the optimizer chooses the turn direction. |
| **Best-direction** | `1` | Edges incident to a DP are forced to their best octilinear sector. |
| **Traditional** | `2` | Turn snapped to the nearest octilinear turn direction. |
| **Klippel** | `3` | Turn snapped using Klippel's turn categorization. |

Model selection strongly affects how **readable** the route feels at intersections.

---

## 🔄 Two-Step Schematization

### Step 1 — Route Schematization

The route is schematized first using an ILP with the hard and soft constraints described above. The route layout criteria — bend minimization, edge orientation, node position, and route section proportion — are all optimized here, giving a schematic route that is easy to read with respect to its decision points.

### Step 2 — Network Adaptation

After the route is schematized, the surrounding street network is added and adapted to the schematic route's distortions. The network schematization:

- **Fixes already-schematized nodes** — nodes that belong to the route keep their schematic positions.
- **Adapts edge lengths** — each street edge receives a *preferred length* based on its distance to the route, so the network smoothly inherits the route's local scale variations.
- **Preserves circular order** — around every intersection node.
- **Keeps topology** — planarity is enforced via lazy constraint generation.

---

## 🔧 Implementation Notes

The algorithm is implemented in **Java** using **IBM CPLEX** for the ILP/MIP model.

### Topology Check

Planarity is not modeled up front. Instead, the algorithm uses **lazy constraint generation**:

1. Solve the ILP without planarity constraints.
2. Detect edge pairs that cross in the solution.
3. Add planarity constraints only for those pairs.
4. Re-solve. Repeat until no crossings remain.

This keeps the model small and solvable, while still guaranteeing a planar output.

### Octilinear Bounding Box

Each variable's domain is restricted to an **octilinear bounding box** around the route, which tightens the LP relaxation and speeds up the solve.

### Time and Gap Control

The solver can be controlled either by a **time limit** (`executionTimeLimit > 0`) or by an **MIP gap** (`executionTimeLimit < 0`).

---

## 📄 Publication

If you use this algorithm, please cite:

```bibtex
@article{galvao2023schematizing,
  title   = {Schematizing car routes with their surrounding street network},
  author  = {Galv{\~a}o, Marcelo L. and Krukar, Jakub and Schwering, Angela},
  journal = {Cartography and Geographic Information Science},
  volume  = {50},
  number  = {1},
  pages   = {20--43},
  year    = {2023},
  doi     = {10.1080/15230406.2022.2125077}
}
