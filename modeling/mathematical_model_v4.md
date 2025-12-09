# Mathematical Model for Drone Routing Problem

## Problem Definition

We need to route $K=4$ drones to visit a set of points $P$ in 3D space, starting and ending at a base station $S$. The goal is to minimize the maximum time any single drone takes to complete its tour (Minimax objective).

This model updates the assumptions to:

- **All 4 drones must be used.**
- A drone may **revisit nodes it has already visited**.
- **No drone may visit a node that another drone visits** (exclusive ownership of each grid point).
- **No multiple base touches**: each drone performs a single out-and-back trip (one departure from base, one return to base).
- Objective: **minimize makespan** (latest return time to base).

## Sets and Indices

- $V = P \cup \{S\}$: Set of all nodes, where $S$ is the base station (node 0) and $P = \{1, \dots, N\}$ is the set of target points.
- $E_{ntry} \subset P$: Set of entry points accessible from the base.
- $K = \{1, \dots, 4\}$: Set of drones.
- $A$: Set of valid arcs $(i, j)$.
  - For $i, j \in P$, $(i, j) \in A$ if they satisfy the connectivity constraints (distance $\le 4$ or distance $\le 11$ with specific coordinate constraints).
  - For base connections: $(S, j) \in A$ iff $j \in E_{ntry}$, and $(i, S) \in A$ iff $i \in E_{ntry}$.

## Parameters

- Coordinates $(x_i,y_i,z_i)$ for all $i \in V$.
- Lateral distance $a_{ij} = \sqrt{(x_i-x_j)^2 + (y_i-y_j)^2}$.
- Vertical difference $\Delta z_{ij} = z_j - z_i$.
- Travel time $t_{ij}$ (asymmetric):
  - If $\Delta z_{ij} > 0$: $t_{ij} = \max\!\big(a_{ij}/1.5,\ \Delta z_{ij}/1\big)$.
  - If $\Delta z_{ij} < 0$: $t_{ij} = \max\!\big(a_{ij}/1.5,\ |\Delta z_{ij}|/2\big)$.
  - If $\Delta z_{ij} = 0$: $t_{ij} = a_{ij}/1.5$.
- Big-$M$ for time linking (choose tight, e.g., upper bound on total route time).

## Decision Variables

- $x_{ij}^k \in \mathbb{Z}_{\ge 0}$: number of times drone $k$ flies arc $(i,j)$.
- $y_j^k \in \{0,1\}$: 1 if grid node $j$ is **owned/visited** by drone $k$.
- $f_{ij}^k \ge 0$: flow from base $S$ to owned nodes for connectivity (single-commodity flow per drone).
- $T \ge 0$: makespan (time of the slowest drone).

## Objective Function

Minimize the makespan (time of the longest tour):

$$
\min T
$$

## Constraints

### 1. Exclusive assignment of grid points

Each grid node is owned by exactly one drone:

$$
\sum_{k \in K} y_j^k = 1 \quad \forall j \in P
$$

### 2. Visit constraints based on ownership

If drone $k$ owns $j$, it must enter and leave $j$ at least once. Conversely, if drone $k$ does not own $j$, it cannot visit $j$ (no transit allowed through unassigned nodes).

$$
\sum_{i : (i,j)\in A} x_{ij}^k \ge y_j^k \quad \forall j \in P,\ \forall k \in K
$$

$$
\sum_{i : (j,i)\in A} x_{ji}^k \ge y_j^k \quad \forall j \in P,\ \forall k \in K
$$

$$
\sum_{i : (i,j)\in A} x_{ij}^k \le M \cdot y_j^k \quad \forall j \in P,\ \forall k \in K
$$

(Using integer $x$ allows revisits; the first two are lower bounds ensuring service, the third is an upper bound preventing unauthorized transit. $M$ is a large constant, e.g., $|V|$.)

### 3. Flow conservation on owned nodes

For any drone $k$, net flow is zero at every owned node:

$$
\sum_{i : (i,v)\in A} x_{iv}^k = \sum_{j : (v,j)\in A} x_{vj}^k \quad \forall v \in P,\ \forall k \in K
$$

### 4. Mandatory use of all drones — single out-and-back (no multiple base touches)

Each drone must depart from the base exactly once and return exactly once:

$$
\sum_{j : (S,j)\in A} x_{Sj}^k = 1,\quad
\sum_{i : (i,S)\in A} x_{iS}^k = 1 \quad \forall k \in K
$$

No additional base visits are allowed; thus, $x_{Sj}^k$ and $x_{jS}^k$ outside these single uses must be zero.

### 5. Makespan linking

Total time per drone is the sum over its traversals:

$$
T \ge \sum_{(i,j)\in A} t_{ij}\, x_{ij}^k \quad \forall k \in K
$$

### 6. Each drone must service at least one non-entry node (prevents “entry-only” tours)

$$
\sum_{j \in P \setminus E_{ntry}} y_j^k \ge 1 \quad \forall k \in K
$$

### 7. Base-connectedness via single-commodity flow (preferred over MTZ here)

Supply at base, proportional to owned nodes:

$$
\sum_{j : (S,j)\in A} f_{Sj}^k = \sum_{p \in P} y_p^k \quad \forall k \in K
$$

Flow conservation on owned nodes:

$$
\sum_{i : (i,v)\in A} f_{iv}^k = y_v^k + \sum_{j : (v,j)\in A} f_{vj}^k \quad \forall v \in P,\ \forall k \in K
$$

Capacity linking to traversals:

$$
f_{ij}^k \le |P| \cdot x_{ij}^k \quad \forall (i,j)\in A,\ \forall k \in K
$$

These three together ensure every owned node of drone $k$ is reachable from $S$ in $k$’s route and eliminate disconnected subtours while still allowing revisits.
