# Mathematical Model for Drone Routing Problem (v5)

## Problem Definition

We need to route $K=4$ drones to visit a set of points $P$ in 3D space, starting and ending at a base station $S$. The goal is to minimize the maximum time any single drone takes to complete its tour (Minimax objective), while ensuring every point in the grid is visited by exactly one drone.

## Sets and Indices

- $V = P \cup \{S\}$: Set of all nodes, where $S$ is the base station (node 0) and $P = \{1, \dots, N\}$ is the set of target points.
- $E_{ntry} \subset P$: Set of entry points accessible from the base.
- $K = \{1, \dots, 4\}$: Set of drones.
- $A$: Set of valid arcs $(i, j)$.
  - For $i, j \in P$, $(i, j) \in A$ if they satisfy the connectivity constraints:
    - Euclidean distance $\le 4$, OR
    - Euclidean distance $\le 11$ AND at least two coordinates differ by at most 0.5.
  - For base connections: $(S, j) \in A$ iff $j \in E_{ntry}$, and $(i, S) \in A$ iff $i \in E_{ntry}$.

## Parameters

- Coordinates $(x_i,y_i,z_i)$ for all $i \in V$.
- Lateral distance $a_{ij} = \sqrt{(x_i-x_j)^2 + (y_i-y_j)^2}$.
- Vertical difference $\Delta z_{ij} = z_j - z_i$.
- Travel time $t_{ij}$ (asymmetric):
  - If $\Delta z_{ij} > 0$ (upward): $t_{ij} = \max\!\big(a_{ij}/1.5,\ \Delta z_{ij}/1\big)$.
  - If $\Delta z_{ij} < 0$ (downward): $t_{ij} = \max\!\big(a_{ij}/1.5,\ |\Delta z_{ij}|/2\big)$.
  - If $\Delta z_{ij} = 0$ (horizontal): $t_{ij} = a_{ij}/1.5$.
- $\epsilon$: A small constant (e.g., $0.001$) to penalize total travel time for secondary optimization.

## Decision Variables

- $x_{ij}^k \in \mathbb{Z}_{\ge 0}$: number of times drone $k$ traverses arc $(i,j)$. (Integer variable allows revisits).
- $y_j^k \in \{0,1\}$: 1 if grid node $j$ is assigned to (visited by) drone $k$.
- $f_{ij}^k \ge 0$: flow variable for drone $k$ on arc $(i,j)$ to ensure connectivity.
- $T \ge 0$: makespan (time of the slowest drone).

## Objective Function

Minimize the makespan, with a small penalty on total time to ensure efficient routing for all drones and break ties:

$$
\min \left( T + \epsilon \sum_{k \in K} \sum_{(i,j) \in A} t_{ij} x_{ij}^k \right)
$$

## Constraints

### 1. Partitioning (Exclusive Assignment)

Each grid node must be visited by exactly one drone.

$$
\sum_{k \in K} y_j^k = 1 \quad \forall j \in P
$$

### 2. Tour Connectivity & Ownership

If drone $k$ owns node $j$ ($y_j^k=1$), it must enter and leave it. If it does not own it, it cannot traverse any arc incident to it.

$$
\sum_{i : (i,j)\in A} x_{ij}^k = \sum_{i : (j,i)\in A} x_{ji}^k \quad \forall j \in P,\ \forall k \in K \quad \text{(Flow balance)}
$$

$$
\sum_{i : (i,j)\in A} x_{ij}^k \ge y_j^k \quad \forall j \in P,\ \forall k \in K \quad \text{(Must visit if owned)}
$$

$$
\sum_{i : (i,j)\in A} x_{ij}^k \le M \cdot y_j^k \quad \forall j \in P,\ \forall k \in K \quad \text{(Cannot visit if not owned)}
$$

_Note: $M$ is a sufficiently large constant (e.g., $|P|$)._

### 3. Base Station Constraints

Each drone must depart from the base exactly once and return exactly once.

$$
\sum_{j \in E_{ntry}} x_{Sj}^k = 1 \quad \forall k \in K
$$

$$
\sum_{i \in E_{ntry}} x_{iS}^k = 1 \quad \forall k \in K
$$

### 4. Subtour Elimination (Single-Commodity Flow)

To ensure that all visited nodes are connected to the base $S$, we use flow variables. The base sends $|P_k|$ units of flow to the nodes assigned to drone $k$.

$$
\sum_{j \in E_{ntry}} f_{Sj}^k = \sum_{p \in P} y_p^k \quad \forall k \in K \quad \text{(Source flow = count of owned nodes)}
$$

$$
\sum_{i : (i,v)\in A} f_{iv}^k - \sum_{j : (v,j)\in A} f_{vj}^k = y_v^k \quad \forall v \in P,\ \forall k \in K \quad \text{(Flow consumption)}
$$

$$
0 \le f_{ij}^k \le (|P| - 1) \cdot x_{ij}^k \quad \forall (i,j) \in A,\ \forall k \in K \quad \text{(Flow capacity)}
$$

### 5. Makespan Definition

The variable $T$ must be greater than or equal to the total travel time of any drone.

$$
T \ge \sum_{(i,j)\in A} t_{ij} x_{ij}^k \quad \forall k \in K
$$

### 6. Symmetry Breaking (Optional but Recommended)

Since all drones are identical, we can impose an ordering to reduce the search space. For example, we can enforce that the number of nodes visited by drone $k$ is greater than or equal to drone $k+1$, or order them by the index of their first entry point.

$$
\sum_{j \in P} y_j^k \ge \sum_{j \in P} y_j^{k+1} \quad \forall k \in \{1, \dots, K-1\}
$$

_Alternatively, order by the ID of the first visited node:_

$$
\sum_{j \in E_{ntry}} j \cdot x_{Sj}^k \le \sum_{j \in E_{ntry}} j \cdot x_{Sj}^{k+1} \quad \forall k \in \{1, \dots, K-1\}
$$

_(Note: This specific symmetry constraint requires careful implementation if drones can share entry points, but since they partition the graph, it's usually valid)._

## Summary of Improvements over v4

1.  **Objective Function**: Added $\epsilon$ term to minimize total time as a secondary objective. This helps the solver find tighter tours and avoid "lazy" loops for non-bottleneck drones.
2.  **Constraint Relaxation**: Removed the constraint "Each drone must service at least one non-entry node". The specification does not strictly require this; a valid optimal solution might have a drone servicing only entry points.
3.  **Symmetry Breaking**: Added constraints to handle the interchangeability of the 4 drones, which significantly improves solver performance.
4.  **Flow Balance**: Explicitly stated flow balance for $x$ variables ($\sum x_{in} = \sum x_{out}$) which was implied but split in v4.
5.  **Flow Formulation**: Refined the Single-Commodity Flow constraints to be standard "consumption" based ($In - Out = Demand$), which is cleaner.
