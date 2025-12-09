# Mathematical Model for Drone Routing Problem (v5 - No Revisits)

## Problem Definition

We need to route $K=4$ drones to visit a set of points $P$ in 3D space, starting and ending at a base station $S$. The goal is to minimize the maximum time any single drone takes to complete its tour (Minimax objective), while ensuring every point in the grid is visited by exactly one drone.

**Key Constraint Change**: In this version, drones are **not allowed to revisit nodes**. Each node is entered exactly once and left exactly once if it is assigned to a drone.

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

- $x_{ij}^k \in \{0, 1\}$: 1 if drone $k$ traverses arc $(i,j)$, 0 otherwise. (Binary variable prevents revisits).
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

### 2. Tour Connectivity & Ownership (Strict No-Revisit)

If drone $k$ owns node $j$ ($y_j^k=1$), it must enter it exactly once and leave it exactly once. If it does not own it, it cannot traverse any arc incident to it.

$$
\sum_{i : (i,j)\in A} x_{ij}^k = y_j^k \quad \forall j \in P,\ \forall k \in K \quad \text{(Incoming = Owned)}
$$

$$
\sum_{i : (j,i)\in A} x_{ji}^k = y_j^k \quad \forall j \in P,\ \forall k \in K \quad \text{(Outgoing = Owned)}
$$

_Note: These two constraints imply flow balance ($\sum_{in} = \sum_{out}$) and prevent any node from being visited more than once._

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

Since all drones are identical, we can impose an ordering to reduce the search space. For example, we can enforce that the number of nodes visited by drone $k$ is greater than or equal to drone $k+1$.

$$
\sum_{j \in P} y_j^k \ge \sum_{j \in P} y_j^{k+1} \quad \forall k \in \{1, \dots, K-1\}
$$

## Summary of Changes for "No Revisits"

1.  **Variables**: $x_{ij}^k$ changed from Integer to Binary $\{0,1\}$.
2.  **Visit Constraints**: Changed from inequality ($\ge y_j^k$) to strict equality ($= y_j^k$). This enforces that every owned node is visited exactly once (1 entry, 1 exit) and non-owned nodes are visited 0 times.
