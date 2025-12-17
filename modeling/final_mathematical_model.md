# Mathematical Model for Drone Routing Problem (Final)

## Problem Definition

We need to route exactly $K=4$ drones to visit a set of points $P$ in 3D space, starting and ending at a base station $S$. The goal is to minimize the maximum time any single drone takes to complete its tour (Minimax objective), while ensuring every point in the grid is visited by exactly one drone.

**Key Constraints**:

- **All 4 drones must be used.**
- **No Revisits**: Each node is entered exactly once and left exactly once if it is assigned to a drone.
- **Exclusive Ownership**: Each node is assigned to exactly one drone.

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

## Decision Variables

- $x_{ij}^k \in \{0, 1\}$: 1 if drone $k$ traverses arc $(i,j)$, 0 otherwise.
- $y_j^k \in \{0,1\}$: 1 if grid node $j$ is assigned to (visited by) drone $k$.
- $f_{ij}^k \ge 0$: flow variable for drone $k$ on arc $(i,j)$ to ensure connectivity.
- $T \ge 0$: makespan (time of the slowest drone).

## Objective Function

Minimize the makespan (pure minimax):

$$
\min T
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

### 3. Base Station Constraints (Mandatory Usage)

Each drone must depart from the base exactly once and return exactly once.

$$
\sum_{j \in E_{ntry}} x_{Sj}^k = 1 \quad \forall k \in K
$$

$$
\sum_{i \in E_{ntry}} x_{iS}^k = 1 \quad \forall k \in K
$$

### 4. Minimum Workload

Each drone must visit at least one target node (to prevent trivial 0-0 loops if not strictly required by base constraints, though base constraints + connectivity usually enforce this if $S$ is not connected to itself).

$$
\sum_{j \in P} y_j^k \ge 1 \quad \forall k \in K
$$

### 5. Subtour Elimination (Single-Commodity Flow)

To ensure that all visited nodes are connected to the base $S$, we use flow variables. The base sends $|P_k|$ units of flow to the nodes assigned to drone $k$.

$$
\sum_{j \in E_{ntry}} f_{Sj}^k = \sum_{p \in P} y_p^k \quad \forall k \in K \quad \text{(Source flow = count of owned nodes)}
$$

$$
\sum_{i : (i,v)\in A} f_{iv}^k - \sum_{j : (v,j)\in A} f_{vj}^k = y_v^k \quad \forall v \in P,\ \forall k \in K \quad \text{(Flow consumption)}
$$

$$
0 \le f_{ij}^k \le |P| \cdot x_{ij}^k \quad \forall (i,j) \in A,\ \forall k \in K \quad \text{(Flow capacity)}
$$

### 6. Makespan Definition

The variable $T$ must be greater than or equal to the total travel time of any drone.

$$
T \ge \sum_{(i,j) \in A} t_{ij} x_{ij}^k \quad \forall k \in K
$$

### 6.5 Valid Inequality (Lower Bound on Makespan)

Since every node $j \in P$ must be visited by some drone, the makespan must be at least the time required to travel from the base to node $j$ and return to the base.

$$
T \ge \text{ShortestPath}(S, j) + \text{ShortestPath}(j, S) \quad \forall j \in P
$$

### 7. Symmetry Breaking

To reduce the search space, we enforce an ordering on the size of the tours (number of visited nodes).

$$
\sum_{j \in P} y_j^k \ge \sum_{j \in P} y_j^{k+1} \quad \forall k \in \{1, \dots, K-1\}
$$
