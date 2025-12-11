# Mathematical Model for Drone Routing Problem (v8 - Cutting Planes)

## Problem Definition

Route $K=4$ drones to visit a set of points $P$ in 3D space, starting and ending at a base station $S$. 
- **Constraint**: Each point in $P$ must be visited **exactly once** by exactly one drone (No revisits allowed).
- **Objective**: Minimize the makespan $T$ (Minimax).
- **Method**: Subtour elimination is performed dynamically using a **Cutting Plane Algorithm**.

## Sets and Indices

- $V = P \cup \{S\}$: All nodes (0 is Base $S$).
- $K = \{1, \dots, 4\}$: Drones.
- $A$: Set of valid arcs $(i, j)$ based on connectivity rules.

## Decision Variables

- $x_{ij}^k \in \{0, 1\}$: 1 if drone $k$ travels from $i$ to $j$.
- $y_j^k \in \{0, 1\}$: 1 if node $j$ is assigned to drone $k$.
- $z_k \in \{0, 1\}$: 1 if drone $k$ is used.
- $T \ge 0$: Continuous variable for the makespan.

## Objective Function

$$
\min T
$$

## Constraints

### 1. Partitioning (Exact Coverage)
Every target node must be visited exactly once by exactly one drone.
$$\sum_{k \in K} y_j^k = 1 \quad \forall j \in P$$

### 2. Flow Conservation & Degree Constraints
If a drone visits a node, it must enter exactly once and leave exactly once.
$$\sum_{i : (i,j)\in A} x_{ij}^k = y_j^k \quad \forall j \in P, k \in K$$
$$\sum_{m : (j,m)\in A} x_{jm}^k = y_j^k \quad \forall j \in P, k \in K$$

### 3. Base Station Logic
If a drone is active ($z_k=1$), it leaves the base once and returns once.
$$\sum_{j \in E_{ntry}} x_{0j}^k = z_k \quad \forall k \in K$$
$$\sum_{i \in E_{ntry}} x_{i0}^k = z_k \quad \forall k \in K$$

### 4. Makespan Definition
$$
T \ge \sum_{(i,j)\in A} t_{ij} x_{ij}^k \quad \forall k \in K
$$

### 5. Symmetry Breaking
$$
\sum_{j \in P} y_j^k \ge \sum_{j \in P} y_j^{k+1} \quad \forall k \in \{1, .. K-1\}
$$
$$
z_k \ge z_{k+1}
$$

### 6. Subtour Elimination (Dynamic Cutting Planes)
Instead of adding $O(N^2)$ static constraints, we relax the problem initially. During the Branch-and-Cut process, we check for integer solutions that contain disconnected cycles (subtours).

For any identified subtour subset $S \subseteq P$ assigned to drone $k$ that does not connect to the base:
$$\sum_{i \in S, j \in S} x_{ij}^k \le |S| - 1$$
This cut is added "lazily" only when violated.