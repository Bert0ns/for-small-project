# Mathematical Model for Drone Routing Problem (v7 - With Revisits)

## Problem Definition

We need to route up to $K=4$ drones to visit a set of points $P$ in 3D space, starting and ending at a base station $S$. The goal is to minimize the maximum time any single _used_ drone takes to complete its tour (Minimax objective), while ensuring every point in the grid is visited by exactly one drone.

**Key Constraint Change**: In this version, drones are **allowed to revisit nodes**. A drone must visit all its assigned nodes at least once, but it may pass through them multiple times (revisits) to reach other nodes. However, a drone can only visit nodes that are assigned to it (Strict Ownership).

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
- $M$: A sufficiently large number (e.g., $|P|$).

## Decision Variables

- $x_{ij}^k \in \mathbb{Z}_{\ge 0}$: Number of times drone $k$ traverses arc $(i,j)$. (Integer variable allows revisits).
- $y_j^k \in \{0,1\}$: 1 if grid node $j$ is assigned to (visited by) drone $k$.
- $z_k \in \{0,1\}$: 1 if drone $k$ is used; 0 if it stays at the base.
- $f_{ij}^k \ge 0$: flow variable for drone $k$ on arc $(i,j)$ to ensure connectivity.
- $T \ge 0$: makespan (time of the slowest used drone).

## Objective Function

Minimize the makespan (pure minimax):

$$
\min T
$$

## Constraints

### 1. Partitioning (Exclusive Assignment)

Each grid node must be assigned to exactly one drone.

$$
\sum_{k \in K} y_j^k = 1 \quad \forall j \in P
$$

### 2. Tour Connectivity & Ownership (Revisits Allowed)

We replace the strict "Incoming = Owned" equality with flow balance and service constraints.

**Flow Balance**:
For every node $j \in P$ and drone $k$, the number of times the drone enters must equal the number of times it leaves.

$$
\sum_{i : (i,j)\in A} x_{ij}^k = \sum_{m : (j,m)\in A} x_{jm}^k \quad \forall j \in P,\ \forall k \in K
$$

**Service Requirement & Exclusivity**:
If drone $k$ owns node $j$ ($y_j^k=1$), it must enter it at least once. If it does not own it ($y_j^k=0$), it cannot enter it (it cannot traverse through nodes owned by others).

$$
y_j^k \le \sum_{i : (i,j)\in A} x_{ij}^k \le M \cdot y_j^k \quad \forall j \in P,\ \forall k \in K
$$

_Note: This allows $\sum x_{ij}^k > 1$, enabling revisits, but forces it to 0 if $y_j^k=0$._

### 3. Base Station Constraints

If a drone is used ($z_k=1$) it must depart from the base exactly once and return exactly once.

$$
\sum_{j \in E_{ntry}} x_{Sj}^k = z_k \quad \forall k \in K
$$

$$
\sum_{i \in E_{ntry}} x_{iS}^k = z_k \quad \forall k \in K
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
0 \le f_{ij}^k \le |P| \cdot x_{ij}^k \quad \forall (i,j) \in A,\ \forall k \in K \quad \text{(Flow capacity)}
$$

### 5. Makespan Definition

The variable $T$ must be greater than or equal to the total travel time of any drone.

$$
T \ge \sum_{(i,j)\in A} t_{ij} x_{ij}^k \quad \forall k \in K
$$

### 6. Symmetry Breaking

Since all drones are identical, we can impose an ordering to reduce the search space.

$$
\sum_{j \in P} y_j^k \ge \sum_{j \in P} y_j^{k+1} \quad \forall k \in \{1, \dots, K-1\}
$$

$$
z_k \ge z_{k+1} \quad \forall k \in \{1, \dots, K-1\}
$$

### 7. Activation Linking

Tie ownership to activation so unused drones stay idle and used drones own at least one node:

$$
\sum_{j \in P} y_j^k \le |P| \cdot z_k \quad \forall k \in K
$$

$$
\sum_{j \in P} y_j^k \ge z_k \quad \forall k \in K
$$

## Summary of Changes for "With Revisits" (v7)

1.  **Variables**: $x_{ij}^k$ changed from Binary $\{0,1\}$ to Integer $\mathbb{Z}_{\ge 0}$ to allow multiple traversals of the same arc (or just multiple entries to a node).
2.  **Visit Constraints**:
    - Replaced strict equality ($\sum x = y$) with two constraints:
      - Flow Balance: $\sum_{in} x = \sum_{out} x$.
      - Service/Exclusivity: $y \le \sum_{in} x \le M \cdot y$.
    - This allows a drone to enter/leave a node multiple times if it owns it, but 0 times if it doesn't.
3.  **Objective**: Remains Minimax.
