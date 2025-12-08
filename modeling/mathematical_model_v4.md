# Mathematical Model for Drone Routing Problem

## Problem Definition

We need to route $K=4$ drones to visit a set of points $P$ in 3D space, starting and ending at a base station $S$. The goal is to minimize the maximum time any single drone takes to complete its tour (Minimax objective).

## Sets and Indices

- $V = P \cup \{S\}$: Set of all nodes, where $S$ is the base station (node 0) and $P = \{1, \dots, N\}$ is the set of target points.
- $E_{ntry} \subset P$: Set of entry points accessible from the base.
- $K = \{1, \dots, 4\}$: Set of drones.
- $A$: Set of valid arcs $(i, j)$.
  - For $i, j \in P$, $(i, j) \in A$ if they satisfy the connectivity constraints (distance $\le 4$ or distance $\le 11$ with specific coordinate constraints).
  - For base connections: $(S, j) \in A$ iff $j \in E_{ntry}$, and $(i, S) \in A$ iff $i \in E_{ntry}$.

## Parameters

- $t_{ij}$: Travel time between node $i$ and node $j$.
  - Calculated based on lateral and vertical speeds provided in the problem description.
  - $t_{ij} = \infty$ if $(i, j) \notin A$.

## Decision Variables

- $x_{ijk} \in \{0, 1\}$: Binary variable, equal to 1 if drone $k$ travels from node $i$ to node $j$, 0 otherwise.

- $Z \in \mathbb{R}^{+}$: The maximum time taken by any drone (the variable to minimize).

## Objective Function

Minimize the makespan (time of the longest tour):
$$ \min Z $$

## Constraints

### 1. Visit Every Point Exactly Once. The same drone can revisit the point.

Each point $j \in P$ must be visited by exactly one drone.
$$ \sum_{k \in K} \sum_{i \in V, (i,j) \in A} x{ijk} >= 1 \quad \forall j \in P $$

### 2. Flow Conservation

If a drone $k$ enters a node $j$, it must leave it.
$$ \sum_{i \in V, (i,j) \in A} x_{ijk} = \sum_{m \in V, (j,m) \in A} x_{jmk} \quad \forall j \in P, \forall k \in K $$

### 3. Depot Constraints

Each drone $k$ must leave the base $S$ exactly once and return to the base $S$ exactly once.
$$ \sum_{j \in E_{ntry}} x_{Sjk} = 1 \quad \forall k \in K $$
$$ \sum_{i \in E_{ntry}} x_{iSk} = 1 \quad \forall k \in K $$

### 4. Minimax Time Constraint

The variable $Z$ must be greater than or equal to the total travel time of each drone $k$.
$$ \sum_{(i,j) \in A} t_{ij} x_{ijk} \le Z \quad \forall k \in K $$

### 5. Subtour Elimination

Iterative approach - currently under development


## Variable Domains

$$ x_{ijk} \in \{0, 1\} \quad \forall (i,j) \in A, \forall k \in K $$
$$ Z \ge 0 $$
