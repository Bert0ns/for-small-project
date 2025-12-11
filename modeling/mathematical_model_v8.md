# Mathematical Model: Min-Max m-TSP for Drone Routing (Improved)

## 1. Problem Definition

The problem is modeled as a **Min-Max Multiple Traveling Salesperson Problem (m-TSP)**. We aim to determine the routes for $K=4$ drones such that every target point is visited exactly once, constraints on connectivity are satisfied, and the time taken by the longest route (make-span) is minimized.

## 2. Sets and Indices

* $V = \{0, 1, \dots, N\}$: Set of all nodes.
  * Node $0$: The Base station (depot).
  * $V' = V \setminus \{0\} = \{1, \dots, N\}$: Set of target points to be analyzed.
* $K = \{1, \dots, 4\}$: Set of drones.
* $A$: Set of valid arcs $(i, j)$ pre-calculated based on connectivity rules.

## 3. Parameters

* $t_{ij}$: Time required (cost) to travel from node $i$ to node $j$ (calculated via definitions in the project spec).

## 4. Decision Variables

* $x_{ij}^k \in \{0, 1\}$: Binary variable. Equal to 1 if drone $k$ travels from node $i$ to node $j$ via arc $(i,j) \in A$; 0 otherwise.
* $Z \in \mathbb{R}_{\ge 0}$: Continuous variable representing the maximum time taken by any drone.

## 5. Objective Function

Minimize the maximum time:

$$ \min Z $$

## 6. Constraints

### 6.1 Assignment Constraint

Each target point $j \in V'$ must be visited exactly once by exactly one drone:

$$ \sum_{k \in K} \sum_{i \in V, (i,j) \in A} x_{ij}^k = 1 \quad \forall j \in V' $$

### 6.2 Flow Conservation

If a drone $k$ enters a target node $j$, it must leave it:

$$ \sum_{i \in V, (i,j) \in A} x_{ij}^k = \sum_{l \in V, (j,l) \in A} x_{jl}^k \quad \forall j \in V', \forall k \in K $$

### 6.3 Depot Constraints

Each drone must leave the base exactly once and return to the base exactly once:

$$ \sum_{j \in V', (0,j) \in A} x_{0j}^k = 1 \quad \forall k \in K $$
$$ \sum_{i \in V', (i,0) \in A} x_{i0}^k = 1 \quad \forall k \in K $$

### 6.4 Min-Max Time Bound

The total travel time for each drone $k$ cannot exceed $Z$:

$$ \sum_{(i,j) \in A} t_{ij} \cdot x_{ij}^k \le Z \quad \forall k \in K $$

### 6.5 Valid Inequality: Lower Bound on Z

The maximum time must be at least the average time of all drones. This strengthens the LP relaxation significantly:

$$ Z \ge \frac{1}{|K|} \sum_{k \in K} \sum_{(i,j) \in A} t_{ij} \cdot x_{ij}^k $$

### 6.6 Symmetry Breaking (Start Node Ordering)

To remove equivalent permutations of drones, we force the index of the first target visited by drone $k$ to be strictly less than that of drone $k+1$. This assumes $V'$ indices are integers $\{1, \dots, N\}$.

$$ \sum_{j \in V'} j \cdot x_{0j}^k \le \sum_{j \in V'} j \cdot x_{0j}^{k+1} - 1 \quad \forall k \in \{1, \dots, |K|-1\} $$

### 6.7 Dynamic Subtour Elimination (Lazy Constraints)

*Note: This replaces the static MTZ formulation.*

We initially solve the problem without subtour elimination constraints. Whenever the solver finds an integer solution containing isolated subtours (cycles not including node 0), we add the following constraint specifically for the set of nodes $S$ involved in that subtour:

$$ \sum_{i \in S} \sum_{j \in S, i \neq j} \sum_{k \in K} x_{ij}^k \le |S| - 1 $$

Alternatively, the **Cutset** form is often stronger and preferred if supported by the separation routine:

$$ \sum_{i \in S} \sum_{j \notin S} \sum_{k \in K} x_{ij}^k \ge 1 $$

(For any proper subset $S \subset V'$, at least one drone must exit $S$).