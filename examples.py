"""Reusable optimization helpers extracted from lab3 notebooks.

This module collects generic code for:
- Solving the base product mix model with all columns.
- Solving the product mix over a subset of columns (column generation style).
- Computing dual variables (shadow prices) and reduced costs.

The functions are written so they can be reused in scripts or
interactive notebooks.
"""

from typing import Iterable, List, Sequence, Tuple

import numpy as np
import mip
from mip import Model, xsum, MAXIMIZE


# ---------------------------------------------------------------------------
# Basic product-mix solver (small instance, as in part I)
# ---------------------------------------------------------------------------

def solve_productmix(
    A: Sequence[Sequence[float]],
    b: Sequence[float],
    c: Sequence[float],
) -> Tuple[mip.Model, List[float], float]:
    """Solve max{c^T x : A x <= b, x >= 0} for a small dense instance.

    Parameters
    ----------
    A : 2D list-like (m x n)
        Coefficients a_{ip}. Each row i represents an ingredient,
        each column j represents a product (perfume).
    b : 1D list-like (m)
        Right-hand side / availability for each ingredient.
    c : 1D list-like (n)
        Profit (objective coefficient) for each product.

    Returns
    -------
    m : mip.Model
        Solved model instance.
    x_sol : list[float]
        Optimal values for each product variable.
    obj : float
        Optimal objective value.
    """

    n = len(c)
    k = len(b)

    # Create a maximization LP model.
    m = Model(sense=MAXIMIZE)

    # One continuous non-negative variable per product j.
    x = [m.add_var(lb=0.0, name=f"x_{j}") for j in range(n)]

    # One capacity/ingredient constraint per row i: sum_j a_{ij} x_j <= b_i.
    for i in range(k):
        m.add_constr(xsum(A[i][j] * x[j] for j in range(n)) <= b[i], name=f"ing_{i}")

    # Objective: maximize total profit c^T x.
    m.objective = xsum(c[j] * x[j] for j in range(n))

    # Optimize the LP.
    m.optimize()

    x_sol = [x[j].x for j in range(n)]
    return m, x_sol, m.objective_value


# ---------------------------------------------------------------------------
# Product-mix solver over a subset of columns (as in part II)
# ---------------------------------------------------------------------------

def solve_productmix_indices(
    A: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    indices: Iterable[int],
) -> Tuple[mip.Model, List[float], float]:
    """Solve max{c^T x : A x <= b, x >= 0} using only a subset of columns.

    The subset is specified by ``indices`` (integers indexing columns of A
    and entries of c). The function returns a full-length solution vector
    (size = number of columns of A), with zeros for non-selected columns.

    Parameters
    ----------
    A : 2D numpy array (|I| x |P|)
        Coefficient matrix for the LP.
    b : 1D numpy array (|I|)
        Right-hand side / availability vector.
    c : 1D numpy array (|P|)
        Profit vector.
    indices : iterable of int
        Subset of column indices to include as decision variables.

    Returns
    -------
    m : mip.Model
        Solved model.
    sol : list[float]
        Full-length solution for all columns, zero outside ``indices``.
    obj : float
        Optimal objective value.
    """

    # Ensure we have a concrete list of indices (e.g., if a range/generator was passed).
    P = list(indices)

    # Create a maximization model.
    m = Model(sense=MAXIMIZE)

    # Decision variables x_p >= 0 for each selected column p.
    x = {p: m.add_var(lb=0.0, name=f"x_{p}") for p in P}

    # One constraint per row i: sum_{p in P} A[i,p] * x_p <= b[i].
    num_ingredients = A.shape[0]
    for i in range(num_ingredients):
        m.add_constr(
            xsum(A[i, p] * x[p] for p in P) <= float(b[i]),
            name=f"ing_{i}",
        )

    # Objective: maximize sum_{p in P} c_p * x_p.
    m.objective = xsum(c[p] * x[p] for p in P)

    # Solve the LP.
    m.optimize()

    # Build full-length solution (one value per column of A).
    n_perfumes = A.shape[1]
    sol = [0.0] * n_perfumes
    for p in P:
        sol[p] = x[p].x

    return m, sol, m.objective_value


# ---------------------------------------------------------------------------
# Dual variables and reduced costs utilities
# ---------------------------------------------------------------------------

def get_dual_values(model: mip.Model) -> List[float]:
    """Return dual values (shadow prices) for all constraints in a model."""

    return [con.pi for con in model.constrs]


def reduced_cost_from_duals(
    A: np.ndarray,
    c: np.ndarray,
    dual: Sequence[float],
    p: int,
) -> float:
    """Compute reduced cost of column p using dual variables.

    This implements the classical formula

        rc_p = c_p - sum_i A[i, p] * dual_i

    which is valid whether or not column p is currently in the model.
    """

    return float(c[p] - np.dot(A[:, p], np.asarray(dual, dtype=float)))


def compute_reduced_costs_for_all(
    A: np.ndarray,
    c: np.ndarray,
    dual: Sequence[float],
    skip_indices: Iterable[int] | None = None,
) -> List[Tuple[int, float]]:
    """Compute reduced costs for all columns, optionally skipping some.

    Parameters
    ----------
    A, c : numpy arrays
        Problem data.
    dual : sequence of float
        Dual multipliers for each row/constraint.
    skip_indices : iterable of int, optional
        Columns (perfumes) for which to skip computing rc (e.g.,
        the ones already present in the current model). If None,
        all columns are processed.

    Returns
    -------
    list of (p, rc_p)
        Pair with column index and its reduced cost.
    """

    n_cols = A.shape[1]
    skip = set(skip_indices or [])
    rcs: List[Tuple[int, float]] = []

    for p in range(n_cols):
        if p in skip:
            continue
        rc = reduced_cost_from_duals(A, c, dual, p)
        rcs.append((p, rc))

    return rcs


def print_positive_reduced_costs(
    A: np.ndarray,
    c: np.ndarray,
    dual: Sequence[float],
    base_indices: Iterable[int],
) -> None:
    """Helper that prints columns with strictly positive reduced cost.

    Useful to quickly identify which perfumes would improve
    the objective if added to the current restricted model.
    """

    rcs = compute_reduced_costs_for_all(A, c, dual, skip_indices=base_indices)
    for p, rc in rcs:
        msg = f"Reduced cost for {p:2d} = {rc:9.2f}"
        if rc > 0:
            msg += " <======"
        print(msg)
