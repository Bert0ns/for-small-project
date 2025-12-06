# Problem description

A consulting company uses drones to analyze the external surfaces of arbitrary buildings by means of cameras and various types of sensors. The company has been tasked with analyzing a specific building, and the
set of points at which the drone must stop to perform measurements is known. Each point in the given set
is identified by its three spatial coordinates; we can instead neglect the issue of the drone orientation at each
point.
The measurement team for the building has k = 4 drones. At the beginning of the measurement, the
team positions itself at a point px0, y0, z0q and launches all drones simultaneously. Each drone explores a
subset of the points and returns to the starting point. The speeds of each drone are:
* 1 m/s when moving upward;
* 2 m/s when moving downward;
* 1.5 m/s for purely horizontal movements;
* a combination for oblique movements: for a segment with lateral movement of length a and vertical
(upward) movement of length b, the travel time is max(a/1.5, b/1).

We may assume that the drones are either stationary or moving at the speeds listed above; that is, we can
neglect the computation of acceleration and deceleration when moving from one point to another.
We must decide the trajectories of each of the k drones so that every point in the given grid is visited by
exactly one drone. Since time is money, we must find trajectories that minimize the time taken by the last
drone to return to the base point.

Two points A and B of the grid are connected (i.e., a drone can travel from A to B and vice versa) if
and only if:
* the Euclidean distance between A and B is at most 4 m, or
* the Euclidean distance between A and B is at most 11 m and two among the coordinates x, y, and z differ by at most 0.5 m.

This connectivity condition does not apply to the segments between the base point and the “entry” points of the grid, specified below, which are the only points that are accessible to all drones when they depart from
the base and when they return from the grid to the base. 
Each drone may travel along the segments between the base point and any of these entry points (we also neglect the possibility of collisions between drones), regardless of the distance.
Two instances of the problem are given in the attached files Edificio1.csv and Edificio2.csv (CSV format), which contain the spatial coordinates px, y, zq of the points to be visited. The coordinates
(x0, y0, z0) of the starting point are as follows:
* for Edificio1.csv: (0, -16, 0); the entry points are all grid points (x, y, z) with y <=  -12.5;
* for Edificio2.csv: (0, -40, 0); the entry points are the grid points (x, y, z) with y <=  -20;

## Operational instructions for submission
Upload a file named Cognome Nome IDpersona.zip, using your surname, name, and personal ID (not
your student number). This must be a .zip archive containing a folder called Cognome Nome IDpersona
with the following files:
• the two files Edificio1.csv and Edificio2.csv;
• one or more Python files that use the mip module to model and solve the problem; there may be
multiple Python scripts, but the Python script to be executed must be named main.py.
The script main.py must accept one input argument, which must be a file with the same format as
Edificio1.csv and Edificio2.csv.
The person evaluating your project must be able to run it by executing the following two commands
from the terminal:
2

