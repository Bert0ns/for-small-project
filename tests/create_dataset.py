import csv
import sys
from pathlib import Path

# Add parent directory to path to allow importing modules from there
sys.path.append(str(Path(__file__).resolve().parent.parent))

from utils import Point3D, read_points_from_csv
from visualizations.plotter import html_plot_generator
from solver import DroneRoutingSolver


def create_building_csv(filename):
    points = []
    step = 6  # Passo aumentato a 5 (distanza < 6 rispettata)

    # --- CONFIGURAZIONE GEOMETRIA ---

    # 1. Grattacielo (Base centrale)
    # Larghezza 20 (-10 a 10), Altezza 60
    sky_x_limit = 8
    sky_y_limit = 8
    sky_height = 50

    # 2. Punta (Semisfera sopra il grattacielo)
    tip_center_z = 50
    tip_radius = 8

    # 3. Sfere Laterali
    # Raggio 15, posizionate a +/- 40 sull'asse X
    sphere_radius = 13
    sphere_z_center = 9
    sphere_left_center = (-21, 0, sphere_z_center)
    sphere_right_center = (21, 0, sphere_z_center)

    # Limiti di iterazione per coprire l'intera scena
    for x in range(-60, 61, step):
        for y in range(-20, 21, step):
            for z in range(0, 76, step):

                add_point = False

                # A. Grattacielo
                if (
                    (-sky_x_limit <= x <= sky_x_limit)
                    and (-sky_y_limit <= y <= sky_y_limit)
                    and (0 <= z <= sky_height)
                ):
                    add_point = True

                # B. Punta
                elif z > sky_height:
                    dist_sq = x**2 + y**2 + (z - tip_center_z) ** 2
                    if dist_sq <= tip_radius**2:
                        add_point = True

                # C. Sfera Sinistra
                if not add_point:
                    dist_sq = (
                        (x - sphere_left_center[0]) ** 2
                        + (y - sphere_left_center[1]) ** 2
                        + (z - sphere_left_center[2]) ** 2
                    )
                    if dist_sq <= sphere_radius**2:
                        add_point = True

                # D. Sfera Destra
                if not add_point:
                    dist_sq = (
                        (x - sphere_right_center[0]) ** 2
                        + (y - sphere_right_center[1]) ** 2
                        + (z - sphere_right_center[2]) ** 2
                    )
                    if dist_sq <= sphere_radius**2:
                        add_point = True

                if add_point:
                    points.append([x, y, z])

    # Scrittura su file
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z"])
        writer.writerows(points)

    return len(points)


# Eseguiamo la generazione
n = create_building_csv("Building5.csv")
print(f"Generato file con {n} punti.")

# Generiamo anche la visualizzazione HTML

points = read_points_from_csv("Building5.csv", skip_header=True, delimiter=",")

points = [Point3D(p.x, p.y, p.z) for p in points]

s = DroneRoutingSolver(points, Point3D(0.0, 0.0, 0.0), 0.0, verbose=False)

html_plot_generator(
    points=s.points,
    arcs=s.arcs,
    entry_points_idx=s.entry_points_idx,
    name="Building5.html",
    output_file="Building5.html",
)
