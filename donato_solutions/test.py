from typing import Final
import sys
from pathlib import Path
from utils import Point3D, read_points_from_csv
from visualizations.plotter import htmpl_plot_generator
from solver import DroneRoutingSolver


# Number of drones
K = 4
# Speed constants (meters per second)
SPEED_UP = 1.0
SPEED_DOWN = 2.0
SPEED_HORIZONTAL = 1.5
# Initial points for each building
BASE_POINT_B1: Final[Point3D] = Point3D(0.0, -16.0, 0.0)
BASE_POINT_B2: Final[Point3D] = Point3D(0.0, -40.0, 0.0)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python ./main.py <csv_file>", file=sys.stderr)
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    if not csv_path.exists() or not csv_path.is_file():
        print(f"Error: file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    try:
        points = read_points_from_csv(str(csv_path))
    except Exception as e:
        print(f"Error reading points from '{csv_path}': {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Loaded {len(points)} points from {csv_path}")

    # Heuristic to pick building thresholds based on filename
    name = csv_path.name.lower()
    if "1" in name or "edificio1" in name or "building1" in name:
        ENTRY_THRESHOLD = -12.5
        base_point = BASE_POINT_B1
    else:
        ENTRY_THRESHOLD = -20.0
        base_point = BASE_POINT_B2
    print(f"Initial point chosen: {base_point}")

    print("Initializing solver...")
    solver = DroneRoutingSolver(
        points,
        base_point,
        ENTRY_THRESHOLD,
        k_drones=K,
        speed_up=SPEED_UP,
        speed_down=SPEED_DOWN,
        speed_horizontal=SPEED_HORIZONTAL,
    )

    points, arcs, costs, entry_points_idx = solver.get_graph()

    print(
        f"Graph has {len(points)} points and {len(arcs)} arcs. number of costs: {len(costs)}"
    )

    htmpl_plot_generator(
        points,
        arcs,
        entry_points_idx,
        str(csv_path.name) + "Before solution",
        output_file="visualizations/graph_visualization_before_solution.html",
    )

    print("Solving routing problem...")
    # paths = solver.solve(max_seconds=300000)
    print("Skipping actual solving for testing purposes.")
    
    paths_dummy_string = """
    Drone 1: 0-544-584-623-622-583-582-581-620-621-618-619-580-579-679-667-617-678-616-615-614-613-612-573-574-575-576-577-578-539-538-537-536-535-534-495-496-497-498-499-500-461-460-459-458-457-456-417-418-419-420-421-422-383-382-381-380-379-378-339-340-341-342-343-344-345-346-347-348-387-386-385-384-423-424-425-426-465-464-463-462-501-502-503-504-540-541-542-543-545-506-505-466-467-428-427-388-389-350-349-351-390-429-468-507-546-585-624-675-670-677-672-682-611-572-533-494-455-416-377-338-337-336-335-334-333-376-375-374-373-372-411-412-413-414-415-454-453-452-451-450-489-490-491-492-493-532-531-530-529-528-567-568-569-570-571-610-609-608-607-606-0
    Drone 2: 0-665-651-646-643-72-71-30-31-647-35-34-33-32-73-74-75-76-117-116-115-114-155-156-157-158-199-198-197-196-195-194-153-112-113-154-240-239-238-237-235-236-316-315-276-277-278-279-280-281-320-319-318-317-1-42-43-84-83-123-164-163-162-161-160-204-203-202-201-159-118-77-200-241-282-321-322-323-324-325-326-286-285-284-283-242-243-244-245-287-247-248-249-250-251-252-253-293-292-291-290-289-288-639-2-36-652-654-41-82-81-80-79-78-119-120-121-122-124-125-166-165-205-246-206-207-208-209-210-211-212-171-170-169-168-167-126-127-128-129-130-89-88-87-86-85-44-45-46-47-48-7-658-3-4-5-6-29-70-111-152-193-234-275-314-37-38-39-40-656-0
    Drone 3: 0-673-683-671-676-626-587-627-629-630-631-632-684-482-443-404-365-555-516-477-438-399-360-359-358-357-356-395-396-397-398-437-436-435-434-473-474-475-476-515-514-513-512-551-552-553-554-593-592-591-590-478-479-480-481-442-441-440-439-400-401-402-403-364-363-362-361-548-509-470-431-392-353-327-328-329-330-331-332-371-370-369-368-367-366-405-406-407-408-409-410-449-448-447-446-445-444-483-484-485-486-487-488-527-526-525-524-523-522-561-562-563-564-565-566-605-604-603-602-601-600-681-638-599-560-521-520-519-517-518-596-595-594-669-633-685-635-634-637-636-597-598-559-558-556-557-352-391-430-469-508-547-586-625-663-666-674-662-680-668-354-355-394-393-432-433-472-471-510-511-550-549-589-588-628-0
    Drone 4: 0-661-660-655-53-52-51-12-11-10-9-50-49-90-91-92-93-94-135-134-133-132-131-172-173-174-175-176-217-216-215-214-213-28-640-300-258-257-256-255-254-294-295-296-297-298-299-313-274-233-192-151-110-69-642-645-650-644-26-25-24-23-22-21-62-63-64-65-66-67-108-107-106-105-104-103-144-145-146-147-148-149-190-189-188-187-186-185-226-227-228-229-230-231-272-271-270-269-268-267-306-307-308-309-310-311-312-273-232-191-150-109-68-27-641-659-664-649-648-20-61-102-143-184-225-266-305-304-303-302-301-262-263-264-265-224-223-222-221-180-181-182-183-142-141-140-139-98-99-100-101-60-59-58-57-16-17-18-19-653-15-14-259-260-261-220-219-218-177-178-179-138-137-136-95-96-97-56-55-54-13-8-657-0
    """

    paths = []
    for line in paths_dummy_string.strip().split('\n'):
        if "Drone" in line:
            try:
                path_str = line.split(':')[1].strip()
                path = [int(node) for node in path_str.split('-')]
                paths.append(path)
            except Exception as e:
                print(f"Error parsing line: {line}. Error: {e}")

    if paths:
        print("Routing problem solved. Generating solution plot...")
        htmpl_plot_generator(
            points,
            arcs,
            entry_points_idx,
            str(csv_path.name) + " - Solution",
            output_file="visualizations/graph_visualization_solution.html",
            paths=paths,
        )
    else:
        print("Could not print solution paths.")
