p1 = [-3.9929, -9.144, 79.3699]

p2 = [-11.8872, -7.2542, 79.3699]

p3 = [-7.9858, 0, 79.3699]

p4 = [-7.9858,-18.288,79.3699]


def isConnected(p1, p2):
    distance = (
        (p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2
    ) ** 0.5
    print(f"Distance between p1 and p2: {distance}")

    diffs = [abs(p1[0] - p2[0]), abs(p1[1] - p2[1]), abs(p1[2] - p2[2])]
    small_diffs = sum(1 for d in diffs if d <= 0.5)
    if distance <= 11.0 and small_diffs >= 2:
        print("Points are connected")
    elif distance <= 4.0:
        print("Points are connected")
    else:
        print("Points are not connected")
        

# Test connections between all points
isConnected(p1, p2)
isConnected(p1, p3)
isConnected(p1, p4)

isConnected(p2, p3)
isConnected(p2, p4)

isConnected(p3, p4)
