import math
import matplotlib
matplotlib.use("Agg")  # pas de fenêtre, rendu fichier
import matplotlib.pyplot as plt

L1, L2, L3 = 10, 5, 3
RX, RY, RZ = 9.5, -4.8, 7

# forward returns the position of the shoulder, the elbow and the hand
def forward(a1, a2, a3, base_angle):
    shoulder_pos = (L1 * math.cos(a1), L1 * math.sin(a1))
    elbow_pos = (shoulder_pos[0] + L2 * math.cos(a1 + a2),
                 shoulder_pos[1] + L2 * math.sin(a1 + a2))
    hand_pos = (elbow_pos[0] + L3 * math.cos(a1 + a2 + a3),
                elbow_pos[1] + L3 * math.sin(a1 + a2 + a3))
    return (to_3d(shoulder_pos[0], shoulder_pos[1], base_angle),
            to_3d(elbow_pos[0], elbow_pos[1], base_angle),
            to_3d(hand_pos[0], hand_pos[1], base_angle))

# to_3d returns the position x, y and y of any point
def to_3d(dist, height, base_angle):
    x = (dist * math.cos(base_angle))
    y = (dist * math.sin(base_angle))
    z = height
    return x, y, z

# is_p3_valid is a boolean function which returns False if p3 is unreachable with L1, L2 or True if it is reachable
def is_p3_valid(L7, L8):
    hyp = hypotenuse(L7, L8)
    smallest = abs(L1 - L2)
    maximum = L1 + L2
    if (hyp > maximum + 1e-9 or hyp < smallest - 1e-9):
        return False
    return True

# solve calculates the 3 angles of the arm
def solve(delta):
    L7, L8 = inverse_kinematics(delta) # x, y of P3
    L9 = hypotenuse(L7, L8)
    if not (is_p3_valid(L7, L8)): #checks if the red dot is reachable or not
        return None

    a1, a2, a3 = law_of_cosines(L1, L2, L9)
    a4  = right_angled_triangle_angles_using_tangent(L7, L8)
    shoulder = a1 + a4
    elbow = -(180 - a2)
    hand = delta - shoulder - elbow
    return shoulder, elbow, hand

# Calculates the hypotenuse of a right angled triangle
def hypotenuse(adj, opp):
    return math.sqrt(adj**2 + opp**2)

# right_angled_triangle_angles_using_tangent returns the angles of a right angled triangle using TOA (tangent = opposide / adjacent)
def right_angled_triangle_angles_using_tangent(x, y):
    angle = math.degrees(math.atan2(y, x))
    return angle

# law_of_cosines is used to get all of the angle of my triangle only using the lenghts of L1, L2 and L3
def law_of_cosines(a, b, c):
    a1 = math.degrees(math.acos((b**2 - a**2 - c**2)/(-2 * a * c)))
    a3 = math.degrees(math.acos((a**2 - b**2 - c**2)/(-2 * b * c)))
    a2 = 180 - a1 - a3
    return [a1, a2, a3]

# inverse_kinematics returns the position X Y of P3 by assuming that the angle of P3 is delta
def inverse_kinematics(delta):
    horizontal_dist = hypotenuse(RX, RY)
    x = horizontal_dist - (L3 * math.cos(math.radians(delta)))
    y = RZ - (L3 * math.sin(math.radians(delta)))
    return [x, y]

# is_red_dot_valid is a boolean function which returns False if the red dot is unreachable with L1, L2 and L3 or True if it is reachable
def is_red_dot_valid():
    # horizontal_dist = Distance between 0, 0, 0 and RX,RY,0
    # Dist = Distance between 0, 0, 0 and RX, RY, RZ
    horizontal_dist = hypotenuse(RX, RY)
    dist = hypotenuse(horizontal_dist, RZ)
    smallest = sum(sorted([L1, L2, L3])[:2])
    maximum = max(L1, L2, L3)
    total = L1 + L2 + L3
    if (dist > total or dist < max(maximum - smallest, 0)):
        return False
    return True

# calculates delta (best angle possible)
def find_best_delta():
    best_delta = None
    best_gap = float("inf") # infinity
    for i in range(-720, 721):
        result = solve(i/4)
        if (result == None):
            continue
        shoulder, elbow, hand = result
        if (abs(elbow - hand) < best_gap):
            best_delta = i/4
            best_gap = abs(elbow - hand)
    return best_delta

# draw creates the board and displays the red dot, the grid and my lines L1, L2 and L3
def draw(a1, a2, a3, name):
    base_angle = math.radians(right_angled_triangle_angles_using_tangent(RX, RY)) # axe of rotation
    shoulder_pos, elbow_pos, hand_pos = forward(a1, a2, a3, base_angle)
    fig = plt.figure() # Creates an empty white paper
    ax = fig.add_subplot(projection = "3d")
    ax.scatter(RX, RY, RZ, color="red", s=20, zorder=5) # Displays a red dot to reach on my file
    plt.plot([0, shoulder_pos[0], elbow_pos[0], hand_pos[0]], [0, shoulder_pos[1], elbow_pos[1], hand_pos[1]], [0, shoulder_pos[2], elbow_pos[2], hand_pos[2]], "o-", markersize = 4) # trace the ligne. plot taked the list of all the x coordonates, then the list of all the y coordonates
    ax.set_xlim(-20, 20) # fixe la zone visible, de -20 à 20 sur l'axe x
    ax.set_ylim(-20, 20) # fixe la zone visible, de -20 à 20 sur l'axe y
    ax.set_zlim(-20, 20) # fixe la zone visible, de -20 à 20 sur l'axe z
    plt.gca().set_aspect("equal") # gca() means “get current axes”
    plt.grid(True) # displays the grid in order to read the coordinates
    plt.savefig(name) # Saves this into $name
    plt.close() # Closes the figure and free memory

if (is_red_dot_valid() == False): #checks if the red dot is reachable or not
    print("The red dot is out of range")
    exit(1)
if (RX < 0):
    print("Target must be in front of the base")
    exit(1)
best_delta = find_best_delta()
if best_delta is None:
    print("No angle reaches the target")
    exit(1)
result = solve(best_delta)
if result is None:
    print("Target unreachable")
    exit(1)
shoulder, elbow, hand = result
print(shoulder, elbow, hand)
draw(math.radians(shoulder), math.radians(elbow), math.radians(hand), "/media/share/epitech/robotics/arms/arm3.png")
