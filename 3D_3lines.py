import math
import matplotlib
matplotlib.use("Agg")  # pas de fenêtre, rendu fichier
import matplotlib.pyplot as plt

L1, L2, L3 = 10, 5, 3
RX, RY = 6, -4

# forward returns the position of the elbow the hand and the finger
def forward(a1, a2, a3):
    elbow = (L1 * math.cos(a1), L1 * math.sin(a1)) # elbow = (x, y) du coude
    hand = (elbow[0] + L2 * math.cos(a1 + a2), elbow[1] + L2 * math.sin(a1 + a2)) # hand = (x, y) de la pince
    finger = (hand[0] + L3 * math.cos(a1 + a2 + a3), hand[1] + L3 * math.sin(a1 + a2 + a3))
    return elbow, hand, finger

# is_p3_valid is a boolean function which returns False if p3 is unreachable with L1, L2 or True if it is reachable
def is_p3_valid(L7, L8): # ✅
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
    a4, a5 = right_angled_triangle_angles_using_tangent(L7, L8)
    elbow = a1 + a4
    hand = -(180 - a2)
    finger = delta - elbow - hand
    return elbow, hand, finger

# Displays a red dot to reach on my file
def red_dot():
    plt.scatter(RX, RY, color="red", s=20, zorder=5)  # (x, y), couleur, taille (s)

# Calculates the hypotenuse of a right angled triangle
def hypotenuse(adj, opp):
    return math.sqrt(adj**2 + opp**2)

# right_angled_triangle_angles_using_tangent returns the angles of a right angled triangle using TOA (tangent = opposide / adjacent)
def right_angled_triangle_angles_using_tangent(opp, adj):
    a1 = math.degrees(math.atan2(adj, opp))
    a2 = math.degrees(math.atan2(opp, adj))
    return a1, a2

# law_of_cosines is used to get all of the angle of my triangle only using the lenghts of L1, L2 and L3
def law_of_cosines(a, b, c): # ✅
    a1 = math.degrees(math.acos((b**2 - a**2 - c**2)/(-2 * a * c)))
    a3 = math.degrees(math.acos((a**2 - b**2 - c**2)/(-2 * b * c)))
    a2 = 180 - a1 - a3
    return [a1, a2, a3]

# inverse_kinematics returns the position X Y of P3 by assuming that the angle of P3 is delta
def inverse_kinematics(delta):
    x = RX - (L3 * math.cos(math.radians(delta)))
    y = RY - (L3 * math.sin(math.radians(delta)))
    return [x, y]

# is_red_dot_valid is a boolean function which returns False if the red dot is unreachable with L1, L2 and L3 or True if it is reachable
def is_red_dot_valid():
    hyp = hypotenuse(RX, RY)
    smallest = sum(sorted([L1, L2, L3])[:2])
    maximum = max(L1, L2, L3)
    total = L1 + L2 + L3
    if (hyp > total or hyp < max(maximum - smallest, 0)):
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
        elbow, hand, finger = result
        if (abs(hand - finger) < best_gap):
            best_delta = i/4
            best_gap = abs(hand - finger)
    return best_delta

# draw creates the board and displays the red dot, the grid and my lines L1, L2 and L3
def draw(a1, a2, a3, name):
    elbow, hand, finger = forward(a1, a2, a3)
    fig = plt.figure() # Creates an empty white paper
    ax = fig.add_subplot(projection="3d")
    red_dot()
    plt.plot([0, elbow[0], hand[0], finger[0]], [0, elbow[1], hand[1], finger[1]], [0, 1, 2, 3], "o-", markersize=2) # trace the ligne. plot taked the list of all the x coordonates, then the list of all the y coordonates
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
elbow, hand, finger = result
draw(math.radians(elbow), math.radians(hand), math.radians(finger), "/media/share/epitech/robotics/arm4.png")
