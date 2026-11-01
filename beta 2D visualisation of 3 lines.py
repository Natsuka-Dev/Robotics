import math
import matplotlib
matplotlib.use("Agg")  # pas de fenêtre, rendu fichier
import matplotlib.pyplot as plt

L1, L2, L3 = 10, 5, 3
RX, RY = 12.4, 4.4

# forward returns the position of the elbow the hand and the finger
def forward(a1, a2, a3):
    elbow = (L1 * math.cos(a1), L1 * math.sin(a1)) # elbow = (x, y) du coude
    hand = (elbow[0] + L2 * math.cos(a1 + a2), elbow[1] + L2 * math.sin(a1 + a2)) # hand = (x, y) de la pince
    finger = (hand[0] + L3 * math.cos(a1 + a2 + a3), hand[1] + L3 * math.sin(a1 + a2 + a3))
    return elbow, hand, finger

# best_angle allows me to find the right angle for both P2 and P3 so that P2 - P3 == 0
def third_angle(): # incorrect for now
    min_angle = -180
    max_angle = 0
    P3X,P3Y = inverse_kinematics(min_angle)
    while (min_angle <= 180 and (hypotenuse(P3X, P3Y) > L1 + L2 or hypotenuse(P3X, P3Y) < abs(L1 - L2))):
        min_angle += 0.5
        P3X,P3Y = inverse_kinematics(min_angle)
    P3X,P3Y = inverse_kinematics(max_angle)
    while (max_angle >= -180 and (hypotenuse(P3X, P3Y) > L1 + L2 or hypotenuse(P3X, P3Y) < abs(L1 - L2))):
        max_angle -= 0.5
        P3X,P3Y = inverse_kinematics(max_angle)
    return (min_angle + max_angle) / 2

# is_p3_valid is a boolean function which returns False if p3 is unreachable with L1, L2 or True if it is reachable
def is_p3_valid(L7, L8):
    hyp = hypotenuse(L7, L8)
    smallest = abs(L1 - L2)
    maximum = L1 + L2
    if (hyp > maximum + 1e-9 or hyp < smallest - + 1e-9):
        return False
    return True

# solve calculates the 3 angles of the arm
def solve(delta):    # solve(-45) should give around (55.9°, −76.6°, −24.3°)
    L7, L8 = inverse_kinematics(delta) # x, y of P3
    L9 = hypotenuse(L7, L8)
    if not (is_p3_valid(L7, L8)): #checks if the red dot is reachable or not
        print("The red dot is out of range.\n")
        return None

    a1, a2, a3 = law_of_cosines(L7, L8, L9)
    print(law_of_cosines(L7, L8, L9))
    a4, a5 = right_angled_triangle_angles_using_tangent(L7, L8)
    elbow = a1 + a4
    hand = a1 - a2
    finger = delta - a1 - a2
    return elbow, hand, finger

# Displays a red dot to reach on my file
def red_dot():
    plt.scatter(RX, RY, color="red", s=15)  # (x, y), couleur, taille (s)
    plt.show()

# Calculates the hypotenuse of a right angled triangle
def hypotenuse(adj, opp):
    return math.sqrt(adj**2 + opp**2)

# right_angled_triangle_angles_using_tangent returns the angles of a right angled triangle using TOA (tangent = opposide / adjacent)
def right_angled_triangle_angles_using_tangent(opp, adj):
    a1 = math.degrees(math.atan2(opp, adj))
    a2 = math.degrees(math.atan2(adj, opp))
    return a1, a2

# law_of_cosines is used to get all of the angle of my triangle only using the lenghts of L1, L2 and L3
def law_of_cosines(a, b, c):
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

# draw creates the board and displays the red dot, the gris and my lines L1, L2 and L3
def draw(a1, a2, a3, name):
    elbow, hand, finger = forward(a1, a2, a3)
    plt.figure() # Creates an empty white paper
    red_dot()
    plt.plot([0, elbow[0], hand[0], finger[0]], [0, elbow[1], hand[1], finger[1]], "o-") # trace the ligne. plot taked the list of all the x coordonates, then the list of all the y coordonates
    plt.xlim(-20, 20) # fixe la zone visible, de -2 à 2 sur l'axe x
    plt.ylim(-20, 20) # fixe la zone visible, de -2 à 2 sur l'axe y
    plt.gca().set_aspect("equal") # gca() means “get current axes”
    plt.grid(True) # displays the grid in order to read the coordinates
    plt.savefig(name) # Saves this into $name
    plt.close() # Closes the figure and free memory

if (is_red_dot_valid() == False): #checks if the red dot is reachable or not
    print("The red dot is out of range.\n")
    exit(1)

p3 = third_angle()
P3X,P3Y = inverse_kinematics(p3)
a1, a2, a3 = law_of_cosines(L1, L2, hypotenuse(P3X, P3Y))
a4, a5 = right_angled_triangle_angles_using_tangent(P3Y, P3X)
print(f"\nSolve(-45) -> {solve(-45)}")
draw(math.radians(a1 + a4), math.radians(a2), math.radians(p3), "arm3.png")
