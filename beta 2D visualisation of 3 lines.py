import math
import matplotlib
matplotlib.use("Agg")  # pas de fenêtre, rendu fichier
import matplotlib.pyplot as plt

L1, L2, L3 = 10, 5, 3
RX, RY = 12.4, 4.4

def forward(a1, a2, a3):
    elbow = (L1 * math.cos(a1), L1 * math.sin(a1)) # elbow = (x, y) du coude
    hand = (elbow[0] + L2 * math.cos(a1 + a2), elbow[1] + L2 * math.sin(a1 + a2)) # hand = (x, y) de la pince
    finger = (hand[0] + L3 * math.cos(a1 + a2 + a3), hand[1] + L3 * math.sin(a1 + a2 + a3))
    return elbow, hand, finger

# best_angle allows me to find the right angle for both P2 and P3 so that P2 - P3 == 0
def third_angle(min_angle, max_angle):
    P3X,P3Y = inverse_kinematics(min_angle)
    while (min_angle <= 180 and (hypotenuse(P3X, P3Y) > L1 + L2 or hypotenuse(P3X, P3Y) < abs(L1 - L2))):
        min_angle += 0.5
        P3X,P3Y = inverse_kinematics(min_angle)
    P3X,P3Y = inverse_kinematics(max_angle)
    while (max_angle >= -180 and (hypotenuse(P3X, P3Y) > L1 + L2 or hypotenuse(P3X, P3Y) < abs(L1 - L2))):
        max_angle -= 0.5
        P3X,P3Y = inverse_kinematics(max_angle)
    return (min_angle + max_angle) / 2

#calculates the 3 angles of the arm
def solve(delta):    # Goal: solve(-45) doit donner environ (55.9°, −76.6°, −24.3°)
    L7, L8 = inverse_kinematics(delta)
    L9 = hypotenuse(L7, L8)
    a1, a2, a3 = law_of_cosines(L7, L8, L9)
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

def right_angled_triangle_angles_using_tangent(opp, adj):
    a1 = math.degrees(math.atan2(opp, adj))
    a2 = math.degrees(math.atan2(adj, opp))
    return a1, a2

def law_of_cosines(a, b, c):
    a1 = math.degrees(math.acos((b**2 - a**2 - c**2)/(-2 * a * c)))
    a3 = math.degrees(math.acos((a**2 - b**2 - c**2)/(-2 * b * c)))
    a2 = 180 - a1 - a3
    return [a1, a2, a3]

def inverse_kinematics(angle):
    x = RX - (L3 * math.cos(math.radians(angle)))
    y = RY - (L3 * math.sin(math.radians(angle)))
    return [x, y]

def is_valid():
    hyp = hypotenuse(RX, RY)
    smallest = sum(sorted([L1, L2, L3])[:2])
    maximum = max(L1, L2, L3)
    total = L1 + L2 + L3
    if (hyp > total or hyp < max(maximum - smallest, 0)):
        return False
    return True

def draw(a1, a2, a3, name): # a1 a2 a3 are the angles in degres
    elbow, hand, finger = forward(a1, a2, a3)
    plt.figure() # crée une feuille blanche vide
    red_dot()
    plt.plot([0, elbow[0], hand[0], finger[0]], [0, elbow[1], hand[1], finger[1]], "o-") # trace la ligne. plot prend la liste de tous les x, puis la liste de tous les y
    plt.xlim(-20, 20) # fixe la zone visible, de -2 à 2 sur l'axe x
    plt.ylim(-20, 20) # fixe la zone visible, de -2 à 2 sur l'axe y
    plt.gca().set_aspect("equal") # gca() signifie “get current axes”
    plt.grid(True) # affiche le quadrillage, pour lire les coordonnées
    plt.savefig(name) # enregistre la feuille dans le fichier name
    plt.close() # ferme la figure et libère la mémoire

if (is_valid() == False): #checks if the red dot is reachable or not
    print("The red dot is out of range.\n")
    exit(1)

min_angle, max_angle = -180, 0
p3 = third_angle(min_angle, max_angle)
P3X,P3Y = inverse_kinematics(p3)
a1, a2, a3 = law_of_cosines(L1, L2, hypotenuse(P3X, P3Y))
a4, a5, a6 = right_angled_triangle_angles_using_tangent(P3Y, P3X)
print(f"A4 is now {a4}°")
print(f"The best angle for p3 is {p3}\nThe best angle for p2 is {a2}")
draw(math.radians(a1 + a4), math.radians(a2), math.radians(p3), "arm3.png")
