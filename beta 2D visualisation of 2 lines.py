import math
import matplotlib
matplotlib.use("Agg")  # pas de fenêtre, rendu fichier
import matplotlib.pyplot as plt

L1, L2 = 1, 0.5
REDX, REDY = -0.51890, 0

def forward(a1, a2):
    elbow = (L1 * math.cos(a1), L1 * math.sin(a1)) # elbow = (x, y) du coude
    hand = (elbow[0] + L2 * math.cos(a1 + a2), elbow[1] + L2 * math.sin(a1 + a2)) # hand = (x, y) de la pince
    return elbow, hand

def reachable():
    dist = math.sqrt(REDX**2 + REDY**2)
    if (L1 + L2 < dist or ((max(L1, L2) - min(L1, L2)) > dist)):
        return False
    return True

def red_dot(x, y, size):
    plt.scatter(x, y, color="red", s=size, zorder=2)  # (x, y), couleur, taille (s), profondeur
    plt.show()

def angles():
    hypotenuse = math.sqrt(REDX ** 2 + REDY ** 2)
    a6 = math.degrees(math.atan2(REDY, REDX)) # atan2 permet d'atteindre des zones négatives et de ne pas resté bloqué à 180°
    a1 = math.degrees(math.acos((L2**2 - L1**2 - hypotenuse**2) / (-2 * L1 * hypotenuse)))
    a2 = math.degrees(math.acos((hypotenuse**2 - L1**2 - L2**2) / (-2 * L1 * L2)))
    angle1 = (-1 if (REDX < 0) else 1) * a1 + a6
    angle2 = (1 if (REDX < 0) else -1) * (180 - a2)
    return angle1, angle2

def draw(a1, a2, name): # a1 a2 a3 are the angles in degres
    elbow, hand = forward(a1, a2)
    plt.figure() # crée une feuille blanche vide
    plt.plot([0, elbow[0], hand[0]], [0, elbow[1], hand[1]], "o-", zorder=1) # trace la ligne. plot prend la liste de tous les x, puis la liste de tous les y
    red_dot(REDX, REDY, 10)
    plt.xlim(-3, 3) # fixe la zone visible, de -2 à 2 sur l'axe x
    plt.ylim(-3, 3) # fixe la zone visible, de -2 à 2 sur l'axe y
    plt.gca().set_aspect("equal") # gca() signifie “get current axes”
    plt.grid(True) # affiche le quadrillage, pour lire les coordonnées
    plt.savefig(name) # enregistre la feuille dans le fichier name
    plt.close() # ferme la figure et libère la mémoire

if  not reachable():
    print("The red dot is not reachable.\n")
    exit(1)
angle1, angle2 = angles()
draw(math.radians(angle1), math.radians(angle2), "arm2.png") # math.radians(30) = 0.5236 = 30 degrés exprimés en radians

# a1 = 0, a2 = 0 : le bras est tendu à l’horizontale, la pince doit être en (1.7, 0).
# a1 = 90°, a2 = 0 : tendu vers le haut, pince en (0, 1.7).
# a1 = 0, a2 = 90° : le deuxième segment monte à la verticale, pince en (1.0, 0.7).

# L1 == 1 (longueur bras 1)
# L2 == 0.5 (longueur bras 2)

# la fonction cosinus (cos) prend un angle et donne un rapport
# la fonction arc cosinus (arccos ou cosinus inverse) prend un rapport et donne un angle.
# acos --> output en radians