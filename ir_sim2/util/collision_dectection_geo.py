import numpy as np
from math import sqrt, pi, cos, sin

# collision detection

## circle:  x, y, r
## segment: [point1, point2]
## point: x, y
## point_set: 2*n, matrix 
## rectangle: 
## polygon: [point1, point2,....]  convex

# collision between circle and circle
def collision_cir_cir(circle1, circle2):
    return sqrt( (circle2.x - circle1.x)**2 + (circle2.y - circle1.y)**2 ) - (circle1.r + circle2.r) < 0

# 
def collision_cir_poly(circle, polygon):
    pass

def collision_cir_point(circle, point):
    pass