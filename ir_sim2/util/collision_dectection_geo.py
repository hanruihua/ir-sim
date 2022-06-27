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
    polygon.append(polygon[0])

    for i in range(len(polygon)-1):
        segment = [ polygon[i], polygon[i+1] ]
        if collision_cir_seg(circle, segment): 
            return True
    
    return False

def collision_cir_seg(circle, segment):
    # reference: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    point = np.array([circle.x, circle.y])
    sp = np.array([segment[0].x, segment[0].y])
    ep = np.array([segment[1].x, segment[1].y])
    
    l2 = (ep - sp) @ (ep - sp)

    if (l2 == 0.0): return sqrt( (circle.x - segment[0].x)**2 + (circle.y - segment[0].y)**2 ) < circle.r
    
    t = max(0, min(1, ((point-sp) @ (ep-sp)) / l2 ))

    projection = sp + t * (ep-sp)
    relative = projection - point

    return sqrt( relative[0]**2 + relative[1]**2 ) < circle.r

