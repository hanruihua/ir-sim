import numpy as np
from math import sqrt

# point: np(2,)
# segment: [point1, point2]
# circle: np(2,)
# r: 1

def range_cir_seg(center, r, segment):
    # reference: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
    sp = segment[0]
    ep = segment[1]

    center = np.squeeze(center)
    sp = np.squeeze(sp)
    ep = np.squeeze(ep)

    d = ep - sp
    f = sp - center

    a = d @ d
    b = 2* f@d
    c = f@f - r ** 2

    discriminant = b**2 - 4 * a * c

    if discriminant < 0:
        return False, None, None
    
    else:
        t1 = (-b - sqrt(discriminant)) / (2*a)
        t2 = (-b + sqrt(discriminant)) / (2*a)

        # 3x HIT cases:
        #          -o->             --|-->  |            |  --|->
        # Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit), 

        # 3x MISS cases:
        #       ->  o                     o ->              | -> |
        # FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

        if t1 >=0 and t1<=1:
            int_point = sp + t1 * d
            lrange = np.linalg.norm(int_point - sp)

            return True, int_point, lrange

        # if t2 >= 0 and t2 <=1:
           #  ExitWound(t1<0, t2 hit), 

        return False, None, None


def range_seg_seg(segment1, segment2):
    # reference https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    # p, p+r, q, q+s

    p = segment1[0]
    r = segment1[1] - segment1[0]
    q = segment2[0]
    s = segment2[1] - segment2[0]

    temp1 = np.cross(r, s)
    temp2 = np.cross(q-p, r)

    if temp1 == 0 and temp2 == 0:
        # collinear
        t0 = (q-p) @ r / (r @ r)
        t1 = t0 + s @ r / (r @ r)

        if max(t0, t1) >= 0 and min(t0, t1) < 0:
            int_point = p
            lrange = 0
            return True, int_point, lrange
        
        elif min(t0, t1) >=0 and min(t0, t1) <= 1:
            int_point = p + min(t0, t1) * r
            lrange = np.linalg.norm(int_point - p)
            return True, int_point, lrange

        else:
            return False, None, None
    
    elif temp1 == 0 and temp2 != 0:
        # parallel and non-intersecting
        return False, None, None

    elif temp1 != 0:

        t = np.cross( q-p, s) / np.cross(r, s)
        u = np.cross( q-p, r) / np.cross(r, s)

        if t >=0 and t<=1 and u>=0 and u<= 1:

            int_point = p + t*r
            lrange = np.linalg.norm(int_point - p)
            return True, int_point, lrange
        else: 
            return False, None, None

    else:
        # not parallel and not intersect
        return False, None, None

