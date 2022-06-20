# import sympy import Point, Polygon
from sympy import Point, Polygon
  
# creating points using Point()
p1, p2, p3, p4 = map(Point, [(0, 2), (0, 0), (1, 0), (1, 2)])

p1, p2, p3, p4 = map(Point, [(0, 5), (0, 0), (1, 0), (1, 2)])
  
# creating polygon using Polygon()
poly = Polygon(p1, p2, p3, p4)
  
# using distance()
shortestDistance = poly.distance(Point(3, 5))

temp = 2 * shortestDistance

print(float(shortestDistance))
print(temp)
