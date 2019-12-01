#!/usr/bin/env python
# import rospy
# from geometry_msgs.msg import Twist
from math import sqrt, atan
from shapely.geometry import LineString, Point
from pprint import pprint


class Sphere:
  def __init__(self, centre, radius, alpha, theta):
    self.centre = centre
    self.radius = radius
    self.alpha = alpha
    self.theta = theta

  def myfunc(self):
    print(" Centre " + str(self.centre) + " alpha = " + str(self.alpha) + " theta = " + str(self.theta))

class Sphere_interval:
  def __init__(self, centre, inner_radius, outer_radius, alpha, theta):
    self.centre = centre
    self.inner_radius = inner_radius
    self.outer_radius = outer_radius
    self.alpha = alpha
    self.theta = theta


def distance_centres(s1, s2):
	return sqrt((s1.centre[0] - s2.centre[0])**2 + (s1.centre[1] - s2.centre[1])**2 + (s1.centre[2] - s2.centre[2])**2)

def bool_sphere_intersect(s1, s2, epsilon=0.001): # 1 = intersect
	d = distance_centres(s1, s2)
	if d - (s1.radius + s2.radius) <= epsilon:
		return 1
	else:
		return 0

def points_sphere_intersect(s1, s2):
	d = distance_centres(s1, s2)
	s2_shifted_center = [d, 0, 0]

	x = (d**2 - s2.radius**2 + s1.radius**2)/(2*d)
	locus_yz = s1.radius**2 - x**2 ## y**2 + z**2 = locus_yz 

	tan_beta = sqrt(locus_yz)/x
	beta = atan(tan_beta)

	tan_a = (s2.centre[1] - s1.centre[1]) / (s2.centre[0] - s1.centre[0])
	a = atan(tan_a)

	if inside_interval(s1.alpha, s1. theta, a, beta) == 1:
		return [x, locus_yz] ## locus_yz is in shifted coordinate system
	else:
		return ["NA", "NA"]


def inside_interval(alpha, theta, a, beta):
	if (a + beta <= alpha + theta and a + beta > alpha) or (a + beta <= alpha - theta and a + beta > alpha):
		return 1
	else:
		return 0 # Interval not overlapping


def recursive_sphere_intersect(s1, s2, dr=0.1):
	
	n1 = int((s1.outer_radius - s1.inner_radius)/dr)
	n2 = int((s2.outer_radius - s2.inner_radius)/dr)

	for i in range(n1, -1, -1):
		for j in range(n2, -1, -1):
			si = Sphere(s1.centre, (s1.inner_radius + i*dr), s1.alpha, s1.theta)
			sj = Sphere(s2.centre, (s2.inner_radius + j*dr), s2.alpha, s2.theta)
			if bool_sphere_intersect(si, sj) == 0:
				return 0
			else:
				print("locus of intersection: " + str(points_sphere_intersect(si, sj)))
	return 1


def free_interval():
	

	

s1 = Sphere_interval([0,0,0], 0.5, 1,0, 90) # s1.myfunc()
s2 = Sphere_interval([1, 0, 0], 0.5, 1,0, 180)

recursive_sphere_intersect(s1, s2)


# print(bool_sphere_intersect(s1, s2))
# print(points_sphere_intersect(s1, s2))
