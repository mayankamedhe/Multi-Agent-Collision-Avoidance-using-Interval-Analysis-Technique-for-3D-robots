#!/usr/bin/env python
# import rospy
# from geometry_msgs.msg import Twist
from math import sin, cos, pi
from shapely.geometry import LineString, Point
from pprint import pprint
# from shapely.geometry import Point
###################################################################################
## Define - I = [[theta_1, theta_2], [r1, r2]], pos = [x, y]
## angles are measured from right in anti-clockwise manner (for theta_1 and theta_2)
## origin of coordinate system = origin of I (pos_!)
## XNOR operator (assumption XNOR('X', 'X') = 'X')
####################################################################################

def XNOR(s1, s2):
    if (s1 == s2) and (s1 != 'X') and (s1 != [0,1]):
        return 1
    elif (s1 != s2) and (s1 != 'X') and (s1 != [0,1]) and (s2 != 'X') and (s2 != [0,1]):
        return 0
    elif ((s1 == 'X') and s2 == [0,1]) or ((s2 == 'X') and s1 == [0,1]):
        return [0,1]
    else:
        return 'X'

def right_arc_lower(I, pos):
    return [pos[0] + I[1][0]*cos(I[0][0]), pos[1] + I[1][0]*sin(I[0][0])]  

def right_arc_upper(I, pos):
    return [pos[0] + I[1][1]*cos(I[0][0]), pos[1] + I[1][1]*sin(I[0][0])]  

def left_arc_lower(I, pos):
    return [pos[0] + I[1][0]*cos(I[0][1]), pos[1] + I[1][0]*sin(I[0][1])]  

def left_arc_upper(I, pos):
    return [pos[0] + I[1][1]*cos(I[0][1]), pos[1] + I[1][1]*sin(I[0][1])]  

def only_line_intersect(l1, l2):
    x = l1.intersection(l2)
   
    if len(x.bounds) == 0:
        return 0
    else:
        return 1

def line_arc_intersect(l, arc, coord_min, coord_max):
    x = l.intersection(arc.boundary)
    if len(x.bounds) == 0:
        return 0
    else:
        if (coord_max[0] <= x.bounds[0] <= coord_min[0]) and (coord_min[1] <= x.bounds[1] <= coord_max[1]):
            return 1
        else:
            return 0
    
def arc_arc_intersect(arc1, arc2, coord_min_arc1, coord_max_arc1, coord_min_arc2, coord_max_arc2):
    x = (arc1.boundary).intersection(arc2.boundary)
    
    if len(x.bounds) == 0:
        return 0
    else:
        if (coord_max_arc1[0] <= x.bounds[0] <= coord_min_arc1[0]) and (coord_min_arc1[1] <= x.bounds[1] <= coord_max_arc1[1]) and (coord_max_arc2[0] <= x.bounds[0] <= coord_min_arc2[0]) and (coord_min_arc2[1] <= x.bounds[1] <= coord_max_arc2[1]):
            return 1
        else:
            return 0

## returning right of arc gamma 1 when s4 = 1 and q = 0 and left extreme point otherwise
def compute_PI(I, q, s4):
	r1 = I[1][1]
	if((q == 0) and (s4 == 1)):
		theta_1 = I[0][0]
		return [[theta_1, theta_1], [r1, r1]]
	else:
		theta_2 = I[0][1]
		return [[theta_2, theta_2], [r1, r1]]

## comparing extreme points of arc gamma 1 of I with I1
def compute_tI(PI, I1):
	if((PI[0][0] >= I1[0][0]) and (PI[0][0] <= I1[0][1]) and (PI[1][0] >= I1[1][0]) and (PI[1][0] <= I1[1][1])):
		return 1
	else:
		return 0

## comparing arc gamma 1 of I and I1 
def compute_s11(I, I1, pos_1, pos_2):
    arc1 = Point(pos_1[0], pos_1[1]).buffer(I[1][0])
    arc2 = Point(pos_2[0], pos_2[1]).buffer(I1[1][0])
    coord_min_arc1 = right_arc_lower(I, pos_1)
    coord_max_arc1 = left_arc_lower(I, pos_1)
    coord_min_arc2 = right_arc_lower(I1, pos_2)
    coord_max_arc2 = left_arc_lower(I1, pos_2)

    return arc_arc_intersect(arc1, arc2, coord_min_arc1, coord_max_arc1, coord_min_arc2, coord_max_arc2) 


def compute_s12(I, I1, pos_1, pos_2):
    arc1 = Point(pos_1[0], pos_1[1]).buffer(I[1][0])
    arc2 = Point(pos_2[0], pos_2[1]).buffer(I1[1][1])
    coord_min_arc1 = right_arc_lower(I, pos_1)
    coord_max_arc1 = left_arc_lower(I, pos_1)
    coord_min_arc2 = right_arc_upper(I1, pos_2)
    coord_max_arc2 = left_arc_upper(I1, pos_2)

    return arc_arc_intersect(arc1, arc2, coord_min_arc1, coord_max_arc1, coord_min_arc2, coord_max_arc2) 
    

def compute_s13(I, I1, pos_1, pos_2):
    right_arc_lower_I1 = right_arc_lower(I1, pos_2)
    right_arc_upper_I1 = right_arc_upper(I1, pos_2)

    right_arc_lower_I = right_arc_lower(I, pos_1)
    left_arc_lower_I = right_arc_lower(I, pos_1)
    
    line = LineString([(right_arc_lower_I1[0], right_arc_lower_I1[1]), (right_arc_upper_I1[0], right_arc_upper_I1[1])])
    arc = Point(pos_1[0], pos_1[1]).buffer(I[1][0])
    coord_min = right_arc_lower_I
    coord_max = left_arc_lower_I

    return line_arc_intersect(line, arc, coord_min, coord_max)

def compute_s14(I, I1, pos_1, pos_2):
    left_arc_lower_I1 = left_arc_lower(I1, pos_2)
    left_arc_upper_I1 = left_arc_upper(I1, pos_2)

    right_arc_lower_I = right_arc_lower(I, pos_1)
    left_arc_lower_I = right_arc_lower(I, pos_1)
    
    line = LineString([(left_arc_lower_I1[0], left_arc_lower_I1[1]), (left_arc_upper_I1[0], left_arc_upper_I1[1])])
    arc = Point(pos_1[0], pos_1[1]).buffer(I[1][0])
    coord_min = right_arc_lower_I
    coord_max = left_arc_lower_I

    return line_arc_intersect(line, arc, coord_min, coord_max)


def compute_s21(I, I1, pos_1, pos_2):
    arc1 = Point(pos_1[0], pos_1[1]).buffer(I[1][1])
    arc2 = Point(pos_2[0], pos_2[1]).buffer(I1[1][0])
    coord_min_arc1 = right_arc_upper(I, pos_1)
    coord_max_arc1 = left_arc_upper(I, pos_1)
    coord_min_arc2 = right_arc_lower(I1, pos_2)
    coord_max_arc2 = left_arc_lower(I1, pos_2)

    return arc_arc_intersect(arc1, arc2, coord_min_arc1, coord_max_arc1, coord_min_arc2, coord_max_arc2) 


## comparing arc gamma 2 of I and I1
def compute_s22(I, I1, pos_1, pos_2):
    arc1 = Point(pos_1[0], pos_1[1]).buffer(I[1][1])
    arc2 = Point(pos_2[0], pos_2[1]).buffer(I1[1][1])
    coord_min_arc1 = right_arc_upper(I, pos_1)
    coord_max_arc1 = left_arc_upper(I, pos_1)
    coord_min_arc2 = right_arc_upper(I1, pos_2)
    coord_max_arc2 = left_arc_upper(I1, pos_2)

    return arc_arc_intersect(arc1, arc2, coord_min_arc1, coord_max_arc1, coord_min_arc2, coord_max_arc2) 
    

def compute_s23(I, I1, pos_1, pos_2):
    right_arc_lower_I1 = right_arc_lower(I1, pos_2)
    right_arc_upper_I1 = right_arc_upper(I1, pos_2)

    right_arc_upper_I = right_arc_upper(I, pos_1)
    left_arc_upper_I = right_arc_upper(I, pos_1)
    
    line = LineString([(right_arc_lower_I1[0], right_arc_lower_I1[1]), (right_arc_upper_I1[0], right_arc_upper_I1[1])])
    arc = Point(pos_1[0], pos_1[1]).buffer(I[1][1])
    coord_min = right_arc_upper_I
    coord_max = left_arc_upper_I

    return line_arc_intersect(line, arc, coord_min, coord_max)

def compute_s24(I, I1, pos_1, pos_2):
    left_arc_lower_I1 = left_arc_lower(I1, pos_2)
    left_arc_upper_I1 = left_arc_upper(I1, pos_2)

    right_arc_upper_I = right_arc_upper(I, pos_1)
    left_arc_upper_I = right_arc_upper(I, pos_1)
    
    line = LineString([(left_arc_lower_I1[0], left_arc_lower_I1[1]), (left_arc_upper_I1[0], left_arc_upper_I1[1])])
    arc = Point(pos_1[0], pos_1[1]).buffer(I[1][1])
    coord_min = right_arc_upper_I
    coord_max = left_arc_upper_I

    return line_arc_intersect(line, arc, coord_min, coord_max)

def compute_s31(I, I1, pos_1, pos_2):
    right_arc_lower_I = right_arc_lower(I, pos_1)
    right_arc_upper_I = right_arc_upper(I, pos_1)

    right_arc_lower_I1 = right_arc_lower(I1, pos_2)
    left_arc_lower_I1 = right_arc_lower(I1, pos_2)
    
    line = LineString([(right_arc_lower_I[0], right_arc_lower_I[1]), (right_arc_upper_I[0], right_arc_upper_I[1])])
    arc = Point(pos_2[0], pos_2[1]).buffer(I1[1][0])
    coord_min = right_arc_lower_I1
    coord_max = left_arc_lower_I1

    return line_arc_intersect(line, arc, coord_min, coord_max)

def compute_s32(I, I1, pos_1, pos_2):
    right_arc_lower_I = right_arc_lower(I, pos_1)
    right_arc_upper_I = right_arc_upper(I, pos_1)

    right_arc_upper_I1 = right_arc_upper(I1, pos_2)
    left_arc_upper_I1 = right_arc_upper(I1, pos_2)
    
    line = LineString([(right_arc_lower_I[0], right_arc_lower_I[1]), (right_arc_upper_I[0], right_arc_upper_I[1])])
    arc = Point(pos_2[0], pos_2[1]).buffer(I1[1][1])
    coord_min = right_arc_upper_I1
    coord_max = left_arc_upper_I1

    return line_arc_intersect(line, arc, coord_min, coord_max)


## comparing line segment l1 of I and I1
def compute_s33(I, I1, pos_1, pos_2):
    right_arc_lower_I = right_arc_lower(I, pos_1)
    right_arc_upper_I = right_arc_upper(I, pos_1)
   
    right_arc_lower_I1 = right_arc_lower(I1, pos_2)
    right_arc_upper_I1 = right_arc_upper(I1, pos_2)
    
    l1 = LineString([(right_arc_lower_I[0], right_arc_lower_I[1]), (right_arc_upper_I[0], right_arc_upper_I[1])])
    l2 = LineString([(right_arc_lower_I1[0], right_arc_lower_I1[1]), (right_arc_upper_I1[0], right_arc_upper_I1[1])])    
    return only_line_intersect(l1, l2)

def compute_s34(I, I1, pos_1, pos_2):
    right_arc_lower_I = right_arc_lower(I, pos_1)
    right_arc_upper_I = right_arc_upper(I, pos_1)
   
    left_arc_lower_I1 = left_arc_lower(I1, pos_2)
    left_arc_upper_I1 = left_arc_upper(I1, pos_2)
    
    l1 = LineString([(right_arc_lower_I[0], right_arc_lower_I[1]), (right_arc_upper_I[0], right_arc_upper_I[1])])
    l2 = LineString([(left_arc_lower_I1[0], left_arc_lower_I1[1]), (left_arc_upper_I1[0], left_arc_upper_I1[1])])    
    return only_line_intersect(l1, l2)    

def compute_s41(I, I1, pos_1, pos_2):
    left_arc_lower_I = left_arc_lower(I, pos_1)
    left_arc_upper_I = left_arc_upper(I, pos_1)

    right_arc_lower_I1 = right_arc_lower(I1, pos_2)
    left_arc_lower_I1 = right_arc_lower(I1, pos_2)
    
    line = LineString([(left_arc_lower_I[0], left_arc_lower_I[1]), (left_arc_upper_I[0], left_arc_upper_I[1])])
    arc = Point(pos_2[0], pos_2[1]).buffer(I1[1][0])
    coord_min = right_arc_lower_I1
    coord_max = left_arc_lower_I1

    return line_arc_intersect(line, arc, coord_min, coord_max)

def compute_s42(I, I1, pos_1, pos_2):
    left_arc_lower_I = left_arc_lower(I, pos_1)
    left_arc_upper_I = left_arc_upper(I, pos_1)

    right_arc_upper_I1 = right_arc_upper(I1, pos_2)
    left_arc_upper_I1 = right_arc_upper(I1, pos_2)
    
    line = LineString([(left_arc_lower_I[0], left_arc_lower_I[1]), (left_arc_upper_I[0], left_arc_upper_I[1])])
    arc = Point(pos_2[0], pos_2[1]).buffer(I1[1][1])
    coord_min = right_arc_upper_I1
    coord_max = left_arc_upper_I1

    return line_arc_intersect(line, arc, coord_min, coord_max)
    

def compute_s43(I, I1, pos_1, pos_2):
    right_arc_lower_I1 = right_arc_lower(I1, pos_2)
    right_arc_upper_I1 = right_arc_upper(I1, pos_2)
   
    left_arc_lower_I = left_arc_lower(I, pos_1)
    left_arc_upper_I = left_arc_upper(I, pos_1)
    
    l1 = LineString([(right_arc_lower_I1[0], right_arc_lower_I1[1]), (right_arc_upper_I1[0], right_arc_upper_I1[1])])
    l2 = LineString([(left_arc_lower_I[0], left_arc_lower_I[1]), (left_arc_upper_I[0], left_arc_upper_I[1])])    
    return only_line_intersect(l1, l2)        


## comparing line segment l2 of I and I1
def compute_s44(I, I1, pos_1, pos_2):
	left_arc_lower_I = left_arc_lower(I, pos_1)
	left_arc_upper_I = left_arc_upper(I, pos_1)

	left_arc_lower_I1 = left_arc_lower(I1, pos_2)
	left_arc_upper_I1 = left_arc_upper(I1, pos_2)
	l1 = LineString([(left_arc_lower_I[0], left_arc_lower_I[1]), (left_arc_upper_I[0], left_arc_upper_I[1])])
	l2 = LineString([(left_arc_lower_I1[0], left_arc_lower_I1[1]), (left_arc_upper_I1[0], left_arc_upper_I1[1])])    
	return only_line_intersect(l1, l2)


## compute inclusion test of I in I1
def compute_eI(I, I1, pos_1, pos_2):
    

    s11 = compute_s11(I, I1, pos_1, pos_2)  
    s12 = compute_s12(I, I1, pos_1, pos_2)
    s13 = compute_s13(I, I1, pos_1, pos_2)
    s14 = compute_s14(I, I1, pos_1, pos_2)

    q1 = XNOR(s11, XNOR(s12, XNOR(s13, s14)))
    PI1 = compute_PI(I, q1, s14)
    tI_PI1 = compute_tI(PI1, I1)
    
    a1 = XNOR(q1, tI_PI1)

    #####
    s21 = compute_s21(I, I1, pos_1, pos_2)  
    s22 = compute_s22(I, I1, pos_1, pos_2)
    s23 = compute_s23(I, I1, pos_1, pos_2)
    s24 = compute_s24(I, I1, pos_1, pos_2)

    q2 = XNOR(s21, XNOR(s22, XNOR(s23, s24)))
    PI2 = compute_PI(I, q2, s24)
    tI_PI2 = compute_tI(PI2, I1)

    a2 = XNOR(q2, tI_PI2)
    
    #####
    s31 = compute_s31(I, I1, pos_1, pos_2)  
    s32 = compute_s32(I, I1, pos_1, pos_2)
    s33 = compute_s33(I, I1, pos_1, pos_2)
    s34 = compute_s34(I, I1, pos_1, pos_2)

    q3 = XNOR(s31, XNOR(s32, XNOR(s33, s34)))
    PI3 = compute_PI(I, q3, s34)
    tI_PI3 = compute_tI(PI3, I1)

    a3 = XNOR(q3, tI_PI3)
    
    #####
    s41 = compute_s41(I, I1, pos_1, pos_2)  
    s42 = compute_s42(I, I1, pos_1, pos_2)
    s43 = compute_s43(I, I1, pos_1, pos_2)
    s44 = compute_s44(I, I1, pos_1, pos_2)

    q4 = XNOR(s41, XNOR(s42, XNOR(s43, s44)))
    PI4 = compute_PI(I, q4, s44)
    tI_PI4 = compute_tI(PI4, I1)

    a4 = XNOR(q4, tI_PI4)

    # print("s11 = " + str(s11))
    # print("s12 = " + str(s12))
    # print("s13 = " + str(s13))
    # print("s14 = " + str(s14))
    # print("s21 = " + str(s21))
    # print("s22 = " + str(s22))
    # print("s23 = " + str(s23))
    # print("s24 = " + str(s24))
    # print("s31 = " + str(s31))
    # print("s32 = " + str(s32))
    # print("s33 = " + str(s33))
    # print("s34 = " + str(s34))
    # print("s41 = " + str(s41))
    # print("s42 = " + str(s42))
    # print("s43 = " + str(s43))
    # print("s44 = " + str(s44))

    # print("q1 = " + str(q1))
    # print("q2 = " + str(q2))
    # print("q3 = " + str(q3))
    # print("q4 = " + str(q4))

    # print("T1 = " + str(tI_PI1))
    # print("T2 = " + str(tI_PI2))
    # print("T3 = " + str(tI_PI3))
    # print("T4 = " + str(tI_PI4))

    # print("a1 = " + str(a1))
    # print("a2 = " + str(a2))
    # print("a3 = " + str(a3))
    # print("a4 = " + str(a4))
    return XNOR(a1, XNOR(a2, XNOR(a3, a4)))

## returns overlapping interval of I in I1
def interval_inclusion(I, I1, N_r, N_theta, pos_1, pos_2):
    if compute_eI(I, I1, pos_1, pos_2) == 0:
        J = [[0, 0], [0, 0]]
    else:
        N = N_r
        n = 0
        for i in range(1, 2):
        	L = [[I[0][0], (I[0][0] + I[0][1])/2 ], [I[1][0], (I[1][0] + I[1][1])/2 ]] 
        	R = [[(I[0][0] + I[0][1])/2, I[0][1] ], [(I[1][0] + I[1][1])/2, I[1][1] ]]
        	while n < N:
        		LL = [[L[0][0], (L[0][0] + L[0][1])/2 ], [L[1][0], (L[1][0] + L[1][1])/2 ]]
        		LR = [[(L[0][0] + L[0][1])/2, L[0][1] ], [(L[1][0] + L[1][1])/2, L[1][1] ]]
        		RL = [[R[0][0], (R[0][0] + R[0][1])/2 ], [R[1][0], (R[1][0] + R[1][1])/2 ]]
        		RR = [[(R[0][0] + R[0][1])/2, R[0][1] ], [(R[1][0] + R[1][1])/2, R[1][1] ]]
        		print("EI of R " + str(compute_eI(R, I1, pos_1, pos_2)))
        		if compute_eI(R, I1, pos_1, pos_2) == 0:
        			L = LL
        			R = LR
        			print("dbg1")
        		elif compute_eI(L, I1, pos_1, pos_2) == 0:
        			L = RL
        			R = RR
        			print("dbg2")
        		else:
        			if compute_eI(LL, I1, pos_1, pos_2) != 0:
        				L = LL
        				print("dbg3")
        			else:
        				L = LR
        				print("dbg4")
        			print("EI of RR " + str(compute_eI(RR, I1, pos_1, pos_2)))
        			if compute_eI(RR, I1, pos_1, pos_2) != 0:
        				R = RR
        				print("dbg5")
        			else:
        				R = RL
        				print("dbg6")
        		n = n+1
        	J = [[L[0][0], R[0][1]], [L[1][0], R[1][1]]]
        	N = N_theta
        	n = 0
        	print("i = " + str(i))  
        return J      

def main():
    pub = rospy.Publisher('simple_create/cmd_vel', Twist, queue_size=10)
    rospy.init_node('circler', anonymous=True)

    rate = rospy.Rate(2) # 10hz
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 3

    while not rospy.is_shutdown():
        # msg.linear.x += .2
        # pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        # case 1 - both origins coincide
        I = [[0, 90], [1, 2]]
        I1 = [[0, 90], [1, 2]]
        pos_1 = [0, 0]
        pos_2 = [0, 0]
        print("CASE 1: l1, l2 of I coincide with I1")
        print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))


        # N_r = 1
        # N_theta = 1

        # print(interval_inclusion(I, I1, N_r, N_theta, pos_1, pos_2))
        print("*************************************")

        ## case 2 - 
        I = [[0, 90], [1, 2]]
        I1 = [[180, 270], [1, 2]]
        pos_1 = [0, 0]
        pos_2 = [3, 0]
        print("CASE 2: l1 of I coincide with I1")
        print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))
        # N_r = 1
        # N_theta = 1

        # print(interval_inclusion(I, I1, N_r, N_theta, pos_1, pos_2))
        
        print("*************************************")

        ## case 3 - 
        I = [[0, 90], [1, 2]]
        I1 = [[60, 90], [2, 3]]
        pos_1 = [0, 0]
        pos_2 = [0, -1]
        print("CASE 3: l2 of I coincide with I1")
        print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        # N_r = 1
        # N_theta = 1

        # print(interval_inclusion(I, I1, N_r, N_theta, pos_1, pos_2))
        
        print("*************************************")

        ## case 4a - 
        I = [[0, 90], [1, 3]]
        I1 = [[180, 190], [1, 1.1]]
        pos_1 = [0, 0]
        pos_2 = [3, 1]
        print("CASE 4a: ")
        print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        # N_r = 1
        # N_theta = 1

        # print(interval_inclusion(I, I1, N_r, N_theta, pos_1, pos_2))
        
        print("*************************************")

        ## case 4b - 
        I = [[0, 90], [1, 2]]
        I1 = [[180, 270], [1, 2]]
        pos_1 = [0, 0]
        pos_2 = [4, 4]
        print("CASE 4b: ")
        print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        # N_r = 1
        # N_theta = 1

        # print(interval_inclusion(I, I1, N_r, N_theta, pos_1, pos_2))
        
        print("*************************************")

        ## case 5 - 
        I = [[0, 90], [1, 2]]
        I1 = [[90, 180], [1, 1.1]]
        pos_1 = [0, 0]
        pos_2 = [3, 0]
        print("CASE 5: ")
        print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        print("*************************************")
        # line1 = LineString([(0, 0), (1, 1)])
        # line2 = LineString([(0, 1), (1, 0)])
        # print(line_line_intersect(line1, line2))

        # line = LineString([(0, 0), (1, 1)])
        # arc = Point(0, 0).buffer(0.5)
        # coord_min = [0.5, 0]
        # coord_max = [0, 0.5]
        # print(line_arc_intersect(line, arc, coord_min, coord_max))


        # I = [[0, 90], [1, 2]]
        # I1 = [[90, 180], [1, 2]]
        # pos_1 = [0, 0]
        # pos_2 = [3, 0]
        # print("CASE random:")
        # print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        # print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        # print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))


        # N_r = 1
        # N_theta = 1

        # print(interval_inclusion(I, I1, N_r, N_theta, pos_1, pos_2))

    except rospy.ROSInterruptException: pass