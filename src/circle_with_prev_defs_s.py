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


def only_line_intersect(l1, l2):
    x = l1.intersection(l2)
    # pprint(x)
    # print(len(x.bounds))

    if len(x.bounds) == 0:
        return 0
    else:
        return 1


def arc_intersect(l, arc, coord_min, coord_max):
    x = l.intersection(arc.boundary)
    
    if len(x.bounds) == 0:
        return 0
    else:
        if (coord_max[0] <= x.bounds[0] <= coord_min[0]) and (coord_min[1] <= x.bounds[1] <= coord_max[1]):
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
def compute_s1(I, I1, pos_1, pos_2):
    if((I[0][0] == I1[0][0]) and (I[0][1] == I1[0][1]) and (I[1][0] == I1[1][0]) and (pos_1 == pos_2)):
        return 1
    else:
        return 0

## comparing arc gamma 2 of I and I1
def compute_s2(I, I1, pos_1, pos_2):
    if((I[0][0] == I1[0][0]) and (I[0][1] == I1[0][1]) and (I[1][1] == I1[1][1]) and (pos_1 == pos_2)):
        return 1
    else:
        return 0

## comparing line segment l1 of I and I1
def compute_s3(I, I1, pos_1, pos_2):
    
    l1 = LineString([((pos_1[0] + (I[1][0]*cos(I[0][0]))), (pos_1[0] + (I[1][0]*sin(I[0][0])))), ((pos_1[0] + (I[1][1]*cos(I[0][0]))), (pos_1[0] + (I[1][1]*sin(I[0][0]))))])
    l2 = LineString([((pos_2[0] + (I1[1][0]*cos(I1[0][0]))), (pos_2[0] + (I1[1][0]*sin(I1[0][0])))), ((pos_2[0] + (I1[1][1]*cos(I1[0][0]))), (pos_2[0] + (I1[1][1]*sin(I1[0][0]))))])    
    only_line_intersect(l1, l2)

    # epsilon = 1e-5
    # rI_1 = I[1][0]
    # rI_2 = I[1][1]
    # theta_I = I[0][0] * pi / 180.0
    # xI = pos_1[0]
    # yI = pos_1[1]

    # rI1_1 = I1[1][0]
    # rI1_2 = I1[1][1]
    # theta_I1 = I1[0][0] * pi / 180.0
    # xI1 = pos_2[0]
    # yI1 = pos_2[1]

    # if(((( abs(xI + (rI_1*cos(theta_I)) -  (xI1 + (rI1_2*cos(theta_I1)))) < epsilon) and 
    #     ( abs(yI + (rI_1*sin(theta_I)) -  (yI1 + (rI1_2*sin(theta_I1)))) < epsilon) and 
    #     ( abs(xI + (rI_2*cos(theta_I)) - (xI1 + (rI1_1*cos(theta_I1)))) < epsilon) and 
    #     ( abs(yI + (rI_2*sin(theta_I)) -  (yI1 + (rI1_1*sin(theta_I1)))) < epsilon))) or 

    #     ((( abs(xI + (rI_1*cos(theta_I)) -  (xI1 + (rI1_1*cos(theta_I1)))) < epsilon) and 
    #     ( abs(yI + (rI_1*sin(theta_I)) -  (yI1 + (rI1_1*sin(theta_I1)))) < epsilon) and 
    #     ( abs(xI + (rI_2*cos(theta_I)) - (xI1 + (rI1_2*cos(theta_I1)))) < epsilon) and 
    #     ( abs(yI + (rI_2*sin(theta_I)) -  (yI1 + (rI1_2*sin(theta_I1))))) < epsilon))):
    #     return 1
    # else:
    #     return 0
    # # if((I[1][0] == I1[1][0]) and (I[1][1] == I1[1][1]) and (I[0][0] == I1[0][0])):
    # #     return 1
    # # else:
    # #     return 0

## comparing line segment l2 of I and I1
def compute_s4(I, I1, pos_1, pos_2):
    l1 = LineString([((pos_1[0] + (I[1][0]*cos(I[0][0]))), (pos_1[0] + (I[1][0]*sin(I[0][0])))), ((pos_1[0] + (I[1][1]*cos(I[0][0]))), (pos_1[0] + (I[1][1]*sin(I[0][0]))))])
    l2 = LineString([((pos_2[0] + (I1[1][0]*cos(I1[0][0]))), (pos_2[0] + (I1[1][0]*sin(I1[0][0])))), ((pos_2[0] + (I1[1][1]*cos(I1[0][0]))), (pos_2[0] + (I1[1][1]*sin(I1[0][0]))))])    
    only_line_intersect(l1, l2)

    # epsilon = 1e-5
    # rI_1 = I[1][0]
    # rI_2 = I[1][1]
    # theta_I = I[0][1] * pi / 180.0
    # xI = pos_1[0]
    # yI = pos_1[1]

    # rI1_1 = I1[1][0]
    # rI1_2 = I1[1][1]
    # theta_I1 = I1[0][1] * pi / 180.0
    # xI1 = pos_2[0]
    # yI1 = pos_2[1]

    # if(( abs((xI + (rI_1*cos(theta_I)) -  (xI1 + (rI1_2*cos(theta_I1)))) < epsilon) and 
    #     ( abs(yI + (rI_1*sin(theta_I)) -  (yI1 + (rI1_2*sin(theta_I1)))) < epsilon) and 
    #     ( abs(xI + (rI_2*cos(theta_I)) - (xI1 + (rI1_1*cos(theta_I1)))) < epsilon) and 
    #     ( abs(yI + (rI_2*sin(theta_I)) -  (yI1 + (rI1_1*sin(theta_I1))))) < epsilon) or

    #     (( abs(xI + (rI_1*cos(theta_I)) -  (xI1 + (rI1_1*cos(theta_I1)))) < epsilon ) and 
    #     ( abs(yI + (rI_1*sin(theta_I)) -  (yI1 + (rI1_1*sin(theta_I1)))) < epsilon) and 
    #     ( abs(xI + (rI_2*cos(theta_I)) - (xI1 + (rI1_2*cos(theta_I1)))) < epsilon) and 
    #     ( abs(yI + (rI_2*sin(theta_I)) - (yI1 + (rI1_2*sin(theta_I1)))) < epsilon))):
    #     return 1
    # else:
    #     return 0

## compute inclusion test of I in I1
def compute_eI(I, I1, pos_1, pos_2):
    
    


    s1 = compute_s1(I, I1, pos_1, pos_2)  
    s2 = compute_s2(I, I1, pos_1, pos_2)
    s3 = compute_s3(I, I1, pos_1, pos_2)
    s4 = compute_s4(I, I1, pos_1, pos_2)

    print("s1 = " + str(s1) + " s2 = " + str(s2) + " s3 = " + str(s3) + " s4 = " + str(s4))
    q = XNOR(s1, XNOR(s2, XNOR(s3, s4)))
    print("q = " + str(q))
    PI = compute_PI(I, q, s4)
    tI_PI = compute_tI(PI, I1)
    print("tI1_PI = " + str(tI_PI))
    return XNOR(q, tI_PI)

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

                if compute_eI(R, I1, pos_1, pos_2) == 0:
                    L = LL
                    R = LR
                elif compute_eI(L, I1, pos_1, pos_2) == 0:
                    L = RL
                    R = RR
                else:
                    if compute_eI(LL, I1, pos_1, pos_2) != 0:
                        L = LL
                    else:
                        L = LR

                    if compute_eI(RR, I1, pos_1, pos_2) != 0:
                        R = RR
                    else:
                        R = RL

                n = n+1
            J = [[L[0][0], R[0][1]], [L[1][0], R[1][1]]]
            N = N_theta
            n = 0  
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
        ## case 1 - both origins coincide
        # I = [[0, 90], [1, 2]]
        # I1 = [[0, 90], [1, 2]]
        # pos_1 = [0, 0]
        # pos_2 = [0, 0]
        # print("CASE 1: l1, l2 of I coincide with I1")
        # print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        # print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        # print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        # print("*************************************")

        # ## case 2 - 
        # I = [[0, 90], [1, 2]]
        # I1 = [[180, 270], [1, 2]]
        # pos_1 = [0, 0]
        # pos_2 = [3, 0]
        # print("CASE 2: l1 of I coincide with I1")
        # print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        # print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        # print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        # print("*************************************")

        # ## case 3 - 
        # I = [[0, 90], [1, 2]]
        # I1 = [[60, 90], [2, 3]]
        # pos_1 = [0, 0]
        # pos_2 = [0, -1]
        # print("CASE 3: l2 of I coincide with I1")
        # print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        # print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        # print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        # print("*************************************")

        # ## case 4a - 
        # I = [[0, 90], [1, 3]]
        # I1 = [[180, 190], [1, 1.1]]
        # pos_1 = [0, 0]
        # pos_2 = [3, 1]
        # print("CASE 4a: ")
        # print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        # print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        # print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        # print("*************************************")

        # ## case 4b - 
        # I = [[0, 90], [1, 2]]
        # I1 = [[180, 270], [1, 2]]
        # pos_1 = [0, 0]
        # pos_2 = [4, 4]
        # print("CASE 4b: ")
        # print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        # print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        # print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        # print("*************************************")

        # ## case 5 - 
        # I = [[0, 90], [1, 2]]
        # I1 = [[90, 180], [1, 1.1]]
        # pos_1 = [0, 0]
        # pos_2 = [3, 0]
        # print("CASE 5: ")
        # print("origin of I = " + str(pos_1) + " Interval = " + str(I))
        # print("origin of I1 = " + str(pos_2) + " Interval = " + str(I1))
        # print("eI = " + str(compute_eI(I, I1, pos_1, pos_2)))

        # print("*************************************")
        # line1 = LineString([(0, 0), (1, 1)])
        # line2 = LineString([(0, 1), (1, 0)])
        # print(line_line_intersect(line1, line2))

        line = LineString([(0, 0), (1, 1)])
        arc = Point(0, 0).buffer(0.5)
        coord_min = [0.5, 0]
        coord_max = [0, 0.5]
        print(line_arc_intersect(line, arc, coord_min, coord_max))


    except rospy.ROSInterruptException: pass