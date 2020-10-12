import math

# Dimensions used for the PhantomX robot :
constL1 = 51
constL2 = 63.7
constL3 = 93

# Dimensions used for the simple arm simulation
# bx = 0.07
# bz = 0.25
# constL1 = 0.085
# constL2 = 0.185
# constL3 = 0.250

#define of Al-Kashi theorem
def alKashi(a, b, c):
    value = ((a*a)+(b*b)-(c*c))/(2*a*b)
    return -math.acos(value)

#calculate Direct Kinematics
def computeDK(theta1, theta2, theta3, l1=constL1, l2=constL2, l3=constL3):
    theta1 = theta1 * math.pi / 100.0
    theta2 = (theta2) * math.pi / 180.0
    theta3 = (theta3) * math.pi / 180.0
    print("corrected angles={}, {}, {}".format(theta1*180.0/math.pi, theta2*180.0/math.pi, theta3*180.0/math.pi))

    planContribution = l1 + l2*math.cos(theta2) + l3*math.cos(theta2 + theta3)

    return theta1,theta2,theta3

#calculate Indirect Kinematics
def computeIK(x, y, z, l1=constL1, l2=constL2, l3=constL3):
    theta1 = math.atan2(y, x)

    xp = math.sqrt(x*x+y*y)-l1
    if (xp < 0) :
        print("Destination point too close")
        xp = 0

    d = math.sqrt(math.pow(xp,2) + math.pow(z,2))
    if (d > l2+l3) :
        print ("Destination point too far away")
        d = l2+l3
    

    theta2 = -(alKashi(l2, d, l3) + math.atan2(z, xp))
    theta3 = (math.pi) + alKashi(l2, l3, d)
    
    return (modulo180(math.degrees(theta1)), modulo180(math.degrees(theta2)), modulo180(math.degrees(theta3)))

#set radian to degree
def modulo180(angle) :
    if (-180 < angle < 180) :
        return angle

        angle = angle % 360
        if (angle > 180) :
            return -360 + angle 
        
        

#show the results in the terminal
def main():
    print ("\nDirect Kinematics--> ", computeDK(0, 0, 0))
    print ("\nIndirect Kinematics--> ", computeIK(90, 0, 0))
   


if __name__ == "__main__":
    main()