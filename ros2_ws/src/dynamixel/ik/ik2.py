import math

def ik_cal(x, y):
    l1 = 90 
    l2 = 135
    l3 = 30 
    l4 = 150

    d = x   # d200 -> x: 300
    h = y

    n = h - l1

    i = math.sqrt(l2**2 + l3**2)
    j = math.sqrt(d**2 + n**2)
    if i == 0 or j == 0:
      return None, None, None
    cos_r1 = (i**2 + j**2 - l4**2) / (2*i*j)
    if cos_r1 < -1.0 or cos_r1 > 1.0:
        return None, None, None
    cos_r1 = max(-1.0, min(1.0, cos_r1))  
    r1 = math.acos(cos_r1)


    b = math.atan(l3/l2) # radian

    w = math.atan(n/d) # radian

    theta_1 =((math.pi/2) - (r1 + b + w)) 

    r2 = math.acos(float(((pow(l4,2) + pow(j,2) - pow(i,2)) / (2*l4*j))))

    a = (math.pi)/2 - b

    theta_2 = -(a -r1 -r2) 
    theta_3 = -(r2 - w)


    deg1 = math.degrees(theta_1)
    deg2 = math.degrees(theta_2)
    deg3 = math.degrees(theta_3)



    deg1 = 180 + (-1)*(deg1)

    deg2 = 90 + (-1)*(deg2)

    deg3 = 90 + (-1)*(deg3)




    print("0, %f, %f, %f" % (deg1, deg2, deg3))
   #   print("0, %f, %f, %f" % (theta_1, theta_2, theta_3))

    return(deg1, deg2, deg3)


