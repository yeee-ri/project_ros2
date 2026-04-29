import math

l1 = 90 
l2 = 135
l3 = 30 
l4 = 150

d = 300   # d200 -> x: 300
h = 175


n = h - l1

i = math.sqrt(pow(l2,2) + pow(l3,2))
j = math.sqrt(pow(d,2) + pow(n,2))

r1 = math.acos(float((((pow(i,2) + pow(j,2) - pow(l4,2)) / (2*i*j))))) # radian

b = math.atan(l3/l2) # radian

w = math.atan(n/d) # radian

theta_1 =((math.pi/2) - (r1 + b + w)) 

r2 = math.acos(float(((pow(l4,2) + pow(j,2) - pow(i,2)) / (2*l4*j))))

a = (math.pi)/2 - b

theta_2 = -(a -r1 -r2) 
theta_3 = -(r2 - w)




print("%f, %f, %f" % (theta_1, theta_2, theta_3))
