def cal_vel(ch):
    vel = 50
    turn = 30

    if ch == 'w': #forward
        return 0,vel,-vel
    if ch == 'x': #backward 
        return 0, -vel, vel
    if ch == 's': #stop
        return 0,0,0

    if ch == 'a': #ccw
        return -vel,-vel,-vel 
    if ch == 'd': #cw
        return  vel,vel,vel 

    if ch == 'q': #left + forward
        return vel,0,-vel
    if ch == 'e': #right + forward
        return -vel,vel, 0


    return 0, 0, 0