import numpy as np
import math
import matplotlib.pyplot as plt

class Vehicle:

    def __init__(self,x=0,y=0,yaw=0,v=0,dt=0.1,l = 2.9):
        self.x = x
        self.y = y          # pos x and y
        self.yaw = yaw      # vehicle heading
        self.v = v          #car longitudinal velocity
        self.dt = dt        #sampling time
        self.l = l          # length of vehicle

    def update(self,a,delta):
        self.x = self.x+self.v*math.cos(self.yaw)*self.dt
        self.y = self.y+self.v*math.sin(self.yaw)*self.dt
        self.yaw = self.yaw + self.v/self.l*math.tan(delta)*self.dt  # delta is the steering angle
        self.v = self.v + a*self.dt     # a is the longitudinal acceleration 



class PID :
    def __init__(self,kp,kd,ki):
        self.kp = kp
        self.ki = ki
        self.kd = kd        #pid gains
        self.pre_cte = 0    #previous cross tracking errors
        self.Icte = 0       #Intergral term of errors

    def PIDcontrol(self,target,current,dt):
        cte = target-current
        dcte = (cte-self.pre_cte)/dt
        #self.Icte += cte*dt
        self.pre_cte = cte
        #u = self.kp*cte + self.kd*dcte + self.ki*self.Icte
        u = self.kp * cte + self.kd * dcte
        return u

class PurePursuit:
    def __init__(self, kld = 0.1 , ld=2.0):
        self.kld = kld          #look ahead gain
        self.ld = ld            #look ahead distance

    def targetIndex(self,vehicle,cx,cy):
        dx = [vehicle.x-x for x in cx]
        dy = [vehicle.y-y for y in cy]
        dist = [math.sqrt(diffx**2 + diffy**2) for (diffx,diffy) in zip(dx,dy)]
        index = dist.index(min(dist))
        length = 0.0
        newld = self.kld*vehicle.v + self.ld
        while newld > length and (index+1) < len(cx):
            diffx = cx[index+1]-cx[index]
            diffy = cy[index+1]-cy[index]
            length += math.sqrt(diffx**2+diffy**2)
            index += 1
    
        return index

    def purePursuitControl(self,vehicle,cx,cy,cind):

        index = self.targetIndex(vehicle,cx,cy)

        if cind >= index:
            index = cind
        if index < len(cx):
            tx = cx[index]
            ty = cy[index]
        else:
            tx = cx[-1]
            ty = cy[-1]
            index = len(cx)-1

        alpha = math.atan2(ty - vehicle.y,tx - vehicle.x)-vehicle.yaw  #calculate the alpah

        if vehicle.v < 0:
            alpha = math.pi - alpha  #check if the vehilce is backwarding

        newld = self.kld*vehicle.v + self.ld

        delta = math.atan2(2.0*vehicle.l* math.sin(alpha), newld)

        return delta,index


 

def main():
    # generate path points
    cx = np.arange(0, 50, 1)
    cy = [math.sin(ix / 5.0) * ix / 1.5 for ix in cx]
    target_speed  = [int(4 + 2*((ix/10)%2)) for ix in cx]  #velocity profile
    T = 100.0   # maximum running time

    # set the inital car location of vehicle
    vehicle = Vehicle(x=-0.0, y=-3.0, yaw=0.0, v=0.0)
    lastIndex = len(cx) - 1
    time = 0.0
    x = [vehicle.x]
    y = [vehicle.y]
    yaw = [vehicle.yaw]
    v = [vehicle.v]
    t = [0.0]
    TargetSpeed = [target_speed[0]]
    PIDController = PID(6.0,0.1,0.1)
    PurePursuitController = PurePursuit()

    target_ind = PurePursuitController.targetIndex(vehicle, cx, cy)
    
    

    while T >= time and lastIndex > target_ind:

        di, target_ind = PurePursuitController.purePursuitControl(vehicle,cx,cy,target_ind)
        ai = PIDController.PIDcontrol(target_speed[target_ind], vehicle.v, vehicle.dt)

        vehicle.update(ai,di)

        time = time + vehicle.dt

        x.append(vehicle.x)
        y.append(vehicle.y)
        yaw.append(vehicle.yaw)
        v.append(vehicle.v)
        t.append(time)
        TargetSpeed.append(target_speed[target_ind])

        
        plt.cla()
        plt.subplot(211)
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.plot(cx[target_ind], cy[target_ind], "go", label="target")
        plt.axis([0,100,-40,40])
        plt.grid(True)
        plt.title("Lateral Tracking using pure pursuit")
        plt.subplot(212)
        plt.plot(t,TargetSpeed,".r",label="desired velocity profile")
        plt.plot(t,v,"-b",label="current velocity")
        plt.grid(True)
        plt.title("velocity tracking using PID")
        plt.pause(0.001)
        

if __name__ == '__main__':
    main()