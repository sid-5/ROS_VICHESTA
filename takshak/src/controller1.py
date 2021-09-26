#!/usr/bin/env python

import rospy
from math import *
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion,Twist
from tf.transformations import quaternion_from_euler,euler_from_quaternion

class Cell:
        def __init__(self,x,y):
            self.parent=None                                    # parent: the point that brought this Cell into the picture of being in desired path
            self.x,self.y=x,y                                   # coordinates of the Cell
            self.g_cost,self.h_cost,self.f_cost=0,0,0
            self.neighbors=[]                                   # neighbors at a distance of 0.25 in x,y or both
            for i in [-0.25,0,0.25]:
                for j in [-0.25,0,0.25]:
                        if i==j==0:
                                continue
                        self.neighbors.append([x+i,y+j])
        def set_cost(self,prev,goal):                                   # calculating costs that determine if the point lies on desired path
            self.g_cost=prev.g_cost+int((((self.x-prev.x)**2+(self.y-prev.y)**2)**0.5)*40)
            self.h_cost = int((((self.x - goal[0]) ** 2 + (self.y - goal[1]) ** 2)**0.5)*40)
            self.f_cost=self.g_cost+self.h_cost

class LandRover:
    extra_obstacles=[
        [5,-4.75],
        [5.75,-4.75],
        [-4,-5],
        [-3,-7]
    ]
    def __init__(self):
        rospy.init_node('controller1')                              # Creating a node
        self.rate = rospy.Rate(10)                              # Rate of 10Hz
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)     # Setting up Publisher that will publish to the topic '/cmd_vel'
        self.vel_msg = Twist()                                  # Object of type Twist to set values for publishing
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)      # Setting up Subscriber to call self.odom_callback when message of type Odometry is received
        self.odom_msg = Odometry()                              # Object of type Odometry where changes are saved
        self.x,self.y,self.yaw=0,0,0
        
        self.occupancy_grid=OccupancyGrid()                         # Object of type OccupancyGrid where map points are saved
        self.occupancy_grid_subscriber=rospy.Subscriber("/map",OccupancyGrid,self.og_callback)  # Setting up Subscriber to call self.og_callback when message of type OccupancyGrid is received

    def og_callback(self,msg):
        ''' The callback function for occupancy_grid_subscriber'''
        self.occupancy_grid=msg
        # print(msg)

    def odom_callback(self,data):
        ''' The callback function for odom_subscriber'''
        self.odom_msg=data
        x  = self.odom_msg.pose.pose.orientation.x
        y  = self.odom_msg.pose.pose.orientation.y
        z  = self.odom_msg.pose.pose.orientation.z
        w  = self.odom_msg.pose.pose.orientation.w

        # getting ebot current coordinates and yaw(orientation)
        self.x,self.y=round(self.odom_msg.pose.pose.position.x,4),round(self.odom_msg.pose.pose.position.y,4)
        self.yaw=round(euler_from_quaternion([x,y,z,w])[2],4)
        # print(self.x,self.y,self.yaw)

    def eulerian_distance(self,x1,y1,x2,y2):
        ''' distance measuring and returning function'''
        return round(((x2-x1)**2+(y2-y1)**2)**0.5,4)

    def obstacles_bool(self,x,y):
        ''' returns True if [x,y] is very close to obstacle'''
        if [x,y] in LandRover.extra_obstacles:
            return True
        i=int(((x+0.0)/0.05)+2000)
        j=int(((y+0.0)/0.05)+2000)
        for i1 in range(-10,11):
                for j1 in range(-10,11):
                # value of -1 indicates unexplored, 0 indicates free space, 100 indicates obstacle
                    point=self.occupancy_grid.data[(j+j1)*4000+(i+i1)]
                    if point!=0:
                        return True
        return False                                        # False as bot can move freely
        
    def points_collector(self,curr):
        ''' collects path points given by A_star_nav'''
        intermediate_points=[]
        curr = curr.parent
        while curr.parent:                                  # loop ends on collecting all intermediate points
            intermediate_points.append([curr.x,curr.y])
            curr=curr.parent                        
        intermediate_points.append([curr.x,curr.y])                     # End point of the journey
        intermediate_points=self.points_reducer(intermediate_points)                # optimising path points
        self.planned_path(intermediate_points)                          # setting out on the journey

    def points_reducer(self,arr):
        ''' optimises path points by rejecting intermediate points that lie on a straight line'''
        local_reducer=[arr[0],arr[1]]
        global_reducer=[]
        for i in range(2,len(arr)):
            dx =(local_reducer[-2][0]-local_reducer[-1][0]==local_reducer[-1][0]-arr[i][0])
            dy =(local_reducer[-2][1]-local_reducer[-1][1]==local_reducer[-1][1]-arr[i][1])
            if dx and dy:                                   # True for 3 points in a line
                local_reducer.append(arr[i])
            else:                                       # straight line ends, reset local points, update global points
                global_reducer.append(local_reducer[0])                 
                local_reducer=[arr[i-1],arr[i]]
        global_reducer.append(local_reducer[0])
        global_reducer.append(local_reducer[-1])
        return global_reducer                                   # returning optimised points to explore
    
    def A_star_nav(self,start,goal):
        ''' A* navigation algorithm to find path points'''
        self.stop()
        if self.obstacles_bool(goal[0],goal[1]) or self.obstacles_bool(start.x,start.y):
            print("No way")                                 # either goal or start recognised as obstacle
            return
        open=[]                                         # list of to-be-explored points: mostly won't lie on the route 
        closed=[]                                       # list of explored points: mostly lie on the route, already with lower cost
        open.append(start)

        while True:
        # choosing point with lowest f_cost to explore
            m=0
            for i in range(len(open)):
                    if open[i].f_cost < open[m].f_cost:
                        m=i
                    elif open[i].f_cost == open[m].f_cost and open[i].h_cost < open[m].h_cost:
                        m=i
            curr=open.pop(m)
            closed.append(curr)

            if curr.x==goal[0] and curr.y==goal[1]:                     # have reached goal, now collect points of the explored path
                    self.points_collector(curr)
                    return

            for pos in curr.neighbors:                          # check for neighbors to explore
                    nbr=Cell(pos[0],pos[1])
                    if self.obstacles_bool(pos[0],pos[1]):                  # reject the neighbor: because of closeness to obstacle
                        continue
                    for i in closed:                            # reject the neighbor: because we have shorter route to this point
                        if nbr.x==i.x and nbr.y==i.y:
                                break
                    else:                                   # neighbor not rejected
                        nbr.set_cost(curr,goal)                     # calculate the route costs: f,g,h
                        for i in open:                          # update if previously included in open
                                if nbr.x==i.x and nbr.y==i.y:
                                    if nbr.g_cost < i.g_cost:
                                            i.g_cost = nbr.g_cost
                                            i.f_cost = nbr.f_cost
                                            i.parent=curr               # updating parent as it provides better route 
                                    break
                        else:                               # never considered this point before: include in open
                                nbr.parent=curr
                                open.append(nbr)

    #Following functions for completing given planned path
    def stop(self):
        ''' Function to stop the ebot '''
        self.vel_msg.linear.x =0
        self.vel_msg.angular.z=0
        self.velocity_publisher.publish(self.vel_msg)
        self.rate.sleep()

    def steer_angle(self,goal_x,goal_y):
        ''' Function to steer and direct ebot's face towards next point'''
        angle=round(atan2(goal_y-self.y,goal_x-self.x)-self.yaw,4)
        if angle >3 or -3 <angle<0:
            b=-1
        elif angle < -3 or 0<angle<3:
            b=1
        while abs(angle)>0.045 and (not rospy.is_shutdown()):
            self.vel_msg.angular.z=b*0.55
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
            angle=round(atan2(goal_y-self.y,goal_x-self.x)-self.yaw,4)
            # print(angle,self.vel_msg)
            if angle >3 or -3 <angle<0:
                b=-1
            elif angle < -3 or 0<angle<3:
                b=1
        self.stop() 
    
    def go_ahead(self,fwd):
        ''' Function to move the ebot towards next point '''
        self.vel_msg.linear.x =fwd
        self.vel_msg.angular.z=0
        self.velocity_publisher.publish(self.vel_msg)
        self.rate.sleep()

    def planned_path(self,arr):
        ''' Implements the motion towards goal with provided optimum points'''
        print(arr)
        for [x,y] in arr[:len(arr)]:
            print(self.x,self.y,self.yaw,x,y)
            self.steer_angle(x,y)
            print("steered towards: ",x,y)
            prev_x,prev_y=self.x,self.y
            while self.eulerian_distance(self.x,self.y,x,y)>0.1:
                if self.eulerian_distance(self.x,self.y,x,y)>0.5 and self.eulerian_distance(self.x,self.y,prev_x,prev_y) >1.0:
                    self.steer_angle(x,y)                       # to remove orientation errors over long distances
                    prev_x,prev_y=self.x,self.y
                    self.go_ahead(1.75)                      # speed up as next point is far
                else:
                    self.go_ahead(0.5)
            self.stop()
        self.stop()
        rospy.loginfo("Reached: x:"+str(round(self.x,2))+" y:"+str(round(self.y,2)))

try:
    x=LandRover()
    for i in range(50):                                     # delaying for 5 seconds 
        x.rate.sleep()
    ix,iy=[0,0]
    Goals=[[11.0,-6],[-1.5,-8],[11,2.75]]        # Task 2 waypoints (provide nearest 0.25 multiple and not exact value)
    i=0
    while i<len(Goals)-1:
        # x.steer_angle(Goals[i+1][0]-ix,Goals[i+1][1]-iy)
        x.A_star_nav(Cell(Goals[i+1][0]-ix,Goals[i+1][1]-iy),
            [Goals[i][0]-ix,Goals[i][1]-iy])
        i+=1
    rospy.loginfo("Reached all Waypoints")
except Exception as e:
    print("Error: ",e)










