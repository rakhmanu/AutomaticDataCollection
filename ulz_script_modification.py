#!/usr/bin/env python3

import sys
import numpy as np
import random as rn
import os

grid_N = 3

roslaunch_command = ""

class grid:
    def __init__(self,rows=0,cols=0):
        self.grid_map = np.array(False)
        self.grid_map.resize(rows,cols)
        self.neighbor_name = ["lower","higher","before","after"]
        self.start = -1
        self.goal = -1
        self.obstacles = []

    def clear(self):
        for i in range(len(self.grid_map)):
            for j in range(len(self.grid_map[0])):
                self.grid_map[i][j] = False

    def index2ij(self,ind):
        return ind//len(self.grid_map[0]), ind%len(self.grid_map[0])

    def ij2index(self,i,j):
        return i*len(self.grid_map[0]) + j

    def neighbor(self, that, ind):
        i,j = self.index2ij(ind)
        out_ind = -1
        if(that == self.neighbor_name[0]):
            if(i-1 < 0): #out of range
                out_ind = -1
            else:
                out_ind =  self.ij2index(i-1,j)

        elif(that >= self.neighbor_name[1]):
            if(i+1 == len(self.grid_map)): #out of range
                out_ind =  -1
            else:
                out_ind =  self.ij2index(i+1,j)

        elif(that == self.neighbor_name[2]):
            if(j-1<0): #out of range
                out_ind =  -1
            else:
                out_ind =  self.ij2index(i,j-1)

        elif(that == self.neighbor_name[3]):
            if(j+1 >= len(self.grid_map[0])): #out of range
                out_ind =  -1
            else:
                out_ind =  self.ij2index(i,j+1)

        else:
            print("No such neighbor")
            out_ind =  -1

        return int(out_ind)

    def make_str_objects_param(self):
        total_num_of_cells = len(self.grid_map) * len(self.grid_map[0])
        out_str="(:objects "
        for n in range(total_num_of_cells):
            out_str+="wp" + str(n) + " "
        out_str += "- waypoints)"
        return out_str


    def make_str_param_grid(self):
        total_num_of_cells = len(self.grid_map) * len(self.grid_map[0])
        out_str=""
        for cell in range(total_num_of_cells):
            out_str+="(grid wp" + str(cell) + ")"
        return out_str

    def make_str_obstacle_param(self):
        out_str = ""
        for n in range(len(self.obstacles)):
            out_str += "(obstacle wp" + str(self.obstacles[n]) + ")"
        return out_str


    def make_str_change_gazebo(self): 
        for n in range(len(self.obstacles)):
            out_str = "[" + str(coordinate_goal_str) + "]," + "[" + str(coordinate_obs_str) + "]," + "[" + str(coordinate_obs_str2) + "]," + "[" + str(coordinate_obs_str3) + "],"  + "[" + str(coordinate_obs_str4) + "],"  + "[" + str(coordinate_obs_str5) + "]"  
        return out_str


    def make_str_connect_param(self):
        out_str = ""
        for row in range(len(self.grid_map)):
            for col in range(len(self.grid_map[row])):
                cell_ind = self.ij2index(row,col)
                #print(map.grid_map[row][col])
                for nei in self.neighbor_name:
                    nei_candidate = self.neighbor(nei,cell_ind)
                    if(nei_candidate != -1):
                        #print(str(map.ij2index(row,col)) +" has " + nei +  " neighbor " + str(map.neighbor(nei,map.ij2index(row,col))) )
                        #generate connect string for each wp here
                        out_str += "  (connected wp" + str(cell_ind) + " wp" + str(nei_candidate) + ")\n"
                        #print(temp_str)
        return out_str

    def make_str_log(self):
        out_str=""
        delim = ","
        out_str += str(self.start)
        out_str += delim
        out_str += str(self.goal)
        out_str += delim
        for n in range(len(self.obstacles)):
            out_str+=str(self.obstacles[n])
            if(n < (len(self.obstacles)-1)):
                out_str+=delim

        return out_str

    def mark_occupancy(self,ind,b):
        i,j = self.index2ij(ind)
        self.grid_map[i][j] = b

    def is_occupied(self,ind):
        i,j = self.index2ij(ind)
        return self.grid_map[i][j]

    def set_start_ind(self):
        num_of_cells = len(self.grid_map) * len(self.grid_map[0])
        start_ind = -1
        while(start_ind == -1):
            start_candidate = rn.randint(0,num_of_cells-1)
            if(self.is_occupied(start_candidate) != True):
                start_ind = start_candidate
                #set
                self.start = start_ind
                #mark as occupied
                self.mark_occupancy(start_ind,True)

    def set_goal_ind(self):
        num_of_cells = len(self.grid_map) * len(self.grid_map[0])
        goal_ind = -1
        while(goal_ind == -1):
            goal_candidate = rn.randint(0,num_of_cells-1)
            if(self.is_occupied(goal_candidate) != True):
                goal_ind = goal_candidate
                #set
                self.goal = goal_ind
                #mark as occupied
                self.mark_occupancy(goal_ind,True)

    def set_obstacles(self, n_of_obs):
        num_of_cells = len(self.grid_map) * len(self.grid_map[0])
        obstacles_ind = []
        while(len(obstacles_ind) != n_of_obs):
            obs_candidate = rn.randint(0,num_of_cells-1)
            if(self.is_occupied(obs_candidate) != True):
                #add to obstables list
                obstacles_ind.append(obs_candidate)
                #mark as occupied
                self.mark_occupancy(obs_candidate,True)

        self.obstacles = sorted(obstacles_ind)

    def make_str_complete(self):
        out_str = "(define (problem p)\n (:domain dir2)\n"
        out_str += " " + self.make_str_objects_param() + "\n"
        out_str += " (:init\n"
        out_str += "  (robot_at wp" + str(self.start) + ")\n"
        out_str += "  " + self.make_str_obstacle_param() + "\n"
        out_str += "  " + self.make_str_param_grid() + "\n"
        out_str += self.make_str_connect_param() #includes indent and line
        out_str += " " + ")\n"
        out_str += " " + "(:goal (and (visited wp" + str(self.goal) + ") ))\n"
        out_str += ")"
        return out_str

    def change_gazebo_model_position_script(self):
        out_str = "import rospy " + "\n"
        out_str += "import gazebo_client_function" + " " + "as" + " " + "GCF" + "\n"
        out_str += "rospy.init_node(" + '\'%s\''%'tm_node' + "," + " " + "anonymous=True)" + "\n"
        out_str += "\n"
        out_str += "global" + " " + "ROBOT" + "\n"
        out_str += "ROBOT = rospy.get_param(" + '\'%s\''%'/ROBOT' + "," + "" + '\'%s\''%'gazebo_robot' + ")" + "\n"
        out_str += "print" + " " + "\"%s\""%"ROBOT" + "," + " " + "ROBOT" + "\n"
        out_str += "\n"
        out_str += "init_box_position" + " " + "=" + " " + "[" + self.make_str_change_gazebo() + "]" + "\n"
        out_str += "\n"
        out_str += "place_z" + " " + "=" + " " + "0.0" + "\n"
        out_str += "\n"
        out_str += "numObj" + " " + "=" + " " + "GCF.get_number_of_gazebo_model('Box')" + "\n"
        out_str += "object_name" + " " + "=" + " " + "[('Box'+str(i).zfill(1)) for i in range(numObj)]" + "\n"
        out_str += "\n"
        out_str += "for obj in object_name:" + "\n"
        out_str += "    GCF.change_gazebo_model_position(object_name[i],[i, 0.0, 0])" + "\n"
        out_str += "\n"
        out_str += "box = []" + "\n"
        out_str += "\n"
        out_str += "for i in range(numObj) :   " + "\n"
        out_str += "    box.append(" + '\'%s\''%'aa#' + "+" + '\'%s\''%'Box' + "+" + "str(i)+" + '\'%s\''%'!' + ")" + "\n"
        out_str += "\n"
        out_str += "for i in range(len(init_box_position)) : " + "\n"
        out_str += "    GCF.change_gazebo_model_position(" + '\'%s\''%'Box' + "+" + "str(i), [init_box_position[i][0] , init_box_position[i][1], place_z])" + "\n"
        out_str += "\n"
        out_str += "\n"
        return out_str    

def modify_pddl_random(map,n_obj,pddl_file):
    map.clear()
    #generate grid parameters string
    print(map.make_str_objects_param())
    print(map.make_str_param_grid())

    #pick start
    map.set_start_ind()
    #pick goal
    map.set_goal_ind()
    #pick obstacles
    map.set_obstacles(n_obj)

    #print("robot: " + str(map.start))
    #print("goal: " + str(map.goal))
    #print("obstacles : ", map.obstacles)
    #print(map.grid_map)

    pddl_content = map.make_str_complete()
    print(pddl_content)

    f = open(pddl_file, "w")
    f.write(pddl_content)
    f.close()

    return map


def modify_change_gazebo(map,n_obj,change_gazebo):
    map.clear()
    #generate grid parameters string
    #print(map.make_str_objects_param())
    #print(map.make_str_param_grid())
    #pick start
    map.set_start_ind()
    #pick goal
    map.set_goal_ind()
    #pick obstacles
    map.set_obstacles(n_obj)


    gazebo_content = map.change_gazebo_model_position_script()
    print(gazebo_content)

    save_path = '/home/umka/Documents/ulz_script_with_modification/change_model_gazebo/scripts/'
    change_gazebo2 = os.path.join(save_path, change_gazebo)

    f = open(change_gazebo2, "w+")
    f.write(gazebo_content)
    f.close()

    return map

def run_terminal(cmd):
    return os.system(cmd)

class parameter_set:
    def __init__(self,file=""):
        self.min_obs_n = -1
        self.max_obs_n = -1
        self.repeat_n = -1
        self.gird_row = -1
        self.pddl_file = ""
        self.change_gazebo = ""
        self.run_cmd = ""
        if(file!=""):
            with open(file) as f:
                lines = f.read().splitlines()
            for line in lines:
                temp = line.split(":")
                if(temp[0] == "min_obs_n"):
                    self.min_obs_n = int(temp[1])
                elif(temp[0] == "max_obs_n"):
                    self.max_obs_n = int(temp[1])
                elif(temp[0] == "repeat_n"):
                    self.repeat_n = int(temp[1])
                elif(temp[0] == "grid_row"):
                    self.gird_row = int(temp[1])
                elif(temp[0] == "pddl_file"):
                    self.pddl_file = temp[1]
                elif(temp[0] == "change_gazebo"):
                    self.change_gazebo = temp[1]
                elif(temp[0] == "run_cmd"):
                    self.run_cmd = temp[1]
                else:
                    #unknown parameter
                    print("unknown parameter name")

#main starts here
#if(len(sys.argv) < 2):
#    print("Missing Parameter: Grid size N as in (NxN) is required")
#    sys.exit()

#read parameters from file
param_file_name = "parameters.csv"
print("Read Parameters from File: " + param_file_name)
param_set = parameter_set(param_file_name)

#grid_N = int(sys.argv[1])
grid_N = param_set.gird_row
print("Generating Square Grid with "+ str(grid_N) +" Rows and "+ str(grid_N)+" Cols")
map = grid(grid_N,grid_N)

log_list = []
min_obstacle = param_set.min_obs_n
max_obstacle = param_set.max_obs_n
n_repeat = param_set.repeat_n

#the loop
for obs in range(min_obstacle,max_obstacle+1):
    for repeat in range(n_repeat):
        #modify pddl
        modified_map = modify_pddl_random(map,obs,param_set.pddl_file)
        
        x, y = map.index2ij(map.goal)
        coordinate_goal = [x, y]
        print(coordinate_goal)
        coordinate_goal_str = ','.join(str(e) for e in coordinate_goal)
        
        for obs in range(max_obstacle):
            #x, y = map.index2ij(map.obstacles[obs])
            x, y = map.index2ij(map.obstacles[obs-5])
            coordinate_obs5 = [x, y]
            coordinate_obs_str5 = ','.join(str(e) for e in coordinate_obs5)

            x, y = map.index2ij(map.obstacles[obs-4])
            coordinate_obs4 = [x, y]
            coordinate_obs_str4 = ','.join(str(e) for e in coordinate_obs4) 

            x, y = map.index2ij(map.obstacles[obs-3])
            coordinate_obs3 = [x, y]
            coordinate_obs_str3 = ','.join(str(e) for e in coordinate_obs3)

            x, y = map.index2ij(map.obstacles[obs-2])
            coordinate_obs2 = [x, y]
            coordinate_obs_str2 = ','.join(str(e) for e in coordinate_obs2)

            x, y = map.index2ij(map.obstacles[obs-1])
            coordinate_obs = [x, y]
            print(coordinate_obs)
            coordinate_obs_str = ','.join(str(e) for e in coordinate_obs)
            
            
        modified_gazebo = modify_change_gazebo(map,obs,param_set.change_gazebo)
        #launch and wait until it returns
        run_terminal(param_set.run_cmd)
        #save log
        #start ind, goal ind, obstacle ind
        log_list.append(map.make_str_log())

#leave log
with open('log.csv', 'w') as f:
    for string_log in log_list:
        f.write("%s\n" % string_log)

print("Ulz task done")
sys.exit()
