import rospy 
import gazebo_client_function as GCF
rospy.init_node('tm_node', anonymous=True)

global ROBOT
ROBOT = rospy.get_param('/ROBOT','gazebo_robot')
print "ROBOT", ROBOT

init_box_position = [[3,2],[3,3],[3,0],[2,1],[1,3],[1,0]]

place_z = 0.0

numObj = GCF.get_number_of_gazebo_model('Box')
object_name = [('Box'+str(i).zfill(1)) for i in range(numObj)]

for obj in object_name:
    GCF.change_gazebo_model_position(object_name[i],[i, 0.0, 0])

box = []

for i in range(numObj) :   
    box.append('aa#'+'Box'+str(i)+'!')

for i in range(len(init_box_position)) : 
    GCF.change_gazebo_model_position('Box'+str(i), [init_box_position[i][0] , init_box_position[i][1], place_z])


