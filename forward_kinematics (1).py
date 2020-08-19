

#from mpmath import *,
import numpy as np
from math import atan2, cos, sin, acos, pi,exp
class robotarm:
    first_matrix =None
    second_matrix =None
    def forward_kinematics_matrix(self,rpy_array,xyz_array,q_array,axis_array):

      
        epsilon = 10*exp(-9)
        self.final=np.identity(4,dtype=np.float64)
      
        #Place holder for forward kinematics for all joints
        
        for counter in range (len(q_array)):
            #gimbal lock
            if (rpy_array[counter][1] == pi/2):
                 #rpy_array[counter][0] += epsilon
                 rpy_array[counter][1] += epsilon
                 #rpy_array[counter][2] += epsilon
                 #print(rpy_array[counter][1])
        #roll angle
            R_x = np.array([[1, 0, 0],
                              [0, np.cos(rpy_array[counter][0]), -np.sin(rpy_array[counter][0])],
                              [0, np.sin(rpy_array[counter][0]), np.cos(rpy_array[counter][0])]],dtype=np.float64)
                
        #pitch angle
            R_y =  np.array([[np.cos(rpy_array[counter][1]), 0, np.sin(rpy_array[counter][1])],
                              [0, 1, 0],
                              [-np.sin(rpy_array[counter][1]), 0, np.cos(rpy_array[counter][1])]],dtype=np.float64)
                
        #yaw angle
            R_z =  np.array([[np.cos(rpy_array[counter][2]), -np.sin(rpy_array[counter][2]), 0],
                              [np.sin(rpy_array[counter][2]), np.cos(rpy_array[counter][2]), 0],
                              [0, 0, 1]],dtype=np.float64)
        #rotational matrix
            Rot =  np.matmul(R_z, np.matmul(R_y, R_x),dtype=np.float64)
            
        #translation matrix
            #translation = np.array([[xyz_array[counter][0]],[xyz_array[counter][1]],[xyz_array[counter][2]]])
            
        #matrix stacking 
            self.first_matrix = np.hstack((Rot,np.array([[xyz_array[counter][0]],[xyz_array[counter][1]],[xyz_array[counter][2]]])))
            
            #sca1 = np.array([[0,0,0,1]])
        ##homogeneous matrix using rpy
            self.first_matrix = np.vstack((self.first_matrix,np.array([[0,0,0,1]])))
            
           
        #second homogeneous matrix using q_array and axis_array
            c = np.cos(q_array[counter])
            s = np.sin(q_array[counter])
            if np.linalg.norm(axis_array[counter]) != 0:
                axis_array[counter] = axis_array[counter]/np.linalg.norm(axis_array[counter])
             
        #rotation matrix using axis_array
               
            rot = np.array([[c,0,0],[0,c,0],[0,0,c]],dtype = np.float64)
            
            
            rot += np.outer(axis_array[counter],axis_array[counter]) * (1.0-c)
            axis_array[counter] *= s
            
            rot += np.array([[0,-axis_array[counter][2],axis_array[counter][1]],
                             [axis_array[counter][2],0,-axis_array[counter][0]],
                             [-axis_array[counter][1],axis_array[counter][0],0]],dtype=np.float64)
            
            
        #translation matrix
           # translation1 = np.array([[0],[0],[0]])
        #matrix stacking
            self.second_matrix = np.hstack((rot,np.array([[0],[0],[0]])))
            
            #sca2 = np.array([[0,0,0,1]])  
        #second homogeneous matrix
            self.second_matrix = np.vstack((self.second_matrix,np.array([[0,0,0,1]])))
        #final end effector position
            
           
            self.final = np.matmul(self.final,np.matmul(self.first_matrix , self.second_matrix),dtype=np.float64)
        return self.final
            
       
    
myarm=robotarm()
myarm.forward_kinematics_matrix(np.array([[0,0,0],[-1.57079633,0,0],[1.57079633,0,0],[1.57079633,0,0],[-1.57079633,0,0],
[1.57079633,0,0],[ 1.57079633,0,0],[0,0,0],[0,0,0]]),

np.array([[0,0,0.333],[0,0,0],[0,-0.316,0],[0.0825,0,0],[-0.0825,0.384,0],
[0,0,0],[0.088,0,0],[0,0,0.107],[0,0,0.1034]]),

np.array([[0],[-pi/4],[0],[-((3*pi)/4)],[0],[pi/2],[pi/4],[0],[0]]),
                                
np.array( [[0.0,0.0,1.0],[0.0,0.0,1.0],[0.0,0.0,1.0],[0.0,0.0,1.0],[0.0,0.0,1.0],
[0.0,0.0,1.0],[0.0,0.0,1.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]))


