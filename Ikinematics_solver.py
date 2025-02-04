# importing important libraries
import math
from sympy import *
import matplotlib.pyplot as plt
import numpy as np




# Creating the basic formula for Transformation Matrices
class TransformationMatrix:

    # creating an object of the basic DH table

    def __init__(self, theta: float, alpha: float, a: float, d: float):
        self.theta = theta
        self.alpha = alpha
        self.a = a
        self.d = d

    # writing each element of the matrix
    def matrix(self):
        r11 = math.cos(self.theta)
        r12 = -math.sin(self.theta) * math.cos(self.alpha)
        r13 = math.sin(self.theta) * math.sin(self.alpha)
        r14 = self.a * math.cos(self.theta)
        r21 = math.sin(self.theta)
        r22 = math.cos(self.theta) * math.cos(self.alpha)
        r23 = -math.cos(self.theta) * math.sin(self.alpha)
        r24 = self.a * math.sin(self.theta)
        r31 = 0
        r32 = math.sin(self.alpha)
        r33 = math.cos(self.alpha)
        r34 = self.d

        matrix = Matrix([[r11, r12, r13, r14], [r21, r22, r23, r24], [r31, r32, r33, r34], [0, 0, 0, 1]])

        return matrix


# creating an object of each unique robot configuration on top of basic DH table
class MatrixAndJacobian:
    def __init__(self, joint_angle1, joint_angle2, joint_angle3, joint_angle4,
                 joint_angle5, joint_angle6):
        self.joint_angle1 = joint_angle1
        self.joint_angle2 = joint_angle2
        self.joint_angle3 = joint_angle3
        self.joint_angle4 = joint_angle4
        self.joint_angle5 = joint_angle5
        self.joint_angle6 = joint_angle6

        self.frame_0_1 = TransformationMatrix(0 + self.joint_angle1, -90, 0, 128)
        self.frame_1_2 = TransformationMatrix(90 + self.joint_angle2, 180, -612.7, 0)
        self.frame_2_3 = TransformationMatrix(0 + self.joint_angle3, -180, -571.6, 0)
        self.frame_3_4 = TransformationMatrix(90 + self.joint_angle4, 90, 0, -163.9)
        self.frame_4_5 = TransformationMatrix(0 + self.joint_angle5, -90, 0, 115.7)
        self.frame_5_6 = TransformationMatrix(0 + self.joint_angle6, 0, 0, -192.2)

#writing all transformations wrt base
        self.frame_0_T_1 = self.frame_0_1.matrix()
        self.frame_0_T_2 = (self.frame_0_1.matrix() * self.frame_1_2.matrix())
        self.frame_0_T_3 = (self.frame_0_1.matrix() * self.frame_1_2.matrix() * self.frame_2_3.matrix())
        self.frame_0_T_4 = (self.frame_0_1.matrix() * self.frame_1_2.matrix() * self.frame_2_3.matrix() * self.frame_3_4.matrix())
        self.frame_0_T_5 = (self.frame_0_1.matrix() * self.frame_1_2.matrix() * self.frame_2_3.matrix() * self.frame_3_4.matrix() * self.frame_4_5.matrix())
        self.frame_0_T_6 = (self.frame_0_1.matrix() * self.frame_1_2.matrix() * self.frame_2_3.matrix() * self.frame_3_4.matrix() * self.frame_4_5.matrix() * self.frame_5_6.matrix())

# Calculating the matrix Jacobian using method 1
    def jacobian(self):

        k = Matrix([[0],[0],[1]])
        r_0_1 = self.frame_0_T_1[:3, :3]
        r_0_2 = self.frame_0_T_2[:3, :3]
        r_0_3 = self.frame_0_T_3[:3, :3]
        r_0_4 = self.frame_0_T_4[:3, :3]
        r_0_5 = self.frame_0_T_5[:3, :3]

        o_0_0 = Matrix([[0],[0],[0]])
        o_0_1 = self.frame_0_T_1[:3, -1:]
        o_0_2 = self.frame_0_T_3[:3, -1:]
        o_0_3 = self.frame_0_T_3[:3, -1:]
        o_0_4 = self.frame_0_T_4[:3, -1:]
        o_0_5 = self.frame_0_T_5[:3, -1:]
        o_0_6 = self.frame_0_T_6[:3, -1:]

        o_0_6_minus_o_0_0 = o_0_6 - o_0_0
        o_0_6_minus_o_0_1 = o_0_6 - o_0_1
        o_0_6_minus_o_0_2 = o_0_6 - o_0_2
        o_0_6_minus_o_0_3 = o_0_6 - o_0_3
        o_0_6_minus_o_0_4 = o_0_6 - o_0_4
        o_0_6_minus_o_0_5 = o_0_6 - o_0_5

        j_angular = Matrix([[k, r_0_1*k, r_0_2*k , r_0_3*k , r_0_4*k , r_0_5*k]])
        j_linear = Matrix([[k.cross(o_0_6_minus_o_0_0), (r_0_1*k).cross(o_0_6_minus_o_0_1), (r_0_2*k).cross(o_0_6_minus_o_0_2),(r_0_3*k).cross(o_0_6_minus_o_0_3),(r_0_4*k).cross(o_0_6_minus_o_0_4),(r_0_5*k).cross(o_0_6_minus_o_0_5)]])

        jacobian_matrix = Matrix([[j_linear],[j_angular]])
        return jacobian_matrix

# Trajectory plotting of the robot
def main():
    i = 0
    end_effector_position_x = []
    end_effector_position_y = []
    end_effector_position_z = []
    robot_1 = MatrixAndJacobian(0,0,0,0,0,0)
    pprint(robot_1.jacobian())
    for i in np.arange(0.0,360.0,1.0):
        joint_angles = Matrix([[0],[0],[0],[0],[i],[0]])
        robot = MatrixAndJacobian(joint_angles[0,0],joint_angles[1,0],joint_angles[2,0],joint_angles[3,0],joint_angles[4,0],joint_angles[5,0])
        robot_jacobian = robot.jacobian()
        jacobian_inverse = robot_jacobian.pinv()
        x_dot = (-100*(math.sin(i)))*(360/20)
        y_dot = 0
        z_dot = (100 * (math.cos(i))) * (360 / 20)
        end_effector_velocities = Matrix([[x_dot], [y_dot], [z_dot], [0], [0], [0]])
        joint_angle_velocities = jacobian_inverse * end_effector_velocities
        joint_angles = joint_angles + (joint_angle_velocities * 0.0005)
        end_effector_position = robot.frame_0_T_6[:3, -1:]
        end_effector_position_x.append(end_effector_position[0, 0])
        end_effector_position_y.append(end_effector_position[1, 0])
        end_effector_position_z.append(end_effector_position[2, 0])

    graph = plt.axes(projection = '3d')
    graph.scatter(end_effector_position_x,end_effector_position_y,end_effector_position_z)
    graph.view_init(15,90)
    #graph.set_xlim(0,400)
    plt.show()



if __name__ == '__main__':
    main()
