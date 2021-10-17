import numpy as np
import matplotlib.pyplot as plt

lenght_base = 0.3 #[m]
wheel_robot = 0.1 #[m]

init_pose = []
goal_pose = []
x_rota = [] 
y_rota = []

dt = 0.1

kv = 0.5 #velocity gain
kh = 2 #heading gain

class GoToGoal:
    def __init__(self, init_pose, goal_pose):
        self.init_pose = init_pose
        self.goal_pose = goal_pose
    
    def plot_robot(self, x, y, theta, x_i, y_i):
        global x_rota, y_rota

        p1_i = np.array([0.3, 0, 1]).T
        p2_i = np.array([-0.3, 0.2, 1]).T
        p3_i = np.array([-0.3, -0.2, 1]).T

        last_pose_point = self.transformation_matrix(x, y, theta)
        
        p1_move = np.matmul(last_pose_point, p1_i)
        p2_move = np.matmul(last_pose_point, p2_i)
        p3_move = np.matmul(last_pose_point, p3_i)

        plt.plot([p1_move[0], p2_move[0]], [p1_move[1], p2_move[1]] , 'b-')
        plt.plot([p2_move[0], p3_move[0]], [p2_move[1], p3_move[1]], 'b-')
        plt.plot([p3_move[0], p1_move[0]], [p3_move[1], p1_move[1]], 'b-')

        plt.plot(x_i, y_i, color='blue', marker='.')

        plt.plot(self.goal_pose[0], self.goal_pose[1], color='green', marker='.')

        plt.plot(x_rota, y_rota, 'r--')

        plt.pause(0.1)

    def transformation_matrix(self, x, y, theta):
        return np.array([[np.cos(theta), -np.sin(theta), x], [np.sin(theta), np.cos(theta), y], [0, 0, 1]])

    def p_controller(self, r, e_theta):
        global kv, kh

        v = kv * r #proportional controller for linear velocity
        w = kh * e_theta #proportional controller for angular velocity

        return v, w

    def calculate_error_heading(self, theta_asterisk):
        e_theta = np.arctan2(np.sin(theta_asterisk - self.init_pose[2]), np.cos(theta_asterisk - self.init_pose[2])) #error heading [-pi, pi]

        return e_theta

    def diff_drive(self, v, w):
        global dt

        x_n = self.init_pose[0] + v * np.cos(self.init_pose[2]) * dt
        y_n = self.init_pose[1] + v * np.sin(self.init_pose[2]) * dt
        theta_n = self.init_pose[2] + w * dt

        return x_n, y_n, theta_n

    def go_to_goal(self, x_i, y_i):
        x_diff = self.goal_pose[0] - self.init_pose[0]
        y_diff = self.goal_pose[1] - self.init_pose[1]

        r = np.hypot(x_diff, y_diff)

        while r > 0.001:
            x_diff = self.goal_pose[0] - self.init_pose[0]
            y_diff = self.goal_pose[1] - self.init_pose[1]

            r = np.hypot(x_diff, y_diff)
            theta_asterisk = np.arctan2(self.goal_pose[1] - self.init_pose[1], self.goal_pose[0] - self.init_pose[0]) #theta* [-pi, pi]

            e_theta = self.calculate_error_heading(theta_asterisk)
            v, w = self.p_controller(r, e_theta)
            x_n, y_n, theta_n = self.diff_drive(v, w)

            x_rota.append(x_n)
            y_rota.append(y_n)

            self.init_pose[0] = x_n
            self.init_pose[1] = y_n
            self.init_pose[2] = theta_n

            plt.cla()
            self.plot_robot(x_n, y_n, theta_n, x_i, y_i)

if __name__ == '__main__':
    x_i = float(input("Initial x position:"))
    init_pose.append(x_i)
    y_i = float(input("Initial y position:"))
    init_pose.append(y_i)
    theta_i = float(input("Initial angular (theta):"))
    init_pose.append(theta_i)

    x_g = float(input("Go to goal x position:"))
    goal_pose.append(x_g)
    y_g = float(input("Go to goal y position:"))
    goal_pose.append(y_g)

    myRobot = GoToGoal(init_pose, goal_pose)
    myRobot.go_to_goal(x_i, y_i)
