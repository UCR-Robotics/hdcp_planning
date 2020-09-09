from math import pi, sqrt, atan2, cos, sin
import numpy as np
import matplotlib.pyplot as plt

import rospy
import tf
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from obstacle_detector.msg import Obstacles, CircleObstacle, SegmentObstacle

from vector2d import Vector2D


class TurtleBot:
    def __init__(self, hexmap, turning_radius):
        self.hexmap = hexmap
        self.turning_radius = turning_radius

        # motion planning parameters
        self.sample_period = 0.1
        self.rate = rospy.Rate(1/self.sample_period)
        self.linear_vel = 1.0
        self.angular_vel = self.linear_vel / self.turning_radius
        self.remaining_time = 0   # within [0, sample_period]
        self.smooth_start_update_interval = 0.03  # second
        self.turning_clockwise = False
        self.turning_clockwise_buffer = False

        # send velocity commands
        self.handpoint_offset = 0.2    # meter
        self.controller_tune_K = 0.3
        self.vel = Twist()
        self.vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # odometry feedback
        self.logging_counter = 0
        self.trajectory = list()
        self.trajectory_hp = list()
        self.trajectory_cmd = list()
        self.pose = Pose2D()
        self.pose_hp = Pose2D()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        # perception    # TODO: probabilistic update
        self.obstacle_list_candidate = dict()
        self.obstacle_threshold = rospy.get_param("/hdcp_planner/obstacle_threshold", 100)
        self.valid_sensing_range = self.hexmap.radius * 6
        self.obstacle_sub = rospy.Subscriber("raw_obstacles", Obstacles, self.obstacle_callback)


    def straight_line_trajectory_planning(self, start_point, end_point):
        direction_vector = end_point - start_point
        angle = direction_vector.angle
        total_distance = abs(direction_vector)
        local_time = self.remaining_time # remaining time is within [0, self.smaple_period]
        while not rospy.is_shutdown():
            current_distance = self.linear_vel * local_time
            x_ref = start_point[0] + current_distance * cos(angle)
            y_ref = start_point[1] + current_distance * sin(angle)
            vx_ref = self.linear_vel * cos(angle)
            vy_ref = self.linear_vel * sin(angle)
            self.tracking_controller(x_ref, y_ref, vx_ref, vy_ref)
            local_time += self.sample_period
            remaining_distance = total_distance - self.linear_vel * local_time
            if remaining_distance < 0:
                self.remaining_time = - remaining_distance/self.linear_vel
                break
        if self.remaining_time > self.sample_period or self.remaining_time < 0:
            rospy.logwarn("line: remaining_time = " + str(self.remaining_time))
            rospy.loginfo("local_time = " + str(local_time))
            rospy.loginfo("start_point = " + str(start_point) + "; end_point = " + str(end_point))
            rospy.loginfo("total_distance = " + str(total_distance) + "; remaining_distance" + str(remaining_distance))


    def circle_trajectory_planning(self, start_point, end_point, center):
        start_angle = atan2(start_point[1]-center[1], start_point[0]-center[0])
        angle_difference = self.get_angle_difference(start_point, end_point, center)
        local_time = self.remaining_time # remaining time is within [0, self.smaple_period]
        while not rospy.is_shutdown():
            if self.turning_clockwise:
                current_angle = start_angle - self.angular_vel * local_time   # from x to x_dot: take derivative wrt local_time
                x_ref = center[0] + self.turning_radius * cos(current_angle)  # x = cx + r*cos(a-vt)
                y_ref = center[1] + self.turning_radius * sin(current_angle)  # y = cy + r*sin(a-vt)
                vx_ref = self.linear_vel * sin(current_angle)                 # x_dot = rv*sin(a-vt)    # lin_vel = r*ang_vel
                vy_ref = - self.linear_vel * cos(current_angle)               # y_dot = -rv*cos(a-vt)
            else:
                current_angle = start_angle + self.angular_vel * local_time   # from x to x_dot: take derivative wrt local_time
                x_ref = center[0] + self.turning_radius * cos(current_angle)  # x = cx + r*cos(a+vt)
                y_ref = center[1] + self.turning_radius * sin(current_angle)  # y = cy + r*sin(a+vt)
                vx_ref = - self.linear_vel * sin(current_angle)               # x_dot = -rv*sin(a+vt)    # lin_vel = r*ang_vel
                vy_ref = self.linear_vel * cos(current_angle)                 # y_dot =  rv*cos(a+vt)
            self.tracking_controller(x_ref, y_ref, vx_ref, vy_ref)
            local_time += self.sample_period
            remaining_angle = angle_difference - self.angular_vel * local_time
            if remaining_angle < 0:
                self.remaining_time = - remaining_angle/self.angular_vel
                break
        if self.remaining_time > self.sample_period or self.remaining_time < 0:
            rospy.logwarn("circle: remaining_time = " + str(self.remaining_time))
            rospy.loginfo("local_time = " + str(local_time) + "; center = " + str(center))
            rospy.loginfo("start_point = " + str(start_point) + "; end_point = " + str(end_point))
            rospy.loginfo("angle_difference = " + str(angle_difference) + "; remaining_angle" + str(remaining_angle))


    def initial_circle_trajectory_planning(self):
        angle_difference = 2*pi
        factor = 0
        current_angle = 0
        while not rospy.is_shutdown():
            factor = factor + self.smooth_start_update_interval
            linear_vel = self.linear_vel * min(factor, 1)
            angular_vel = self.angular_vel * min(factor, 1)
            current_angle = current_angle + angular_vel * self.sample_period
            x_ref = self.turning_radius * cos(current_angle)
            y_ref = self.turning_radius * sin(current_angle)
            vx_ref = - linear_vel * sin(current_angle)
            vy_ref = linear_vel * cos(current_angle)
            self.tracking_controller(x_ref, y_ref, vx_ref, vy_ref)
            remaining_angle = angle_difference - current_angle
            if remaining_angle < 0:
                self.remaining_time = - remaining_angle/self.angular_vel
                break


    def get_angle_difference(self, start_point, end_point, center):
        # compute CCW angle difference between two points on circumference
        start_angle = atan2(start_point[1]-center[1], start_point[0]-center[0])
        end_angle = atan2(end_point[1]-center[1], end_point[0]-center[0])
        angle_difference = end_angle - start_angle
        if angle_difference <= 0: # make sure value is within (0, 2pi]
            angle_difference += 2*pi
        # switch to CW angle difference if needed
        if self.turning_clockwise and angle_difference != 2*pi:    # 2*pi means we will turn full circle
            angle_difference = 2*pi - angle_difference
        return angle_difference


    def find_tangent_points(self, current_hex, next_hex, init_point):
        current_hex_center = self.hexmap.cube_to_cat(current_hex)
        outer_start, outer_end = self.outer_tangent_points(current_hex, next_hex)
        inner_start, inner_end = self.inner_tangent_points(current_hex, next_hex)
        outer_diff = self.get_angle_difference(init_point, outer_start, current_hex_center)
        inner_diff = self.get_angle_difference(init_point, inner_start, current_hex_center)
        if inner_diff < outer_diff:
            self.turning_clockwise_buffer = not self.turning_clockwise
            return inner_start, inner_end
        else:
            self.turning_clockwise_buffer = self.turning_clockwise
            return outer_start, outer_end


    def inner_tangent_points(self, current_hex, target_hex):
        current_center = self.hexmap.cube_to_cat(current_hex)
        target_center = self.hexmap.cube_to_cat(target_hex)
        ai = current_center[0]
        bi = current_center[1]
        aj = target_center[0]
        bj = target_center[1]
        w = (aj-ai)**2 + (bj-bi)**2
        rt = self.turning_radius
        rt2 = rt**2
        if self.turning_clockwise:
            xi = ai + (2*rt2*(aj-ai) - rt*(bj-bi)*sqrt(w-4*rt2))/w  # minus sign for CW
            yi = bi + (2*rt2*(bj-bi) - rt*(ai-aj)*sqrt(w-4*rt2))/w
            xj = aj + (2*rt2*(ai-aj) - rt*(bi-bj)*sqrt(w-4*rt2))/w
            yj = bj + (2*rt2*(bi-bj) - rt*(aj-ai)*sqrt(w-4*rt2))/w
        else:
            xi = ai + (2*rt2*(aj-ai) + rt*(bj-bi)*sqrt(w-4*rt2))/w  # plus sign for CCW
            yi = bi + (2*rt2*(bj-bi) + rt*(ai-aj)*sqrt(w-4*rt2))/w
            xj = aj + (2*rt2*(ai-aj) + rt*(bi-bj)*sqrt(w-4*rt2))/w
            yj = bj + (2*rt2*(bi-bj) + rt*(aj-ai)*sqrt(w-4*rt2))/w
        return Vector2D(xi, yi), Vector2D(xj, yj)  # start_point, end_point


    def outer_tangent_points(self, current_hex, target_hex):
        current_center = self.hexmap.cube_to_cat(current_hex)
        target_center = self.hexmap.cube_to_cat(target_hex)
        ai = current_center[0]
        bi = current_center[1]
        aj = target_center[0]
        bj = target_center[1]
        w = (aj-ai)**2 + (bj-bi)**2
        rt = self.turning_radius
        if self.turning_clockwise:
            xi = ai + rt*(bi-bj)/sqrt(w)   # plus sign for CW
            yi = bi + rt*(aj-ai)/sqrt(w)
            xj = aj + rt*(bi-bj)/sqrt(w)
            yj = bj + rt*(aj-ai)/sqrt(w)
        else:
            xi = ai - rt*(bi-bj)/sqrt(w)   # minus sign for CCW
            yi = bi - rt*(aj-ai)/sqrt(w)
            xj = aj - rt*(bi-bj)/sqrt(w)
            yj = bj - rt*(aj-ai)/sqrt(w)
        return Vector2D(xi, yi), Vector2D(xj, yj)  # start_point, end_point


    def tracking_controller(self, x_ref, y_ref, vx_ref, vy_ref):
        '''
          tracking controller design
          vx = xh_d_dot - K * (xh - xh_d)    => xh --> xh_d
          vy = yh_d_dot - K * (yh - yh_d)    => yh --> yh_d
        '''
        self.trajectory_cmd.append([x_ref, y_ref])
        K = self.controller_tune_K  # controller parameter
        ux = vx_ref - K * (self.pose_hp.x - x_ref)
        uy = vy_ref - K * (self.pose_hp.y - y_ref)
        vel_hp = [ux, uy]
        self.pub_vel_hp(vel_hp)


    def pub_vel_hp(self, vel_hp):
        ''' 
            matrix transform
            [ v ]    1    [ L*cos0  L*sin0 ]   [ x ]
            [   ] = --- * [                ] * [   ]
            [ w ]    L    [ -sin0    cos0  ]   [ y ]
        '''
        x = vel_hp[0]
        y = vel_hp[1]
        theta = self.pose_hp.theta

        v = x*cos(theta) + y*sin(theta)
        w = (x*(-sin(theta)) + y*cos(theta))/self.handpoint_offset
        rospy.logdebug("vel: theta=" + str(theta) + "; x=" + str(x) +\
         "; y=" + str(y) + "; v=" + str(v) + "; w=" + str(w))

        self.vel.linear.x = v
        self.vel.angular.z = w
        self.vel_pub.publish(self.vel)
        self.rate.sleep()


    def odom_callback(self, msg):
        # get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        self.pose_hp.theta = yaw
        self.pose_hp.x = self.pose.x + self.handpoint_offset * cos(yaw)
        self.pose_hp.y = self.pose.y + self.handpoint_offset * sin(yaw)

        # reduce the number of saved messages to 1/10
        self.logging_counter += 1
        if self.logging_counter == 10:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y, self.pose.theta])
            self.trajectory_hp.append([self.pose_hp.x, self.pose_hp.y, self.pose_hp.theta])
            rospy.logdebug("odom: x=" + str(self.pose.x) +\
             ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))
            rospy.logdebug("odom_hp: x_hp=" + str(self.pose_hp.x) +\
             ";  y_hp=" + str(self.pose_hp.y) + ";  theta=" + str(yaw))


    def valid_sensing(self, point):
        return abs(point - Vector2D(self.pose.x, self.pose.y)) < self.valid_sensing_range


    def obstacle_callback(self, msg):
        # sampling points on the obstacles
        points = list()

        for circle in msg.circles:
            center = Vector2D(circle.center.x, circle.center.y)
            points.append(center)
            r = circle.true_radius
            if r > 0:
                for theta in np.arange(0, 2*pi, 0.3):
                    radius = Vector2D(r*cos(theta), r*sin(theta))
                    points.append(center + radius)
        
        for segment in msg.segments:
            line = Vector2D(segment.last_point.x - segment.first_point.x, \
                            segment.last_point.y - segment.first_point.y)
            interval = 0
            while abs(line) - interval > 0:
                cx = segment.first_point.x + interval * cos(line.angle)
                cy = segment.first_point.y + interval * sin(line.angle)
                points.append(Vector2D(cx, cy))
                interval += 0.2

        for p in points:
            if self.valid_sensing(p):
                p_hex = self.hexmap.cat_to_cube(p)
                if p_hex not in self.obstacle_list_candidate:
                    self.obstacle_list_candidate[p_hex] = 1
                else:
                    self.obstacle_list_candidate[p_hex] += 1

        for candidate, times in self.obstacle_list_candidate.items():  #TODO: probabilistic update
            if times > self.obstacle_threshold and not self.hexmap.is_explored(candidate):
                self.hexmap.add_obstacle(candidate) 


    def save_and_plot_trajectory(self, directory, debugging_mode=False):
        if not isinstance(directory, str):
            raise TypeError("please specify the directory using string type")

        trajectory = np.array(self.trajectory)
        trajectory_hp = np.array(self.trajectory_hp)
        trajectory_cmd = np.array(self.trajectory_cmd)

        np.savetxt(directory + "/trajectory.csv", trajectory, fmt='%f', delimiter=',')
        np.savetxt(directory + "/trajectory_hp.csv", trajectory_hp, fmt='%f', delimiter=',')
        np.savetxt(directory + "/trajectory_cmd.csv", trajectory_cmd, fmt='%f', delimiter=',')

        if debugging_mode:
            plt.plot(trajectory[:, 0], trajectory[:, 1])
            plt.plot(trajectory_hp[:, 0], trajectory_hp[:, 1])
            plt.plot(trajectory_cmd[:, 0], trajectory_cmd[:, 1])
        else:
            plt.plot(trajectory[:, 0], trajectory[:, 1])


    @staticmethod
    def load_and_plot_trajectory(directory, debugging_mode=False):
        if not isinstance(directory, str):
            raise TypeError("please specify the directory using string type")

        if debugging_mode:
            trajectory = np.loadtxt(directory + "/trajectory.csv", delimiter=',')
            trajectory_hp = np.loadtxt(directory + "/trajectory_hp.csv", delimiter=',')
            trajectory_cmd = np.loadtxt(directory + "/trajectory_cmd.csv", delimiter=',')
            plt.plot(trajectory[:, 0], trajectory[:, 1])
            plt.plot(trajectory_hp[:, 0], trajectory_hp[:, 1])
            plt.plot(trajectory_cmd[:, 0], trajectory_cmd[:, 1])
        else:
            trajectory = np.loadtxt(directory + "/trajectory.csv", delimiter=',')
            plt.plot(trajectory[:, 0], trajectory[:, 1])
