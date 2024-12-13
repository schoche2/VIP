import numpy as np
from scipy.stats import multivariate_normal
import rospy
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf

class FastSLAM:
    def __init__(self, num_particles=100, landmark_threshold=0.5):
        self.num_particles = num_particles
        self.landmark_threshold = landmark_threshold

        # Initialize particles: each particle has a pose and an empty map (landmarks)
        self.particles = [{'pose': np.array([0.0, 0.0, 0.0]), 'landmarks': {}, 'weight': 1.0} for _ in range(num_particles)]
        self.prev_odom = None  # To store the previous odometry data for calculating deltas

    def motion_update(self, odom):
        if self.prev_odom is None:
            self.prev_odom = odom
            return

        # Calculate the change in position and orientation
        delta_x = odom.pose.pose.position.x - self.prev_odom.pose.pose.position.x
        delta_y = odom.pose.pose.position.y - self.prev_odom.pose.pose.position.y

        prev_yaw = tf.transformations.euler_from_quaternion([
            self.prev_odom.pose.pose.orientation.x,
            self.prev_odom.pose.pose.orientation.y,
            self.prev_odom.pose.pose.orientation.z,
            self.prev_odom.pose.pose.orientation.w
        ])[2]

        curr_yaw = tf.transformations.euler_from_quaternion([
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        ])[2]

        delta_theta = curr_yaw - prev_yaw

        # Update each particle with motion and small noise
        for particle in self.particles:
            noise = np.random.normal(0, 0.01, 3)  # Small noise for x, y, and theta
            particle['pose'] += np.array([delta_x, delta_y, delta_theta]) + noise

        self.prev_odom = odom

    def measurement_update(self, scan):
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        for particle in self.particles:
            for i, r in enumerate(ranges):
                if np.isinf(r) or np.isnan(r) or r > 3.5:  # Filter out invalid or too-distant readings
                    continue

                # Calculate the landmark's position in the world frame
                lx = particle['pose'][0] + r * np.cos(particle['pose'][2] + angles[i])
                ly = particle['pose'][1] + r * np.sin(particle['pose'][2] + angles[i])
                landmark = np.array([lx, ly])

                # Data Association with Mahalanobis Distance
                matched = False
                for lm_id, lm in particle['landmarks'].items():
                    mean = lm['mean']
                    cov = lm['cov']
                    mahalanobis_dist = np.sqrt((landmark - mean).T @ np.linalg.inv(cov) @ (landmark - mean))
                    if mahalanobis_dist < self.landmark_threshold:
                        # Update landmark with Kalman Filter
                        K = cov @ np.linalg.inv(cov + np.eye(2) * 0.01)
                        lm['mean'] += K @ (landmark - mean)
                        lm['cov'] = (np.eye(2) - K) @ cov
                        matched = True
                        break

                # If no match, add new landmark
                if not matched:
                    particle['landmarks'][len(particle['landmarks'])] = {'mean': landmark, 'cov': np.eye(2) * 0.1}

    def resample(self):
        weights = np.array([p['weight'] for p in self.particles])
        weights /= np.sum(weights)  # Normalize weights to sum to 1

        # Systematic resampling
        positions = (np.arange(self.num_particles) + np.random.random()) / self.num_particles
        cumulative_sum = np.cumsum(weights)
        i, j = 0, 0
        new_particles = []
        while i < self.num_particles:
            if positions[i] < cumulative_sum[j]:
                new_particles.append(self.particles[j].copy())
                i += 1
            else:
                j += 1

        self.particles = new_particles

    def get_best_particle(self):
        # Return the particle with the highest weight
        return max(self.particles, key=lambda p: p['weight'])

    def publish_particles(self, pub):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for particle in self.particles:
            pose = Pose()
            pose.position.x = particle['pose'][0]
            pose.position.y = particle['pose'][1]
            q = tf.transformations.quaternion_from_euler(0, 0, particle['pose'][2])
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            pose_array.poses.append(pose)

        pub.publish(pose_array)

