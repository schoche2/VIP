import subprocess

def save_map():
    rospy.loginfo("Saving the map...")
    try:
        subprocess.call(["rosrun", "map_server", "map_saver", "-f", "slam_map"])
        rospy.loginfo("Map saved successfully as slam_map.pgm and slam_map.yaml")
    except Exception as e:
        rospy.logerr(f"Error saving map: {e}")

if __name__ == "__main__":
    try:
        rospy.init_node('fastslam_node')
        slam = FastSLAM()
        
        # ROS subscribers
        rospy.Subscriber('/odom', Odometry, slam.motion_update)
        rospy.Subscriber('/scan', LaserScan, slam.measurement_update)
        
        # ROS publisher for particles
        particle_pub = rospy.Publisher('/particle_poses', PoseArray, queue_size=10)
        
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            slam.resample()
            slam.publish_particles(particle_pub)
            rate.sleep()
        
        save_map()  # Automatically save the map when the node stops

    except rospy.ROSInterruptException:
        save_map()

