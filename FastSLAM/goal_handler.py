import json
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

def load_goal_from_json(file_path):
    with open(file_path, 'r') as f:
        goal_data = json.load(f)
    return goal_data['x'], goal_data['y']

def send_goal(x, y):
    rospy.init_node('goal_sender', anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Goal reached successfully!")

if __name__ == '__main__':
    goal_file = 'goal.json'
