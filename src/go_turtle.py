#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
from geometry_msgs.msg import Twist
import geometry_msgs.msg
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions
from geometry_msgs.msg import Quaternion
from hsrb_tf_service.srv import target_tf_service,target_tf_serviceResponse
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import smach_ros
import smach
from mqtt_interface_sub import MQTT_SUB
from mqtt_interface_pub import MQTT_PUB
from position_data import PositionData as PD

def read_target(tf_listener):
    (trans, rot) = tf_listener.lookupTransform("map", "target", rospy.Time(0))
    rospy.loginfo(trans)
    rospy.loginfo(rot)
    tx = trans.pop(0)
    ty = trans.pop(0)
    tz = trans.pop(0)

    rx = rot.pop(0)
    ry = rot.pop(0)
    rz = rot.pop(0)
    rw = rot.pop(0)

    data = [tx, ty, tz, rx, ry, rz, rw]
    return data

def set_goal():
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = "map"
    goal_pose.target_pose.pose.position.x = 0.01
    goal_pose.target_pose.pose.position.y = 0
    goal_pose.target_pose.pose.position.z = 0
    goal_pose.target_pose.pose.orientation.x = 0
    goal_pose.target_pose.pose.orientation.y = 0
    goal_pose.target_pose.pose.orientation.z = 0
    goal_pose.target_pose.pose.orientation.w = 1
    return goal_pose

class WAIT_MQTT(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=["sub"],output_keys=["msgs"])
        self.wait_msg_sub = MQTT_SUB()
        self.msg_list = None
        self.flag = True
        # sys.argv[1] よりロボットごとのトピック名に変更
        self.wait_msg_sub.sub_run(broker_ip="localhost",topic_name="robot_sub"+sys.argv[1],cb=self.callback)

    def callback(self,msg):
        rospy.loginfo("aaaa callback")
        self.msg_list = msg.split("/")
        self.flag = False
        rospy.loginfo(self.flag)


    def execute(self, ud):
        while self.flag:
            pass
        rospy.loginfo("while 抜けた")
        # userdataよりデータの受け渡し
        rospy.loginfo(self.msg_list)
        ud.msgs = self.msg_list
        self.flag = True
        return "sub"

# req/go/destination
# req/go/temp_posi
class ACTION(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=["finish"],input_keys=["msgs"])
        self.goal_data = MoveBaseGoal()
        self.goal_data.target_pose.header.frame_id = "map"
        self.action_client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        self.action_client.wait_for_server()


    def execute(self, ud):
        data = PD.dist_position[ud.msgs[2]]
        # self.goal_dataの値を埋める
        self.goal_data.target_pose.pose.position.x = data[0]
        self.goal_data.target_pose.pose.position.y = data[1]
        self.goal_data.target_pose.pose.position.z = data[2]
        self.goal_data.target_pose.pose.orientation.x = data[3]
        self.goal_data.target_pose.pose.orientation.y = data[4]
        self.goal_data.target_pose.pose.orientation.z = data[5]
        self.goal_data.target_pose.pose.orientation.w = data[6]

        # actionlib 実行
        self.action_client.send_goal(self.goal_data)

        # wait result
        self.action_client.wait_for_result()

        return "finish"

class STATUS_UPDATE(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=["finish"],input_keys=["msgs"])
        self.status_update_pub = MQTT_PUB()
        self.status_update_pub.pub_con(broker_ip="localhost",topic_name="status_update",pubmsg="Y"+sys.argv[1])

    def execute(self, ud):
        rospy.loginfo("status update class")
        rospy.loginfo(ud.msgs)
        self.status_update_pub.pubmsg_setter(f"status_update/{ud.msgs[2]}/"+sys.argv[1])
        self.status_update_pub.pub_run()
        return "finish"

if __name__ =="__main__":
    rospy.init_node("aaa",anonymous=True)
    sm = smach.StateMachine(outcomes="END")

    sm = smach.StateMachine(outcomes=["END"])
    sm.userdata.sm_msgs = None
    
    with sm:
        smach.StateMachine.add('WAIT_MQTT',WAIT_MQTT(),transitions={"sub":"ACTION"},remapping={"msgs":"sm_msgs"})
        smach.StateMachine.add("ACTION",ACTION(),transitions={"finish":"STATUS_UPDATE"},remapping={"msgs":"sm_msgs"})
        smach.StateMachine.add("STATUS_UPDATE",STATUS_UPDATE(),transitions={"finish":"WAIT_MQTT"},remapping={"msgs":"sm_msgs"})

    sis = smach_ros.IntrospectionServer("server_name", sm, "/START")

    sis.start()
    # rospy.sleep(1)
    result = sm.execute()
    rospy.loginfo("finished!!!")
    # client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    # client.wait_for_server()
    # goal = set_goal()
    # client.send_goal(goal)
    # client.wait_for_result()