import rospy
from arx_x7_controller.srv import Pose3D, Pose3DRequest
from arx_x7_controller.srv import LeftPose, LeftPoseRequest
from arx_x7_controller.srv import RightPose, RightPoseRequest
import numpy as np

#bimanual
def send_grasp_pose(left_pose, right_pose):
    rospy.wait_for_service('/send_grasp_pose')
    try:
        service_proxy = rospy.ServiceProxy('/send_grasp_pose', Pose3D)
        left_xyz = left_pose[:3, 3].tolist()
        right_xyz = right_pose[:3, 3].tolist()
        req = Pose3DRequest(left_xyz=left_xyz, right_xyz=right_xyz)
        resp = service_proxy(req)
        if resp.success:
            rospy.loginfo("Grasp poses sent successfully.")
        else:
            rospy.logwarn("Service call failed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

#left
def send_left_pose(left_pose):
    rospy.wait_for_service('/send_left_pose')
    try:
        service_proxy = rospy.ServiceProxy('/send_left_pose', LeftPose)
        left_xyz = left_pose[:3, 3].tolist()
        req = LeftPoseRequest(left_xyz=left_xyz)
        resp = service_proxy(req)
        if resp.success:
            rospy.loginfo("LEFT Grasp poses sent successfully.")
        else:
            rospy.logwarn("Service call failed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

#right
def send_right_pose(right_pose):
    rospy.wait_for_service('/send_right_pose')
    try:
        service_proxy = rospy.ServiceProxy('/send_right_pose', RightPose)
        right_xyz = right_pose[:3, 3].tolist()
        req = RightPoseRequest(right_xyz=right_xyz)
        resp = service_proxy(req)
        if resp.success:
            rospy.loginfo("RIGHT Grasp poses sent successfully.")
        else:
            rospy.logwarn("Service call failed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

# 예시
#if __name__ == "__main__":
#    import open3d as o3d
#    from util import transform_matrix
#    rospy.init_node("pose_sender")

#    left_pose = transform_matrix(np.array([0.1, 0.2, 0.3]))
#    right_pose = transform_matrix(np.array([0.4, 0.5, 0.6]))
#    send_grasp_pose(left_pose)