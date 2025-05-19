import rospy
from arx_x7_controller.srv import Pose3D, Pose3DRequest
from arx_x7_controller.srv import SinglePose, SinglePoseRequest
import numpy as np

#bimanual
def send_bimanual_pose(left_pose, right_pose):
    rospy.wait_for_service('/send_bimanual_pose')
    try:
        service_proxy = rospy.ServiceProxy('/send_bimanual_pose', Pose3D)
        left_xyz = left_pose[:3, 3].tolist()
        right_xyz = right_pose[:3, 3].tolist()
        req = Pose3DRequest(left_xyz=left_xyz, right_xyz=right_xyz)
        resp = service_proxy(req)
        if resp.success:
            rospy.loginfo("Bimanual poses sent successfully.")
        else:
            rospy.logwarn("Service call failed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        
def send_bimanual_pose_loop(left_pose, right_pose, num):
    rospy.wait_for_service('/send_pose_loop')
    try:
        service_proxy = rospy.ServiceProxy('/send_pose_loop', Pose_loop)
        left_xyz = left_pose[:3, 3].tolist()
        right_xyz = right_pose[:3, 3].tolist()
        num = num
        req = Pose_loopRequest(left_xyz=left_xyz, right_xyz=right_xyz, num=num)
        resp = service_proxy(req)
        if resp.success:
            rospy.loginfo("Bimanual poses loop sent successfully.")
        else:
            rospy.logwarn("Service call failed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

# task3_container_close
def send_bimanual_close_pose(left_pose, right_pose):
    rospy.wait_for_service('/send_bimanual_close_pose')
    try:
        service_proxy = rospy.ServiceProxy('/send_bimanual_close_pose', Pose3D)
        left_xyz = left_pose[:3, 3].tolist()
        right_xyz = right_pose[:3, 3].tolist()
        req = Pose3DRequest(left_xyz=left_xyz, right_xyz=right_xyz)
        resp = service_proxy(req)
        if resp.success:
            rospy.loginfo("Bimanual close poses sent successfully.")
        else:
            rospy.logwarn("Service call failed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

#single
def send_single_pose(single_pose, is_left):
    rospy.wait_for_service('/send_single_pose')
    try:
        service_proxy = rospy.ServiceProxy('/send_single_pose', SinglePose)
        single_xyz = single_pose[:3, 3].tolist()
        req = SinglePoseRequest(single_xyz=single_xyz, is_left=is_left)
        resp = service_proxy(req)
        if resp.success:
            rospy.loginfo(f"{'LEFT' if is_left else 'RIGHT'} Grasp pose sent successfully.")
        else:
            rospy.logwarn("Service call failed.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def send_moved_single_pose(single_pose, is_left):
    rospy.wait_for_service('/send_moved_single_pose')
    try:
        service_proxy = rospy.ServiceProxy('/send_moved_single_pose', SinglePose)
        single_xyz = single_pose[:3, 3].tolist()
        req = SinglePoseRequest(single_xyz=single_xyz, is_left=is_left)
        resp = service_proxy(req)
        if resp.success:
            rospy.loginfo(f"{'LEFT' if is_left else 'RIGHT'} Grasp pose sent successfully.")
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
