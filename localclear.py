#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from costmap_2d.srv import ClearCostmap

def clear_local_costmap_service(req):
    try:
        rospy.wait_for_service('/local_costmap/clear')
        clear_local_service = rospy.ServiceProxy('/local_costmap/clear', ClearCostmap)
        clear_local_service()
        rospy.loginfo("Local costmap cleared successfully.")
        return EmptyResponse()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return EmptyResponse()

def clear_costmaps_timer(event):
    clear_local_costmap_service(EmptyRequest())  # เรียกใช้งาน service เคลียร์ costmaps

def main():
    rospy.init_node('clear_local_costmap_server')
    rospy.Service('/clear_local_costmap', Empty, clear_local_costmap_service)
    rospy.loginfo("Clear local costmap service is ready.")

    # สร้าง Timer เพื่อเรียกใช้งานฟังก์ชันเคลียร์ costmaps ทุก ๆ 2 วินาที
    rospy.Timer(rospy.Duration(2), clear_costmaps_timer)

    rospy.spin()

if __name__ == "__main__":
    main()
#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from costmap_2d.srv import ClearCostmap

def clear_local_costmap_service(req):
    try:
        rospy.wait_for_service('/local_costmap/clear')
        clear_local_service = rospy.ServiceProxy('/local_costmap/clear', ClearCostmap)
        clear_local_service()
        rospy.loginfo("Local costmap cleared successfully.")
        return EmptyResponse()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return EmptyResponse()

def clear_costmaps_timer(event):
    clear_local_costmap_service(EmptyRequest())  # เรียกใช้งาน service เคลียร์ costmaps

def main():
    rospy.init_node('clear_local_costmap_server')
    rospy.Service('/clear_local_costmap', Empty, clear_local_costmap_service)
    rospy.loginfo("Clear local costmap service is ready.")

    # สร้าง Timer เพื่อเรียกใช้งานฟังก์ชันเคลียร์ costmaps ทุก ๆ 2 วินาที
    rospy.Timer(rospy.Duration(2), clear_costmaps_timer)

    rospy.spin()

if __name__ == "__main__":
    main()

