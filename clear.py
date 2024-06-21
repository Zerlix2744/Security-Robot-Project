#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def clear_costmaps():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps_service()
        rospy.loginfo("Costmaps cleared successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('navigation_with_clear_costmaps')
    
    # Example navigation task
    while not rospy.is_shutdown():
        # Perform navigation tasks here
        # ...
        
        # Clear costmaps periodically
        clear_costmaps()
        rospy.sleep(2)  # Adjust the sleep time as needed

if __name__ == "__main__":
    main()

