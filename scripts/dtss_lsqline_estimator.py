#!/usr/bin/python

import rospy
import numpy as np
import lsqline_estimator

if __name__ == "__main__":
    try:
        rospy.init_node("dtss_lsqline_node")
        node = lsqline_estimator.CentroidFinder(
                    w = np.array([[1.0], [1.0], [1.0], [1.0], # sensor mask [100] - to deselect; [0] - to select.
                    [1.0], [1.0], [1.0], [1.0]]),
                    lab = False)
    except rospy.ROSInterruptException:
        pass
