#!/usr/bin/env python3

import roslaunch
import rospy

package = 'map_server'
executable = 'map_saver'

path = "~/catkin_ws/src/acs6121_team26/maps/"

node = roslaunch.core.Node(package, executable, 'path')

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
process.stop()