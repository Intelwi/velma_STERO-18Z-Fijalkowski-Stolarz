#roslaunch velma_common octomap_offline_server.launch octomap_file:=/home/gfijalko/ws_velma/src/stero_velma/octomaps/octomap2.bt
# Uwzględnianie mapy
velma = VelmaInterface()
velma.waitForInit()
p = Planner(velma.maxJointTrajLen())
p.waitForInit():
oml = OctomapListener("/octomap_binary")
rospy.sleep(1.0)
octomap = oml.getOctomap(timeout_s=5.0)
p.processWorld(octomap)