#!/usr/bin/env python
import sys
import threading
import rospy
from marble_virtual_comms.srv import CreatePeer


class CommsHandler:
    """ Setup comms between two agents using the DARPA comms simulator """

    def __init__(self, source, dest):
        self.subData = None

        self.source = source
        self.dest = dest

        # Setup peer-connections topics
        service = '/' + source + '/marble_virtual_comms/create_peer'
        print("Waiting for services {}".format(service))

        rospy.wait_for_service(service)
        print("Calling service {}".format(service))
        rospy.ServiceProxy(service, CreatePeer).call(dest)


class CommsRun(threading.Thread):
    def __init__(self, source, dest):
        self.comm = CommsHandler(source, dest)
        threading.Thread.__init__(self)


if __name__ == '__main__':
    # Run as: python create_peers.py X1 X2 X3 X4
    # Will create peers X2, X3 and X4 on X1
    if len(sys.argv) < 3:
        print("Node requires at least 2 agent names")
        exit()

    # Start a connection between each agent in the list given
    i = 1
    agent1 = sys.argv[1]
    for agent2 in sys.argv[i + 1:]:
        CommsRun(agent1, agent2)
        i += 1

    rospy.init_node('CommsHandler', anonymous=True)
