#!/usr/bin/env python

import rospy
import numpy as np
import networkx as nx
from scipy.spatial import KDTree
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Parameter PRM
NUM_NODES = 100            # Jumlah node acak
MAX_DISTANCE = 20          # Jarak maksimum antar node
MAP_SIZE = 100             # Ukuran area peta (100x100)
START = (5, 5)             # Titik awal
GOAL = (95, 95)            # Titik tujuan

class PRM:
    def __init__(self):
        # Inisialisasi publisher untuk roadmap dan path
        self.roadmap_pub = rospy.Publisher('/prm_roadmap', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/prm_path', Marker, queue_size=10)
        self.graph = nx.Graph()
        self.nodes = []

    def sample_nodes(self):
        """Generate random nodes and add START and GOAL."""
        self.nodes = [tuple(np.random.uniform(0, MAP_SIZE, 2)) for _ in range(NUM_NODES)]
        self.nodes.append(START)
        self.nodes.append(GOAL)

    def connect_nodes(self):
        """Connect nodes within a maximum distance to form a roadmap."""
        tree = KDTree(self.nodes)
        for i, node in enumerate(self.nodes):
            distances, neighbors = tree.query(node, k=len(self.nodes))
            for j, distance in zip(neighbors[1:], distances[1:]):
                if distance <= MAX_DISTANCE:
                    self.graph.add_edge(i, j, weight=distance)

    def find_path(self):
        """Find the shortest path between START and GOAL."""
        start_idx = self.nodes.index(START)
        goal_idx = self.nodes.index(GOAL)
        try:
            return nx.shortest_path(self.graph, source=start_idx, target=goal_idx, weight='weight')
        except nx.NetworkXNoPath:
            return None

    def visualize_roadmap(self):
        """Visualize the roadmap as a Marker in Rviz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Width of lines
        marker.color = ColorRGBA(0.7, 0.7, 0.7, 1.0)  # Gray color

        for (i, j) in self.graph.edges:
            point1 = Point(x=self.nodes[i][0], y=self.nodes[i][1], z=0)
            point2 = Point(x=self.nodes[j][0], y=self.nodes[j][1], z=0)
            marker.points.append(point1)
            marker.points.append(point2)

        self.roadmap_pub.publish(marker)

    def visualize_path(self, path):
        """Visualize the shortest path as a Marker in Rviz."""
        if path is None:
            rospy.loginfo("No path found!")
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Width of line for path
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color

        for i in path:
            point = Point(x=self.nodes[i][0], y=self.nodes[i][1], z=0)
            marker.points.append(point)

        self.path_pub.publish(marker)

    def run(self):
        rospy.loginfo("Sampling nodes...")
        self.sample_nodes()
        rospy.loginfo("Connecting nodes to create a roadmap...")
        self.connect_nodes()
        rospy.loginfo("Finding shortest path...")
        path = self.find_path()
        
        # Publish roadmap and path
        self.visualize_roadmap()
        self.visualize_path(path)

if __name__ == "__main__":
    rospy.init_node("prm_node")
    prm = PRM()
    prm.run()
    rospy.spin()
