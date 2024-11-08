#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math

# Parameter RRT
MAX_NODES = 500             # Jumlah maksimum node dalam tree
STEP_SIZE = 5.0             # Panjang langkah tiap ekspansi
MAP_SIZE = 100              # Ukuran area peta (100x100)
START = (5, 5)              # Titik awal
GOAL = (95, 95)             # Titik tujuan
GOAL_THRESHOLD = 5.0        # Jarak maksimum untuk mencapai tujuan

class RRT:
    def __init__(self):
        # Inisialisasi publisher untuk tree dan jalur
        self.tree_pub = rospy.Publisher('/rrt_tree', Marker, queue_size=10)
        self.path_pub = rospy.Publisher('/rrt_path', Marker, queue_size=10)
        
        self.nodes = [START]  # Tree diinisialisasi dengan titik awal
        self.parent = {START: None}  # Menyimpan parent dari setiap node

    def distance(self, p1, p2):
        """Menghitung jarak Euclidean antara dua titik."""
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def sample_random_point(self):
        """Sampel titik acak dalam batas peta."""
        return (np.random.uniform(0, MAP_SIZE), np.random.uniform(0, MAP_SIZE))

    def get_nearest_node(self, point):
        """Temukan node terdekat di tree terhadap titik tertentu."""
        nearest_node = min(self.nodes, key=lambda node: self.distance(node, point))
        return nearest_node

    def steer(self, from_node, to_point):
        """Ekspansi dari from_node menuju to_point dengan langkah STEP_SIZE."""
        angle = math.atan2(to_point[1] - from_node[1], to_point[0] - from_node[0])
        new_x = from_node[0] + STEP_SIZE * math.cos(angle)
        new_y = from_node[1] + STEP_SIZE * math.sin(angle)
        return (new_x, new_y)

    def reached_goal(self, node):
        """Periksa apakah node telah mencapai tujuan."""
        return self.distance(node, GOAL) < GOAL_THRESHOLD

    def build_tree(self):
        """Bangun tree sampai mencapai tujuan atau jumlah node maksimum."""
        for _ in range(MAX_NODES):
            rand_point = self.sample_random_point()
            nearest_node = self.get_nearest_node(rand_point)
            new_node = self.steer(nearest_node, rand_point)
            
            # Tambah node baru ke tree jika tidak di luar batas
            if 0 <= new_node[0] <= MAP_SIZE and 0 <= new_node[1] <= MAP_SIZE:
                self.nodes.append(new_node)
                self.parent[new_node] = nearest_node

                # Visualisasi tree di Rviz
                self.visualize_tree(nearest_node, new_node)

                # Periksa apakah node mencapai tujuan
                if self.reached_goal(new_node):
                    rospy.loginfo("Goal reached!")
                    return self.get_path(new_node)
        rospy.logwarn("Goal not reached, maximum nodes added.")
        return None

    def get_path(self, node):
        """Temukan path dari tujuan ke awal dengan mengikuti parent setiap node."""
        path = []
        while node is not None:
            path.append(node)
            node = self.parent[node]
        path.reverse()
        return path

    def visualize_tree(self, node1, node2):
        """Visualisasikan edge dalam tree sebagai Marker LINE_LIST di Rviz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.7)  # Hijau

        point1 = Point(x=node1[0], y=node1[1], z=0)
        point2 = Point(x=node2[0], y=node2[1], z=0)
        marker.points.extend([point1, point2])

        self.tree_pub.publish(marker)

    def visualize_path(self, path):
        """Visualisasikan jalur terpendek ke tujuan sebagai Marker LINE_STRIP di Rviz."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Merah

        for node in path:
            point = Point(x=node[0], y=node[1], z=0)
            marker.points.append(point)

        self.path_pub.publish(marker)

    def run(self):
        rospy.loginfo("Building RRT...")
        path = self.build_tree()
        if path:
            self.visualize_path(path)
        else:
            rospy.logwarn("Path not found!")

if __name__ == "__main__":
    rospy.init_node("rrt_node")
    rrt = RRT()
    rrt.run()
    rospy.spin()
