import numpy as np
import matplotlib.pyplot as plt
import yaml
import networkx as nx
from scipy.spatial import KDTree

# Load parameters from YAML file
with open('params.yaml', 'r') as f:
    params = yaml.safe_load(f)

NUM_NODES = params['num_nodes']
MAX_DISTANCE = params['max_distance']
MAP_SIZE = params['map_size']
START = tuple(params['start'])
GOAL = tuple(params['goal'])

class PRM:
    def __init__(self, num_nodes, max_distance, map_size):
        self.num_nodes = num_nodes
        self.max_distance = max_distance
        self.map_size = map_size
        self.nodes = []
        self.graph = nx.Graph()

    def sample_nodes(self):
        """Sample random nodes within the map boundaries."""
        self.nodes = [tuple(np.random.uniform(0, self.map_size, 2)) for _ in range(self.num_nodes)]
        self.nodes.append(START)
        self.nodes.append(GOAL)

    def connect_nodes(self):
        """Connect nodes to form a roadmap."""
        tree = KDTree(self.nodes)
        for i, node in enumerate(self.nodes):
            distances, neighbors = tree.query(node, k=self.num_nodes)
            for j, distance in zip(neighbors[1:], distances[1:]):
                if distance <= self.max_distance:
                    self.graph.add_edge(i, j, weight=distance)

    def find_path(self):
        """Find the shortest path from START to GOAL."""
        start_idx = self.nodes.index(START)
        goal_idx = self.nodes.index(GOAL)
        try:
            return nx.shortest_path(self.graph, source=start_idx, target=goal_idx, weight='weight')
        except nx.NetworkXNoPath:
            return None

    def visualize(self, path=None):
        """Visualize the roadmap and the path."""
        plt.figure(figsize=(8, 8))
        plt.xlim(0, self.map_size)
        plt.ylim(0, self.map_size)

        # Draw edges
        for (i, j) in self.graph.edges:
            node1, node2 = self.nodes[i], self.nodes[j]
            plt.plot([node1[0], node2[0]], [node1[1], node2[1]], 'gray', linewidth=0.5)

        # Draw nodes
        for node in self.nodes:
            plt.plot(node[0], node[1], 'bo', markersize=3)

        # Draw start and goal nodes
        plt.plot(START[0], START[1], 'go', markersize=8, label="Start")
        plt.plot(GOAL[0], GOAL[1], 'ro', markersize=8, label="Goal")

        # Draw path
        if path:
            path_coords = [self.nodes[i] for i in path]
            plt.plot([x[0] for x in path_coords], [x[1] for x in path_coords], 'r-', linewidth=2, label="Path")

        plt.legend()
        plt.title("Probabilistic Roadmap (PRM)")
        plt.show()

# Main function
def main():
    prm = PRM(NUM_NODES, MAX_DISTANCE, MAP_SIZE)
    prm.sample_nodes()
    prm.connect_nodes()
    path = prm.find_path()
    prm.visualize(path)

if __name__ == "__main__":
    main()
