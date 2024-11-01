import numpy as np
from collections import deque

class CellDecomposition:
    def __init__(self, grid):
        self.grid = np.array(grid)
        self.rows, self.cols = self.grid.shape
        self.cells = []  # Menyimpan cell bebas rintangan
        self.cell_graph = {}  # Menyimpan koneksi antar cell

    def decompose(self):
        # Lakukan decomposisi cell bebas di grid
        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid[row][col] == 0:  # Cell kosong
                    self.cells.append((row, col))
                    self.cell_graph[(row, col)] = []  # Tambahkan cell ke graf

        # Membangun koneksi antara cell-cell yang bebas
        for cell in self.cells:
            row, col = cell
            neighbors = [(row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)]
            for n_row, n_col in neighbors:
                if (n_row, n_col) in self.cell_graph:
                    self.cell_graph[cell].append((n_row, n_col))

    def find_path(self, start, goal):
        # Lakukan BFS untuk mencari jalur dari start ke goal
        queue = deque([start])
        came_from = {start: None}

        while queue:
            current = queue.popleft()

            if current == goal:
                break

            for neighbor in self.cell_graph.get(current, []):
                if neighbor not in came_from:
                    queue.append(neighbor)
                    came_from[neighbor] = current

        # Rekonstruksi jalur
        path = []
        node = goal
        while node:
            path.append(node)
            node = came_from.get(node)
        path.reverse()

        return path if path[0] == start else None

# Contoh penggunaan
# 0 = cell bebas, 1 = rintangan
grid = [
    [0, 0, 1, 0, 0],
    [0, 1, 1, 0, 0],
    [0, 0, 0, 0, 0],
    [1, 0, 1, 1, 0],
    [0, 0, 0, 0, 0]
]

cd = CellDecomposition(grid)
cd.decompose()
start, goal = (0, 0), (4, 4)
path = cd.find_path(start, goal)

if path:
    print("Jalur ditemukan:", path)
else:
    print("Tidak ada jalur yang tersedia.")
