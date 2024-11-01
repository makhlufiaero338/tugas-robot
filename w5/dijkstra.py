import heapq

def dijkstra(graph, start, goal):
    # Inisialisasi jarak dari start ke setiap node dengan infinity, kecuali start itu sendiri (0)
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    # Inisialisasi priority queue dan tambahkan node awal
    priority_queue = [(0, start)]
    # Inisialisasi dictionary untuk melacak jalur
    previous_nodes = {node: None for node in graph}
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        # Jika node saat ini adalah tujuan, kita sudah selesai
        if current_node == goal:
            break
        
        # Memproses tetangga dari node saat ini
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))
    
    # Rekonstruksi jalur dari goal ke start menggunakan previous_nodes
    path, node = [], goal
    while node:
        path.append(node)
        node = previous_nodes[node]
    path.reverse()  # Membalik jalur agar dari start ke goal
    
    return path, distances[goal]  # Mengembalikan jalur dan biaya total

# Contoh penggunaan graf
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}

# Mencari jalur terpendek dari 'A' ke 'D'
path, cost = dijkstra(graph, 'A', 'D')
print("Jalur terpendek:", path)
print("Biaya total:", cost)
