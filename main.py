import random
import matplotlib.pyplot as plt
import networkx as nx
import heapq
import time


def generate_grid_graph(rows, cols, obstacle_ratio=0.2):
    """
    Genera un grafo de cuadrícula con obstáculos.

    Args:
        rows (int): Número de filas de la cuadrícula.
        cols (int): Número de columnas de la cuadrícula.
        obstacle_ratio (float): Proporción de celdas que serán obstáculos.

    Returns:
        G (networkx.Graph): Grafo de la cuadrícula.
        obstacles (set): Conjunto de obstáculos.
    """
    G = nx.grid_2d_graph(rows, cols)
    obstacles = set()
    num_obstacles = int(rows * cols * obstacle_ratio)

    while len(obstacles) < num_obstacles:
        obstacle = (random.randint(0, rows - 1), random.randint(0, cols - 1))
        if obstacle in G:
            obstacles.add(obstacle)
            G.remove_node(obstacle)

    return G, obstacles


def heuristic(a, b):
    """
    Calcula la heurística de Manhattan entre dos puntos.

    Args:
        a (tuple): Primer punto (fila, columna).
        b (tuple): Segundo punto (fila, columna).

    Returns:
        int: Distancia de Manhattan entre los puntos.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search(G, start, goal):
    """
    Realiza la búsqueda A* en el grafo.

    Args:
        G (networkx.Graph): Grafo en el que se realiza la búsqueda.
        start (tuple): Nodo de inicio (fila, columna).
        goal (tuple): Nodo objetivo (fila, columna).

    Returns:
        list: Camino desde el inicio hasta el objetivo.
    """
    queue = []
    heapq.heappush(queue, (0, start))  # Añadir el nodo de inicio a la cola de prioridad
    came_from = {start: None}
    cost_so_far = {start: 0}

    while queue:
        _, current = heapq.heappop(queue)  # Obtener el nodo con la menor prioridad

        if current == goal:
            break

        for neighbor in G.neighbors(current):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(queue, (priority, neighbor))
                came_from[neighbor] = current

    return reconstruct_path(came_from, start, goal)


def reconstruct_path(came_from, start, goal):
    """
    Reconstruye el camino desde el inicio hasta el objetivo.

    Args:
        came_from (dict): Diccionario de nodos y sus predecesores.
        start (tuple): Nodo de inicio (fila, columna).
        goal (tuple): Nodo objetivo (fila, columna).

    Returns:
        list: Camino desde el inicio hasta el objetivo.
    """
    path = []
    current = goal
    while current != start:
        if current is None:
            return []
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path


def draw_grid_graph(G, rows, cols, obstacles, path, start, goal):
    """
    Dibuja el grafo de la cuadrícula con obstáculos y el camino encontrado.

    Args:
        G (networkx.Graph): Grafo de la cuadrícula.
        rows (int): Número de filas de la cuadrícula.
        cols (int): Número de columnas de la cuadrícula.
        obstacles (set): Conjunto de obstáculos.
        path (list): Camino desde el inicio hasta el objetivo.
        start (tuple): Nodo de inicio (fila, columna).
        goal (tuple): Nodo objetivo (fila, columna).
    """
    plt.figure(figsize=(6, 6))
    pos = {node: (node[1], -node[0]) for node in G.nodes()}
    path_whitout_start_goal = path[1:-1]

    nx.draw(
        G,
        pos,
        node_size=200,
        node_color="lightblue",
        with_labels=False,
        edge_color="gray",
    )

    for obs in obstacles:
        plt.scatter(obs[1], -obs[0], color="red", s=500, marker="s")

    if path:
        path_edges = list(zip(path, path[1:]))

        nx.draw_networkx_nodes(
            G,
            pos,
            nodelist=path_whitout_start_goal,
            node_color="#ffd800",
            node_size=200,
        )
        nx.draw_networkx_edges(
            G, pos, edgelist=path_edges, edge_color="#ffd800", width=2
        )

    nx.draw_networkx_nodes(
        G,
        pos,
        nodelist=[start],
        node_color="none",
        edgecolors="green",
        node_size=500,
        linewidths=2,
    )

    nx.draw_networkx_nodes(
        G,
        pos,
        nodelist=[goal],
        node_color="none",
        edgecolors="purple",
        node_size=500,
        linewidths=2,
    )

    plt.scatter(start[1], -start[0], color="green", s=400, marker="x", label="Inicio")
    plt.scatter(goal[1], -goal[0], color="purple", s=400, marker="x", label="Objetivo")
    plt.legend()
    plt.xlim(-0.5, cols - 0.5)
    plt.ylim(-rows + 0.5, 0.5)
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    rows, cols = 10, 10
    G, obstacles = generate_grid_graph(rows, cols)

    start, goal = (0, 0), (rows - 1, cols - 1)
    while start in obstacles or goal in obstacles:
        start = (random.randint(0, rows - 1), random.randint(0, cols - 1))
        goal = (random.randint(0, rows - 1), random.randint(0, cols - 1))

    # Medición del tiempo de ejecución de la búsqueda A*
    start_time = time.time()
    path = a_star_search(G, start, goal)
    end_time = time.time()

    print(f"\n\nTiempo de ejecución: {end_time - start_time:.5f} segundos")

    # Ruta encontrada
    print("\n\nRuta encontrada:", path)

    # Costo de la ruta encontrada
    print("\n\nCosto de la ruta:", len(path) - 1)

    draw_grid_graph(G, rows, cols, obstacles, path, start, goal)
