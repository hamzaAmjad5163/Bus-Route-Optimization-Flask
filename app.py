from flask import Flask, render_template, request

app = Flask(__name__,static_url_path='/static')

# Define the bus stops and their coordinates
bus_stops = {
    'A': (1, 2),
    'B': (4, 5),
    'C': (6, 8),
    'D': (9, 10),
    'E': (3, 2),
    'F': (20, 8),
    'G': (11, 5),
    'H': (15, 4),
    'I': (12, 13),
}

# Define the graph of bus stop connections
bus_connections = {
    'A': ['B', 'C', 'E'],
    'B': ['A', 'D'],
    'C': ['A', 'D'],
    'D': ['B', 'C', 'I'],
    'E': ['A', 'G', 'F'],
    'F': ['E', 'G', 'H'],
    'G': ['F', 'E', 'I'],
    'H': ['F', 'I'],
    'I': ['D', 'G', 'H'],
   
}

# Function to calculate the Euclidean distance between two points
def distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

# Function to perform the A* algorithm
def astar(start, goal):
    # Calculate the heuristic function (Euclidean distance) for each bus stop
    h = {bus_stop: distance(bus_stops[bus_stop], bus_stops[goal]) for bus_stop in bus_stops}

    # Initialize the open and closed sets
    open_set = {start}
    closed_set = set()

    # Initialize the cost and came_from dictionaries
    g = {bus_stop: float('inf') for bus_stop in bus_stops}
    g[start] = 0
    came_from = {}

    while open_set:
        # Find the bus stop in the open set with the lowest f value
        current = min(open_set, key=lambda bus_stop: g[bus_stop] + h[bus_stop])

        if current == goal:
            # Reconstruct the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        open_set.remove(current)
        closed_set.add(current)

        for neighbor in bus_connections[current]:
            if neighbor in closed_set:
                continue

            # Calculate the tentative g value
            tentative_g = g[current] + distance(bus_stops[current], bus_stops[neighbor])

            if tentative_g < g[neighbor]:
                # Update the g value and the came_from dictionary
                g[neighbor] = tentative_g
                came_from[neighbor] = current

                if neighbor not in open_set:
                    open_set.add(neighbor)

    return None

# ...remaining code...


@app.route('/')
def index():
    return render_template('index.html', bus_stops=bus_stops)


@app.route('/shortest_path', methods=['POST'])
def shortest_path():
    start = request.form['start']
    goal = request.form['goal']

    # Find the shortest path using A*
    shortest_path = astar(start, goal)

    # Render the results on the graph
    return render_template('index.html', bus_stops=bus_stops, shortest_path=shortest_path)


if __name__ == '__main__':
    app.run(debug=True)
