import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import math
import matplotlib.pyplot as plt


plot_graph = True

node_visited_color = 'yellow'
node_shortestpath_color = 'red'
edge_visted_color = 'blue'
edge_shortedpath_color = 'red'

class Edge:
    def __init__(self, start, end, weight):
        self.start = start
        self.end = end
        self.weight = weight

class Tile:
    def __init__(self, id, x, y, edges, heuristic, parent):
        self.id = id
        self.x = x
        self.y = y
        self.edges = edges
        self.distance = float("inf")
        self.heuristic = heuristic
        self.parent = parent

class PathNode(Node):

    def __init__(self):
        super().__init__('path')
        self.message = ""
        self.map = None
        self.start = None
        self.goal = None

        self.map_subscription = self.create_subscription(OccupancyGrid,'maze_solve/map', self.map_listener_callback, 10)
        self.start_subscription = self.create_subscription(Point,'maze_solve/start', self.start_listener_callback, 10)
        self.goal_subscription = self.create_subscription(Point,'maze_solve/goal', self.goal_listener_callback, 10)
        self.publisher = self.create_publisher(String, 'maze_solve/path', 10)

    def map_listener_callback(self, grid: OccupancyGrid):
        self.get_logger().info('Map: Recieved')
        self.grid = grid
        self.map = [] #grid.data[i*grid.info.width:(i+1)*grid.info.width]
        for i in range(0,grid.info.height):
            self.map.append(grid.data[i*grid.info.width:(i+1)*grid.info.width])

        self.shortest_path()

    def start_listener_callback(self, start: Point):
        self.get_logger().info('Start: Recieved')
        self.start = start
        self.shortest_path()
        
    def goal_listener_callback(self, goal: Point):
        self.get_logger().info('Goal: Recieved')
        self.goal = goal
        self.shortest_path()


    def shortest_path(self):
        # Make sure we have all of the nessecary data
        self.get_logger().info('shortest_path called')
        if(self.map is None or self.start is None or self.goal is None):
            return
        
        self.get_logger().info('A* Begins')

        self.width = self.grid.info.width
        self.height = self.grid.info.height

        self.start_id = int((self.start.y * self.width) + self.start.x+ 1)
        self.goal_id = int((self.goal.y * self.width) + self.goal.x+ 1)

        edge_list = self.generate_edges()

        self.get_logger().info('Edges Created')

        self.create_tiles(edge_list)

        self.get_logger().info('Tiles Created')

        ax, fig = self.create_graph(edge_list, self.tiles)

        [shortestpath_ids, shortestpath_weights] = self.a_star(self.tiles, self.start_id, self.goal_id, ax, fig)

        self.get_logger().info("Shortest path:" + str(shortestpath_ids))
        self.get_logger().info("Path Weights: " + str(shortestpath_weights))

    
    def generate_edges(self):

        edges_list = []

        for i in range(1,self.height-2): # -1 for indexing and -1 because the edges of the maze is all occupied
            for j in range(1,self.width-2):
                if(self.map[i][j] != 0):
                    continue
                
                #These comment directions are refrencing how the maze will look in a 2D array in python,
                #when the maze is plotted it may be oriented differently

                #Up
                if(self.map[i-1][j] == 0):
                    edges_list.append(Edge((i*self.width) + j+ 1, ((i-1)*self.width)+j + 1, 1))

                #Up Right
                if(self.map[i-1][j] == 0 and self.map[i][j+1] == 0 and self.map[i-1][j+1] == 0):
                    edges_list.append(Edge((i*self.width) + j+ 1, ((i-1)*self.width)+j+1+ 1, 1.414))

                #Right
                if(self.map[i][j+1] == 0):
                    edges_list.append(Edge((i*self.width) + j+ 1, ((i)*self.width)+j+1+ 1, 1))

                #Down Right
                if(self.map[i][j+1] == 0 and self.map[i+1][j] == 0 and self.map[i+1][j+1] == 0):
                    edges_list.append(Edge((i*self.width) + j+ 1, ((i+1)*self.width)+j+1+ 1, 1.414))

                #Down
                if(self.map[i+1][j] == 0):
                    edges_list.append(Edge((i*self.width) + j+ 1, ((i+1)*self.width)+j+ 1, 1))

                #Left Down
                if(self.map[i+1][j] == 0 and self.map[i][j-1] == 0 and self.map[i-1][j-1] == 0):
                    edges_list.append(Edge((i*self.width) + j+ 1, ((i-1)*self.width)+j-1+ 1, 1.414))

                #Left
                if(self.map[i][j-1] == 0):
                    edges_list.append(Edge((i*self.width) + j+ 1, ((i)*self.width)+j-1+ 1, 1))

                #Up Left
                if(self.map[i-1][j] == 0 and self.map[i][j-1] == 0 and self.map[i-1][j-1] == 0):
                    edges_list.append(Edge((i*self.width) + j+ 1, ((i-1)*self.width)+j-1+ 1, 1.414))

        self.get_logger().info("Length: {}, Edge 1: start: {}, end{}, weight{}"
                                .format(len(edges_list), edges_list[0].start, edges_list[0].end, edges_list[0].weight))
        return edges_list
        
    # create Tiles for each empty element in the array
    def create_tiles(self, edges: Edge):
                
        self.tiles = []
        for i in range(self.height):
            for j in range(self.width):
                edge_list = []
                for edge in edges: #this for loop (searching through all edges for every node) could be optimized
                    if edge.start == (i*self.width)+j+ 1:
                        edge_list.append(edge) 
                heuristic = math.isqrt((j - int(self.goal.x))**2 + (i - int(self.goal.y))**2)
                tile = Tile((i*self.width)+j+ 1, i, j, edge_list, heuristic, None) 
                self.tiles.append(tile)
        self.get_logger().info("Length: {}, Edge 1: x: {}, y: {}, heuristic{}"
                               .format(len(self.tiles), self.tiles[0].x, self.tiles[0].y, self.tiles[0].heuristic))
        return self.tiles


    #Create Graph
    def create_graph(self, edges, tiles):
        fig, ax = plt.subplots(figsize=(5, 5))

        tiles = self.plot_tiles(ax, tiles)
        self.plot_edges(ax, edges)

        if(plot_graph):
            plt.show(block=False)

        return ax, fig

    #Plotting Nodes
    def plot_tiles(self, ax, nodes):
        for n in nodes:
            ax.plot(n.x, n.y, 'o', color='black', zorder=2)
            
        
        # Mark the End node and start node with respective colors

        ax.plot(nodes[self.goal_id-1].x, nodes[self.goal_id-1].y, 'o', color=node_shortestpath_color, zorder=4)
        ax.plot(nodes[self.start_id-1].x, nodes[self.start_id-1].y, 'o', color=node_visited_color, zorder=4)
        
        return nodes

        #Plotting Edges
    def plot_edges(self, ax, edges):

        for edge in edges:
            x_coords = [(edge.start -1) % self.width, (edge.end -1) % self.width]
            y_coords = [(edge.start -1) / self.width, (edge.end -1) / self.width]
            (ax.plot(x_coords, y_coords, color="black", linewidth=2, zorder=1))


    # A* Algorithm implemented
    def a_star(self, nodes, start, end, ax, fig):

        # Create open set and add the start node
        open = nodes[:]
        open[start-1].distance = 0
        open[start-1].parent = None
        ax.plot(nodes[start-1].x, nodes[start-1].y, 'o', color=node_visited_color, zorder= 4)

        # interate to every open node
        while open:

            # assign the node with the lowest distance to current
            current = min(open, key=lambda node: node.distance + node.heuristic) 

            # minimum node from open is the end node, then we have found the shortest path for end n
            if(current.id == end):
                break

            ax.plot(current.x, current.y, 'o', color=node_visited_color, zorder= 4)

            # 
            for edge in current.edges:
                neighbour = nodes[edge.end-1]
                new_distance = current.distance + edge.weight

                x_coords = nodes[edge.start-1].x, nodes[edge.end-1].x
                y_coords = nodes[edge.start-1].y, nodes[edge.end-1].y

                ax.plot(x_coords, y_coords, color=edge_visted_color, linewidth=2, zorder= 3)

                if new_distance < neighbour.distance:
                    neighbour.distance = new_distance
                    neighbour.parent = current

                if(plot_graph):
                    plt.pause(.01)
                    plt.draw()

            open.remove(current) 

        backtrack = nodes[end-1]

        if(plot_graph):
            plt.pause(.01)
            plt.draw()

        shortestpath_ids = []
        shortestpath_weights = []

        shortestpath_ids.insert(0, backtrack.id)
        shortestpath_weights.insert(0, round(backtrack.distance, 2))

        ax.plot(backtrack.x, backtrack.y, 'o', color=node_shortestpath_color, zorder= 4)

        while backtrack.parent is not None:
            backtrack = backtrack.parent

            shortestpath_ids.insert(0, backtrack.id)
            shortestpath_weights.insert(0, round(backtrack.distance,2))

            x_coords = nodes[shortestpath_ids[0]-1].x, nodes[shortestpath_ids[1]-1].x
            y_coords = nodes[shortestpath_ids[0]-1].y, nodes[shortestpath_ids[1]-1].y
            ax.plot(x_coords, y_coords, color=node_shortestpath_color, linewidth=2, zorder= 3)

            ax.plot(backtrack.x, backtrack.y, 'o', color=node_shortestpath_color, zorder= 4)

            if(plot_graph):
                plt.pause(.7)
                plt.draw()

        return [shortestpath_ids, shortestpath_weights]


def main(args=None):
    rclpy.init(args=args)

    path_node = PathNode()

    rclpy.spin(path_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()