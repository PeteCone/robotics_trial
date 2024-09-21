import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData


class MapNode(Node):

    def __init__(self):
        super().__init__('map')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'maze_solve/map', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.map_publisher)
        self.i = 0

    def map_publisher(self):
        map2D =[[1,1,1,1,1,1,1,1,1,1],
                [1,0,0,1,1,1,1,0,1,1],
                [1,1,0,1,0,0,0,0,0,1],
                [1,1,0,1,1,1,0,1,1,1],
                [1,1,0,0,0,0,0,0,1,1],
                [1,1,0,0,0,1,1,0,0,1],
                [1,0,0,1,0,0,0,0,0,1],
                [1,1,1,1,1,1,1,1,1,1]]
        
        flatmap = [x for xs in map2D for x in xs] #flattens array
        self.map = OccupancyGrid()
        self.map.data = flatmap

        mapmetadata = MapMetaData()
        mapmetadata.width = 10
        mapmetadata.height = 8
        self.map.info = mapmetadata
        self.publisher_.publish(self.map)
        self.get_logger().info('Publishing: Map')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapNode()

    rclpy.spin_once(map_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    

                        #[[0,0,0,0,0,0,0,0,0,0],
                        # [0,1,1,0,0,0,0,0,0,0],
                        # [0,1,1,0,0,0,0,0,0,0],
                        # [0,1,0,0,1,1,1,1,0,0],
                        # [0,1,0,0,1,1,1,0,0,0],
                        # [0,1,1,0,1,0,0,0,0,0],
                        # [0,0,1,0,0,0,0,0,0,0],
                        # [0,0,0,0,0,0,0,0,0,0]]