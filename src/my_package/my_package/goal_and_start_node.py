import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point


class GoalStartNode(Node):

    def __init__(self):
        super().__init__('goalstart')
        self.message = ""
        self.subscription = self.create_subscription(OccupancyGrid,'maze_solve/map',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.goal_publisher = self.create_publisher(Point, 'maze_solve/goal', 10)
        self.start_publisher = self.create_publisher(Point, 'maze_solve/start', 10)

    def listener_callback(self, map: OccupancyGrid):
        
        self.start = Point()
        self.goal = Point()

        self.start.x = 1.0
        self.start.y = 1.0

        self.goal.x = 8.0
        self.goal.y = 6.0

    
        try:
            if(map.data[int((self.start.y * map.info.width) + self.start.x)] != 0):
                raise Exception("Starting Node is an occupied space")
            
            if(map.data[int((self.goal.y * map.info.width) + self.goal.x)] != 0):
                raise Exception("Goal is an occupied space")
            
        except IndexError:
            raise Exception("Start or Goal is Out of bounds")


        self.get_logger().info('Publishing start: "%s"' % [self.start.x,self.start.y])
        self.start_publisher.publish(self.start)


        self.get_logger().info('Publishing goal: "%s"' % [self.goal.x,self.goal.y])
        self.goal_publisher.publish(self.goal)






def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GoalStartNode()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()