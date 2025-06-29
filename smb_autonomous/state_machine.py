import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # declare termination topic param.
        termination_topic = '/exploration_finish'

        # state variable
        self.termination_received = False
        self.starting_pose = None

        # subscribe termination topic + robot pose
        self.create_subscription(Bool, termination_topic, self.tare_callback, 10)
        self.create_subscription(Odometry, '/state_estimation', self.pose_callback, 10)

        # pub.back to org.
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)

        # serv.term.
        self.object_detect_cli = self.create_client(Trigger, '/save_processed_detections_csv')
        self.save_map_cli = self.create_client(Trigger, '/open3d/save_map')

        self.get_logger().info(f'listening to termination: {termination_topic}')

    def pose_callback(self, msg):
        if self.starting_pose is None:
            point_stamped = PointStamped()
            point_stamped.header = msg.header
            point_stamped.point = msg.pose.pose.position
            self.starting_pose = point_stamped
            self.get_logger().info(
                f'save org. point: position=({point_stamped.point.x:.2f}, {point_stamped.point.y:.2f}, {point_stamped.point.z:.2f})'
            )

    def tare_callback(self, msg):
        if msg.data and not self.termination_received:
            self.get_logger().info('get termination info.doitnow')
            self.termination_received = True
            self.trigger_actions()

    def trigger_actions(self):
        if self.starting_pose is not None:
            self.waypoint_pub.publish(self.starting_pose)
            self.get_logger().info(' published waypoint ')

        if self.object_detect_cli.service_is_ready():
            future = self.object_detect_cli.call_async(Trigger.Request())
            future.add_done_callback(self.handle_save_csv_response)
            self.get_logger().info('Request to save CSV has been sent')
        else:
            self.get_logger().warn('save_detections_csv Service not ready!')

        if self.save_map_cli.service_is_ready():
            self.save_map_cli.call_async(Trigger.Request())
            self.get_logger().info('Request to save map has been sent')
    
    def handle_save_csv_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.get_logger().error(f'Failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Exception when calling save_detections_csv: {e}')

def main():
    rclpy.init()
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()