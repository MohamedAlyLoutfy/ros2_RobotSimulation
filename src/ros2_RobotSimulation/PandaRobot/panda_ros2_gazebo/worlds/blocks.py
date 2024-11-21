import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os

def main(args=None):
    rclpy.init(args=args)

    node = Node('spawn_blocks_node')
    client = node.create_client(SpawnEntity, '/spawn_entity')
    client.wait_for_service()
    request = SpawnEntity.Request()
    request.xml = """
    <sdf version="1.6">
      <model name="block1">
        <pose>0.8 0 0.025 0 0 0</pose>  <!-- Position of the block -->
        <static>false</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>0.05 0.05 0.05</size>  <!-- 5cm block -->
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>0.05 0.05 0.05</size>
              </box>
            </geometry>
            <material>
              <ambient>1 0 0 1</ambient>  <!-- Red color -->
            </material>
          </visual>
          <inertial>
            <mass>0.1</mass>
            <inertia>
              <ixx>0.0001</ixx>
              <iyy>0.0001</iyy>
              <izz>0.0001</izz>
            </inertia>
          </inertial>
        </link>
      </model>
    </sdf>
    """
    request.name = "block1"
    request.robot_namespace = ""
    request.reference_frame = "world"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Block spawned successfully!')
        os.system("ros2 run ros2_actions add_block_to_scene")
        
    else:
        node.get_logger().error('Failed to spawn block.')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
