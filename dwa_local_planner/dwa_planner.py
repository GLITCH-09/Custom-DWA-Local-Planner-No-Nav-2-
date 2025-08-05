import rclpy
import math
import random
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion


goal_x = None
goal_y = None
odom_data = None
scan_data = None
goal_reached = False


def ask_for_goal():
    global goal_x, goal_y, goal_reached
    goal_x = float(input("Enter your desired co-ordinates below:\nGoal X: "))
    goal_y = float(input("Goal Y: "))
    goal_reached = False


def odom_callback(msg):
    global odom_data
    odom_data = msg


def scan_callback(msg):
    global scan_data
    scan_data = msg


def predict_motion(speed, turn_rate, step_time, num_steps=200):
    if odom_data is None:
        return []
    x = odom_data.pose.pose.position.x
    y = odom_data.pose.pose.position.y
    _, _, yaw = euler_from_quaternion(
        [odom_data.pose.pose.orientation.x,
         odom_data.pose.pose.orientation.y,
         odom_data.pose.pose.orientation.z,
         odom_data.pose.pose.orientation.w])
    path = []
    for _ in range(num_steps):
        yaw += turn_rate * step_time
        x += speed * math.cos(yaw) * step_time
        y += speed * math.sin(yaw) * step_time
        path.append((x, y))
    return path


def check_for_collisions(path):
    if scan_data is None or odom_data is None or not path:
        return -float('inf')
    px = odom_data.pose.pose.position.x
    py = odom_data.pose.pose.position.y
    _, _, yaw = euler_from_quaternion(
        [odom_data.pose.pose.orientation.x,
         odom_data.pose.pose.orientation.y,
         odom_data.pose.pose.orientation.z,
         odom_data.pose.pose.orientation.w])
    cos_y = math.cos(-yaw)
    sin_y = math.sin(-yaw)
    footprint = 0.55       # 0.18 m radius + buffer
    beams = len(scan_data.ranges)
    for gx, gy in path[::3]:
        dx, dy = gx - px, gy - py
        rel_x = dx * cos_y - dy * sin_y
        rel_y = dx * sin_y + dy * cos_y
        d = math.hypot(rel_x, rel_y)
        if d < 0.05:
            continue
        ang = math.atan2(rel_y, rel_x)
        idx = int((ang + math.pi) * beams / (2 * math.pi))
        idx = max(0, min(beams - 1, idx))
        scan_d = scan_data.ranges[idx]
        if 0.10 < scan_d < float('inf') and d > scan_d - footprint:
            return -10000
    return 0


def choose_best_path(node, paths):
    global goal_x, goal_y, goal_reached
    if odom_data is None:
        return 0.0, 0.0
    cx = odom_data.pose.pose.position.x
    cy = odom_data.pose.pose.position.y
    _, _, yaw = euler_from_quaternion(
        [odom_data.pose.pose.orientation.x,
         odom_data.pose.pose.orientation.y,
         odom_data.pose.pose.orientation.z,
         odom_data.pose.pose.orientation.w])
    if math.hypot(goal_x - cx, goal_y - cy) < 0.15:
        if not goal_reached:
            goal_reached = True
            node.get_logger().info("Goal reached!")
        return 0.0, 0.0
    best_score = float('-inf')
    best_speed, best_turn = 0.0, 0.0
    for speed, turn, path in paths:
        if not path:
            continue
        gx, gy = path[-1]
        g_score = -math.hypot(gx - goal_x, gy - goal_y) * 15
        desired = math.atan2(goal_y - cx, goal_x - cx)
        h_score = -abs(math.atan2(math.sin(desired - yaw),
                                  math.cos(desired - yaw))) * 4
        c_score = check_for_collisions(path)
        s_score = speed * 10
        smooth = -abs(turn) * 1.5
        total = g_score + h_score + c_score + s_score + smooth
        if total > best_score:
            best_score = total
            best_speed, best_turn = speed, turn
    return best_speed, best_turn


def generate_paths(max_speed, max_turn, step_time, paths=300, steps=200):
    out = [(0.0, 0.0, predict_motion(0.0, 0.0, step_time, steps))]
    for i in range(paths):
        spd = random.uniform(0.02, max_speed)
        trn = random.uniform(-max_turn, max_turn)
        out.append((spd, trn, predict_motion(spd, trn, step_time, steps)))
    return out


def movement_loop(node, pub, m_pub, max_speed, max_turn, step, steps):
    global goal_reached
    if odom_data is None or scan_data is None or goal_reached:
        return
    paths = generate_paths(max_speed, max_turn, step, 300, steps)
    speed, turn = choose_best_path(node, paths)
    cmd = TwistStamped()
    cmd.header.stamp = node.get_clock().now().to_msg()
    cmd.header.frame_id = 'base_link'
    cmd.twist.linear.x = max(0.0, min(max_speed, speed))
    cmd.twist.angular.z = max(-max_turn, min(max_turn, turn))
    pub.publish(cmd)
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = Marker.LINE_STRIP
    marker.scale.x = 0.02
    marker.color.g = 1.0
    marker.color.a = 0.7
    for _, _, pth in paths[:5]:
        for x, y in pth[::3]:
            pt = Point()
            pt.x, pt.y = x, y
            marker.points.append(pt)
    m_pub.publish(marker)


def main():
    rclpy.init()
    node = Node('dwa_planner')
    ask_for_goal()
    node.create_subscription(Odometry, '/odom', odom_callback, 10)
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)
    cmd_pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)
    m_pub = node.create_publisher(Marker, '/visual_paths', 10)
    max_speed = 0.35
    max_turn = 2.0
    step_time = 0.05      # 20 Hz
    num_steps = 200       # 10 s horizon
    node.create_timer(step_time,
                      lambda: movement_loop(node, cmd_pub, m_pub,
                                            max_speed, max_turn,
                                            step_time, num_steps))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
