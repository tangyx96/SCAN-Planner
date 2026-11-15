#!/usr/bin/env python3

import argparse
import math
import os
import select
import sys
import termios
import tty

try:
    import rospy
    from nav_msgs.msg import Odometry
except ImportError as exc:
    print("Failed to import ROS Python modules: {}".format(exc), file=sys.stderr)
    print("Source your ROS workspace first, for example:", file=sys.stderr)
    print("  source devel/setup.bash", file=sys.stderr)
    sys.exit(2)


# 默认将记录结果保存到当前脚本所在目录下的 keypoint.yaml。
DEFAULT_OUTPUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keypoint.yaml")


def strip_ros_args(argv):
    # 移除 ROS 启动时自动注入的参数，仅保留脚本自身的命令行参数。
    return [arg for arg in argv if ":=" not in arg and not arg.startswith("__")]


def format_float(value):
    # 将浮点数格式化为紧凑的 YAML 文本，去除无意义的末尾零。
    text = "{:.6f}".format(float(value)).rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def atomic_write(path, content):
    # 通过临时文件和原子替换写入内容，避免中断时生成不完整配置文件。
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)

    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        handle.write(content)
    os.replace(tmp_path, path)


class KeypointRecorder:
    # 订阅里程计并通过键盘维护三维导航关键点列表。

    def __init__(self, args):
        # 初始化输入话题、输出路径、关键点缓存和里程计订阅器。
        self.odom_topic = args.odom
        self.output_path = DEFAULT_OUTPUT
        self.latest_odom = None
        self.waypoints = []

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

    def odom_callback(self, msg):
        # 保存最新里程计消息，供键盘操作触发时读取当前位置。
        self.latest_odom = msg

    def current_point(self):
        # 提取最新里程计中的位置，并过滤尚未收到或包含非有限值的坐标。
        if self.latest_odom is None:
            return None

        position = self.latest_odom.pose.pose.position
        point = (float(position.x), float(position.y), float(position.z))
        return point if all(math.isfinite(value) for value in point) else None

    def record_current(self):
        # 将当前机器人位置追加到关键点序列末尾。
        point = self.current_point()
        if point is None:
            rospy.logwarn("No valid odometry yet; cannot record waypoint.")
            return

        self.waypoints.append(point)
        rospy.loginfo(
            "Recorded waypoint %d: [%.3f, %.3f, %.3f]",
            len(self.waypoints),
            point[0],
            point[1],
            point[2],
        )

    def replace_current(self, index):
        # 用当前机器人位置替换指定编号的关键点；编号从 1 开始。
        if index < 1 or index > len(self.waypoints):
            rospy.logwarn("Invalid waypoint index %d. Valid range: 1-%d.", index, len(self.waypoints))
            return

        point = self.current_point()
        if point is None:
            rospy.logwarn("No valid odometry yet; cannot replace waypoint.")
            return

        self.waypoints[index - 1] = point
        rospy.loginfo(
            "Replaced waypoint %d: [%.3f, %.3f, %.3f]",
            index,
            point[0],
            point[1],
            point[2],
        )

    def delete_waypoint(self, index):
        # 删除指定编号的关键点，并保持其余关键点的相对顺序。
        if index < 1 or index > len(self.waypoints):
            rospy.logwarn("Invalid waypoint index %d. Valid range: 1-%d.", index, len(self.waypoints))
            return

        point = self.waypoints.pop(index - 1)
        rospy.loginfo(
            "Deleted waypoint %d: [%.3f, %.3f, %.3f]",
            index,
            point[0],
            point[1],
            point[2],
        )

    def undo_last(self):
        # 删除最近一次记录的关键点，相当于撤销最后一次追加操作。
        if not self.waypoints:
            rospy.logwarn("No waypoint to undo.")
            return

        point = self.waypoints.pop()
        rospy.loginfo(
            "Removed waypoint %d: [%.3f, %.3f, %.3f]",
            len(self.waypoints) + 1,
            point[0],
            point[1],
            point[2],
        )

    def list_waypoints(self):
        # 按记录顺序输出当前关键点列表及其三维坐标。
        if not self.waypoints:
            rospy.loginfo("No recorded waypoints.")
            return

        rospy.loginfo("Recorded waypoints:")
        for index, point in enumerate(self.waypoints, 1):
            rospy.loginfo("  %02d: [%.3f, %.3f, %.3f]", index, point[0], point[1], point[2])

    def build_yaml(self):
        # 将内存中的关键点转换为 SCAN-Planner 约定的 ROS 参数 YAML 格式。
        lines = ["fsm:", "  waypoint_num: {}".format(len(self.waypoints))]
        for index, point in enumerate(self.waypoints):
            lines.append("  waypoint{}_x: {}".format(index, format_float(point[0])))
            lines.append("  waypoint{}_y: {}".format(index, format_float(point[1])))
            lines.append("  waypoint{}_z: {}".format(index, format_float(point[2])))
        return "\n".join(lines) + "\n"

    def save(self):
        # 生成 YAML 文本并写入默认配置文件。
        atomic_write(self.output_path, self.build_yaml())
        rospy.loginfo("Saved %d waypoint(s) to %s", len(self.waypoints), self.output_path)

    def print_help(self):
        # 打印里程计话题、输出文件和键盘命令说明。
        print("")
        print("keypoint_recorder")
        print("  odom topic : {}".format(self.odom_topic))
        print("  output     : {}".format(self.output_path))
        print("")
        print("Keyboard commands:")
        print("  Enter / Space / a : record current odom position")
        print("  r                 : replace an existing waypoint")
        print("  d                 : delete an existing waypoint")
        print("  u                 : undo last waypoint")
        print("  l                 : list waypoints")
        print("  s                 : save YAML")
        print("  h / ?             : show this help")
        print("  q                 : save and quit")
        print("")

    def prompt_index(self, old_settings, action):
        # 临时恢复规范终端输入，读取用户指定的关键点编号后恢复即时输入模式。
        if not self.waypoints:
            rospy.logwarn("No waypoint to %s.", action)
            return None

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        try:
            text = input("{} waypoint index (1-{}): ".format(action.capitalize(), len(self.waypoints))).strip()
            if not text:
                print("{} cancelled.".format(action.capitalize()))
                return None
            return int(text)
        except ValueError:
            rospy.logwarn("Invalid waypoint index: %s", text)
            return None
        finally:
            tty.setcbreak(sys.stdin.fileno())

    def run(self):
        # 运行键盘事件循环，并以 20 Hz 维持 ROS 回调处理和终端轮询。
        self.print_help()
        rospy.loginfo("Waiting for odometry on %s ...", self.odom_topic)

        if not sys.stdin.isatty():
            rospy.logerr("stdin is not a TTY; keyboard control is required.")
            return

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        rate = rospy.Rate(20)

        try:
            while not rospy.is_shutdown():
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch in ("\n", "\r", " ", "a", "A"):
                        self.record_current()
                    elif ch in ("r", "R"):
                        index = self.prompt_index(old_settings, "replace")
                        if index is not None:
                            self.replace_current(index)
                    elif ch in ("d", "D"):
                        index = self.prompt_index(old_settings, "delete")
                        if index is not None:
                            self.delete_waypoint(index)
                    elif ch in ("u", "U"):
                        self.undo_last()
                    elif ch in ("l", "L"):
                        self.list_waypoints()
                    elif ch in ("s", "S"):
                        self.save()
                    elif ch in ("h", "H", "?"):
                        self.print_help()
                    elif ch in ("q", "Q"):
                        self.save()
                        rospy.signal_shutdown("user requested quit")
                        break
                    elif ch == "\x03":
                        rospy.signal_shutdown("keyboard interrupt")
                        break
                rate.sleep()
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            rospy.signal_shutdown("interrupted")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def build_arg_parser():
    # 创建命令行参数解析器，允许用户覆盖默认里程计话题。
    parser = argparse.ArgumentParser(description="Record odometry positions to tools/keypoint.yaml.")
    parser.add_argument("--odom", default="/LIO/odom_vehicle", help="nav_msgs/Odometry topic to record.")
    return parser


def main():
    # 解析参数、初始化 ROS 节点并启动关键点记录器。
    parser = build_arg_parser()
    args = parser.parse_args(strip_ros_args(sys.argv[1:]))

    rospy.init_node("keypoint_recorder")
    recorder = KeypointRecorder(args)
    recorder.run()


if __name__ == "__main__":
    main()
