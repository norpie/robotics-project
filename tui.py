import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from nav2_msgs.srv import SaveMap
from nav_msgs.msg import Odometry
from turtlebot3_mapper.turtlebot3_mission_client.turtlebot3_mission_client import Turtlebot3MissionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
import threading
import signal
import time
import os
import signal

import pytermgui as ptg

PALETTE_LIGHT = "#FCBA03"
PALETTE_MID = "#8C6701"
PALETTE_DARK = "#4D4940"
PALETTE_DARKER = "#242321"

# Define your ROS2 node and TUI functions here
lock = threading.Lock()

shutdown_requested = False

node = None

manager = None
window = None

maps = []
waypoints = []
map_name = ""

expecting_remove = False

pose = None
navigator_status = "Idle"
explorer_status = "Idle"
explorer_end = False
explorer_end_status = ""
frontiers = 0

def navigator_status_callback(msg):
    global navigator_status
    navigator_status = msg.data

def explorer_status_callback(msg):
    global explorer_status
    explorer_status = msg.data

def complete_goal_callback(msg):
    if expecting_remove:
        _main_menu(window, manager)

def explorer_end_callback(msg):
    global explorer_end
    global explorer_end_status
    explorer_end_status = msg.data
    explorer_end = True
    if expecting_remove:
        _main_menu(window, manager)

def odom_callback(msg):
    global pose
    pose = msg.pose.pose

def load_maps():
    # Maps reside in the ./maps/directory, we only need to load the names

    if os.path.isdir("./maps") is False:
        os.mkdir("./maps")

    found_maps = os.listdir("./maps")

    for map in found_maps:
        maps.append(map)


def load_waypoints():
    if map_name == "":
        return

    waypoints.clear()

    if not os.path.isdir("./maps/" + map_name + "/waypoints"):
        os.mkdir("./maps/" + map_name + "/waypoints")
        return

    waypoint_files = os.listdir("./maps/" + map_name + "/waypoints")

    for waypoint_file in waypoint_files:
        if waypoint_file.endswith(".txt") is False:
            continue
        # Waypoint looks like this:
        with open("./maps/" + map_name + "/waypoints/" + waypoint_file, "r") as f:
            name = waypoint_file[:-4]
            x = float(f.readline())
            y = float(f.readline())
            o_z = float(f.readline())
            o_w = float(f.readline())

            waypoint = {
                "name": name,
                "x": x,
                "y": y,
                "o_z": o_z,
                "o_w": o_w,
            }

            waypoints.append(waypoint)

def _create_aliases() -> None:
    """Creates all the TIM aliases used by the application.

    Aliases should generally follow the following format:

        namespace.item

    For example, the title color of an app named "myapp" could be something like:

        myapp.title
    """

    ptg.tim.alias("app.text", "#cfc7b0")

    ptg.tim.alias("app.header", f"bold @{PALETTE_MID} #d9d2bd")
    ptg.tim.alias("app.header.fill", f"@{PALETTE_LIGHT}")

    ptg.tim.alias("app.title", f"bold {PALETTE_LIGHT}")
    ptg.tim.alias("app.button.label", f"bold @{PALETTE_DARK} app.text")
    ptg.tim.alias("app.button.highlight", "inverse app.button.label")

    ptg.tim.alias("app.footer", f"@{PALETTE_DARKER}")


def _configure_widgets() -> None:
    """Defines all the global widget configurations.

    Some example lines you could use here:

        ptg.boxes.DOUBLE.set_chars_of(ptg.Window)
        ptg.Splitter.set_char("separator", " ")
        ptg.Button.styles.label = "myapp.button.label"
        ptg.Container.styles.border__corner = "myapp.border"
    """

    ptg.boxes.DOUBLE.set_chars_of(ptg.Window)
    ptg.boxes.ROUNDED.set_chars_of(ptg.Container)

    ptg.Button.styles.label = "app.button.label"
    ptg.Button.styles.highlight = "app.button.highlight"

    ptg.Slider.styles.filled__cursor = PALETTE_MID
    ptg.Slider.styles.filled_selected = PALETTE_LIGHT

    ptg.Label.styles.value = "app.text"

    ptg.Window.styles.border__corner = "#C2B280"
    ptg.Container.styles.border__corner = PALETTE_DARK

    ptg.Splitter.set_char("separator", "")


def _define_layout() -> ptg.Layout:
    """Defines the application layout.

    Layouts work based on "slots" within them. Each slot can be given dimensions for
    both width and height. Integer values are interpreted to mean a static width, float
    values will be used to "scale" the relevant terminal dimension, and giving nothing
    will allow PTG to calculate the corrent dimension.
    """

    layout = ptg.Layout()

    # A header slot with a height of 1
    layout.add_slot("Header", height=1)
    layout.add_break()

    # A body slot that will fill the entire width, and the height is remaining
    layout.add_slot("Body")

    layout.add_break()

    # A footer with a static height of 1
    layout.add_slot("Footer", height=1)

    return layout


def _confirm_quit(manager: ptg.WindowManager) -> None:
    """Creates an "Are you sure you want to quit" modal window"""

    def quit_app():
        """Quits the application."""

        manager.stop()
        global shutdown_requested
        shutdown_requested = True

    modal = ptg.Window(
        "[app.title]Are you sure you want to quit?",
        "",
        ptg.Container(
            ptg.Splitter(
                ptg.Button("Yes", lambda *_: quit_app()),
                ptg.Button("No", lambda *_: modal.close()),
            ),
        ),
    ).center()

    modal.select(1)
    manager.add(modal)


def _create_new_map(window: ptg.Window, manager: ptg.WindowManager) -> None:
    manager.remove(window)

    window = ptg.Window(
        "Enter a name for the new map",
        "",
        "This will overwrite any existing map with the same name"
        "",
        ptg.Container(
            ptg.Label("Name"),
            ptg.InputField(""),
            ptg.Button("Create", lambda *_: save_new_map(window, manager)),
        ),
        ""
    ).center()

    manager.add(window)


def save_new_waypoint(window: ptg.Window, manager: ptg.WindowManager):
    name = ""
    for widget in window:
        if isinstance(widget, ptg.Container):
            for sub_widget in widget:
                if isinstance(sub_widget, ptg.InputField):
                    name = sub_widget.value
                    break
    if name == "":
        _main_menu(window, manager)
        return

    # TODO: Get location
    x = pose.position.x
    y = pose.position.y
    o_z = pose.orientation.z
    o_w = pose.orientation.w

    with open("./maps/" + map_name + "/waypoints/" + name + ".txt", "w") as f:
        f.write(str(x) + "\n")
        f.write(str(y) + "\n")
        f.write(str(o_z) + "\n")
        f.write(str(o_w) + "\n")

    _main_menu(window, manager)


def _create_new_waypoint(window: ptg.Window, manager: ptg.WindowManager) -> None:
    manager.remove(window)

    window = ptg.Window(
        "Enter a name for the new waypoint (location will be current location))",
        "",
        ptg.Container(
            ptg.Label("Name"),
            ptg.InputField(""),
            ptg.Button("Create", lambda *_: save_new_waypoint(window, manager)),
        ),
        ""
    ).center()

    manager.add(window)


def _main_menu(window: ptg.Window, manager: ptg.WindowManager) -> None:
    manager.remove(window)
    load_waypoints()

    waypoint_button_or_text = None
    if len(waypoints) == 0:
        waypoint_button_or_text = ptg.Label("No waypoints found")
    else:
        waypoint_button_or_text = ptg.Button("Waypoints", lambda *_: _waypoint_menu(window, manager))

    window = ptg.Window(
        "Main Menu - " + map_name,
        ptg.Container(
            "Main Controls",
            ptg.Button("New Waypoint", lambda *_: _create_new_waypoint(window, manager)),
            waypoint_button_or_text,
            ptg.Button("Explore Map", lambda *_: _explore_map(window, manager)),
        ),
        "",
        ptg.Container(
            ptg.Splitter(
                ptg.Button("Map List", lambda *_: _select_map(window, manager)),
                ptg.Button("Save", lambda *_: _save_map(window, manager)),
                ptg.Button("Quit", lambda *_: _confirm_quit(manager)),
            )
        ),
    ).center()

    manager.add(window)


def _save_map(window: ptg.Window, manager: ptg.WindowManager) -> None:
    req = SaveMap.Request()
    req.map_topic = "map"
    req.map_url = "./maps/" + map_name + "/map"
    req.image_format = "pgm"
    req.map_mode = "trinary"
    req.free_thresh = 0.25
    req.occupied_thresh = 0.65
    future = map_saver_client.call_async(req)
    while not future.done():
        time.sleep(0.3)
        print("Waiting...")
    _select_map(window, manager)


def _explore_map(window2: ptg.Window, manager2: ptg.WindowManager) -> None:
    manager2.remove(window2)

    window2 = ptg.Window(
        "Exploring " + map_name,
        "",
        ptg.Container(
            ptg.Splitter(
                ptg.Button("Cancel", lambda *_: _cancel_explore(window2, manager2)),
            )
        ),
    ).center()

    manager2.add(window2)

    msg = Int32()
    msg.data = 500
    explorer_publisher.publish(msg)

    global expecting_remove
    expecting_remove = True

    global manager
    global window
    manager = manager2
    window = window2

def _cancel_explore(window: ptg.Window, manager: ptg.WindowManager) -> None:
    msg = String()
    msg.data = "cancel"
    explorer_cancel_publisher.publish(msg)
    global expecting_remove
    expecting_remove = False
    _main_menu(window, manager)


def _navigate_to_waypoint(window2: ptg.Window, manager2: ptg.WindowManager, waypoint) -> None:
    manager2.remove(window2)

    window2 = ptg.Window(
        "Navigating to " + waypoint["name"],
        "",
        ptg.Container(
            ptg.Splitter(
                ptg.Button("Cancel", lambda *_: _cancel_waypoint(window2, manager2)),
            )
        ),
    ).center()

    global expecting_remove
    expecting_remove = True

    msg = String()
    msg.data = str(waypoint["x"]) + "," + str(waypoint["y"]) + "," + str(waypoint["o_z"]) + "," + str(waypoint["o_w"])
    goal_publisher.publish(msg)
    manager2.add(window2)
    global manager
    global window
    manager = manager2
    window = window2


def _cancel_waypoint(window: ptg.Window, manager: ptg.WindowManager) -> None:
    msg = String()
    msg.data = "cancel"
    cancel_publisher.publish(msg)
    global expecting_remove
    expecting_remove = False
    _main_menu(window, manager)


def _waypoint_menu(window: ptg.Window, manager: ptg.WindowManager) -> None:
    manager.remove(window)

    # Waypoint buttons
    buttons = []
    for waypoint in waypoints:
        buttons.append(ptg.Button(
            waypoint["name"],
            lambda _, param=waypoint: _navigate_to_waypoint(window, manager, param),
        ))

    if len(buttons) == 0:
        buttons.append(ptg.Label("No waypoints found"))

    window = ptg.Window(
        "Waypoint Menu",
        "",
        *buttons,
        "",
        ptg.Container(
            ptg.Splitter(
                ptg.Button("Go back", lambda *_: _main_menu(window, manager)),
            )
        ),
    ).center()

    manager.add(window)


def _set_map_name_and_main_menu(window: ptg.Window, manager: ptg.WindowManager, map: str) -> None:
    global map_name
    map_name = map
    if os.path.isfile("./maps/" + map_name + "/map.pgm") and os.path.isfile("./maps/" + map_name + "/map.yaml"):
        msg = String()
        msg.data = "./maps/" + map_name + "/map.yaml"
        map_publisher.publish(msg)
    init_pose = String()
    init_pose.data = str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.orientation.z) + "," + str(pose.orientation.w)
    init_publisher.publish(init_pose)
    _main_menu(window, manager)


def _select_map(window: ptg.Window, manager: ptg.WindowManager) -> None:
    manager.remove(window)
    global map_server_node

    # Map buttons
    buttons = []

    for map in maps:
        buttons.append(ptg.Button(
            map,
            lambda _, map=map: _set_map_name_and_main_menu(window, manager, map),
        ))

    window = ptg.Window(
        "Select a map",
        "",
        *buttons,
        "",
        ptg.Button("New Map", lambda _: _create_new_map(window, manager))
    ).center()

    manager.add(window)


def save_new_map(window: ptg.Window, manager: ptg.WindowManager):
    map = ""
    for widget in window:
        if isinstance(widget, ptg.Container):
            for sub_widget in widget:
                if isinstance(sub_widget, ptg.InputField):
                    map = sub_widget.value
                    break

    if map == "":
        _select_map(window, manager)
        return

    global map_name
    map_name = map

    if os.path.isdir("./maps/" + map_name):
        os.rmdir("./maps/" + map_name)
    os.mkdir("./maps/" + map_name)

    _main_menu(window, manager)

def run_tui():
    """Runs the application."""
    load_maps()

    while pose is None:
        print("Waiting for pose")
        time.sleep(0.1)

    _create_aliases()
    _configure_widgets()

    with ptg.WindowManager() as manager:
        manager.layout = _define_layout()

        header = ptg.Window(
            "[app.header] Waypoint Navigator ",
            box="EMPTY",
            is_persistant=True,
        )

        header.styles.fill = "app.header.fill"

        # Since header is the first defined slot, this will assign to the correct place
        manager.add(header)

        footer = ptg.Window(
            ptg.Button("Quit", lambda *_: _confirm_quit(manager)),
            box="EMPTY",
        )
        footer.styles.fill = "app.footer"

        # Since the second slot, body was not assigned to, we need to manually assign
        # to "footer"
        manager.add(footer, assign="footer")

        window = ptg.Window(
            "",
        ).center()

        manager.add(window)

        _select_map(window, manager)

    global shutdown_requested
    shutdown_requested = True

def run_ros():
    while shutdown_requested is False:
        rclpy.spin_once(node)
        time.sleep(0.1)

    rclpy.shutdown()


def main() -> None:

    rclpy.init()

    global node
    node = Node('navigator_gui')

    global goal_publisher
    global init_publisher
    global complete_subscriber
    global map_publisher
    global cancel_publisher
    global status_subscriber
    global explorer_publisher
    global explorer_cancel_publisher
    global explorer_status_subscriber
    global explorer_end_subscriber
    global odom_subscriber
    global map_saver_client

    goal_publisher = node.create_publisher(String, 'navigator_goal', 10)
    init_publisher = node.create_publisher(String, 'navigator_init', 10)
    complete_subscriber = node.create_subscription(Bool, 'navigator_goal_complete', complete_goal_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    map_publisher = node.create_publisher(String, 'navigator_map', 10)
    cancel_publisher = node.create_publisher(String, 'navigator_goal_cancel', 10)
    status_subscriber = node.create_subscription(String, 'navigator_goal_status', navigator_status_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    explorer_publisher = node.create_publisher(Int32, 'explorer_goal', 10)
    explorer_cancel_publisher = node.create_publisher(String, 'explorer_goal_cancel', 10)
    explorer_status_subscriber = node.create_subscription(Int32, 'explorer_goal_status', explorer_status_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    explorer_end_subscriber = node.create_subscription(String, 'explorer_goal_end', explorer_end_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    odom_subscriber = node.create_subscription(Odometry, 'odom', odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    map_saver_client = node.create_client(SaveMap, 'map_saver/save_map')
    while not map_saver_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('saver service not available, waiting again...')


    # Set up threads for ROS2 node and TUI
    ros2_thread = threading.Thread(target=run_ros)
    tui_thread = threading.Thread(target=run_tui)

    # Start both threads
    ros2_thread.start()
    tui_thread.start()

    tui_thread.join()
    # When TUI thread finishes, signal ROS2 thread to stop
    ros2_thread.join()


if __name__ == "__main__":
    main()
