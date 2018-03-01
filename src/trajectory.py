from collections import OrderedDict, namedtuple
import rospy

Point = namedtuple('Point', 'x y z')
Quaternion = namedtuple('Quaternion', 'x y z w')
Transform = namedtuple('Transform', 'translation rotation')

def print_trajectory(trajectory):
    for stamp in trajectory:
        print '{}:'.format(stamp)
        for identifier in trajectory[stamp]:
            print '\t {}: {}'.format(identifier, trajectory[stamp][identifier])

def joint_state_trajectory():
    joint_names = ["joint_b", "joint_a"]
    trajectory = OrderedDict()  # PRO: preserve order of samples even if time stamps are faulty (or all 0.0)

    for i in range(0,2):
        stamp = rospy.Time.now().to_sec()
        trajectory[stamp] = OrderedDict()  # PRO: preserve insertion order for easy conversion with ROS messages
        for joint_name in joint_names:
            trajectory[stamp][joint_name] = {}  # PRO: flexible extensions
            trajectory[stamp][joint_name]["pos"] = 41 + i  # CON: not easy to get position trajectory of a single DOF
            trajectory[stamp][joint_name]["vel"] = -0.2*i
        rospy.sleep(rospy.Duration(0.1))

    print_trajectory(trajectory)

def robot_state_trajectory():
    link_names = ["shoulder", "elbow"]
    trajectory = OrderedDict()

    for i in range(0,2):
        stamp = rospy.Time.now().to_sec()
        trajectory[stamp] = OrderedDict()
        for link_name in link_names:
            trajectory[stamp][link_name] = {}
            trajectory[stamp][link_name]['reference_frame'] = 'world' # PRO/CON: reference frame optional
            # PRO: structure of Transform fixed, and compatible with geometry_msgs/Transform
            trajectory[stamp][link_name]['transform'] = Transform(Point(1,2,3), Quaternion(0,0,0,1))
        rospy.sleep(rospy.Duration(0.1))

    print_trajectory(trajectory)


rospy.init_node("foobar")
print '\njoint state trajectory'
joint_state_trajectory()
print '\nrobot state trajectory'
robot_state_trajectory()
