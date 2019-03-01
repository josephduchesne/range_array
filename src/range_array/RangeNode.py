#!/usr/bin/python

from serial import Serial
import rospy, time, tf2_ros, tf_conversions, geometry_msgs
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

serial_port = None
scan_pub = None
range_pub = []
# position of each sensor: x, y, yaw, pitch in m
range_pos = [[-0.23,0.055,-3.5*26, 11.2], [-0.1725, 0.015, -2.5*26, 11.2], [-0.115,0,-1.5*26, 11.2], [-0.0575, 0, -0.5*26, 11.2], \
             [0.0575, 0, 0.5*26, 11.2], [0.115,0,1.5*26, 11.2], [0.1725, 0.015, 2.5*26, 11.2], [0.23,0.055,3.5*26, 11.2]]

def init(port):
    global serial_port, scan_pub, range_pub
    rospy.init_node('range_node')  # Init ROS

    # Initialize all publishers
    scan_pub = rospy.Publisher('/range_array_scan', LaserScan, queue_size=1)
    for i in range(0,8):
       range_pub.append(rospy.Publisher('/range_pub_%d' % i, Range, queue_size=1))

    # Connect to the arduino
    print("Init Serial %s" % port)
    serial_port = Serial(port, 115200, timeout=None,  stopbits = 1)
    time.sleep(1) #nap while the arduino boots

def parse_sensor_data(line):
    global scan_pub, range_pub

    # Handle debug text from the arduino
    if "," not in line:
        rospy.loginfo("RangeNode: %s" % line)
        return
    
    # Parse the range string into a float array
    ranges = [float(x)/1000.0 for x in line.split(",")[::-1]]
    if len(ranges) != 8:
        rospy.logwarn("Received other than 8 scan ranges", ranges)
        return

    br = tf2_ros.TransformBroadcaster()

    # msg = LaserScan()
    # msg.header.frame_id = "base_link"
    # msg.header.stamp = rospy.Time.now()
    # msg.angle_increment = 26.0/180.0*3.141592
    # msg.angle_min = msg.angle_increment*-3.5
    # msg.angle_max = msg.angle_increment*3.5
    # msg.range_min  = 0.1
    # msg.range_max  = 4.0
    # msg.ranges = ranges  # reverse!
    # scan_pub.publish(msg)

    for i in range(0,8):
        # Emit the range data for this range
        rmsg = Range()
        rmsg.header.frame_id = "base_link"
        rmsg.header.stamp = rospy.Time.now()
        rmsg.header.frame_id = "rangefinder_%d" % i
        rmsg.min_range = 0.1
        rmsg.max_range = 4.0
        rmsg.field_of_view = 26.0/180.0*3.141592
        rmsg.radiation_type = rmsg.INFRARED
        rmsg.range = ranges[i]
        range_pub[i].publish(rmsg)

        # output the TF2 for this range
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = rmsg.header.frame_id
        t.transform.translation.x = range_pos[i][0]
        t.transform.translation.y = range_pos[i][1]-0.2
        t.transform.translation.z = 0.2
        q = tf_conversions.transformations.quaternion_from_euler(0, -range_pos[i][3]/180.0*3.1415, range_pos[i][2]/180.0*3.1415-3.1415/2)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)


def main():
    global serial_port, scan_pub
    
    init('/dev/ttyUSB1')
    
    # Wait for serial lines, emitting ROS messages whenever one is received
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        while serial_port.inWaiting():
            parse_sensor_data(serial_port.readline().strip())
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass