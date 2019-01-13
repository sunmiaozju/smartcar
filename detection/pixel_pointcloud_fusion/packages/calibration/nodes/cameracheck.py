#!/usr/bin/python

import rospy
from autoware_camera_calibration.camera_checker import CameraCheckerNode


def main():
    from optparse import OptionParser
    rospy.init_node('cameracheck')
    parser = OptionParser()
    parser.add_option("-s", "--size", default="8x6", help="specify chessboard size as nxm [default: %default]")
    parser.add_option("-q", "--square", default=".108", help="specify chessboard square size in meters [default: %default]")
    parser.add_option("--approximate",
                      type="float", default=0.0,
                      help="allow specified slop (in seconds) when pairing images from unsynchronized stereo cameras")

    options, args = parser.parse_args()
    size = tuple([int(c) for c in options.size.split('x')])
    dim = float(options.square)
    approximate = float(options.approximate)
    CameraCheckerNode(size, dim, approximate)
    rospy.spin()

if __name__ == "__main__":
    main()