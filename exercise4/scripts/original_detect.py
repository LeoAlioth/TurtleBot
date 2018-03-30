#!/usr/bin/env python
from __future__ import print_function
from __future__ import division

import sys
import cv2
import numpy as np

import time

class The_Ring:
    def __init__(self):
        # image
        self.cv_image = None
        # found ellipses
        self.candidates = []


        rospy.init_node('image_converter', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)

        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)


    def get_pose(self,e,dist):
        # Calculate the position of the detected ellipse

        k_f = 525 # kinect focal length in pixels

        elipse_x = self.dims[1] / 2 - e[0][0]
        elipse_y = self.dims[0] / 2 - e[0][1]

        angle_to_target = np.arctan2(elipse_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x,y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        # Define a stamped message for transformation
        point_s = PointStamped()
        point_s.point.x = x
        point_s.point.y = y
        point_s.point.z = 0.3
        point_s.header.frame_id = "base_link"
        point_s.header.stamp = rospy.Time(0)

        # Get the point in the "map" coordinate system
        point_world = self.tf_buf.transform(point_s,"map")

        # Create a Pose object with the same position
        pose = Pose()
        pose.position.x = point_world.point.x
        pose.position.y = point_world.point.y
        pose.position.z = point_world.point.z

        # Create a marker used for visualization
        self.marker_num += 1
        marker = Marker()
        marker.header.stamp = point_world.header.stamp
        marker.header.frame_id = point_world.header.frame_id
        marker.pose = pose
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(30)
        marker.id = self.marker_num
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0, 1, 0, 1)
        self.marker_array.markers.append(marker)

        self.markers_pub.publish(self.marker_array)


    def image_callback(self,data):
        # print('Iam here!')

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = self.cv_image.shape

        # get height and width of image
        height, width = self.get_dimensions()
        print("height: {}, width: {}".format(height, width))
        q1 = int(height / 4)
        q2 = int(height - (height / 4))

        # crop image, 1/4 from bottom and top
        self.cv_image = self.cv_image[q1:q2, ]

        # Tranform image to gayscale
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        hist = cv2.equalizeHist(gray)

        start = time.clock()
        self.candidates += self.calculate(self.calcContours(hist, 30, 255, 0))
        self.candidates += self.calculate(self.calcContours(hist, 50, 255, 0))
        self.candidates += self.calculate(self.calcContours(hist, 110, 255, 0))
        self.candidates += self.calculate(self.calcContours(hist, 130, 255, 0))
        self.candidates += self.calculate(self.calcContours(hist, 150, 255, 0))
        self.candidates += self.calculate(self.calcContours(hist, 210, 255, 0))
        end = time.clock()
        print("time elapsed for calculation:", end - start)
        self.draw(self.candidates)
        self.marker_array += self.candidates
        self.markers_pub.publish(self.marker_array)

        # Example how to draw the contours
        # cv2.drawContours(img, contours, -1, (255, 0, 0), 3)

    def get_dimensions(self):
        height, width = self.cv_image.shape[:2]
        return height, width

    def groupCalculations(self, *all_contours):
        noncandidates, candidates = []

        # calculate for each contour
        for contour in all_contours:
            # calculate elipses from this contour, only candidates -> c, are interesting
            non_c, c = self.calculate(self.contours)

        return noncandidates, candidates

    def calcContours(self, img, x, y, z):
        # Binarize the image
        ret, thresh = cv2.threshold(img, x, y, z)
        # Extract contours
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.imshow("contours", im2)
        cv2.waitKey(0)
        return contours

    def calculate(self, contours):
        # precalulate top and bottom line for approving center
        height, _ = self.get_dimensions()
        top = height / 4
        bottom = height - (top)
        print("CENTER THRESH:", top, bottom)

        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)

        # Find two elipses with same centers
        goodPairs = [] # pairs of 2 already found elipses
        noncandidates = []
        candidates = []
        for n in range(len(elps)):
            i = 0
            for m in range(n + 1, len(elps)):
                e1 = elps[n]
                e2 = elps[m]

                c1x = e1[0][0]
                c1y = e1[0][1]
                c2x = e2[0][0]
                c2y = e2[0][1]

                # take a look at ratios
                # if ratio of one circle is 0, than this pair is not valid
                if e2[1][0] == 0 and e2[1][1] == 0:
                    continue

                # centers are not the same
                x_of_elipses = abs(c1x - c2x)
                y_of_elipses = abs(c1y - c2y)

                if x_of_elipses > 0.5 and y_of_elipses > 0.5:
                    noncandidates.append((e1, e2))
                    continue

                # so what about ratios?
                # get ratios
                ratio1 = e1[1][0] / e2[1][0]
                ratio2 = e1[1][1] / e2[1][1]

                if abs(ratio1-ratio2) > 0.1:
                    noncandidates.append((e1, e2))
                    continue

                # let's check where is center of potential circles
                # check height, width is not importnatn
                if c1y > bottom or c1y < top:
                    noncandidates.append((e1, e2))
                    continue

                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                if dist < 5:
                    # irretate through center coordinates of already found pairs of ellipses
                    # thoose elipses are valid
                    # there can't be new circle in their radius
                    # let's approximate it to about 5-10px
                    x_of_elipses = (c1x + c2x) / 2
                    y_of_elipses = (c1y + c2y) / 2
                    found = False

                    for x, y in goodPairs:
                        #print("SUBSTRACTION of ({:.4f} + {:.4f})/2 - {:.4f} and ({:.4f} + {:.4f})/2 - {:.4f}".format(
                        #    c1x, c2x, x, c1y, c2y, y
                        #),x_of_elipses-x,y_of_elipses-y)

                        if (abs(x_of_elipses-x) < 10) and (abs(y_of_elipses-y) < 10):
                            #print("continue")
                            found = True
                            break

                    if found:
                        continue

                    # save average value of coordinates
                    goodPairs.append((x_of_elipses, y_of_elipses))
                    #print("end of pairs")

                    # save coordinates of good circles
                    candidates.append((e1, e2))

                    #    i += 1
                    #ratio1 = e1[1][0] / e2[1][0] if e2[1][0] != 0 else e2[1][0] / e1[1][0]
                    #ratio2 = e1[1][1] / e2[1][1] if e2[1][1] != 0 else e2[1][1] / e1[1][1]
                    #print(goodPairs)
                    #print("e1 {0:.4f},{1:.4f}, e2 {2:.4f},{3:.4f}".format(
                    #    e1[0][0], e1[0][1], e2[0][0], e2[0][1]
                    #))
                    print("DISTANCE",dist, "CORD", e1[0][0], e1[0][1], "AND", e2[0][0], e2[0][1])
                    #print("RATIO1: {} RATIO2: {}, difference: {}".format(ratio1, ratio2, abs(ratio1-ratio2)))

                    #if ratio <
                    #print(dist)
                    #for j in range(len(e1)):
                    #    print("e1 {}".format(e1[j]), j)
                    #    print("e2 {}".format(e2[j]), j)

                #    candidates.append((e1, e2))
        return candidates
        #return (candidates, noncandidates)

    def draw(self, candidates, noncandidates=[]):
        for c in noncandidates:
            e1 = c[0]
            e2 = c[1]

            cv2.ellipse(self.cv_image, e1, (0, 0, 255), 1)
            cv2.ellipse(self.cv_image, e2, (0, 0, 255), 1)

            size = (e1[1][0] + e1[1][1]) / 2
            center = (e1[0][1], e1[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1 > 0 else 0
            x_max = x2 if x2 < self.cv_image.shape[0] else self.cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < self.cv_image.shape[1] else self.cv_image.shape[1]

            # Extract the depth from the depth image
        for c in candidates:
            e1 = c[0]
            e2 = c[1]

            cv2.ellipse(self.cv_image, e1, (0, 255, 0), 1)
            cv2.ellipse(self.cv_image, e2, (0, 255, 0), 1)

            size = int(e1[1][0] + e1[1][1]) / 2
            center = (e1[0][1], e1[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1 > 0 else 0
            x_max = x2 if x2 < self.cv_image.shape[0] else self.cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < self.cv_image.shape[1] else self.cv_image.shape[1]

        cv2.imshow("Image window", self.cv_image)
        cv2.waitKey(1)


    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV
        image_1 = depth_image / 65536.0 * 255
        image_1 =image_1/np.max(image_1)*255


        image_viz = np.array(image_1, dtype= np.uint8)

        cv2.imshow("Depth window", image_viz)
        cv2.waitKey(1)


def main(args):

    ring_finder = The_Ring()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
