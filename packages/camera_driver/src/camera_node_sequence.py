#!/usr/bin/env python
import io
import thread
import os
import yaml
import rospy

from duckietown_msgs.msg import BoolStamped
from picamera import PiCamera
from sensor_msgs.msg import CompressedImage
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse


class CameraNode(object):

    def __init__(self):
        """Handles the imagery.

        The node handles the image stream, initializing it, publishing frames
        according to the required frequency and stops it at shutdown.

        Subscribers:
            sub_switch_high:
                topic: ~framerate_high_switch
                type: BoolStamped
        Publishers:
            pub_img:
                topic: ~image/compressed
                type: CompressedImage
        Services:
            srv_set_camera_info:
                topic: ~set_camera_info
                type: SetCameraInfo
        """
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.framerate_high = self.setupParam("~framerate_high", 30.0)
        self.framerate_low = self.setupParam("~framerate_low", 15.0)
        self.res_w = self.setupParam("~res_w", 640)
        self.res_h = self.setupParam("~res_h", 480)

        self.image_msg = CompressedImage()

        # Setup PiCamera
        self.camera = PiCamera()
        self.framerate = self.framerate_high  # default to high
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w, self.res_h)

        # For intrinsic calibration
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'

        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'

        self.has_published = False
        self.pub_img = rospy.Publisher("~image/compressed",
                                       CompressedImage,
                                       queue_size=1)
        self.sub_switch_high = rospy.Subscriber("~framerate_high_switch",
                                                BoolStamped,
                                                self.cbSwitchHigh,
                                                queue_size=1)

        # Create service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info",
                                                 SetCameraInfo,
                                                 self.cbSrvSetCameraInfo)
        self.stream = io.BytesIO()

        self.is_shutdown = False
        self.update_framerate = False

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbSwitchHigh(self, switch_msg):
        """Callback for the imagery frequency switch.

        Increases / decreases the frequency of imagery.

        Args:
            switch_msg (BoolStamped): switch_msg
        """
        rospy.loginfo("[%s] frequency switch to %s." % (self.node_name,
                                                        switch_msg.data))
        if switch_msg.data and self.framerate != self.framerate_high:
            self.framerate = self.framerate_high
            self.update_framerate = True
        elif not switch_msg.data and self.framerate != self.framerate_low:
            self.framerate = self.framerate_low
            self.update_framerate = True

    def startCapturing(self):
        """Initialize and closes image stream.

            Begin the camera capturing. When the node shutdowns, closes the
            image stream.
        """
        rospy.loginfo("[%s] Start capturing." % (self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            gen = self.grabAndPublish(self.stream, self.pub_img)
            try:
                self.camera.capture_sequence(gen,
                                             'jpeg',
                                             use_video_port=True,
                                             splitter_port=0)
            except StopIteration:
                pass
            self.camera.framerate = self.framerate
            self.update_framerate = False

        self.camera.close()
        rospy.loginfo("[%s] Capture Ended." % (self.node_name))

    def grabAndPublish(self, stream, publisher):
        """Captures a frame from stream and publishes it.

            If the stream is stable (no updates in frequency or shutdowns),
            grabs a frame, creates the image message and publishes it.

            Args:
                stream (BytesIO): imagery stream
                publisher (Publisher): publisher of topic
        """
        while not (self.update_framerate or self.is_shutdown or rospy.is_shutdown()):
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()
            # Generate compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            image_msg.data = stream_data

            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            publisher.publish(image_msg)

            # Clear stream
            stream.seek(0)
            stream.truncate()

            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." % (self.node_name))
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))

    def setupParam(self, param_name, default_value):
        """Parameter server handler.

            Sets value of parameter and file and prints it.

            Args:
                param_name (String): name of the parameter
                value(String): value of the paramter
        """
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def onShutdown(self):
        """Shutdown procedure."""
        rospy.loginfo("[%s] Closing camera." % (self.node_name))
        self.is_shutdown = True
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

    def cbSrvSetCameraInfo(self, req):
        rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info, filename)
        response.status_message = "Write to %s" % filename
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        """Saves intrinsic calibration to file.

            Args:
                camera_info_msg (CameraInfo): Camera Info containg calibration
                filename (String): filename where to save calibration
        """
        # Convert camera_info_msg and save to a yaml file
        rospy.loginfo("[saveCameraInfo] filename: %s" % (filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
                 'image_height': camera_info_msg.height,
                 'camera_name': rospy.get_name().strip("/"),  # TODO check this
                 'distortion_model': camera_info_msg.distortion_model,
                 'distortion_coefficients': {'data': camera_info_msg.D,
                                             'rows': 1,
                                             'cols': 5},
                 'camera_matrix': {'data': camera_info_msg.K,
                                   'rows': 3,
                                   'cols': 3},
                 'rectification_matrix': {'data': camera_info_msg.R,
                                          'rows': 3,
                                          'cols': 3},
                 'projection_matrix': {'data': camera_info_msg.P,
                                       'rows': 3,
                                       'cols': 4}}

        rospy.loginfo("[saveCameraInfo] calib %s" % (calib))

        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('camera', anonymous=False)
    # Create the TrafficLightNode object
    camera_node = CameraNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(camera_node.onShutdown)
    thread.start_new_thread(camera_node.startCapturing, ())
    # Keep it spinning to keep the node alive
    rospy.spin()