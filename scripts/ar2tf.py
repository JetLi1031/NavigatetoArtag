#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from apriltagtut.msg import Rotate
import cv2
from pupil_apriltags import Detector
import numpy as np
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped,Vector3,Quaternion,Transform
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster

class artotf(Node):
    def __init__(self):
       super().__init__('ar2tf')
       qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
       self.subscription = self.create_subscription(
            Image,
            "/depth_camera/image_raw",
            self.videoprocess,
            qos_profile=qos_policy)
       self.subscription  # prevent unused variable warning
       self.broadcast = TransformBroadcaster(self)
       family = "tagStandard52h13 tagCircle21h7 tagCustom48h12 tag36h11"
       self.detector = Detector(families= family,
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)
    def videoprocess(self,image_message):
        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(image_message)#, desired_encoding='passthrough'
                # Our operations on the frame come here
        self.detsingleframe(self.detector)
        # Display the resulting frame
        #cv2.imshow('frame', self.cv_image)
        #cv2.waitKey(1)
        return


    #detect single frame
    def detsingleframe(self,detector):
      #print(sinimage)
      #previousshape = sinimage.shape[:2][::-1]
      #no distortion in gazebo
      #sinimage = self.undistorted(sinimage) 
      self.calldetector(detector)
      #sinimage = cv2.resize(sinimage, previousshape, interpolation = cv2.INTER_AREA)
      
    
    # remove distortion 
    def undistorted(self,image):
      h,  w = image.shape[:2]
      mtx =np.array([580.008354, 0.000000, 340.189832, 0.000000, 574.758875, 247.655467, 0.000000, 0.000000, 1.000000]).reshape(3,3)
      dist= np.array([0.094975, -0.423554, 0.010464, -0.003432, 0.000000]).reshape(1,-1)
      newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
      image = cv2.undistort(image, mtx, dist, None, newcameramtx)
      x, y, w, h = roi
      image= image[y:y+h, x:x+w]
      return image


    # draw each tag and pub each tag tf  
    def calldetector(self,detector):
      #try:
        
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        #print(detector)
        tags = detector.detect(gray, estimate_tag_pose=True,camera_params=[565.6,565.6,320.5,240.5], tag_size=0.15)
        if not tags: 
            print("not tag found")
            return
        for tag in tags:
            pts = tag.corners.reshape((-1,1,2)).astype(int)
            ctrpt = tuple(tag.center.astype(int))
            self.cv_image = cv2.polylines(self.cv_image,[pts],True,(255,0,0),3)
            self.cv_image = cv2.circle(self.cv_image,ctrpt,5,(0,0,255),-1)
            self.draw_imageframe(tag)
            tlabel = list(zip(['x','y','z'],list(tag.pose_t.reshape(-1))))
            tdict = {x:y for x,y in tlabel}
            vect = Vector3(**tdict)
           
           
            r = R.from_matrix(tag.pose_R)
            rotatebyx = R.from_euler('xy', [90,-90], degrees=True)
            rotated = r * rotatebyx
            rlabel = list(zip(['x','y','z','w'],list(rotated.as_quat())))
            rdict = {x:y for x,y in rlabel}
            quat = Quaternion(**rdict)
            
            #defien tfstamp
            
            tfs = TransformStamped()
            tfs.header.stamp = self.get_clock().now().to_msg()
            tfs.header.frame_id="camera_depth_frame"
            tfs.child_frame_id = "Ar_tagframe"
            tfs.transform = Transform(translation=vect,rotation=quat)
            #print(tfs)
            self.broadcast.sendTransform(tfs)
            #self.get_logger().info('testing inheritance: "%s"' % msg) 
            
    #  except Exception as exp:
      #  print("error found {}".format(exp))
     #  return None

        return

    #draw a frame by inversing tf matrix
    def draw_imageframe(self,tag):
          #print(tag.pose_R)
          thecenter = tag.center
          newcenter = np.append(thecenter,0)
          newcenter = newcenter.reshape(-1,1)
          centerinreal = tag.pose_R.T@newcenter-tag.pose_t
          # @tag.center + tag.pose_t
          # print(centerinreal)
          for i in range(3):
            initl = np.zeros((3,1))
            initl[i][0] = 100
            inittup = [0,0,0]
            inittup.insert(i,255)
            inittup = tuple(inittup)
            # print(inittup)
            newcordinate = centerinreal + initl
            aftertf = tag.pose_R@newcordinate + tag.pose_t
            self.cv_image = cv2.line(self.cv_image, tuple(tag.center.astype(int)), tuple(aftertf[:2].reshape(2).astype(int)), inittup, 3)
            
            
            
def main(args=None):
    rclpy.init(args=args)

    artotf2 = artotf()

    rclpy.spin(artotf2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Apriltagpub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

