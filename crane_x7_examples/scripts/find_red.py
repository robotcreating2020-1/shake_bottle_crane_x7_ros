#!/usr/bin/env python 
# -*- coding: utf-8 -*-
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # RGB表色系からHSV表色系に変換                                                           
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # しきい値の設定1（ここでは赤を抽出）                                                     
    color_min = np.array([0,200,50])
    color_max = np.array([30,255,255])

    # マスク画像を生成1                                                                       
    color_mask1 = cv2.inRange(hsv_image, color_min, color_max)
    
    # しきい値の設定2
    color_min = np.array([150,200,50])
    color_max = np.array([179,255,255])

    # マスク画像を生成2                                                                      
    color_mask2 = cv2.inRange(hsv_image, color_min, color_max)
    
    # 赤色のマスク
    mask = color_mask1 + color_mask2

    # 画像配列のビット毎の倫理席。マスク画像だけが抽出される。                               
    cv_image2  = cv2.bitwise_and(cv_image, cv_image, mask = mask)

    # マスクした画像からグレースケールへ変換
    gray_image = cv2.cvtColor(cv_image2, cv2.COLOR_BGR2GRAY)

    # グレースケールから白黒に変換
    ret,thresh = cv2.threshold(gray_image, 0, 255, cv2.THRESH_OTSU)
    
    
    #ウインドウのサイズを変更                                                               
    cv_half_image = cv2.resize(cv_image,   (0,0),fx=0.5, fy=0.5)
    cv_half_image2 = cv2.resize(cv_image2, (0,0),fx=0.5,fy=0.5);
    cv_half_image3 = cv2.resize(gray_image, (0,0),fx=0.5,fy=0.5);
    cv_half_image4 = cv2.resize(thresh, (0,0),fx=0.5,fy=0.5);

    # ウインドウ表示                                                                         
    cv2.imshow("Origin Image", cv_half_image)
    cv2.imshow("Result Image", cv_half_image2)
    cv2.imshow("gray Image",   cv_half_image3)
    cv2.imshow("Image",   cv_half_image4)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
