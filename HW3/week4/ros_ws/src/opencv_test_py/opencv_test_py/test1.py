# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from skimage.feature import peak_local_max

def get_center_of_mask( mask):
    x_i = 0;
    y_i = 0;
    count = 0;
    coords = np.argwhere(mask)
    count = coords.shape[0]
    coords = np.sum(coords,axis=0)
    if count > 0:
        coords = coords/count
        return coords

    return (0,0)
    
   
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
      Image, 
      '/camera1/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'output_image', 10)

      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)


    # PLACE YOUR CODE HERE. PROCESS THE CURRENT FRAME AND PUBLISH IT. IF YOU ARE HAVING DIFFICULTY PUBLISHING IT YOU CAN USE THE FOLLOWING LINES TO DISPLAY IT VIA OPENCV FUNCTIONS
    #cv2.imshow("output_image", current_frame)
    #cv2.waitKey(1)
    # read once and save
    #cv2.imwrite("output_image.png",current_frame)
    #exit()
    blue_low = (100,100,100)
    blue_high = (120,255,255)

    green_low = (50,100,100)
    green_high = (100,255,255)

    red_low = (0,100,100)
    red_high = (50,255,255)

    pink_low = (145,100,100)
    pink_high = (150,255,255)

    
    hsv = cv2.cvtColor(current_frame,cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(hsv,blue_low,blue_high)
    red_mask = cv2.inRange(hsv,red_low,red_high)
    green_mask = cv2.inRange(hsv,green_low,green_high)
    pink_mask = cv2.inRange(hsv,pink_low,pink_high)

    final_mask = blue_mask + red_mask + green_mask + pink_mask
    #cv2.imshow("output_image", final_mask)

    x_blue,y_blue =  get_center_of_mask(blue_mask)
    x_green,y_green =  get_center_of_mask(green_mask)
    x_pink,y_pink =  get_center_of_mask(pink_mask)
    x_red,y_red =  get_center_of_mask(red_mask)
    print(f"blue:{x_blue,y_blue}")
    print(f"green:{x_green,y_green}")
    print(f"pink:{x_pink,y_pink}")
    print(f"red:{x_red,y_red}")
    # blue:(287.82439744220363, 427.43384161337923)
    # green:(371.5760549558391, 511.1143277723258)
    # pink:(288.0802088277171, 510.936877076412)
    # red:(371.3253638253638, 427.65696465696465)

    masked_img= cv2.bitwise_and(current_frame, current_frame, mask=final_mask)
    cv2.imshow("mask",final_mask)

    # get gray scale image
    gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

    ## Canny Edge detection
    canny = cv2.Canny(gray,200,300)
    #cv2.imshow("canny", canny)
    cv2.waitKey(1)


    ## corners
    # apply harris corners
    img_corners = cv2.cornerHarris(gray,4,7,0.04)

    # find local maximas in the corner score image
    coords = peak_local_max(img_corners,min_distance=3,threshold_abs=0.01)

    # draw markers in the image  
#    for coord in coords:
#        cv2.drawMarker(current_frame, [coord[1],coord[0]], color=[165,42,42])


    # hough circles
    circles = cv2.HoughCircles(gray, 
                            cv2.HOUGH_GRADIENT,1,20, 
                            param1=100,
                            param2=30,
                            minRadius=0,
                            maxRadius=0)

    circles = np.uint16(np.around(circles))
    img_circles = current_frame.copy()
    for i in circles[0,:]:
        cv2.circle(img_circles,(i[0],i[1]),i[2],(165,42,42),2)
        cv2.circle(img_circles,(i[0],i[1]),2,(165,42,42),3)
    #blue center:(426, 288)
    #red center:(428, 370)
    #pink center:(508, 286)
    #green center:(512, 370)


    #cv2.imshow("circles",img_circles)

    # Publish the image.
    # The 'cv2_to_imgmsg' method converts an OpenCV
    # image to a ROS 2 image message
    self.publisher_.publish(self.br.cv2_to_imgmsg(masked_img , encoding="bgr8"))

    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
