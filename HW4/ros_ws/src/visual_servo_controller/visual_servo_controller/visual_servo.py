import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np

# Transformation related libraries
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from tf_transformations import quaternion_matrix
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray 
def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

def get_center_of_mask(mask):
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
 
class VisualServoController(Node):
    def __init__(self):
        super().__init__('visual_servo_controller')

        self.subscription = self.create_subscription(
                                              Image, 
                                              '/camera1/image_raw', 
                                              self.listener_callback, 
                                              10)

        self.publisher_ = self.create_publisher(
                                            Image, 
                                            'output_image', 
                                            10)

        # Create a publisher to send velocity commands
        self.velocity_publisher_ = self.create_publisher(
                                            Float64MultiArray, 
                                            'forward_velocity_controller/commands', 
                                            10)

          
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # For HSV thresholds
        self.blue_low_ = (100,100,100)
        self.blue_high_ = (120,255,255)

        self.green_low_ = (50,100,100)
        self.green_high_ = (100,255,255)

        self.red_low_ = (0,100,100)
        self.red_high_ = (50,255,255)

        self.pink_low_ = (145,100,100)
        self.pink_high_ = (150,255,255)

        
        # TF related
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def calculate_image_feature_centers(self,current_frame):
        hsv = cv2.cvtColor(current_frame,cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv,self.blue_low_,self.blue_high_)
        red_mask = cv2.inRange(hsv,self.red_low_,self.red_high_)
        green_mask = cv2.inRange(hsv,self.green_low_,self.green_high_)
        pink_mask = cv2.inRange(hsv,self.pink_low_,self.pink_high_)

        final_mask = blue_mask + red_mask + green_mask + pink_mask

        x_blue,y_blue =  get_center_of_mask(blue_mask)
        x_green,y_green =  get_center_of_mask(green_mask)
        x_pink,y_pink =  get_center_of_mask(pink_mask)
        x_red,y_red =  get_center_of_mask(red_mask)
        feats_curr = np.array([[x_blue,y_blue],
                                [x_green,y_green],
                                [x_pink,y_pink],
                                [x_red,y_red]],dtype=np.float32)
        return feats_curr,final_mask

        # print(f"blue:{x_blue,y_blue}")
        # print(f"green:{x_green,y_green}")
        # print(f"pink:{x_pink,y_pink}")
        # print(f"red:{x_red,y_red}")
        # blue:(287.82439744220363, 427.43384161337923)
        # green:(371.5760549558391, 511.1143277723258)
        # pink:(288.0802088277171, 510.936877076412)
        # red:(371.3253638253638, 427.65696465696465)

    def get_transforms_R_and_T(self, to_frame_rel, from_frame_rel):
        """
        input:
            to_frame_rel - to which transformation has to be calculated
            from_frame_rel - from which transformation has to be calculated
        output:
            T - translation vector (x,y,z) 3 x 1
            R - homogeneous rotation matrix 4 x 4
        """
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                          to_frame_rel,
                          from_frame_rel,
                          now)

            x = trans.transform.rotation.x
            y = trans.transform.rotation.y
            z = trans.transform.rotation.z
            w = trans.transform.rotation.w
            R = quaternion_matrix([x,y,z,w])

            T = np.array([trans.transform.translation.x,
                            trans.transform.translation.y,
                            trans.transform.translation.z])

            return True, R, T

        except TransformException as ex:
          self.get_logger().info(
          f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
          return False, None, None

    def get_rrbot_jacobian(self):
        """
        output:
            RR Bot Jacobian - 6 x 2
        """
        is_transform1, R_link1_to_cam, T_link1_to_cam = self.get_transforms_R_and_T("link1","camera_link")
        is_transform2, R_link1_to_link2, T_link1_to_link2 = self.get_transforms_R_and_T("link1","link2")
        if not is_transform1 or not is_transform2:
            return None
        z_local = np.array([0,0,1]) # 3,
        z_local = skew(z_local) # 3 x 3
        J11 = np.eye(3) @ z_local @ T_link1_to_cam # 3 x 1
        J12 = R_link1_to_link2[0:3,0:3] @ z_local @ (T_link1_to_cam - T_link1_to_link2) # 3 x 1
        J21 = np.eye(3) @ z_local # 3 x 1
        J22 = R_link1_to_link2[0:3,0:3] @ z_local # 3 x 1
        J1  = np.hstack((J11,J12)) # 3 x 2
        J2  = np.hstack((J21,J22)) # 3 x 2
        J   = np.vstack((J1,J2)) # 6 x 2
        return J
        
    def calculate_required_joint_velocities(self, feats_ref, feats_curr, lmbda = 0.003):
        """
        input:
            feats_ref - reference feature points 4 x 2
            feats_curr - current feature points 4 x 2
        output:
            joint_velocities - 2 x 1
        """
        # Calculate image inverse jacobian
        L_e_i = np.eye(2) # 2 x 2
        L_e = np.vstack((L_e_i,L_e_i,L_e_i,L_e_i)) # 8 x 2
        L_e_inv = np.linalg.pinv(L_e) # 2 x 8
        
        # calculate error in feature points
        error = feats_curr - feats_ref # 4 x 2
        error = lmbda*error # TODO confirm these equations
        error = error.reshape((8,1))
        print(f"error:{error.flatten()}")

        # calculate required camera velocities in camera frame
        v_cam = L_e_inv @ error # 2 x 1

        # calculate required camera velocities in joint1 frame
        is_transform,R,T = self.get_transforms_R_and_T("link1","camera_link")
        if not is_transform:
            return None

        v_link1 = R[0:2,0:2] @ v_cam # 2 x 1
        Jaco = self.get_rrbot_jacobian() # 6 x 2
        if Jaco is None:
            return None
        Jaco_inv = np.linalg.pinv(Jaco) # 2 x 6
        joint_vel = Jaco_inv[0:2,0:2] @ v_link1 # 2 x 1
        return joint_vel


    def listener_callback(self, data):
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        
        # get current feature locations
        feats_curr,final_mask = self.calculate_image_feature_centers(current_frame)
        print(f"features:{feats_curr.flatten()}")

        x_ref_blue,y_ref_blue=(288,427)
        x_ref_green,y_ref_green=(371,511)
        x_ref_pink,y_ref_pink=(288,510)
        x_ref_red,y_ref_red=(371,428)

        feats_ref = np.array([[x_ref_blue,y_ref_blue],
                                [x_ref_green,y_ref_green],
                                [x_ref_pink,y_ref_pink],
                                [x_ref_red,y_ref_red]],dtype=np.float32)

        # get the required joint velocities
        joint_velocities = self.calculate_required_joint_velocities(feats_ref,feats_curr)
        if joint_velocities is None:
            return

        # format and publish the velocities
        velocities = Float64MultiArray()
        velocities.data = [joint_velocities[0,0],joint_velocities[1,0]]
        self.velocity_publisher_.publish(velocities)

        # extract and publish the masked output
        masked_img= cv2.bitwise_and(current_frame, current_frame, mask=final_mask)

        self.publisher_.publish(self.br.cv2_to_imgmsg(masked_img , encoding="bgr8"))
    
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    controller = VisualServoController()

    # Spin the node so the callback function is called.
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
