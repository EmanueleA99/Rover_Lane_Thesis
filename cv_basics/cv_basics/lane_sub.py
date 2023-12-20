# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import matplotlib.pyplot as plt 
import numpy as np
from geometry_msgs.msg import TwistStamped 
from std_msgs.msg import Int32

class LaneSubscriber(Node):
  """
  Create an LaneSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    self.stop = 0 #variable for stop and go

    # Initiate the Node class's constructor and give it a name
    super().__init__('lane_subscriber')

    #parameter declaration
    from rcl_interfaces.msg import ParameterDescriptor
    kc_parameter_descriptor = ParameterDescriptor(description='Coefficient of steering')
    velocity_parameter_descriptor = ParameterDescriptor(description='Standard linear velocity')
    self.declare_parameter('proportional_kc',float(0.1),kc_parameter_descriptor)
    self.declare_parameter('linear_velocity',float(5),velocity_parameter_descriptor)
    
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/car1/video_frames', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    #Create the publisher for cmd_vel topic
    self.publisher_ = self.create_publisher(TwistStamped, '/car1/cmd_vel', 10)

    #Create subscriber aruco_code
    self.aruco_sub = self.create_subscription(Int32,'/car1/nav_cmd',self.aruco_callback, 10)
   
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def aruco_callback(self,msg):
    aruco = msg.data #assign data to aruco variable
       #check aruco code:
    if (aruco == 16) : 
       self.stop = 1 #stop car until see a new aruco restart
    elif (aruco == 42):
       self.stop = 0 #aruco restart detected, now car can move

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    #For every frame, Lane detection algorithm :
    copy = np.copy(current_frame) 
    grey_im = grey(copy) #convert frame in grayscale image

    # apply image thresholding
    ret, thresh = cv2.threshold(grey_im, 115, 255, cv2.THRESH_BINARY)

    gaus = gauss(thresh) #smooth the image
    edges = canny(gaus) 
    isolated = region(edges) #Isolate the area of interest of the lines
    cv2.imshow("isolated", isolated) #show what the "camera" sees
    #extract all the lanes
    lines = cv2.HoughLinesP(isolated, 1, np.pi/180, 80, np.array([]), minLineLength=60, maxLineGap=15)
    averaged_lines = average(copy, lines)
    [black_lines,control_result] = display_lines(self,copy, averaged_lines)
    #taking wighted sum of original image and lane lines image
    lanes = cv2.addWeighted(copy, 0.8, black_lines, 1, 1)
    cv2.imshow("Lane_view", lanes)

    #Fill TwistStamped message control
    msg = TwistStamped()
    if(self.stop == 1): #check if aruco code "stop" is detected
       msg.twist.linear.x = 0.0
       print("Stop Rilevato")
    else : 
       msg.twist.linear.x = control_result[0]
    msg.twist.angular.z = control_result[1]
    msg.header.frame_id = "car1/base_footprint"
    self.publisher_.publish(msg)

    cv2.waitKey(1)
    
def grey(image):
  #convert to grayscale
    image = np.asarray(image)
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

  #Apply Gaussian Blur --> Reduce noise and smoothen image
def gauss(image):
    return cv2.GaussianBlur(image, (13, 13), 0)

  #outline the strongest gradients in the image --> this is where lines in the image are
def canny(image):
    edges = cv2.Canny(image,80,150,L2gradient = True)
    return edges

def region(image):
    dimensions = image.shape
    height = dimensions[0]
    width = dimensions[1] 
    #isolate the gradients that correspond to the lane lines
    triangle = np.array([
                       [(0, height),(0, int(height*0.7)), (width//2, int(height*0.55)),(width, int(height*0.7)), (width, height)]
                       ])
    #create a black image with the same dimensions as original image
    mask = np.zeros_like(image)
    #create a mask (triangle that isolates the region of interest in our image)
    mask = cv2.fillPoly(mask,pts = [triangle],color=(255, 255, 255))
    mask = cv2.bitwise_and(image, mask)
    return mask

def display_lines(self, image, lines):
    lines_image = np.zeros_like(image)
    dimensions = image.shape
    height = dimensions[0]
    width = dimensions[1] 
    offset = int((width/100)*(2.5))
    #make sure array isn't empty
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            print(line)
            #draw lines on a black image
            cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    if len(lines)>1:
      [x,y] = intersections(lines)
      control_result = control_law(self,x,offset,width) #call the control law for lane keeping
      cv2.circle(image,(x,y) ,5, (0,0,255), 5)
      cv2.line(image,(width//2 - offset,0),(width//2 - offset,480),(0,0,255),3,8,0)
      cv2.line(image,(width//2 + offset,0),(width//2 + offset,480),(0,0,255),3,8,0)
      #cv2.circle(image,(360+18,240) ,5, (0,0,255), 5)
    return [lines_image,control_result]

def control_law(self,x,offset,width):
   #apply a Proportional control law for lane keeping
   kc = self.get_parameter('proportional_kc').get_parameter_value().double_value
   velocity = self.get_parameter('linear_velocity').get_parameter_value().double_value
   #dead zone steering wheel
   if ((x>(width//2 - offset)) and (x<(width//2 + offset))):
      print("Dritto")
      steering_angle = float(0)
   else: 
      new_x = x-width//2 #shift the zero value to the center of the screen
      steering_angle = kc*new_x
      print(steering_angle)
   law = [velocity,float(steering_angle)]
   return law
   

def intersections(lines):
    x1_a, y1_a, x2_a, y2_a = lines[0]
    x1_b, y1_b, x2_b, y2_b = lines[1]
    m1 = (y1_a - y2_a)/(x1_a-x2_a)
    q1 = (x1_a * y2_a - x2_a*y1_a)/(x1_a-x2_a)
    m2 = (y1_b - y2_b)/(x1_b-x2_b)
    q2 = (x1_b * y2_b - x2_b*y1_b)/(x1_b-x2_b)
    x = int((q2-q1)//(m1-m2))
    y = int((m1*x)+q1)
    return [x,y]

def average(image, lines):
    left = []
    right = []

    if lines is not None:
      for line in lines:
        #print(line)
        x1, y1, x2, y2 = line.reshape(4)
        #fit line to points, return slope and y-int
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        print(parameters)
        slope = parameters[0]
        y_int = parameters[1]
        #lines on the right have positive slope, and lines on the left have neg slope
        if slope < 0 :
            left.append((slope, y_int))
        else:
            right.append((slope, y_int))

    #takes average among all the columns (column0: slope, column1: y_int)
    right_avg = np.average(right, axis=0)
    left_avg = np.average(left, axis=0)
    #create lines based on averages calculates
    left_line = make_points(image, left_avg)
    right_line = make_points(image, right_avg)
    return np.array([left_line, right_line])

def make_points(image, average):
    print(average)
    #slope, y_int = average
    try:
      slope, y_int = average
    except TypeError:
      slope, y_int = 1,1
    y1 = image.shape[0]
    #how long we want our lines to be --> 3/5 the size of the image
    y2 = int(y1 * (3/5))
    #determine algebraically
    x1 = int((y1 - y_int) // slope)
    x2 = int((y2 - y_int) // slope)
    return np.array([x1, y1, x2, y2])
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  lane_subscriber = LaneSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(lane_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  lane_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()