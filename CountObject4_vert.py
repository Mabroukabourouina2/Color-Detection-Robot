"""
This counts objects based on ID using the function of the Tracker class.
"""
from pyniryo import*
import cv2
import time
from utils import robot_connection, take_workspace_img
from pyniryo import PoseObject, ObjectShape, ObjectColor, vision, Command, ColorHSV, MorphoType
from pyniryo import *
from PIL import Image
import numpy as np
from tracker import *

tracker=Tracker()
workspace = "workspace_name"
robot_ip_address = "10.10.10.10"

place_pose = PoseObject(
    x=-0.039, y=0.309, z=0.215,
    roll=0.215, pitch=1.486, yaw=1.486,
)

observation_pose = PoseObject(
    x=0.180, y=-0.002, z=0.276,
    roll=-2.508, pitch=1.485, yaw=-2.485,
)

# Connect to the robot
client = robot_connection(robot_ip_address)
client.move_pose(*observation_pose.to_list())

object_count = []  # Initialize object counter
tracked_objects = {}  # Dictionary to track objects with their states (tracked by object center)
offset=30  
id =client.set_conveyor()
client.run_conveyor(id,50,ConveyorDirection.BACKWARD)

try:
    while True:
        success, img = take_workspace_img(client)
        frame = cv2.resize(img, (400, 400))
        
        # Use a predefined ColorHSV value (e.g., GREEN)
        color_hsv = ColorHSV.GREEN
        
        # Threshold the color in the image
        mask = debug_threshold_color(frame, color_hsv)
        
        # Convert the mask to grayscale (binary image)
        if len(mask.shape) == 3:  # If the mask is in BGR or RGB format
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        
        # Find contours of the masked areas
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        res_frame = frame.copy()
        list=[]

        # Draw a horizontal line in the middle of the frame to visualize the "pass line"
        cv2.line(res_frame, (200+offset,50), (200+offset, 350), (0,0, 255), 2)
        # Process each contour
        if contours:
            for contour in contours:
                # Get the bounding box for each contour
                x, y, w, h = cv2.boundingRect(contour)
                # Draw the bounding box around the object
                cv2.rectangle(res_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
                list.append([x,y,w,h]) 
                
            print(f"Detected {len(contours)} object(s).")
        else:
            print("No object found matching the HSV color filter.")
        
       
        bbox_id=tracker.update(list)
        for bbox in bbox_id:
            x1,y1,w1,h1,id=bbox
            cx=int(x1+ w1//2)
            cy=int(y1+ h1//2)
            print(x1,y1,w1,h1,id)
            cv2.circle(res_frame,(cx,cy),5,RED,-1) 
            cv2.putText(res_frame, f"id: {id}", (x1,y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
            
            # Only track objects moving 
            if 200<cx <200+offset:
                if object_count.count(id)==0:
                    # Track the object based on its center position
                    object_count.append(id)
                    cv2.line(res_frame, (200+offset,50), (200+offset, 350), (0,255, 0), 2)


        # Display the object count on the frame
        cv2.putText(res_frame, f"Counts: {len(object_count)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
        cv2.imshow("mask and result", res_frame)
        
        key = cv2.waitKey(1)
        if key == 27:  # Press 'Esc' to exit
            break

finally:
    client.stop_conveyor(id)
    cv2.destroyAllWindows() 
    client.close_connection()
