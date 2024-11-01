from pyniryo import*
import cv2
import time
from utils import robot_connection, take_workspace_img
from pyniryo import PoseObject, ColorHSV
from tracker import *

tracker = Tracker()
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

client = robot_connection(robot_ip_address)
client.move_pose(*observation_pose.to_list())

object_count = []
offset = 30
id = client.set_conveyor()
client.run_conveyor(id, 50, ConveyorDirection.BACKWARD)

try:
    while True:
        success, img = take_workspace_img(client)
        frame = cv2.resize(img, (400, 400))
        
        color_hsv = ColorHSV.RED
        mask = debug_threshold_color(frame, color_hsv)
        if len(mask.shape) == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        res_frame = frame.copy()
        list = []
        cv2.line(res_frame, (200 + offset, 50), (200 + offset, 350), (0, 0, 255), 2)
        
        if contours:
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(res_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                list.append([x, y, w, h])
                
            print(f"Detected {len(contours)} red object(s).")
        
        bbox_id = tracker.update(list)
        for bbox in bbox_id:
            x1, y1, w1, h1, id = bbox
            cx = int(x1 + w1 // 2)
            cy = int(y1 + h1 // 2)
            cv2.circle(res_frame, (cx, cy), 5, (0, 0, 255), -1) 
            cv2.putText(res_frame, f"id: {id}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
            
            if 200 < cx < 200 + offset:
                if object_count.count(id) == 0:
                    object_count.append(id)
                    cv2.line(res_frame, (200 + offset, 50), (200 + offset, 350), (0, 255, 0), 2)

        cv2.putText(res_frame, f"Counts: {len(object_count)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
        cv2.imshow("Red Detection", res_frame)
        
        if cv2.waitKey(1) == 27:
            break

finally:
    client.stop_conveyor(id)
    cv2.destroyAllWindows()
    client.close_connection()
