import numpy as np
# import matplotlib.pyplot as plt  
from pyniryo import *
ip_address="10.10.10.10"
# take an image of the workspace
def take_workspace_img(client):
    mtx, dist = client.get_camera_intrinsics()
    while 1:
        img_compressed = client.get_img_compressed()
        img_raw = uncompress_image(img_compressed)
        img = undistort_image(img_raw, mtx, dist)
        img_work = extract_img_workspace(img, workspace_ratio=1)
        if img_work is not None:
            break

        return False, img

    return True, img_work

# connect to  the robot
def robot_connection(ip_address):
    client = NiryoRobot(ip_address)
    try:
        client.calibrate(CalibrateMode.AUTO)
        client.update_tool()
    except NiryoRobotException:
        print("Calibration failed") 
    return client   
    