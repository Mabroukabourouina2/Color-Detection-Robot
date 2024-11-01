from pyniryo import*
client=NiryoRobot("10.10.10.10")
client.calibrate(CalibrateMode.AUTO)

place_ob=PoseObject(x=0.176, y=-0.035 ,z=0.359,roll=-0.317,pitch=1.425,yaw=-0.011,)
place_depart=PoseObject(x=0.262, y=-0.165 ,z=0.106,roll=2.717,pitch=1.438,yaw=2.682,)
place_ob=PoseObject(x=0.176, y=-0.035 ,z=0.359,roll=-0.317,pitch=1.425,yaw=-0.011,)

place_pose=PoseObject(x=0.238, y=0.253 ,z=0.113,roll=2.415,pitch=1.537,yaw=2.613,)
client.update_tool()
client.grasp_with_tool()
client.release_with_tool()

# place_pose=PoseObject(x=-0.039, y=0.309 ,z=0.215,roll=-0.039,pitch=1.486,yaw=1.486,)
client.move_pose(place_ob.to_list())
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






# client.move_pose(place_depart.to_list())
# client.grasp_with_tool()

# client.move_pose(place_ob.to_list())
# client.move_pose(place_pose.to_list())
client.grasp_with_tool()
client.release_with_tool()
client.move_pose(place_ob.to_list())

