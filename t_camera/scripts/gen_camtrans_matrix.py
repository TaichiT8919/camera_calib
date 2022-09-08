#!usr/bin/env python3

import os
import cv2
import time
import pyrealsense2 as rs
import numpy as np
from matplotlib.pyplot import get

save_path = "/home/rllab/catkin_ws/src/t_hitz_crane/t_camera/config/transform_matrix_14.csv"
CAMERA_ID_1 = "017322074156"
CAMERA_ID_2 = "950122070596"
# CAMERA_ID_3 = "017322073307"
# CAMERA_ID_4 = "017322074043"
MEASUREMENT_TIME = 2.0
MEASUREMENT_NUM = 20

ID1 = CAMERA_ID_1
ID2 = CAMERA_ID_2


def get_marker_pos(aruco, dictionary, align, pipeline):
    """
    マーカーの３次元座標を取得する。
    """
    cv2.namedWindow('RealsenseImage', cv2.WINDOW_AUTOSIZE)

    marker_dic = {0:[], 1:[], 2:[], 3:[]}

    t = time.time()
    while ((time.time() - t) < MEASUREMENT_TIME):
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, dictionary)

        if len(corners)!=0:
            # print(corners)
            # print(ids)

            for index in range(len(corners)):
                point = np.average(corners[index][0], axis=0)
                # print(point)
                depth = depth_frame.get_distance(point[0], point[1])
                point = np.append(point,depth)
                if depth!=0:
                    x=point[0]
                    y=point[1]
                    z=point[2]
                    x,y,z=rs.rs2_deproject_pixel_to_point(color_intrinsics, [x, y], z)

                    marker_dic[ids[index, 0]].append([x,y,z,1])
                    # print(f"point({ids[index]}):",x,y,z)

        aruco.drawDetectedMarkers(color_image, corners, ids, (0,255,0))

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))

        cv2.imshow("RealsenseImage",images)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            id0 = np.mean(np.array(marker_dic[0]), axis=0).reshape([1, 4])
            id1 = np.mean(np.array(marker_dic[1]), axis=0).reshape([1, 4])
            id2 = np.mean(np.array(marker_dic[2]), axis=0).reshape([1, 4])
            id3 = np.mean(np.array(marker_dic[3]), axis=0).reshape([1, 4])
            print(id1)
            break
    
    id0 = np.mean(np.array(marker_dic[0]), axis=0).reshape([1, 4])
    id1 = np.mean(np.array(marker_dic[1]), axis=0).reshape([1, 4])
    id2 = np.mean(np.array(marker_dic[2]), axis=0).reshape([1, 4])
    id3 = np.mean(np.array(marker_dic[3]), axis=0).reshape([1, 4])

    return [id0, id1, id2, id3]

# calculation transform matrix
def calc_trans_matrix(x1, x2):
    diag = 1e-10 * np.identity(x1.shape[0])

    term1 = np.dot(x1, x2.T)
    inv = np.linalg.inv(np.dot(x2, x2.T) + diag)
    pinv = np.linalg.pinv(x2)
    print(pinv.shape)
    # return np.dot(term1, inv)
    return np.dot(x1, pinv)


if __name__ == "__main__":
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    align_to = rs.stream.color
    align = rs.align(align_to)

    # camera1 setting
    pipeline_1 = rs.pipeline()
    config_1 = rs.config()
    config_1.enable_device(ID1)
    config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # camera2 setting
    pipeline_2 = rs.pipeline()
    config_2 = rs.config()
    config_2.enable_device(ID2)
    config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    markers_pos_1 = []
    markers_pos_2 = []

    pipeline_1.start(config_1)
    pipeline_2.start(config_2)

    for n in range(MEASUREMENT_NUM):
        print("========================================================")
        print(f"Trial: {n+1}/{MEASUREMENT_NUM}")
        v = input("After changing the marker position, press the button")
        loop_flag = True

        while 1:
            try:
                # camera1 streaming starts
                print("Start camera1 streaming")
                get_pos1 = get_marker_pos(aruco, dictionary, align, pipeline_1)

                # camera2 streaming starts
                print("Start camera2 streaming")
                get_pos2 = get_marker_pos(aruco, dictionary, align, pipeline_2)

            except:
                v = input("After changing the marker position, press Enter...")

            else:
                markers_pos_1 += get_pos1
                markers_pos_2 += get_pos2
                break


        # print(markers_pos_1)
        # print(markers_pos_2)

    # Solving transform matrix for each marker
    # trans0 = calc_trans_matrix(markers_pos_1[0].T, markers_pos_2[0].T)
    # trans1 = calc_trans_matrix(markers_pos_1[1].T, markers_pos_2[1].T)
    # trans2 = calc_trans_matrix(markers_pos_1[2].T, markers_pos_2[2].T)
    # trans3 = calc_trans_matrix(markers_pos_1[3].T, markers_pos_2[3].T)
    # trans_matrix = np.mean(np.dstack([trans0, trans1, trans2, trans3]), axis=2)

    # Solving transform matrix
    markers_pos_array1 = np.vstack(markers_pos_1)
    markers_pos_array2 = np.vstack(markers_pos_2)
    trans_matrix = calc_trans_matrix(markers_pos_array1.T, markers_pos_array2.T)

    trans_pos1 = np.dot(trans_matrix, markers_pos_2[0].T).T
    trans_pos2 = np.dot(trans_matrix, markers_pos_2[1].T).T
    trans_pos3 = np.dot(trans_matrix, markers_pos_2[2].T).T
    trans_pos4 = np.dot(trans_matrix, markers_pos_2[3].T).T

    print("\n############### CHECK ##################")
    print("original matrix:\n", markers_pos_1[0])
    print("transformed matrix:\n", trans_pos1)

    print("original matrix:\n", markers_pos_1[1])
    print("transformed matrix:\n", trans_pos2)

    print("original matrix:\n", markers_pos_1[2])
    print("transformed matrix:\n", trans_pos3)

    print("original matrix:\n", markers_pos_1[3])
    print("transformed matrix:\n", trans_pos4)

    # dif = []
    # dif.append((markers_pos_1[0] - trans_pos1)[0])
    # dif.append((markers_pos_1[1] - trans_pos2)[0])
    # dif.append((markers_pos_1[2] - trans_pos3)[0])
    # dif.append((markers_pos_1[3] - trans_pos4)[0])

    # dif = np.mean(np.array(dif), axis=1)
    # trans_matrix[:3,3:] = (trans_matrix[:3,3:].T + dif[:3]).T

    # print("\n############### CHECK ##################")
    # print("original matrix:\n", markers_pos_1[0])
    # print("transformed matrix:\n", np.dot(trans_matrix, markers_pos_2[0].T).T)

    # print("original matrix:\n", markers_pos_1[1])
    # print("transformed matrix:\n", np.dot(trans_matrix, markers_pos_2[1].T).T)

    # print("original matrix:\n", markers_pos_1[2])
    # print("transformed matrix:\n", np.dot(trans_matrix, markers_pos_2[2].T).T)

    # print("original matrix:\n", markers_pos_1[3])
    # print("transformed matrix:\n", np.dot(trans_matrix, markers_pos_2[3].T).T)


    print("Saving transform matrix")
    # np.savetxt(os.path.join(dir_path = os.path.dirname(__file__), "config/transform_matrix.csv"), trans_matrix)
    np.savetxt(save_path, trans_matrix)
