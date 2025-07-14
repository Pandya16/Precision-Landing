# Script for Precision landing and landing target messages using "Pymavlink"
# Set correct marker size, camera callibration

import cv2
import cv2.aruco as aruco
import numpy as np
import os
import pickle
from scipy.spatial.transform import Rotation as R
import time
from pymavlink import mavutil

master = mavutil.mavlink_connection(
    device = '/dev/serial0',
    baud = 921600,
    source_system = 1,
    source_component = 191
)

with open("cameraMatrix.pkl", "rb") as f:
    cameraMatrix = pickle.load(f)

with open("dist.pkl", "rb") as f:
    distCoeffs = pickle.load(f)

url = "http://100.118.194.50:4747/video"
MARKER_SIZE = 0.20


fx = cameraMatrix[0, 0]
fy = cameraMatrix[1, 1]
cx = cameraMatrix[0, 2]
cy = cameraMatrix[1, 2]

def send_landing_target_message(unixtime, targetno, frame, angle_x, angle_y, distance, size_x, size_y):
    master.mav.landing_target_send(
        unixtime,
        targetno,
        frame,
        angle_x,
        angle_y,
        distance,
        size_x,
        size_y,
        )
    print("Mavlink messages sent")

        
def findArucoMarkers(img, markerSize=6, totalMarkers=250, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.getPredefinedDictionary(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, bboxs)

    return [bboxs, ids]

freq_send = 30

def main():
    cap = cv2.VideoCapture(url)
    time_0 = time.time()
    while True:
        success, img = cap.read()
        arucoFound = findArucoMarkers(img)
        bboxs, ids = arucoFound
        
        if bboxs:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(bboxs, MARKER_SIZE, cameraMatrix, distCoeffs)

            for i in range(len(ids)):
                unixTime = int(time.time()*1000000)
                position_valid = 1
                x_aruco = tvec[i][0][0]
                y_aruco = tvec[i][0][1]
                z_aruco = tvec[i][0][2]

                x_frd = y_aruco
                y_frd = x_aruco
                z_frd = z_aruco

                rot_mat, _ = cv2.Rodrigues(rvec[i])
                quat = R.from_matrix(rot_mat).as_quat()
                quat_wxyz = np.roll(quat, 1)

                cv2.drawFrameAxes(img, cameraMatrix, distCoeffs, rvec[i], tvec[i], 0.1)

                corners = bboxs[i][0]
                center_u = int(np.mean(corners[:, 0]))
                center_v = int(np.mean(corners[:, 1]))

                width_px = np.linalg.norm(corners[0] - corners[1])
                height_px = np.linalg.norm(corners[1] - corners[2])

                angle_x = np.arctan2((center_u - cx), fx)
                angle_y = np.arctan2((center_v - cy), fy)

                size_x = 2 * np.arctan2(width_px / 2, fx)
                size_y = 2 * np.arctan2(height_px / 2, fy)

                distance = np.linalg.norm(tvec[i][0])
                
                print(distance)

                if time.time() >= time_0 + 1.0/freq_send:
                    time_0 = time.time()
                    send_landing_target_message(unixTime, 23, 12, float(angle_x), float(angle_y), float(distance), float(size_x), float(size_y))
                                
        # cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()

