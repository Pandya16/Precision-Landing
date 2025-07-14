import cv2
import cv2.aruco as aruco
import numpy as np
import os
import pickle
from scipy.spatial.transform import Rotation as R
import time
from pymavlink import mavutil
from dronekit import VehicleMode, connect

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

url = "http://192.168.68.106:4747/video"
MARKER_SIZE = 0.06

fx = cameraMatrix[0, 0]
fy = cameraMatrix[1, 1]
cx = cameraMatrix[0, 2]
cy = cameraMatrix[1, 2]

def send_land_message(unixtime, targetno, frame, angle_x, angle_y, distance, size_x, size_y, x_m, y_m, z_m, q_m, land_type, pos_valid):
    msg = vehicle.message_factory.landing_target_encode(
        unixtime,
        targetno,
        frame,
        angle_x,
        angle_y,
        distance,
        size_x,
        size_y,
        x_m, 
        y_m, 
        z_m, 
        q_m,
        land_type,
        pos_valid
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def findArucoMarkers(img, markerSize=6, totalMarkers=250, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.getPredefinedDictionary(key)
    arucoParam = aruco.DetectorParameters()
    bboxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, bboxs)

    return [bboxs, ids]


def main():
    cap = cv2.VideoCapture(url)
    while True:
        success, img = cap.read()
        arucoFound = findArucoMarkers(img)
        bboxs, ids = arucoFound
        position_valid = 0

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
                
                send_land_message(unixTime, 23, 12, float(angle_x), float(angle_y), float(distance), 0, 0, 
                                    float(x_frd), float(y_frd), float(z_frd), quat_wxyz.tolist(), 2, position_valid)
                
        cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()

