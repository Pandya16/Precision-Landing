import cv2
import cv2.aruco as aruco
import numpy as np
import pickle
from scipy.spatial.transform import Rotation as R 
import time
import matplotlib.pyplot as plt

 
url = "http://192.168.68.114:4747/video"
cap = cv2.VideoCapture(url)

while True:    
    success, img = cap.read()
    cv2.imshow("Video", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break