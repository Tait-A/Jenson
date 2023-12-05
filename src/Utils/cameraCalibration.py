# Calibration of the Camera, to calculate the intrinsic and extrinsic parameters
# To compute the calibration matrix


# Read in images from ../Output/Chess

import os
import cv2
import numpy as np

# Specify the directory path
directory = os.path.join('/' + os.path.join(*os.path.dirname(__file__).split('/')[0:-1]), "Output/Chess")


# Get the file paths of all the images in the directory
image_paths = [os.path.join(directory, "chess"+str(i+1)+".jpg") for i in range(5)]

# Read each image
images = []
for image_path in image_paths:
    image = cv2.imread(image_path)
    if image is not None:
        images.append(image)

chessboard = (7,7)



# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = []

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((1, chessboard[0] * chessboard[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:chessboard[0], 0:chessboard[1]].T.reshape(-1, 2)
prev_img_shape = None

for image in images:
    retval, corners = cv2.findChessboardCorners(image, chessboard, flags = cv2.CALIB_USE_INTRINSIC_GUESS)
    print(retval)

    if retval == True:
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(image, corners, (11,11),(-1,-1), criteria)
         
        imgpoints.append(corners2)
 
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, chessboard, corners2, retval)
     
    cv2.imshow('img',img)
    cv2.waitKey(0)
 
cv2.destroyAllWindows()
 
h,w = img.shape[:2]

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image.shape[::-1], None, None)
 
print("Camera matrix : \n")
print(mtx)
print("dist : \n")
print(dist)
print("rvecs : \n")
print(rvecs)
print("tvecs : \n")
print(tvecs)
