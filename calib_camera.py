import cv2
import numpy as np
import os
import glob

# nRow = 26
# nColumn = 35
# square_size = 7
# termination criteria
find_chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FILTER_QUADS + cv2.CALIB_CB_NORMALIZE_IMAGE
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
calibrate_criteria = (cv2.TermCriteria_COUNT + cv2.TermCriteria_EPS, 500, 0.0001)

def calibrate(nRow, nColumn, square_size):
    # Define the world coordinates for 3D points
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    worldPoint = np.zeros((nColumn * nRow, 3), np.float32)
    worldPoint[:, :2] = np.mgrid[0:nColumn, 0:nRow].T.reshape(-1, 2)
    worldPoint = worldPoint * square_size  # Create real world coords. Use your metric.
    

    # Arrays to store object points and image points from all the images.
    worldPointList = []
    imgPointList = []

    images = glob.glob('D:/Do_an_tot_nghiep/yolov8/Calib_image/*.jpg')

    for i in range(1,101):
        print(i)
        img_var = cv2.imread('D:/Do_an_tot_nghiep/yolov8/Calib_image/{}.jpg'.format(i))
        gray = cv2.cvtColor(img_var,cv2.COLOR_BGR2GRAY)
        
        # Find the chess board corners
        # If desired number of corners are found in the image then cornersFound = true
        cornersFound, cornersOrg = cv2.findChessboardCorners(gray, (nColumn,nRow), None,find_chessboard_flags)
        
    #     for i in range(len(cornersOrg)):
    #         print(cornersOrg[i,0,])
    #         cv2.putText(img_var, f'({int(cornersOrg[i,0,0])}, {int(cornersOrg[i,0,1])})', (int(cornersOrg[i,0,0]), int(cornersOrg[i,0,1]) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
        
        if cornersFound == True:
            worldPointList.append(worldPoint)
            # refining pixel coordinates for given 2d points.
            cornersRefined = cv2.cornerSubPix(gray, cornersOrg, (11,11),(-1,-1), criteria)
    #         print(cornersRefined)
            imgPointList.append(cornersRefined)
            cv2.drawChessboardCorners(img_var, (nColumn,nRow), cornersRefined, cornersFound )
            img_var = cv2.resize(img_var,(640,480),interpolation=cv2.INTER_AREA)
    #         cv2.imshow('test',img_var)
    #         cv2.waitKey(0)
    #         cv2.destroyAllWindows()
        else:
            print("CHECKERBOARD NOT DETECTED!")
    
    flags = (cv2.CALIB_FIX_ASPECT_RATIO + cv2.CALIB_FIX_K3 + cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_PRINCIPAL_POINT)

    print("Calibrating...")
    repError, camMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(worldPointList, imgPointList, (2592,1944), None, cv2.CALIB_USE_INTRINSIC_GUESS, criteria=calibrate_criteria)
    # print("Camera matrix : \n",camMatrix)
    # print("dist : \n")
    # print(dist)
    # print("rvecs : \n")
    # print(rvecs)
    # print("tvecs : \n")
    # print(tvecs)

    fx = camMatrix[0][0]
    fy = camMatrix[1][1]
    cx = camMatrix[0][2]
    cy = camMatrix[1][2]
    print(fx)
    print(fy)
    print(cx)
    print(cy)
    return camMatrix, dist
def save_intrinsic(path, mtx):
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        cv_file.write("Intrinsic_matrix", mtx)
        cv_file.release()
def save_distortion(path, mtx):
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        cv_file.write("Distortion_matrix", mtx)
        cv_file.release()
if __name__ == "__main__":
    camMatrix, dist = calibrate(nColumn=18, nRow=11, square_size=13)
    save_camMatrix = save_intrinsic("D:/Do_an_tot_nghiep/Calib_camera/Calib_process/Intrinsic_matrix.txt",camMatrix)
    save_dist = save_distortion("D:/Do_an_tot_nghiep/Calib_camera/Calib_process/Distortion.txt",dist)
    