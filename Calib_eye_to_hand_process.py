
from pypylon import pylon
import cv2 as cv
import numpy as np
from math import pi, sin, cos
import threading
import time
import socket
import os
from matplotlib import pyplot as plt

# Create Camera Basler grab the images
class CameraBasler():
    def __init__(self):
        self.image = 1
        self.desired_model = "acA2040-25gc"
        self._tlFactory = pylon.TlFactory.GetInstance()
        self.devices = self._tlFactory.EnumerateDevices()
        self.k = 1
        for dev_info in self.devices:
            print("Device model:", dev_info.GetModelName())
        self.device = None
        for dev_info in self.devices:
            if dev_info.GetModelName() == self.desired_model:
                self.device = dev_info
                break
        if self.device is not None:
            self.flag_cam_ex = 1
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(self.device))
            self.camera.Open()
            # self.camera.Width.SetValue(820)
            # self.camera.Height.SetValue(820)
            
        else:
            print("Can't find the device:", self.desired_model)
            self.flag_cam_ex = 0   
    def GrabImg(self):
        # Grabing Continusely (video) with minimal delay
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
        self.converter = pylon.ImageFormatConverter() 
        
        prev_time=0
        new_time=0

        # converting to opencv bgr format
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        self.camera.ExposureTimeAbs.SetValue(50000)
        flag_collecting=False
        images_collected=40
        images_required=42
        directory = "D:/Do_an_tot_nghiep/Calib_camera/Calib_process/Eye_to_hand_calibration"
        while self.camera.IsGrabbing():
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grabResult.GrabSucceeded():
            # Access the image data
                image = self.converter.Convert(grabResult)
                self.frame = image.GetArray()
                new_time=time.time()
                fps=1/(new_time-prev_time)
                prev_time=new_time
                fps=int(fps)
                fps=str(fps)
                # cv.putText(self.frame,fps,(7,70),cv.FONT_HERSHEY_SIMPLEX,3,(100,255,0),3,cv.LINE_AA)
                # frame=cv.flip(img,1)
                height, width = self.frame.shape[:2]
                # print(height, width)
                if images_collected==images_required:
                    break
                if flag_collecting==True:
                    # sliced_frame=frame[0:0,480:640]
                    save_path=os.path.join(directory,"{}.jpg".format(images_collected+1))
                    cv.imwrite(save_path,self.frame)
                    flag_collecting=False
                    images_collected=images_collected+1
                
                cv.putText(self.frame,"Saved images: {}".format(images_collected),(400,25),cv.FONT_HERSHEY_SIMPLEX,0.7,(0,0,0),2)
                cv.imshow('Data_collection',self.frame)
                self.k = cv.waitKey(10)
                if self.k==ord('s'):
                    flag_collecting= not flag_collecting
                if self.k==ord('q'):
                    self.stop()
                    break
            grabResult.Release()
               
    def start(self):
        thread=threading.Thread(target=self.GrabImg, args=())
        thread.start()
        return self
    def stop (self):
        self.camera.StopGrabbing()
        cv.destroyAllWindows() 
        self.camera.Close()
class UR5_Robot():
    def __init__(self):
        self.SERVER = socket.gethostbyname(socket.gethostname())        #get the IP of the SERVER
        print(self.SERVER)
        self.PORT = 5050
        self.ADDR=(self.SERVER,self.PORT)
    def read_pos_from_robot(self):    
        print ("Starting Program")
        count = 0
        R_matrix = []
        t_matrix = []
        while (count < 1000):
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(self.ADDR)     # Bind to the port 
            s.listen(5) # Now wait for client connection.
            connection, address = s.accept() # Establish connection with client.
            try:
                while count < 40:
                    print("Receive pose of robot")
                    msg = connection.recv(1024).decode("utf-8")
                    time.sleep(1)
                    if msg == "Pose_of_robot":
                        msg_pose = connection.recv(1024).decode("utf-8")
                        msg_pose = self.convert_pose(msg_pose)
                        self.CurrentPos = msg_pose
                        R_pos_matrix , t_pos_matrix = self.read_R_t()
                        R_matrix.append(R_pos_matrix)
                        t_matrix.append(t_pos_matrix)
                        print("Pose_of_robot: ", msg_pose)
                        print("R_Pose matrix", R_pos_matrix )
                        print("t_Pose matrix", t_pos_matrix )
                        count = count + 1
                msg = connection.recv(1024).decode("utf-8")
                if msg == "Finish":
                    connection.close()
                    s.close()
                    print("Program finish")
                R_matrix = np.array(R_matrix)
                t_matrix = np.array(t_matrix)
                self.save("D:/Do_an_tot_nghiep/Calib_camera/Calib_process/R_Calib_eye_to_hand.txt", R_matrix)
                self.save("D:/Do_an_tot_nghiep/Calib_camera/Calib_process/t_Calib_eye_to_hand.txt", t_matrix)

            except socket.error as socketerror:
                print(count)
            # self.save("D:/Do_an_tot_nghiep/Calib_camera/Calib_process/R_Calib_eye_to_hand.txt", R_matrix)
    def convert_pose(self, msg_pose):
        msg_pose = msg_pose[2:-2]
        msg_pose = np.fromstring(msg_pose, dtype=float, sep= ",")
        return msg_pose
    def RPY2mtrx(self):
        Rx = self.CurrentPos[3]
        Ry = self.CurrentPos[4]
        Rz = self.CurrentPos[5] 
        return np.array([[cos(Rz) * cos(Ry), cos(Rz) * sin(Ry) * sin(Rx) - sin(Rz) * cos(Rx),
                          sin(Rz) * sin(Rx) + cos(Rz) * sin(Ry) * cos(Rx)],
                         [sin(Rz) * cos(Ry), cos(Rz) * cos(Rx) + sin(Rz) * sin(Ry) * sin(Rx),
                          sin(Rz) * sin(Ry) * cos(Rx) - cos(Rz) * sin(Rx)],
                         [-sin(Ry), cos(Ry) * sin(Rx), cos(Ry) * cos(Rx)]])
    def read_R_t(self):
        R = self.RPY2mtrx()
        t = np.array([self.CurrentPos[0], self.CurrentPos[1], self.CurrentPos[2]]).reshape(3,1)
        return R, t
    def save(self, path, mtx):
        cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
        cv_file.write("Rotation_matrix", mtx)
        cv_file.release()

class eye2handCalib_process():
    def __init__(self):
        self.R_path = 'D:/Do_an_tot_nghiep/Calib_camera/Calib_process/R_Calib_eye_to_hand.txt'
        self.t_path = 'D:/Do_an_tot_nghiep/Calib_camera/Calib_process/t_Calib_eye_to_hand.txt'
    def load_pos_R(self, path):
        m = []
        cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
        a = cv_file.getNode("Rotation_matrix").mat()
        cv_file.release()
        for i in a:
            m.append(i)
        return m
    def load_pos_t(self, path):
        m = []
        cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
        a = cv_file.getNode("Rotation_matrix").mat()
        a = np.multiply(a,1000)
        cv_file.release()
        for i in a:
            m.append(i)
        return m
    def load_intrinsic_matrix(self, path):
        """ Loads camera matrix and distortion coefficients. """
        # FILE_STORAGE_READ
        cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
        # note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        camera_matrix = cv_file.getNode("Intrinsic_matrix").mat()
        cv_file.release()
        return camera_matrix
    def load_dist_matrix(self, path):
        """ Loads camera matrix and distortion coefficients. """
        # FILE_STORAGE_READ
        cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
        # note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        dist_matrix = cv_file.getNode("Distortion_matrix").mat()
        cv_file.release()
        return dist_matrix
    def convert_to_Homogeneous(self, R, t):
        c = np.array([[0, 0, 0, 1]])
        d = np.concatenate((R, t), axis = 1)
        homogeneous_matrix = np.concatenate((d, c))
        return homogeneous_matrix
    def create_Worldpoint(self, nRow, nColumn, square_size):
        # Define the world coordinates for 3D points
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
        worldPoint = np.zeros((nColumn * nRow, 3), np.float32)
        worldPoint[:, :2] = np.mgrid[0:nColumn, 0:nRow].T.reshape(-1, 2)
        worldPoint = worldPoint * square_size  # Create real world coords. Use your metric.
        return worldPoint
    def start(self):
        thread=threading.Thread(target=self.eye2HandCalibrate, args=())
        thread.start()
        return self
    def coordinate(self,variable, position, rotation, coordinateNo):
        xvecx = position[0][0]
        yvecx = position[1][0]
        zvecx = position[2][0]
        uvecx = rotation[0][0]
        vvecx = rotation[1][0]
        wvecx = rotation[2][0]

        xvecy = position[0][0]
        yvecy = position[1][0]
        zvecy = position[2][0]
        uvecy = rotation[0][1]
        vvecy = rotation[1][1]
        wvecy = rotation[2][1]

        xvecz = position[0][0]
        yvecz = position[1][0]
        zvecz = position[2][0]
        uvecz = rotation[0][2]
        vvecz = rotation[1][2]
        wvecz = rotation[2][2]

        return variable.quiver(xvecx, yvecx, zvecx, uvecx, vvecx, wvecx, length=50, color='r', normalize=True),\
            variable.quiver(xvecy, yvecy, zvecy, uvecy, vvecy, wvecy, length=50, color='g', normalize=True),\
            variable.quiver(xvecz, yvecz, zvecz, uvecz, vvecz, wvecz, length=50, color='b', normalize=True),\
            variable.text(position[0][0] , position[1][0], position[2][0], coordinateNo, fontsize=10)
    def split_homo(self,mat):
        r = mat[0:3, 0:3]
        rot =[]
        for i in range (3):
            rot.append(r[i])
        t = mat[0:3, 3]
        trans =[[t[0]], [t[1]], [t[2]]]
        return rot, trans
    def show_coordinate(self,mat):
        rot, trans = self.split_homo(mat)
        fig = plt.figure()
        ax = plt.axes(projection="3d")
        # ax.scatter(X, Y, Z, c='r' , marker = '.')
        ax.set_xlabel('X - axis')
        ax.set_ylabel('Y - axis')
        ax.set_zlabel('Z - axis')
        self.coordinate(ax, [[0], [0], [0]], [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "  Base Coordinate")
        self.coordinate(ax, trans, rot, " Camera Coordinate")
        plt.show()
    def eye2HandCalibrate(self):
        R_gripper2base = self.load_pos_R(self.R_path)
        t_gripper2base = self.load_pos_t(self.t_path)
        R_base2gripper = []
        t_base2gripper = []
        print(t_gripper2base)
        print("\n HAND-EYE CALIBRATING _____________________________________________")
        start = time.time()
        for i in R_gripper2base:
            matrix = np.transpose(i)
            R_base2gripper.append(matrix)
        k=0
        for j in t_gripper2base:
            vec = -np.dot(R_base2gripper[k], j)
            t_base2gripper.append(vec)
            k+=1
        
        
        R_target2cam = []
        t_target2cam = []
        # Start calibrating
        intrinsic = self.load_intrinsic_matrix("D:/Do_an_tot_nghiep/Calib_camera/Calib_process/Intrinsic_matrix.txt")
        dist = self.load_dist_matrix("D:/Do_an_tot_nghiep/Calib_camera/Calib_process/Distortion.txt")
        nRow = 11
        nColumn = 18
        square_size = 13
        WorldPoint = self.create_Worldpoint(11,18,13)
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 3000, 0.00001)
        find_chessboard_flags = cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FILTER_QUADS + cv.CALIB_CB_NORMALIZE_IMAGE
        worldPointList = []
        imgPointList = []
        for i in range(1,40):
            print(i)
            img_var = cv.imread('D:/Do_an_tot_nghiep/Calib_camera/Calib_process/Eye_to_hand_calibration/{}.jpg'.format(i))
            gray = cv.cvtColor(img_var,cv.COLOR_BGR2GRAY)
            # Find the chess board corners
            # If desired number of corners are found in the image then cornersFound = true
            cornersFound, cornersOrg = cv.findChessboardCorners(gray, (nColumn,nRow), None,find_chessboard_flags)
            if cornersFound == True:
                # refining pixel coordinates for given 2d points.
                cornersRefined = cv.cornerSubPix(gray, cornersOrg, (11,11),(-1,-1), criteria)
                cv.drawChessboardCorners(img_var, (nColumn,nRow), cornersRefined, cornersFound )
                # img_var = cv.resize(img_var,(640,480),interpolation=cv.INTER_AREA)
                # cv.imshow('test',img_var)
                # cv.waitKey(0)
                # cv.destroyAllWindows()
                # Return the Rotation and the Translation VECTORS that transform a
                # 3D point expressed in the object coordinate frame to the camera coordinate frame
                retval, rvec, tvec = cv.solvePnP(WorldPoint, cornersRefined, intrinsic, dist, flags=cv.SOLVEPNP_ITERATIVE)
                rotation_matrix = np.zeros(shape=(3, 3))
                # Convert a rotation matrix to a rotation vector or vice versa
                cv.Rodrigues(rvec,rotation_matrix)
                R_target2cam.append(rotation_matrix)
                t_target2cam.append(tvec)
                
            else:
                print("CHECKERBOARD NOT DETECTED!")
        
        # Use Daniilidis due to better accuracy than Tsai
        # Do not use Tsai due to poor accuracy
        # R_cam2gripper, t_cam2gripper = cv.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=cv.CALIB_HAND_EYE_DANIILIDIS)
        R_cam2base, t_cam2base = cv.calibrateHandEye(R_base2gripper, t_base2gripper, R_target2cam, t_target2cam,method = cv.CALIB_HAND_EYE_TSAI)
        # R_cam2gripper, t_cam2gripper = cv.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method = cv.CALIB_HAND_EYE_TSAI)
        end = time.time()
        print("\t:: TRANSFORMATION MATRIX OF CAMERA TO ROBOT BASE:")
        print("\t:: Rotation R:\n", R_cam2base)
        print("\t:: Translation t:\n", t_cam2base)
        H = self.convert_to_Homogeneous(R_cam2base,t_cam2base)
        # save_hand_eye_mat(H)
        
        # t_target2base = np.multiply(H,t_target2cam)
        # print(t_target2base)
        print(f'\t:: Homogenous transformation:\n{H}')
        print("\t:: Time consumed: %.2lf s\t\t" %(end-start))
        print("_______________________________________ HAND-EYE CALIBRATION DONE |")
        self.show_coordinate(H)
        return H

if __name__ == "__main__":
    # VisionSysem = CameraBasler()
    # VisionSysem.start()
    # UR5 = UR5_Robot()
    # UR5.read_pos_from_robot()
    e2h_calib = eye2handCalib_process()
    e2h_calib.start()
    
   
    
    


    

    
        
