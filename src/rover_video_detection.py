#!/usr/bin/env python
import cv2 as cv
import numpy as np

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
"DICT_4X4_50": cv.aruco.DICT_4X4_50,
"DICT_4X4_100": cv.aruco.DICT_4X4_100,
"DICT_4X4_250": cv.aruco.DICT_4X4_250,
"DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
"DICT_5X5_50": cv.aruco.DICT_5X5_50,
"DICT_5X5_100": cv.aruco.DICT_5X5_100,
"DICT_5X5_250": cv.aruco.DICT_5X5_250,
"DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
"DICT_6X6_50": cv.aruco.DICT_6X6_50,
"DICT_6X6_100": cv.aruco.DICT_6X6_100,
"DICT_6X6_250": cv.aruco.DICT_6X6_250,
"DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
"DICT_7X7_50": cv.aruco.DICT_7X7_50,
"DICT_7X7_100": cv.aruco.DICT_7X7_100,
"DICT_7X7_250": cv.aruco.DICT_7X7_250,
"DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
"DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL
}

# load the aruco dictionary and grab the aruco parameters
arucoDict = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_100)
arucoParams = cv.aruco.DetectorParameters_create()

class SubImage(object):

    def __init__(self):
    
      self.cap = cv.VideoCapture("/home/mustafauludag/output.mp4")
      fourcc = cv.VideoWriter_fourcc(*'XVID')
      self.out = cv.VideoWriter('../videos/rover_video.avi', fourcc, 60.0, (640, 480))

    def camera_callback(self):
      while 1:
        success,frame = self.cap.read()
        
        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = frame.shape
        

        
        # descentre = 160
        # rows_to_watch = 60
        # crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]

        # error_x = cx - width / 2;
        # speed_cmd = Twist();
        # speed_cmd.linear.x = 0.2;
        # speed_cmd.angular.z = -error_x / 100;
        
        # self.speed_pub.publish(speed_cmd)

        # detect ArUCo markers in the input frame
        (corners,ids,rejected)=cv.aruco.detectMarkers(frame,
                arucoDict,parameters=arucoParams)

        # verify 'at least' one ArUCo marker was detected
        if len(corners)>0:
            ids=ids.flatten()
            
            for (markerCorner, markerID) in zip(corners,ids):
                
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
        

                # draw the bounding box of the ArUCo detection
                cv.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                markerCenter = (cX,cY)
                fX = frame.shape[1]
                fY = frame.shape[0]
                frameCenter = (int(fX/2),int(fY/2))
                cv.circle(frame, markerCenter, 4, (0, 0, 255), -1)
                
                
                oX = int((topLeft[0] + topRight[0]) / 2.0)
                oY = int((topLeft[1] + topRight[1]) / 2.0)
                
                oX1 = int((bottomRight[0] + topRight[0]) / 2.0)
                oY1 = int((bottomRight[1] + topRight[1]) / 2.0)
                
                
                cv.line(frame, markerCenter, (oX, oY),(0, 0, 255), 2)
                
                cv.line(frame, markerCenter, (oX1, oY1),(255, 0, 0), 2)
                dist= np.sqrt(np.power((int(fX/2)-cX),2) + np.power((int(fY/2)-cY),2))
                
                # if dist <25:
                #     cv.line(frame, markerCenter, frameCenter, (0, 255, 0), 3)
                # elif 25< dist <75:
                #     cv.line(frame, markerCenter, frameCenter, (0, 255, 255), 3)
                # else: 
                #     cv.line(frame, markerCenter, frameCenter, (0, 0, 255), 3)
                    
                # draw the ArUco marker ID on the frame
                if (markerID == 42):
                    cv.putText(frame, 'charger',
                        (topLeft[0], topLeft[1] - 15),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
                    
                if (markerID == 87):
                    cv.putText(frame, 'drop',
                        (topLeft[0], topLeft[1] - 15),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
                    
                if (markerID == 66):
                    cv.putText(frame, 'patient',
                        (topLeft[0], topLeft[1] - 15),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)

        
        # show the output frame
        self.out.write(frame)
        cv.imshow("Frame", frame)
        if cv.waitKey(10) & 0xFF == ord('q'):
          break
		      
                       
        
        


def main():
    img = SubImage()
    try:
      img.camera_callback()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()