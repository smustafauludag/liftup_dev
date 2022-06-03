#!/usr/bin/env python
"""
Drone control with aruco markers
@author Sefer Mustafa Uludag
@mail smustafauludag@gmail.com
"""
from math import radians, cos, sin, asin, sqrt
import cv2 as cv
import numpy as np
import rospy
import time
import serial 
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from pymavlink import mavutil
from sensor_msgs.msg import Image

import bash_info_helper as sh

def Distance(p1 ,p2):
  ''' 
  Calculates the distance between two points 
  :param p1: array [x1,y1]
  :param p2: array [x2,y2]
  :returns:  distance
  '''
  dist= np.sqrt(np.power((p1[0]-p2[0]),2) + np.power((p1[1]-p2[1]),2))
  return dist


def DistanceGlobal(p1,p2):
  """
  Calculates distance between two global coordinates
  :param1: array [lat1,lon1]
  :param2: array [lat2,lon2]
  :returns: distance
  """
  lat1 = p1[0]
  lon1 = p1[1]
  lat2 = p2[0]
  lon2 = p2[1]

  # Haversine Formula
  dlon = lon2 - lon1
  dlat = lat2 - lat1
  a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
  c = 2 * asin(sqrt(a))
    
  # Radius of earth in kilometers. Use 3956 for miles
  r = 6371

  return(c*r)


def Clamp(var,n):
  """ 
  Puts the variable in a boundary [-n,var,n]
  :param var: variable
  :param n:   -+ boundary
  """
  if var >n: return n
  elif var < -n: return -n
  else: return var


def rotation_matrix_z(yaw):
  ''' Rotation in axix '''
  mat = np.array([[np.cos(yaw),-np.sin(yaw)],
                  [np.sin(yaw),np.cos(yaw)]])
  return mat


def Pixel2MeterXY(px,marker_area):
  return px*(0.2/np.sqrt(marker_area))


def Pixel2MeterZ(marker_area):
  return -35/np.sqrt(marker_area)


def QGCSendVoiceMessage(msg):
  """
  :param msg: string
  """
  qgc_connection = mavutil.mavlink_connection("udpout:localhost:14550",
                                              source_system=1)
  qgc_connection.mav.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_NOTICE, msg.encode())


class PyMavlink():
  '''Quadrotor navigation and communication class via pymavlink'''
  def __init__(self,dev):
    self.__DICT_DEVICE_ADRESS = {"USB" : ["/dev/ttyUSB0",57600],
                                 "GAZEBO_1" : ["udpin:localhost:14550",115200],
                                 "GAZEBO_2" : ["udpin:localhost:14560",115200],
                                 "GAZEBO_3" : ["udpin:localhost:14570",115200]}
    if dev not in self.__DICT_DEVICE_ADRESS.keys():
      sh.error("Wrong key for device adress")
    else:
      self._the_connection = mavutil.mavlink_connection(
        device=self.__DICT_DEVICE_ADRESS[dev][0],
        baud=self.__DICT_DEVICE_ADRESS[dev][1])
      sh.success("Device adress set done")

    self._the_connection.wait_heartbeat()
    sh.info("Heartbeat from system (system %u component %u)" % 
      (self._the_connection.target_system,
       self._the_connection.target_component))

    self.__DICT_QUAD_MODE = {
    'ACRO' :      1,
    'STABILIZE' : 0,
    'ALT_HOLD' :  2,
    'AUTO' :      3,
    'GUIDED' :    4,
    'LOITER' :    5,
    'RTL' :       6,
    'CIRCLE' :    7,
    'POSITION' :  8,
    'LAND' :      9,
    'OF_LOITER' : 10,
    'DRIFT' :     11,
    'SPORT' :     13,
    'FLIP' :      14,
    'AUTOTUNE' :  15,
    'POSHOLD' :   16,
    'BRAKE' :     17,
    'THROW' :     18,
    'AVOID_ADSB' :19,
    'GUIDED_NOGPS':20,
    'SMART_RTL' :  21,
    'FLOWHOLD' :   22,
    'FOLLOW' :     23,
    'ZIGZAG' :     24,
    'SYSTEMID' :   25,
    'AUTOROTATE' : 26,
    'AUTO_RTL' :   27}
    
    self.__DICT_ROVER_MODE = {
      'MANUAL' :  0,
      'ACRO' :    1,
      'STEERING' :3,
      'HOLD' :    4,
      'LOITER' :  5,
      'FOLLOW' :  6,
      'SIMPLE' :  7,
      'AUTO' :    10,
      'RTL' :     11,
      'SMART_RTL' : 12,
      'GUIDED' :    15,
      'INITIALISING' : 16}

    self._yaw = 0
    self._altitude = 0
    self.curent_mode = "STABILIZE"
    sh.success("Connection Initialized")


  def SetMode(self,mode,vehicle):
    """ 
    Set the mode of quadrotor from DICT_QUAD_MODE 
    :param mode: vehicle
    :param vehivle: 0 == quad; 1 == rover
    :returns 0:  Succes
    :returns -1: Failure
    """
    if vehicle == 0 and mode not in self.__DICT_QUAD_MODE.keys():
      sh.error("Unknown mode for quad")
      return -1
    elif vehicle == 1 and mode not in self.__DICT_ROVER_MODE.keys():
      sh.error("Unknown mode for rover")
      return -1
    else:
      if vehicle == 0:
        mode_id = self.__DICT_QUAD_MODE[mode]
      if vehicle == 1:
        mode_id = self.__DICT_ROVER_MODE[mode]
      self._the_connection.mav.command_long_send(
        self._the_connection.target_system, 
        self._the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,0,0,0,0,0)
      sh.info("Vehicle type: {}, Mode: {}".format(vehicle,mode))
      self.curent_mode = mode
      # while True:
      #   msg = self._the_connection.recv_match(type='COMMAND_ACK',blocking=True)
      #   msg = msg.to_dict()
      #   if msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
      #     continue
      #   print(mavutil.mavlink.enums['MAV_RESULT'][msg['result']].description)
      #   break
      return 0


  def ArmDisarm(self,key):
    """ 
    Arm or disarm the dron
    :param key: 1 for arm, 0 for disarm
    """
    if (key == 1):
      self._the_connection.mav.command_long_send(
        self._the_connection.target_system,
        self._the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,key,0,0,0,0,0,0)
      sh.info("Arming")
      self._the_connection.motors_armed_wait()
      sh.success("Armed")

      # msg = self._the_connection.recv_match(type='COMMAND_ACK',blocking=True)
      # sh.info(msg)
    elif (key == 0):
      self._the_connection.mav.command_long_send(
        self._the_connection.target_system,
        self._the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,key,0,0,0,0,0,0)
      sh.info("Disarming")
      self._the_connection.motors_disarmed_wait()
      sh.success("Disarmed")
      
      # msg = self._the_connection.recv_match(type='COMMAND_ACK',blocking=True)
      # sh.info(msg)
    else:
      sh.error("Key is not invalid. Must be 1 to arm, 0 to disarm")
      return -1
    # while True:
    #   msg = self._the_connection.recv_match(type='HEARTBEAT',blocking=False)
  
    #   #TODO Check heartbeat message to get mode status


  def Takeoff(self,h):
    """
    Arm the quadrotor and takae off
    :param h: altitude
    """
    self.ArmDisarm(1)
    self._the_connection.mav.command_long_send(
      self._the_connection.target_system,
      self._the_connection.target_component,
      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,h)
    sh.info("Takeoff {}m".format(h))
    # msg = self._the_connection.recv_match(type='COMMAND_ACK',blocking=True)
    # sh.info(msg)


  def Land(self):
    """ Land the quadrotor were it is """
    self.SetMode("LAND")
    sh.info("Landing")


  def SetSpeedLocalNed(self,vx=0,vy=0,vz=0,wpsi=0):
    '''
    Sets the linear and angular speed of the quadrotor in local referance frame
    :param vx: linear speed in x axis of the quadrotor [m/s]
    :param vy: linear speed in y axis of the quadrotor [m/s]
    :param vz: linear speed in z axis of the quadrotor [m/s]
    :param wpsi: angular speed in yaw angle of the quadrotor [rad/s] 
    '''
    #typmask : int(0b(yawrate)(yaw)0(az)(ay)(ax)(vz)(vy)(vx)(z)(y)(x)); 0 => control; 1=> not control
    self._the_connection.mav.send(
      mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
        self._the_connection.target_system,
        self._the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111000111), 0, 0, 0,
        vx, vy, vz, 0, 0, 0, 0, wpsi))


  def SetPositionGlobal(self,lat,lon,alt):
    self._the_connection.mav.send(
      mavutil.mavlink.MAVLink_set_position_target_global_int_message(10,
        self._the_connection.target_system,
        self._the_connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000),
        int(lat*10**7), int(lon*10**7), alt,
        0, 0, 0, 0, 0, 0, 0, 0))


  def GetYaw(self):
    """ Yaw angle from ATTITUDE """
    print("yaw in")
    msg = self._the_connection.recv_match(type='ATTITUDE',blocking=True)
    print("yaw out")
    self._yaw = msg.yaw
    return self._yaw


  def GetAltitude(self):
    """ ALTITUDE """
    msg = self._the_connection.recv_match(type='LOCAL_POSITION_NED',
                                          blocking=True)
    self._altitude = msg.z
    return self._altitude
  
  
  def GetGlobalPosition(self):
    """
    :returns: (lat,lon,alt,relative_alt)
    """
    msg = self._the_connection.recv_match(type='GLOBAL_POSITION_INT',
                                          blocking=True)
    pos = (msg.lat,msg.lon,msg.alt,msg.relative_alt)
    return pos


class RangeFinder():
  def __init__(self,serial_port,boud):
    self.ser = serial.Serial(serial_port,boud,timeout=1)
    self.ser.reset_input_buffer()
    self.data = 0

  def __DataCallback(self):
    self.data = self.ser.readline().decode("utf-8").rstrip()

  def GetData(self):
    self.__DataCallback()
    return self.data


class Camera():
  """ Aruco marker detection class """
  def __init__(self,dictionary,
               frameWidth=640, frameHeight=480, channels=3,
               is_sim=False):
    self.kFrameWidth = frameWidth
    self.kFrameHeight = frameHeight
    self.kChannels = channels
    self.is_sim = is_sim
    if not is_sim:
      self.cap=cv.VideoCapture(0)
      self.cap.set(3,self.kFrameWidth)
      self.cap.set(10,150)
      self.cap.set(4,self.kFrameHeight)
    else:
      rospy.init_node('DroneNode',anonymous=True)
      self.image_sub = rospy.Subscriber(
        "/webcam/image_raw",Image,self._SimCameraCallback)
      self.bridge_object = CvBridge()
      self.imgmsg = Image()

    self.DICT_ARUCO={
    "DICT_4X4_50":         cv.aruco.DICT_4X4_50,
    "DICT_4X4_100":        cv.aruco.DICT_4X4_100,
    "DICT_4X4_250":        cv.aruco.DICT_4X4_250,
    "DICT_4X4_1000":       cv.aruco.DICT_4X4_1000,
    "DICT_5X5_50":         cv.aruco.DICT_5X5_50,
    "DICT_5X5_100":        cv.aruco.DICT_5X5_100,
    "DICT_5X5_250":        cv.aruco.DICT_5X5_250,
    "DICT_5X5_1000":       cv.aruco.DICT_5X5_1000,
    "DICT_6X6_50":         cv.aruco.DICT_6X6_50,
    "DICT_6X6_100":        cv.aruco.DICT_6X6_100,
    "DICT_6X6_250":        cv.aruco.DICT_6X6_250,
    "DICT_6X6_1000":       cv.aruco.DICT_6X6_1000,
    "DICT_7X7_50":         cv.aruco.DICT_7X7_50,
    "DICT_7X7_100":        cv.aruco.DICT_7X7_100,
    "DICT_7X7_250":        cv.aruco.DICT_7X7_250,
    "DICT_7X7_1000":       cv.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL
    }

    self.arucoDict=cv.aruco.Dictionary_get(self.DICT_ARUCO[dictionary])
    self.arucoParams = cv.aruco.DetectorParameters_create()

    self._is_marker_detected = False
    self._is_steady_state = False
    self.kFrameCenter = (int(self.kFrameWidth/2),(int(self.kFrameHeight/2)))
    self.marker_center = (0,0)
    self.marker_area = 0
    self.marker_top_center = (0,0)
    self._frame = 0
    self._dist = 0
    sh.success("Camera Initialized")


  def _SimCameraCallback(self,img):
    self.imgmsg = img


  def SimGetImage(self):
    img = self.bridge_object.imgmsg_to_cv2(self.imgmsg, desired_encoding="bgr8")
    return img


  def _SteadyStateCallback(self):
    if self._dist < 0.04:
      self._is_steady_state = True
    else:self._is_steady_state = False


  def IsSteadyState(self):
    return self._is_steady_state


  def _MarkerDetection(self,marker_id):
    if not self.is_sim:
      success,self._frame = self.cap.read()
    else:
      self._frame = self.SimGetImage()
    (corners,ids,rejected)=cv.aruco.detectMarkers(self._frame,self.arucoDict,
                                                  parameters=self.arucoParams)
    if len(corners)>0:
      ids=ids.flatten()
      for (markerCorner, markerID) in zip(corners,ids):
        if markerID == marker_id:
          self._is_marker_detected = True
          corners = markerCorner.reshape((4, 2))
          (topLeft, topRight, bottomRight, bottomLeft) = corners

          # Convert each of the (x, y)-coordinate pairs to integers
          TopRight = (int(topRight[0]), int(topRight[1]))
          BottomRight = (int(bottomRight[0]), int(bottomRight[1]))
          BottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
          TopLeft = (int(topLeft[0]), int(topLeft[1]))

          # Compute and draw the center (x, y)-coordinates of the
          # Aruco marker
          cX = int((topLeft[0] + bottomRight[0]) / 2.0)
          cY = int((topLeft[1] + bottomRight[1]) / 2.0)
          self.marker_center = (cX,cY)
          
          oX = int((topLeft[0] + topRight[0]) / 2.0)
          oY = int((topLeft[1] + topRight[1]) / 2.0)
          self.marker_top_center = (oX,oY)

          self.marker_area = round(Distance(TopLeft,TopRight)*Distance(TopLeft,BottomLeft),2)
          self._dist = Pixel2MeterXY(Distance(self.kFrameCenter,self.marker_center),self.marker_area)
          self._SteadyStateCallback()
        else: self._is_marker_detected = False


  def IsMarkerDetected(self):
    return self._is_marker_detected


  def GetMarkerFrameInfo(self,marker_id):
    """ :returns: array of [markerCenter, frameCenter, markerPixelArea] """
    self._MarkerDetection(marker_id)
    mc = np.array(self.marker_center)
    fc = np.array(self.kFrameCenter)
    if not self._is_marker_detected:
      sh.warning("No marker in visual")
    return np.array([mc,fc,self.marker_area])



  def DrawVisual(self):
    """ Draw lines to show distance and orientation """
    if self._is_marker_detected:
      cv.line(self._frame, self.marker_center, self.marker_top_center, (0,255,0),2)
      
      if self._dist <0.25:
        cv.line(self._frame, self.marker_center, self.kFrameCenter, (0,255,0),2)
      elif 0.25< self._dist <0.5:
        cv.line(self._frame, self.marker_center, self.kFrameCenter, (0,255,255),2)
      else: 
        cv.line(self._frame, self.marker_center, self.kFrameCenter, (0,0,255),2)
    else:
      return -1


  def YawMarkerRelative(self):
    """ Angle between marker and drone """
    marker_center = np.array(self.marker_center)
    frame_center = np.array(self.kFrameCenter)
    marker_top_center = np.array(self.marker_top_center)
    
    vec_marker = marker_top_center-marker_center
    vec_frame = np.array([0,-frame_center[1]])
    
    psi = round(np.arcsin(np.cross(vec_frame,vec_marker)/(Distance(np.array([0,0]),vec_marker)*Distance(np.array([0,0]),vec_frame))),2)
    return psi


  def AltitudeMarkerRelative(self):
    return Pixel2MeterZ(self.marker_area)


  def ShowFrame(self):
    cv.imshow("Frame",self._frame)
    cv.waitKey(10) & 0xFF == ord("q")

class PID():
  """ PID controller class """
  def __init__(self,
               kp_x=1,kd_x=1,ki_x=1,
               kp_y=1,kd_y=1,ki_y=1,
               kp_z=1,kd_z=1,ki_z=1,
               kp_psi=1,kd_psi=1,ki_psi=1):
    self.kConstants = np.array([[kp_x,kd_x,ki_x],
                               [kp_y,kd_y,ki_y],
                               [kp_z,kd_z,ki_z],
                               [kp_psi,kd_psi,ki_psi]])
    self.kSampleTime = 0.1
    self.error = np.array([[0.0,0.0],
                          [0.0,0.0],
                          [0.0,0.0],
                          [0.0,0.0]]) #e[k],e[k-1]
    
    self.params = np.array([[0.0,0.0,0.0],
                           [0.0,0.0,0.0],
                           [0.0,0.0,0.0],
                           [0.0,0.0,0.0]])#[e[k],e[k]-e[k-1],e[k]+e[k-1]];x,y,z,psi
    
    self.output = np.array([0.0,0.0,0.0,0.0])


  def ErrorSet(self,
                    desired_x=0,measured_x=0,
                    desired_y=0,measured_y=0,
                    desired_z=0,measured_z=0,
                    desired_psi=0,measured_psi=0,
                    sampleTime=0.1,yaw=0,area=0):
    
    # Image frame to quadrotor frame
    ex_img = desired_x-measured_x
    ey_img = desired_y-measured_y
    ex_q = Pixel2MeterXY(-ey_img,area) 
    ey_q = Pixel2MeterXY(ex_img,area)

    # Quad frame to global frame
    exy_g = np.dot(rotation_matrix_z(yaw),np.array([ex_q,ey_q]).T)
    
    self.error[0][0] = exy_g[0]
    self.error[1][0] = exy_g[1]
    
    self.error[2][0] = desired_z-measured_z
    self.error[3][0] = desired_psi-measured_psi
    self.kSampleTime = sampleTime


  def _ErrorCallback(self):
    # Param set
    
    # e[k]
    self.params[0][0] = self.error[0][0]
    self.params[1][0] = self.error[1][0]
    self.params[2][0] = self.error[2][0]
    self.params[3][0] = self.error[3][0]
    
    # (e[k] - e[k-1])/T
    self.params[0][1] = (self.error[0][0] - self.error[0][1])/self.kSampleTime
    self.params[1][1] = (self.error[1][0] - self.error[1][1])/self.kSampleTime
    self.params[2][1] = (self.error[2][0] - self.error[2][1])/self.kSampleTime
    self.params[3][1] = (self.error[3][0] - self.error[3][1])/self.kSampleTime
    
    # (e[k] + e[k-1])
    self.params[0][2] = self.error[0][0] + self.error[0][1]
    self.params[1][2] = self.error[1][0] + self.error[1][1]
    self.params[2][2] = self.error[2][0] + self.error[2][1]
    self.params[3][2] = self.error[3][0] + self.error[3][1]
    
    # u = [e_proportional,e_derivative,e_integral]*[kp,kd,ki].T
    # Max speed was changed from 1.5 to 1.0 to avoid real time problems
    self.output[0] = Clamp(np.dot(self.params[0],self.kConstants[0].T),1.0)
    self.output[1] = Clamp(np.dot(self.params[1],self.kConstants[1].T),1.0)
    self.output[2] = Clamp(np.dot(self.params[2],self.kConstants[2].T),1.0)
    self.output[3] = Clamp(np.dot(self.params[3],self.kConstants[3].T),1.0)
    
    # e[k-1] = e[k]
    self.error[0][1] = self.error[0][0]
    self.error[1][1] = self.error[1][0]
    self.error[2][1] = self.error[2][0]
    self.error[3][1] = self.error[3][0]


  def Process(self):
    self._ErrorCallback()
    return self.output


#TODO Mission classes
class Mission(object):
  """ Handles the mission info """
  def __init__(self,lat,lon,alt,name,marker_id):
    self.position = (lat,lon,alt)
    self.name = name
    self.marker_id = marker_id


class EKF():
  """ Extendent Kalman Filter class """
  def __init__(self):
    pass


class Plotter():
  """ Array plotter class """
  def __init__(self):
    self.arr_100 = np.zeros(100)
    self.arr_100_full = False


  def PutVar100(self,var):
    """ Put 100 variables to an array """
    if not self.arr_100_full:
      self.arr_100[0] = var # [var,0,0]
      self.arr_100 = np.roll(self.arr_100,-1) # [0,0,var]

      if self.arr_100[0] != 0 and self.arr_100[99] != 0:
        self.arr_100_full = True
        
    else:sh.info("Plotter array is full")


  def Plot100(self):
    if not self.arr_100_full:
      sh.warning("Plotter array not full yet")
    else:  
      plt.plot(self.arr_100)
      plt.ylabel("command")
      plt.xlabel("iterations")
      plt.show()


class Vehicle():
  """ One class to rule them all !!! """
  def __init__(self,vehicle_type,nav_dev,
               cam_arucoDict,cam_is_sim,
               pid_kp_x,pid_kd_x,pid_ki_x,
               pid_kp_y,pid_kd_y,pid_ki_y,
               pid_kp_z,pid_kd_z,pid_ki_z,
               pid_kp_psi,pid_kd_psi,pid_ki_psi):

    if vehicle_type == "QUAD":
      self.vc_type = 0
      rng = RangeFinder("/dev/ttyUSB1",9600)
      self.rng_z = rng.GetData()
    elif vehicle_type == "ROVER":
      self.vc_type = 1
    else:
      sh.error("Wrong vehicle type")
    
    self.plot=Plotter()
    self.dt = 0.1
    self.nav = PyMavlink(nav_dev)
    self.cam = Camera(cam_arucoDict,640,480,3,cam_is_sim)
    self.pid = PID(pid_kp_x,pid_kd_x,pid_ki_x,
                   pid_kp_y,pid_kd_y,pid_ki_y,
                   pid_kp_z,pid_kd_z,pid_ki_z,
                   pid_kp_psi,pid_kd_psi,pid_ki_psi)
    self._global_position = (0,0,0,0)
    self.DICT_MISSIONS = {"DEFAULT_1" : Mission(0,0,0,"DEFAULT_1",42)}
    self.current_mission = "DEFAULT_1"
    self._land_on_marker = False
    self._item_drop = False
    self.home = Mission(0,0,0,"HOME",0)



  def Sleep(self,second):
    rospy.sleep(second)


  def GlobalPosition(self):
    self._global_position = self.nav.GetGlobalPosition()
    return self._global_position


  def SetHomePosition(self):
    for i in range(10):
      lat,lon,alt,rel = self.GlobalPosition()
      self.home.position = (lat,lon,alt)


  def MissionAdd(self,lat,lon,alt,name,marker_id):
    self.DICT_MISSIONS[name] = Mission(lat,lon,alt,name,marker_id)


  def SetMode(self,mode):
    """ 
    Set quadrotor mode
    :param mode: PyMavlink.DICT_QUAD_MODE
    """
    self.nav.SetMode(mode,self.vc_type)


  def LandOnMarker(self):
    if self._land_on_marker:
      self.SetMode("LAND")
      return 1
    else: return 0


  def RTL(self):
    self.SetMode("RTL")


  def Takeoff(self,h):
    """ Takeoff the quadrotor """
    self.nav.Takeoff(h)


  def ShowCam(self,mode=0):
    """ 
    Show camera frame
    :param mode: 1 for drawing distance, 0 otherwise
    """
    if mode:
      self.cam.DrawVisual()
    self.cam.ShowFrame()
    
  def DropItem(self):
    self._item_drop = True


  def Go2Aruco(self):
    print("im in")
    """ Navigate the quadrotor to the aruco marker in visual """
    marker_id = self.DICT_MISSIONS[self.current_mission].marker_id
    print(marker_id)
    marker_center, frame_center, area = self.cam.GetMarkerFrameInfo(marker_id)
    print(marker_center)
    if self.cam.IsMarkerDetected():
      print("lalalal")
      yaw = self.nav.GetYaw()
      print("yaw get")
      yaw_relative = self.cam.YawMarkerRelative()
      print("yaw relative")
      alt_desired = self.DICT_MISSIONS[self.current_mission].position[2]
      
      #PROBLEM : Simulation cannot detect marker under 0.7,0.6 meters altitude
      self.cam.AltitudeMarkerRelative()
      self.pid.ErrorSet(marker_center[0],frame_center[0], # X
                        marker_center[1],frame_center[1], # Y
                        -alt_desired,self.cam.AltitudeMarkerRelative(),# -Z
                        0,yaw_relative,     # PSI
                        self.dt,yaw,area)
      out = self.pid.Process()

      if self.cam.IsSteadyState():
        out_land = out[2]
      else:out_land = 0
      #self.plot.PutVar100(self.cam.dist)
      if (self.cam.AltitudeMarkerRelative() >= -0.75):
        self._land_on_marker = True
      else: self._land_on_marker = False

      self.nav.SetSpeedLocalNed(out[0],out[1],out_land,out[3])
    else:sh.warning("No aruco in visual")


  def Go2MissionPoint(self,mission_name):
    if mission_name in self.DICT_MISSIONS.keys():
      lat,lon,alt = self.DICT_MISSIONS[mission_name].position
      self.current_mission = mission_name
      marker_id = self.DICT_MISSIONS[self.current_mission].marker_id 

      self.nav.SetPositionGlobal(lat,lon,alt)
      if self.vc_type == 0:
        while True:
          self.Sleep(0.1)
          self.cam.GetMarkerFrameInfo(marker_id)
          sh.info("Going to mission {}, no marker[{}] seen yet".format(
            mission_name,marker_id))
          if self.cam.IsMarkerDetected():
            break
        print("koddan cikiyoruz")
        while True:
          start_time = time.time()
          print("entering go2aruco")
          self.Go2Aruco()
          #self.ShowCam(1)
          print("go to arukodan cikti")
          self.Terminal()
          self.dt = round(time.time()-start_time,2)
          if self.cam.IsSteadyState():
            if mission_name[:5] == "MEDIC":
              self.DropItem()
            cv.destroyAllWindows()
            break
      return 0
    else:
      sh.error("No mission with the name {}".format(mission_name))
      return -1

  def odroid_test_alt(self):
    return self.nav.GetAltitude()

  def Terminal(self):
    print("""
          ===============TERMINAL===============
          mode: {}
          home position: {}
          vehicle position {}
          current mission: {}
          mission position {}
          is marker detected: {}
          is item dropped: {}
          dt: {}
          ======================================
          """.format(self.nav.curent_mode,
                     self.home.position,
                     self.GlobalPosition(),
                     self.current_mission,
                     self.DICT_MISSIONS[self.current_mission].position,
                     self.cam.IsMarkerDetected(),
                     self._item_drop,
                     1/self.dt))