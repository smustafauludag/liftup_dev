#!/usr/bin/env python

import vehicle as vc
import cv2 as cv
import time
import bash_info_helper as sh
import rospy

def main():
  try:
    uav = vc.Vehicle("GAZEBO","DICT_5X5_100",True,
                    0.33,0.05,0,
                    0.33,0.05,0,
                    0.1,0,0,
                    1,0,0)
    
    uav.MissionAdd(lat=-35.36322005,
                    lon=149.16515675,
                    alt=2,
                    name="MEDIC_1",
                    marker_id=42)
    
    uav.SetMode("GUIDED")
    uav.SetHomePosition()
    uav.Takeoff(2)
    uav.Sleep(3)
    
    uav.Go2MissionPoint("MEDIC_1")
    while not rospy.is_shutdown():
      start_time = time.time()

      uav.Go2Aruco()
      uav.ShowCam(1)
      uav.Terminal()
      if uav.LandOnMarker():
        break

      uav.dt = round(time.time()-start_time,2)
      if cv.waitKey(10) & 0xFF == ord("q"):
        cv.destroyAllWindows()
        break
    #uav.plot.Plot100()
  except KeyboardInterrupt:
    sh.warning("Keyboard Interrupt, Shutting down")
    
if __name__ == "__main__":
  main()