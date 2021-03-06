#!/usr/bin/env python

import vehicle as vc
import bash_info_helper as sh

def main():
  uav = vc.Vehicle("QUAD","USB","DICT_5X5_100",False,
                0.05,0.05,0,
                0.05,0.05,0,
                0.1,0,0,
                1,0,0)

  uav.MissionAdd(lat=-35.36322005,
                lon=149.16515675,
                alt=2,
                name="MEDIC_1",
                marker_id=0)
  

######################################################
  #Takeoff test
  try:
    uav.SetMode("GUIDED")
    uav.Takeoff(2)
    while 1:
      uav.Terminal()  
  except KeyboardInterrupt:
    sh.warning("Keyboard Interrupt, Shutting down")
  
######################################################
  #Marker test
  # try:
  #   uav.SetMode("GUIDED")
  #   uav.Takeoff(2)
  #   while 1:
  #     print(uav.cam.GetMarkerFrameInfo())
  # except KeyboardInterrupt:
  #   sh.warning("Keyboard Interrupt, Shutting down")

######################################################
  #Speed test
  # try:
  #   uav.SetMode("GUIDED")
  #   uav.Takeoff(2)
  #   uav.Sleep(4)
  #   uav.nav.SetSpeedLocalNed(0.5,0,0,0)
  # except KeyboardInterrupt:
  #   sh.warning("Keyboard Interrupt, Shutting down")
  
######################################################
  # #Navigation test
  # try:
  #   uav.SetMode("GUIDED")
  #   uav.Takeoff(2)
  #   uav.Sleep(2)
  #   uav.Go2MissionPoint("MEDIC_1")
  #   while 1:
  #     uav.Terminal()  
  # except KeyboardInterrupt:
  #   uav.nav.Land()
  #   sh.warning("Keyboard Interrupt, Shutting down")
    
######################################################


if __name__ == "__main__":
  main()