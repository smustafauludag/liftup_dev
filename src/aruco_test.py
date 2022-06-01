#!/usr/bin/env python

import vehicle as vc
import bash_info_helper as sh

def main():
  uav = vc.Vehicle("QUAD","USB","DICT_5X5_100",False,
                0.33,0.05,0,
                0.33,0.05,0,
                0.1,0,0,
                1,0,0)

  uav.MissionAdd(lat=-35.36322005,
                lon=149.16515675,
                alt=2,
                name="LAND_1",
                marker_id=0)
  
  try:
    uav.SetMode("GUIDED")
    uav.SetHomePosition()
    uav.Takeoff(2)
    uav.Sleep(1)
    uav.Go2MissionPoint("LAND_1")
    uav.LandOnMarker()

  except KeyboardInterrupt:
    sh.warning("Keyboard Interrupt, Shutting down")
    
if __name__ == "__main__":
  main()