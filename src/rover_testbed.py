#!/usr/bin/env python

import vehicle as vc
import bash_info_helper as sh

def main():
  uav = vc.Vehicle("ROVER","ACM0","DICT_5X5_100",False,
                0.33,0.05,0,
                0.33,0.05,0,
                0.1,0,0,
                1,0,0)

  # uav.MissionAdd(lat=-35.36322005,
  #               lon=149.16515675,
  #               alt=2,
  #               name="MEDIC_1",
  #               marker_id=0)
  
  try:
    uav.SetMode("GUIDED")
    uav.nav.ArmDisarm(1)
    #uav.Go2MissionPoint("MEDIC_1")
    #uav.Takeoff(2)
    uav.nav.SetSpeedLocalNed(1,0,0,0)


  except KeyboardInterrupt:
    sh.warning("Keyboard Interrupt, Shutting down")
    
if __name__ == "__main__":
  main()