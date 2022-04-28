#!/usr/bin/env python

import vehicle as vc
import bash_info_helper as sh

uav_1 = vc.Vehicle("QUAD","GAZEBO_1","DICT_5X5_100",True,
                0.33,0.05,0,
                0.33,0.05,0,
                0.1,0,0,
                1,0,0)

ugv_1 = vc.Vehicle("ROVER","GAZEBO_2","DICT_5X5_100",True,
                0.33,0.05,0,
                0.33,0.05,0,
                0.1,0,0,
                1,0,0)


try:
  print(uav_1.vc_type)
  print(ugv_1.vc_type)
  
  uav_1.SetMode("GUIDED")
  uav_1.Sleep(3)
  ugv_1.SetMode("GUIDED")
  ugv_1.Sleep(2)
except KeyboardInterrupt:
  sh.warning("Keyboard Interrupt, Shutting down")