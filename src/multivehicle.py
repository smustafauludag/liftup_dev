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

uav_1.SetHomePosition()
ugv_1.SetHomePosition()

uav_1.MissionAdd(lat=-35.36322005,
                lon=149.16515675,
                alt=2,
                name="MEDIC_1",
                marker_id=42)

ugv_1.MissionAdd(lat=-35.36321188,
                lon=149.16517617,
                alt=1.5,
                name="POSE_1",
                marker_id=0)

uav_1.SetMode("GUIDED")
uav_1.Sleep(1)
ugv_1.SetMode("GUIDED")
ugv_1.Sleep(1)

try:
  uav_1.Takeoff(2)
  uav_1.Sleep(2)
  ugv_1.nav.ArmDisarm(1)

  uav_1.Go2MissionPoint("MEDIC_1")
  ugv_1.Go2MissionPoint("POSE_1")

  if uav_1._item_drop:
    for i in range(10):
      lat,lon,alt,rel = ugv_1.GlobalPosition()
    uav_1.MissionAdd(lat=lat/10**7,
                  lon=lon/10**7,
                  alt=1.5,
                  name="LAND_1",
                  marker_id=0)
  uav_1.Go2MissionPoint("LAND_1")
except KeyboardInterrupt:
  sh.warning("Keyboard Interrupt, Shutting down")