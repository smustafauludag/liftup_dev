#!/usr/bin/env python
"""
Creates aruco marker images in a file
@author Sefer Mustafa Uludag
@mail smustafauludag@gmail.com

sh messages can be changed with print()
check filepath
"""

import cv2 as cv
import numpy as np
import argparse
import sys
import os
import bash_info_helper as sh

ap = argparse.ArgumentParser()

# ap.add_argument("-o","--output",required=True,
#                 help="path to images")
ap.add_argument("-d","--dict",type=str,
                default="DICT_ARUCO_ORIGINAL",
                help="dictionary wanted to generate")
args = vars(ap.parse_args())

DICT_ARUCO={
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
  "DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL}

if not args["dict"] in DICT_ARUCO.keys():
  sh.error("Aruco dict of {} is not supported".format(args["dict"]))
  sys.exit(0)

file_path = "/home/mustafauludag/aruco_tags"
if not os.path.exists(file_path):
  os.makedirs(file_path)
os.chdir(file_path)

dict_path = file_path+("/{}").format(args["dict"])
if not os.path.exists(dict_path):
  os.makedirs(dict_path)
os.chdir(dict_path)


ids = ""
for i in range(9,len(args["dict"])):
  ids = ids+("{}".format(args["dict"][i]))

aruco_dict = cv.aruco.Dictionary_get(DICT_ARUCO[args["dict"]])
tag = np.zeros((300,300,1),dtype = "uint8")

for id in range (0,int(ids)):
  img_name = args["dict"]+"_"+"id{}".format(id)+".png"
  cv.aruco.drawMarker(aruco_dict,id,300,tag,1)
  cv.imwrite(img_name,tag)

sh.success("tags added to path "+dict_path)