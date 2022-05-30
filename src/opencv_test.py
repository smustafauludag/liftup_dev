#!/usr/bin/env python
import cv2 as cv

cap = cv.VideoCapture(0)

while 1:
	success,frame = cap.read()
	cv.imshow('frame',frame)

	if cv.waitKey(10) & 0xFF == ord('q'):
		break