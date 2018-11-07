################################################################################

# threaded frame capture from camera to avoid camera frame buffering delays
# (always delivers the latest frame from the camera)

# The MIT License (MIT)
# Copyright (c) 2018 Toby Breckon, Durham University, UK
# Copyright (c) 2015-2016 Adrian Rosebrock, http://www.pyimagesearch.com

# based on code from this tutorial, with changes to make object method compatible
# with cv2.VideoCapture(src):
# https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/

################################################################################

# import the necessary packages

from threading import Thread
import cv2

################################################################################

class CameraVideoStream:
	def __init__(self, src=0, name="CameraVideoStream"):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.camera = cv2.VideoCapture(src)
		(self.grabbed, self.frame) = self.camera.read()

		# initialize the thread name
		self.name = name

		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False

	def open(self):
		# start the thread to read frames from the video stream
		t = Thread(target=self.update, name=self.name, args=())
		t.daemon = True
		t.start()
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				camera.release();
				return

			# otherwise, read the next frame from the stream

			(self.grabbed, self.frame) = self.camera.read()

	def read(self):
		# return the frame most recently read
		return (self.grabbed, self.frame)

	def release(self):
		# indicate that the thread should be stopped
		self.stopped = True

	def isOpened(self):
		# indicate that the camera is open successfully
		return (self.grabbed > 0);

################################################################################
