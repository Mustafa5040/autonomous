import datetime, time
import cv2 as cv
from camera.frameDistributor import frameDistributor
from camera.Camera import Camera
from camera.videoSaver import videoSaver
from camera.processedVideoSaver import processedVideoSaver
from classification.objectClassifier import objectClassifier
from classification.colorDetector import colorDetector
import time
print(datetime.datetime.today().strftime('%d-%m-%Y_%H-%M-%S'))
cam = Camera('imx219',3.04,3.68,2.86,62.2,48.8,(1280,720,30))
frame_distributor = frameDistributor(1280,720,0,10)
video_save_worker = videoSaver(30,1280,720,frame_distributor)
color_detector = colorDetector(frame_distributor,10,1000)
object_classifier = objectClassifier(cam,color_detector,100,10)
processed_video_saver = processedVideoSaver(30,1280,720,object_classifier,10)

frame_queue = frame_distributor.register_customer_queue()
processed_frame_queue = processed_video_saver.register_customer_queue()

while True:
    fto = processed_frame_queue.get()
    if fto.getFrame() is not None:
        cv.imshow('mdd',fto.getFrame())
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

frame_distributor.stop()
video_save_worker.stop()
color_detector.stop()
object_classifier.stop()
processed_video_saver.stop()