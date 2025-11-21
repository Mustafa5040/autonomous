import threading, time, queue, copy
import numpy as np
import cv2 as cv
from dto.Duba import Duba
from classification.colorDetector import colorDetector
from dto.DataTransferObject import DataTransferObject
from camera.Camera import Camera
class objectClassifier:
    def __init__(self,cam:Camera, colorDetector:colorDetector, queue_size,CONTOUR_SMALL_OBJECTS_FILTER:int,interval=0.1, max_aspect_ratio= 6, min_aspect_ratio = 4):
        self.frame_counter = 0
        self.frame_interval = 2 #process every 2 frame
        self.lock = threading.Lock()
        self.queue_size = queue_size
        self.last_processed_time = 0
        self.real_obj_size = (10,10) #m
        self.thread = threading.Thread(target=self._loop)
        self.interval = interval
        self.consumer_queues = []
        self.color_detector = colorDetector
        self.color_queue = colorDetector.register_customer_queue()
        self.max_aspect_ratio = max_aspect_ratio
        self.min_aspect_ratio = min_aspect_ratio
        self.latest_classified_frame_obj = None
        self.cam = cam
        self.CONTOUR_SMALL_OBJECTS_FILTER = CONTOUR_SMALL_OBJECTS_FILTER
        self.running = True
        self.thread.daemon = False
        self.thread.start()

    def _loop(self):
        while self.running:
            if self.frame_counter % self.frame_interval != 0:
                continue
            processed_fto = None
            try:
                processed_fto = self.color_queue.get(timeout=1)
            except queue.Empty:
                print("[CLASSIFIER]: queue is empty...")
                pass
            if processed_fto is None:
                print("[CLASSIFIER]: processed FTO is None...")
                continue
            if processed_fto.frame is None:
                print("[CLASSIFIER]: FTO frame is None...")
                continue
            print("[CLASSIFIER]: Frame geldi")
        
            processed_fto = self.classify(processed_fto)
            print(str(processed_fto.getFrame()))
            print("[CLASSIFIER]: len of the frame_object: " + str(len(processed_fto.color_detected_objects)))
            with self.lock:
                self.latest_classified_frame_obj = processed_fto
            try:
                for q in self.consumer_queues:
                    q.put_nowait(processed_fto)
            except queue.Full:
                    print('[CLASSIFIER]: queue is full, dropping frame...')
                    q.get(timeout=0.1)
                    q.put_nowait(processed_fto)
    def classify(self, processed_fto: DataTransferObject):
        if processed_fto is None:
            return None
        if processed_fto.getFrame() is None:
            return None
        if processed_fto.color_processed is None:
            return None
        for duba in processed_fto.color_detected_objects:
            duba.distance = self.cam.calculateDistanceToObject( (duba.x,duba.y,duba.w,duba.h),processed_fto.getFrame(),self.real_obj_size, False )
            duba.angle = self.cam.calculateCamAngleToObject(duba,processed_fto.getFrame())
            duba.gps_loc = self.cam.calculateObjectGPSLoc(duba,(0,0))
            processed_fto.detected_objects.append(duba)
        return processed_fto

    def register_customer_queue(self):
        with self.lock:
            q = queue.Queue(maxsize=self.queue_size)
            self.consumer_queues.append(q)
        return q
    def getLatestFrameObject(self):
        with self.lock:
            if self.latestFrameObject is not None:
                return self.latestFrameObject
    def stop(self):
        print('[CLASSIFIER]: STOP METHOD HAS BEEN CALLED')
        self.running = False

        # frame_queue'yu temizle
        while not self.color_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break

        # consumer_queues'u temizle ve None ekle
        with self.lock:
            for q in self.consumer_queues:
                # Kuyruğu temizle
                while not q.empty():
                    try:
                        q.get_nowait()
                    except queue.Empty:
                        break
                # None ekleyerek bağımlı thread'lere sinyal gönder
                try:
                    q.put_nowait(None)
                except queue.Full:
                    print("[COLOR]: Consumer queue is full, cannot put None")