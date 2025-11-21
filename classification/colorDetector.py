import threading, time, queue, copy
import numpy as np
import cv2 as cv
from camera.frameDistributor import frameDistributor
from dto.Duba import Duba
from dto.DataTransferObject import DataTransferObject
class colorDetector:
    def __init__(self,frame_distributor:frameDistributor,queue_size,CONTOUR_SMALL_OBJECTS_FILTER:int,interval=0.05, max_aspect_ratio= 6, min_aspect_ratio = 4):
        self.CONTOUR_SMALL_OBJECTS_FILTER = CONTOUR_SMALL_OBJECTS_FILTER
        self.frame_counter = 0
        self.frame_interval = 2 #process every 2 frame
        self.interval = interval
        self.lock = threading.Lock()
        self.queue_size = queue_size
        self.thread = threading.Thread(target=self._loop)
        self.consumer_queues = []
        self.max_aspect_ratio = max_aspect_ratio
        self.min_aspect_ratio = min_aspect_ratio
        self.latest_color_detected_frame_obj = None
        self.frame_queue = frame_distributor.register_customer_queue()
        
        self.running = True
        self.thread.daemon = False
        self.thread.start()

    def _loop(self):
        while self.running:
            if self.frame_counter % self.frame_interval !=0:
                continue
            fto = None
            try:
                fto = self.frame_queue.get(timeout=1)
            except queue.Empty:
                print("[COLOR]: queue is empty...")
                pass
            if fto is None:
                continue
            if fto.frame is None:
                continue
            print("[COLOR]: Frame geldi")
            
            fto = self.detect_objects(fto)
            print("[COLOR]: len of the frame_object: " + str(len(fto.color_detected_objects)))
            with self.lock:
                self.latestFrameObject = fto
            
            try:
                for q in self.consumer_queues:
                    q.put(fto)
            except queue.Full:
                    print('[COLOR]: queue is full, dropping frame...')
                    q.get(timeout=0.1)
                    q.put(fto)
    def detect_objects(self, fto: DataTransferObject):
        img = fto.frame
        color_ranges = {

            "red": [
                    (np.array([0, 150, 150]), np.array([10, 255, 255])),
                    (np.array([160, 150, 150]), np.array([179, 255, 255]))
                ],
            "green": [(np.array([59, 175, 59]), np.array([79, 255, 219]))],
            "black": [(np.array([0, 0, 0]), np.array([179, 255, 50]))]  # RAL 9005
        }

        #img = cv.GaussianBlur(img, (5, 5), 0)
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        for color, ranges in color_ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:  # Iterate over the list of (lower, upper) tuples
                mask = cv.bitwise_or(mask, cv.inRange(hsv, lower, upper))
            
            #kernel = np.ones((5, 5), np.uint8)
            #mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
            
            # Find contours
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv.contourArea(contour) > self.CONTOUR_SMALL_OBJECTS_FILTER:  # Filter small objects
                    x, y, w, h = cv.boundingRect(contour)
                    #if self.min_aspect_ratio < w / h < self.max_aspect_ratio:
                    try:
                        fto.color_detected_objects.append(Duba(color, x, y, w, h))
                        fto.color_processed = True
                        #print("[COLOR: ]:" ,frame_object.color_detected_objects)
                    except (NameError, TypeError) as e:
                        raise RuntimeError(f"[COLOR]: failed to create Duba object: {str(e)}")
        return fto
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
        print('[COLOR]: STOP METHOD HAS BEEN CALLED')
        self.running = False

        # frame_queue'yu temizle
        while not self.frame_queue.empty():
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