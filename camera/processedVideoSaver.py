import cv2 as cv
from datetime import datetime
import threading,time
import queue, copy
from classification.objectClassifier import objectClassifier
from dto.DataTransferObject import DataTransferObject
class processedVideoSaver:
    def __init__(self,fps,width,height,object_classifier:objectClassifier,queue_size=1000):
        self.codec = cv.VideoWriter_fourcc(*"mp4v")
        self.queue_size = queue_size
        self.out = cv.VideoWriter(self.generateVideoName(),self.codec, fps, (width,height))
        self.object_classifier = object_classifier
        self.object_queue = self.object_classifier.register_customer_queue()
        self.consumer_queues = []
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._loop)
        self.running = True
        self.thread.daemon = False
        self.thread.start()
    def _loop(self):
        print("[PROCESSED]: LOOP")
        while self.running:
            print("[PROCESSED]: RUNNING")
            if not self.out.isOpened():
                break
            fto = None
            try:
                fto = self.object_queue.get(timeout=0.1)
            except queue.Empty:
                print("[PROCESSED]: queue is empty, passing...")
                pass
            if fto is None:
                print("[PROCESSED]: fto is None")
                continue
            if fto.getFrame() is None:
                print("[PROCESSED]: fto.frame is None")
                continue
            if len(fto.detected_objects) == 0:
                print("[PROCESSED]: current queue size: " + str(self.object_queue.qsize()))
                print("[PROCESSED]: fto.detected_objects length is 0")
                continue
            print("[PROCESSED]: Frame geldi" + str(fto.frame))
            with self.lock:
                fto = self.drawBoundingBoxes(fto)
                if fto is None:
                    continue
                #print("[PROCESSED]: Frame KONTROL" + str(fto.frame))
                self.out.write(fto.getFrame())
                print("[PROCESSED]: Frame yazildi")
                try:
                    for q in self.consumer_queues:
                        q.put_nowait(fto.copy())
                except queue.Full:
                        print('[PROCESSED]: queue is full, dropping frame...')
                        q.get(timeout=0.1)
                        q.put_nowait(fto.copy())
    def drawBoundingBoxes(self,fto):
        frame = fto.getFrame()
        if frame is None:
            return None
        for duba in fto.detected_objects:
            cv.rectangle(frame,(duba.x,duba.y),(duba.x+duba.w,duba.y+duba.h),(255,0,255),2,0)
            cv.putText(frame, str(duba.distance) + "m", (duba.x, duba.y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 1)
        fto.frame = frame
        return fto.copy()
    def register_customer_queue(self):
        with self.lock:
            q = queue.Queue(maxsize=self.queue_size)
            self.consumer_queues.append(q)
        return q
    def generateVideoName(self):
        return "logs/[processed]_" + datetime.now().strftime('%d-%m-%Y_%H-%M-%S') + ".mp4"
    
    def stop(self):
        print('[PROCESSED]: STOP METHOD HAS BEEN CALLED')
        self.running = False  # Thread döngüsünü durdur
        
        # frame_queue'yu temizle
        while not self.object_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        
        # consumer_queues'a None ekle ve kuyrukları temizle
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
                    print("[PROCESSED]: Consumer queue is full, cannot put None")
        
        # Thread'in kapanmasını bekle (zaman aşımı ile)
        self.thread.join(timeout=5.0)  # 5 saniye zaman aşımı
        if self.thread.is_alive():
            print("[PROCESSED]: Thread did not terminate within timeout!")
        
        # consumer_queues listesini temizle
        with self.lock:
            self.consumer_queues.clear()