import cv2 as cv
import threading, queue, copy, time
from dto.DataTransferObject import DataTransferObject
class frameDistributor:
    def __init__(self,width,height,deviceId=0,queue_size = 100,interval=0.05):
        try:
            deviceId = int(deviceId)
        except (ValueError,TypeError):
            print('[DIST] capture device id: Value/Type error...')
            pass
        self.frame_interval = 2
        self.frame_count = 0
        self.capture = cv.VideoCapture(deviceId)
        self.fps = self.capture.get(cv.CAP_PROP_FPS)
        self.setupCap(width,height)
        
        self.interval = interval
        self.last_processed_time = 0
        self.queue_size = queue_size
        self.latest_frame_object = DataTransferObject()
        self.consumer_queues = []
    
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._loop)
        self.running = True
        self.thread.daemon = False
        self.thread.start()
    
    def _loop(self):
        print("[DIST] LOOP")
        while self.running:
            if self.capture.isOpened() is False:
                print("[DIST]: CAPTURE IS CLOSED")
                break
            status, frame = self.capture.read()
            if not status:
                print("[DIST]: NOT STATUS")
                time.sleep(0.05)
                continue
            with self.lock:
                self.latest_frame_object.frame = frame
                self.latest_frame_object.frame_id = self.frame_count
            try:
                for q in self.consumer_queues:
                    q.put_nowait(self.latest_frame_object.deepcopy())
                self.frame_count += 1
            except queue.Full:
                print('[DIST]: queue is full, dropping frame...')
                pass
    def getLatestFrame(self):
        with self.lock:
            if self.latest_frame_object is not None:
                if self.latest_frame_object.frame is not None:
                    frame_copy = copy.deepcopy(self.latest_frame_object.frame)
                    self.latest_frame_object.frame = frame_copy
                    return self.latest_frame_object.copy()
            return None
    def stop(self):
        print('[DIST] STOP METHOD HAS BEEN CALLED')
        self.running = False
        for q in self.consumer_queues:
            while not q.empty():
                try:
                    q.get_nowait()
                except queue.Empty:
                    break
            try:
                q.put_nowait(None)
            except queue.Full:
                print("[DIST]: Consumer queue is full, cannot put None")

        self.thread.join(timeout=5.0)
        if self.thread.is_alive():
            print("[DIST]: Thread did not terminate within timeout!")

        if self.capture.isOpened():
            self.capture.release()
        else:
            print("[DIST]: VideoCapture was already closed!")

        # consumer_queues listesini temizle
        self.consumer_queues.clear()
    def register_customer_queue(self):
        q = queue.Queue(maxsize=self.queue_size)
        self.consumer_queues.append(q)
        return q
    def setupCap(self,width,height):
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, height)
    def putFPSText(self,frame):
        cv.putText(frame,"FPS: " + str(self.fps),(0,20),cv.FONT_HERSHEY_COMPLEX,0.4,(255,0,255),1)
    
