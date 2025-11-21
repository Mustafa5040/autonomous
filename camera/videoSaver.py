import cv2 as cv
from datetime import datetime
import threading,time
import queue
from camera.frameDistributor import frameDistributor
from dto.DataTransferObject import DataTransferObject
class videoSaver:
    def __init__(self,fps,width,height,frame_distributor:frameDistributor):
        self.codec = cv.VideoWriter_fourcc(*"mp4v")
        self.out = cv.VideoWriter(self.generateVideoName(),self.codec, fps, (width,height))
        self.frame_distributor = frame_distributor
        self.writer_queue = frame_distributor.register_customer_queue()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._loop)
        self.running = True
        self.thread.daemon = False
        self.thread.start()

    def generateVideoName(self):
        return "logs/[unprocessed]_" + datetime.now().strftime('%d-%m-%Y_%H-%M-%S') + ".mp4"
    def _loop(self):
        while self.running:
            if not self.out.isOpened():
                break
            try:
                frame_object = self.writer_queue.get(timeout=0.1)
                if frame_object is None:
                    print("[video_writer]: Received None. Exiting thread.")
                    break
            except queue.Empty:
                print('[video_writer]: Queue is empty... Continue...')
                continue
            with self.lock:
                self.out.write(frame_object.frame)
    def stop(self):
        print('[video_writer] STOP METHOD HAS BEEN CALLED')
        self.running = False
        # Kuyruktaki kalan frame'leri yaz
        while not self.writer_queue.empty():
            try:
                frame_object = self.writer_queue.get_nowait()
                if frame_object is not None:
                    with self.lock:
                        if self.out.isOpened():
                            self.out.write(frame_object.frame)
                        else:
                            print("[video_writer] is not opened during queue flush!")
            except queue.Empty:
                break

        if self.out.isOpened():
            self.out.release()
        else:
            print("[video_writer] was already closed!")

        self.thread.join(timeout=5.0)
        if self.thread.is_alive():
            print("[video_writer]: Thread did not terminate within timeout!")