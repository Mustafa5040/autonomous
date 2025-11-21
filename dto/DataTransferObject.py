import time,copy
class DataTransferObject:
    def __init__(self, frame_id=None, frame=None, detected_objects=[], classified=False, color_processed=False, image_processed=False):
        self.frame_id = frame_id
        self.frame = frame
        self.timestamp = time.time()
        self.detected_objects = detected_objects
        self.color_detected_objects = []
        self.image_processed = image_processed
        self.color_processed = color_processed
        self.classified = classified
    def getFrame(self):
        return self.frame.copy()
    def copy(self): #deepcopy
        new_obj = DataTransferObject(
            frame_id=self.frame_id,
            frame=self.frame,
            classified=self.classified,
            color_processed=self.color_processed,
            image_processed=self.image_processed
        )
        new_obj.color_detected_objects = copy.deepcopy(self.color_detected_objects)
        new_obj.detected_objects = copy.deepcopy(self.detected_objects)
        return new_obj
    def deepcopy(self):
        new_obj = DataTransferObject(
            frame_id=self.frame_id,
            frame=copy.deepcopy(self.frame),
            classified=self.classified,
            color_processed=self.color_processed,
            image_processed=self.image_processed
        )
        new_obj.color_detected_objects = copy.deepcopy(self.color_detected_objects)
        new_obj.detected_objects = copy.deepcopy(self.detected_objects)
        return new_obj
