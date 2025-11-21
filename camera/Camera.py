import cv2 as cv
import math
from dto.Duba import Duba
class Camera:
    def __init__(self, camera_sensor: str, focal_length: float, sensor_width: float, sensor_height: float,horizontal_fov:float,vertical_fov:float,resolution_mode:tuple):
        self.camera_sensor = camera_sensor            # ex: 'imx219'
        self.focal_length = focal_length             # meters
        self.sensor_width = sensor_width          # meter
        self.sensor_height = sensor_height        # meters
        self.horizontal_fov = horizontal_fov          # degree
        self.vertical_fov = vertical_fov              # degree
        self.resolution_mode = resolution_mode        # (width, height, fps)
        self.CAMERA_IMU_YAW_OFFSET = math.radians(5.0) #mesela kalibrasyon

    def calculateDistanceToObject(self,
        object: cv.typing.Rect,              # (x, y, w, h) yani (x,y,genislik,yukseklik) formatında
        image: cv.UMat,                      # Görüntü karesi
        realObjectSize: float,               # m cinsinden
        isWidth: bool                        # True: genişlik üzerinden, False: yükseklik
    ) -> float:
        # Piksel başına m değeri
        pixel_per_unit= (
            self.sensor_width / image.shape[1] if isWidth
            else self.sensor_height / image.shape[0]
        )
        # Görüntüdeki nesnenin fiziksel sensör boyutu (m)
        object_sensor_size = object[2] * pixel_per_unit if isWidth else object[3] * pixel_per_unit

        # Uzaklık hesaplama formülü
        return self.focal_length * (realObjectSize[1] / object_sensor_size) if isWidth else self.focal_length * (realObjectSize[0] / object_sensor_size)
    def calculateCamAngleToObject(self,
        duba:Duba,
        image: cv.UMat):
        """
        Nesnenin (Duba) görüntüdeki konumuna göre kameradan olan yatay açisini (radyan) döndürür.
        Pozitif açilar, saat yönünün tersini ifade eder.
        """
        img_center = (image.shape[1] / 2, image.shape[0] / 2)
        angle_per_horizontal_px = math.radians(self.horizontal_fov) / image.shape[1]
        horizontal_distance_to_obj_px = img_center[0] - duba.getCenterCoordinates()[0]
        return horizontal_distance_to_obj_px * angle_per_horizontal_px
    def calculateObjectGPSLoc(self,
        duba:Duba,
        cam_gps:tuple):
        real_angle = duba.angle + self.CAMERA_IMU_YAW_OFFSET # + IMU
        horizontal_lat_angle =  duba.distance * math.sin(real_angle) / 111320.0
        vertical_long_angle = duba.distance * math.cos(real_angle) / (111320.0 * math.cos(math.radians(cam_gps[1])))
        return ( cam_gps[0] + horizontal_lat_angle, cam_gps[1] + vertical_long_angle )
        

        


