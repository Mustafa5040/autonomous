class Duba():
    """
    Represents a detected barrier in the image.

    Attributes:
    color (str): The color of the barrier (e.g., 'red', 'green', 'yellow').
    x (int): The x-coordinate of the top-left corner of the barrier.
    y (int): The y-coordinate of the top-left corner of the barrier.
    w (int): The width of the barrier.
    h (int): The height of the barrier.
    area (int): The area of the barrier (w * h).
    positions (list): A list to store the extreme positions of the barrier ('l', 'r', 'u', 'd').
    """
    def __init__(self,positions=None,center_gps=None,angle=None,distance=None,color=None,x=None,y=None,w=None,h=None):
        self.color = color
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.area = w*h
        self.polygon = None
        self.distance = distance #distanceToCamera
        self.angle = angle #(-90, +90)
        self.center_gps = center_gps #lon,lat, GPS object
        self.positions = positions if positions is not None else [] # # Left Right Up Down --> 'l' 'r' 'u' 'd'
    def getCenterCoordinates(self):
        return ( ( self.x + (self.w/2 ) ), ( self.y + (self.h/2) ) )
    def __repr__(self):
         return (
            f"\n-----------DUBA OBJESI BASLANGIC---------\n"
            f"Color: {self.color}\n"
            f"x: {self.x}\n"
            f"y: {self.y}\n"
            f"w: {self.w}\n"
            f"h: {self.h}\n"
            f"area: {self.area}\n"
            f"Position (lrud): {self.positions}\n"
            f"------------DUBA OBJESI SON---------------\n"
        )
    
