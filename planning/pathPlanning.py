from math import sin, cos, sqrt, radians, ceil, atan2, degrees, fabs, floor
from Duba import Duba
from shapely.geometry import Polygon,Point
from shapely.strtree import STRtree
from shapely.prepared import prep
from shapely.ops import unary_union
from heapq import heappush,heappop
from scipy.ndimage import distance_transform_edt
from scipy.interpolate import splprep, splev
from matplotlib.path import Path
import numpy as np
from WGS84Def import *
class WGS84GPS:
    def __init__(self,lat,lon,alt=0): # Derece
        self.lat = lat
        self.lon = lon
        self.alt = alt
    def getTuple(self):
        return (self.lat,self.lon,self.alt)
class ENU:
    def __init__(self,e,n,u=0):
        self.east = e
        self.north = n
        self.up = u
    def getTuple(self):
        return (self.east,self.north,self.up)
class ECEF:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
    def getTuple(self):
        return (self.x,self.y,self.z)
class VehicleMock():
    def __init__(self,center_gps,w,h,angle):
        self.center_gps = center_gps
        self.w = w
        self.h = h
        self.polygon = None
        self.angle = angle
    def getCenterGPS(self):
        return self.center_gps
    def getAngle(self):
        return self.angle
class State():
    def __init__(self,grid_pos,angle,parent,cost=0):
        self.row = grid_pos[0]
        self.column = grid_pos[1]
        self.angle = angle
        self.cost = cost
        self.parent = parent
    def __lt__(self,other):
        return self.cost < other.cost
    def getTuple(self):
        return(self.row, self.column, self.angle)
class PathPlanner():
    def __init__(self,reference_gps,MARGIN,CELL_SIZE=3):
        self.CELL_SIZE = CELL_SIZE
        self.R = 6378137
        self.MARGIN = MARGIN
        self.map_min_x = None
        self.map_min_y = None
        self.map_rows = None
        self.map_cols = None
        self.reference_gps = reference_gps
        self.steering_angles = [-30,0,30]
        self.grid_map = None

    # def haversine(self,gpsA:GPS,gpsB:GPS):
    #     delta_lon = abs( radians(gpsA.lon) - radians(gpsB.lon))
    #     delta_lat =  abs( radians(gpsA.lat) - radians(gpsB.lat))
    #     distance = 2 * self.R * asin(sqrt( ( sin(delta_lat/2)**2)*(cos( delta_lon/2 ) **2) + (cos( delta_lat/2 )**2)*(sin(delta_lon/2)**2)))
    #     return distance
    def euclidianDistance(self,pointA,pointB):
        return sqrt( (pointA[0] - pointB[0]) ** 2 + (pointA[1] - pointB[1]) ** 2)
    def GEOtoGrid(self,min_geo,geo_pos,cell_size):
        min_x,min_y = self.GEOtoENU(min_geo).getTuple()[:2]
        x,y = self.GEOtoENU(geo_pos).getTuple()[:2]
        return self.ENUToGrid(min_x,min_y,x,y,cell_size)
    def ENUToGrid(self,map_min_x,map_min_y,x,y,cell_size):
        index_column = floor ( (x-map_min_x) / cell_size)
        index_row = floor ( (y - map_min_y) / cell_size)
        return (index_row,index_column)
    def gridToENU(self,map_min_x,map_min_y,grid_pos,cell_size):
        row,col = grid_pos
        x = col * cell_size + map_min_x + 0.5 * cell_size
        y = row * cell_size + map_min_y + 0.5 * cell_size
        return ENU(x,y,0)
    def gridToGEO(self,map_min_geo,grid_pos,cell_size):
        min_x, min_y = self.GEOtoENU(map_min_geo).getTuple()[:2]
        row,col = grid_pos
        x = col * cell_size + min_x + 0.5 * cell_size
        y = row * cell_size + min_y + 0.5 * cell_size
        return self.ENUtoGEO(ENU(x,y,0))
    def calculateMapSize(self,map_bounds,cell_size):
        self.map_max_x,self.map_max_y = self.GEOtoENU(map_bounds[1]).getTuple()[:2] # upper boundries
        self.map_min_x,self.map_min_y = self.GEOtoENU(map_bounds[0]).getTuple()[:2] #lower boundries
        self.map_cols = ceil( (self.map_max_x-self.map_min_x) / cell_size)
        self.map_rows = ceil( (self.map_max_y-self.map_min_y) / cell_size)
        return (self.map_rows,self.map_cols)
    def generatePolygonFromGEO(self, center_gps, w, h, angle, margin):
        margin = self.MARGIN
        cx,cy = self.GEOtoENU(center_gps).getTuple()[:2]
        polygon = Polygon(self.rotateRect(cx,cy,w+margin,h+margin,angle))
        return polygon
    def generatePolygonFromENU(self,cx,cy,w,h,angle,margin):
        margin = self.MARGIN
        polygon = Polygon(self.rotateRect(cx,cy,w+margin,h+margin,angle))
        return polygon
    def generatePolygonFromState(self,state,vehicle,margin): # Sıkıntılı (vehicle.w/h ve margin, grid veya enu cinsinden değil)
        margin = self.MARGIN
        enu = self.gridToENU(self.map_min_x,self.map_min_y,(state.row,state.column),self.CELL_SIZE)
        polygon = Polygon(self.rotateRect(enu.east,enu.north,vehicle.w+margin,vehicle.h+margin,state.angle))
        return polygon
    def rotateRect(self,center_x,center_y,width,height,angle):
        angle = radians(angle)
        corners = [ # relative positions
            (+width/2, +height/2),   # top-right
            (-width/2, +height/2),   # top-left
            (-width/2, -height/2),   # bottom-left
            (+width/2, -height/2)    # bottom-right
        ]
        rotated_corners = [
            (center_x + x*cos(angle) - y*sin(angle),
            center_y + x*sin(angle) + y*cos(angle))
            for (x, y) in corners
            ]
        return rotated_corners
    def calculateBounds(self,vehicle,obstacles,margin):
        margin = self.MARGIN
        polygons = []
        if vehicle:
            vehicle.polygon = self.generatePolygonFromGEO(vehicle.getCenterGPS(),vehicle.w + margin,vehicle.h + margin,vehicle.getAngle(), margin)
            polygons.append(vehicle.polygon)
        if obstacles:
            for duba in obstacles:
                duba.polygon = self.generatePolygonFromGEO(duba.center_gps,duba.w+margin,duba.h+margin,duba.angle, margin)
                polygons.append(duba.polygon)
        full_area = unary_union(polygons)

        minx, miny, maxx, maxy = full_area.bounds

        min_lat, min_lon = self.ENUtoGEO(ENU(minx,miny)).getTuple()[:2]
        max_lat, max_lon = self.ENUtoGEO(ENU(maxx,maxy)).getTuple()[:2]
        return WGS84GPS(min_lat, min_lon, 0), WGS84GPS(max_lat, max_lon, 0)
    def createMap(self,map_size,obstacles):
        curr_map = np.zeros(map_size,dtype=int)
        for duba in obstacles:
            self.addObstacleToGridEx(curr_map,duba,map_size)
            #self.rasterize_polygon(curr_map,duba,self.map_min_x,self.map_min_y,self.CELL_SIZE)
        return curr_map
    def addObstacleToGrid(self,grid,duba,map_size):
        if duba.polygon is None:
            duba.polygon = self.generatePolygonFromGEO(duba.center_gps,duba.w,duba.h,duba.angle,self.MARGIN)
        minx,miny,maxx, maxy = duba.polygon.bounds
        gymin,gxmin = self.ENUToGrid(self.map_min_x,self.map_min_y,minx,miny,self.CELL_SIZE)
        gymax,gxmax = self.ENUToGrid(self.map_min_x,self.map_min_y,maxx,maxy,self.CELL_SIZE)
        gxmin = max(gxmin,0)
        gymin = max(gymin,0)
        gxmax = min(gxmax,map_size[1]-1)
        gymax = min(gymax,map_size[0]-1)

        for row in range(gymin,gymax+1):
            cy = self.map_min_y + (row + 0.5) * self.CELL_SIZE
            for column in range(gxmin,gxmax+1):
                    cx = self.map_min_x + (column + 0.5) * self.CELL_SIZE

                    cell_poly = Polygon([
                (cx - self.CELL_SIZE/2, cy - self.CELL_SIZE/2),
                (cx + self.CELL_SIZE/2, cy - self.CELL_SIZE/2),
                (cx + self.CELL_SIZE/2, cy + self.CELL_SIZE/2),
                (cx - self.CELL_SIZE/2, cy + self.CELL_SIZE/2)
            ])
                    if duba.polygon.intersects(cell_poly):
                        grid[row, column] = 1
                        #p = Point(cx, cy)
                        #print("point P:",p)
                        #print("duba polygon: ", duba.polygon)
                        #if duba.polygon.contains(p):
                            #print("contains")
                            #grid[row,column] = 1


        # points = [
        # Point(self.map_min_x + (col + 0.5) * self.CELL_SIZE,
        #     self.map_min_y + (row + 0.5) * self.CELL_SIZE)
        # for row in range(gymin, gymax + 1)
        # for col in range(gxmin, gxmax + 1)
        # ]
        # print("points",points)
        # tree = STRtree(points)
        # indices = tree.query(duba.polygon, predicate="within")
        # print("incides: ",indices)
        # width = gxmax - gxmin + 1
        # for idx in indices:
        #     row = gymin + idx // width
        #     col = gxmin + idx % width
        #     grid[row, col] = 1
    def rasterize_polygon(self,grid,obstacle,map_min_x,map_min_y,cell_size):
        obstacle_poly = obstacle.polygon if obstacle.polygon else self.generatePolygonFromGEO(obstacle.center_gps,obstacle.w,obstacle.h,obstacle.angle,self.MARGIN)
        poly = np.array(obstacle_poly.exterior.coords)
        path = Path(poly)
        h, w = grid.shape
        xs = np.arange(w) * cell_size + map_min_x + cell_size/2
        ys = np.arange(h) * cell_size + map_min_y + cell_size/2
        xv, yv = np.meshgrid(xs, ys)
        points = np.vstack((xv.flatten(), yv.flatten())).T
        mask = path.contains_points(points).reshape(h, w)
        grid[mask] = 1
    def addObstacleToGridEx(self,grid,duba,map_size):
        duba_poly = duba.polygon if duba.polygon else self.generatePolygonFromGEO(duba.center_gps,duba.w,duba.h,duba.angle,self.MARGIN)
        prepped = prep(duba_poly)
        minx,miny,maxx, maxy = duba.polygon.bounds
        gymin,gxmin = self.ENUToGrid(self.map_min_x,self.map_min_y,minx,miny,self.CELL_SIZE)
        gymax,gxmax = self.ENUToGrid(self.map_min_x,self.map_min_y,maxx,maxy,self.CELL_SIZE)
        gxmin = max(gxmin,0)
        gymin = max(gymin,0)
        gxmax = min(gxmax,map_size[1]-1)
        gymax = min(gymax,map_size[0]-1)
        for row in range(gymin,gymax+1):
            cy = self.map_min_y + (row+0.5) * self.CELL_SIZE
            for column in range(gxmin,gxmax+1):
                cx = self.map_min_x + (column + 0.5) * self.CELL_SIZE
                cell_poly = Polygon([
                (cx - self.CELL_SIZE/2, cy - self.CELL_SIZE/2),
                (cx + self.CELL_SIZE/2, cy - self.CELL_SIZE/2),
                (cx + self.CELL_SIZE/2, cy + self.CELL_SIZE/2),
                (cx - self.CELL_SIZE/2, cy + self.CELL_SIZE/2)
                ])
                if prepped.intersects(cell_poly):
                    grid[row,column] = 1
    def isCollision(self,state,vehicle,obstacles_prepped):
        enu = self.gridToENU(self.map_min_x, self.map_min_y, (state.row, state.column), self.CELL_SIZE)
        cx, cy = enu.east, enu.north

        # Fast bounding box check first
        vehicle_box = Polygon([
            (cx - vehicle.w/2, cy - vehicle.h/2),
            (cx + vehicle.w/2, cy - vehicle.h/2),
            (cx + vehicle.w/2, cy + vehicle.h/2),
            (cx - vehicle.w/2, cy + vehicle.h/2)
        ])

        # if bounding boxes are far away → skip shapely intersection
        for p in obstacles_prepped:
            if not p.context.envelope.intersects(vehicle_box.envelope):
                continue
            if p.intersects(vehicle_box):
                return True
        return False
    def isCollisionOrigin(self,state,vehicle,obstacles_prepped):
        cell_poly = self.generatePolygonFromState(state,vehicle,self.MARGIN)
        return any(p.intersects(cell_poly) for p in obstacles_prepped)
    def generateNeighboursEx(self,state,vehicle,map_size,obstacles_polygons,steering_angles=[-30,0,30],step=3):
        neighbours = []
        steering_angles = [radians(angle) for angle in steering_angles]
        rads = [radians(a) for a in self.steering_angles]
        for rad in rads:
            new_angle = (state.angle + degrees(rad)) % 360
            dx = state.column + cos(rad) * step
            dy = state.row + sin(rad) * step
        for steering_angle in steering_angles:
            new_angle = (state.angle + steering_angle) % 360
            new_column = round(state.column + cos(radians(new_angle))*step)
            new_row = round(state.row + sin(radians(new_angle))*step)
            #new_column = min(max(0,new_column),map_size[1])
            #new_row = min(max(0,new_row),map_size[0])
            new_state = State((new_row,new_column),new_angle,parent=state)
            if not self.isCollision(new_state,vehicle,obstacles_polygons):
                 neighbours.append(new_state)
        return neighbours
    def generateNeighbours(self,state,vehicle,map_size,obstacles_polygons,steering_angles=[-30,0,30],step=3):
        neighbours = []
        steering_angles = [radians(angle) for angle in steering_angles]
        for steering_angle in steering_angles:
            new_angle = (state.angle + degrees(steering_angle)) % 360
            new_column = int(state.column + cos(radians(new_angle))*step)
            new_row = int(state.row + sin(radians(new_angle))*step)
            #new_column = min(max(0,new_column),map_size[1])
            #new_row = min(max(0,new_row),map_size[0])
            new_state = State((new_row,new_column),new_angle,parent=state)
            if not self.isCollision(new_state,vehicle,obstacles_polygons):
                 neighbours.append(new_state)
        return neighbours
    def generateNeighboursEx(self,state,vehicle,map_size,obstacles_polygons,steering_angles=[-30,0,30],step=3):
        neighbours = []
    def calculateDistanceMap(self,grid_map):
        inversed_map = grid_map == 0
        return distance_transform_edt(inversed_map)
    def calculateCost(self,current_state,new_state,start,goal,distance_map):
        d = distance_map[round(new_state.row),round(new_state.column)]
        t = 1 / max(d,0.1)
        g = self.euclidianDistance( (new_state.row,new_state.column), (start.row,start.column) ) + self.euclidianDistance((new_state.row,new_state.column), (current_state.row,current_state.column))
        h = self.euclidianDistance( (new_state.row,new_state.column), (goal.row, goal.column) )
        return g + h + t
    def prepObstaclesPolygons(self,obstacles):
        prepped_obstacles = []
        for obstacle in obstacles:
            if not obstacle.polygon:
                obstacle.polygon = self.generatePolygonFromGEO(obstacle.center_gps,obstacle.w,obstacle.h,obstacle.angle,self.MARGIN)
            prepped_obstacles.append(prep(obstacle.polygon))
        return prepped_obstacles

    def planPath(self,start_geo,goal_geo,obstacles,vehicle):
        print("planPath")
        print("calculateBounds")
        prepped_obstacles_polygons = self.prepObstaclesPolygons(obstacles)
        map_bounds = self.calculateBounds(vehicle,obstacles,self.MARGIN)
        print("calculateMapSize")
        map_size = self.calculateMapSize(map_bounds,self.CELL_SIZE)
        print("createMap")
        grid_map = self.createMap(map_size,obstacles)
        print("calculateDistanceMap")
        dist_map = self.calculateDistanceMap(grid_map)
        start_geo = start_geo if start_geo else vehicle.center_gps
        goal_grid_pos = self.GEOtoGrid(map_bounds[0],goal_geo,self.CELL_SIZE)
        goal_state = State(goal_grid_pos,0,None)
        start_grid_pos = self.GEOtoGrid(map_bounds[0],start_geo,self.CELL_SIZE)
        start_state = State(start_grid_pos,vehicle.angle,None)
        open_list = [] # Possible States
        open_dict = {} # Key -> State
        closed_set = set() # Visited cells
        heappush(open_list,start_state)
        open_dict[start_state.getTuple()] = start_state
        print("before loop")
        while open_list:
            current = heappop(open_list)
            key = (round(current.row), round(current.column), round(current.angle, 2))
            if key in closed_set: continue
            closed_set.add(key)
            if self.isGoal(current,goal_state,1.5): return self.reconstructPath(current,vehicle,prepped_obstacles_polygons)
            print("loop")
            for neighbour in self.generateNeighbours(current,vehicle,map_size,prepped_obstacles_polygons,self.steering_angles,3):
                if self.isValidState(neighbour,map_size):
                    neighbour_key = (int(neighbour.row), int(neighbour.column), int(neighbour.angle, 2))
                    if neighbour_key in closed_set: continue
                    neighbour.cost = self.calculateCost(current,neighbour,start_state,goal_state,dist_map)
                    neighbour.parent = current
                    if neighbour_key not in open_dict or neighbour.cost < open_dict[neighbour_key].cost:
                        print("adding to open list")
                        heappush(open_list,neighbour)
                        open_dict[neighbour_key] = neighbour
        print("yol bulamadi...")
    def isValidState(self,neighbour,map_size):
        max_row, max_column = map_size
        return neighbour.row < max_row and neighbour.row >= 0 and neighbour.column < max_column and neighbour.column >= 0
    
    def isGoal(self,current,goal,threshold_meters):
        return self.euclidianDistance((current.row,current.column),(goal.row,goal.column)) * self.CELL_SIZE < threshold_meters
    
    def smoothPathOrigin(self,path_in_ENU,n_degree=2,smoothing_factor=0):
        points = np.array( [ p.getTuple()[:2] for p in path_in_ENU  ] )
        print(points)
        tck, u = splprep(points.T, k=n_degree) # returns (vector knots, coefficents, degree) and array of parameters
        u_fine = np.linspace(0,1,1000)
        smooth_path_nd = splev(u_fine,tck)
        print(smooth_path_nd)
        print(type(smooth_path_nd))
        smoothed_path = [ENU(e, n, 0) for e, n in zip(smooth_path_nd[0], smooth_path_nd[1])]
        return smoothed_path
    def simplifyPath(self, path_in_ENU, vehicle, obstacles):
        simplified_path = [path_in_ENU[0]]
        i = 0
        while i < len(path_in_ENU)-1:
            found = False
            j = len(path_in_ENU) - 1
            while j > i + 1:
                if self.canConnect(path_in_ENU[i],path_in_ENU[j],vehicle,obstacles):
                    simplified_path.append(path_in_ENU[j])
                    i = j
                    found = True
                    break
                j -= 1
            if not found:
                simplified_path.append(path_in_ENU[i+1])
                i += 1
            if i == len(path_in_ENU) - 2:
                simplified_path.append(path_in_ENU[-1]) # last element
            print("lensimp",len(simplified_path))
        return simplified_path
    def canConnect(self, pointA_enu, pointB_enu, vehicle, obstacles,num_steps = 10):
        print("step size: ",num_steps)
        for i in range(num_steps+1):
            t = i / max(1,num_steps)
            inter_x = pointA_enu.east + t*(pointB_enu.east - pointA_enu.east)
            inter_y = pointA_enu.north + t*(pointB_enu.north - pointA_enu.north)
            dx = pointB_enu.east - pointA_enu.east
            dy = pointB_enu.north - pointA_enu.north
            angle = degrees(atan2(dy, dx)) % 360
            print("angle",angle)
            if self.isCollision(State(self.ENUToGrid(self.map_min_x,self.map_min_y,inter_x,inter_y,self.CELL_SIZE),angle,None),vehicle,obstacles ): return False
            print("NOT COL")
        return True
    def smoothPath(self, path_in_ENU, n_degree=3, smoothing_factor=2):
        # Convert path to 2D points
        points = np.array([p.getTuple()[:2] for p in path_in_ENU])
        
        # Remove consecutive duplicate points
        points_unique = np.array([points[0]] + [p for i, p in enumerate(points[1:]) if not np.all(p == points[i])])
        
        # Ensure there are enough points for the requested spline degree
        k = min(n_degree, len(points_unique) - 1)
        if k < 1:
            # Too few points to smooth, just return original path
            print("few points to smooth")
            return path_in_ENU
        print("not few points")
        # Create spline
        tck, u = splprep(points_unique.T, k=k, s=smoothing_factor)
        # Sample fine points along the spline
        u_fine = np.linspace(0, 1, 1000)
        smooth_path_nd = splev(u_fine, tck)
        
        # Convert back to ENU objects with Z=0
        smoothed_path = [ENU(e, n, 0) for e, n in zip(smooth_path_nd[0], smooth_path_nd[1])]
        
        return smoothed_path
    def smoothPathEx(self,path_in_ENU,n_degree=3):
        k = len(path_in_ENU)
        n = n_degree # degree
        m = k + n # number of knots
        knot_vector = [0 for i in range(0,n)]
        knot_vector.extend([1 for i in range(m-n,m)])

    def reconstructPath(self,goal,vehicle,prepped_obstacles):
        path = []
        state = goal
        while state is not None:
            path.append(state)
            state = state.parent
        path.reverse()
        print("path first: ",len(path))
        enu_path = [self.gridToENU(self.map_min_x,self.map_min_y,(p.row,p.column),self.CELL_SIZE) for p in path]
        simplified_enu_path = self.simplifyPath(enu_path,vehicle,prepped_obstacles)
        return simplified_enu_path
    
    def GEOtoECEF(self,gps:WGS84GPS):
        ecef = ECEF(0,0,0)
        lat = radians(gps.lat)
        lon = radians(gps.lon)
        alt = gps.alt
        N = WGS84_AADC / sqrt(cos(lat) * cos(lat) + WGS84_BBDCC)
        d = (N + alt) * cos(lat)
        ecef.x = d * cos(lon)
        ecef.y = d * sin(lon)
        ecef.z = (WGS84_P1MEE * N + alt) * sin(lat)
        return ecef
    def ECEFtoENU(self,ecef:ECEF,reference_gps:WGS84GPS):
        x0, y0, z0 = self.GEOtoECEF(reference_gps).getTuple()
        dx = ecef.x - x0
        dy = ecef.y - y0
        dz = ecef.z - z0
        lam = radians(reference_gps.lon)
        phi = radians(reference_gps.lat)
        sinp = sin(phi) 
        cosp = cos(phi)
        sinl = sin(lam) 
        cosl = cos(lam)
        east  = -sinl * dx + cosl * dy
        north = -sinp * cosl * dx - sinp * sinl * dy + cosp * dz
        up    =  cosp * cosl * dx + cosp * sinl * dy + sinp * dz
        return ENU(east,north,up)
    def GEOtoENU(self,gps:WGS84GPS):
        ecef = self.GEOtoECEF(gps)
        return self.ECEFtoENU(ecef,self.reference_gps)
    def ENUtoGEO(self,enu:ENU):
        ecef = self.ENUtoECEF(enu,self.reference_gps)
        return self.ECEFtoGEO(ecef)
    def ENUtoECEF(self,enu:ENU,reference_gps:WGS84GPS):
        x0, y0, z0 = self.GEOtoECEF(reference_gps).getTuple()
        lam = radians(reference_gps.lon)
        phi = radians(reference_gps.lat)
        sinp = sin(phi)
        cosp = cos(phi)
        sinl = sin(lam)
        cosl = cos(lam)
        dx = -sinl * enu.east - sinp * cosl * enu.north + cosp * cosl * enu.up
        dy =  cosl * enu.east - sinp * sinl * enu.north + cosp * sinl * enu.up
        dz =  cosp * enu.north + sinp * enu.up
        x = x0 + dx
        y = y0 + dy
        z = z0 + dz
        return ECEF(x, y, z)
    def ECEFtoGEO(self,ecef:ECEF):
        geo = WGS84GPS(0,0,0)
        x = ecef.x
        y = ecef.y
        z = ecef.z
        ww = x * x + y * y
        m = ww * WGS84_INVAA
        n = z * z * WGS84_P1MEEDAA
        mpn = m + n
        p = WGS84_INV6 * (mpn - WGS84_EEEE)
        G = m * n * WGS84_EEEED4
        H = 2 * p * p * p + G
        if H < WGS84_HMIN: return -1
        C = pow(H + G + 2 * sqrt(H * G), WGS84_INV3) * WGS84_INVCBRT2
        i = -WGS84_EEEED4 - 0.5 * mpn
        P = p * p
        beta = WGS84_INV3 * i - C - P / C
        k = WGS84_EEEED4 * (WGS84_EEEED4 - mpn)
        t1 = beta * beta - k
        t2 = sqrt(t1)
        t3 = t2 - 0.5 * (beta + i)
        t4 = sqrt(t3)
        t5 = 0.5 * (beta - i)
        t5 = fabs(t5)
        t6 = sqrt(t5)
        t7 = t6 if (m<n) else -t6
        t = t4 + t7
        j = WGS84_EED2 * (m - n)
        g = 2 * j
        tt = t * t
        ttt = tt * t
        tttt = tt * tt
        F = tttt + 2 * i * tt + g * t + k
        dFdt = 4 * ttt + 4 * i * t + g
        dt = -F / dFdt
        u = t + dt + WGS84_EED2
        v = t + dt - WGS84_EED2
        w = sqrt(ww)
        zu = z * u
        wv = w * v
        lat = atan2(zu, wv)
        invuv = 1 / (u * v)
        dw = w - wv * invuv
        dz = z - zu * WGS84_P1MEE * invuv
        da = sqrt(dw * dw + dz * dz)
        alt = -da if (u<1) else da
        lon = atan2(y, x)
        geo.lat = degrees(lat)
        geo.lon = degrees(lon)
        geo.alt = alt
        return geo