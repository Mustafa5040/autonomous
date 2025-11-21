from pathPlanning import *
from sys import maxsize
from math import pow
import matplotlib.pyplot as plt
vehicle = VehicleMock(WGS84GPS(37.7749, -122.4191, 0),1,2,0)
reference_gps = WGS84GPS(37.7749, -122.4191, 0)
test_gps = WGS84GPS(37.7765, -122.4199, 0)
test_lower_bound_gps = WGS84GPS(37.7740,-122.4185,0)
test_vehicle = VehicleMock(center_gps=WGS84GPS(37.77470, -122.4143, 0), w=1, h=1, angle=0)
goal_geo = WGS84GPS(37.77505, -122.4136, 0)
# test_obstacles = [
#     Duba(center_gps=WGS84GPS(37.7750, -122.4195, 0), w=0.5, h=1, angle=45),
#     Duba(center_gps=WGS84GPS(37.7744, -122.4192, 0), w=0.5, h=1, angle=45),
#     Duba(center_gps=WGS84GPS(37.7741, -122.4192, 0), w=0.5, h=1, angle=45),
#     Duba(center_gps=WGS84GPS(37.7740, -122.4190, 0), w=0.5, h=1, angle=45),
#     Duba(center_gps=WGS84GPS(37.7739, -122.4192, 0), w=0.5, h=1, angle=45),
#     Duba(center_gps=WGS84GPS(37.7742, -122.4194, 0), w=0.5, h=1, angle=45),
#     Duba(center_gps=WGS84GPS(37.7737, -122.4190, 0), w=0.5, h=1, angle=45)
# ]

test_obstacles = [
     Duba(center_gps=WGS84GPS(37.7750, -122.41359, 0), w=1, h=1, angle=0),
     Duba(center_gps=WGS84GPS(37.7751, -122.41375, 0), w=1, h=1, angle=0),
     Duba(center_gps=WGS84GPS(37.7750, -122.4141, 0), w=1, h=1, angle=0),
     Duba(center_gps=WGS84GPS(37.7749, -122.41389, 0), w=1, h=1, angle=0),
     Duba(center_gps=WGS84GPS(37.77485, -122.4140, 0), w=1, h=1, angle=0),
     # Added more obstacles to create a denser, maze-like environment for harder path planning
     Duba(center_gps=WGS84GPS(37.77475, -122.4141, 0), w=1, h=1, angle=0),  # Blocks near start
     Duba(center_gps=WGS84GPS(37.7748, -122.4139, 0), w=1, h=1, angle=0),   # Mid-path blocker
     Duba(center_gps=WGS84GPS(37.7749, -122.4137, 0), w=1, h=1, angle=0),   # Near goal approach
     Duba(center_gps=WGS84GPS(37.7750, -122.4137, 0), w=1, h=1, angle=0),   # Creates a narrow passage
     Duba(center_gps=WGS84GPS(37.77505, -122.4138, 0), w=1, h=1, angle=0),  # Forces detour
     Duba(center_gps=WGS84GPS(37.77495, -122.41395, 0), w=1, h=1, angle=0), # Additional cluster
     Duba(center_gps=WGS84GPS(37.77515, -122.41385, 0), w=1, h=1, angle=0), # Northern blocker
     Duba(center_gps=WGS84GPS(37.77465, -122.41405, 0), w=1, h=1, angle=0)  # Southern blocker near start
 ]
testobj = Duba(center_gps=WGS84GPS(37.7750, -122.4195, 0), w=0.5, h=1, angle=45)

planner = PathPlanner(reference_gps, 1.0,0.5)

def testCoordinateConversion():
    tolerance = 1e-6 * planner.CELL_SIZE 
    ecef = planner.GEOtoECEF(test_gps)
    enu = planner.ECEFtoENU(ecef,reference_gps)
    geo_back = planner.ECEFtoGEO(planner.ENUtoECEF(enu,reference_gps))
    lat_diff = abs(test_gps.lat - geo_back.lat)
    lon_diff = abs(test_gps.lon - geo_back.lon)
    alt_diff = abs(test_gps.alt - geo_back.alt)
    print(f"GEO-ECEF-GEO Conversion differences: lat={lat_diff:.6e}, lon={lon_diff:.6e}, alt={alt_diff:.6e}")
    assert lat_diff < tolerance, f"Latitude difference {lat_diff} exceeds tolerance {tolerance}"
    assert lon_diff < tolerance, f"Longitude difference {lon_diff} exceeds tolerance {tolerance}"
    assert alt_diff < tolerance, f"Altitude difference {alt_diff} exceeds tolerance {tolerance}"
    print("GEO-ECEF - ECEF-GEO transformation test passed!")
def testGEOGRIDTransformation():
    tolerance = 1e-5 * planner.CELL_SIZE
    print(tolerance)
    grid_pos = planner.GEOtoGrid(test_lower_bound_gps,test_gps,planner.CELL_SIZE)
    geo_back = planner.gridToGEO(test_lower_bound_gps,grid_pos,planner.CELL_SIZE)
    lat_diff_percentage = abs(test_gps.lat - geo_back.lat) / abs(test_gps.lat) * 100
    lat_diff = abs(test_gps.lat - geo_back.lat)
    lon_diff = abs(test_gps.lon - geo_back.lon)
    alt_diff = abs(test_gps.alt - geo_back.alt)
    print(f"GEO-GRID-GEO Conversion differences: lat={lat_diff_percentage:.5f}%, lon={lon_diff:.6e}, alt={alt_diff:.6e}")
    assert lat_diff < tolerance, f"Latitude difference {lat_diff} exceeds tolerance {tolerance}"
    assert lon_diff < tolerance, f"Longitude difference {lon_diff} exceeds tolerance {tolerance}"
    #ALTITUDE HESAPLAMASI HATALI OLABİLİR ??
    #assert alt_diff < tolerance, f"Altitude difference {alt_diff} exceeds tolerance {tolerance}"
    print("GEO-WORLD-GRID-WORLD-GEO transformation test passed!")
def testCalculateBounds():
    tolerance = 1e-3 * planner.CELL_SIZE
    bounds = planner.calculateBounds(test_vehicle,test_obstacles,planner.MARGIN)
    lower_bound = bounds[0]
    upper_bound = bounds[1]
    test_lower_bound = WGS84GPS(maxsize,maxsize,maxsize)
    test_upper_bound = WGS84GPS(-maxsize,-maxsize,-maxsize)
    test_objects = test_obstacles.copy()
    test_objects.append(test_vehicle)
    for obj in test_objects:
        test_lower_bound.lat = min(test_lower_bound.lat,obj.center_gps.lat)
        test_lower_bound.lon = min(test_lower_bound.lon,obj.center_gps.lon)
        test_lower_bound.alt = min(test_lower_bound.alt,obj.center_gps.alt)
        test_upper_bound.lat = max(test_upper_bound.lat,obj.center_gps.lat)
        test_upper_bound.lon = max(test_upper_bound.lon,obj.center_gps.lon)
        test_upper_bound.alt = max(test_upper_bound.alt,obj.center_gps.alt)
    #error = abs(test_lower_bound.lat - lower_bound.lat) / abs(test_lower_bound.lat) * 100
    #print(error)
    #print(f"ERROR RATE : {error:.5f}%")
    assert abs(lower_bound.lat - test_lower_bound.lat) < tolerance , f"Lower Latitude Bound (CalcBounds) Mismatch, expected {lower_bound.lat} got {test_lower_bound.lat}"
    assert abs(lower_bound.lon - test_lower_bound.lon) < tolerance, f"Lower Latitude Bound (CalcBounds) Mismatch, expected  {lower_bound.lon} got {test_lower_bound.lon}"
    assert abs(lower_bound.alt - test_lower_bound.alt) < tolerance, f"Lower Latitude Bound (CalcBounds) Mismatch, expected {lower_bound.alt} got {test_lower_bound.alt}"
    assert abs(upper_bound.lat - test_upper_bound.lat) < tolerance, f"Upper Latitude Bound (CalcBounds) Mismatch, expected {upper_bound.lat} got {test_upper_bound.lat}"
    assert abs(upper_bound.lon - test_upper_bound.lon) < tolerance, f"Upper Longitude Bound (CalcBounds) Mismatch, expected {upper_bound.lon} got {test_upper_bound.lon}"
    assert abs(upper_bound.alt - test_upper_bound.alt) < tolerance, f"Upper Latitude Bound (CalcBounds) Mismatch, expected {upper_bound.alt} got {test_upper_bound.lat}"
    print("Bounds Calculations (calculateBounds) test passed!")
def testCalculateMapSize():
    #test 1
    max_gps = WGS84GPS(37.7759, -122.4184,0)
    min_gps = WGS84GPS(37.7740, -122.4204,0)
    rows,cols = planner.calculateMapSize(max_gps,min_gps,planner.CELL_SIZE)
    assert rows > 0 and cols > 0, f"CalculateMapSize function returns negative rows or cols on Normal TEST!"
    #EDGE CASE 1
    max_gps = WGS84GPS(37.7759, -122.4184,0)
    min_gps = WGS84GPS(37.7759, -122.4184,0)
    rows,cols = planner.calculateMapSize(max_gps,min_gps,planner.CELL_SIZE)
    assert rows == cols, f"EDGE CASE 1: (Upper and Lower bound is the same): row and cols isn not 0!, ROWS:{rows}, COLS:{cols}"
def testSmoothPath():
    path_in_enu = [ ENU(0,0,0), ENU(12,0,0), ENU(12,14,0), ENU(13,15,0), ENU(14,19,0), ENU(16,21,0)]
    path_as_points = [p.getTuple()[:2] for p in path_in_enu]
    path_x = [ p[0] for p in path_as_points]
    path_y = [ p[1] for p in path_as_points]

    smoothed_path = planner.smoothPath(path_in_enu,smoothing_factor=0.5)

    smoothed_path_as_points = [p.getTuple()[:2] for p in smoothed_path]
    smoothed_path_x = [ p[0] for p in smoothed_path_as_points]
    smoothed_path_y = [ p[1] for p in smoothed_path_as_points]
    plt.plot(path_x,path_y,marker='o')
    #plt.plot(path_y, marker = 'o')
    plt.plot(smoothed_path_x,smoothed_path_y,marker='o')
    #plt.plot(smoothed_path_y, marker = 'o')
    plt.show()
def testPlanPath():
    obstacles_ENUs = [planner.GEOtoENU(d.center_gps) for d in test_obstacles]
    obstacles_x = [p.getTuple()[0] for p in obstacles_ENUs]
    obstacles_y = [p.getTuple()[1] for p in obstacles_ENUs]
    plt.plot(obstacles_x,obstacles_y,'o')
    v_enu = planner.GEOtoENU(test_vehicle.getCenterGPS())
    plt.plot(v_enu.east,v_enu.north,'o')
    g_enu = planner.GEOtoENU(goal_geo)
    plt.plot(g_enu.east,g_enu.north,'o')
    plt.text(g_enu.east,g_enu.north,"goal")
    plt.text(v_enu.east,v_enu.north,"vehicle")
    #plt.plot(obstacles_y,'o')
    #plt.show()
    path = planner.planPath(test_vehicle.getCenterGPS(),goal_geo,test_obstacles,test_vehicle)
    path_x = [p.getTuple()[0] for p in path]
    path_y = [p.getTuple()[1] for p in path]
    plt.plot(path_x,path_y,marker='o')
    plt.show()

#testCoordinateConversion()
#testGEOGRIDTransformation()
#testCalculateBounds()
#testCalculateMapSize()
#testSmoothPath()
testPlanPath()