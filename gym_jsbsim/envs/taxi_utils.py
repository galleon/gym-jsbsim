import numpy as np
from shapely import ops
from shapely.geometry import MultiPoint,Point,MultiLineString,LineString
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, atan2, radians,pi, acos
from geographiclib.geodesic import Geodesic
import geopandas as gpd
import pandas as pd
import os
from gym_jsbsim.catalogs import utils


def get_bearing(p1, p2):
    """
    :param p1: (long,lat)
    :param p2: (long,lat)
    :return: bearing in degrees
    """
    long1, lat1 = p1
    long2, lat2 = p2
    brng = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']
    return brng


def get_distance(p1, p2):
    """
    :param p1: (long,lat)
    :param p2: (long,lat)
    :return: distance in km
    """
    lat1 = radians(p1[1])
    lon1 = radians(p1[0])
    lat2 = radians(p2[1])
    lon2 = radians(p2[0])
    dist = 6371.01 * acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon1 - lon2))
    return dist


class taxi_path(object):
    """
    methods to use:
    1. self.update(ref_pts,aircraft_heading_deg)  -> list[[(x,y),distance,heading,(long,lat)],[.....]]
    2. self.shortest_dist  -> returns the shortest (normal) distance to target path.
    3. self.plot() -> to visualize the airport, current path, next target points and aircraft position.
    """

    def __init__(self, path_to_points="../points.json", number_of_points_to_use=10):
        """
        :param ambd_folder_path: path to folder with airport shapefiles
        :param number_of_points_to_use: number of next points to return
        """

        # path_to_points = '/home/jyotsna/Documents/RL/learning-to-fly-master/code/notebooks/points.json'
        points_df = pd.read_json(path_to_points, orient='index', precise_float=True, )
        points_df.columns = ['long', 'lat']
        points_df['points'] = points_df.apply(lambda row: (row.long, row.lat), axis=1)
        points_df.drop(['long', 'lat'], axis=1, inplace=True)
        # add relative properties of points
        relative_distance = [0]
        relative_distance[1:] = [
            float("{:.4f}".format(get_distance(points_df.points.iloc[i], points_df.points.iloc[i - 1]))) for i in
            range(1, points_df.shape[0])]
        relative_bearing = [0]
        relative_bearing[1:] = [
            float("{:.4f}".format(get_bearing(points_df.points.iloc[i - 1], points_df.points.iloc[i]))) for i in
            range(1, points_df.shape[0])]

        points_df['rel_distance'] = relative_distance
        points_df['rel_bearing'] = relative_bearing
        self.points_df = points_df

        self.centerline = LineString(list(points_df.points.values[:]))
        self.n = number_of_points_to_use
        # to update in reward function
        self.shortest_dist = None

    def update_path(self, aircraft_loc, aircraft_heading):
        """
        :param ref_pts: aircraft (longitude,latitude)
        :param ac_heading:
        :return: list[[(long,lat),distance,heading],[.....]]
        """

        n = self.n
        self.aircraft_loc = aircraft_loc
        line = MultiPoint(list(self.points_df.points.values[:]))
        nearest_point = ops.nearest_points(line, Point(aircraft_loc))[0]
        idx = self.points_df[self.points_df.points == nearest_point.coords[:2][0]].index[0]
        shortest_dist = float("{:.4f}".format(get_distance(aircraft_loc, self.points_df.points.loc[idx])))
        target_heading = (float("{:.4f}".format(get_bearing(aircraft_loc, self.points_df.points.loc[idx]))) + 360) % 360
        delta_heading = utils.reduce_reflex_angle_deg(aircraft_heading - target_heading)
        
        
        if abs(delta_heading) > 90:
            idx = idx + 1
            shortest_dist = float("{:.4f}".format(get_distance(aircraft_loc, self.points_df.points.loc[idx])))
            target_heading = (float("{:.4f}".format(get_bearing(aircraft_loc, self.points_df.points.loc[idx]))) + 360) % 360
        
        #print(target_heading)


        output = [[nearest_point.coords[:2][0], shortest_dist, target_heading]]
        output2 = [[self.points_df.points.loc[i], self.points_df.rel_distance.loc[i], self.points_df.rel_bearing.loc[i]]
                   for i in range(idx + 1, idx + n)]
        output += output2

        self.shortest_dist = self.centerline.distance(Point(aircraft_loc))

        return output


#--------------------------------------------------------------
# FUNCTION TO GENERATE points.json file used in taxi_path class
#--------------------------------------------------------------
def get_path_points(ambd_folder_path="../amdb", save_folder=os.getcwd()):
    path_id_numbers = [1977, 1974, 1973, 2001, 2202, 2203, 2453, 2204, 2206, 2000, 1996, 1968, 1969,
                       1971, 2020, 2019, 2018, 1931, 1929, 1925, 1926, 2031, 2042, 2134, 2091, 2099,
                       2103, 2385, 2105, 2136, 1978, 1947, 1949]
    number_of_points_to_use = 5
    shapefile_dir = ambd_folder_path  # 'amdb' folder path
    id_str = ['%d' % x for x in path_id_numbers]
    centerline = 'AM_AsrnEdge.shp'
    n = number_of_points_to_use
    shortest_dist = None

    # get linestrings in the right order
    df = gpd.read_file(shapefile_dir + '/' + centerline)
    df = df[df['idnumber'].isin(id_str)][['idnumber', 'geometry']]
    order = {}
    i = 1
    for id_n in id_str:
        order[id_n] = i
        i += 1
    df['order'] = df.idnumber.map(order)
    df.index = df.order
    df.sort_index(inplace=True)

    # get all path points in order
    all_lines = [df.geometry.values[i] for i in range(df.shape[0])]
    multi_line = MultiLineString(all_lines)
    merged_line = ops.linemerge(multi_line)
    """ TO DO: check if line is reversed"""
    reverse = True
    if reverse:
        merged_line = LineString(ops.linemerge(multi_line).coords[::-1])
    coords = [Point(merged_line.coords[i][:2]) for i in range(len(merged_line.coords))]
    points_df = gpd.GeoDataFrame(coords, columns=['geometry'])
    points_df['points'] = points_df.geometry.apply(lambda x: x.coords[:2][0])
    del df
    path_to_save = save_folder + '/points.csv'
    points_df.points.to_csv(path_to_save)
