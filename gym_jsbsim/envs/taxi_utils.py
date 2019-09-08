import numpy as np
from shapely import ops
from shapely.geometry import MultiPoint,Point,MultiLineString,LineString
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, atan2, radians,pi, acos
from geographiclib.geodesic import Geodesic
import geopandas as gpd


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
    1. self.update(aircraft_loc)  -> list[[(long,lat),distance,heading],[.....]]
    2. self.shortest_dist  -> returns the shortest (normal) distance to target path.
    3. self.plot() -> to visualize the airport, current path, next target points and aircraft position.
    """

    def __init__(self, ambd_folder_path="../amdb", number_of_points_to_use='all'):
        """
        :param ambd_folder_path: path to folder with airport shapefiles
        :param number_of_points_to_use: number of next points to return
        """

        path_id_numbers = [1977, 1974, 1973, 2001, 2202, 2203, 2453, 2204, 2206, 2000, 1996, 1968, 1969,
                           1971, 2020, 2019, 2018, 1931, 1929, 1925, 1926, 2031, 2042, 2134, 2091, 2099,
                           2103, 2385, 2105, 2136, 1978, 1947, 1949]
        self.shapefile_dir = ambd_folder_path  # 'amdb' folder path
        id_str = ['%d' % x for x in path_id_numbers]
        centerline = 'AM_AsrnEdge.shp'
        self.n = number_of_points_to_use

        df = gpd.read_file(self.shapefile_dir + '/' + centerline)
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
        self.points_df = gpd.GeoDataFrame(coords, columns=['geometry'])
        self.points_df['points'] = self.points_df.geometry.apply(lambda x: x.coords[:2][0])
        del df
        # add relative properties of points
        relative_distance = [0]
        relative_distance[1:] = [
            float("{:.4f}".format(get_distance(self.points_df.points.iloc[i], self.points_df.points.iloc[i - 1]))) for i
            in range(1, self.points_df.shape[0])]
        relative_bearing = [0]
        relative_bearing[1:] = [
            float("{:.4f}".format(get_bearing(self.points_df.points.iloc[i - 1], self.points_df.points.iloc[i]))) for i
            in range(1, self.points_df.shape[0])]

        self.points_df['rel_distance'] = relative_distance
        self.points_df['rel_bearing'] = relative_bearing

        # to update in reward function
        self.shortest_dist = None

    def plot(self):
        plt.figure(figsize=(10, 20))
        ax = plt.subplot(111)
        ax = self.points_df.plot(ax=ax, color=(0, 1, 1), lw=1)
        ax.plot(self.aircraft_loc[0], self.aircraft_loc[1], 's', color='r', lw=4)
        plt.show()

    def update_path(self, aircraft_loc):
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
        target_heading = float("{:.4f}".format(get_bearing(aircraft_loc, self.points_df.points.loc[idx])))

        output = [[nearest_point.coords[:2][0], shortest_dist, target_heading]]
        output2 = [[self.points_df.points.loc[i], self.points_df.rel_distance.loc[i], self.points_df.rel_bearing.loc[i]]
                   for i in range(idx + 1, idx + n)]
        output += output2

        return output



