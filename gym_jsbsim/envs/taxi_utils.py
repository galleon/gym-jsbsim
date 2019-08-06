import geopandas as gpd
import pycrs
from shapely.geometry import Point,Polygon
import matplotlib.pyplot as plt
import pyproj
import numpy as np
from math import atan2
from gym_jsbsim.catalogs.utils import reduce_reflex_angle_deg


def heading(row,src_col):
    point = row[src_col]
    a,b = point.xy[0][0],  point.xy[1][0]
    heading = np.rad2deg(atan2(a,b))
    if heading<0:
        heading = 360+ heading
    return heading


class taxi_path(object):
    """
    methods to use:
    1. self.update(ref_pts,aircraft_heading_deg)  -> list[[(x,y),distance,heading,(long,lat)],[.....]]
    2. self.shortest_dist  -> returns the shortest (normal) distance to target path.
    3. self.plot() -> to visualize the airport, current path, next target points and aircraft position.
    """

    def __init__(self, ambd_folder_path="../amdb", number_of_points_to_use='all'):
        """
        :param ambd_folder_path: path to folder with airport shapefiles
        :param number_of_points_to_use: number of next points to return
        """

        path_id_numbers = [1977, 1974, 1973, 2001, 2202, 2203, 2453, 2204, 2206, 2000, 1996,  1968,  1969,
                           1971, 2020, 2019, 2018,  1931, 1929, 1925, 1926, 2031, 2042,   2134, 2091,  2099,
                           2103, 2385, 2105, 2136, 1978,  1947, 1949]
        self.shapefile_dir =  ambd_folder_path # 'amdb' folder path
        self.id_str = ['%d'%x for x in path_id_numbers]
        centerline = 'AM_AsrnEdge.shp'
        self.n = number_of_points_to_use

        self.df = gpd.read_file(self.shapefile_dir + '/' + centerline)
        self.current_track = self.df[self.df['idnumber'].isin(self.id_str)][['idnumber', 'geometry']]
        self.current_track['centroid'] = self.current_track.centroid

        self.order = {}
        i = 1
        for id_n in self.id_str:
            self.order[id_n]=i
            i +=1

        self.current_track['order'] = self.current_track.idnumber.map(self.order)
        self.current_track.index = self.current_track.order

        # to update in reward function
        self.shortest_dist = None

    def plot(self):
        plt.figure(figsize=(10, 20))
        ax = plt.subplot(111)
        aeqd = pyproj.Proj(proj='aeqd', ellps='WGS84', datum='WGS84', lat_0=aircraft_loc[1], lon_0=aircraft_loc[0]).srs
        df_new = self.df.to_crs(crs=aeqd)
        ax = df_new.plot(ax=ax, color=(0, 1, 1), lw=1)
        ax = self.current_track_new.plot(ax=ax, color=(1, 0, 1), lw=2)
        ax.plot(0, 0, 's', color='r', lw=4)
        ax = self.current_track_new.centroid.plot(ax=ax)
        plt.show()

    def update_path(self, aircraft_loc,ac_heading):
        """
        :param ref_pts: aircraft (longitude,latitude)
        :param ac_heading:
        :return: list[[(x,y),distance,heading,(long,lat)],[.....]]
        """
        aeqd = pyproj.Proj(proj='aeqd', ellps='WGS84', datum='WGS84', lat_0=aircraft_loc[1], lon_0=aircraft_loc[0]).srs
        current_track_new = self.current_track.to_crs(crs=aeqd)
        current_track_new['centroid'] = current_track_new.centroid
        current_track_new['distance'] = current_track_new['geometry'].distance(Point(0, 0))
        current_track_new = current_track_new.sort_values(by=['distance'])

        ind_start = None
        for ind, row in enumerate(self.id_str):
            if row == current_track_new.idnumber.iloc[0]:
                ind_start = ind
                break
            else:
                pass
        new_path = self.id_str[ind_start:]

        # update to remaining path
        current_track_new = current_track_new[current_track_new.idnumber.isin(new_path)]
        current_track_new = current_track_new.sort_index()

        current_track_new['heading'] = current_track_new.apply(heading, src_col='centroid', axis=1)
        current_track_new['delta_heading'] = current_track_new.heading.apply(
            lambda x: reduce_reflex_angle_deg(x - ac_heading))

        if self.n == 'all':
            n = len(current_track_new.index)
        else:
            n = self.n

        output = [[(current_track_new.loc[i].centroid.xy[0][0], current_track_new.loc[i].centroid.xy[1][0]),
                   current_track_new.loc[i].distance, current_track_new.loc[i].delta_heading,
                   (self.current_track.loc[i].centroid.xy[0][0], self.current_track.loc[i].centroid.xy[1][0])]
                  for i in current_track_new.index[:n]]

        self.shortest_dist = output[0][1]

        if (output[0][2] > 45) or (output[0][2] < -45):
            output = output[1:n]

        self.current_track_new = current_track_new

        return output


