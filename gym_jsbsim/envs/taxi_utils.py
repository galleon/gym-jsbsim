import geopandas as pd
import numpy as np
import shapefile
#from src.taxi.naviguation import *
from shapely.geometry import Point, LineString, Polygon
#from taxi.geo import *
#from src import taxi
#from bin import *
import math
import pandas as pd
import time


D2R = np.deg2rad(1)
def fromPolarToCart(x0, h, Lat, Long):
    '''
        x0  = (lat, long) of reference point [ rad ]
        h   = height of reference points [ m ]
        Lat = [lat0; lat1; ...] lattitudes of points [rad]
        Long = [long0; long1; ...] longitudes of points [rad]
    '''
    X = np.array([[Lat[i] * D2R, Long[i] * D2R, h]
                  for i in range(len(Lat))])
    X = polar2cart(X)
    return ecef_c2ned(np.array((x0[0] *D2R, x0[1] * D2R, h)), X)

def ecef_c2ned(x0,x=None,ref='spheroid'):
    """ Projection of coordinates from ECEF frame to NED frame defined at the
        point x0
        Implementation of Eq (1.6-22) of "Johnson, Lewis, Stevens (2015) Aircraft
        control and simulation, Wiley-Blackwell, 3rd edition"
        If no points are given, the function returns the rotation matrix

    Input
    ----------

    x0 : np.array
        Vector of origin point x0 = [latitude, longitude, h] in [rad,rad,m]

    x : np.array
        [N x 3] matrix of the N points to move from ECEF to NED, given in
        cartesian coordinates in [m, m, m]

    Output
    ----------

    y : np.array
        [N x 3] matrix of the N points in the NED frame in [m, m, m]

    or

    mat : np.array
        [3 x 3] matrix of rotation from ECEF_C to NED in x0

    """
    lat     = x0[0].copy()
    lon     = x0[1].copy()
    x0_cart = polar2cart(x0.reshape((1,3)))
    #h0 = x0_cart[0,2]
    sphi    = np.sin(lat)
    cphi    = np.cos(lat)
    slon    = np.sin(lon)
    clon    = np.cos(lon)
    mat     = np.array([[-sphi*clon,-sphi*slon,cphi],
                        [-slon,clon,0],
                        [-cphi*clon,-cphi*slon,-sphi]])
    if x is None:
        return mat;
    else:
        if x.size == 3:
            x = x.reshape((1,3))
        y = x.copy()
        for i in range(len(y[:,0])):
            vec = x[[i],:].reshape((3,1)) - x0_cart.T
            #vec[2] = vec[2] - h0
            y[[i],:] = np.dot(mat,vec).reshape((1,3))
        return y;


def polar2cart(Xin):
    """ Conversion of coordinates from polar to cartesian in ECEF frame
        Implementation of Eq (1.6-17) of "Johnson, Lewis, Stevens (2015) Aircraft
        control and simulation, Wiley-Blackwell, 3rd edition"

    Input
    ----------

    Xin : np.array
         [N x 3] matrix of the N points to convert, given in polar ccordinates
         such as [latitude, longitude, h] in [rad,rad,m]
         with h the height above the spheroid, along the normal.


    Output
    ----------

    Xout : np.array
          [N x 3] matrix of the N points in cartesian coordiantes in [m, m, m]

    """
    a = 6378.1370*1e3   # in m
    b = 6356.7523*1e3   # in m
    e = np.sqrt(1 - (b/a)**2)
    dh = 0.0

    if Xin.size == 3:
        Xin = Xin.reshape((1,3))
        lat = Xin[0,0].copy()
        lon = Xin[0,1].copy()
        h   = Xin[0,2] + dh
    else:
        lat = Xin[:,0].copy()
        lon = Xin[:,1].copy()
        h   = Xin[:,2].copy()

    N = a/np.sqrt(1 - (e*np.sin(lat))**2)

    x = (N + h)*np.cos(lat)*np.cos(lon)
    y  = (N + h)*np.cos(lat)*np.sin(lon)
    z = (N*(1-e**2) + h)*np.sin(lat)

    if Xin.size == 3:
        Xout = np.array([x,y,z]).reshape((1,3))
    else:
        Xout = np.array([x,y,z]).transpose();
    return Xout;

def plot_line_issimple(ax, ob, **kwargs):
    kwargs["color"] = color_issimple(ob)
    plot_line(ax, ob, **kwargs)

def plot_line(ax, ob, color='#6699cc', zorder=1, linewidth=3, alpha=1):
    x, y = ob.xy
    ax.plot(x, y, color=color, linewidth=linewidth, solid_capstyle='round', zorder=zorder, alpha=alpha)

#-----------------------------------------------------------------------------------------------------------------------
#           Taxi Path Class 
# FIXME: Need to be optimise for computational issue
# TODO: Add a cart2geo(self, refPoint, Point) >> (lat, lon)
#-----------------------------------------------------------------------------------------------------------------------


class taxi_path(object):

    def __init__(self, ambd_folder_path="/home/jyotsna/src/attol_taxi_ctrl/amdb", ref_pts=None):
        self.fname_ref = 'AM_AerodromeReferencePoint.shp'  #Airport reference point
        self.shapefile_dir =  ambd_folder_path # 'amdb' folder path
        self.default_h = 148.72 # Altitude of toulouse airport
        self.ref = shapefile.Reader(shapefile_dir + '/' + self.fname_ref)
        self.ref_pts = np.array(self.ref.shapeRecords()[0].shape.points)[0][::-1]   # reference (x,y) in (Lat, long)  Example TLS: array([43.635     ,  1.36777778])
        path_id_numbers = [1977, 1974, 1973, 2001, 2202, 2203, 2453, 2204, 2206, 2000, 1996,  1968,  1969, 1971, 2020, 2019, 2018,  1931, 1929, 1925, 1926, 2031, 2042,   2134, 2091,  2099, 2103, 2385, 2105, 2136, 1978,  1947, 1949]
        self.path_id_numbers = [str(id) for id in path_id_numbers]
        self.number_of_points_to_use = 8    # number of points to use on the path for learning
        reader = shapefile.Reader(shapefile_dir+'/AM_AsrnEdge.shp', encodingErrors="replace")
        self.edges = [edge for edge in reader.shapeRecords()]
        self.edges_longlat = [(shp.shape.points[:], shp.record[2]) for idn in self.path_id_numbers for shp in self.edges if shp.record[2] == idn]
        # self.edges_longlat[0] = ([(long1,lat1),(long2,lat2)],'idnumber')
        print('selected path: ', self.path_id_numbers)

    def plot_path(self):
        print('TBD')

    def loadLinefile(self, ref_pts, height):

        edges_cartesian = []
        for edge in self.edges_longlat:
            edges_cartesian.append((np.array([fromPolarToCart(ref_pts, height, [i[1]], [i[0]])[0] for i in edge[0]]), edge[1]))
        # edges_cartesian[0] = ([[x1,y1],[x2,y2]],'idnumber')
        return edges_cartesian


    def update_path(self, ref_pts):
        self.edges_cartesian = self.loadLinefile(ref_pts, self.default_h)
        df = []
        points = []
        i = 0
        for line in self.edges:
            x, y = line[0].xy
            for a, b in zip(x, y):
                if (a, b) in points:
                    # print('Copy')
                    pass

                else:
                    points.append((a, b))
                    if b > 0:  # only points with +y with respect to ref
                        distance = Point((0, 0)).distance(Point(a, b))
                        heading = math.degrees(math.atan2(b, a))  # considering the ref point will always be (0,0)
                        df.append([Point(a, b), distance, heading])

        df.sort(key=lambda x: x[1])  # sorted by distance from ref
        return df[:self.number_of_points_to_use]
