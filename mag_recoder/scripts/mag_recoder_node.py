#!/usr/bin/env python  
# ROS
import roslib
roslib.load_manifest('mag_recoder')
import rospy
import tf
from sensor_msgs.msg import MagneticField
from std_srvs.srv import *
# Utils
import math
import numpy as np
import cv2
# Read Yaml File
import yaml
import re
# Plotting Map
import matplotlib as mpl
import matplotlib.cm as mtpltcm
import matplotlib.pyplot as plt
import time

global mag_data
global mag_map
global mag_map_x
global mag_map_y
global mag_map_z
# mag_map = np.zeros((1, 1, 3))

def read_pgm(filename, byteorder='>'):
    """
        Return image data from a raw PGM file as numpy array.
    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))


def calc_p2p_dist(x1, y1, x2, y2):
    return math.hypot((x1-x2),(y1-y2))
    
def mag_CB(data):
    global mag_data
    mag_data = [0.0, 0.0, 0.0]
    mag_data[0] = data.magnetic_field.x
    mag_data[1] = data.magnetic_field.y
    mag_data[2] = data.magnetic_field.z
    # print('x: '+mag_data[0]+', y:'+mag_data[1]+', z:'+mag_data[2])
    
def handle_save_img(req):
    cv2.imwrite('mag_map.png',mag_map)
    write_data2csv('x',mag_map_x)
    write_data2csv('y',mag_map_y)
    write_data2csv('z',mag_map_z)
    return SetBoolResponse(success=True, message='Success!')

def write_data2csv(axis_name ,data):
    fileName = 'mag_data_' + axis_name + '.csv'
    np.savetxt(fileName, data, delimiter=",")

if __name__ == '__main__':
    map_origin = [0.0, 0.0, 0.0]
    map_resolution = 0.0;
    
    map_name = 'map_r1'
    # Read YAML
    stream = open(map_name+'.yaml', "r")
    docs = yaml.load_all(stream)
    for doc in docs:
        for k,v in doc.items():
            if k=='origin':
                map_origin = v
            if k=='resolution':
                map_resolution = v;
    print(map_origin)
    # Read Map
    image = read_pgm(map_name+".pgm", byteorder='<')
    
    # Generate Mag Map
    mag_map_resolution = 0.05
    height = int(image.shape[0]*map_resolution/mag_map_resolution)
    width = int(image.shape[1]*map_resolution/mag_map_resolution)
    mag_map = np.zeros((height, width, 3))
    mag_map_x = np.zeros((height,width,1))
    mag_map_y = np.zeros((height,width,1))
    mag_map_z = np.zeros((height,width,1))
    print("(Generate Magnetic Map... size: %(x)s* %(y)s" % {'x': width, 'y': height})
    #initialize the colormap (jet)
    colormap = mpl.cm.jet
    #add a normalization
    cNorm = mpl.colors.Normalize(vmin=0, vmax=100)
    #init the mapping
    scalarMap = mtpltcm.ScalarMappable(norm=cNorm, cmap=colormap)
    # Draw Map
    im = plt.imshow(mag_map)
    plt.colorbar()

    # Param
    pre_location = [0.0, 0.0];

    # Start ROS Node
    rospy.init_node('mag_recoder')
    listener = tf.TransformListener()
    
    rospy.Subscriber("m_mag",MagneticField, mag_CB)
    rospy.Service('save_mag_data', SetBool, handle_save_img)
    
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/gyro_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # print(trans)        
        if(calc_p2p_dist(pre_location[0],pre_location[1],trans[0],trans[1])%mag_map_resolution < mag_map_resolution):
            y = int(trans[0]/mag_map_resolution - map_origin[0]/mag_map_resolution)
            x = int(trans[1]/mag_map_resolution - map_origin[1]/mag_map_resolution)
            
            if x>=height or y>= width or x<0 or y<0:
                continue
            global mag_data
            mag_map[x][y][0] = math.sqrt(pow(mag_data[0],2) + pow(mag_data[1],2) + pow(mag_data[2],2))*100
            mag_map[x][y][1] = mag_map[x][y][0]
            mag_map[x][y][2] = mag_map[x][y][0]
            
            mag_map_x[x][y] = mag_data[0]
            mag_map_y[x][y] = mag_data[1]
            mag_map_z[x][y] = mag_data[2]

            print("(X: %(x)s, Y: %(y)s) = %(value)s" % {'x': x, 'y': y, 'value': mag_map[x][y]})
        
        show_mag_map = cv2.flip(mag_map, 0)
        gray = cv2.cvtColor(show_mag_map.astype(np.uint8), cv2.COLOR_BGR2GRAY)
        colors = scalarMap.to_rgba(gray)
            
        im.set_data(colors)
        plt.pause(0.1)


