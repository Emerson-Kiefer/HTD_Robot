#!/usr/bin/env python3
import rospy
import rospkg
import pickle
import os


import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.colors as mcolors
import numpy as np
import cv2
import yaml

def get_map_paths():
    try:
        rospack = rospkg.RosPack()
        pack_path = rospack.get_path('cs603_particle_filter')
        MAP_PATH = os.path.join(pack_path, 'maps/map.pgm')
        ROOMS_PATH = os.path.join(pack_path, 'maps/map_rooms.pkl')
        MAP_INFO_PATH = os.path.join(pack_path, 'maps/map.yaml')
    except:
        MAP_PATH = 'src/cs603_particle_filter/maps/map.pgm'
        ROOMS_PATH = 'src/cs603_particle_filter/maps/map_rooms.pkl'
        MAP_INFO_PATH = 'src/cs603_particle_filter/maps/map.yaml'
    return MAP_PATH, ROOMS_PATH, MAP_INFO_PATH

MAP_PATH, ROOMS_PATH, MAP_INFO_PATH = get_map_paths()


class RectangleSelector:
    def __init__(self, ax, map_array):
        self.ax = ax
        self.map_array = map_array
        self.colors = list(mcolors.TABLEAU_COLORS.keys())  # List of available colors
        self.rooms = [{'room_num':0, 'rects':[]}]
        self.cur_rect = None
        self.ax.imshow(self.map_array, cmap='gray')
        self.ax.figure.canvas.mpl_connect('button_press_event', self.on_press)
        self.ax.figure.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.ax.figure.canvas.mpl_connect('button_release_event', self.on_release)
        self.start = None
        self.end = None
        self.cur_room = 0
        self.allow_draw = True

    def on_press(self, event):
        if event.inaxes != self.ax or not self.allow_draw:
            return
        
        self.start = (event.xdata, event.ydata)
        self.cur_rect = Rectangle((self.start[0], self.start[1]), 0, 0, alpha=0.4,
                                  color=self.colors[self.cur_room], visible=True)
        self.ax.add_patch(self.cur_rect)

    def on_motion(self, event):
        if event.inaxes != self.ax or self.start is None:
            return
        x1, y1 = self.start
        x2, y2 = event.xdata, event.ydata
        width = abs(x2 - x1)
        height = abs(y2 - y1)
        self.cur_rect.set_width(width)
        self.cur_rect.set_height(height)
        self.cur_rect.set_xy((min(x1, x2), min(y1, y2)))
        plt.draw()


    def reset_rect(self):
        self.start = None
        self.end = None
        self.cur_rect = None

    def on_release(self, event):
        if event.inaxes != self.ax or self.start is None:
            return
        
        self.end = (event.xdata, event.ydata)
        dist0 = int(abs(self.start[0] - self.end[0]))
        dist1 = int(abs(self.start[1] - self.end[1]))

        if dist0 + dist1 < 10:
            self.reset_rect()
            self.redraw_rectangles()
            return 
        
        rect = {
            'x': int(min(self.start[0], self.end[0])),
            'y': int(min(self.start[1], self.end[1])),
            'D_x': dist0,
            'D_y': dist1,
            'color': self.colors[self.cur_room],
        }

        self.rooms[self.cur_room]['rects'].append(rect)

        self.reset_rect()
        plt.draw()
        self.print_rectangles()

    def print_rectangles(self):
        print()
        for room in self.rooms:
            print(f"Room Number: {room['room_num']}")
            for idx, rect in enumerate(room['rects']):
                print(f"  Rectangle {idx+1}: {rect}")

    def request_new_room(self):
        self.cur_room += 1
        self.rooms.append({'room_num':self.cur_room, 'rects':[]})

    def undo_last_rectangle(self):
        if self.rooms[self.cur_room]['rects']:
            self.rooms[self.cur_room]['rects'].pop()
            self.redraw_rectangles()

    def write_and_exit(self):
        print(self.rooms)
        with open(ROOMS_PATH, 'wb') as file:
            pickle.dump(self.rooms, file)
        exit()
    
    def exit_unsaved(self):
        exit()
    
    def toggle_draw(self):
        self.allow_draw = not self.allow_draw

    def redraw_rectangles(self):
        self.ax.clear()
        self.ax.imshow(self.map_array, cmap='gray')
        for room in self.rooms:
            for rect in room['rects']:
                self.ax.add_patch(Rectangle((rect['x'], rect['y']), rect['D_x'], rect['D_y'],
                                             alpha=0.4, color=rect['color'], visible=True))
        plt.draw()

def set_room_bounds():
    global selector  # Make selector global
    map_array = get_map_array()

    fig, ax = plt.subplots()
    selector = RectangleSelector(ax, map_array)
    plt.connect('key_press_event', selector_key_press)
    plt.show()

def selector_key_press(event):
    if event.key == 'n' and hasattr(selector, 'request_new_room'):  # Check if selector has request_new_room method
        selector.request_new_room()
    elif event.key == 'u' and hasattr(selector, 'undo_last_rectangle'):  # Check if selector has undo_last_rectangle method
        selector.undo_last_rectangle()
    elif event.key == 'w' and hasattr(selector, 'write_and_exit'):
        selector.write_and_exit()
    elif event.key == 'q' and hasattr(selector, 'exit_unsaved'):
        selector.exit_unsaved()
    elif event.key == 'd' and hasattr(selector, 'toggle_draw'):
        selector.toggle_draw()

def get_map_array():
    map_image = plt.imread(MAP_PATH)
    return np.array(map_image)

def get_distance_matrix():
    #distance matrix with distances as # of pixels
    map_image = plt.imread(MAP_PATH)
    return cv2.distanceTransform(map_image, distanceType=cv2.DIST_L2, maskSize=5)

def get_map_info():
    with open(MAP_INFO_PATH, 'r') as file:
        map_data = yaml.safe_load(file)
    map_resolution = map_data['resolution']
    map_origin = map_data['origin']
    return map_resolution, map_origin

def get_all_map():
    map_arr = get_map_array()
    dist_mat = get_distance_matrix()
    map_res, map_origin = get_map_info()
    return map_arr, dist_mat, map_res, map_origin

# def get_map_index_from_coords(x, y, m):
#     D1, _ = m['map'].shape
#     x_index = int((x - m['origin'][0])/m['res'])
#     y_index = D1 - int((y - m['origin'][1])/m['res']) - 1
#     return x_index, y_index

def get_map_coords_from_index(x, y, map_arr, map_res, map_origin):
    Dy, Dx = map_arr.shape
    x_coord = x * map_res + map_origin[0]
    y_coord = (Dy - y) * map_res + map_origin[1]
    print("X, Y", Dy, x_coord, y_coord)
    return x_coord, y_coord

# if __name__ =='main':
# set_room_bounds()