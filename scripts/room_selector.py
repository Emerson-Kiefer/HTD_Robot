#!/usr/bin/env python3
import rospy
import rospkg
import pickle
import os
from mapping_operations import get_all_map, get_map_paths


import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.colors as mcolors

MAP_PATH, ROOMS_PATH, MAP_INFO_PATH = get_map_paths()
SELECTED_ROOM = None

def select_room():

    map_array, _, _, _ = get_all_map()

    with open(ROOMS_PATH, 'rb') as file:
        rooms = pickle.load(file)
    
    fig, ax = plt.subplots()
    ax.imshow(map_array, cmap='gray')
    draw_rectangles(rooms, ax)

    fig.canvas.mpl_connect('button_press_event', lambda event: on_click(event, rooms, ax))
    plt.show()

    return SELECTED_ROOM


def draw_rectangles(rooms, ax):
    for room in rooms:
        for rect in room['rects']:
            ax.add_patch(Rectangle((rect['x'], rect['y']), rect['D_x'], rect['D_y'],
                                            alpha=0.4, color=rect['color'], visible=True))

def on_click(event, rooms, ax):
    global SELECTED_ROOM
    if event.inaxes != ax:
        return
    x, y = int(event.xdata), int(event.ydata)

    if x is None or y is None:
        return
    for room in rooms:
        for rect in room['rects']:
            if rect['x'] <= x and rect['y'] <= y and rect['x'] + rect['D_x'] >= x and rect['y'] + rect['D_y'] >= y:
                print("Selected room", room['room_num'])
                SELECTED_ROOM = room['room_num']
                plt.close('all')
                return
    print('Clicked outside of all rooms')


# print("ROOM_SELECTED", select_room())