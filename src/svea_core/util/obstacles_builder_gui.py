#!/usr/bin/env python3

import pickle

import numpy as np
from PIL import Image, ImageTk
import tkinter as tk
from tkinter.filedialog import asksaveasfile, askopenfilename


class ObstaclesBuilderGUI(tk.Frame):
    """Obstacles Builder GUI, which allows the user to load a map and
    add obstacles defined by their edges. Finally, it allows the user to
    export the map in the YAML format.

    :param master: master widget (root), defaults to tk.Tk()
    :type master: tkinter.Tk, optional
    """
    CANVAS_HEIGHT = 600
    CANVAS_WIDTH = 800
    CLOSE_ENOUGH_POINTS_TOLERANCE = 10
    EXPORT_BUTTON_LABEL = 'Export'
    LINE_OPTIONS = {
        'fill': 'red',
        'dash': (4, 4),
        'width': 5
    }
    LOAD_BUTTON_LABEL = 'Load map'
    OBSTACLE_CREATION_ACTIVE_LABEL = 'Finish obstacle'
    OBSTACLE_CREATION_INACTIVE_LABEL = 'Add new obstacle'

    def __init__(self, master=tk.Tk()):
        super().__init__(master)
        self.pack()
        self.create_widgets()
        ## List of obstacles, each one defined by its edges
        self.obstacles = []
        ## New obstacle, list of edges
        self.new_obstacle = []
        ## Tuple to store the (x, y) coordinates of the previously selected point
        self.previous_coordinates = None
        ## Flag to indicate if a new obstacle can be created
        self.obstacle_creation_mode = False
        ## X coordinate of the origin of the occupancy grid (loaded with the map)
        self.origin_x = 0
        ## Y coordinate of the origin of the occupancy grid (loaded with the map)
        self.origin_y = 0
        ## Resolution of the occupancy grid (loaded with the map)
        self.resolution = 1

    def create_widgets(self):
        """Creates widgets which will be used in the GUI
        """
        self.pack(fill=tk.BOTH, expand=True)
        self.top_frame = tk.Frame(self)
        self.top_frame.pack(fill=tk.X, expand=False)

        # Create obstacle button
        self.create_obstacle_button = tk.Button(
            self.top_frame,
            text=self.OBSTACLE_CREATION_INACTIVE_LABEL,
            command=self._toggle_creation_mode_cb
        )
        self.create_obstacle_button.pack(side=tk.LEFT)

        # Load button
        self.load_button = tk.Button(
            self.top_frame,
            text=self.LOAD_BUTTON_LABEL,
            command=self._load_button_cb
        )
        self.load_button.pack(side=tk.LEFT)

        # Export button
        export_button = tk.Button(
            self.top_frame,
            text=self.EXPORT_BUTTON_LABEL,
            command=self._export_button_cb
        )
        export_button.pack(side=tk.RIGHT)

        # Main canvas
        self.canvas = tk.Canvas(self, background='white')
        self.canvas.config(width=self.CANVAS_WIDTH, height=self.CANVAS_HEIGHT)
        self.canvas.bind('<ButtonRelease-1>', self._draw_line)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.focus_set()

    def _toggle_creation_mode_cb(self):
        """Enables or disables the creation mode. Additionally, if the creation
        mode is disabled, the new obstacle is added to the list of obstacles
        and the helper variables are reset
        """
        if not self.obstacle_creation_mode:
            self.create_obstacle_button.config(text=self.OBSTACLE_CREATION_ACTIVE_LABEL)
        else:
            self.create_obstacle_button.config(text=self.OBSTACLE_CREATION_INACTIVE_LABEL)
            self.obstacles.append(
                self._convert_obstacle_to_map_coordinates(self.new_obstacle)
            )
            self.new_obstacle = []
            self.previous_coordinates = None

        self.obstacle_creation_mode = not self.obstacle_creation_mode

    def _export_button_cb(self):
        """Exports the obstacles to a YAML file
        """
        filename = asksaveasfile(
            mode='w',
            filetypes=(('YAML files', '*.yaml'), ('All files', '*.*'))
        )

        if not filename:
            return

        with open(filename.name, 'w') as f:
            f.write('obstacles:\n')
            for obstacle in self.obstacles:
                f.write(f'    - {str(obstacle)}')
                f.write('\n')

    def _load_button_cb(self):
        """Loads a map from an Occupancy Grid object (pickle)
        and uses it as background for the GUI
        """
        filename = askopenfilename(
            defaultextension='pickle',
            filetypes=(("Pickle files", "*.pickle"), ('All files', '*.*'))
        )

        with open(filename, 'rb') as f:
            occupancy_grid = pickle.load(f)

        self.resolution = occupancy_grid.info.resolution
        self.origin_x = occupancy_grid.info.origin.position.x
        self.origin_y = occupancy_grid.info.origin.position.y

        grid_data = np.array(occupancy_grid.data)
        grid_data = np.reshape(grid_data, (occupancy_grid.info.height, -1))
        # Don't flip the map so it's easier to handle the transformations
        #grid_data = np.flip(grid_data, 0)

        map_image = Image.fromarray(np.uint8(grid_data))
        tk_map_image = ImageTk.PhotoImage(map_image)

        self.canvas.create_image(0, 0, image=tk_map_image, anchor='nw')

        label = tk.Label(image=tk_map_image)
        label.image = tk_map_image

    def _draw_line(self, event):
        """Draws a line between the coordinates of the clicked point and the
        coordinates of the previously clicked point in the canvas

        :param event: Event generated from a mouse-released action
        :type event: tkinter.Event
        """
        if not self.obstacle_creation_mode:
            return

        if self.previous_coordinates is None:
            self.previous_coordinates = event.x, event.y
            self.new_obstacle.append([event.x, event.y])
            return

        x1, y1 = event.x, event.y

        if self._is_closing_shape(x1, y1, self.new_obstacle):
            x1, y1 = self.new_obstacle[0]
        else:
            self.new_obstacle.append([x1, y1])

        x0, y0 = self.previous_coordinates
        self.canvas.create_line(x0, y0, x1, y1, **self.LINE_OPTIONS)
        self.previous_coordinates = x1, y1

    def _is_closing_shape(self, x, y, new_obstacle):
        """Determines if the clicked point (x, y) is close enough to the first
        point in the new obstacle, given a tolerance in pixels for x and y.

        :param x: x coordinate of the clicked point
        :type x: int
        :param y: y coordinate of the clicked point
        :type y: int
        :param new_obstacle: List of edges for the new obstacle
        :type new_obstacle: list
        :return: `True` if both x and y are close enough, `False` otherwise
        :rtype: bool
        """
        first_obstacle_edge = new_obstacle[0]

        close_x = abs(first_obstacle_edge[0] - x) < self.CLOSE_ENOUGH_POINTS_TOLERANCE
        close_y = abs(first_obstacle_edge[1] - y) < self.CLOSE_ENOUGH_POINTS_TOLERANCE

        return close_x and close_y

    def _convert_obstacle_to_map_coordinates(self, obstacle_edges):
        """Converts and obstacle from the canvas coordinates (clicked point)
        to the actual coordinates defined by the map (occupancy grid)

        :param obstacle_edges: list of obstacle edges
        :type obstacle_edges: list
        :return: list of obstacle edges in new coordinates
        :rtype: list
        """
        new_obstacle_edges = []

        for edge in obstacle_edges:
            new_obstacle_edges.append([
                edge[0] * self.resolution + self.origin_x,
                edge[1] * self.resolution + self.origin_y
            ])

        return new_obstacle_edges


if __name__ == '__main__':
    app = ObstaclesBuilderGUI()
    app.mainloop()
