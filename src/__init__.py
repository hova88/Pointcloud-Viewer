import numpy as np
from .open3d_coordinate import create_coordinate
from .open3d_arrow import create_arrow
from .open3d_box import create_box


def create_box_with_arrow(box, color=None):
    """
    box: list(8) [ x, y, z, dx, dy, dz, yaw]
    """

    box_o3d = create_box(box, color)
    x = box[0]
    y = box[1]
    z = box[2]
    l = box[3]
    yaw = box[6]
    # get direction arrow
    dir_x = l / 2.0 * np.cos(yaw)
    dir_y = l / 2.0 * np.sin(yaw)

    arrow_origin = [x - dir_x, y - dir_y, z]
    arrow_end = [x + dir_x, y + dir_y, z]
    arrow = create_arrow(arrow_origin, arrow_end, color)

    return box_o3d, arrow
