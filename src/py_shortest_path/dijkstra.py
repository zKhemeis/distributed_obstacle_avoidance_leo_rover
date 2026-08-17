import numpy as np
import time
from scipy.sparse.csgraph import shortest_path
from scipy.sparse import dok_array, coo_array
from enum import Enum
import matplotlib.pyplot as plt


def pos2idx(x: int, y:int, width: int, height: int) -> int:
    return x + y*width if 0 <= x < width and 0 <= y < height else -1

def idx2pos(idx: int, width: int, height: int) -> (int, int):
    return (idx%width, int(idx/width)) if 0 <= idx < width*height else (-1, -1)

def make_graph_coo(width: int, height: int):
    """
    Makes a (coo) graph of a 2-D lattice
    Includes diagonal connections

    :param width: How wide the lattice is
    :param height: How tall the lattice is
    :return: The graph in coo format
    """
    if width <= 0 or height <= 0:
        return None

    row_idxs = []
    col_idxs = []
    data = []
    size = width * height

    horizontal = [1]*width
    diagonal = [np.sqrt(2)]*(width-1)
    for y in range(height):
        tmp_row = []
        tmp_col = []
        tmp_data = []
        row = list(range(y*width, (y+1)*width))
        right = range(y*width+1, (y+1)*width+1)
        down = range((y-1)*width, y*width)
        downRight = range((y-1)*width+1, y*width+1)
        upRight = range((y+1)*width+1, (y+2)*width+1)

        # Add right, all but rightmost
        tmp_row += row[:-1]
        tmp_col += list(right[:-1])
        tmp_data += horizontal[:-1]
        # Add down, if not first row
        if y != 0:
            tmp_row += row
            tmp_col += list(down)
            tmp_data += horizontal
        # Add downRight, if not first row, all but rightmost
        if y != 0:
            tmp_row += row[:-1]
            tmp_col += list(downRight[:-1])
            tmp_data += diagonal
        # Add upRight, if not last row, all but rightmost
        if y < height-1:
            tmp_row += row[:-1]
            tmp_col += list(upRight[:-1])
            tmp_data += diagonal

        row_idxs += tmp_row
        col_idxs += tmp_col
        data += tmp_data

    return coo_array((data, (row_idxs, col_idxs)), shape=(size, size))

def make_circle_bounded(center_x: int, center_y: int, robot_radius: int, circle_radius: int, width: int, height: int):
    """
    Makes a (coo) graph of a circle at the specified position
    Actual circle is extended by the size of the robot

    :param center_x: x-coordinate of circle center
    :param center_y: y-coordinate of circle center
    :param robot_radius: Size of the robot, assumed to be circular
    :param circle_radius: Radius of the main circle
    :param width: Width of the simulated area
    :param height: Height of the simulated area
    :return: coo graph of size width*height, weights added along the sum of circle and robot (it's still a circle)
    """
    combined_radius = robot_radius + circle_radius
    return make_circle(center_x, center_y, combined_radius, width, height)

def make_circle(center_x: int, center_y: int, radius: int, width: int, height: int):
    if not (0 <= center_x <= width-1 and 0 <= center_y <= height-1):
        return None
    if radius <= 0:
        return None
    if width <= 0 or height <= 0:
        return None

    row_idxs = []
    col_idxs = []
    data = []
    size = width * height

    min_x = center_x - radius - 1
    max_x = center_x + radius + 1
    min_y = center_y - radius - 1
    max_y = center_y + radius + 1

    r2 = radius**2
    for y in range(min_y, max_y):
        for x in range(min_x, max_x):
            if not (0 <= x < width and 0 <= y < height):
                continue

            # Insideness computations
            botLeft = (center_x - x)**2 + (center_y - y)**2 < r2
            botRight = (center_x - x + 1)**2 + (center_y - y)**2 < r2
            topLeft = (center_x - x)**2 + (center_y - y + 1)**2 < r2
            topRight = (center_x - x + 1)**2 + (center_y - y + 1)**2 < r2

            if not ((botLeft and botRight and topLeft and topRight) or (not botLeft and not botRight and not topLeft and not topRight)):
                # If some corners are within the circle and some aren't, increase the weight of all six connections massively
                # Horizontal connections
                row_idxs += [x + y * width]
                col_idxs += [x + 1 + y * width]
                data += [2 ** 15]  # Some value larger than the maximum distance
                row_idxs += [x   + (y+1)*width]
                col_idxs += [x+1 + (y+1)*width]
                data += [2**15]

                # Vertical connections
                row_idxs += [x   +  y    * width]
                col_idxs += [x+1 + (y+1) * width]
                data += [2 ** 15]
                row_idxs += [x+1 + (y+1) * width]
                col_idxs += [x+1 +  y    * width]
                data += [2 ** 15]

                # Diagonal connections
                row_idxs += [x   +  y    * width]
                col_idxs += [x+1 + (y+1) * width]
                data += [2 ** 15]
                row_idxs += [x     + (y + 1) * width]
                col_idxs += [x + 1 + y       * width]
                data += [2 ** 15]

            """
            # Horizontals
            if botLeft != botRight:
                row_idxs += [x   + y*width]
                col_idxs += [x+1 + y*width]
                data += [2**15]  # Some value larger than the maximum distance
            if topLeft != topRight:
                row_idxs += [x   + (y+1)*width]
                col_idxs += [x+1 + (y+1)*width]
                data += [2**15]
            # Verticals
            if topLeft != botLeft:
                row_idxs += [x + (y+1)*width]
                col_idxs += [x +  y   *width]
                data += [2**15]
            if topRight != botRight:
                row_idxs += [x+1 + (y+1) * width]
                col_idxs += [x+1 +  y    * width]
                data += [2 ** 15]
            # Diagonals
            if botLeft != topRight:
                row_idxs += [x   +  y    * width]
                col_idxs += [x+1 + (y+1) * width]
                data += [2 ** 15]
            if topLeft != botRight:
                row_idxs += [x     + (y + 1) * width]
                col_idxs += [x + 1 + y       * width]
                data += [2 ** 15]
            """




    return coo_array((data, (row_idxs, col_idxs)), shape=(size, size))

def distance_point_line(point: (int, int), line_1: (int, int), line_2: (int, int)) -> (int, bool):
    """
    Checks how far the point is from the line and which side it lies on

    :param point: Point to check
    :param line_1: First point of the line to compare against
    :param line_2: Second point of the line to compare against
    :return: distance, isRightOfLine
    """

    xp, yp = point
    x1, y1 = line_1
    x2, y2 = line_2

    distance = ((y2 - y1)*xp - (x2 - x1)*yp + x2*y1 - y2*x1) / (np.sqrt((y2-y1)**2+(x2-x1)**2))  # 86s
    return distance, distance > 0

def rectangle_check(point: (int, int), vertices: [(int, int)], radius: int) -> int:
    """
    Checks if the point is considered close to or even within the rectangle
    Do make sure the shape is actually approximately rectangular as the algorithms assumes right angles

    :param point: Point to check
    :param vertices: Vertices of the 'rectangle'
    :param radius: Maximal distance to 'rectangle'
    :return: Whether the point is close to the 'rectangle'
    """
    x, y = point
    d0, right0 = distance_point_line((x, y), vertices[0], vertices[1])
    d1, right1 = distance_point_line((x, y), vertices[1], vertices[2])
    d2, right2 = distance_point_line((x, y), vertices[2], vertices[3])
    d3, right3 = distance_point_line((x, y), vertices[3], vertices[0])

    case = -1
    if right0:
        if right3:
            case = 0
        elif right1:
            case = 2
        else:
            case = 1
    elif right2:
        if right3:
            case = 6
        elif right1:
            case = 4
        else:
            case = 5
    elif right1:
        case = 3
    elif right3:
        case = 7
    else:
        case = 8

    # Imagine if Python had switch statements
    # Corner cases
    r2 = radius**2
    if case == 0:
        vertex = vertices[0]
        inside = (vertex[0] - x)**2 + (vertex[1] - y)**2 <= r2
    elif case == 2:
        vertex = vertices[1]
        inside = (vertex[0] - x)**2 + (vertex[1] - y)**2 <= r2
    elif case == 4:
        vertex = vertices[2]
        inside = (vertex[0] - x)**2 + (vertex[1] - y)**2 <= r2
    elif case == 6:
        vertex = vertices[3]
        inside = (vertex[0] - x)**2 + (vertex[1] - y)**2 <= r2
    # Edge cases
    elif case == 1:
        inside = d0 <= radius
    elif case == 3:
        inside = d1 <= radius
    elif case == 5:
        inside = d2 <= radius
    elif case == 7:
        inside = d3 <= radius
    # Literally inside already
    else:
        inside = True

    return inside


def make_rectangle_bounded(vertices: [(int, int)], radius: int, width: int, height: int):
    """
    Makes a (coo) graph at the specified position
    Includes round boundary of the robot

    :param vertices: Vertices of the 'rectangle' in counterclockwise order
    :param radius: Size of the robot, assumed to be circular
    :param width: Width of the simulated area
    :param height: Height of the simulated area
    :return: coo graph of size width*height, weights added along the sum of 'rectangle' and robot (rounded rectangle)
    """
    if len(vertices) != 4:
        return None

    row_idxs = []
    col_idxs = []
    data = []
    size = width * height

    min_x = min([p[0] for p in vertices]) - 1 - radius
    max_x = max([p[0] for p in vertices]) + 1 + radius
    min_y = min([p[1] for p in vertices]) - 1 - radius
    max_y = max([p[1] for p in vertices]) + 1 + radius

    if not (0 <= min_x and max_x < width and 0 <= min_y and max_y < height):
        return None

    for y in range(min_y, max_y):
        for x in range(min_x, max_x):
            if x < 0 or y < 0 or x >= width or y >= height:
                continue

            botLeft  = rectangle_check((x  , y  ), vertices, radius)
            botRight = rectangle_check((x+1, y  ), vertices, radius)
            topLeft  = rectangle_check((x  , y+1), vertices, radius)
            topRight = rectangle_check((x+1, y+1), vertices, radius)

            if not ((botLeft and botRight and topLeft and topRight) or (not botLeft and not botRight and not topLeft and not topRight)):
                # If some corners are within the circle and some aren't, increase the weight of all six connections massively
                # Horizontal connections
                row_idxs += [x + y * width]
                col_idxs += [x + 1 + y * width]
                data += [2 ** 15]  # Some value larger than the maximum distance
                row_idxs += [x + (y + 1) * width]
                col_idxs += [x + 1 + (y + 1) * width]
                data += [2 ** 15]

                # Vertical connections
                row_idxs += [x + y * width]
                col_idxs += [x + 1 + (y + 1) * width]
                data += [2 ** 15]
                row_idxs += [x + 1 + (y + 1) * width]
                col_idxs += [x + 1 + y * width]
                data += [2 ** 15]

                # Diagonal connections
                row_idxs += [x + y * width]
                col_idxs += [x + 1 + (y + 1) * width]
                data += [2 ** 15]
                row_idxs += [x + (y + 1) * width]
                col_idxs += [x + 1 + y * width]
                data += [2 ** 15]



    return coo_array((data, (row_idxs, col_idxs)), shape=(size, size))


def display_dist(data: np.ndarray, width: int, height: int):
    if data.shape[0] != 1 and data.shape[1] != width*height:
        print("Wrong shape")
        return

    fig, ax = plt.subplots(figsize=(20, 30))

    image = data.reshape((height, width))
    ax.imshow(image, vmin=0, vmax=2*np.sqrt(height**2+width**2))
    plt.show()


if __name__ == "__main__":
    width = 2000  # mm
    height = 3000  # mm

    before_graph = time.time()
    array = make_graph_coo(width, height)
    after_graph = time.time()
    print(f"Graph: {after_graph - before_graph}, {array.size}")

    before_circle = time.time()
    circle = make_circle(1000, 1500, 300, width, height)
    after_circle = time.time()
    print(f"Circle: {after_circle - before_circle}, {circle.size}")

    before_rectangle = time.time()
    #rectangle = make_rectangle_bounded([(1000, 1500), (1000, 1000), (1500, 1000), (1500, 1500)], 200, width, height)
    #rectangle = make_rectangle_bounded([(1000, 1500), (800, 1100), (1000, 700), (1200, 1100)], 200, width, height)
    rectangle = make_rectangle_bounded([(1000, 1500), (700, 1000), (1000, 700), (1300, 1000)], 200, width, height)
    after_rectangle = time.time()
    print(f"Rectangle: {after_rectangle - before_rectangle}, {"rectangle.size"}")

    before_add = time.time()
    array += rectangle
    #array += circle
    after_add = time.time()
    print(f"Addition: {after_add - before_add}, {array.size}")

    before_dist = time.time()
    dist = shortest_path(csgraph=array, method='D', directed=False, indices=[pos2idx(width-1, height-1, width, height)])
    after_dist = time.time()
    print(f"Dist: {after_dist - before_dist}")
    # print(np.round(dist.reshape((height, width)), 2))

    display_dist(dist, width, height)
