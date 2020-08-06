# Function that maps abstr_to_pixels and pixels_to_abstr
# Function mapping T&E abstraction of physical objects in the AVP to pixel map:
# Ex: Input: state = 1, object = "car" or "pedestrian" or "obstacle", Output: pixel_square = [[x_low, x_up], [y_low, y_up]], and midpoint = ((x_low+x_up)/2, (y_low+y_up)/2)
# For static_obstacles, if object is not present on the grid, state = 0, else state = 1. If state == 0, the pixel square returned would be empty
def abstr_to_pixels(state, object_type):
    pass


# Function that returns the abstraction in T&E state for the object given the (x_pix, y_pix) location in terms of pixels (or every 10 pixels) 
# For static_obstacle, if pixel_loc is empty, then return state = 0
def pixel_to_abstr(pixel_loc, object_type):
    pass