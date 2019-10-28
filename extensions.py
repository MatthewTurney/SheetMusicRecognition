import cv2 as cv
import numpy as np

def height(img):
    return img.shape[0]

def width(img):
    return img.shape[1]

def draw_staff_lines(img_no_staff, staff_bases, staff_gap, staff_line_thickness):
    img_colored_staff_base = cv.cvtColor(img_no_staff, cv.COLOR_GRAY2RGB)
    for b in staff_bases:
        for line in b-(np.array(range(5)) * (staff_gap + staff_line_thickness)):
            for col in range(width(img_colored_staff_base)):
                img_colored_staff_base[line][col][0] = 0
                img_colored_staff_base[line][col][1] = 0
                img_colored_staff_base[line][col][2] = 255

    return img_colored_staff_base

def show_wait_destroy(winname, img):
    cv.imshow(winname, img)
    cv.moveWindow(winname, 500, 0)
    cv.waitKey(0)
    cv.destroyWindow(winname)