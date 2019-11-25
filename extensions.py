import cv2 as cv
import numpy as np

def height(img):
    return img.shape[0]

def width(img):
    return img.shape[1]

def draw_staff_lines(img_no_staff, staff):
    img_colored_staff_base = cv.cvtColor(img_no_staff, cv.COLOR_GRAY2RGB)
    for i in range(len(staff.bases)):
        for b in staff.bases[i]:
            for line in b-(np.array(range(5)) * (staff.gap[i] + staff.line_thickness[i])):
                for col in range(i * staff.chunk_size, (i+1) * staff.chunk_size):
                    if (col >= width(img_no_staff) or line >= height(img_no_staff)):
                        break
                    img_colored_staff_base[line][col][0] = 0
                    img_colored_staff_base[line][col][1] = 0
                    img_colored_staff_base[line][col][2] = 255

    return img_colored_staff_base

def show_wait_destroy(winname, img, resize=True):
    if (resize):
        img_resized = cv.resize(img.copy(), (720, 480))
    else:
        img_resized = img
    cv.imshow(winname, img_resized)
    cv.moveWindow(winname, 500, 0)
    cv.waitKey(0)
    cv.destroyWindow(winname)