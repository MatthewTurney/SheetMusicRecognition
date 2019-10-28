import cv2 as cv
import sys
import numpy as np
from preprocessing import binary_threshold
from staff import remove_staff
from extensions import height, width, draw_staff_lines, show_wait_destroy



if __name__ == "__main__":
    # Read specified image from file
    img_file = sys.argv[1]
    img = cv.imread(img_file, 0)

    # Apply binary thresholding so values are in {0, 255}
    # where 0 is black, 255 is white
    #binary_img = binary_threshold(img)
    #cv.imwrite('./images/binary_img.jpg', binary_img) # save thresholded image for debugging
    show_wait_destroy("Staff removed", img)
    img_no_staff, staff_bases, staff_gap, staff_line_thickness = remove_staff(img)
    show_wait_destroy("Staff removed", img_no_staff)
    img_colored_staff_base = draw_staff_lines(img_no_staff, staff_bases, staff_gap, staff_line_thickness)
    show_wait_destroy("Staff lines drawn in", img_colored_staff_base)

    #cv.imwrite('./images/img_colored_staffs.jpg', img_colored_staff_base)
    #cv.imwrite('./images/img_no_staff.jpg', img_no_staff)