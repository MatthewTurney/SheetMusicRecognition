import cv2
import sys
import numpy as np
from preprocessing import binary_threshold
from staff import remove_staff
from extensions import height, width, draw_staff_lines

if __name__ == "__main__":
    # Read specified image from file
    img_file = sys.argv[1]
    img = cv2.imread(img_file, 0)

    # Apply binary thresholding so values are in {0, 255}
    # where 0 is black, 255 is white
    binary_img = binary_threshold(img)
    cv2.imwrite('./images/binary_img.jpg', img) # save thresholded image for debugging


    img_no_staff, staff_bases, staff_gap, staff_line_thickness = remove_staff(binary_img)

    img_colored_staff_base = draw_staff_lines(img_no_staff, staff_bases, staff_gap, staff_line_thickness)
    cv2.imwrite('./images/img_colored_staffs.jpg', img_colored_staff_base)
    cv2.imwrite('./images/img_no_staff.jpg', img_no_staff)