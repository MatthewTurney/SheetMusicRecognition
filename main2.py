import cv2 as cv
import sys
import numpy as np
import imutils

from preprocessing import binary_threshold
from staff import remove_staff
from extensions import height, width, draw_staff_lines, show_wait_destroy
from template_matching import match_template, is_duplicate
from note import Note

def calc_runs(arr, val):
    ret = []
    i = 0
    while(i < len(arr)):
        while(i < len(arr) and arr[i] != val):
            i+=1
        mark = i
        while(i < len(arr) and arr[i] == val):
            i+=1
        ret.append(i - mark)
    return ret

def dist_to_nearest(center, centers2, threshold=3):
    start = center
    dist = 0
    while((start + dist <= max(centers2) or start - dist >= 0) and dist < threshold):
        if (start - dist in centers2):
            return -1.0 * dist
        if (start + dist in centers2):
            return 1.0 * dist
        dist += 1
    return 0

if __name__ == "__main__":
    # Read specified image from file
    img_file = sys.argv[1]
    img = cv.imread(img_file, 0)
    if img is None:
        raise RuntimeError("Image path not found. Did you forget to specify the images/ folder?")

    # display original image
    show_wait_destroy("Original image", img)

    # binary threshold
    img = (cv.bitwise_not(binary_threshold(img))).astype(np.uint8)
    show_wait_destroy("preprocessed image", img)

    # get rid of text, markings, etc
    new_img = img.copy()
    kernel = cv.getStructuringElement(cv.MORPH_CROSS, (3,3))
    dilated = cv.dilate(new_img, kernel, iterations=1)
    contours, hierarchy = cv.findContours(dilated, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    for contour in contours:
        [x, y, w, h] = cv.boundingRect(contour)
        #if w < 35 and h < 35:
        #    continue
        if (w < width(img) // 2):
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 0), cv.FILLED)

    show_wait_destroy("text removed", img)


    # create initial staff image
    white_runs = []
    black_runs = []
    for i in range(width(img)):
        white_runs.extend(calc_runs(img[:, i], 0))
        black_runs.extend(calc_runs(img[:, i], 255))

    staff_height = np.median(black_runs)
    staff_space = np.median(white_runs)

    T_length = min(2.0 * staff_height, staff_height + staff_space)

    for col in range(width(img)):
        i = 0
        while(i < height(img)):
            while(i < height(img) and img[i][col] == 0):
                i+=1
            mark = i
            while(i < height(img) and img[i][col] == 255):
                i+=1
            if (i - mark > T_length):
                for j in range(mark, i):
                    img[j][col] = 0
        
    show_wait_destroy("initial staff image", img)

    # model line shape
    T_staff_lines = 10
    k = 3

    list_o = []
    ccs = []
    for col in range(width(img)):
        cc = []
        row = 0
        while(row < height(img)):
            while(row < height(img) and img[row][col] == 0):
                row+=1
            mark = row
            while(row < height(img) and img[row][col] == 255):
                row+=1
            #cc.append((mark, row - 1)) # inclusive
            cc.append(mark + ((row - mark) // 2))
        ccs.append(cc)

    missing_cols = []
    for i in range(width(img) - 2):
        if (len(ccs[i]) >= T_staff_lines):
            oriens = []
            for center in ccs[i]:
                o = 0.0
                jcol = 1.0
                for j in range(i + 1, i + k + 1):
                    if (j < width(img) and len(ccs[j]) > 0):
                       o += (1.0/jcol) * dist_to_nearest(center, ccs[j])
                    jcol+=1.0
                oriens.append(o)
            #o_i = ((1.0 / len(oriens)) * sum(oriens))
            o_i = np.median(oriens)
            list_o.append(o_i)
        else:
            list_o.append(0.0)
            missing_cols.append(i)


    print(sum(list_o))
    print(len(missing_cols))

    img_blank = img.copy()
    row = height(img_blank) / 2.0
    col = 0
    for orientation in list_o:
        img_blank[int(np.round(row))][col] = 255
        row += orientation
        col += 1

    show_wait_destroy("staff orientations", img_blank)

