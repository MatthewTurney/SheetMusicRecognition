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

def calc_runs(arr, val1, val2):
    ret1 = []
    ret2 = []
    i = 0
    while(i < len(arr)):
        mark = i
        while(i < len(arr) and arr[i] == val1):
            i+=1
        ret1.append(i - mark)
        mark = i
        while(i < len(arr) and arr[i] == val2):
            i+=1
        ret2.append(i - mark)
    return ret1, ret2

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

def calc_staff_line(slopes, row, img):
    col = 0
    r = row
    line = []
    for slope in slopes:
        if (int(np.round(r)) < height(img) and col < width(img)):
            line.append((int(np.round(r)), col))
        r += slope
        col += 1
    return line

def calc_staff_line_chunks(slopes, row, img, chunk_size):
    col = 0
    r = row
    line = []
    for col in range(width(img)):
        if (int(np.round(r)) < height(img) and col < width(img)):
            line.append((int(np.round(r)), col))
        if (int(r // chunk_size) < len(slopes) and int(r // chunk_size) >= 0):
            r += slopes[int(r // chunk_size)][col]
        col += 1
    return line

def draw_candidates(draw_img, staff_cand):
    colored_img = cv.cvtColor(draw_img.copy(), cv.COLOR_GRAY2RGB)
    for cand in staff_cand:
        x1,y1,x2,y2 = 0, cand[0], 40, cand[0]
        cv.line(colored_img,(x1,y1),(x2,y2),(0,255,0),2)
    return colored_img

def draw_staff_line(draw_img, line):
    for pt in line:
        cv.circle(draw_img, (pt[1], pt[0]), 1,(0,255,0),-1)
    return draw_img


if __name__ == "__main__":
    # Read specified image from file
    img_file = sys.argv[1]
    img = cv.imread(img_file, 0)
    if img is None:
        raise RuntimeError("Image path not found. Did you forget to specify the images/ folder?")

    img = cv.resize(img, (1920, 1080), interpolation = cv.INTER_AREA)

    # display original image
    show_wait_destroy("Original image", img)

    # binary threshold
    img = (cv.bitwise_not(binary_threshold(img))).astype(np.uint8)
    show_wait_destroy("preprocessed image", img)

    # get rid of text, markings, etc
    new_img = img.copy()
    kernel = cv.getStructuringElement(cv.MORPH_CROSS, (3,3))
    dilated = cv.dilate(new_img, kernel, iterations=1)
    _, contours, hierarchy = cv.findContours(dilated, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
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
        wr, br = calc_runs(img[:, i], 255, 0)
        white_runs.extend(wr)
        black_runs.extend(br)

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
    staff_img = img.copy()
    edges = cv.Canny(staff_img,50,150,apertureSize = 3)
    minLineLength = 20
    maxLineGap = 5
    lines = cv.HoughLinesP(edges,1,np.pi/180,100,minLineLength=minLineLength,maxLineGap=maxLineGap)
    if (lines is None):
      raise RuntimeError("No staff lines found")
    colored_img = cv.cvtColor(staff_img, cv.COLOR_GRAY2RGB)
    for line in lines:
        x1,y1,x2,y2 = line[0]
        cv.line(colored_img,(x1,y1),(x2,y2),(0,255,0),2)

    # calculate average staff line slope at every x pos
    chunks = 10
    chunk_size = height(img) // chunks
    slopes = [[0 for _ in range(width(img))] for c in range(chunks)]
    for i in range(width(img)):
        s = [0 for _ in range(chunks)]
        num = [0 for _ in range(chunks)]
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if (x1 < i and x2 > i) or (x1 < i and x2 > i):
                for chunk in range(chunks):
                    c1 = y1 // chunk_size
                    c2 = y2 // chunk_size
                    if (c1 >= chunk and c2 <= chunk) or (c2 >= chunk and c1 <= chunk):
                        s[chunk] += (y2 - y1) / (x2 - x1)
                        num[chunk] += 1

        for j in range(chunks):
            if num[j] > 0:
                slopes[j][i] = s[j] / num[j]

    show_wait_destroy("lines", colored_img)

    img_blank = img.copy()
    k = 0
    for s in slopes:
        row = int(np.round((height(img_blank) // len(slopes)) * (k + .5)))
        k += 1
        line = calc_staff_line(s, row, img)
        for pt in line:
            img_blank[pt[0]][pt[1]] = 255

    show_wait_destroy("average staff line orientation", img_blank)

    T_staff_cand = .15
    staff_candidates = []
    for row in range(height(img)):
        pts = calc_staff_line_chunks(slopes, row, img, chunk_size)
        total_white = sum(1 if img[p[0]][p[1]] == 255 else 0 for p in pts)
        if (total_white / width(img) > T_staff_cand):
            if (len(staff_candidates) == 0
            or (len(staff_candidates) > 0 and row - staff_candidates[-1][0] > staff_height / 3)):
                staff_candidates.append((row, pts, total_white))
            if (len(staff_candidates) > 0 and row - staff_candidates[-1][0] < staff_height / 3 and total_white > staff_candidates[-1][2]):
                staff_candidates[-1] = (row, pts, total_white)


    print([str(x[0]) + " " + str(x[2]) for x in staff_candidates])
    #cand_img = draw_candidates(img, staff_candidates)
    #show_wait_destroy("staff_candidates", cand_img)
    colored_img = cv.cvtColor(img.copy(), cv.COLOR_GRAY2RGB)
    for cand in staff_candidates:
        colored_img = draw_staff_line(colored_img, cand[1])

    show_wait_destroy("candidates", colored_img)

    

    

    # T_staff_lines = 10
    # k = 3

    # list_o = []
    # ccs = []
    # for col in range(width(img)):
    #     cc = []
    #     row = 0
    #     while(row < height(img)):
    #         while(row < height(img) and img[row][col] == 0):
    #             row+=1
    #         mark = row
    #         while(row < height(img) and img[row][col] == 255):
    #             row+=1
    #         #cc.append((mark, row - 1)) # inclusive
    #         cc.append(mark + ((row - mark) // 2))
    #     ccs.append(cc)

    # missing_cols = []
    # for i in range(width(img) - 2):
    #     if (len(ccs[i]) >= T_staff_lines):
    #         oriens = []
    #         for center in ccs[i]:
    #             o = 0.0
    #             jcol = 1.0
    #             for j in range(i + 1, i + k + 1):
    #                 if (j < width(img) and len(ccs[j]) > 0):
    #                    o += (1.0/jcol) * dist_to_nearest(center, ccs[j])
    #                 jcol+=1.0
    #             oriens.append(o)
    #         #o_i = ((1.0 / len(oriens)) * sum(oriens))
    #         o_i = np.median(oriens)
    #         list_o.append(o_i)
    #     else:
    #         list_o.append(0.0)
    #         missing_cols.append(i)


    # print(sum(list_o))
    # print(len(missing_cols))

    # img_blank = img.copy()
    # row = height(img_blank) / 2.0
    # col = 0
    # for orientation in list_o:
    #     img_blank[int(np.round(row))][col] = 255
    #     row += orientation
    #     col += 1

    # show_wait_destroy("staff orientations", img_blank)


