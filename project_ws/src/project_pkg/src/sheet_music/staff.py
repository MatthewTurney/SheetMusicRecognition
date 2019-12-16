import numpy as np
import cv2 as cv
from extensions import height, width, show_wait_destroy
from preprocessing import binary_threshold, calc_runs
from extensions import show_wait_destroy

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

def calc_staff_line_chunks(slopes, row, img, chunk_size, interval = 15):
    white_seen = 0
    col = 0
    r = row
    line = []
    for col in range(0, width(img), interval):
        if (int(np.round(r)) < height(img) and col < width(img)):
            for i in range(interval):
                line.append((int(np.round(r)), col + i))
            if(img[int(np.round(r))][col] == 255):
                white_seen += 1
        if (int(r // chunk_size) < len(slopes) and int(r // chunk_size) >= 0):
            r += slopes[int(r // chunk_size)][col] * float(interval)
        col += 1
    return line, white_seen*interval

def calc_staff(chunked_slopes, row, col, chunk_size):
    chunk = row // chunk_size
    r = row
    for c in range(col):
        chunk = int(np.round(r // chunk_size))
        r += chunked_slopes[chunk][c]
    return int(np.round(r))

def draw_candidates(draw_img, staff_cand):
    colored_img = draw_img
    for cand in staff_cand:
        x1,y1,x2,y2 = 0, cand[0], 40, cand[0]
        cv.line(colored_img,(x1,y1),(x2,y2),(0,255,0),2)
    return colored_img

def draw_staff_line(draw_img, row, line):
    for pt in line:
        if (pt != -1):
            cv.circle(draw_img, (pt[1], pt[0]), 1,(0,255,0),-1)
    return draw_img

def intersects(start, end, candidates):
    for i in range(start, end):
        for c in candidates:
            if (i in c[1]):
                return True
    return False

def estimate_staff_stats(img):
    white_runs = []
    black_runs = []
    for i in range(0, width(img), 20):
        wr, br = calc_runs(img[:, i], 255, 0)
        white_runs.extend(wr)
        black_runs.extend(br)

    staff_space = int(np.median(black_runs))
    staff_height = int(np.median(white_runs)) #image is bit flipped

    T_length = int(min(2.0 * staff_height, staff_height + staff_space))

    return staff_height, staff_space, T_length

def create_initial_staff_image(img, T_length):
    img_copy = img.copy()
    for col in range(width(img_copy)):
        i = 0
        while(i < height(img_copy)):
            while(i < height(img_copy) and img_copy[i][col] == 0):
                i+=1
            mark = i
            while(i < height(img_copy) and img_copy[i][col] == 255):
                i+=1
            if (i - mark > T_length):
                for j in range(mark, i):
                    img_copy[j][col] = 0

    return img_copy

def find_lines(img, minLineLength, maxLineGap):
    staff_img = img.copy()
    edges = cv.Canny(staff_img,50,150,apertureSize = 3)
    lines = cv.HoughLinesP(edges,1,np.pi/180,100,minLineLength=minLineLength,maxLineGap=maxLineGap)
    if (lines is None):
      raise RuntimeError("No staff lines found")
    colored_img = cv.cvtColor(staff_img, cv.COLOR_GRAY2RGB)
    for line in lines:
        x1,y1,x2,y2 = line[0]
        cv.line(colored_img,(x1,y1),(x2,y2),(0,255,0),2)

    return colored_img, lines

def calc_slopes(img, chunks, lines):
    chunk_size = height(img) // chunks
    slopes = [[0 for _ in range(width(img))] for c in range(chunks)]
    for i in range(width(img)):
        s = [0 for _ in range(chunks)]
        num = [0 for _ in range(chunks)]
        for line in lines:
            x1,y1,x2,y2 = line[0]
            if (x1 <= i and x2 >= i) or (x1 <= i and x2 >= i):
                for chunk in range(chunks):
                    c1 = y1 // chunk_size
                    c2 = y2 // chunk_size
                    if (c1 >= chunk and c2 <= chunk) or (c2 >= chunk and c1 <= chunk):
                        s[chunk] += float(y2 - y1) / float(x2 - x1)
                        num[chunk] += 1

        for j in range(chunks):
            if num[j] > 0:
                slopes[j][i] = float(s[j]) / float(num[j])


    img_blank = img.copy()
    k = 0
    for s in slopes:
        row = int(np.round((height(img_blank) // len(slopes)) * (k + .5)))
        k += 1
        line = calc_staff_line(s, row, img)
        for pt in line:
            img_blank[pt[0]][pt[1]] = 255

    return slopes, img_blank


def find_staff_candidates(img, slopes, T_staff_cand, T_length, num_chunks, staff_height, staff_space, min_y, max_y):
    staff_candidates = []
    chunk_size = height(img) // num_chunks

    rows = [i for i in range(min_y - 50, max_y + 50)]

    for row in rows:
        pts, total_white = calc_staff_line_chunks(slopes, row, img, chunk_size)
        if total_white > 0:
            if (float(total_white) / width(img) > T_staff_cand):
                if (len(staff_candidates) == 0
                or (len(staff_candidates) > 0 and row - staff_candidates[-1][0] > T_length)):
                    staff_candidates.append((row, pts, total_white))
                if (len(staff_candidates) > 0 and row - staff_candidates[-1][0] <= T_length and total_white > staff_candidates[-1][2]):
                    staff_candidates[len(staff_candidates) - 1] = (row, pts, total_white)

    to_remove = []
    T_staff = (5 * staff_height) + (7 * staff_space)
    curr = []
    for cand in staff_candidates:
        curr.append(cand)
        if (curr[-1][0] - curr[0][0] > T_staff):
            curr = curr[:len(curr) - 1]
            while (len(curr) > 5):
                tr = [x for x in curr if x[2] == min([c[2] for c in curr])][0]
                to_remove.append(tr)
                curr = [x for x in curr if x != tr]

            curr = []

    for r in to_remove:
        staff_candidates.remove(r)

    print([str(x[0]) + " " + str(x[2]) for x in staff_candidates])
    return staff_candidates


def remove_staff(img, staff_candidates, T_length):
    staff_removed = img.copy()
    for c in range(len(staff_candidates)):
        cand = staff_candidates[c]
        for row, col in cand[1]:
            mids = []
            j = max(row - int(T_length * 1.5), 0)
            while(j < height(staff_removed) - 1 and j <= row + T_length):
                while staff_removed[j][col] == 255:
                    j+=1
                while(j < height(staff_removed) - 1 and j <= row + int(T_length * 1.5) and staff_removed[j][col] == 0):
                    j += 1
                if (staff_removed[j][col] == 255):
                    start = j
                    while(j < height(staff_removed) - 1 and j <= row + int(T_length * 1.5) and staff_removed[j][col] == 255):
                        j += 1
                    if (j - start <= T_length):
                        middle = start + ((j - start) // 2) + 1
                        if (np.abs(row - middle) < 10):
                            mids.append(middle)
                        for x in range(start, j):
                            staff_removed[x][col] = 0
            if len(mids) > 0 and col > 5:
                avg = int(np.round(sum([staff_candidates[c][1][col - i][0] for i in range(5)]) / 5.0))


                dists = [np.abs(row - m) for m in mids]
                staff_candidates[c][1][col] = ((mids[np.argmin(dists)] + avg) // 2, col)
            elif col > 20:
                avg = int(np.round(sum([staff_candidates[c][1][col - i][0] for i in range(20)]) / 20.0))
                staff_candidates[c][1][col] = (avg, col)

    return staff_removed, staff_candidates


def draw_staff(img, staff_candidates):
    colored_img = img.copy()
    for cand in staff_candidates:
        for row, col in cand[1]:
            colored_img[row, col][0] = 0
            colored_img[row, col][1] = 255
            colored_img[row, col][2] = 0

            colored_img[row + 1, col][0] = 0
            colored_img[row + 1, col][1] = 255
            colored_img[row + 1, col][2] = 0

            colored_img[row - 1, col][0] = 0
            colored_img[row - 1, col][1] = 255
            colored_img[row - 1, col][2] = 0

    return colored_img