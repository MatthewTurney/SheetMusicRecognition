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

def calc_staff_line_chunks(slopes, row, img, chunk_size):
    white_seen = 0
    col = 0
    r = row
    line = []
    for col in range(width(img)):
        if (int(np.round(r)) < height(img) and col < width(img)):
            line.append((int(np.round(r)), col))
            if(img[int(np.round(r))][col] == 255):
                white_seen += 1
        if (int(r // chunk_size) < len(slopes) and int(r // chunk_size) >= 0):
            r += slopes[int(r // chunk_size)][col]
        col += 1
    return line, white_seen

def calc_staff(chunked_slopes, row, col, chunk_size):
    chunk = row // chunk_size
    r = row
    for c in range(col):
        chunk = int(np.round(r // chunk_size))
        r += chunked_slopes[chunk][c]
    return int(np.round(r))

def draw_candidates(draw_img, staff_cand):
    colored_img = cv.cvtColor(draw_img.copy(), cv.COLOR_GRAY2RGB)
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
    for i in range(0, width(img), 10):
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

    # only consider rows in chunks that contain white pixels (perf improvement)
    chunk_size = height(img) // num_chunks
    #rows = []
    #for chunk in range(num_chunks):
    #    chunk_start = chunk * chunk_size
    #    chunk_end = min(((chunk + 1) * chunk_size), height(img))
        #if (np.sum(img[chunk_start : chunk_end][:]) > 0):
    #    rows.extend([i for i in range(chunk_start, chunk_end)])

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
    for cand in staff_candidates:
        for row, col in cand[1]:
            if staff_removed[row][col] == 255:
                start = row
                while(start > 0 and staff_removed[start][col] == 255):
                    start -= 1
                end = row
                while(end < height(img) - 1 and staff_removed[end][col] == 255):
                    end += 1
                if (end - start <= T_length):
                    for j in range(start + 1, end):
                        staff_removed[j][col] = 0
            else:
                j = max(row - T_length, 0)
                while(j < height(staff_removed) - 1 and j <= row + T_length and staff_removed[j][col] == 0):
                    j += 1
                if (staff_removed[j][col] == 255):
                    start = j
                    while(j < height(staff_removed) - 1 and j <= row + T_length and staff_removed[j][col] == 255):
                        j += 1
                    if (j - start <= T_length):
                        for x in range(start, j):
                            staff_removed[x][col] = 0

    return staff_removed
        
def draw_staff(img, staff_candidates):
    colored_img = img.copy()
    #colored_img = cv.cvtColor(staff_img, cv.COLOR_GRAY2RGB)
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



# class Staff():

#     def __init__(self, bases, white_gap, line_thickness, chunk_size):
#         self.bases = bases
#         self.gap = white_gap
#         self.line_thickness = line_thickness
#         self.chunk_size = chunk_size

#     def calculate_pitch(self, note):
#         bs = [b for b in self.bases if b > note.centery]
#         if len(bs) == 0:
#             return None
#         base = min(bs)
#         return (base - note.centery) // ((self.line_thickness + self.gap) // 2)

# # detect and remove the staff from the image
# # returns: the new image, an array of indices of the bottom line of each staff, and the average width (in pixels) between each staff line
# def remove_staff(img, original, method="morph"):
#     #img = binary_threshold(img)
#     # create binary mask for staff rows
#     staff_masks = []

#     chunk_size = 300
#     for i in range(0, width(img), chunk_size):
#         staff_mask = generate_staff_mask_morph(img[:, i:min(width(img), i + chunk_size)]) #if method == "morph" else generate_staff_mask_histogram(img)
#         staff_masks.append(staff_mask)

#     ww = []
#     bw = []
#     for mask in staff_masks:
#         white_widths, black_widths = calc_image_widths(mask)
#         ww.append(white_widths)
#         bw.append(black_widths)

#     sb = []
#     bt = []
#     wt = []

#     for i in range(len(ww)):       
#         # find the average thickness of each staff line
#         w = ww[i]
#         b = bw[i]

#         avg_staff_thickness = 1
#         if (len(b) > 0):
#             avg_staff_thickness = int(np.round(np.mean(b)))

#         bt.append(avg_staff_thickness)

#         # convolve to find each staff stanza, assuming they are done in rows of one staff
#         # (use conv = [0, 1, 1, 1, 1, 0, -1, -1, -1, -1, 0]) for sheet music that has 2 staffs per stanza row - but that's more complex
#         staff_bases = []
#         conv = [0, 1, 1, -1, -1,0]
#         conv_result = [np.abs(n) for n in np.convolve(conv, w, mode='valid')]
#         thresh = 20
#         for i in range(len(conv_result)):
#             if (conv_result[i] < thresh):
#                 staff_bases.append(sum(w[:i+5]) + ((1 + len(staff_bases)) * 5 * avg_staff_thickness) - 1)

#         sb.append(staff_bases)
#         # find the average gap in between each staff line
#         thresh = np.mean(w) #np.max(white_widths) / 10.0
#         if (len([gap for gap in w if gap < thresh]) > 0):
#             avg_staff_gap = int(np.round(np.mean([gap for gap in w if gap < thresh])))
#         else:
#             avg_staff_gap = 0
#         wt.append(avg_staff_gap)


#     chunks = [remove_staff_lines(original[:, i:min(width(original), i + chunk_size)], staff_masks[i // chunk_size]) for i in range(0, width(original), chunk_size)]
#     #chunks = [np.array(img[:, i:min(width(img), i + chunk_size)]) for i in range(0, width(img), chunk_size)]
#     img_no_staff = np.concatenate(chunks, axis=1)
#     show_wait_destroy("img no staff", img_no_staff)
#     return img_no_staff, Staff(sb, bt, wt, chunk_size)

# def calc_image_widths(mask):
#     # create an array of the widths of each continuous chunk of white
#     white_widths = []
#     black_widths = []
#     i = 0
#     while(i < height(mask) and mask[i] == 1):
#         i+=1
#     while (i < height(mask)):
#         start = i
#         while(i < height(mask) and mask[i] == 0):
#             i+=1
#         white_widths.append(i - start)
#         b = i
#         while(i < height(mask) and mask[i] == 1):
#             i+=1
#         if (i - b > 0):
#             black_widths.append(i - b)

#     return white_widths, black_widths

# # remove the staff lines from the image, given a mask of staff line locations. Fill pixels with the values 
# # above/below to avoid leaving white gaps in notes
# def remove_staff_lines(img, hist):
#     img_no_staff = img.copy()
#     for i in range(height(img_no_staff)):
#         if (hist[i] == 1):
#             for j in range(width(img_no_staff)):
#                 if (i == 0 or i == height(img_no_staff) - 1):
#                     break
#                 elif (get_next_non_staff_above_pixel(img_no_staff, hist, i, j) == 255 and get_next_non_staff_below_pixel(img_no_staff, hist, i, j) == 255):
#                     img_no_staff[i][j] = 255
#                 else:
#                     img_no_staff[i][j] == 0
#     return img_no_staff


# def generate_staff_mask_histogram(img):
#     # create a histogram of percent black pixels per row
#     hist = np.zeros((height(img)))
#     for row in range(height(img)):
#         hist[row] = float(sum(img[row] == 0)) / width(img)

#     # threshold histogram by values > (mean + 2 * stddev)
#     #return (hist > (np.mean(hist) + 2.0*np.std(hist))).astype(np.int32)
#     return (hist > np.array([width(img) // 2])).astype(np.int32)


# def generate_staff_mask_morph(img):
#     gray = img.copy()

#     # Apply adaptiveThreshold at the bitwise_not of gray
#     #gray = cv.bitwise_not(gray)
#     #bw = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, \
#     #                            cv.THRESH_BINARY, 15, -2)

#     # Show binary image
#     #show_wait_destroy("binary", bw)

#     horizontal = np.copy(gray)
#     horizontal_size = width(gray) // 10

#     # Create structure element for extracting horizontal lines through morphology operations
#     horizontalStructure = cv.getStructuringElement(cv.MORPH_RECT, (horizontal_size, 1))

#     # Apply morphology operations
#     horizontal = cv.erode(horizontal, horizontalStructure)
#     horizontal = cv.dilate(horizontal, horizontalStructure)



#     # Show extracted horizontal lines
#     #show_wait_destroy("horizontal", horizontal, resize=False)

#     # create <image_height> length binary mask for staff lines, 125 threshold arbitrary
#     mask = np.array([np.mean(row) < 0.4 for row in horizontal]).astype(np.int32)
#     return mask


# # since the staff may be more that one pixel thick, get the value of the lowest pixel above this one *that is not part of the staff*
# def get_next_non_staff_above_pixel(img, hist, i, j):
#     next_pixel = i - 1
#     while(next_pixel > 1):
#         if (hist[next_pixel] == 0): # not part of the staff
#             return img[next_pixel][j]
#         next_pixel -= 1
#     return 255 # default to white

# # since the staff may be more that one pixel thick, get the value of the highest pixel below this one *that is not part of the staff*
# def get_next_non_staff_below_pixel(img, hist, i, j):
#     next_pixel = i + 1
#     while(next_pixel < height(img) - 1):
#         if (hist[next_pixel] == 0): # not part of the staff
#             return img[next_pixel][j]
#         next_pixel += 1
#     return 255 # default to white