import numpy as np
import cv2 as cv
from extensions import height, width, show_wait_destroy
from preprocessing import binary_threshold
from extensions import show_wait_destroy

class Staff():

    def __init__(self, bases, white_gap, line_thickness, chunk_size):
        self.bases = bases
        self.gap = white_gap
        self.line_thickness = line_thickness
        self.chunk_size = chunk_size

    def calculate_pitch(self, note):
        bs = [b for b in self.bases if b > note.centery]
        if len(bs) == 0:
            return None
        base = min(bs)
        return (base - note.centery) // ((self.line_thickness + self.gap) // 2)

# detect and remove the staff from the image
# returns: the new image, an array of indices of the bottom line of each staff, and the average width (in pixels) between each staff line
def remove_staff(img, original, method="morph"):
    #img = binary_threshold(img)
    # create binary mask for staff rows
    staff_masks = []

    chunk_size = 300
    for i in range(0, width(img), chunk_size):
        staff_mask = generate_staff_mask_morph(img[:, i:min(width(img), i + chunk_size)]) #if method == "morph" else generate_staff_mask_histogram(img)
        staff_masks.append(staff_mask)

    ww = []
    bw = []
    for mask in staff_masks:
        white_widths, black_widths = calc_image_widths(mask)
        ww.append(white_widths)
        bw.append(black_widths)

    sb = []
    bt = []
    wt = []

    for i in range(len(ww)):       
        # find the average thickness of each staff line
        w = ww[i]
        b = bw[i]

        avg_staff_thickness = 1
        if (len(b) > 0):
            avg_staff_thickness = int(np.round(np.mean(b)))

        bt.append(avg_staff_thickness)

        # convolve to find each staff stanza, assuming they are done in rows of one staff
        # (use conv = [0, 1, 1, 1, 1, 0, -1, -1, -1, -1, 0]) for sheet music that has 2 staffs per stanza row - but that's more complex
        staff_bases = []
        conv = [0, 1, 1, -1, -1,0]
        conv_result = [np.abs(n) for n in np.convolve(conv, w, mode='valid')]
        thresh = 20
        for i in range(len(conv_result)):
            if (conv_result[i] < thresh):
                staff_bases.append(sum(w[:i+5]) + ((1 + len(staff_bases)) * 5 * avg_staff_thickness) - 1)

        sb.append(staff_bases)
        # find the average gap in between each staff line
        thresh = np.mean(w) #np.max(white_widths) / 10.0
        if (len([gap for gap in w if gap < thresh]) > 0):
            avg_staff_gap = int(np.round(np.mean([gap for gap in w if gap < thresh])))
        else:
            avg_staff_gap = 0
        wt.append(avg_staff_gap)


    chunks = [remove_staff_lines(original[:, i:min(width(original), i + chunk_size)], staff_masks[i // chunk_size]) for i in range(0, width(original), chunk_size)]
    #chunks = [np.array(img[:, i:min(width(img), i + chunk_size)]) for i in range(0, width(img), chunk_size)]
    img_no_staff = np.concatenate(chunks, axis=1)
    show_wait_destroy("img no staff", img_no_staff)
    return img_no_staff, Staff(sb, bt, wt, chunk_size)

def calc_image_widths(mask):
    # create an array of the widths of each continuous chunk of white
    white_widths = []
    black_widths = []
    i = 0
    while(i < height(mask) and mask[i] == 1):
        i+=1
    while (i < height(mask)):
        start = i
        while(i < height(mask) and mask[i] == 0):
            i+=1
        white_widths.append(i - start)
        b = i
        while(i < height(mask) and mask[i] == 1):
            i+=1
        if (i - b > 0):
            black_widths.append(i - b)

    return white_widths, black_widths

# remove the staff lines from the image, given a mask of staff line locations. Fill pixels with the values 
# above/below to avoid leaving white gaps in notes
def remove_staff_lines(img, hist):
    img_no_staff = img.copy()
    for i in range(height(img_no_staff)):
        if (hist[i] == 1):
            for j in range(width(img_no_staff)):
                if (i == 0 or i == height(img_no_staff) - 1):
                    break
                elif (get_next_non_staff_above_pixel(img_no_staff, hist, i, j) == 255 and get_next_non_staff_below_pixel(img_no_staff, hist, i, j) == 255):
                    img_no_staff[i][j] = 255
                else:
                    img_no_staff[i][j] == 0
    return img_no_staff


def generate_staff_mask_histogram(img):
    # create a histogram of percent black pixels per row
    hist = np.zeros((height(img)))
    for row in range(height(img)):
        hist[row] = float(sum(img[row] == 0)) / width(img)

    # threshold histogram by values > (mean + 2 * stddev)
    #return (hist > (np.mean(hist) + 2.0*np.std(hist))).astype(np.int32)
    return (hist > np.array([width(img) // 2])).astype(np.int32)


def generate_staff_mask_morph(img):
    gray = img.copy()

    # Apply adaptiveThreshold at the bitwise_not of gray
    #gray = cv.bitwise_not(gray)
    #bw = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, \
    #                            cv.THRESH_BINARY, 15, -2)

    # Show binary image
    #show_wait_destroy("binary", bw)

    horizontal = np.copy(gray)
    horizontal_size = width(gray) // 10

    # Create structure element for extracting horizontal lines through morphology operations
    horizontalStructure = cv.getStructuringElement(cv.MORPH_RECT, (horizontal_size, 1))

    # Apply morphology operations
    horizontal = cv.erode(horizontal, horizontalStructure)
    horizontal = cv.dilate(horizontal, horizontalStructure)



    # Show extracted horizontal lines
    #show_wait_destroy("horizontal", horizontal, resize=False)

    # create <image_height> length binary mask for staff lines, 125 threshold arbitrary
    mask = np.array([np.mean(row) < 0.4 for row in horizontal]).astype(np.int32)
    return mask


# since the staff may be more that one pixel thick, get the value of the lowest pixel above this one *that is not part of the staff*
def get_next_non_staff_above_pixel(img, hist, i, j):
    next_pixel = i - 1
    while(next_pixel > 1):
        if (hist[next_pixel] == 0): # not part of the staff
            return img[next_pixel][j]
        next_pixel -= 1
    return 255 # default to white

# since the staff may be more that one pixel thick, get the value of the highest pixel below this one *that is not part of the staff*
def get_next_non_staff_below_pixel(img, hist, i, j):
    next_pixel = i + 1
    while(next_pixel < height(img) - 1):
        if (hist[next_pixel] == 0): # not part of the staff
            return img[next_pixel][j]
        next_pixel += 1
    return 255 # default to white