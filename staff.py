import numpy as np
import cv2 as cv
from extensions import height, width, show_wait_destroy
from preprocessing import binary_threshold

# detect and remove the staff from the image
# returns: the new image, an array of indices of the bottom line of each staff, and the average width (in pixels) between each staff line
def remove_staff(img, method="morph"):
    img = binary_threshold(img)

    # create binary mask for staff rows
    staff_mask = generate_staff_mask_morph(img) if method == "morph" else generate_staff_mask_histogram(img)

    white_widths, black_widths = calc_image_widths(img, staff_mask)
        
    # find the average thickness of each staff line
    avg_staff_thickness = 1
    if (len(black_widths) > 0):
        avg_staff_thickness = int(np.round(np.mean(black_widths)))

    # convolve to find each staff stanza, assuming they are done in rows of one staff
    # (use conv = [0, 1, 1, 1, 1, 0, -1, -1, -1, -1, 0]) for sheet music that has 2 staffs per stanza row - but that's more complex
    staff_bases = []
    conv = [0, 1, 1, -1, -1,0]
    conv_result = [np.abs(n) for n in np.convolve(conv, white_widths, mode='valid')]
    thresh = np.max(conv_result) / 10.0
    for i in range(len(conv_result)):
        if (conv_result[i] < thresh):
            staff_bases.append(sum(white_widths[:i+5]) + ((1 + len(staff_bases)) * 5 * avg_staff_thickness) - 1)

    # find the average gap in between each staff line
    thresh = np.max(white_widths) / 10.0
    avg_staff_gap = int(np.round(np.mean([gap for gap in white_widths if gap < thresh])))

    img_no_staff = remove_staff_lines(img, staff_mask)

    return img_no_staff, staff_bases, avg_staff_gap, avg_staff_thickness

def calc_image_widths(img, mask):
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
    return (hist > (np.mean(hist) + 2.0*np.std(hist))).astype(np.int32)


def generate_staff_mask_morph(img):
    gray = img.copy()

    # Apply adaptiveThreshold at the bitwise_not of gray
    gray = cv.bitwise_not(gray)
    bw = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, \
                                cv.THRESH_BINARY, 15, -2)

    # Show binary image
    #show_wait_destroy("binary", bw)

    horizontal = np.copy(bw)
    horizontal_size = width(bw) // 30

    # Create structure element for extracting horizontal lines through morphology operations
    horizontalStructure = cv.getStructuringElement(cv.MORPH_RECT, (horizontal_size, 1))

    # Apply morphology operations
    horizontal = cv.erode(horizontal, horizontalStructure)
    horizontal = cv.dilate(horizontal, horizontalStructure)

    # Show extracted horizontal lines
    #show_wait_destroy("horizontal", horizontal)

    # create <image_height> length binary mask for staff lines, 125 threshold arbitrary
    return np.array([int(np.round(np.mean(row))) > 125 for row in horizontal]).astype(np.int32)


# since the staff may be more that one pixel thick, get the value of the lowest pixel above this one *that is not part of the staff*
def get_next_non_staff_above_pixel(img, hist, i, j):
    next_pixel = i - 1
    while(i > 1):
        if (hist[next_pixel] == 0): # not part of the staff
            return img[next_pixel][j]
        next_pixel -= 1
    return 255 # default to white

# since the staff may be more that one pixel thick, get the value of the highest pixel below this one *that is not part of the staff*
def get_next_non_staff_below_pixel(img, hist, i, j):
    next_pixel = i + 1
    while(i < height(img) - 1):
        if (hist[next_pixel] == 0): # not part of the staff
            return img[next_pixel][j]
        next_pixel += 1
    return 255 # default to white