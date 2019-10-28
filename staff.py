import numpy as np
from extensions import height, width

# detect and remove the staff from the image
# returns: the new image, an array of indices of the bottom line of each staff, and the average width (in pixels) between each staff line
def remove_staff(img):

    # create a histogram of percent black pixels per row
    hist = np.zeros((height(img)))
    for row in range(height(img)):
        hist[row] = float(sum(img[row] == 0)) / width(img)

    # threshold histogram by values > (mean + 2 * stddev)
    hist = (hist > (np.mean(hist) + 2.0*np.std(hist))).astype(np.int32)

    # create an array of the widths of each continuous chunk of white
    white_widths = []
    black_widths = []
    i = 0
    while(i < height(hist) and hist[i] == 1):
        i+=1
    while (i < height(hist)):
        start = i
        while(i < height(hist) and hist[i] == 0):
            i+=1
        white_widths.append(i - start)
        b = i
        while(i < height(hist) and hist[i] == 1):
            i+=1
        if (i - b > 0):
            black_widths.append(i - b)

        
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

    # remove the staff lines from the image
    # where a staff line is removed, replace with the value of the next pixels above/below that aren't part of the staff line
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

    return img_no_staff, staff_bases, avg_staff_gap, avg_staff_thickness



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