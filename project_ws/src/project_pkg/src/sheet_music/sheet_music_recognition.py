import cv2 as cv
import sys
import numpy as np
import imutils
import MTM

from preprocessing import *
from staff import *
from extensions import *
from template_matching import *

# constants
IMAGE_SIZE = (1920, 1080)
NUM_CHUNKS = 10
T_STAFF_CAND = 0.15
T_STAFF_MATCH = 0.3
T_BAR_MATCH = 0.8
T_END_MATCH = 0.6
T_TIME_MATCH = 0.6
T_QUARTER_MATCH = 0.05
T_WHOLE_MATCH = 0.45
T_NOTES = [-1.8, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5]
minLineLength = 75
maxLineGap = 15

def process_sheet_music(filename, show_steps = True):
    # read image from file
    img = read_image_from_file(filename, color = True)
    
    # resize image
    img = cv.resize(img, IMAGE_SIZE, interpolation = cv.INTER_AREA)

    # display original image
    if show_steps:
        show_wait_destroy("Original Image", img)

    # remove shadows
    _, img = remove_shadows(img)

    if show_steps:
        show_wait_destroy("Shadows removed", img)

    img = convert_to_grayscale(img)

    # binary threshold
    img = (cv.bitwise_not(binary_threshold(img))).astype(np.uint8)
    if show_steps:
        show_wait_destroy("Binary Threshold", img)

    # get rid of text, markings, etc
    img = remove_text(img)
    if show_steps:
        show_wait_destroy("Text Removed", img)

    # save preprocessed original image for later use
    processed_img = img.copy()

    # create initial staff image
    staff_height, staff_space, T_length = estimate_staff_stats(img)
    print(staff_height)
    print(staff_space)
    img = create_initial_staff_image(img, T_length)
    isi = img.copy()
    if show_steps:
        show_wait_destroy("Initial Staff Image", img)

    # model line shape
    minLineLength = 75
    maxLineGap = 15
    colored_img, lines = find_lines(img, minLineLength, maxLineGap)
    min_y = min([l[0][1] for l in lines])
    max_y = max([l[0][1] for l in lines])
    if show_steps:
        show_wait_destroy("lines", colored_img)

    # calculate average staff line slope at every x pos
    slopes, slope_img = calc_slopes(img, NUM_CHUNKS, lines)
    if show_steps:
        show_wait_destroy("Average staff line orientation", slope_img)

    # find staff and display candidates
    staff_candidates = find_staff_candidates(img, slopes, T_STAFF_CAND, T_length, NUM_CHUNKS, staff_height, staff_space, min_y, max_y)

    colored_img = cv.cvtColor(img.copy(), cv.COLOR_GRAY2RGB)
    for cand in staff_candidates:
        colored_img = draw_staff_line(colored_img, cand[0], cand[1])
    if show_steps:
        show_wait_destroy("Staff candidates", colored_img)

    # remove staff from original (text removed) image
    staff_removed_img, staff_candidates = remove_staff(processed_img.copy(), staff_candidates, T_length)
    if show_steps:
        show_wait_destroy("Staff removed", staff_removed_img)
        show_wait_destroy("staff candidates corrected", draw_staff(cv.cvtColor(isi, cv.COLOR_GRAY2RGB), staff_candidates))


    est_total_staff_height = (4 * staff_space) + (5 * staff_height)
    colored = cv.cvtColor(staff_removed_img.copy(), cv.COLOR_GRAY2RGB)

    # remove isolated pixels
    img = remove_isolated_pixels(staff_removed_img)
    if show_steps:
        show_wait_destroy("isolated pixels removed", img)

    # remove staff thingys
    template = cv.imread('./images/staff_template.jpg', 0)
    if template is None:
        template = cv.imread('sheet_music/images/staff_template.jpg', 0)
    template = imutils.resize(template, height=int(est_total_staff_height * 2))
    img, _ = remove_template_matches(img, template, T_STAFF_MATCH)

    # find bar lines
    template = cv.imread('./images/bar_template.jpg', 0) 
    if template is None:
        template = cv.imread('sheet_music/images/bar_template.jpg', 0)
    img, bars = remove_template_matches_with_buffer(img, template, T_BAR_MATCH, 20)

    # remove 4/4 time and end markers
    template = cv.imread('./images/end.jpg', 0)
    if template is None:
        template = cv.imread('sheet_music/images/end.jpg', 0)
    img, end = remove_template_matches(img, template, T_END_MATCH)
    if (len(end) > 0):
        end = end[0]

    template = cv.imread('./images/time_template.jpg', 0)
    if template is None:
        template = cv.imread('sheet_music/images/time_template.jpg', 0)
    img, _ = remove_template_matches(img, template, T_TIME_MATCH)
    if show_steps:
        show_wait_destroy("Removed extraneous markings", img)

    # match quarter/half notes first
    template = cv.imread('./images/vertical_quarter_template5.jpg', 0)
    if template is None:
        template = cv.imread('sheet_music/images/vertical_quarter_template5.jpg', 0)
    template = imutils.resize(template, height=int(np.round(est_total_staff_height * 1.0)))

    whole_note_img = img.copy()
    notes = []
    colored_img = cv.cvtColor(img, cv.COLOR_GRAY2RGB)

    # fix center position based on bottom of note
    def fix_center(img_cropped, h):
        img_cropped = img_cropped.copy()
        h = h - 1
        while (h > 0):
            for x in range(img_cropped.shape[1]):
                if img_cropped[h, x] == 255:
                    return h
            h -= 1
        return 0
            
    # for distinguishing C's from D's based on horizontal line
    def left_right_diff(img_cropped, w):
        l = 0
        while l < w:
            if (sum(img_cropped[len(img_cropped)//2:, l]) > 0):
                break
            l+=1
        r = w - 1
        while r > 0:
            if (sum(img_cropped[len(img_cropped)//2:, r]) > 0):
                break
            r-=1
        return r - l

    lr = []

    for x, y, w, h in calc_boxes(template, T_QUARTER_MATCH, img):
        if (np.sum(img[y:y+h+1, x:x+w+1]) / 255 > 300) and \
            (np.sum(img[y:y+(h//2)+1, x:x+w+1]) < np.sum(img[y+(h//2):y+h+1, x:x+w+1])) and \
            (np.sum(img[y:y+(h//2)+1, x:x+w+1]) != 0) and \
            (np.sum(img[y+(h//2):y+h+1, x:x+w+1]) != 0):
            cv.rectangle(whole_note_img, (x,y-50), (x+w, y+h), (0, 0, 0), -1) # black out quarter/half notes for whole note detection
            new_vertical_offset = fix_center(img[y:y+h, x:x+w], h)
            cx = x + (w//2)
            cy = y+new_vertical_offset-(h//9)
            cv.circle(colored_img, (cx, cy), 4, (0, 0, 255), -1)

            lr_note = left_right_diff(img[y:y+h+1, x:x+w+1], w)

            if np.sum(img[y-5:y+20, x-10:x+w+30]) / 255 > 300:
                notes.append((cx,cy, "eighth", lr_note))
                cv.rectangle(colored_img, (x,y), (x+w, y+h), (0, 0, 255), 2)
            elif np.sum(img[cy:cy+3,cx-5:cx+5]) / 255 > 10: # half notes have hole in the middle
                notes.append((cx,cy, "quarter", lr_note))
                cv.rectangle(colored_img, (x,y), (x+w, y+h), (0, 255, 0), 2)
            else:
                notes.append((cx,cy, "half", lr_note))
                cv.rectangle(colored_img, (x,y), (x+w, y+h), (255, 0, 0), 2)

            lr.append(lr_note)

                #notes.append((x + 25, y+new_vertical_offset, "half"))


    # match whole notes
    template = cv.imread('./images/whole_note_template.jpg', 0)
    if template is None:
        template = cv.imread('sheet_music/images/whole_note_template.jpg', 0)
    template = imutils.resize(template, height=int(np.round(est_total_staff_height * 0.7)))
    for x, y, w, h in calc_boxes(template, T_WHOLE_MATCH, whole_note_img):
        cv.rectangle(colored_img, (x,y), (x+w, y+h), (0, 255, 0), 2)
        new_vertical_offset = fix_center(img[y:y+h, x:x+w], h)
        cx = x + (w//2)
        cy = y+new_vertical_offset-(h//4)
        cv.circle(colored_img, (cx,cy), 5, (0, 0, 255), -1)

        lr_note = left_right_diff(img[y:y+h+1, x:x+w+1], w)
        notes.append((cx,cy, "whole", lr_note))

    if show_steps:
        colored_img = draw_staff(colored_img, staff_candidates)
        show_wait_destroy("notes", colored_img)

    # order notes and determine pitch
    ordered_notes = []
    notes.sort(key = lambda note: note[0]) # sort by x position

    lr_mean = np.max(lr)

    thresholds = T_NOTES # notes are not spaced quite linearly

    for staff_index in range(4,len(staff_candidates), 5):
        notes_in_staff = [note for note in notes if note[1] < staff_candidates[staff_index][0] + (staff_space * 3) + staff_height and note[1] > staff_candidates[staff_index][0] - ((staff_space + staff_height)*6)]


        for note in notes_in_staff:
            staff_base = calc_staff(slopes, staff_candidates[staff_index][0], note[0], height(img) // NUM_CHUNKS)
            
            staff_lines = [cand[1][note[0]][0] for cand in staff_candidates[staff_index - 4 : staff_index + 1]]
            for l in range(len(staff_lines) - 1, -1, -1):
                if l > 0:
                    staff_lines.insert(l, staff_lines[l] - (staff_lines[l] - staff_lines[l-1])//2)
            staff_lines.append(staff_lines[-1] + ((staff_lines[-1] - staff_lines[-3])//2))
            staff_lines.reverse()
            print("note x: ", note[0])
            print(staff_lines)
            print(staff_base)
            print()

            closest = 0
            dist = np.abs(staff_lines[0] - note[1])
            c = 0
            for sl in staff_lines:
                if np.abs(sl - note[1]) < dist:
                    closest = c
                    dist = np.abs(sl - note[1])
                elif np.abs(sl - note[1]) == dist and c % 2 == 1:
                    closest = c
                    dist = np.abs(sl - note[1])
                c+=1

            pitch_index = closest + 1
            pitch = note[1]

            if note[3] > lr_mean - 10 and pitch_index == 1:
                pitch_index = 0

            ordered_notes.append((pitch_index, note[2], 0, pitch, note[3]))

    return ordered_notes


if __name__ == "__main__":
    # Read specified image from file
    if (len(sys.argv) < 2 and len(sys.argv) > 3):
        raise RuntimeError("Wrong number of arguments")

    img_file = sys.argv[1]
    show_steps = False
    if len(sys.argv) == 3:
        show_steps = sys.argv[2] == "True" or sys.argv[2] == "true"


    notes = process_sheet_music(img_file, show_steps=show_steps)

    for note in notes:
        print(note)







