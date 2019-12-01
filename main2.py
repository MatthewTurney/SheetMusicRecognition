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
T_STAFF_MATCH = 0.5
T_BAR_MATCH = 0.7
T_END_MATCH = 0.5
T_TIME_MATCH = 0.5

if __name__ == "__main__":
    # Read specified image from file
    img = read_image_from_args()

    # resize image
    img = cv.resize(img, IMAGE_SIZE, interpolation = cv.INTER_AREA)

    # display original image
    show_wait_destroy("Original Image", img)

    # binary threshold
    img = (cv.bitwise_not(binary_threshold(img))).astype(np.uint8)
    show_wait_destroy("Binary Threshold", img)

    # get rid of text, markings, etc
    img = remove_text(img)
    show_wait_destroy("Text Removed", img)

    # save preprocessed original image for later use
    processed_img = img.copy()

    # create initial staff image
    staff_height, staff_space, T_length = estimate_staff_stats(img)
    img = create_initial_staff_image(img, T_length)
    show_wait_destroy("Initial Staff Image", img)

    # model line shape
    colored_img, lines = find_lines(img)
    show_wait_destroy("lines", colored_img)

    # calculate average staff line slope at every x pos
    slopes, slope_img = calc_slopes(img, NUM_CHUNKS, lines)
    show_wait_destroy("Average staff line orientation", slope_img)

    # find staff and display candidates
    staff_candidates = find_staff_candidates(img, slopes, T_STAFF_CAND, T_length, NUM_CHUNKS, staff_height, staff_space)

    colored_img = cv.cvtColor(img.copy(), cv.COLOR_GRAY2RGB)
    for cand in staff_candidates:
        colored_img = draw_staff_line(colored_img, cand[0], cand[1])
    show_wait_destroy("Staff candidates", colored_img)

    # remove staff from original (text removed) image
    staff_removed_img = remove_staff(processed_img.copy(), staff_candidates, T_length)
    show_wait_destroy("Staff removed", staff_removed_img)


    est_total_staff_height = (4 * staff_space) + (5 * staff_height) #80
    colored = cv.cvtColor(staff_removed_img.copy(), cv.COLOR_GRAY2RGB)
    #cv.rectangle(colored, (0,260), (200, 340), (0, 255, 0), cv.FILLED)
    #show_wait_destroy("color", colored) 

    # remove isolated pixels
    img = remove_isolated_pixels(staff_removed_img)
    show_wait_destroy("isolated pixels removed", img)

    # remove staff thingys
    template = cv.imread('./images/staff_template.jpg', 0)
    template = imutils.resize(template, height=int(est_total_staff_height * 2))
    img, _ = remove_template_matches(img, template, T_STAFF_MATCH)

    # find bar lines
    template = cv.imread('./images/bar_template.jpg', 0)  
    img, bars = remove_template_matches(img, template, T_BAR_MATCH)

    # remove 4/4 time and end markers
    template = cv.imread('./images/end.jpg', 0)
    img, end = remove_template_matches(img, template, T_END_MATCH)
    if (len(end) > 0):
        end = end[0]

    template = cv.imread('./images/time_template.jpg', 0)
    img, _ = remove_template_matches(img, template, T_TIME_MATCH)

    show_wait_destroy("Removed extraneous markings", img)

    # mach quarter/half notes first
    template = cv.imread('./images/vertical_quarter_template.jpg', 0)
    whole_note_img = img.copy()
    notes = []
    colored_img = cv.cvtColor(img, cv.COLOR_GRAY2RGB)

    for x, y, w, h in calc_boxes(template, 0.25, img):
        if (np.sum(img[y:y+h+1, x:x+w+1]) / 255 > 300) and \
            (np.sum(img[y:y+(h//2)+1, x:x+w+1]) < np.sum(img[y+(h//2):y+h+1, x:x+w+1])) and \
            (np.sum(img[y:y+(h//2)+1, x:x+w+1]) != 0) and \
            (np.sum(img[y+(h//2):y+h+1, x:x+w+1]) != 0):
            cv.rectangle(colored_img, (x,y), (x+w, y+h), (0, 255, 0), 2)
            cv.rectangle(whole_note_img, (x,y), (x+w, y+h), (0, 0, 0), -1) # black out quarter/half notes for whole note detection
            cv.circle(colored_img, (x+25,y+85), 5, (0, 0, 255), -1)
            if np.sum(img[y+80:y+90,x+20:x+30]) / 255 > 85: # half notes have whole in the middle
                notes.append((x+25,y+85, "quarter"))
            else:
                notes.append((x+25,y+85, "half"))

    template = cv.imread('./images/whole_note_template.jpg', 0)
    for x, y, w, h in calc_boxes(template, 0.5, whole_note_img):
        cv.rectangle(colored_img, (x,y), (x+w, y+h), (0, 255, 0), 2)
        cv.circle(colored_img, (x+45,y+28), 5, (0, 0, 255), -1)
        notes.append((x+45,y+28, "whole"))

    show_wait_destroy("notes", colored_img)

    # order notes and determine pitch
    ordered_notes = []
    notes.sort(key = lambda note: note[0]) # sort by x position

    thresholds = [-1.8, -0.8, 0.5, 1.8, 2.5] # notes are not spaced quite linearly

    for staff_index in range(4,len(staff_candidates), 5):
        notes_in_staff = [note for note in notes if note[1] < staff_candidates[staff_index][0] + (staff_space * 2) + staff_height and note[1] > staff_candidates[staff_index][0] - ((staff_space + staff_height)*5)]
        for note in notes_in_staff:
            staff_base = calc_staff(slopes, staff_candidates[staff_index][0], note[0], height(img) // NUM_CHUNKS)
            #dist_above = (staff + staff_space) - note[1]
            #print(staff, staff_candidates[staff_index][0], dist_above, note[2])
            #print()
            #print("Note y: ", note[1])
            #print("Staff base: ", staff_base)
            pitch = ((staff_base - note[1]) / (staff_space + staff_height))*2.0
            pitch_index = 0
            while(pitch_index < len(thresholds) - 1 and pitch >= thresholds[pitch_index]):
                pitch_index+=1

            ordered_notes.append((pitch_index, note[2], 0, pitch))


    for note in ordered_notes:
        print(note)

# OUTPUT
# pitch_index: integer starting at 0 = middle C
# type = {"quarter", "half", "whole"}
# delta = 0 (no rests rn)
# pitch = for debugging purposes only (raw pitch value)






