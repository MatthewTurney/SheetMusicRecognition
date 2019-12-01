import cv2 as cv
import sys
import numpy as np
import imutils
import MTM

from preprocessing import binary_threshold
from staff import remove_staff
from extensions import height, width, draw_staff_lines, show_wait_destroy
from template_matching import match_template, is_duplicate
from note import Note
    
def calc_boxes(template, threshold, img):
    try:
        hits = MTM.matchTemplates([("staff", template)], img, method=cv.TM_CCOEFF_NORMED, 
            N_object=float("inf"), score_threshold=threshold, maxOverlap=0.25, searchBox=None)
        return hits["BBox"]
    except KeyError as ke:
        print(str(ke))
        return []

def remove_isolated_pixels(image):
    connectivity = 8

    output = cv.connectedComponentsWithStats(image, connectivity, cv.CV_32S)

    num_stats = output[0]
    labels = output[1]
    stats = output[2]

    new_image = image.copy()

    for label in range(num_stats):
        if stats[label,cv.CC_STAT_AREA] < 20:
            new_image[labels == label] = 0

    return new_image

if __name__ == "__main__":
    staff_removed = cv.imread("./images/hcb_staff_removed.jpg", 0)
    est_total_staff_height = 340 - 260
    colored = cv.cvtColor(staff_removed.copy(), cv.COLOR_GRAY2RGB)
    cv.rectangle(colored, (0,260), (200, 340), (0, 255, 0), cv.FILLED)
    show_wait_destroy("color", colored) 

    #staff_removed = cv.imread("./images/alphabet_song_staff_removed.jpg", 0)

    if (staff_removed is None):
        raise RuntimeError("couldn't load image")

    # show original
    img = (binary_threshold(staff_removed)).astype(np.uint8)
    img = remove_isolated_pixels(img)
    show_wait_destroy("staff removed", img)

    # remove staff thingys
    template = cv.imread('./images/staff_template.jpg', 0)
    template = imutils.resize(template, height=int(est_total_staff_height * 2))
    for x, y, w, h in calc_boxes(template, 0.5, img):
        cv.rectangle(img, (x,y), (x+w, y+h), (0, 0, 0), cv.FILLED)

    # find bar lines
    bars = []
    template = cv.imread('./images/bar_template.jpg', 0)  
    for x, y, w, h in calc_boxes(template, 0.7, img):
        cv.rectangle(img, (x,y), (x+w, y+h), (0, 0, 0), cv.FILLED)
        bars.append((x + (w//2), y + (h//2)))

    # remove 4/4 time and end markers
    end = None
    template = cv.imread('./images/end.jpg', 0)
    for x, y, w, h in calc_boxes(template, 0.5, img):
        cv.rectangle(img, (x,y), (x+w, y+h), (0, 0, 0), cv.FILLED)
        end = (x + (w//2), y + (h//2))

    template = cv.imread('./images/time_template.jpg', 0)
    for x, y, w, h in calc_boxes(template, 0.5, img):
        cv.rectangle(img, (x,y), (x+w, y+h), (0, 0, 0), cv.FILLED)

    show_wait_destroy("processed", img)


    template = cv.imread('./images/vertical_quarter_template.jpg', 0)
    colored_img = cv.cvtColor(img, cv.COLOR_GRAY2RGB)

    #boxes = MTM.drawBoxesOnRGB(colored_img, hits, boxThickness=2, 
    #    boxColor=(0, 255, 0), showLabel=False, labelColor=(0, 255, 0), labelScale=0.5)

    whole_note_img = img.copy()
    notes = []
    for x, y, w, h in calc_boxes(template, 0.25, img):
        if (np.sum(img[y:y+h+1, x:x+w+1]) / 255 > 300) and \
            (np.sum(img[y:y+(h//2)+1, x:x+w+1]) < np.sum(img[y+(h//2):y+h+1, x:x+w+1])) and \
            (np.sum(img[y:y+(h//2)+1, x:x+w+1]) != 0) and \
            (np.sum(img[y+(h//2):y+h+1, x:x+w+1]) != 0):
            cv.rectangle(colored_img, (x,y), (x+w, y+h), (0, 255, 0), 2)
            cv.rectangle(whole_note_img, (x,y), (x+w, y+h), (0, 0, 0), -1)
            cv.circle(colored_img, (x+25,y+85), 5, (0, 0, 255), -1)
            if np.sum(img[y+80:y+90,x+20:x+30]) / 255 > 85:
                notes.append((x+25,y+85, "quarter"))
            else:
                notes.append((x+25,y+85, "half"))
            #print("note at: ", x+25, y+85, np.sum(img[y+80:y+90,x+20:x+30]) / 255)

    template = cv.imread('./images/whole_note_template.jpg', 0)
    for x, y, w, h in calc_boxes(template, 0.5, whole_note_img):
        cv.rectangle(colored_img, (x,y), (x+w, y+h), (0, 255, 0), 2)
        cv.circle(colored_img, (x+45,y+28), 5, (0, 0, 255), -1)
        notes.append((x+25,y+85, "whole"))


    show_wait_destroy("notes", colored_img)

    ordered_notes = []
    notes.sort(key = lambda note: note[0])
    #left = 0
    #right = 0
    #staff = 0
    #for bar in bars:
        #right = bar[0]
        #left = right
        #if (right < left):
            #left = 0

        #notes_in_measure = [note for note in notes if note[0] > left and note[0] < right and note[1] > staff_candidates[staff] and note[1] < staff_candidates[staff] + (staff_height * 5) + (staff_gap * 8)]
    #for 
    
    staff_height = 3
    staff_gap = 10
    staff_candidates = [300, 0, 0, 0, 0, 500, 0, 0, 0, 0, 800, 0, 0, 0, 0,]
    for staff in range(0,len(staff_candidates), 5):
        notes_in_staff = [note for note in notes if note[1] > staff_candidates[staff] and note[1] < staff_candidates[staff] + 150]#(staff_height * 5) + (staff_gap * 8)]
        ordered_notes.extend(notes_in_staff)

    for note in ordered_notes:
        print(note)