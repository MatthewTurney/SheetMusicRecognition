import cv2 as cv
import imutils
import numpy as np
import MTM

def calc_boxes(template, threshold, img):
    try:
        hits = MTM.matchTemplates([("staff", template)], img, method=cv.TM_CCOEFF_NORMED, 
            N_object=float("inf"), score_threshold=threshold, maxOverlap=0.2, searchBox=None)
        return hits["BBox"]
    except KeyError as ke:
        print(str(ke))
        return []

def remove_template_matches(img, template, threshold):
    matches = []
    for x, y, w, h in calc_boxes(template, threshold, img):
        cv.rectangle(img, (x,y), (x+w, y+h), (0, 0, 0), cv.FILLED)
        matches.append((x + (w//2), y + (h//2)))
    return img, matches

def remove_template_matches_with_buffer(img, template, threshold, b):
    matches = []
    for x, y, w, h in calc_boxes(template, threshold, img):
        cv.rectangle(img, (x+b,y), (x+w-b, y+h), (0, 0, 0), cv.FILLED)
        matches.append((x + (w//2), y + (h//2)))
    return img, matches