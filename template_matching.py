import cv2 as cv
import imutils
import numpy as np
import MTM

def calc_boxes(template, threshold, img):
    try:
        hits = MTM.matchTemplates([("staff", template)], img, method=cv.TM_CCOEFF_NORMED, 
            N_object=float("inf"), score_threshold=threshold, maxOverlap=0.25, searchBox=None)
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

# def match_template(img, template, center_offset, match_threshold=.85, scales=[1.0]):
#     # read template
#     template = cv.imread(template,0)
   
#     image = img.copy()
#     found = []
#     img_gray = binary_threshold(image)

#     for scale in scales:#np.linspace(0.6, 1.2, 20)[::-1]: 
#         resized = imutils.resize(img_gray, width = int(img_gray.shape[1] * scale)) 
#         r = img_gray.shape[1] / float(resized.shape[1]) 
    
#         result = cv.matchTemplate(resized, template, cv.TM_CCOEFF_NORMED) 

#         locs = np.where(result > match_threshold)
#         found.extend(zip(locs[1], locs[0], [r] * len(locs[0])))

#         if resized.shape[0] < height(template) or resized.shape[1] < width(template): 
#                 break
    
#     # draw a bounding box around the detected result and display the image 

#     matches = []
#     for tup in found:
#         (x, y, r) = tup
#         (startX, startY) = (int(x * r), int(y * r)) 
#         (endX, endY) = (int((x + width(template)) * r), int((y + height(template)) * r))
        
#         if len(matches) == 0 or not is_duplicate(x, y, [m[0] for m in matches], [m[1] for m in matches]):
#             matches.append((startX, startY, endX, endY))

#     res = [Note(x + center_offset[0], y + center_offset[1], x, y, ex, ey) for x, y, ex, ey in matches]
#     return res

# def is_duplicate(x, y, xs, ys):
#     for xm, ym in zip(xs, ys):
#         if np.abs(x - xm) < 5 and np.abs(y - ym) < 20:
#             return True
#     return False