import cv2 as cv
import sys
import numpy as np
import imutils

from preprocessing import preprocess
from staff import remove_staff
from extensions import height, width, draw_staff_lines, show_wait_destroy
from template_matching import match_template, is_duplicate
from note import Note


if __name__ == "__main__":
    # Read specified image from file
    img_file = sys.argv[1]
    img = cv.imread(img_file, 0)
    if (img is None):
        raise RuntimeError("Image not found.")
    img = cv.resize(img, (1920, 1080), interpolation = cv.INTER_AREA)

    # display original image
    show_wait_destroy("Original image", img)

    # Preprocessing
    preprocessed_img, staff_segments = preprocess(img)
    show_wait_destroy("Preprocessed", preprocessed_img)
    show_wait_destroy("Staff segments", staff_segments)

    # remove staff

    #img_no_staff, staff = remove_staff(preprocessed_img)
    #_, staff = remove_staff(staff_segments, preprocessed_img)
    #print(staff)
    #show_wait_destroy("Staff removed", img_no_staff)
    

    # redraw staff
    #img_colored_staff_base = draw_staff_lines(img_no_staff, staff)
    #show_wait_destroy("Staff lines drawn in", img_colored_staff_base)

    #img_colored_staff_base = draw_staff_lines(preprocessed_img, staff)
    #show_wait_destroy("Staff lines drawn in", img_colored_staff_base)

    # determine notes via template matching
    #notes = match_template(img_no_staff, './images/quarter_note_template.jpg', (4, 23), match_threshold=.70, scales=[1.0])
    #for note in match_template(img_no_staff, './images/quarter_note_flipped_template.jpg', (4, 4), match_threshold=.80, scales=[1.0]):
    #    if not is_duplicate(note.centerx, note.centery, [n.centerx for n in notes], [n.centery for n in notes]):
    #        notes.append(note)

    #if len(notes) == 0:
    #    raise RuntimeError("No notes found.")

    # draw and display notes
    #img_colored = cv.cvtColor(img, cv.COLOR_GRAY2RGB)
    #for note in notes:
    #    cv.circle(img_colored, (note.centerx, note.centery), 2, (0, 255, 0), -1)
    #    cv.rectangle(img_colored, (note.box[0], note.box[1]), (note.box[2], note.box[3]), (0, 0, 255), 1) 
    #show_wait_destroy("template_matched", img_colored)

    # calculate note pitch
    #for note in notes:
    #    note.pitch = staff.calculate_pitch(note)
    #    if note.pitch is None:
    #        notes.remove(note)
    
    #notes = Note.order_notes(notes, staff)
    #print([n.pitch for n in notes])

    #cv.imwrite('./images/img_colored_staffs.jpg', img_colored_staff_base)
