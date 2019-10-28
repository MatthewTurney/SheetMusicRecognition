
class Note():

    def __init__(self, centerx, centery, startx, starty, endx, endy):
        self.centerx = centerx
        self.centery = centery
        self.box = (startx, starty, endx, endy)
        self.pitch = -1

    def order_notes(notes, staff):
        ordered = []
        for base in staff.bases:
            note_list = [n for n in notes if (n.centery < base) and (n.centery > base - ((staff.line_thickness + staff.gap)*5))]
            note_list = sorted(note_list, key=lambda note: note.centerx)
            ordered.extend(note_list)
        return ordered