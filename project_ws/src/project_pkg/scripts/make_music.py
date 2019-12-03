import rospy
import pickle
from std_msgs.msg import UInt16
from project_pkg.msg import Music, Note
"""
# hot cross buns
song = [['E', 1.0/4, 0],
        ['D', 1.0/4, 0],
        ['C', 1.0/2, 0],
        ['E', 1.0/4, 0],
        ['D', 1.0/4, 0],
        ['C', 1.0/2, 0],
        ['C', 1.0/8, 0],
        ['C', 1.0/8, 0],
        ['C', 1.0/8, 0],
        ['C', 1.0/8, 0],
        ['D', 1.0/8, 0],
        ['D', 1.0/8, 0],
        ['D', 1.0/8, 0],
        ['D', 1.0/8, 0],
        ['E', 1.0/4, 0],
        ['D', 1.0/4, 0],
        ['C', 1.0/2, 0]]
"""

# twinkle twinkle
song = [['C', 1.0/4, 0],
        ['C', 1.0/4, 0],
        ['G', 1.0/4, 0],
        ['G', 1.0/4, 0],
        ['A', 1.0/4, 0],
        ['A', 1.0/4, 0],
        ['G', 1.0/2, 0],
        ['F', 1.0/4, 0],
        ['F', 1.0/4, 0],
        ['E', 1.0/4, 0],
        ['E', 1.0/4, 0],
        ['D', 1.0/4, 0],
        ['D', 1.0/4, 0],
        ['C', 1.0/2, 0],
        ['G', 1.0/4, 0],
        ['G', 1.0/4, 0],
        ['F', 1.0/4, 0],
        ['F', 1.0/4, 0],
        ['E', 1.0/4, 0],
        ['E', 1.0/4, 0],
        ['D', 1.0/2, 0],
        ['G', 1.0/4, 0],
        ['G', 1.0/4, 0],
        ['F', 1.0/4, 0],
        ['F', 1.0/4, 0],
        ['E', 1.0/4, 0],
        ['E', 1.0/4, 0],
        ['D', 1.0/2, 0],
        ['C', 1.0/4, 0],
        ['C', 1.0/4, 0],
        ['G', 1.0/4, 0],
        ['G', 1.0/4, 0],
        ['A', 1.0/4, 0],
        ['A', 1.0/4, 0],
        ['G', 1.0/2, 0],
        ['F', 1.0/4, 0],
        ['F', 1.0/4, 0],
        ['E', 1.0/4, 0],
        ['E', 1.0/4, 0],
        ['D', 1.0/4, 0],
        ['D', 1.0/4, 0],
        ['C', 1.0/2, 0]]

key_to_int = {'C': 0,
              'D': 1,
              'E': 2,
              'F': 3,
              'G': 4,
              'A': 5,
              'B': 6}

lst = []
for n in song:
    lst.append(Note(key_to_int[n[0]], n[1], n[2]))

music = Music(lst)

with open('music/twinkle_twinkle.music', 'wb') as music_file:
    pickle.dump(music, music_file)