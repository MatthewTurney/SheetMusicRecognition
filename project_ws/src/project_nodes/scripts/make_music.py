import rospy
import json
from std_msgs.msg import UInt16
from project_nodes.msg import Music, Note

song = [['E', 1/4, 0],
        ['D', 1/4, 0],
        ['C', 1/2, 0],
        ['E', 1/4, 0],
        ['D', 1/4, 0],
        ['C', 1/2, 0],
        ['C', 1/8, 0],
        ['C', 1/8, 0],
        ['C', 1/8, 0],
        ['C', 1/8, 0],
        ['D', 1/8, 0],
        ['D', 1/8, 0],
        ['D', 1/8, 0],
        ['D', 1/8, 0],
        ['E', 1/4, 0],
        ['D', 1/4, 0],
        ['C', 1/2, 0]]

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
json.dump(music, "music.json")