import re
import sys
import time
import pdb

# argv[1]: world file path
# argv[2]: map bit file
# argv[3]: turtle pose
argv = sys.argv

with open(argv[1], 'r') as fin:
    file = fin.read()

file = file.replace('\n', '\t')
turtle = re.findall("turtlebot\t\(.*\)", file)[0]

turtle = re.sub("pose.*\]\t", "pose " + argv[3] + '\t', turtle)

file = re.sub("turtlebot\t\(.*\)", turtle, file)
if 'com1' in argv[2]:
    content = "floorplan\t(\tname \"maze\"\tbitmap \"../com1.jpg\"\tsize [ 50.0 19.66 2.0 ]\tpose [  25.0  9.83 0.0 0.0 ]\t)\t"
else:
    content = "floorplan\t(\tname \"maze\"\tbitmap \"{}\"\tsize [ 10.0 10.0 2.0 ]\tpose [ 5.0 5.0 0.0 0.0 ]\t)\t".format(argv[2])

content = content + "# end of floor plan\t"
file = re.sub("floorplan\t\(.*\)\t# end of floor plan\t", content, file)

# map = re.sub("bitmap.*png\"", "bitmap " + "\"" + argv[2] + "\"", map)

# file = re.sub("floorplan\t\(.*\)", map, file)
file = file.replace('\t', "\n")

with open(argv[1], 'w') as fout:
    fout.write(file)
