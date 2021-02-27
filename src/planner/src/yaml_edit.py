import yaml
import sys
import pdb

# argv[1]: yaml file path
# argv[2]: map file path
argv = sys.argv

with open(argv[1], 'r') as fin:
    file = yaml.safe_load(fin)

if '/' in argv[2]:
    argv[2] = argv[2].split('/')[-1]

if 'com' in argv[2]:
    file['resolution'] = 0.02
    file['occupied_thresh'] = 0.95
else:
    file['resolution'] = 0.05
    file['occupied_thresh'] = 0.65

file['image'] = argv[2]

with open(argv[1], 'w') as fout:
    yaml.safe_dump(file, fout)
