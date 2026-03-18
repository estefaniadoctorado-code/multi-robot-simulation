import os
import sys

filename = sys.argv[1]

with open(filename) as f:
    content = f.readlines()

for line in content[1:]:
    values = line.split()
    print(
        str(float(values[0])-1.0)+"\t"+\
          values[1]+"\t"+\
          values[2])

