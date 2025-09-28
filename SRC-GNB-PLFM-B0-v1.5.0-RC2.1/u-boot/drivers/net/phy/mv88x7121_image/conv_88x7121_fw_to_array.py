#!/usr/bin/python
#
# This file takes 88x7121 fw file and convert it to a header file with an array
# of addr/data pair.
#
import sys

if len(sys.argv) != 2:
    print "Usage: python conv_88x7121_fw_to_array.py <fw_file>"
    sys.exit()
 
infile_name = sys.argv[1]
outfile_name = "88x7121_fw.h"
print "infile:  %s" % (infile_name)
print "outfile: %s" % (outfile_name)

outfile = open(outfile_name, "w+")
outfile.write("// Converted from " + sys.argv[1] + " by " + sys.argv[0] + "\n")
outfile.write("static u8 x7121_fw_image" + "[] = {")

infile = open(infile_name, "r")
line_cnt = 0

for line in infile:
    #print(line)
    if (line_cnt % 8) == 0:
        outfile.write("\n\t")
    s = line.split(' ', 3)
    #print(s[1])
    outfile.write("0x" + s[1] + ", ")
    line_cnt += 1

outfile.write("};\n")

print "Total lines converted: %d" % (line_cnt)
infile.close()
outfile.close()
