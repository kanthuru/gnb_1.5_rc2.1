#!/usr/bin/python
#
# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) 2020 EdgeQ Inc
#
# This file takes e32 fw file and convert it to a header file with an array
# of addr/data pair.
#
import sys

if len(sys.argv) != 3:
    print("Usage: python conv_b0_e32_tcl_to_array.py <in_file> <out_file>")
    sys.exit()
 
infile_name = sys.argv[1]
outfile_name = sys.argv[2]
print("infile:  %s" % (infile_name))
print("outfile: %s" % (outfile_name))

outfile = open(outfile_name, "w+")
outfile.write("// Converted from " + infile_name + " by " + sys.argv[0] + "\n")
outfile.write("static u16 image_1g_2g5_10g_25g" + "[][2] = {\n")

infile = open(infile_name, "r")
line_cnt = 0

for line in infile:
    # Skip empty lines
    if len(line) <= 1:
        continue;
    # Skip comments
    if line[0] == '/' and line[1] == '/':
        continue

#    print line.strip()
    res = line.split()
    outfile.write("\t{" + res[1] + ", " + res[2] + "},\n")
    line_cnt += 1

outfile.write("};\n")

print("Total lines converted: %d" % (line_cnt))
infile.close()
outfile.close()
