#!/usr/bin/env python
import sys
import re

if len(sys.argv) < 2:
	print("Not enough or too many parameters! ({})".format(len(sys.argv)))
	sys.exit(1)
	
if len(sys.argv) > 2:
	fw = open(sys.argv[2], 'w')
else:
	fw = open(sys.argv[1]+"-noranges.txt", 'w')
	
if len(sys.argv) > 3:
	fwr = open(sys.argv[3], 'w')
else:
	fwr = open(sys.argv[1]+"-ranges.txt", 'w')

#patternfull = r"[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+\t[^\t]+"
patterns = re.compile(r"(.*)\t([^\t]+)")

i = 0

try:
	with open(sys.argv[1],'r') as f:
		for x in f:
			x = x.rstrip()
			if not x:
				continue
			matches = patterns.match(x)
			if matches:
				i = i+1
				fw.write(matches.group(1) + '\n')
				if matches.group(2) and matches.group(2) != 'NaN':
					splitted = matches.group(2).split('|')
					for line in splitted:
						if line != '':
							fwr.write(line+'\n')
			else:
				print "No match: "+x+"\n"
except IOError as e:
	print "Error opening datafile!"
	print "I/O error({0}): {1}".format(e.errno, e.strerror)
	fw.close()
	fwr.close()
	sys.exit(2)
				
fw.close()
fwr.close()

print "Done %d lines!" % i