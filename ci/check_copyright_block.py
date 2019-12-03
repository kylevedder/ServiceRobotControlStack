#!/usr/bin/env python3
from datetime import datetime
import re
import sys
file_path = sys.argv[1]
n_lines = open(file_path, 'r').readlines()
email_address = open('.git/copyrightemail', 'r').readlines()[0].strip()

if not email_address in "\n".join(n_lines[:5]):
    print("{}: Your email, {}, is not found in the Copyright header!".format(file_path, email_address))
    exit(-1)

date_idx = 0
if n_lines[date_idx].strip() == "#pragma once":
    date_idx += 1


def fix_date(line):
    if str(datetime.today().year) in line:
        return line
    date_string = line
    range_match = re.match(r'.*([1-3][0-9]{3})( )*-( )*([1-3][0-9]{3})', date_string)
    if range_match is not None:
        date_string = date_string.replace(str(range_match.group(4)), str(datetime.today().year))
    else:
        single_match = re.match(r'.*([1-3][0-9]{3})', date_string)
        if single_match is not None:
            date_string = date_string.replace(str(single_match.group(1)), str(single_match.group(1)) + ' - ' + str(datetime.today().year))
    return date_string

old_date_line = n_lines[date_idx]
new_date_line = fix_date(old_date_line)

if old_date_line != new_date_line:
    print("Updated date!")
    print("Old date line:", old_date_line)
    print("New date line:", new_date_line)
    n_lines[date_idx] = new_date_line
    f = open(file_path, 'w')
    f.writelines(n_lines)
    f.close()

