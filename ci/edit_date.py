#! /usr/bin/env python3

from datetime import datetime
import re
import sys

today = datetime.today()
date_string = sys.argv[1]
range_match = re.match(r'.*([1-3][0-9]{3})( )*-( )*([1-3][0-9]{3})', date_string)
if range_match is not None:
    date_string = date_string.replace(str(range_match.group(4)), str(datetime.today().year))
else:
    single_match = re.match(r'.*([1-3][0-9]{3})', date_string)
    if single_match is not None:
        date_string = date_string.replace(str(single_match.group(1)), str(single_match.group(1)) + ' - ' + str(datetime.today().year))
print(date_string.replace(' ', '\\ ').replace('-','\\-').replace('/','\\/'))
