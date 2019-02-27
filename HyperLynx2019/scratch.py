import os
import re
import datetime

date = datetime.datetime.today()
new_number = str(date.year) + str(date.month) + str(date.day)\
             + str(date.hour) + str(date.minute) + str(date.second)
file_name = 'output'+new_number

column_separator = "\t"
columns = [
    "State",
    "Sensor",
    "Value"
]
file = open(file_name, 'a')
with file:
    file.write(column_separator.join(map(lambda column_title: "\"" + column_title + "\"", columns)))
    file.write("\n")