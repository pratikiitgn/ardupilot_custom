#!/usr/bin/env python
'''
create alternate function tables, without using tabula

This assumes a csv file generated by STM32CubeMX this way:
"Pinout" (Above the IC view) -> "Export pinout with Alt. Functions"
'''

import sys, csv, os, re
from functools import cmp_to_key

def is_pin(str):
    '''see if a string is a valid pin name'''
    if len(str) < 3:
        return False
    if str[0] != 'P':
        return False
    if str[1] not in "ABCDEFGHIJK":
        return False
    try:
        p = int(str[2:])
        if p < 0 or p > 15:
            return False
        return True
    except ValueError:
        return False

def pin_compare(p1, p2):
    '''control pin sort order'''
    (p1,f1) = p1.split(':')
    (p2,f2) = p2.split(':')
    port1 = p1[:2]
    port2 = p2[:2]
    pin1 = int(p1[2:])
    pin2 = int(p2[2:])
    #print(port1, pin1, port2, pin2)
    if port1 == port2:
        if pin1 == pin2:
            if f1 < f2:
                return -1
            return 1
        if pin1 < pin2:
            return -1
        return 1
    if port1 < port2:
        return -1
    return 1

def parse_af_table(fname, table):
    csvt = csv.reader(open(fname,'r'))
    i = 0
    aflist = []
    row = next(csvt)

    AF_COLUMN = 5  # Columns are: "Position","Name","Type","Signal","Label","AF0","AF1"...

    if len(row) < AF_COLUMN or row[AF_COLUMN] != 'AF0':
        print("Error: This doesn't look like CubeMX pinout csv")
        sys.exit(1)

    for row in csvt:
        pin = re.findall('\w\w\d+', row[1])  # Strip function after pin like 'PC14-OSC32_IN'
        if len(pin) == 0:
            continue
        elif not is_pin(pin[0]):
            continue
        
        pin = pin[0]
        for af_index, value in enumerate(row[AF_COLUMN:]):
            if len(value) > 0:
                for single_function in value.split('/'):
                    table['{0}:{1}'.format(pin, single_function)] = af_index

# Key: PIN:FUNCTION
# Value: AFNUMBER
table = {}

if len(sys.argv) != 2:
    print("Error: expected 1 CSV file")
    sys.exit(1)

parse_af_table(sys.argv[1], table)

sys.stdout.write("AltFunction_map = {\n");
sys.stdout.write('\t# format is PIN:FUNCTION : AFNUM\n')
sys.stdout.write('\t# extracted from %s\n' % os.path.basename(sys.argv[1]))
for k in sorted(table.keys(), key=cmp_to_key(pin_compare)):
    s = '"' + k + '"'
    sys.stdout.write('\t%-20s\t:\t%s,\n' % (s, table[k]))
sys.stdout.write("}\n");
