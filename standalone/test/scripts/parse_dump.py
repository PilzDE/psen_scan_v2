#!/usr/bin/env python3
# Copyright (c) 2021 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

""" Dump parser
This is a simple parser to transofrm a wireshark dump of the shape 

0000 6f 10 20 f3 6f 10 20 f3 6f 10 20 f3 6f 10 20 f3
...

to the format used in the udp_frame_dumps.h

0x6f, 0x10, ..... // 0030
...

The output has to be corrected by:
- truncating the header bits at the start
- inputing the desired diagnostic bits
- nullifying the physical input bits.

Additionally this script outputs the locations that fit the additional header IDs and sizes, since they are needed in the same header file
for the deserialization.

To use this script create a "dump" file next to this with the wireshark dump and then execute this script.

"""

from pathlib import Path
from collections import namedtuple


DValue = namedtuple("DValue", ["line", "column", "value"])


def d_values(lines):
    for l, line in enumerate(lines):
        line, data = line.split('   ')
        for c, dp in enumerate(data.rstrip().split(' ')):
            yield DValue(l, c, dp)


def print_line_end(line_index):
    line_in_hex = hex(line_index*1).split('x')[1].rjust(3, "0")
    print(f" // {line_in_hex}0")


def print_formated_dump(lines):
    last_line = 0
    for d in d_values(lines):
        if d.line != last_line:
            print_line_end(last_line)
        print(f"0x{d.value}, ", end="")
        last_line = d.line
    print_line_end(last_line)


HEADER_ID = {
    "01": "IO State",
    "02": "Scan Counter",
    "03": "Zone Set",
    "04": "Diagnostics",
    "05": "Measurements",
    "06": "Intensities",
    "09": "End of Frame",
}


EXPECTED_LENGHT = {
    "01": 63,
    "02": 5,
    "03": 2,
    "04": 41,
    "09": 0
}

PPos = namedtuple("PPos", ["pos", "len"])

def print_positions(lines):
    possible_field_positions = {"01": [], "02": [], "03": [], "04": [], "05": [], "06": [], "09": []}
    last_ones = []
    for d in d_values(lines):
        last_ones.append(d)
        if len(last_ones) == 3:
            id_ = last_ones[0].value
            len_hex_ = "0x" + last_ones[2].value + last_ones[1].value
            if id_ in HEADER_ID.keys():
                pos_ = last_ones[0].line * 16 + last_ones[0].column
                len_ = int(len_hex_, 16)
                if not id_ in EXPECTED_LENGHT or EXPECTED_LENGHT[id_] == len_:
                    possible_field_positions[id_].append(PPos(pos_, len_ - 1))
            last_ones.pop(0)

    pos_measurements = []
    pos_intensities = []
    for mp in possible_field_positions["05"]:
        for ip in possible_field_positions["06"]:
            if mp.len == ip.len:
                pos_measurements.append(mp)
                pos_intensities.append(ip)
    possible_field_positions["05"] = pos_measurements
    possible_field_positions["06"] = pos_intensities

    print("Possible Field locations:")
    for k, v in possible_field_positions.items():
        print(f"{HEADER_ID[k]}: {v}")


if __name__ == "__main__":
    with open(Path(__file__).parent / 'dump', 'r') as fh:
        lines = fh.readlines()
        print_formated_dump(lines)
        print("")
        print_positions(lines)


