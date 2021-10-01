#!/bin/bash
# usage: ./record_udp_pakets.sh interface duration_in_sec udp_port_range_start udp_port_range_end

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
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

SCAN_COUNTER_POSITION="49"
FROM_THETA_POSITION="35"

OUTPUT_FORMAT_STR='udp_port: %s time_epoch: %s scan_counter: %s from_theta: %s'

tshark -i $1 -a duration:$2 -f 'udp portrange '$3'-'$4 -q -T fields -e udp.dstport -e frame.time_epoch -e data.data \
    | awk -v out="$OUTPUT_FORMAT_STR" -v scan_counter_pos="$SCAN_COUNTER_POSITION" -v from_theta_pos="$FROM_THETA_POSITION" \
    '{ printf out"\n", $1, $2, substr($3, scan_counter_pos, 8), substr($3, from_theta_pos, 4) }'
