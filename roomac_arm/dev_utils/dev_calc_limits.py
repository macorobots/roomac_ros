#!/usr/bin/env python

#  Software License Agreement
#
#  Copyright (C) 2022, Maciej Stępień, All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#  Authors: Maciej Stępień

import math

zero_positions = [850, 320, 512, 1770, 1500, 650]

analog_lower_signal_bound = 500
analog_upper_signal_bound = 2500
digital_lower_signal_bound = 0
digital_upper_signal_bound = 1024

lower_signal_bound = [
    digital_lower_signal_bound,
    digital_lower_signal_bound,
    digital_lower_signal_bound,
    analog_lower_signal_bound,
    analog_lower_signal_bound,
    analog_lower_signal_bound,
]
upper_signal_bound = [
    digital_upper_signal_bound,
    digital_upper_signal_bound,
    digital_upper_signal_bound,
    analog_upper_signal_bound,
    analog_upper_signal_bound,
    analog_upper_signal_bound,
]

digital_scale_factor = 1024 / ((330.0 / 2.0) * math.pi / 180.0)
analog_scale_factor = 2000 / (180.0 * math.pi / 180.0)

scaling_factors = [
    digital_scale_factor,
    digital_scale_factor,
    digital_scale_factor / 2.0,
    analog_scale_factor,
    analog_scale_factor,
    analog_scale_factor,
]


for i in range(0, 6):
    lower_angle = (lower_signal_bound[i] - zero_positions[i]) / scaling_factors[i]
    upper_angle = (upper_signal_bound[i] - zero_positions[i]) / scaling_factors[i]
    print(str(i) + " " + str(lower_angle) + " " + str(upper_angle))
