#!/usr/bin/env python

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
