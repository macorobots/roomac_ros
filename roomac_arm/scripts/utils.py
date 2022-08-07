#!/usr/bin/env python


def calculate_scale_factor(upper_signal, lower_signal, angle_difference):
    return (upper_signal - lower_signal) / angle_difference


def linear_transform_angle_to_dist(a, b, angle):
    return -(a * angle + b)


def linear_transform_dist_to_angle(a, b, dist):
    return (-dist - b) / a
