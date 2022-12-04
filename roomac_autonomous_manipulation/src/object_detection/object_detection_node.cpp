/*********************************************************************
 *  Software License Agreement
 *
 *  Copyright (C) 2022, Maciej Stępień, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Maciej Stępień
 *********************************************************************/

#include <ros/ros.h>

#include <roomac_autonomous_manipulation/object_detection/ObjectDetection.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle nh;
  ObjectDetection object_detection;
  ros::spin();
}