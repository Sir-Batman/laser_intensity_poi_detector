/*
 *   Detects POI's by their relative intensity, assuming it is higher than the background values.
 *   Copyright (C) 2016  Connor Yates
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/console.h"
//#include <cmath>
//#include <vector>

/* Function: poiCallback
 * Description: Takes in a laserScan message, which it then searches for the series of the 
 *		four points with the max sum of their intensities, to try to find the POI's.
 *		This information is currently unused, but will eventually be converted to a
 *		POI message format and broadcaster.
 * Preconditions: the /laserScan topic is up and running, and this node has successfully
 * 		subscribed and attached this function as the callback for it.
 * Postconditions: Currently, nothing is set. Calculations are done and then forgotten.
 */ 
void poiCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	// Calculate the size of the intensities array based on the angle measurements 
	int size = (int)((scan->angle_max - scan->angle_min) / scan->angle_increment);

	// Find the maximum intensity out of all of the intensities 
	float max = 0;
	int max_i = 0;
	float temp = 0;
	float range_max = scan->range_max;
	float range_min = scan->range_min;
	for (int i = 0; i < size - 4; ++i)
	{
		// Only consider the sequence of four points to be a possible POI if
		// the next four points are within the sensor's reported min and max distances,
		// to eliminate known erroneous values
		if (!(scan->ranges[i] < range_min || scan->ranges[i] > range_max
					|| scan->ranges[i+1] < range_min || scan->ranges[i+1] > range_max
					|| scan->ranges[i+2] < range_min || scan->ranges[i+2] > range_max
					|| scan->ranges[i+3] < range_min || scan->ranges[i+3] > range_max ))
		{
			// calculate a sum of this point and its next three neighbors, and use that to compare
			temp =    scan->intensities[i]  
				+ scan->intensities[i+1] 
				+ scan->intensities[i+2] 
				+ scan->intensities[i+3] ;
			// 90 is currently an arbitrary filter tolerance, which hopes to make sure that
			// only POI's are detected (with their high intensity) and filter out background noise
			if (temp > max && temp > 90)
			{
				max_i = i;
				max = temp;
			}
		}

	}
	// If a max was found that met our criteria, print to stdout
	if (max != 0) { std::cout << "intensity["<< max_i << "]: " << max << std::endl; }
}

/* Function: main
 * Notes: Currently is a very simple ROS subscriber, which subscribes to the /laserScan topic to retrieve
 * 	the raw lidar data. The poiCallback function is attached as the callback function for this topic.
 */
int main(int argc, char **argv)
{
	std::cout << "Starting laser_intensity_poi_node..." << std::endl;
	ros::init(argc, argv, "laser_intensity_poi_node");
	ros::NodeHandle sub_handle;

	ros::Subscriber sub = sub_handle.subscribe("/laserScan", 10, &poiCallback);
	
	ros::spin();
	return 0;
}
