/*
 * MessageUtil.h
 *
 *  Created on: Nov 28, 2012
 *      Author: mitchell
 */

#ifndef MESSAGEUTIL_H_
#define MESSAGEUTIL_H_

#include "std_msgs/Header.h"

static inline void fill_header_message(std_msgs::Header& header, uint32_t seq){
  header.frame_id = "0";
  header.seq = seq;
  header.stamp = ros::Time::now();
}


#endif /* MESSAGEUTIL_H_ */
