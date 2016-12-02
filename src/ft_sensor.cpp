/*
 * ft_sensor.cpp
 *
 *  Created on: 2 Dec 2016
 *      Author: vahid
 */

#include "db_ati_sensor/ft_sensor.h"
#include <stdio.h>  /* for printf() */
#include <stdlib.h>
#include <unistd.h>

#include <math.h>
#include <iostream>

#include <string.h>

#ifdef ROS
  #include <ros/ros.h>
  #include <geometry_msgs/WrenchStamped.h>
  #include <ros/package.h>
  #include <kcl_ftsensor/KCL_Sensor_Bias.h>
#endif

#include <comedilib.h>
#include <comedi.h>
#include <db_ati_sensor/sensor.h>
#include <db_ati_sensor/ftconfig.h>
#include "db_ati_sensor/daq_sensor.h"
#include <ctime>
const std::string FTSensor::cmdtest_messages[]={
    "success",
    "invalid source",
    "source conflict",
    "invalid argument",
    "argument conflict",
    "invalid chanlist",
  };
FTSensor::FTSensor() {
  // TODO Auto-generated constructor stub


}

FTSensor::~FTSensor() {
  // TODO Auto-generated destructor stub
}

int FTSensor::prepare_cmd_lib(comedi_t *dev, int subdevice, int n_scan, int n_chan, unsigned period_nanosec, comedi_cmd *cmd)
{
  return 0;
}

void FTSensor::do_cmd(comedi_t *dev,comedi_cmd *cmd)
{
}

void FTSensor::print_datum(lsampl_t raw, int channel_index)
{
}

std::vector<double> FTSensor::read_force_torque_sensor()
{
  std::vector<double> force_torques;
  return force_torques;
}
