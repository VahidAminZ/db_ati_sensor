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
#include <comedilib.hpp>
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
#include <errno.h>
#include <ctime>

#include "ros/ros.h"

const std::string FTSensor::cmdtest_messages_[]=
{
    "success",
    "invalid source",
    "source conflict",
    "invalid argument",
    "argument conflict",
    "invalid chanlist",
};
FTSensor::FTSensor():cmd_(&c), raw_values_(N_CHANS_), voltages_(N_CHANS_), results_(N_CHANS_), first_run_(true), converter_vector_(N_CHANS_)
{
  ft_publisher_ = node_.advertise<std_msgs::Float64MultiArray>("force_torque_values", 1);

  memset(&options_, 0, sizeof(options_));

  options_.filename = "/dev/comedi0";
  options_.subdevice = 0;
  options_.channel = 0;
  options_.range = 0;
  options_.aref = AREF_GROUND;
  options_.n_chan = 16;
  options_.n_scan = 1000;
  options_.freq = 10000.0;
  dev_ = comedi_open(options_.filename.c_str());
  if(!dev_)
  {
    comedi_perror(options_.filename.c_str());
    exit(1);
  }
  // Print numbers for clipped inputs
  comedi_set_global_oor_behavior(COMEDI_OOR_NUMBER);

  /* Set up channel list */
  for(size_t i = 0; i < options_.n_chan; i++){
    chanlist[i] = CR_PACK(options_.channel + i, options_.range, options_.aref);
    range_info[i] = comedi_get_range(dev_, options_.subdevice, options_.channel, options_.range);
    maxdata[i] = comedi_get_maxdata(dev_, options_.subdevice, options_.channel);
  }

  /* prepare_cmd_lib() uses a Comedilib routine to find a
   * good command for the device.  prepare_cmd() explicitly
   * creates a command, which may not work for your device. */
  prepare_cmd_lib(dev_, options_.subdevice, options_.n_scan, options_.n_chan, 1e9 / options_.freq, cmd_);

  /* comedi_command_test() tests a command to see if the
   * trigger sources and arguments are valid for the subdevice.
   * If a trigger source is invalid, it will be logically ANDed
   * with valid values (trigger sources are actually bitmasks),
   * which may or may not result iraw_values_n a valid trigger source.
   * If an argument is invalid, it will be adjusted to the
   * nearest valid value.  In this way, for many commands, you
   * can test it multiple times until it passes.  Typically,
   * if you can't get a valid command in two tests, the original
   * command wasn't specified very well. */
  int ret;
  ret = comedi_command_test(dev_, cmd_);
  if(ret < 0)
  {
    comedi_perror("comedi_command_test");
    if(errno == EIO)
    {
      ROS_ERROR_STREAM(stderr << "Ummm... this subdevice doesn't support commands\n" << std::endl);
    }
    exit(1);
  }
  ret = comedi_command_test(dev_, cmd_);
  if(ret < 0){
    comedi_perror("comedi_command_test");
    exit(1);
  }
  std::cerr << "second test returned "<< ret << std::endl << cmdtest_messages_[ret] << std::endl;
  if(ret!=0){
    ROS_ERROR_STREAM(stderr <<  "Error preparing command\n" << std::endl);
    exit(1);
  }

  int subdev_flags = comedi_get_subdevice_flags(dev_, options_.subdevice);
  if(subdev_flags < 0)
  {
      comedi_perror("comedi_get_subdevice_flags");
      return;
  }

  if(subdev_flags & SDF_SOFT_CALIBRATED) // board uses software calibration
  {
    char *calibration_file_path = comedi_get_default_calibration_path(dev_);
    ROS_INFO_STREAM("calibration file path is " << calibration_file_path << std::endl);

    //parse a calibration file which was produced by the comedi_soft_calibrate program
    soft_calibration_ = comedi_parse_calibration_file(calibration_file_path);
    free(calibration_file_path);
    if(soft_calibration_ == NULL)
    {
        ROS_ERROR_STREAM("calibration_file_path:  " << calibration_file_path << std::endl);
        comedi_perror("comedi_parse_calibration_file");
        return;
    }
  }
}

FTSensor::~FTSensor()
{
  comedi_cleanup_calibration(soft_calibration_);
}

int FTSensor::prepare_cmd_lib(comedi_t *dev, int subdevice, int n_scan, int n_chan, unsigned scan_period_nanosec, comedi_cmd *cmd)
{
  int ret;

  memset(cmd,0,sizeof(*cmd));

  /* This comedilib function will get us a generic timed
   * command for a particular board.  If it returns -1,
   * that's bad. */
  ret = comedi_get_cmd_generic_timed(dev, subdevice, cmd, n_chan, scan_period_nanosec);
  if(ret<0){
    ROS_ERROR_STREAM("comedi_get_cmd_generic_timed failed");
    return ret;
  }

  /* Modify parts of the command */
  cmd->chanlist = chanlist;
  cmd->chanlist_len = n_chan;
  if(cmd->stop_src == TRIG_COUNT) cmd->stop_arg = n_scan;

  return 0;
}

void FTSensor::generate_voltages(std::vector<lsampl_t> raw_vector)
{
  std::vector<double> physical_value(N_CHANS_);

  for (size_t index = 0; index < raw_vector.size(); ++index)
  {
    if (first_run_)
    {
      comedi_get_softcal_converter(options_.subdevice, index, options_.range,
                                                          COMEDI_TO_PHYSICAL, soft_calibration_, &converter_vector_[index]);
    }
    voltages_[index] = comedi_to_physical(raw_vector[index], &converter_vector_[index]);
  }


  if (first_run_)
  {
    char calfilepath[1024];
    std::string a="/home/vahid/projects/de_beers/src/db_ati_sensor/calibration/FT19470.cal";
    strcpy(calfilepath,a.c_str());

    ft_cal_=createCalibration(calfilepath,1);
    SetForceUnits(ft_cal_,(char*) "N");
    SetTorqueUnits(ft_cal_,(char*) "N-mm");
    first_run_ = false;
    float voltages_array[voltages_.size()];
    for (size_t index = 0; index < raw_vector.size(); ++index)
    {
      voltages_array[index] = voltages_[index];
    }
    Bias(ft_cal_, voltages_array);
    ROS_INFO_STREAM("Biased the values for the first run");
  }

  float voltages[N_CHANS_], results[N_CHANS_];
  for (size_t i = 0; i < N_CHANS_; ++i)
  {
    voltages[i] = static_cast<float>(voltages_[i]);
  }
  ConvertToFT(ft_cal_,voltages,results);

  for (size_t i = 0; i < N_CHANS_; ++i)
  {
    results_[i] = static_cast<double>(results[i]);
  }
  topic_.data = results_;
  ft_publisher_.publish(topic_);
}

std::vector<double> FTSensor::read_force_torque_sensor()
{
  int ret;
  int subdev_flags;
  int total;
  lsampl_t raw;

  ret = comedi_command(dev_, cmd_);
  if(ret < 0){
    comedi_perror("comedi_command");
    exit(1);
  }
  subdev_flags = comedi_get_subdevice_flags(dev_, options_.subdevice);
  while(1)
  {
    ret = read(comedi_fileno(dev_),buf,BUFSZ_);
    if(ret < 0)
    {
      /* some error occurred */
      perror("read");
      break;
    }
    else if(ret == 0)
    {
      /* reached stop condition */
      break;
    }
    else
    {
      static int col = 0;
      int bytes_per_sample;
      total += ret;
      if(options_.verbose)
      {
        fprintf(stderr, "read %d %d\n", ret, total);
      }
      if(subdev_flags & SDF_LSAMPL)
      {
        bytes_per_sample = sizeof(lsampl_t);
      }
      else
      {
        bytes_per_sample = sizeof(sampl_t);
      }
      for(size_t i = 0; i < ret / bytes_per_sample; ++i)
      {
        if(subdev_flags & SDF_LSAMPL)
        {
          raw_values_[col]  = ((lsampl_t *)buf)[i];
        } else
        {
          raw_values_[col]  = ((sampl_t *)buf)[i];
        }
        col++;
        if(col == options_.n_chan){
          col=0;
        }
      }
      generate_voltages(raw_values_);
    }
  }

  std::vector<double> force_torques;
  return force_torques;
}

int main(int argc,char *argv[])
{
  ros::init(argc, argv, "FT_node");
  FTSensor ft_sensor;
  ros::Rate rate(100);
  while(ros::ok())
  {
    ft_sensor.read_force_torque_sensor();
    rate.sleep();
  }
}

