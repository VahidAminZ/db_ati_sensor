/*
 * ft_sensor.h
 *
 *  Created on: 2 Dec 2016
 *      Author: vahid
 */

#ifndef DB_ATI_SENSOR_INCLUDE_DB_ATI_SENSOR_FT_SENSOR_H_
#define DB_ATI_SENSOR_INCLUDE_DB_ATI_SENSOR_FT_SENSOR_H_

#include <stdio.h>
#include <comedilib.h>
#include <string>
#include <vector>
#include "db_ati_sensor/ftconfig.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "boost/scoped_ptr.hpp"

class FTSensor {
  struct parsed_options
  {
    char *filename;
    double value;
    int subdevice;
    int channel;
    int aref;
    int range;
    int verbose;
    int n_chan;
    int n_scan;
    double freq;
  };

public:
  FTSensor();
  virtual ~FTSensor();

  std::vector<double> read_force_torque_sensor();
  std::vector<lsampl_t> raw_values_;

private:
bool first_run_;
comedi_t *dev_;
comedi_cmd c,*cmd_;
parsed_options options_;
comedi_calibration_t *soft_calibration_;
comedi_polynomial_t converter_;
std::vector<comedi_polynomial_t> converter_vector_;
std::vector<double> voltages_;
std::vector<double> results_;
ros::NodeHandle node_;
std_msgs::Float64MultiArray topic_;

Calibration *ft_cal_;

ros::Publisher ft_publisher_;




static const std::string cmdtest_messages_[];
static const size_t BUFSZ_ = 100;
char buf[BUFSZ_];

static const size_t N_CHANS_ = 16;

unsigned int chanlist[N_CHANS_];
comedi_range * range_info[N_CHANS_];
lsampl_t maxdata[N_CHANS_];

int prepare_cmd_lib(comedi_t *dev, int subdevice, int n_scan, int n_chan, unsigned period_nanosec, comedi_cmd *cmd);
void generate_voltages(std::vector<lsampl_t> raw_vector);
void generate_voltages(lsampl_t raw, int channel_index);
};

#endif /* DB_ATI_SENSOR_INCLUDE_DB_ATI_SENSOR_FT_SENSOR_H_ */
