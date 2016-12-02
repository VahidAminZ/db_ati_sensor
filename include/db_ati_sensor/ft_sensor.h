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

private:
static const std::string cmdtest_messages[];
static const size_t BUFSZ = 100;
char buf[BUFSZ];

static const size_t N_CHANS = 16;

static unsigned int chanlist[N_CHANS];
static comedi_range * range_info[N_CHANS];
static lsampl_t maxdata[N_CHANS];

int prepare_cmd_lib(comedi_t *dev, int subdevice, int n_scan, int n_chan, unsigned period_nanosec, comedi_cmd *cmd);

void do_cmd(comedi_t *dev,comedi_cmd *cmd);

void print_datum(lsampl_t raw, int channel_index);


};

#endif /* DB_ATI_SENSOR_INCLUDE_DB_ATI_SENSOR_FT_SENSOR_H_ */
