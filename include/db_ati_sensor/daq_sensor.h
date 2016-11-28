/*
 * daq_sensor.h
 *
 *  Created on: 25 Nov 2016
 *      Author: vahid
 */

#ifndef DB_ATI_SENSOR_INCLUDE_DB_ATI_SENSOR_DAQ_SENSOR_H_
#define DB_ATI_SENSOR_INCLUDE_DB_ATI_SENSOR_DAQ_SENSOR_H_

class DAQ_Sensor{

public:
    DAQ_Sensor();
    ~DAQ_Sensor(){}
    double rate;

    int init_comedi();
    int run(float forces[12]);
#ifdef ROS
    bool sensorbias(sr_grasp_msgs::KCL_Sensor_Bias::Request& req,sr_grasp_msgs::KCL_Sensor_Bias::Response& response);
#endif
    Calibration *cal;
private:

    int subdev;		/* change this to your input subdevice */
    //    int chan = 0;		/* change this to your channel */
    int range;		/* more on this later */
    int aref;/* more on this later */
    float voltages[NUM_CHAN];
    int flags;

    comedi_t *it;
    int chan;
    lsampl_t data;
    int retval;
    comedi_polynomial_t converter;
    float result[NUM_CHAN];

#ifdef ROS
    ros::NodeHandle n;
    geometry_msgs::WrenchStamped sensor;
    ros::Publisher sensor_pub;
    ros::ServiceServer biasservice;
#endif
    std::string calfile;
    std::string daqdevice;
    char calfilepath[1024];

    int get_converter(comedi_t *device, unsigned subdevice, unsigned channel,
                      unsigned range, comedi_polynomial_t *converter);
};



#endif /* DB_ATI_SENSOR_INCLUDE_DB_ATI_SENSOR_DAQ_SENSOR_H_ */
