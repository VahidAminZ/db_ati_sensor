#include <stdio.h>	/* for printf() */
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

//write to file
#include <fstream>

using namespace std;

DAQ_Sensor::DAQ_Sensor(){

    subdev = 0;		/* change this to your input subdevice */
    //    int chan = 0;		/* change this to your channel */
    range = 0;		/* more on this later */
    aref = AREF_GROUND;	/* more on this later */
    chan=0;

#ifdef ROS
    if(!n.getParam("sensor/calibration", calfile) || !n.getParam("sensor/device", daqdevice) || !n.getParam("sensor/rate", rate)){
        ROS_ERROR("Unable to load the sensor parameters");
    }
    sensor_pub=n.advertise<geometry_msgs::WrenchStamped>("nano17ft",10);
    biasservice= n.advertiseService("sensor_bias", &DAQ_Sensor::sensorbias, this);
    sensor.header.frame_id=ros::this_node::getNamespace().substr(1);
#else
	daqdevice="/dev/comedi0";
#endif

}

#ifdef ROS
bool DAQ_Sensor::sensorbias(sr_grasp_msgs::KCL_Sensor_Bias::Request& req,sr_grasp_msgs::KCL_Sensor_Bias::Response& response){
    ROS_INFO("Biasing %s", ros::this_node::getName().c_str());
    Bias(cal,voltages);
    response.reply=true;
    return true;
}
#endif

/* figure out if we are talking to a hardware-calibrated or software-calibrated board,
    then obtain a comedi_polynomial_t which can be used with comedi_to_physical */
int DAQ_Sensor::get_converter(comedi_t *device, unsigned subdevice, unsigned channel,
                  unsigned range, comedi_polynomial_t *converter)
{
    flags = comedi_get_subdevice_flags(device, subdevice);
    if(flags < 0)
    {
        comedi_perror("comedi_get_subdevice_flags");
        return -1;
    }
    // TODO(Vahid): Check if this is correct or not (software calibration)
//    flags=0; //CHANGE ME!!!!! TO BYPASS SOFTWARE CALIBRATION


    if(flags & SDF_SOFT_CALIBRATED) // board uses software calibration
    {
        char *calibration_file_path = comedi_get_default_calibration_path(device);
        std::cout << "calibration file path is " << calibration_file_path << std::endl;

        //parse a calibration file which was produced by the comedi_soft_calibrate program
        comedi_calibration_t* parsed_calibration =
                comedi_parse_calibration_file(calibration_file_path);
        free(calibration_file_path);
        if(parsed_calibration == NULL)
        {
            printf("calibration_file_path: %s",calibration_file_path);
            comedi_perror("comedi_parse_calibration_file");
            return -1;
        }

        // get the comedi_polynomial_t for the subdevice/channel/range we are interested in
        retval = comedi_get_softcal_converter(subdevice, channel, range,
                                              COMEDI_TO_PHYSICAL, parsed_calibration, converter);
        comedi_cleanup_calibration(parsed_calibration);

        if(retval < 0)
        {
            comedi_perror("comedi_get_softcal_converter");
            return -1;
        }
    }

    else // board uses hardware calibration
    {
        retval = comedi_get_hardcal_converter(device, subdevice, channel, range,
                                              COMEDI_TO_PHYSICAL, converter);
        if(retval < 0)
        {
            comedi_perror("comedi_get_hardcal_converter");
            return -1;
        }
    }

    return 0;
}


int DAQ_Sensor::init_comedi(){
    it = comedi_open(daqdevice.c_str());
    if(it == NULL || it<0)
    {
#ifdef ROS
        ROS_ERROR("Is the comedi installed? Can this user access the device (sudo chown <user> /dev/comedi*)?");
#else
	printf("Is the comedi installed? Can this user access the device (sudo chown <user> /dev/comedi*)?");
#endif
        comedi_perror("comedi_open");
        return -1;
    }

    /*First reading and Biasing */
    for (chan=0;chan<NUM_CHAN; chan++){
        retval = comedi_data_read(it, subdev, chan, range, aref, &data);
        if(retval < 0)	{
            comedi_perror("comedi_data_read");
            return -1;
        }
        retval = get_converter(it, subdev, chan, range, &converter);

        if(retval < 0)	{
            return -1;
        }
        voltages[chan] = comedi_to_physical(data, &converter);
        std::cout << "voltages for chanel " << chan << " is " << voltages[chan] << std::endl;
    }

#ifdef ROS
    std::string path = ros::package::getPath("sr_grasp_tactile");
#else
    std::string path = "/";
#endif

//    strcpy(calfilepath,path.c_str());
//    strcat(calfilepath,calfile.c_str());
    std::string a="/home/vahid/projects/de_beers/src/db_ati_sensor/calibration/FT19470.cal";
    strcpy(calfilepath,a.c_str());
    printf("File %s\n",calfilepath);

    cal=createCalibration(calfilepath,1);
    SetForceUnits(cal,(char*) "N");
    SetTorqueUnits(cal,(char*) "N-mm");
printf("bbb\n");
    Bias(cal,voltages);
printf("aaa\n");
  return 0;
}


int DAQ_Sensor::run(float forces[12]){


    for (chan=0;chan<NUM_CHAN; chan++){
        retval = comedi_data_read(it, subdev, chan, range, aref, &data);

        if(retval < 0)	{
            comedi_perror("comedi_data_read");
            return -1;
        }
        std::cout << data << std::endl;
  //      retval = get_converter(it, subdev, chan, range, &converter);
        //if(retval < 0)	return -1;

        voltages[chan]= comedi_to_physical(data, &converter);
    }

    ConvertToFT(cal,voltages,result);


#ifdef ROS
    ROS_DEBUG_STREAM_THROTTLE(1,ros::Time::now().toSec() << "," <<voltages[0] << ","<< voltages[1] << ","<< voltages[2] << ","<< voltages[3] << ","<< voltages[4] << ","<< voltages[5] << ","<< result[0] << "," << result[1] << "," << result[2] << "," << result[3] << "," << result[4] << "," << result[5] << std::endl);

    sensor.wrench.force.x=result[0];
    sensor.wrench.force.y=result[1];
    sensor.wrench.force.z=result[2];
    sensor.wrench.torque.x=result[3];
    sensor.wrench.torque.y=result[4];
    sensor.wrench.torque.z=result[5];
    sensor.header.stamp=ros::Time::now();

    sensor_pub.publish(sensor);
#endif
    forces=result;
    return 0;
}

int main(int argc,char *argv[])
{

    DAQ_Sensor daq_sensor;


#ifdef ROS
    ros::init(argc, argv, "sensor");
    ros::Rate loop_rate(daq_sensor.rate);
#endif
    //Comedi Stuff

    if(daq_sensor.init_comedi()!=0) {
#ifdef ROS
        ROS_ERROR("Error in init_comedi");
#else
  printf("Error in init_comedi");
#endif
        return -1;
}

    /* Start Readings */

float ftdata[12];
#ifdef ROS
    while (ros::ok()) {
#else
    while(1){
#endif
        if(daq_sensor.run(ftdata)!=0){
#ifdef ROS
            ROS_ERROR("Error running");
        }
        //    ROS_DEBUG("[%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f]", sensor.wrench.force.x, sensor.wrench.force.y, sensor.wrench.force.z,sensor.wrench.torque.x, sensor.wrench.torque.y, sensor.wrench.torque.z);

        loop_rate.sleep();
        ros::spinOnce();
#else
        printf("Error running");
	}
    std::cout << ftdata[0] << " " << ftdata[1] << " " << ftdata[2] << std::endl;
	usleep(100000);
#endif

    }

    destroyCalibration(daq_sensor.cal);
    return 0;
}

