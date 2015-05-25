// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 The University of Sheffield
 * Author: Uriel Martinez
 * email: uriel.marherz@gmail.com
 *
*/

#ifndef __TXZY_H__
#define __TXZY_H__

/**
 * @file txzy.h
 * @class Txzy
 * @brief this file contains the definition of the main module code.
 * @see example.cpp
 */

/** 
 * @ingroup Sheffield_modules
 *
 * \defgroup Sheffield_modules Txzy
 *
 * Addressing:
 * - documentation and coding standards
 *
 *
 * \section tested_os_sec Tested OS
 *
 * Linux \n
 *
 * \section compiling Configuring CMakeLists
 * The path of SerialDeviceDriver.h has to be specified when the module is compiled. The easiest way is using ccmake in advanced mode and in CXX_FLAGS parameter
 * the header file has to be added which is inside your YARP path, i.e $YARP_ROOT/src/modules/serial
 *
 * \author Uriel Martinez
 * 
 * Copyright (C) 2011 The University of Sheffield
 * 
 */


#include <fstream>
#include <yarp/os/Time.h>
#include <SerialDeviceDriver.h>
#include <yarp/dev/DeviceDriver.h>

using namespace std;
using namespace yarp::dev;

//!  Txzy class. 
/*!
    Txzy class contains methods to manage the communicate and control a 4 degrees of freedom positioning robot.
 */

class Txzy
{
	private:
        //! Establish the settings of the communication
		SerialDeviceDriverSettings settings;
        //! Object to establish the communication with the servo
		SerialDeviceDriver commPort;

        //! MOVE command
		Bottle *moveCommand;

        //! Name of the communication port
		char *port;

        //! Path of the folder that contains prog.txt and pars.txt files
		string pathName;
        //! Name of the folder that contains prog.txt and pars.txt files
		string fileName;

        //! Temporary prog file to store the content of prog.txt file
		fstream xyProgFile;
        //! Temporary pars file to store the content of pars.txt file
		fstream xyParsFile;

        //! Lenght of the content of prog.txt file
		int progLength;
        //! Lenght of the content of pars.txt file
		int parsLength;

	protected:

	public:
		/**
		 * Default constructor.
         * Set the parameters.
         *
         * port_name = /dev/ttyUSB0
         * baudrate_conf = 9600
         * parity_mode = odd
         * databits_conf = 8
         * stopbits_conf = 1
         *
         * @see xytable.cpp
		 */
		Txzy();
		/**
		 * Optional Constructor
         *
         * @param port_name name of the communication port (i.e. COM1, /dev/ttyUSB0)
         * @param baudrate_conf baudrate configuration
         * @param parity_mode parity configuration
         * @param databits_conf databits configuration
         * @param stopbits_conf stopbits configuration
         * @see xytable.cpp
		 */
		Txzy(char *port_name, int baudrate_conf, char *parity_mode, int databits_conf, int stopbits_conf);
		Txzy(char *port_name, int baudrate_conf, char *parity_mode, int databits_conf, int stopbits_conf, char *flowcontrol_mode);
		/**
		 * Default destructor
		 */
		~Txzy();
		/**
		 * Initializes the attributes required for the communication
		 */
		void initCommands();
		/**
		 * Starts the servo, this allows the XYTable to execute some action
         *
         * @return true/false if the action was successful/unsuccessful respectively
		 */
		bool servoOn();
		/**
		 * Stops the servo, this disable the XYTable to execute some action
         *
         * @return true/false if the action was successful/unsuccessful respectively
         * @see xytable.cpp
		 */
		bool servoOff();
		/**
		 * Starts the communication with the servo
         *
         * @return true/false if the action was successful/unsuccessful respectively
         * @see xytable.cpp
		 */
		bool openPort();
		/**
		 * Closes the communication with the servo
         *
         * @return true/false if the action was successful/unsuccessful respectively
         * @see xytable.cpp
		 */
		bool closePort();
        bool move(char p_axis, int p_position, int p_speed);
        bool move(int p_positionX, int p_positionY, int p_positionZ, int p_positionT, int p_speedX, int p_speedY, int p_speedZ, int p_speedT);
        bool moveTo(char p_axis, int p_position, int p_speed);
        bool moveTo(int p_positionX, int p_positionY, int p_positionZ, int p_positionT, int p_speedX, int p_speedY, int p_speedZ, int p_speedT);
		/**
		 * Resets the system
         *
         * @return true/false if the action was successful/unsuccessful respectively
         * @see xytable.cpp
		 */
		bool reset();
		bool manual();
		/**
		 * Gets the position of the robot
         *
         * @return true/false if the action was successful/unsuccessful respectively
         * @see xytable.cpp
		 */
		bool where();
		/**
         * Loads in memory the files prog.txt and pars.txt
         *
         * @param path is the path where the files prog.txt and pars.txt are stored
         * @return true/false if the action was successful/unsuccessful respectively
         * @see xytable.cpp
		 */
		bool program(string path);
		/**
		 * Executes a program
         *
         * @return true/false if the action was successful/unsuccessful respectively
         * @see xytable.cpp
		 */
		bool execute();
		/**
		 * Gets the position of the robot as double values, this way the values can be manipulated
         *
         * @param x_p X position
         * @param y_p Y position
         * @see xytable.cpp
		 */
		void getPosition(int &x_p, int &y_p, int &z_p, int &t_p);
		/**
		 * Sends commands to the servo, i.e. SERVO ON, SERVO OFF, WHERE
         *
         * @param s_command action to be executed
         * @return true/false if the action was successful/unsuccessful respectively
         * @see xytable.cpp
		 */
		bool command(string s_command);
		/**
		 * Splits the path and substracts the name of the folder where the files prog.txt and pars.txt are stored
         *
         * @param path is the path where the files prog.txt and pars.txt are stored
         * @see xytable.cpp
		 */
		void splitFileName(string path);
		/**
		 * Converts to uppercase the characters of a string
         *
         * @param str string to be converted
         * @see xytable.cpp
		 */
		void toUpperCase(string &);

        bool isCalibrated();
        bool goHomePosition();
        Bottle status();

        bool handShake();

        bool setPWM(int pwm_thumb, int pwm_index, int pwm_middle, int pwm_ring, int pwm_little, int pwm_palm);

        bool setPWM(int right_pwm_thumb, int right_pwm_index, int right_pwm_middle, int right_pwm_ring, int right_pwm_little, int right_pwm_palm, int left_pwm_thumb, int left_pwm_index, int left_pwm_middle, int left_pwm_ring, int left_pwm_little, int left_pwm_palm);
};

#endif /*__TXZY_H__*/

