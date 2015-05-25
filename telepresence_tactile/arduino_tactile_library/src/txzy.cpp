// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 The University of Sheffield
 * Author: Uriel Martinez-Hernandez
 * email: uriel.marherz@gmail.com
 *
*/

/**
 * @file txzy.cpp
 * @brief this file contains the implementation of the main code.
 * @include txzy.cpp
 */

#include <txzy.h>

/* Default constructor */
Txzy::Txzy()
{
	port = "/dev/ttyUSB0";
	strcpy(settings.CommChannel, port);
	settings.SerialParams.baudrate = 9600;
	settings.SerialParams.paritymode = "none";
	settings.SerialParams.databits = 8;
	settings.SerialParams.stopbits = 1;

	initCommands();
}

/* Optional constructor */
Txzy::Txzy(char *port_name, int baudrate_conf, char *parity_mode, int databits_conf, int stopbits_conf)
{
	port = port_name;
	strcpy(settings.CommChannel, port);
	settings.SerialParams.baudrate = baudrate_conf;
	settings.SerialParams.paritymode = parity_mode;
	settings.SerialParams.databits = databits_conf;
	settings.SerialParams.stopbits = stopbits_conf;

	initCommands();
}

/* Optional constructor */
Txzy::Txzy(char *port_name, int baudrate_conf, char *parity_mode, int databits_conf, int stopbits_conf, char *flowcontrol_mode)
{
	port = port_name;
	strcpy(settings.CommChannel, port);
	settings.SerialParams.baudrate = baudrate_conf;
	settings.SerialParams.paritymode = parity_mode;
	settings.SerialParams.databits = databits_conf;
	settings.SerialParams.stopbits = stopbits_conf;
	settings.SerialParams.xinenb = flowcontrol_mode;

	initCommands();
}

/* Initializes attributes */
void Txzy::initCommands()
{
    moveCommand = new Bottle;
}

/* Destructor */
Txzy::~Txzy()
{
	delete moveCommand;
}

/* Starts the servo */
bool Txzy::servoOn()
{
    return false;
}

/* Stops the servo */
bool Txzy::servoOff()
{
    return false;
}

/* Starts the communication */
bool Txzy::openPort()
{
	if( commPort.open(settings) )
		return true;
	else
		return false;
}

/* Stops the communication */
bool Txzy::closePort()
{
	if( commPort.close() )
		return true;
	else
		return false;
}

/* Relative movement of the robot in X axis*/
bool Txzy::move(char p_axis, int p_position, int p_speed)
{
    char *command = new char[100];
	char s_axis[2];
	char *s_position = new char[10];
	char *s_speed = new char[10];

	//if( p_position < 0 || p_position > 100000 )
	//{
	//	cout << endl << "Error!, XMin:0 XMax:80000" << endl;
	//	return false;
	//}

	s_axis[0] = p_axis;
    s_axis[0] = toupper(s_axis[0]);
    s_axis[1] = '\0';
	sprintf(s_position, "%d", p_position);
	sprintf(s_speed, "%d", p_speed);
	strcpy(command,"@");
	strcat(command, "MOVR");
	strcat(command, s_axis);
	strcat(command, " ");
	strcat(command, s_position);
	strcat(command, " ");
	strcat(command, s_speed);
	strcat(command, "\r");
	moveCommand->addString(command);

    commPort.send(*moveCommand);
    while( !handShake() );
    Time::delay(0.3);

    while( !handShake() );
    Time::delay(0.3);

    moveCommand->clear();
	strcpy(s_axis,"");
	strcpy(s_position,"");
	strcpy(s_speed,"");

	return true;
}

/* Relative movement of the robot in X axis*/
bool Txzy::move(int p_positionX, int p_positionY, int p_positionZ, int p_positionT, int p_speedX, int p_speedY, int p_speedZ, int p_speedT)
{
	char *command = new char[100];
	char *s_positionX = new char[10];
	char *s_positionY = new char[10];
	char *s_positionZ = new char[10];
	char *s_positionT = new char[10];
	char *s_speedX = new char[10];
	char *s_speedY = new char[10];
	char *s_speedZ = new char[10];
	char *s_speedT = new char[10];

	//if( p_positionX < 0 || p_positionX > 100000 )
	//{
	//	cout << endl << "Error!, XMin:0 XMax:80000" << endl;
	//	return false;
	//}

	sprintf(s_positionX, "%d", p_positionX);
	sprintf(s_positionY, "%d", p_positionY);
	sprintf(s_positionZ, "%d", p_positionZ);
	sprintf(s_positionT, "%d", p_positionT);

	sprintf(s_speedX, "%d", p_speedX);
	sprintf(s_speedY, "%d", p_speedY);
	sprintf(s_speedZ, "%d", p_speedZ);
	sprintf(s_speedT, "%d", p_speedT);

	strcpy(command,"@");
	strcat(command, "MOVRALL");
	strcat(command, " ");
	strcat(command, s_positionX);
	strcat(command, " ");
	strcat(command, s_positionY);
	strcat(command, " ");
	strcat(command, s_positionZ);
	strcat(command, " ");
	strcat(command, s_positionT);
	strcat(command, " ");
	strcat(command, s_speedX);
	strcat(command, " ");
	strcat(command, s_speedY);
	strcat(command, " ");
	strcat(command, s_speedZ);
	strcat(command, " ");
	strcat(command, s_speedT);
	strcat(command, "\r");
	moveCommand->addString(command);

    commPort.send(*moveCommand);
    while( !handShake() );
    Time::delay(0.3);

    while( !handShake() );
    Time::delay(0.3);

    moveCommand->clear();
	strcpy(s_positionX,"");
	strcpy(s_positionY,"");
	strcpy(s_positionZ,"");
	strcpy(s_positionT,"");
	strcpy(s_speedX,"");
	strcpy(s_speedY,"");
	strcpy(s_speedZ,"");
	strcpy(s_speedT,"");

	return true;
}

/* Absolute movement of the robot in X axis*/
bool Txzy::moveTo(char p_axis, int p_position, int p_speed)
{
	char *command = new char[100];
	char s_axis[2];
	char *s_position = new char[10];
	char *s_speed = new char[10];

//	if( p_position < 0 || p_position > 100000 )
//	{
//		cout << endl << "Error!, XMin:0 XMax:80000" << endl;
//		return false;
//	}

	s_axis[0] = p_axis;
    s_axis[0] = toupper(s_axis[0]);
    s_axis[1] = '\0';
	sprintf(s_position, "%d", p_position);
	sprintf(s_speed, "%d", p_speed);
	strcpy(command,"@");
	strcat(command, "MOVA");
	strcat(command, s_axis);
	strcat(command, " ");
	strcat(command, s_position);
	strcat(command, " ");
	strcat(command, s_speed);
	strcat(command, "\r");
	moveCommand->addString(command);

    commPort.send(*moveCommand);
    while( !handShake() );
    Time::delay(0.3);

    while( !handShake() );
    Time::delay(0.3);

	moveCommand->clear();
	strcpy(s_axis,"");
	strcpy(s_position,"");
	strcpy(s_speed,"");

	return true;
}


/* Absolute movement of the robot in X axis*/
bool Txzy::moveTo(int p_positionX, int p_positionY, int p_positionZ, int p_positionT, int p_speedX, int p_speedY, int p_speedZ, int p_speedT)
{
	char *command = new char[100];
	char *s_positionX = new char[10];
	char *s_positionY = new char[10];
	char *s_positionZ = new char[10];
	char *s_positionT = new char[10];
	char *s_speedX = new char[10];
	char *s_speedY = new char[10];
	char *s_speedZ = new char[10];
	char *s_speedT = new char[10];

	sprintf(s_positionX, "%d", p_positionX);
	sprintf(s_positionY, "%d", p_positionY);
	sprintf(s_positionZ, "%d", p_positionZ);
	sprintf(s_positionT, "%d", p_positionT);

	sprintf(s_speedX, "%d", p_speedX);
	sprintf(s_speedY, "%d", p_speedY);
	sprintf(s_speedZ, "%d", p_speedZ);
	sprintf(s_speedT, "%d", p_speedT);

	strcpy(command,"@");
	strcat(command, "MOVAALL");
	strcat(command, " ");
	strcat(command, s_positionX);
	strcat(command, " ");
	strcat(command, s_positionY);
	strcat(command, " ");
	strcat(command, s_positionZ);
	strcat(command, " ");
	strcat(command, s_positionT);
	strcat(command, " ");
	strcat(command, s_speedX);
	strcat(command, " ");
	strcat(command, s_speedY);
	strcat(command, " ");
	strcat(command, s_speedZ);
	strcat(command, " ");
	strcat(command, s_speedT);
	strcat(command, "\r");
	moveCommand->addString(command);

    commPort.send(*moveCommand);
    while( !handShake() );
    Time::delay(0.3);

    while( !handShake() );
    Time::delay(0.3);

    moveCommand->clear();
	strcpy(s_positionX,"");
	strcpy(s_positionY,"");
	strcpy(s_positionZ,"");
	strcpy(s_positionT,"");
	strcpy(s_speedX,"");
	strcpy(s_speedY,"");
	strcpy(s_speedZ,"");
	strcpy(s_speedT,"");

	return true;
}

/* Resets the servo */
bool Txzy::reset()
{
    return false;
}

/* Gets the position of the robot */
bool Txzy::where()
{
    return false;
}

/* Loads in memory the program */
bool Txzy::program(string path)
{
	return true;
}

/* Executes the program stored in memory */
bool Txzy::execute()
{
	return true;
}

/* Gets the position of the robot in as double values */
void Txzy::getPosition(int &x_p, int &y_p, int &z_p, int &t_p)
{
	char *line = new char[100];
	string temp;
	int px;
	int py;
	int pz;
	int pt;
}

/* Sends commands to the servo */
bool Txzy::command(string s_command)
{
	Bottle command;
	char *f_command = new char[100];

	strcpy(f_command, "@");
	strcat(f_command, s_command.c_str());
	strcat(f_command, " \r");
	command.addString(f_command);
	commPort.send(command);
	
    commPort.send(*moveCommand);
    while( !handShake() );
    Time::delay(0.3);

    while( !handShake() );
    Time::delay(0.3);
	
	return true;
}

/* Splits the path and substracts the name of the folder where the files prog.txt and pars.txt are stored */
void Txzy::splitFileName(string path)
{
	size_t found;

	found = path.find_last_of("/\\");
	pathName = path + "/";
	fileName = path.substr(found+1);
}

/* Converts to uppercase the characters of a string */
void Txzy::toUpperCase(string &str)
{
	for( int i = 0; i < str.length(); i++ )
		str[ i ] = toupper(str[ i ]);
}

/* Check if the texy table is calibrated */
bool Txzy::isCalibrated()
{
    return true;
}

/* Move texy table to home position */
bool Txzy::goHomePosition()
{
	char *command = new char[100];
	strcpy(command,"");

	strcpy(command,"@");
	strcat(command, "MOVHOME");
	strcat(command, "\r");
	moveCommand->addString(command);

    commPort.send(*moveCommand);
    while( !handShake() );
    Time::delay(0.3);

    while( !handShake() );
    Time::delay(0.3);

	moveCommand->clear();

    return true;
}

Bottle Txzy::status()
{
    Bottle botStatus;
    botStatus.add("empty");

    commPort.receive(botStatus);

    return botStatus;
}

bool Txzy::handShake()
{
	char *line = new char[10];
	
	for( int i = 0; i < 10; i++ )
	    line[i] = '\0';
	    
	commPort.receiveLine(line,10);

    if( strcmp(line,"ACK") )
    {
        printf("%s",line);
        return true;
    }
    else
    {
        printf("%s",line);
        return false;
    }
}

bool Txzy::setPWM(int pwm_thumb, int pwm_index, int pwm_middle, int pwm_ring, int pwm_little, int pwm_palm)
{
	char *command = new char[100];
	char *s_pwm_thumb = new char[10];
	char *s_pwm_index = new char[10];
	char *s_pwm_middle = new char[10];
	char *s_pwm_ring = new char[10];
	char *s_pwm_little = new char[10];
	char *s_pwm_palm = new char[10];

	sprintf(s_pwm_thumb, "%d", pwm_thumb);
	sprintf(s_pwm_index, "%d", pwm_index);
	sprintf(s_pwm_middle, "%d", pwm_middle);
	sprintf(s_pwm_ring, "%d", pwm_ring);
	sprintf(s_pwm_little, "%d", pwm_little);
	sprintf(s_pwm_palm, "%d", pwm_palm);

	strcpy(command,"@");
	strcat(command, "SETPWM");
	strcat(command, " ");
	strcat(command, s_pwm_thumb);
	strcat(command, " ");
	strcat(command, s_pwm_index);
	strcat(command, " ");
	strcat(command, s_pwm_middle);
	strcat(command, " ");
	strcat(command, s_pwm_ring);
	strcat(command, " ");
	strcat(command, s_pwm_little);
	strcat(command, " ");
	strcat(command, s_pwm_palm);
	strcat(command, "\r");
	moveCommand->addString(command);

    commPort.send(*moveCommand);
    while( !handShake() );
    Time::delay(0.001);

    while( !handShake() );
    Time::delay(0.001);

    moveCommand->clear();

	return true;
}

bool Txzy::setPWM(int right_pwm_thumb, int right_pwm_index, int right_pwm_middle, int right_pwm_ring, int right_pwm_little, int right_pwm_palm, int left_pwm_thumb, int left_pwm_index, int left_pwm_middle, int left_pwm_ring, int left_pwm_little, int left_pwm_palm)
{
	char *command = new char[200];
	char *r_pwm_thumb = new char[10];
	char *r_pwm_index = new char[10];
	char *r_pwm_middle = new char[10];
	char *r_pwm_ring = new char[10];
	char *r_pwm_little = new char[10];
	char *r_pwm_palm = new char[10];
	char *l_pwm_thumb = new char[10];
	char *l_pwm_index = new char[10];
	char *l_pwm_middle = new char[10];
	char *l_pwm_ring = new char[10];
	char *l_pwm_little = new char[10];
	char *l_pwm_palm = new char[10];

	sprintf(r_pwm_thumb, "%d", right_pwm_thumb);
	sprintf(r_pwm_index, "%d", right_pwm_index);
	sprintf(r_pwm_middle, "%d", right_pwm_middle);
	sprintf(r_pwm_ring, "%d", right_pwm_ring);
	sprintf(r_pwm_little, "%d", right_pwm_little);
	sprintf(r_pwm_palm, "%d", right_pwm_palm);
	sprintf(l_pwm_thumb, "%d", left_pwm_thumb);
	sprintf(l_pwm_index, "%d", left_pwm_index);
	sprintf(l_pwm_middle, "%d", left_pwm_middle);
	sprintf(l_pwm_ring, "%d", left_pwm_ring);
	sprintf(l_pwm_little, "%d", left_pwm_little);
	sprintf(l_pwm_palm, "%d", left_pwm_palm);

	strcpy(command,"@");
	strcat(command, "SETPWM");
	strcat(command, " ");
	strcat(command, r_pwm_thumb);
	strcat(command, " ");
	strcat(command, r_pwm_index);
	strcat(command, " ");
	strcat(command, r_pwm_middle);
	strcat(command, " ");
	strcat(command, r_pwm_ring);
	strcat(command, " ");
	strcat(command, r_pwm_little);
	strcat(command, " ");
	strcat(command, r_pwm_palm);
	strcat(command, " ");
	strcat(command, l_pwm_thumb);
	strcat(command, " ");
	strcat(command, l_pwm_index);
	strcat(command, " ");
	strcat(command, l_pwm_middle);
	strcat(command, " ");
	strcat(command, l_pwm_ring);
	strcat(command, " ");
	strcat(command, l_pwm_little);
	strcat(command, " ");
	strcat(command, l_pwm_palm);
	strcat(command, "\r");
	moveCommand->addString(command);

    commPort.send(*moveCommand);
    while( !handShake() );
    Time::delay(0.001);

    while( !handShake() );
    Time::delay(0.001);

    moveCommand->clear();

	return true;
}
