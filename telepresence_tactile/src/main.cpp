
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 RobotCub Consortium, European Commission FP7 Project
 * Author: Uriel Martinez
 * email: uriel.marherz@gmail.com
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 * *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <txzy.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


class TactileModule: public RFModule
{
    private:
        Txzy *myTxzy;
        BufferedPort<Vector> leftSensorValues;
        BufferedPort<Vector> rightSensorValues;

        int taxelsPerHand;
        int numOfHands;
        double *left_values = NULL;
        double *right_values = NULL;
        int taxelsRanges[2][6];
        int normalizer;

    public:
        TactileModule(){}

        ~TactileModule(){}

        bool updateModule()
        {
            Vector *tempLeftValues = leftSensorValues.read();
            Vector *tempRightValues = rightSensorValues.read();

            for( int i = 0; i < taxelsPerHand; i++ )
            {
                left_values[i] = 0.0;
                right_values[i] = 0.0;
            }

            for( int i = 0; i < taxelsPerHand; i++ )
            {
                for( int j = taxelsRanges[0][i]; j < taxelsRanges[1][i]; j++ )
                    left_values[i] += (*tempLeftValues)[ j ];                
                left_values[i] /= normalizer;
            }

            for( int i = 0; i < taxelsPerHand; i++ )
            {
                for( int j = taxelsRanges[0][i]; j < taxelsRanges[1][i]; j++ )
                    right_values[i] += (*tempRightValues)[ j ];                
                right_values[i] /= normalizer;
            }

            myTxzy->setPWM(right_values[0],right_values[1],right_values[2],right_values[3],right_values[4],right_values[5],left_values[0],left_values[1],left_values[2],left_values[3],left_values[4],left_values[5]);

            tempLeftValues->clear();
            tempRightValues->clear();

            return true;
        }

        bool configure(ResourceFinder &rf)
        {
            myTxzy = new Txzy("/dev/ttyACM0",9600,"none",8,1);

            cout << "Opening port" << endl;
            myTxzy->openPort();

            leftSensorValues.open("/telepresence/tactile/left:i");  // give the port a name
            rightSensorValues.open("/telepresence/tactile/right:i");  // give the port a name

            cout << "Waiting for connections: /gtw/telepresence/tactile/left:o -> /telepresence/tactile/left:i   and   /gtw/telepresence/tactile/right:o -> /telepresence/tactile/right:i" << endl;
            while( !Network::isConnected("/gtw/telepresence/tactile/left:o", "/telepresence/tactile/left:i") || !Network::isConnected("/gtw/telepresence/tactile/right:o", "/telepresence/tactile/right:i") )
            {}
            cout << "Connections ready: /gtw/telepresence/tactile/left:o -> /telepresence/tactile/left:i   and   /gtw/telepresence/tactile/right:o -> /telepresence/tactile/right:i" << endl;

            taxelsPerHand = 6;
            numOfHands = 2;
            normalizer = 2;  // normalizer values has to be improved

            left_values = new double[taxelsPerHand];
            right_values = new double[taxelsPerHand];

            taxelsRanges[0][0] = 0;
            taxelsRanges[0][1] = 12;
            taxelsRanges[0][2] = 24;
            taxelsRanges[0][3] = 36;
            taxelsRanges[0][4] = 48;
            taxelsRanges[0][5] = 92;
            taxelsRanges[1][0] = 11;
            taxelsRanges[1][1] = 23;
            taxelsRanges[1][2] = 35;
            taxelsRanges[1][3] = 47;
            taxelsRanges[1][4] = 59;
            taxelsRanges[1][5] = 139;

            return true;
        }

        bool interruptModule()
        {
            myTxzy->setPWM(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

            Network::disconnect("/gtw/telepresence/tactile/left:o", "/telepresence/tactile/left:i");
            Network::disconnect("/gtw/telepresence/tactile/right:o", "/telepresence/tactile/right:i");

            leftSensorValues.close();  // give the port a name
            rightSensorValues.close();  // give the port a name

            myTxzy->closePort();

            return true;
        }

        double getPeriod()
        {
            return 0.001;
        }

};

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        cout << endl << "ERROR: yarpserver not found" << endl;
        return false;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("TactileModule");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);
    
    TactileModule mod;

    return mod.runModule(rf);    
}
