/*
* Copyright: (C) 2015 Sheffield Robotics
* Authors: Uriel Martinez
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include <stdio.h>
#include <iostream>

// Include the OculusVR SDK
#include "OVR_CAPI.h"

// Include YARP
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


class OculusModule: public RFModule
{
    private:
        ovrHmd hmd;
        bool OculusDeviceReady;

		BufferedPort<Bottle> neckOutputPort;
		BufferedPort<Bottle> neckConfOutputPort;

		int nsample;
        float yaw;
        float eyePitch;
        float eyeRoll;
        float wParam;

        float calibrationValue;
        bool calibrate;

        string networkType;

    public:
        OculusModule()
        {
            OculusDeviceReady = false;
            cout << "Running OculusModule" << endl;
			nsample = 0;
            calibrationValue = 0.0;
            calibrate = false;
        }

        ~OculusModule(){}

        bool updateModule()
        {
            if( OculusDeviceReady )
            {
                // Get more details about the HMD.
                //ovrSizei resolution = hmd->Resolution;    // maybe not needed for now
                // Start the sensor which provides the Rift’s pose and motion.
                ovrHmd_ConfigureTracking(hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);
                // Query the HMD for the current tracking state.
                ovrTrackingState ts = ovrHmd_GetTrackingState(hmd, ovr_GetTimeInSeconds());
                
                if(ts.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked))
                {
                    ovrPosef pose = ts.HeadPose.ThePose;
                    // pose.Orientation.GetEurlerAngles<Axis_Y, Aixs_X, Axis_Z>(&yaw, &eyePitch, &eyeRoll);


                    if( !calibrate )
                    {
                        calibrationValue = pose.Orientation.y;
                        calibrate= true;
                    }


                    // yaw = pose.Orientation.y;
                    yaw = pose.Orientation.y - calibrationValue;    // this calibration needs to be improved
                    eyePitch = pose.Orientation.x;
                    eyeRoll = pose.Orientation.z;
                    wParam = pose.Orientation.w;

                    cout << "POSE [y,x,z => yaw,pitch,roll]: " << yaw << " " << eyePitch << " " << eyeRoll << endl;
                    toEuler(&yaw, &eyePitch, &eyeRoll, wParam);
                    cout << "POSE Euler [yaw,pitch,roll]: " << yaw << " " << eyePitch << " " << eyeRoll << endl;

                    nsample = 0;
                    Bottle &bot = neckOutputPort.prepare();
                    bot.clear();
                    bot.addString("abs");
                    bot.addDouble(yaw*-180);		// (y) yaw in oculus which is x in icub
                    bot.addDouble(eyePitch*180);	// (x) pitch in oculus which is y in icub
                    bot.addDouble(0);
                    neckOutputPort.write();
                }
            }

            return true;
        }

        bool configure(ResourceFinder &rf)
        {
            cout << "Waiting for device..." << endl;
            do
            {
                OculusDeviceReady = Initialization();
            }while( OculusDeviceReady != true );

            Property config;
            config.fromConfigFile(rf.findFile("from").c_str());

            Bottle &networkSettings = config.findGroup("network_settings");
            networkType = networkSettings.find("network_type").asString();

            cout << "==== Network settings ====" << endl;
            cout << "networkType: " << networkType.c_str() << endl;

			neckOutputPort.open("/telepresence/neck:o");
            neckConfOutputPort.open("/telepresence/neckconf:o");

            if( networkType.compare("local") == 0 )
            {

                cout << "Waiting for gateway connections: /telepresence/neckconf:o -> /iKinGazeCtrl/rpc" << endl;
                while( !Network::isConnected("/telepresence/neckconf:o", "/iKinGazeCtrl/rpc") )
                {}
                cout << "Connections ready" << endl;                

                cout << "Waiting for connections: /telepresence/neck:o   and   /iKinGazeCtrl/angles:i" << endl;
                while( !Network::isConnected("/telepresence/neck:o", "/iKinGazeCtrl/angles:i") )
                {}
                cout << "Connections ready" << endl;                

            }
            else if( networkType.compare("remote") == 0 )
            {
                cout << "Waiting for port /gtw/telepresence/neckconf:i to be opened" << endl;
                while( !Network::exists("/gtw/telepresence/neckconf:i") )
                {}
                cout << "Port /gtw/telepresence/neckconf:i ready" << endl;

                cout << "Waiting for port /gtw/telepresence/neck:i to be opened" << endl;
                while( !Network::exists("/gtw/telepresence/neck:i") )
                {}
                cout << "Port /gtw/telepresence/neck:i ready" << endl;

                cout << "Waiting for connections: /telepresence/neckconf:o   and   /gtw/telepresence/neckconf:i" << endl;
                while( !Network::isConnected("/telepresence/neckconf:o", "/gtw/telepresence/neckconf:i") )
                {}
                cout << "Connections ready" << endl;                

                cout << "Waiting for connections: /telepresence/neck:o   and   /gtw/telepresence/neck:i" << endl;
                while( !Network::isConnected("/telepresence/neck:o", "/gtw/telepresence/neck:i") )
                {}
                cout << "Connections ready" << endl;                
	        }		
            else
            {
                cout << "Error network type" << endl;
                return false;
            }


            Bottle &bot = neckConfOutputPort.prepare();
            bot.clear();
            bot.addString("bind");
            bot.addString("roll");
            bot.addDouble(-0.01);
            bot.addDouble(0.01);
            neckConfOutputPort.write();


            return true;
        }

        bool interruptModule()
        {
            if( networkType.compare("local") == 0 )
            {
                Network::disconnect("/telepresence/neckconf:o","/iKinGazeCtrl/rpc");
                Network::disconnect("/telepresence/neck:o","/iKinGazeCtrl/angles:i");
            }
            else if( networkType.compare("remote") == 0 )
            {
                Network::disconnect("/telepresence/neckconf:o","/gtw/telepresence/neckconf:i");
                Network::disconnect("/telepresence/neck:o","/gtw/telepresence/neck:i");
            }            

            // Do something with the HMD.
            //ovrHmd_Destroy(hmd);
            ovr_Shutdown();

			neckOutputPort.close();
			neckConfOutputPort.close();

            return true;
        }

        double getPeriod()
        {
            return 0.03;
        }

        bool Initialization()
        {
            ovr_Initialize();
            hmd = ovrHmd_Create(0);
            
            if(hmd)
                return true; 
            else
                return false;             
        }

        void toEuler(float *x, float *y, float *z, float w)
        {
            float s=sin(w);
            float c=cos(w);
            float t=1-c;
            float ap, bp, cp;

            if (((*x)*(*y)*t + (*z)*s) > 0.998)
            { // north pole singularity detected
                ap = 0;
                bp = 2*atan2((*x)*sin(w/2),cos(w/2));
                cp = M_PI/2;
            }
            else if (((*x)*(*y)*t + (*z)*s) < -0.998)
            { // south pole singularity detected
                ap = 0;
                bp = -2*atan2((*x)*sin(w/2),cos(w/2));
                cp = -M_PI/2;
            }
            else
            {
                ap = atan2((*x) * s - (*y) * (*z) * t , 1 - ((*x)*(*x) + (*z)*(*z)) * t);
                bp = atan2((*y) * s- (*x) * (*z) * t , 1 - ((*y)*(*y)+ (*z)*(*z) ) * t);
                cp = asin((*x) * (*y) * t + (*z) * s) ;
            }

            *x = ap;
            *y = bp;
            *z = cp;
        }
};

int main(int argc, char **argv)
{
    Network yarp;

    if( !yarp.checkNetwork() )
    {
        cout << "yarp server not found..." << endl;
        return 1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("NeckModule");
    rf.setDefaultConfigFile("NeckModule.ini");
    rf.configure(argc,argv);
    
    OculusModule mod;

    return mod.runModule(rf);
}
