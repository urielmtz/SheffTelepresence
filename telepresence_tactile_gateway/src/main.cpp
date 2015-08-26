/*
* Copyright: (C) 2015 Sheffield Robotics
* Authors: Uriel Martinez
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include <iostream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


class TelepresenceNeck_gtw: public RFModule
{
    private:
        BufferedPort<Bottle> gtw_tactileLeftInputPort; // make a port for reading images
        BufferedPort<Bottle> gtw_tactileRightInputPort; // make a port for reading images
        BufferedPort<Bottle> gtw_tactileLeftOutputPort; // make a port for reading images
        BufferedPort<Bottle> gtw_tactileRightOutputPort; // make a port for reading images

        Contact gtw_contactTactileLeftInputPort;
        Contact gtw_contactTactileRightInputPort;
        Contact gtw_contactTactileLeftOutputPort;
        Contact gtw_contactTactileRightOutputPort;

        string ip_address;

    public:
        TelepresenceNeck_gtw()
        {}

        ~TelepresenceNeck_gtw()
        {}

        bool configure(ResourceFinder &rf)
        {
            Property config;
            config.fromConfigFile(rf.getContext() + "config.ini");
            
            Bottle &bGeneral = config.findGroup("host_computer");

            ip_address = bGeneral.find("ip_address").asString().c_str();
            cout << "Host computer ip address: " << ip_address << endl;

            gtw_contactTactileLeftInputPort = gtw_contactTactileLeftInputPort.addName("/gtw/telepresence/tactile/left:i");
            gtw_contactTactileLeftInputPort = gtw_contactTactileLeftInputPort.addCarrier("tcp");
            gtw_contactTactileLeftInputPort = gtw_contactTactileLeftInputPort.addHost(ip_address);
            gtw_contactTactileLeftInputPort = gtw_contactTactileLeftInputPort.addPort(80005);

            gtw_contactTactileRightInputPort = gtw_contactTactileRightInputPort.addName("/gtw/telepresence/tactile/right:i");
            gtw_contactTactileRightInputPort = gtw_contactTactileRightInputPort.addCarrier("tcp");
            gtw_contactTactileRightInputPort = gtw_contactTactileRightInputPort.addHost(ip_address);
            gtw_contactTactileRightInputPort = gtw_contactTactileRightInputPort.addPort(80006);

            gtw_contactTactileLeftOutputPort = gtw_contactTactileLeftOutputPort.addName("/gtw/telepresence/tactile/left:o");
            gtw_contactTactileLeftOutputPort = gtw_contactTactileLeftOutputPort.addCarrier("tcp");
            gtw_contactTactileLeftOutputPort = gtw_contactTactileLeftOutputPort.addHost(ip_address);
            gtw_contactTactileLeftOutputPort = gtw_contactTactileLeftOutputPort.addPort(80007);

            gtw_contactTactileRightOutputPort = gtw_contactTactileRightOutputPort.addName("/gtw/telepresence/tactile/right:o");
            gtw_contactTactileRightOutputPort = gtw_contactTactileRightOutputPort.addCarrier("tcp");
            gtw_contactTactileRightOutputPort = gtw_contactTactileRightOutputPort.addHost(ip_address);
            gtw_contactTactileRightOutputPort = gtw_contactTactileRightOutputPort.addPort(80008);


            Network::registerContact(gtw_contactTactileLeftInputPort);
            Network::registerContact(gtw_contactTactileRightInputPort);
            Network::registerContact(gtw_contactTactileLeftOutputPort);
            Network::registerContact(gtw_contactTactileRightOutputPort);

            gtw_tactileLeftInputPort.open(gtw_contactTactileLeftInputPort, true); // give the port a name
            gtw_tactileRightInputPort.open(gtw_contactTactileRightInputPort, true); // give the port a name
            gtw_tactileLeftOutputPort.open(gtw_contactTactileLeftOutputPort, true); // give the port a name
            gtw_tactileRightOutputPort.open(gtw_contactTactileRightOutputPort, true); // give the port a name


            cout << "Waiting for connection: /icub/skin/left_hand_comp -> /gtw/telepresence/tactile/left:i" << endl;
            while( !Network::isConnected("/icub/skin/left_hand_comp", "/gtw/telepresence/tactile/left:i") )
            {}
            cout << "Connection ready: /icub/skin/left_hand_comp -> /gtw/telepresence/tactile/left:i" << endl;

            cout << "Waiting for connection: /icub/skin/right_hand_comp -> /gtw/telepresence/tactile/right:i" << endl;
            while( !Network::isConnected("/icub/skin/right_hand_comp", "/gtw/telepresence/tactile/right:i") )
            {}
            cout << "Connection ready: /icub/skin/right_hand_comp -> /gtw/telepresence/tactile/right:i" << endl;

            return true;
        }

        bool updateModule()
        {
            Bottle *inputBottleTactileLeft = gtw_tactileLeftInputPort.read();
            Bottle *inputBottleTactileRight = gtw_tactileRightInputPort.read();

            Bottle &outputBottleTactileLeft = gtw_tactileLeftOutputPort.prepare();
            Bottle &outputBottleTactileRight = gtw_tactileRightOutputPort.prepare();

            outputBottleTactileLeft = *inputBottleTactileLeft;
            outputBottleTactileRight = *inputBottleTactileRight;

            gtw_tactileLeftOutputPort.write();
            gtw_tactileRightOutputPort.write();

            return true;
        }

        bool interruptModule()
        {
            Network::unregisterContact(gtw_contactTactileLeftInputPort);
            Network::unregisterContact(gtw_contactTactileRightInputPort);
            Network::unregisterContact(gtw_contactTactileLeftOutputPort);
            Network::unregisterContact(gtw_contactTactileRightOutputPort);

            gtw_tactileLeftInputPort.close();
            gtw_tactileRightInputPort.close();
            gtw_tactileLeftOutputPort.close();
            gtw_tactileRightOutputPort.close();

            return true;
        }

        double getPeriod()
        {
            return 0.03;
        }
};


int main( int argc, char **argv )
{
    Network yarp; // set up yarp

    if( !yarp.checkNetwork() )
    {
        cout << "yarp server not found..." << endl;
        return 1;
    }

    ResourceFinder rf;

    rf.setVerbose(true);
    rf.setDefaultContext("TactileModule_Gateway");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    TelepresenceNeck_gtw mod;

    return mod.runModule(rf);
}

