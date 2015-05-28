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
        BufferedPort<Bottle> gtw_neckInputPort; // make a port for reading images
        BufferedPort<Bottle> gtw_neckOutputPort; // make a port for reading images
        BufferedPort<Bottle> gtw_neckConfOutputPort; // make a port for reading images

        Contact gtw_contactNeckInputPort;
        Contact gtw_contactNeckOutputPort;
        Contact gtw_contactNeckConfOutputPort;

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
//            config.fromConfigFile(rf.findFile("from").c_str());
            
            Bottle &bGeneral = config.findGroup("host_computer");

            ip_address = bGeneral.find("ip_address").asString().c_str();
            cout << "Host computer ip address: " << ip_address << endl;

            gtw_contactNeckInputPort = gtw_contactNeckInputPort.addName("/gtw/telepresence/neck:i");
            gtw_contactNeckInputPort = gtw_contactNeckInputPort.addCarrier("tcp");
            gtw_contactNeckInputPort = gtw_contactNeckInputPort.addHost(ip_address);
            gtw_contactNeckInputPort = gtw_contactNeckInputPort.addPort(80002);

            gtw_contactNeckOutputPort = gtw_contactNeckOutputPort.addName("/gtw/telepresence/neck:o");
            gtw_contactNeckOutputPort = gtw_contactNeckOutputPort.addCarrier("tcp");
            gtw_contactNeckOutputPort = gtw_contactNeckOutputPort.addHost(ip_address);
            gtw_contactNeckOutputPort = gtw_contactNeckOutputPort.addPort(80003);

            gtw_contactNeckConfOutputPort = gtw_contactNeckConfOutputPort.addName("/gtw/telepresence/neckconf:o");
            gtw_contactNeckConfOutputPort = gtw_contactNeckConfOutputPort.addCarrier("tcp");
            gtw_contactNeckConfOutputPort = gtw_contactNeckConfOutputPort.addHost(ip_address);
            gtw_contactNeckConfOutputPort = gtw_contactNeckConfOutputPort.addPort(80004);

            Network::registerContact(gtw_contactNeckInputPort);
            Network::registerContact(gtw_contactNeckOutputPort);
            Network::registerContact(gtw_contactNeckConfOutputPort);

            gtw_neckInputPort.open(gtw_contactNeckInputPort, true); // give the port a name
            gtw_neckOutputPort.open(gtw_contactNeckOutputPort, true); // give the port a name
            gtw_neckConfOutputPort.open(gtw_contactNeckConfOutputPort, true); // give the port a name


            cout << "Waiting for connection: /gtw/telepresence/neckconf:o -> /iKinGazeCtrl/rpc" << endl;
            while( !Network::isConnected("/gtw/telepresence/neckconf:o", "/iKinGazeCtrl/rpc") )
            {}
            cout << "Connection ready: /gtw/telepresence/neckconf:o -> /iKinGazeCtrl/rpc" << endl;

            Bottle &bot = gtw_neckConfOutputPort.prepare();
            bot.clear();
            bot.addString("bind");
            bot.addString("roll");
            bot.addDouble(-0.01);
            bot.addDouble(0.01);
            gtw_neckConfOutputPort.write();
            cout << "Sending initial configuration: " << bot.toString().c_str() << " from /gtw/telepresence/neckconf:o to /iKinGazeCtrl/rpc" << endl;



            cout << "Waiting for gateway connections: /telepresence/neck:o -> /gtw/telepresence/neck:i" << endl;
            while( !Network::isConnected("/telepresence/neck:o", "/gtw/telepresence/neck:i") )
            {}

            cout << "Connections ready: /telepresence/neck:o -> /gtw/telepresence/neck:i" << endl;

            return true;
        }

        bool updateModule()
        {
            Bottle *inputBottleNeck = gtw_neckInputPort.read();

            Bottle &outputBottleNeck = gtw_neckOutputPort.prepare();

            outputBottleNeck = *inputBottleNeck;

            gtw_neckOutputPort.write();
            return true;
        }

        bool interruptModule()
        {
            Network::unregisterContact(gtw_contactNeckInputPort);
            Network::unregisterContact(gtw_contactNeckOutputPort);
            Network::unregisterContact(gtw_contactNeckConfOutputPort);

            gtw_neckInputPort.close();
            gtw_neckOutputPort.close();
            gtw_neckConfOutputPort.close();

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
    rf.setDefaultContext("/home/icub/SheffTelepresence/telepresence_neck_gateway/conf/");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("SHEFFTELEPRESENCE_ROOT",argc,argv);

    TelepresenceNeck_gtw mod;

    return mod.runModule(rf);
}

