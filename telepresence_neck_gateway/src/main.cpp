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

int main()
{
    Network yarp; // set up yarp

    BufferedPort<Bottle> gtw_neckInputPort; // make a port for reading images
    BufferedPort<Bottle> gtw_neckOutputPort; // make a port for reading images
    BufferedPort<Bottle> gtw_neckConfOutputPort; // make a port for reading images

    Contact gtw_contactNeckInputPort;
    Contact gtw_contactNeckOutputPort;
    Contact gtw_contactNeckConfOutputPort;

    gtw_contactNeckInputPort = gtw_contactNeckInputPort.addName("/gtw/telepresence/neck:i");
    gtw_contactNeckInputPort = gtw_contactNeckInputPort.addCarrier("tcp");
    gtw_contactNeckInputPort = gtw_contactNeckInputPort.addHost("143.167.49.238");
    gtw_contactNeckInputPort = gtw_contactNeckInputPort.addPort(60000);

    gtw_contactNeckOutputPort = gtw_contactNeckOutputPort.addName("/gtw/telepresence/neck:o");
    gtw_contactNeckOutputPort = gtw_contactNeckOutputPort.addCarrier("tcp");
    gtw_contactNeckOutputPort = gtw_contactNeckOutputPort.addHost("143.167.49.238");
    gtw_contactNeckOutputPort = gtw_contactNeckOutputPort.addPort(60001);

    gtw_contactNeckConfOutputPort = gtw_contactNeckConfOutputPort.addName("/gtw/telepresence/neckconf:o");
    gtw_contactNeckConfOutputPort = gtw_contactNeckConfOutputPort.addCarrier("tcp");
    gtw_contactNeckConfOutputPort = gtw_contactNeckConfOutputPort.addHost("143.167.49.238");
    gtw_contactNeckConfOutputPort = gtw_contactNeckConfOutputPort.addPort(60003);

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

    while(1)
    {
        Bottle *inputBottleNeck = gtw_neckInputPort.read();

        Bottle &outputBottleNeck = gtw_neckOutputPort.prepare();

        outputBottleNeck = *inputBottleNeck;

        gtw_neckOutputPort.write();
    }

    return 0;
}

