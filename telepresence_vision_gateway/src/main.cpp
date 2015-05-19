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

    BufferedPort<ImageOf<PixelRgb> > gtw_leftImageInputPort; // make a port for reading images
    BufferedPort<ImageOf<PixelRgb> > gtw_rightImageInputPort; // make a port for reading images
    BufferedPort<ImageOf<PixelRgb> > gtw_leftImageOutputPort; // make a port for reading images
    BufferedPort<ImageOf<PixelRgb> > gtw_rightImageOutputPort; // make a port for reading images

    Contact gtw_leftEyeInput;
    Contact gtw_rightEyeInput;
    Contact gtw_leftEyeOutput;
    Contact gtw_rightEyeOutput;

    gtw_leftEyeInput = gtw_leftEyeInput.addName("/gtw/telepresence/leftEye:i");
    gtw_leftEyeInput = gtw_leftEyeInput.addCarrier("tcp");
    gtw_leftEyeInput = gtw_leftEyeInput.addHost("143.167.49.238");
    gtw_leftEyeInput = gtw_leftEyeInput.addPort(10100);

    gtw_rightEyeInput = gtw_rightEyeInput.addName("/gtw/telepresence/rightEye:i");
    gtw_rightEyeInput = gtw_rightEyeInput.addCarrier("tcp");
    gtw_rightEyeInput = gtw_rightEyeInput.addHost("143.167.49.238");
    gtw_rightEyeInput = gtw_rightEyeInput.addPort(10101);

    gtw_leftEyeOutput = gtw_leftEyeOutput.addName("/gtw/telepresence/leftEye:o");
    gtw_leftEyeOutput = gtw_leftEyeOutput.addCarrier("tcp");
    gtw_leftEyeOutput = gtw_leftEyeOutput.addHost("143.167.49.238");
    gtw_leftEyeOutput = gtw_leftEyeOutput.addPort(10102);

    gtw_rightEyeOutput = gtw_rightEyeOutput.addName("/gtw/telepresence/rightEye:o");
    gtw_rightEyeOutput = gtw_rightEyeOutput.addCarrier("tcp");
    gtw_rightEyeOutput = gtw_rightEyeOutput.addHost("143.167.49.238");
    gtw_rightEyeOutput = gtw_rightEyeOutput.addPort(10103);


    Network::registerContact(gtw_leftEyeInput);
    Network::registerContact(gtw_rightEyeInput);
    Network::registerContact(gtw_leftEyeOutput);
    Network::registerContact(gtw_rightEyeOutput);

    gtw_leftImageInputPort.open(gtw_leftEyeInput, true); // give the port a name
    gtw_rightImageInputPort.open(gtw_rightEyeInput, true); // give the port a name

    gtw_leftImageOutputPort.open(gtw_leftEyeOutput, true); // give the port a name
    gtw_rightImageOutputPort.open(gtw_rightEyeOutput, true); // give the port a name


    cout << "Waiting for connections: /icub/cam/left to /gtw/telepresence/leftEye:i   and   /icub/cam/right to /gtw/telepresence/rightEye:i" << endl;
    while( !Network::isConnected("/icub/cam/left", "/gtw/telepresence/leftEye:i") || !Network::isConnected("/icub/cam/right", "/gtw/telepresence/rightEye:i") )
    {}

    cout << "Connections ready: /icub/cam/left to /gtw/telepresence/leftEye:i   and   /icub/cam/right to /gtw/telepresence/rightEye:i" << endl;

    while(1)
    {
        ImageOf<PixelRgb> *leftEyeInputImage = gtw_leftImageInputPort.read();
        ImageOf<PixelRgb> *rightEyeInputImage = gtw_rightImageInputPort.read();

        ImageOf<PixelRgb> &leftEyeOutputImage = gtw_leftImageOutputPort.prepare();
        ImageOf<PixelRgb> &rightEyeOutputImage = gtw_rightImageOutputPort.prepare();

        leftEyeOutputImage = *leftEyeInputImage;
        rightEyeOutputImage = *rightEyeInputImage;

        gtw_leftImageOutputPort.write();
        gtw_rightImageOutputPort.write();
    }

    return 0;
}

