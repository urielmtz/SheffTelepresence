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


class TelepresenceVision_gtw: public RFModule
{
    private:
        BufferedPort<ImageOf<PixelRgb> > gtw_leftImageInputPort; // make a port for reading images
        BufferedPort<ImageOf<PixelRgb> > gtw_rightImageInputPort; // make a port for reading images
        BufferedPort<ImageOf<PixelRgb> > gtw_leftImageOutputPort; // make a port for reading images
        BufferedPort<ImageOf<PixelRgb> > gtw_rightImageOutputPort; // make a port for reading images

        Contact gtw_leftEyeInput;
        Contact gtw_rightEyeInput;
        Contact gtw_leftEyeOutput;
        Contact gtw_rightEyeOutput;

        string ip_address;

    public:
        TelepresenceVision_gtw()
        {}

        ~TelepresenceVision_gtw()
        {}

        bool configure(ResourceFinder &rf)
        {
            Property config;
            config.fromConfigFile(rf.getContext() + "config.ini");
//            config.fromConfigFile(rf.findFile("from").c_str());
            
            Bottle &bGeneral = config.findGroup("host_computer");

            ip_address = bGeneral.find("ip_address").asString().c_str();
            cout << "Host computer ip address: " << ip_address << endl;

            gtw_leftEyeInput = gtw_leftEyeInput.addName("/gtw/telepresence/leftEye:i");
            gtw_leftEyeInput = gtw_leftEyeInput.addCarrier("tcp");
            gtw_leftEyeInput = gtw_leftEyeInput.addHost(ip_address);
            gtw_leftEyeInput = gtw_leftEyeInput.addPort(60010);

            gtw_rightEyeInput = gtw_rightEyeInput.addName("/gtw/telepresence/rightEye:i");
            gtw_rightEyeInput = gtw_rightEyeInput.addCarrier("tcp");
            gtw_rightEyeInput = gtw_rightEyeInput.addHost(ip_address);
            gtw_rightEyeInput = gtw_rightEyeInput.addPort(60011);

            gtw_leftEyeOutput = gtw_leftEyeOutput.addName("/gtw/telepresence/leftEye:o");
            gtw_leftEyeOutput = gtw_leftEyeOutput.addCarrier("tcp");
            gtw_leftEyeOutput = gtw_leftEyeOutput.addHost(ip_address);
            gtw_leftEyeOutput = gtw_leftEyeOutput.addPort(60012);

            gtw_rightEyeOutput = gtw_rightEyeOutput.addName("/gtw/telepresence/rightEye:o");
            gtw_rightEyeOutput = gtw_rightEyeOutput.addCarrier("tcp");
            gtw_rightEyeOutput = gtw_rightEyeOutput.addHost(ip_address);
            gtw_rightEyeOutput = gtw_rightEyeOutput.addPort(60013);


            Network::registerContact(gtw_leftEyeInput);
            Network::registerContact(gtw_rightEyeInput);
            Network::registerContact(gtw_leftEyeOutput);
            Network::registerContact(gtw_rightEyeOutput);

            gtw_leftImageInputPort.open(gtw_leftEyeInput, true); // give the port a name
            gtw_rightImageInputPort.open(gtw_rightEyeInput, true); // give the port a name

            gtw_leftImageOutputPort.open(gtw_leftEyeOutput, true); // give the port a name
            gtw_rightImageOutputPort.open(gtw_rightEyeOutput, true); // give the port a name


            cout << "Waiting for connections: mjpeg://icub/cam/left to /gtw/telepresence/leftEye:i   and   mjpeg://icub/cam/right to /gtw/telepresence/rightEye:i" << endl;
            while( !Network::isConnected("mjpeg://icub/cam/left", "/gtw/telepresence/leftEye:i") || !Network::isConnected("mjpeg://icub/cam/right", "/gtw/telepresence/rightEye:i") )
            {}

            cout << "Connections ready: mjpeg://icub/cam/left to /gtw/telepresence/leftEye:i   and   mjpeg://icub/cam/right to /gtw/telepresence/rightEye:i" << endl;

            return true;
        }

        bool updateModule()
        {
            ImageOf<PixelRgb> *leftEyeInputImage = gtw_leftImageInputPort.read();
            ImageOf<PixelRgb> *rightEyeInputImage = gtw_rightImageInputPort.read();

            ImageOf<PixelRgb> &leftEyeOutputImage = gtw_leftImageOutputPort.prepare();
            ImageOf<PixelRgb> &rightEyeOutputImage = gtw_rightImageOutputPort.prepare();

            leftEyeOutputImage = *leftEyeInputImage;
            rightEyeOutputImage = *rightEyeInputImage;

            gtw_leftImageOutputPort.write();
            gtw_rightImageOutputPort.write();

            return true;
        }

        bool interruptModule()
        {
            Network::unregisterContact(gtw_leftEyeInput);
            Network::unregisterContact(gtw_rightEyeInput);
            Network::unregisterContact(gtw_leftEyeOutput);
            Network::unregisterContact(gtw_rightEyeOutput);

            gtw_leftImageInputPort.close();
            gtw_rightImageInputPort.close();
            gtw_leftImageOutputPort.close();
            gtw_rightImageOutputPort.close();

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
    rf.setDefaultContext("/home/uriel/Packages/SheffTelepresence/telepresence_vision_gateway/conf/");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    TelepresenceVision_gtw mod;

    return mod.runModule(rf);
}

