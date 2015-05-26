/*
* Copyright: (C) 2015 Sheffield Robotics
* Authors: Uriel Martinez, Luke Boorman
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include <iostream>
#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

const int rec_seconds = 1;
const int sample_block = 8192;
const int sample_rate = 8000;


class AudioRobotModule: public RFModule
{
    private:
        Port senderPort;
        BufferedPort<Sound> receiverPort;
        IAudioGrabberSound *get;
        Property receiverConf;
        Property senderConf;
        IAudioRender *put;

        Sound sendSound;
        Sound *receiveSound;

        Contact gtw_contactAudioInputPort;
        Contact gtw_contactAudioOutputPort;

        string ip_address;

    public:
        AudioRobotModule(){}

        ~AudioRobotModule(){}

        bool updateModule()
        {
            double t1=yarp::os::Time::now();
            get->getSound(sendSound);
            double t2=yarp::os::Time::now();
            printf("acquired %f seconds\n", t2-t1);
            senderPort.write(sendSound);

            receiveSound = receiverPort.read(false);
            if (receiveSound!=NULL)
                put->renderSound(*receiveSound);

            return true;
        }

        bool configure(ResourceFinder &rf)
        {
            Property config;
            config.fromConfigFile(rf.getContext() + "config.ini");
            
            Bottle &bGeneral = config.findGroup("host_computer");

            ip_address = bGeneral.find("ip_address").asString().c_str();
            cout << "Host computer ip address: " << ip_address << endl;

            gtw_contactAudioInputPort = gtw_contactAudioInputPort.addName("/telepresence/audio/robot:i");
            gtw_contactAudioInputPort = gtw_contactAudioInputPort.addCarrier("tcp");
            gtw_contactAudioInputPort = gtw_contactAudioInputPort.addHost(ip_address);
            gtw_contactAudioInputPort = gtw_contactAudioInputPort.addPort(80000);

            gtw_contactAudioOutputPort = gtw_contactAudioOutputPort.addName("/telepresence/audio/robot:o");
            gtw_contactAudioOutputPort = gtw_contactAudioOutputPort.addCarrier("tcp");
            gtw_contactAudioOutputPort = gtw_contactAudioOutputPort.addHost(ip_address);
            gtw_contactAudioOutputPort = gtw_contactAudioOutputPort.addPort(80001);

            Network::registerContact(gtw_contactAudioInputPort);
            Network::registerContact(gtw_contactAudioOutputPort);

            senderPort.open(gtw_contactAudioOutputPort);
            receiverPort.open(gtw_contactAudioInputPort);

            senderConf.put("device","portaudio");
            senderConf.put("read", "");
            senderConf.put("samples", sample_block*rec_seconds);
            senderConf.put("rate", sample_rate);
            PolyDriver senderPoly(senderConf);

            // Make sure we can read sound
            senderPoly.view(get);

            if (get==NULL)
            {
                printf("cannot open read device interface\n");
                return 1;
            }


            // Receiver: Get an audio write device.
            receiverConf.put("device","portaudio");
            receiverConf.put("samples", sample_rate);
            receiverConf.put("write", "1");
            PolyDriver receiverPoly(receiverConf);
            // Make sure we can write sound
            receiverPoly.view(put);

            if (put==NULL)
            {
                printf("cannot open write device interface\n");
                return 1;
            }

            get->startRecording(); //this is optional, the first get->getsound() will do this anyway.

            return true;
        }

        bool interruptModule()
        {
            get->stopRecording(); //stops recording.

            Network::unregisterContact(gtw_contactAudioInputPort);
            Network::unregisterContact(gtw_contactAudioOutputPort);

            senderPort.close();
            receiverPort.close();

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
    rf.setDefaultContext("/home/uriel/Packages/SheffTelepresence/telepresence_audio/conf/");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);
    
    AudioRobotModule mod;

    return mod.runModule(rf);    
}

