/*
* Copyright: (C) 2015 Sheffield Robotics
* Authors: Uriel Martinez
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/AudioGrabberInterfaces.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

const int rec_seconds = 1;
const int sample_block = 2048;
const int sample_rate = 8000;

int main(int argc, char *argv[])
{
    // Open the network
    Network yarp;
    Port senderPort;
    BufferedPort<Sound> receiverPort;

    senderPort.open("/telepresence/human:o");
    receiverPort.open("/telepresence/human:i");

//    Network::connect("/sender", "/receiver");

    // Sender: Get a portaudio read device.
    Property senderConf;
    senderConf.put("device","portaudio");
    senderConf.put("read", "");
    senderConf.put("samples", sample_block*rec_seconds);
//    senderConf.put("samples", 44100*rec_seconds);
//    senderConf.put("rate", 16000);
    senderConf.put("rate", sample_rate);
    PolyDriver senderPoly(senderConf);
    IAudioGrabberSound *get;
    // Make sure we can read sound
    senderPoly.view(get);

    if (get==NULL)
    {
        printf("cannot open read device interface\n");
        return 1;
    }


    // Receiver: Get an audio write device.
    Property receiverConf;
    receiverConf.put("device","portaudio");
    receiverConf.put("samples", sample_block);
//    conf.put("samples", "8192");
    receiverConf.put("write", "1");
    PolyDriver receiverPoly(receiverConf);
    IAudioRender *put;
    // Make sure we can write sound
    receiverPoly.view(put);

    if (put==NULL)
    {
        printf("cannot open write device interface\n");
        return 1;
    }

    //Grab and send
    Sound sendSound;

    //Receive and render
    Sound *receiveSound;

    get->startRecording(); //this is optional, the first get->getsound() will do this anyway.

    while (true)
    {
        double t1=yarp::os::Time::now();
        get->getSound(sendSound);
        double t2=yarp::os::Time::now();
        printf("acquired %f seconds\n", t2-t1);
        senderPort.write(sendSound);

        receiveSound = receiverPort.read(false);
        if (receiveSound!=NULL)
            put->renderSound(*receiveSound);
    }

    get->stopRecording(); //stops recording.
    return 0;
}

