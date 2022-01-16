#include "multiCommClass.h"
#include <stdio.h>
#include <unistd.h>

IDDPComm::IDDPComm(int ownPort, int destPort, int _size, int _parameters[])
{
    sendSize = _size;
    receiveSize = _size;
    destinationPort = destPort;
    sendParameters = _parameters; // For IDDPComm, we use either send or receive parameters, so they can safely share one constructor argument.
    receiveParameters = _parameters;
    xenoCommClass = new IDDPconnection(ownPort);
}

IDDPComm::~IDDPComm()
{
    xenoCommClass->~xenoCommunication(); // Shouldn't this be `delete xenoCommClass`?
}

XDDPComm::XDDPComm(int ownPort, int destPort, int _size, int _parameters[])
{
    sendSize = _size;
    receiveSize = _size;
    destinationPort = destPort;
    sendParameters = _parameters;
    receiveParameters = _parameters;
    xenoCommClass = new XDDPconnection(ownPort);
}

XDDPComm::~XDDPComm()
{
    xenoCommClass->~xenoCommunication();
}

void frameworkComm::setVerbose(bool _verbose)
{
    verbose = _verbose;
    xenoCommClass->setVerbose(verbose);
}

int frameworkComm::send(double value[])
{
    //printf("send function of IDDP\n");
    double toSend[sendSize];
    int ret;
    for (int i = 0; i < sendSize; i++)
    {
        if (sendParameters[i]!=-1)
            toSend[i] = value[sendParameters[i]];
        //printf("value on %d is: %f\n", sendParameters[i], value[sendParameters[i]]);
    }
    ret = xenoCommClass->sendDoubleArray(toSend, destinationPort, sendSize);
    return ret;
}

int frameworkComm::receive(double value[])
{
    double received[receiveSize];
    int ret;
    ret = xenoCommClass->receiveDoubleArray(received, receiveSize);
    if (ret > 0)
    {
        for (int i = 0; i < receiveSize; i++)
        {
            if (receiveParameters[i]!=-1)
                value[receiveParameters[i]] = received[i];
        }
    }
    return ret;
}
