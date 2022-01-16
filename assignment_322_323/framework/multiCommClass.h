#ifndef MULTICOMMCLASS_H
#define MULTICOMMCLASS_H

#include "communicationClass.h"

/**
 * @brief Class which provides top level access to XDDP/IDDP
 * TODO: Integrate with xenoCommunication classes. 
 * 
 */

class frameworkComm
{
public:
    /**
     * @brief Send value to predetermined port and with predetermined parameters. 
     * 
     * @param value value which needs to be send.
     * @return int: size of values send. 
     */
    int send(double value[]);

    /**
     * @brief Receive function which receives an array of data and writes it over the specified indexes of the output value. 
     * Port is pre-assigned. 
     * @param value output array. 
     * @param parameters array filled with indexes of where the receives array is put. (in order of received array)
     * @return int: size of received values. 
     */
    // int receive(double value[], int parameters[]);

    /**
     * @brief receive value to predetermined port and with predetermined parameters. 
     * Port is pre-assigned. 
     * @param value array which is filled with values according to predetermined parameters. 
     * @return int: size of received values. 
     */
    int receive(double value[]);
    
    /**
     * @brief Set the function to be Verbose or not. 
     * 
     * @param verbose value of verbosity. 
     */
    void setVerbose(bool verbose);

    virtual ~frameworkComm() {}

protected:
    int *sendParameters;
    int *receiveParameters;
    int destinationPort;
    int receiveSize;
    int sendSize;
    xenoCommunication *xenoCommClass;
    bool verbose = false;
};

class IDDPComm : public frameworkComm
{
public:
    IDDPComm(int ownPort, int destPort, int _size, int _parameters[]);
    ~IDDPComm();
};

class XDDPComm : public frameworkComm
{
public:
    XDDPComm(int ownPort, int destPort, int _size, int _parameters[]);
    ~XDDPComm();
};

#endif