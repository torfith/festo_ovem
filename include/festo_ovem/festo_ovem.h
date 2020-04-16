#pragma once

#include <iolink/device.h>

class FestoOvem : private iolink::Device
{
public:
    FestoOvem();

    FestoOvem(iolink::Port port);

    void setSuctionOn();

    void setSuctionOff();

    void setEjectionOn();

    void setEjectionOff();

    float pressureBar();

    bool isOutA();

    bool isOutB();
};