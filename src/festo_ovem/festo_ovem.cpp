#include <festo_ovem/festo_ovem.h>

#include <cstdint>

using namespace std;

static const uint16_t PDV  = 0xfffc;
static const uint8_t BDC3 = 0x02;
static const uint8_t BDC1_BDC2 = 0x01;

static const uint8_t BCS2 = 0x02;
static const uint8_t BCS1 = 0x01;

FestoOvem::FestoOvem()
{
}

FestoOvem::FestoOvem(iolink::Port port) : iolink::Device(port)
{
}

bool FestoOvem::isOutA()
{
    return processDataIn()[1] & BDC1_BDC2;
}

bool FestoOvem::isOutB()
{
    return processDataIn()[1] & BDC3;
}

float FestoOvem::pressureBar()
{
    uint16_t data = (processDataIn()[0] << 8) +
                    (processDataIn()[1] << 0);
    uint16_t value = (data & PDV) >> 2;
    return -value/16383.;
}

void FestoOvem::setSuctionOn()
{
    processDataOut()[0] |= BCS1;
}

void FestoOvem::setSuctionOff()
{
    processDataOut()[0] &= !BCS1;
}

void FestoOvem::setEjectionOn()
{
    processDataOut()[0] |= BCS2;
}

void FestoOvem::setEjectionOff()
{
    processDataOut()[0] &= !BCS2;
}