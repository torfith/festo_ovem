/** \file
 * \brief festo_ovem_test based on example code for Simple Open EtherCAT master
 *
 * Usage : festo_ovem_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Torfi Thorhallsson 2020
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>

#include <ethercat/master.h>
#include <iolink/master.h>
#include <festo_ovem/festo_ovem.h>

int main(int argc, char *argv[])
{
   printf("festo_ovem\nFesto OVEM test\n");

   if (argc > 1)
   {
      ethercat::Master em;
      em.monitor();  // create thread to handle slave errors in OP
      em.open(argv[1]);

      iolink::Master im;
      im.open();  // autodetect IO-Link master

      iolink::Port port = im.port(iolink::PortName::X01);
      FestoOvem ovem(port);
   
      /* cyclic loop */
      for (int i = 1; i <= 10000; i++)
      {
         if (em.spinOnce())
         {
            //printf("Processdata cycle %4d, ", i);
            // em.printProcessData();
            if (port.isDeviceAvailable())
            {
               if (port.isInputValid())
               {
                  printf("timeStampNs: %ld \n", port.timeStampNs());
                  printf("isOnDigitalInput2: %d \n", port.isOnDigitalInput2());
                  printf("isOnDigitalInput4: %d \n", port.isOnDigitalInput4());
                  // printf("O: %x ", port.processDataOut()[0]);
                  // printf("I: %x ", port.processDataIn()[0]);
                  // printf("%x \n", port.processDataIn()[1]);

                  printf("isOutA: %d\n", ovem.isOutA());
                  printf("isOutB: %d\n", ovem.isOutB());
                  printf("pressureBar: %f\n", ovem.pressureBar());

                  switch ((i / 100) % 4)
                  {
                  case 0:
                     ovem.setSuctionOn();
                     printf("setSuctionOn\n");
                     break;
                  case 1:
                     ovem.setSuctionOff();
                     printf("setSuctionOff\n");
                     break;
                  case 2:
                     ovem.setEjectionOn();
                     printf("setEjectionOn\n");
                     break;
                  case 3:
                     ovem.setEjectionOff();
                     printf("setEjectionOff\n");
                     break;
                  }
               }
            }
            if (port.isDeviceError())
            {
               printf("\n");
               printf("X01 Error Code: %x\n", port.errorCode());
            }
            osal_usleep(5000);
         }
      }
   }
   else
   {
      printf("Usage: festo_ovem_test ifname\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}
