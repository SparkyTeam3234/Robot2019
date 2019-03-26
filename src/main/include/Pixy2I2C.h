//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// Arduino I2C link class

#ifndef _PIXY2I2C_H
#define _PIXY2I2C_H

#include "TPixy2.h"
#include "frc/I2C.h"
#include <hal/I2CTypes.h>

#define PIXY_I2C_DEFAULT_ADDR           0x54  
#define PIXY_I2C_MAX_SEND               16 // don't send any more than 16 bytes at a time

class Link2I2C
{
public:
  int8_t open(uint32_t arg) // take I2C address as argument to open
  {
    if (arg==PIXY_DEFAULT_ARGVAL)
      m_addr = PIXY_I2C_DEFAULT_ADDR;
    else
      m_addr = arg;
    wire = new frc::I2C(frc::I2C::Port::kOnboard,m_addr);
    wpi::errs() << (wire->AddressOnly() ? "\nI2C Open Success" : "\nI2C Open Success") << "\n";
	return 0;
  }
	
  void close()
  {
    delete wire;
  }
    
  int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs=NULL, bool debug=false)
  {
    //wpi::errs() << "Pixy2I2C::recv\n";
    uint8_t i, j, n;
    uint8_t *buffer = new uint8_t[len];
    if (cs)
      (*cs) = 0;
    for (i=0; i<len; i+=n)
    {
      /*
        wpi::errs() << "Recieve Iteration " << (uint16_t) i << "\n";
      */
      uint8_t *bufferpointer = buffer;
      // n is the number read -- it most likely won't be equal to len
      //(uint8_t)m_addr,
      //n = !wire->ReadOnly((uint8_t)(len-i),buffer);
      if (debug) wpi::errs() << "\nReadOnly";
      bool success = !wire->ReadOnly(len,buffer);
      if (success) 
      {
        n=len;
      } 
      else 
      {
        len=-1;
        wpi::errs() << "\n!!!ERROR IN Wire->ReadOnly!!!";
        break;
      }
      if (debug) wpi::errs() << "...returned\n";
      for (j=0; j<n; j++)
      {		  
        buf[j+i] = *bufferpointer;
        //wpi::errs() << "i " << (uint16_t) i << " - " << (uint16_t) buf[j+i] << "\n";
        if (cs)
          (*cs) += buf[j+i];
        bufferpointer++;
      }	  
    }
    delete buffer;
    return len;
  }
    
  int16_t send(uint8_t *buf, uint8_t len)
  {
    int8_t i, packet;
	  for (i=0; i<len; i+=PIXY_I2C_MAX_SEND)
    {
      if (len-i<PIXY_I2C_MAX_SEND)
        packet = len-i;
      else 
        packet = PIXY_I2C_MAX_SEND;

      wire->WriteBulk(buf+i, packet);
    }
    return len;
  }
	
private:
  uint8_t m_addr;
  frc::I2C *wire=0;
};


typedef TPixy2<Link2I2C> Pixy2I2C;


#endif
