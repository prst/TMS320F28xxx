//#############################################################################
//
//! \file   f2802x_common/source/sci_io.c
//!
//! \brief  Contains the various functions related to the serial 
//!         communications interface (SCI) object
//
//  Group:          C2000
//  Target Device:  TMS320F2802x
//
//#############################################################################
// $TI Release: F2802x Support Library v222 $
// $Release Date: Thu Jan 15 13:56:57 CST 2015 $
// $Copyright: Copyright (C) 2008-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

// **************************************************************************
// the includes
#include <stdio.h>
#include <file.h>

#include "../../include/DSP28x_Project.h"
#include "../../include/sci.h"
#include "../../include/sci_io.h"


// **************************************************************************
// the defines

#if (1==USE_F28027_SCI_IO)

// **************************************************************************
// the globals

uint16_t deviceOpen = 0;
SCI_Handle ioSci;


// **************************************************************************
// the functions

int SCI_open(const char * path, unsigned flags, int llv_fd)
{
    if(deviceOpen){
        return (-1);
    }else{
        ioSci = SCI_init((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
        deviceOpen = 1;
        return (1);    
    }    
    
}

int SCI_close(int dev_fd)
{
    if((dev_fd != 1) || (!deviceOpen)){
        return (-1);
    }else{
        deviceOpen = 0;
        return (0);
    }    
    
}

int SCI_read(int dev_fd, char * buf, unsigned count)
{
    uint16_t readCount = 0;
    uint16_t * bufPtr = (uint16_t *) buf;
    
    if(count == 0) {
        return (0);
    }
    
    while((readCount < count) && SCI_isRxDataReady(ioSci)){
        *bufPtr = SCI_getData(ioSci);
        readCount++;
        bufPtr++;
    }
    
    return (readCount);
    
}

//int SCI_write(int dev_fd, char * buf, unsigned count)
int SCI_write(SCI_Handle sciHandle, uint16_t data)
{
    uint16_t writeCount = 0;
    //uint16_t * bufPtr = (uint16_t *) buf;
    uint16_t *bufPtr;


    bufPtr = &data;
    
    /*if(count == 0) {
        return (0);
    }*/
    
    ioSci = sciHandle;
    //while(writeCount < count)
    {
        SCI_putDataBlocking(ioSci, *bufPtr);
        writeCount++;
        bufPtr++;
    }
    
    return (writeCount);
}

off_t SCI_lseek(int dev_fd, off_t offset, int origin)
{
    return (0);   
}

int SCI_unlink(const char * path)
{
    return (0);
}

int SCI_rename(const char * old_name, const char * new_name)
{
    return (0);    
}

#endif //(1==USE_F28027_SCI_IO)

// end of file
