
#include "init.h"


#if (1==__USE_DS1307__)

/* ******** I2C ************************************************************* */
#define I2C_RTC_SLAVE_ADD      0x68
#define I2C_EEPROM_SLAVE_ADD   0x50
#define I2C_IP_SLAVE_ADD       0x20   //PCA9554
#define I2C_LED_SLAVE_ADD      0x21   //PCA9554
#define I2C_NUMBYTES           1
#define I2C_RNUMBYTES          8
#define I2C_RTC_HIGH_ADDR      0x00
#define I2C_RTC_LOW_ADDR       0x0
#define I2C_EEPROM_HIGH_ADD    0x0
#define I2C_EEPROM_LOW_ADD     0x0
#define	Y2K		0x0037
#define	DW		0x0036
#define	YR		0x0035
#define	MO		0x0034
#define	DT		0x0033
#define	HR		0x0032
#define	MN		0x0031
#define	SC		0x0030

Uint16	YEAR       = 0x2009;
Uint16	MONTH      = 0x12;
Uint16	DAY        = 0x25;
Uint16	WEEK       = 0x01;
Uint16	HOUR       = 0x15;
Uint16	MINUTE     = 0x10;
Uint16	SECOND     = 0x00;
Uint16  MsgStatus  = 100;
Uint16  NumOfBytes = 2;
Uint16  MsgBuffer [7];
Uint16  date_initial[7] = { 0, 0x50, 0x14, 0x5, 0x25, 0x12, 0x09 };

struct I2CMSG  I2cMsgOut_RTC = {
		I2C_MSGSTAT_SEND_WITHSTOP,
		I2C_RTC_SLAVE_ADD,
		I2C_NUMBYTES,
		I2C_RTC_HIGH_ADDR,
		I2C_RTC_LOW_ADDR };
struct I2CMSG I2cMsgIn1 = {
		I2C_MSGSTAT_SEND_NOSTOP,
		I2C_RTC_SLAVE_ADD,
		I2C_RNUMBYTES,
		I2C_RTC_HIGH_ADDR,
		I2C_RTC_LOW_ADDR };
struct  I2CMSG  *CurrentMsgPtr;  // Used in interrupts

void   I2CA_Init (void);
Uint16 I2CA_WriteData (struct I2CMSG *msg);
Uint16 I2CA_ReadData (struct I2CMSG *msg);
void   WriteData (struct I2CMSG *msg, Uint16 *MsgBuffer, Uint16 MemoryAdd, Uint16 NumOfBytes);
interrupt void i2c_int1a_isr (void);
interrupt void i2c_int_isr (void);
void   RTC_Initial (void);
void   RTC_Write (void);
void   RTC_Read (void);
/* ******** I2C ************************************************************* */



void I2CA_Init(void)
{
   // Initialize I2C
   I2caRegs.I2CMDR.all = 0x0000;	// Take I2C reset
   									// Stop I2C when suspended

   I2caRegs.I2CFFTX.all = 0x0000;	// Disable FIFO mode and TXFIFO
   I2caRegs.I2CFFRX.all = 0x0040;	// Disable RXFIFO, clear RXFFINT,

   #if (CPU_FRQ_150MHZ)             // Default - For 150MHz SYSCLKOUT
        I2caRegs.I2CPSC.all = 14;   // Prescaler - need 7-12 Mhz on module clk (150/15 = 10MHz)
   #endif
   #if (CPU_FRQ_100MHZ)             // For 100 MHz SYSCLKOUT
     I2caRegs.I2CPSC.all = 9;	    // Prescaler - need 7-12 Mhz on module clk (100/10 = 10MHz)
   #endif
   I2caRegs.I2CPSC.all = 9;	    // Prescaler - need 7-12 Mhz on module clk (100/10 = 10MHz)
   I2caRegs.I2CCLKL = 10;			// NOTE: must be non zero
   I2caRegs.I2CCLKH = 5;			// NOTE: must be non zero
   I2caRegs.I2CIER.all = 0x24;		// Enable SCD & ARDY interrupts

   I2caRegs.I2CMDR.all = 0x0020;	// Take I2C out of reset
   									// Stop I2C when suspended

   I2caRegs.I2CFFTX.all = 0x6000;	// Enable FIFO mode and TXFIFO
   I2caRegs.I2CFFRX.all = 0x2040;	// Enable RXFIFO, clear RXFFINT,

   return;
}



Uint16 I2CA_WriteData(struct I2CMSG *msg)
{
   Uint16 i;

   if (I2caRegs.I2CMDR.bit.STP == 1)
   {
      return I2C_STP_NOT_READY_ERROR;
   }
   I2caRegs.I2CSAR = msg->SlaveAddress;
   // 检查总线是否繁忙
   if (I2caRegs.I2CSTR.bit.BB == 1)
   {
      return I2C_BUS_BUSY_ERROR;
   }
   I2caRegs.I2CCNT = msg->NumOfBytes+2;
   I2caRegs.I2CDXR = msg->MemoryHighAddr;
   I2caRegs.I2CDXR = msg->MemoryLowAddr;
// for (i=0; i<msg->NumOfBytes-2; i++)
   for (i=0; i<msg->NumOfBytes; i++)

   {
      I2caRegs.I2CDXR = *(msg->MsgBuffer+i);
   }
   // 作为主发送器开始发送
   I2caRegs.I2CMDR.all = 0x6E20;

   return I2C_SUCCESS;
}


Uint16 I2CA_ReadData(struct I2CMSG *msg)
{
   if (I2caRegs.I2CMDR.bit.STP == 1)
      return I2C_STP_NOT_READY_ERROR;
   I2caRegs.I2CSAR = msg->SlaveAddress;
   if(msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
   {
      if (I2caRegs.I2CSTR.bit.BB == 1)// 检查总线是否繁忙
         return I2C_BUS_BUSY_ERROR;
      I2caRegs.I2CCNT = 2;
      I2caRegs.I2CDXR = msg->MemoryHighAddr;
      I2caRegs.I2CDXR = msg->MemoryLowAddr;
      I2caRegs.I2CMDR.all = 0x2620;			// 作为主发送器开始发送Send data to setup RTC address
   }
   else if(msg->MsgStatus == I2C_MSGSTAT_RESTART)
   {
      I2caRegs.I2CCNT = msg->NumOfBytes;	// Setup how many bytes to expect
      I2caRegs.I2CMDR.all = 0x2C20;			// 作为主接收器接收Send restart as master receiver
   }

   return I2C_SUCCESS;
}

interrupt void i2c_int_isr(void)
{
   Uint16 IntSource,i;
   IntSource=I2caRegs.I2CISRC.all;//读中断源
   if(IntSource==I2C_SCD_ISRC)
     {
      if (MsgStatus == I2C_MSGSTAT_WRITE_BUSY)//写完成
          MsgStatus = I2C_MSGSTAT_INACTIVE;//总线空闲
      else
      {
         if(MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)//读数据操作里的写地址
         {
            MsgStatus = I2C_MSGSTAT_INACTIVE;//I2C_MSGSTAT_SEND_NOSTOP;
         }
         else if (MsgStatus == I2C_MSGSTAT_READ_BUSY)
         {
            MsgStatus = I2C_MSGSTAT_INACTIVE;//I2C_MSGSTAT_SEND_NOSTOP;//I2C_MSGSTAT_INACTIVE;
            for(i=0; i < NumOfBytes; i++)
            {
              MsgBuffer[i] = I2caRegs.I2CDRR;//读fifo
            }
		 }
      }

	 }
   else if(IntSource==I2C_ARDY_ISRC)//接收
     {
      if(I2caRegs.I2CSTR.bit.NACK == 1)
        {
         I2caRegs.I2CMDR.bit.STP = 1;
         I2caRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
        }
      else if(MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
      {
         MsgStatus = I2C_MSGSTAT_RESTART;
      }

	 }
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

interrupt void i2c_int1a_isr(void)     // I2C-A
{
   Uint16 IntSource, i;
   IntSource = I2caRegs.I2CISRC.all;//读中断源
   if(IntSource == I2C_SCD_ISRC) // 中断源 = 检测到停止条件  说明写数据完成
   {
      // If completed message was writing data, reset msg to inactive state
      if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY)
      {
         CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
      }
      else
      {
         if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
         {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
         }
         // If completed message was reading RTC data, reset msg to inactive state
         // and read data from FIFO.
         else if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY)
         {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;//I2C_MSGSTAT_INACTIVE;
            for(i=0; i < CurrentMsgPtr->NumOfBytes; i++)
            {
              CurrentMsgPtr->MsgBuffer[i] = I2caRegs.I2CDRR;
            }
		 }
      }
   }  // end of stop condition detected

   // Interrupt source = Register Access Ready
   // This interrupt is used to determine when the RTC address setup portion of the
   // read data communication is complete. Since no stop bit is commanded, this flag
   // tells us when the message has been sent instead of the SCD flag. If a NACK is
   // received, clear the NACK bit and command a stop. Otherwise, move on to the read
   // data portion of the communication.
   else if(IntSource == I2C_ARDY_ISRC)
   {
      if(I2caRegs.I2CSTR.bit.NACK == 1)
      {
         I2caRegs.I2CMDR.bit.STP = 1;
         I2caRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
      }
      else if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
      {
         CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_RESTART;
      }
   }  // end of register access ready

   else
   {
      // Generate some error due to invalid interrupt source
      asm("   ESTOP0");
   }

   // Enable future I2C (PIE Group 8) interrupts
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

void pass()
{
    asm("   ESTOP0");
    for(;;);
}

void fail()
{
    asm("   ESTOP0");
    for(;;);
}

void WriteData(struct I2CMSG *msg,Uint16 *MsgBuffer,Uint16 MemoryAdd,Uint16 NumOfBytes)
{
	Uint16 i,Error;
	for(i = 0; i < NumOfBytes; i++)
	{
		msg->MsgBuffer[i] = MsgBuffer[i];
	}
	msg->MemoryHighAddr = MemoryAdd >> 8;
	msg->MemoryLowAddr = MemoryAdd & 0xff;
	msg->NumOfBytes = NumOfBytes;
	Error = I2CA_WriteData(&I2cMsgOut_RTC);

	if (Error == I2C_SUCCESS)//正确发送
	{
		CurrentMsgPtr = &I2cMsgOut_RTC;
		I2cMsgOut_RTC.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;
	}
	while(I2cMsgOut_RTC.MsgStatus != I2C_MSGSTAT_INACTIVE);
	DELAY_US(1000);//延时1ms
}

#endif //(1==__USE_DS1307__)
