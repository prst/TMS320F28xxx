//#include"F2802x_Device.h"
#include "include/F2802x_Device.h"     // DSP2802x Headerfile Include File

#include"init.h"


void DELAY(char s) {
  int i;

  //i = s * 100;
  //i = s * 10;
  i = s * 1;

  while(i--);
}

