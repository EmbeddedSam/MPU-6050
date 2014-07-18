/* ========================================
 *
 * Copyright Samuel Walsh, 2014
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Samuel Walsh.
 *
 * ========================================
*/
#include <project.h>
#include <mpu6050.h>
#include <stdio.h>

#define numberOfTests   100

char buf[200]; //just to hold text values in for writing to UART
int i = 0; //for loop increment variable
   
int16_t CAX, CAY, CAZ; //current acceleration values
int16_t CGX, CGY, CGZ; //current gyroscope values
int16_t CT;            //current temperature
   
float   AXoff, AYoff, AZoff; //accelerometer offset values
float   GXoff, GYoff, GZoff; //gyroscope offset values
    
int main()
{ 
	I2C_MPU6050_Start();
	SERIAL_Start();
	
    CyGlobalIntEnable;

	MPU6050_init();
	MPU6050_initialize();
	SERIAL_UartPutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");
    
    SERIAL_UartPutString("Starting to calibrate values from sensor..\n\r");
     
    //Count and average the first n values, defined by numberOfTests above..
    for(i=0; i<numberOfTests; i++)
    {
      sprintf(buf, "Test Number: %d \n\r", i);
      SERIAL_UartPutString(buf);
    
      MPU6050_getMotion6t(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ, &CT);
      AXoff += CAX;
      AYoff += CAY;
      AZoff += CAZ;
      GXoff += CGX;
      GYoff += CGY;
      GZoff += CGZ;
       
      sprintf(buf, "AX:%d, AY:%d, AZ:%d || GX:%d, GY:%d, GZ:%d,\t", CAX,CAY,CAZ,CGX,CGY,CGZ);
      SERIAL_UartPutString(buf);
      SERIAL_UartPutString("\n\r");
      CyDelay(25);
    }
    
    AXoff = AXoff/numberOfTests;
    AYoff = AYoff/numberOfTests;
    AZoff = AZoff/numberOfTests;
    GXoff = GXoff/numberOfTests;
    GYoff = GYoff/numberOfTests;
    GZoff = GZoff/numberOfTests;
    
    SERIAL_UartPutString("\n\nTest finished, offset values are shown below\n\n\r");
    sprintf(buf, "AXoff:%d, AYoff:%d, AZoff:%d || GXoff:%d, GYoff:%d, GZoff:%d,\t", (int)AXoff,(int)AYoff,(int)AZoff,(int)GXoff,(int)GYoff,(int)GZoff);
    SERIAL_UartPutString(buf);
    
    while(1){}
    //From here you will want to look at converting values to angles and then complimentary filters to combine the values
    //and to try and stop the drift which will inevitably happen on the gyroscope...
    
}
