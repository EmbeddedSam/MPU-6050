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

int main()
{
    char buf[50]; //just to hold text values in for writing to UART
    
	int16_t ax, ay, az, i;
	int16_t gx, gy, gz;
    
	I2C_MPU6050_Start();
	SERIAL_Start();
	
    CyGlobalIntEnable;

	MPU6050_init();
	MPU6050_initialize();
	SERIAL_UartPutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");
    
    SERIAL_UartPutString("Raw values from gyroscope..\n\r");
        
    while(1)
    {
      MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      sprintf(buf, "AX:%d, AY:%d, AZ:%d || GX:%d, GY:%d, GZ:%d,\t", ax,ay,az,gx,gy,gz);
      SERIAL_UartPutString(buf);
      SERIAL_UartPutString("\n\r");
      CyDelay(300);
    }
    
    while(1){}
    
    //From here you will want to look at converting values to angles and then complimentary filters to combine the values
    //and to try and stop the drift which will inevitably happen on the gyroscope...
    
}
