/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>
#include <interrupts.h>
#include <mpu6050.h>
#include <stdio.h>
#include <math.h>

#define forever 1
#define numberOfTests   100

void calibrateOffsets(void);

uint8 flag = 0;

char buf[200];    //general buffer for messages
char Abuf[200];   //to hold acceleration value text

int i = 0; //for loop increment variable
   
int16_t CAX, CAY, CAZ; //current acceleration values
int16_t CGX, CGY, CGZ; //current gyroscope values
int16_t CT;            //current temperature
   
float   AXoff, AYoff, AZoff; //accelerometer offset values
float   GXoff, GYoff, GZoff; //gyroscope offset values

float   AX, AY, AZ; //acceleration floats
float   GX, GY, GZ; //gyroscope floats

float   Roll,Pitch,Yaw;

int main()
{
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    uint8 volatile ch;  
    /* Start the SCB UART */
    UART_Start(); 
    /* Start the Interrupt */
    UART_RX_ISR_StartEx(uartInterrupt);  
    /* Transmit string through UART */
    UART_UartPutString("UART Initialised");

    CyGlobalIntEnable; /* Uncomment this line to enable global interrupts. */
    
    I2C_MPU6050_Start();

	MPU6050_init();
	MPU6050_initialize();
	UART_UartPutString(MPU6050_testConnection() ? "MPU6050 connection successful\n\r" : "MPU6050 connection failed\n\n\r");
    
    UART_UartPutString("Starting to calibrate values from sensor..\n\r");
    calibrateOffsets(); //work out our offsets
    
    while(forever)
    {         
      //Convert values to G's
      MPU6050_getMotion6t(&CAY, &CAX, &CAZ, &CGX, &CGY, &CGZ, &CT);
      AX = ((float)CAX-AXoff)/16384.00;
      AY = ((float)CAY-AYoff)/16384.00; //16384 is just 32768/2 to get our 1G value
      AZ = ((float)CAZ-(AZoff-16384))/16384.00; //remove 1G before dividing
    
      GX = ((float)CGX-GXoff)/131.07; //131.07 is just 32768/250 to get us our 1deg/sec value
      GY = ((float)CGY-GYoff)/131.07;
      GZ = ((float)CGZ-GZoff)/131.07; 
    
      //UART_UartPutString(buf);
    
      Roll  = atan2f(AY, AZ) * 180/M_PI;
      Pitch = atan2f(AX, sqrt(AY*AY + AZ*AZ)) * 180/M_PI;    
    
      sprintf(Abuf, "%f,%f,%f\t\r\n", AX,AY,AZ);
        
      if(flag == 1)
      {
        flag = 0;
        ch = UART_UartGetChar();
        if(ch == 'a') //asking for acceleration values
        {
          UART_UartPutString(Abuf);   
        }
        TX_LED_Write(!TX_LED_Read());  
      }
        
          //CyDelay(500);
    }
}

void calibrateOffsets(void)
{
  //Count and average the first n values, defined by numberOfTests above..
    for(i=0; i<numberOfTests; i++)
    {  
      MPU6050_getMotion6t(&CAX, &CAY, &CAZ, &CGX, &CGY, &CGZ, &CT);
      AXoff += CAX;
      AYoff += CAY;
      AZoff += CAZ;
      GXoff += CGX;
      GYoff += CGY;
      GZoff += CGZ;
      
      //Uncomment these lines to show serial output...
      //sprintf(buf, "Test Number: %d \n\r", i);
      //SERIAL_UartPutString(buf);
      //sprintf(buf, "AX:%d, AY:%d, AZ:%d || GX:%d, GY:%d, GZ:%d,\t", CAX,CAY,CAZ,CGX,CGY,CGZ);
      //SERIAL_UartPutString(buf);
      //SERIAL_UartPutString("\n\r");
      CyDelay(25);
    }
    
    AXoff = AXoff/numberOfTests;
    AYoff = AYoff/numberOfTests;
    AZoff = AZoff/numberOfTests;
    GXoff = GXoff/numberOfTests;
    GYoff = GYoff/numberOfTests;
    GZoff = GZoff/numberOfTests;
    
    UART_UartPutString("\nTest finished, offset values are shown below\n\r");
    sprintf(buf, "\n\rAXoff:%d, AYoff:%d, AZoff:%d || GXoff:%d, GYoff:%d, GZoff:%d,\t\n\r", (int)AXoff,(int)AYoff,(int)AZoff,(int)GXoff,(int)GYoff,(int)GZoff);
    UART_UartPutString(buf);  
}


/* [] END OF FILE */
