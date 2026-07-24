#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file me446crs.c  You just need to create them here 
// and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.37;
float offset_Enc3_rad = 0.27;

// Your global varialbes.


long mycount = 0;

int UARTprint = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float printtau1 = 0;
float printtau2 = 0;
float printtau3 = 0;

// Assign these float to the values you would like to plot in Simulink
int WhatToSendToSimulink = 0; // Use this flag to change what is uploaded to the Simulink Plot
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

// this #pargma is Only used for the Matlab Functions given in Lab1's Appendix.
// You will more than likely not be using the Matlab Functions  
#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

// this #pargma and theta1array are Only used for the Matlab Functions given in Lab1's Appendix.
// You will more than likely not be using the Matlab Functions
#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];
long arrayindex = 0;


void mains_code(void);

//
// Main
//
void main(void)
{
	mains_code();
}




// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {
	
	// Calculate your Desired Trajectory

	
	// Calculate the States of your Controller equations

	
	// Calculate your Controller equations
    *tau1 = 0;
    *tau2 = 0;
    *tau3 = 0;

	
	// save past states so they can be used the next 1ms when lab() is called


    //Motor torque limitation(Max: 5 Min: -5)




	// This is the bottom of your Lab function.  
	// Blinking LEDS, Setting Tera Term print rate.  
    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }
	
	// Setting global variables for printing and debug
    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;
	// You can use these printtau global variables to print tau to Tera Term or 
	// Display tau in CCS Watch Expressions Window  You cannot put *tau in Watch Expressions.
	printtau1 = *tau1;
	printtau2 = *tau2;
	printtau3 = *tau3;

	// Sending Data to Simulink simulink5ms_plotAndGains.slx
	// Use WhatToSendToSimulink to send different variable values to Simuink for debug and report plotsq
	if (WhatToSendToSimulink == 0) {
		Simulink_PlotVar1 = theta1motor;
		Simulink_PlotVar2 = theta2motor;
		Simulink_PlotVar3 = theta3motor;
		Simulink_PlotVar4 = 0;
	} 
//	else if (WhatToSendToSimulink == 1) {  // Maybe here send errors
//		Simulink_PlotVar1 = ???;
//		Simulink_PlotVar2 = ???;
//		Simulink_PlotVar3 = ???;
//		Simulink_PlotVar4 = ???;
//	} else if (WhatToSendToSimulink == 2) {  // Maybe here send trajectory for debugging
//		Simulink_PlotVar1 = ???;
//		Simulink_PlotVar2 = ???;
//		Simulink_PlotVar3 = ???;
//		Simulink_PlotVar4 = ???;
//	}


// this is just an example of how to use the Matlab Functions given in Lab1's Appendix.
// You will more than likely not be using the Matlab Functions
// You can comment this code out if you would like.  
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;

        if (arrayindex >= 99) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

	
    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}

