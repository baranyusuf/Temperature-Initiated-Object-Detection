#include "TM4C123GH6PM.h"  
#include <stdint.h>
#include <stdio.h>
#include "algorithm.h"


float sensor_sum = 0;
float sensor_sum_frac = 0;
float avg_sum = 0;


// Main function
int main(void) {
		//pulse_init();
		GPIO_B_INITIALIZE();
		Button_Init();
    DeepSleep_Init();
		GPIOB_Interrupt_Init();
		GPIO_Init_for_Analog_Comparator(); 
		AnalogComparator_Init();	   
		__asm("NOP");
  	__asm("NOP");
		LED_Init();

		ADC_Init();  
		ADC_Init_PE4();
		PortC_Init();
		
		// Initialize GPIO for TRIG pin
    //TRIG_Init();
		__asm("NOP");
  	PortE_Init(); 
    //TIMER2_Init();

    //SysTick_Init(16000000 / stepRates[currentRate]); // Initialize SysTick
		//TIMER2->CTL |= 0x01;       // Enable Timer1A
		SysTick_Init();	
		
		//pulse_init();
		//TIMER2->CTL &= ~0x0100; 
    GPIOB->DATA &= ~0x02; 
		I2C3_Init();
		LOW = 1000000000;
		HIGH = 1000000000;
		
		Nokia5110_Init();
		Nokia5110_Clear();
		//Nokia5110_OutString("************* LCD Test *************Letter: Num:------- ---- ");
		//Nokia5110_OutString("**Voltage** -----------");
		Nokia5110_OutChar(95);
		//SysTick_Init();
;
		while (1) {
			__asm("WFI");
			switch(count_case){
				case 0:
					SysTick_Init();
					COMP->ACINTEN |= (1 << 0);
					__asm("NOP");
					__asm("NOP");
				break;
				case 1:
					SysTick_Init();
					sensor_sum = Read_IC();
					
					avg_sum = sensor_sum/128.0000;
					LOW = (1000000)/(2*(2100+40*(avg_sum-20)));
					HIGH = LOW;
					pulse_init();
					//TIMER2->TBILR = HIGH;
	
					run_time = 0;              
					//TIMER2->CTL |= 0x0100; 
					if (avg_sum >= result_threshold){
						__asm("NOP");
						__asm("NOP");
						count_case = 2;
					}
					else{ 
						delay_ms(1000);
						COMP->ACINTEN |= (1 << 0);
						count_case = 0;
						Set_PC2_Low();
					}
				break;
					
				case 2:
					SysTick_Init();
				    TRIG_Init();
					__asm("NOP");
					__asm("NOP"); 
					TIMER2_Init();
					
					//alg_motor_sensor();
					//count_case++;
				break;
				
				case 3:
					SysTick_Init();
					Set_LED_Color(distance);
					delay_ms(5000);
					//SysTick_Init();
				break;
				
				case 4:
					SysTick_Init();
					Nokia5110_Clear();
					Set_LED_Color(distance);
					
					
					COMP->ACINTEN |= (1 << 0);
					delay_ms(3000);
					Turn_Off();
					count_case = 0;
					stepCount = 0;
					Set_PC2_Low();
				break;
			}














			
			}
}