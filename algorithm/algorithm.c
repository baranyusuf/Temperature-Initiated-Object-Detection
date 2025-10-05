#include "TM4C123GH6PM.h"  
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "algorithm.h"
#include <math.h>

// ------------------------ Cartesian -----------------------
#define SCREEN_WIDTH  84
#define SCREEN_HEIGHT 48

uint32_t x, y;

uint8_t cartesian[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3F,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00
};

// --------------- Variables ----------------------------

float buffer_distance[1000];
int16_t result; // Define initially.
int16_t adcValue;   // Variable to store ADC result
float voltage;
uint16_t result_th;
uint16_t adcValue_th;
float voltage_th;

int sensor_count = 0;
int distance_count = 0;
float last_angle = 0;
int stepCount = 0;

int button_case = 0;

// Buzzer
int LOW = 1000000000;
int HIGH = 1000000000;
volatile int run_time;


int read_button = 0;
int start_motor = 0;

int count_case = -1;

uint8_t step = 1;        // Initial step
uint32_t risingEdge = 0, fallingEdge = 0;
float pulse_Width_motor = 0;
float distance = 0;
char formatted_distance[20];
char formatted_angle[20];
float angle = 0;


float value_1;
float value_2;
float value_3;
float result_threshold = 99.9;
// Define memory addresses
volatile uint32_t *ADDR_1 = (uint32_t *)0x20000910;
volatile uint32_t *ADDR_2 = (uint32_t *)0x20000914;
volatile uint32_t *ADDR_3 = (uint32_t *)0x20000918;

// Keypad mapping
int keypad_map[3][4] = {
    {0x11, 0x12, 0x14, 0x18},  // Row 1
    {0x21, 0x22, 0x24, 0x28},  // Row 2
    {0x41, 0x42, 0x44, 0x48},  // Row 3
};

// Corresponding integer values
int key_values[3][4] = {
    {1, 2, 3, 4},      // Row 1 values
    {5, 6, 7, 8},      // Row 2 values
    {9, 0, 0, 0},    // Row 3 values (-1 for null keys)
};

volatile uint32_t *recorded_value = (uint32_t *)0x20000011; ///////////////////////////////  eski deger:  0x20000010

// --------------- Buzzer ------------------------

void pulse_init(void) {
    volatile int *NVIC_PRI5 = (volatile int *)0xE000E414; 
    volatile int *NVIC_EN0 = (volatile int *)0xE000E100;  

    SYSCTL->RCGCGPIO |= 0x02;
    __ASM("NOP");
    __ASM("NOP");
    __ASM("NOP");

 
    GPIOB->DIR |= 0x02;         
    GPIOB->AFSEL &= ~0x02;      
    GPIOB->PCTL &= 0xFFFFFF0F; 
    GPIOB->DEN |= 0x02;         

    SYSCTL->RCGCTIMER |= 0x04;
    __ASM("NOP");
    __ASM("NOP");
    __ASM("NOP");

    TIMER2->CTL &= ~0x0100;    
    TIMER2->CFG = 0x04;        
    TIMER2->TBMR = 0x02;       
    TIMER2->TBILR = LOW;       
    TIMER2->TBPR = 15;         
    TIMER2->IMR |= 0x0100;     

    
    *NVIC_PRI5 &= ~0x00E00000; 
    *NVIC_PRI5 |= 0x00400000;  
    *NVIC_EN0 |= 0x01000000;   


    TIMER2->CTL |= 0x0100;     
}

void TIMER2B_Handler(void) {
    
    TIMER2->ICR |= 0x0100;  

    static int state = 0;  
		if (run_time >= 5000) { 
        TIMER2->CTL &= ~0x0100; 
        GPIOB->DATA &= ~0x02;
				LOW = 1000000000;
				HIGH = 1000000000;
        return;
    }
	

    if (state == 0) {
      
        GPIOB->DATA |= 0x02;   
        TIMER2->TBILR = HIGH; 
        state = 1;           
    } else {
        
        GPIOB->DATA &= ~0x02;  
        TIMER2->TBILR = LOW; 
        state = 0;            
    }
		run_time++;

    return;
}


// --------------- Digital Sensor -----------------
//

double Tempreture;
long signed int temporary;
int32_t temporary_2 = 0;
uint8_t val3 ;
double var1,var2;
char I2C3_Write_Multiple(int slave_address, char register_adress, int bytes_count, char data);
char I2C3_read_Multiple(int slave_address, char register_adress, int bytes_count, char* array);

void I2C_busy(void)
{
    while(I2C3->MCS & 1){
		__ASM("NOP");
		}			
    return ;
}

char I2C3_Write_Multiple(int slave_address, char register_address, int bytes_count, char data)
{
    I2C3->MSA = (slave_address << 1);
    I2C3->MDR = register_address;
    I2C3->MCS = (1 << 1) | (1 << 0);  
    I2C_busy();

    for (int i = 0; i < bytes_count; i++) {  
        I2C3->MDR = data;
        if (i == bytes_count - 1) {
            I2C3->MCS = (1 << 2) | (1 << 0); 
        } else {
            I2C3->MCS = (1 << 0); 
        }
        I2C_busy();
    }

    while (I2C3->MCS & (1 << 6));  
    return 0; 
}


char I2C3_Read_Multiple(int slave_address, char register_address, int bytes_count, char* array)
{

    if (bytes_count <= 0) {
        return -1; 
    }

    I2C3->MSA = (slave_address << 1); 
    I2C3->MDR = register_address;
    I2C3->MCS = (1 << 1) | (1 << 0); 
    I2C_busy();

    I2C3->MSA = (slave_address << 1) | 1; 

    if (bytes_count == 1) {
        I2C3->MCS = (1 << 2) | (1 << 1) | (1 << 0); 
    } else {
        I2C3->MCS = (1 << 1) | (1 << 0); 
    }
    I2C_busy();


    *array++ = I2C3->MDR;
    bytes_count--;


    while (bytes_count > 1) {
        I2C3->MCS = (1 << 3) | (1 << 0); 
        I2C_busy();
        *array++ = I2C3->MDR;
        bytes_count--;
    }


    I2C3->MCS = (1 << 2) | (1 << 0);
    I2C_busy();
    *array = I2C3->MDR;
    while (I2C3->MCS & (1 << 6)); 
    return 0; 
}


void I2C3_Init ( void )
{
SYSCTL->RCGCGPIO  |= 0x00000008 ; 
SYSCTL->RCGCI2C   |= 0x00000008 ; 
GPIOD->DEN |= 0x03; 
GPIOD->AFSEL |= 0x00000003 ;
GPIOD->PCTL |= 0x00000033 ;
GPIOD->ODR |= 0x00000002 ; 
I2C3->MCR  = 0x0010 ; 
I2C3->MTPR  = 0x07 ;
}

int Read_IC(void){
	float sum = 0;	
	
	for (int i = 0; i<128; i++){
			char array[2];
			I2C3_Write_Multiple(0x76, 0xF5, 1, 0x00);
			I2C3_Write_Multiple(0x76, 0xF4, 1, 0x23);
			I2C3_Read_Multiple(0x76, 0XFA, 2,array  ); 

			temporary_2 =(uint32_t) (array[0] << 12) | (array[1] << 4) | val3;
			var1=(((double)temporary_2)/16384.0-(27504)/1024.0)*(26435);
		  var2=((((double)temporary_2)/131072.0 - (27504)/8192.0)*(((double)temporary_2)/131072.0 - (27504)/8192.0))*(-1000);
		  temporary=(long signed int)(var1+var2);
		  Tempreture =(var1+var2)/5120.0;	
			sum+=Tempreture;
}
		return sum;
		
}

// ---------------- KEYPAD FUNCTION -----------------
// --------------------------------------------------

void GPIO_Init_for_Analog_Comparator(void) {
    SYSCTL->RCGCGPIO |= 0x04;            
    while ((SYSCTL->PRGPIO & 0x04) == 0); 
    GPIOC->DIR &= ~0xC0;                
    GPIOC->AFSEL |= 0xC0;                 // This GPIO Initializer sets PC6    
    GPIOC->DEN &= ~0xC0;                  // This GPIO Initializer sets PC7    
    GPIOC->AMSEL |= 0xC0;               
    GPIOC->PCTL &= ~0xFF000000;        
    GPIOC->PCTL |= 0x00000000;          
}

void DeepSleep_Init(void) {
    SCB->SCR |= 0x04;                  // Set SLEEPDEEP bit in System Control Register
																			 // __asm("WFI");  <== use this command in the main block to begin deep sleep mode	
}

void AnalogComparator_Init(void) {
    SYSCTL->RCGCACMP |= 0x01;          // run mode clock
    while ((SYSCTL->PRACMP & 0x01) == 0); 
    
		SYSCTL->RCGC1 |= (1 << 24);        // Enable clock for COMP0 
    SYSCTL->DCGCACMP |= 0x01;          // deep sleep mode clock
    while ((SYSCTL->DCGCACMP & 0x01) == 0);
    COMP->ACCTL0 &= ~(0x03 << 2);     
    COMP->ACCTL0 |= (0x00 << 2);      
    
		COMP->ACCTL0 &= ~(1 << 4);  
    COMP->ACCTL0 |= (1 << 4);  
	
		COMP->ACCTL0 &= ~(1 << 3);  
    COMP->ACCTL0 |= (1 << 3);  
    
		COMP->ACCTL0 &= ~(0x03 << 9);      
    COMP->ACCTL0 |= (0x00 << 9);       																																
    
		COMP->ACINTEN |= (1 << 0);         
    
		NVIC_EnableIRQ(COMP0_IRQn );        // Enable COMP0 interrupt in NVIC
		NVIC_SetPriority(COMP0_IRQn, 3);  // Set priority for Comparator interrupt
}

void GPIOB_Interrupt_Init(void) {

    // Configure PB7, PB6, PB5, and PB4 for interrupt
    GPIOB->IS |= ((1 << 5) );  // Edge-sensitive
    GPIOB->IBE &= ~((1 << 5)); // Interrupt on one edge
    GPIOB->IEV |= ((1 << 5) ); // Falling edge trigger
    GPIOB->IM |= ((1 << 5) );   // Unmask interrupts

    // Enable interrupt in NVIC
    NVIC_EnableIRQ(GPIOB_IRQn);  // Enable Port B interrupt in NVIC
		NVIC_SetPriority(GPIOB_IRQn, 2);  // Set priority for GPIO Port B interrupt
}

void COMP0_Handler(void) {
		COMP->ACMIS |= (1 << 0);  // Clear interrupt then start the action
		
		if (button_case == 1){
			count_case = 0;
			button_case == 0;
			
		}
	
		if (count_case == -1) {
				count_case = 0;
		}
		else {
			COMP->ACINTEN &= ~(1 << 0);
			count_case=1;
			Set_PC2_High();
		}

		//start_motor++;
		

}

void GPIOB_Handler(void) {
    
				delay_ms(200);
        keypad();
				GPIOD->DATA &= ~0xCC;
				__asm("NOP");
				__asm("NOP");
				__asm("NOP");
				__asm("NOP");
				__asm("NOP");
				__asm("NOP");
				__asm("NOP");
				__asm("NOP");
				

				GPIOD->DATA |= 0xCC;
				GPIOB->ICR |= 0x20;  // Clear PB7 interrupt flag
				uint32_t interrupt_status = GPIOB->MIS;
				if (interrupt_status & 0x20) {
						GPIOB->ICR |= 0x20;  // Clear PB7 interrupt flag
				}
				__asm("NOP");
				__asm("NOP");
				delay_ms(200);
				
    // Read the values from memory
    uint32_t key_1 = *ADDR_1;
		delay_ms(2);
    uint32_t key_2 = *ADDR_2;
		delay_ms(2);
    uint32_t key_3 = *ADDR_3;
		delay_ms(2);
    // Get the corresponding integer values
		delay_ms(2);
    
		if (key_1 == 0x12){
			value_1 = 1;
		}		
		
		if (key_1 == 0x18){
			value_1 = 2;
		}
		
		if (key_1 == 0x11){
			value_1 = 3;
		}
		
		if (key_1 == 0x14){
			value_1 = 4;
		}
		
		if (key_1 == 0x22){
			value_1 = 5;
		}
		
		if (key_1 == 0x28){
			value_1 = 6;
		}
		
		if (key_1 == 0x21){
			value_1 = 7;
		}
		
		if (key_1 == 0x24){
			value_1 = 8;
		}
		
		if (key_1 == 0x42){
			value_1 = 9;
		}
		
		if (key_1 == 0x48){
			value_1 = 0;
		}
		
		if (key_2 == 0x12){
			value_2 = 1;
		}		
		
		if (key_2 == 0x18){
			value_2 = 2;
		}
		
		if (key_2 == 0x11){
			value_2 = 3;
		}
		
		if (key_2 == 0x14){
			value_2 = 4;
		}
		
		if (key_2 == 0x22){
			value_2 = 5;
		}
		
		if (key_2 == 0x28){
			value_2 = 6;
		}
		
		if (key_2 == 0x21){
			value_2 = 7;
		}
		
		if (key_2 == 0x24){
			value_2 = 8;
		}
		
		if (key_2 == 0x42){
			value_2 = 9;
		}
		
		if (key_2 == 0x48){
			value_2 = 0;
		}
		
		if (key_3 == 0x12){
			value_3 = 1;
		}		
		
		if (key_3 == 0x18){
			value_3 = 2;
		}
		
		if (key_3 == 0x11){
			value_3 = 3;
		}
		
		if (key_3 == 0x14){
			value_3 = 4;
		}
		
		if (key_3 == 0x22){
			value_3 = 5;
		}
		
		if (key_3 == 0x28){
			value_3 = 6;
		}
		
		if (key_3 == 0x21){
			value_3 = 7;
		}
		
		if (key_3 == 0x24){
			value_3 = 8;
		}
		
		if (key_3 == 0x42){
			value_3 = 9;
		}
		
		if (key_3 == 0x48){
			value_3 = 0;
		}
		
		
		float frac = value_3 / 10;
		result_threshold = (value_1 * 10) + (value_2) + frac;
		read_button = 1;
    }


		

void Button_Init(void) {
    SYSCTL->RCGCGPIO |= 0x20;               // Enable clock for GPIO Port F
    while ((SYSCTL->PRGPIO & 0x20) == 0);   // Wait until Port F is ready

    // Unlock PF0 (SW2) to configure it
    GPIOF->LOCK = 0x4C4F434B;               // Unlock GPIOCR register
    GPIOF->CR = 0x01;                       // Allow changes to PF0
    GPIOF->LOCK = 0;                        // Relock GPIOCR register

    GPIOF->DIR &= ~0x01;                    // Set PF0 as input
    GPIOF->DEN |= 0x01;                     // Enable digital function for PF0
    GPIOF->PUR |= 0x01;                     // Enable pull-up resistor for PF0

    // Configure interrupt for PF0
    GPIOF->IS &= ~0x01;                     // Make PF0 edge-sensitive
    GPIOF->IBE &= ~0x01;                    // Disable both edges trigger
    GPIOF->IEV &= ~0x01;                    // Trigger on falling edge
    GPIOF->ICR |= 0x01;                     // Clear any prior interrupt
    GPIOF->IM |= 0x01;                      // Unmask interrupt for PF0

    NVIC_EnableIRQ(GPIOF_IRQn);             // Enable Port F interrupt in NVIC
    NVIC_SetPriority(GPIOF_IRQn, 2);        // Set priority for GPIO Port F interrupt
}

// GPIOF Interrupt Handler
void GPIOF_Handler(void) {
    if (GPIOF->MIS & 0x01) {                // Check if the interrupt is for PF0
        count_case = 0;                     // Reset count_case
				Set_PC2_Low();
				Turn_Off();

				//COMP->ACINTEN |= (1 << 0);
				
				GPIOF->ICR |= 0x01;                 // Clear the interrupt flag
    }
}

void LED_Init(void) {
    
    SYSCTL->RCGCGPIO |= 0x20;
    while ((SYSCTL->PRGPIO & 0x20) == 0); 

    GPIOF->DIR |= (RED_LED | BLUE_LED | GREEN_LED);
    GPIOF->AFSEL &= ~(RED_LED | BLUE_LED | GREEN_LED);   //RED_LED   ==> PF1
    GPIOF->DEN |= (RED_LED | BLUE_LED | GREEN_LED);      //BLUE_LED  ==> PF2
    GPIOF->DATA &= ~(RED_LED | BLUE_LED | GREEN_LED);    //GREEN_LED ==> PF3
}

void Set_LED_Color(uint16_t distance_cm) {
	
    GPIOF->DATA &= ~(RED_LED | BLUE_LED | GREEN_LED);
		
    if (distance_cm >= 750 && distance_cm <= 1000) {
        GPIOF->DATA |= GREEN_LED;
    } else if (distance_cm >= 500 && distance_cm < 750) {
        GPIOF->DATA |= BLUE_LED;
    } else if (distance_cm < 500 & distance != 0) {
        GPIOF->DATA |= RED_LED;
    }
}

void Turn_Off(void){
	GPIOF->DATA &= ~RED_LED;
	GPIOF->DATA &= ~BLUE_LED;
	GPIOF->DATA &= ~GREEN_LED;
}
		


void PortC_Init(void) {
    SYSCTL->RCGCGPIO |= 0x04;       // Enable clock for Port D (bit 3 for Port D)
    while ((SYSCTL->PRGPIO & 0x04) == 0) {} // Wait until Port D is ready

    GPIOC->DIR |= 0x20;            // Set PD2 as output (1 = output)
    GPIOC->AFSEL &= ~0x20;         // Disable alternate functions for PD2
    GPIOC->DEN |= 0x20;            // Enable digital functionality for PD2
    GPIOC->AMSEL &= ~0x20;         // Disable analog functionality for PD2
    GPIOC->PCTL &= ~0x00F00000;    // Configure PD2 as GPIO
}

void Set_PC2_High(void) {
    GPIOC->DATA |= 0x20;           // Set PD2 high
}

void Set_PC2_Low(void) {
    GPIOC->DATA &= ~0x20;          // Set PD2 low
}
 
       
		
		
		
		
// --------------------------- SysTick_Handler For LCD --------------------------
void SysTick_Init(void) {
		if (count_case !=2){
			SysTick->CTRL = 0;             // Disable SysTick during setup
			SysTick->LOAD = 7999999;      // Set reload value     7999999
			SysTick->VAL = 0;              // Clear current value
			SysTick->CTRL = 0x07;          // Enable SysTick with interrupts
			NVIC_EnableIRQ(SysTick_IRQn);  // Enable Port B interrupt in NVIC
			NVIC_SetPriority(SysTick_IRQn, 5);  // Set priority for GPIO Port B interrupt
		}
		else{
			SysTick->CTRL = 0;             // Disable SysTick during setup
			SysTick->LOAD = 999;      // Set reload value
			SysTick->VAL = 0;              // Clear current value
			SysTick->CTRL = 0x07;          // Enable SysTick with interrupts}
			NVIC_EnableIRQ(SysTick_IRQn);  // Enable Port B interrupt in NVIC
			NVIC_SetPriority(SysTick_IRQn, 5);  // Set priority for GPIO Port B interrupt
		}
}
// SysTick Handler
void SysTick_Handler(void) {

			float voltage = take_ADC_value();
			float voltage_th = calculate_threshold_PE4();
			sensor_count++;
			if (((sensor_count %28) == 0 ) & (sensor_count != 0) ){
			
			alg_motor_sensor();
			
		}
	
			switch(count_case)
			  {
			  case 0:
					Nokia5110_SetCursor(5, 2);          // five leading spaces, 3th row
					Nokia5110_OutUDec(voltage*100);
					
					Nokia5110_SetCursor(5, 3);          // five leading spaces, 4th row
					Nokia5110_OutUDec(voltage_th*100);
				
				
					if (read_button == 1){
							Nokia5110_SetCursor(5, 5); 
							Nokia5110_OutUDec(result_threshold);
					}				
					if (stepCount != 0 & stepCount >= 1024){
							stepCount = 2048 - stepCount;
					}
					if (stepCount != 0 & stepCount < 1024){
							step = (step - 2 + 4) % 4 + 1; // Decrement step, wrap around to 4
							setStep(step);
							angle = 180 - ((stepCount - 1024) * 180 / 1024);
							stepCount--;
							if (stepCount == 0){
								SysTick_Init();
								delay_ms(3000);
								button_case = 1;
							}
					}
					break;
			  case 1:

				  break;
			  case 2:
					Nokia5110_SetCursor(5, 4);          // five leading spaces, 4th row
					Nokia5110_OutUDec(avg_sum);
					Motor_Handler();
					
				  break;
				
				case 3	:
					Nokia5110_Clear();
					if ((distance <= 1000) & (distance != 0)){
						
							
							Nokia5110_OutString("**Detected**");
						
							Nokia5110_SetCursor(1, 2); 
							Nokia5110_OutString("Distance: ");
							Nokia5110_SetCursor(5, 3); 
							Nokia5110_OutUDec(distance);
					
							Nokia5110_SetCursor(1, 4); 
							Nokia5110_OutString("Angle: ");
							Nokia5110_SetCursor(5, 5);          // five leading spaces, 4th row
							Nokia5110_OutUDec(last_angle);
							count_case = 5;
						}
					
					else{
							
							Nokia5110_OutString("** Object is not detected **");
							count_case = 4;
					}
					delay_ms(5000);
					

					break;
					
					case 4:
						Nokia5110_Clear();
				  break;
					
					case 5:
							Nokia5110_DrawFullImage(cartesian);
							delay_ms(1000);
							GetPixelCoordinates(last_angle, distance, &x, &y);
							Nokia5110_SetPxl(y,x);
							delay_ms(5000);
							count_case = 4;
			  }	
				delay_ms(5);                     
}
		
		
		
		

// ------------------- ADC FUNCTIONS ----------------------------

// Function to initialize the ADC (FOR LM35)
void ADC_Init(void) {
    SYSCTL->RCGCGPIO |= 0x10;         // Enable clock for GPIO Port E
    __ASM("NOP");                     // Allow time for clock to stabilize
    __ASM("NOP");
    __ASM("NOP");

    SYSCTL->RCGCADC |= 0x01;          // Enable clock for ADC0
    __ASM("NOP");
    __ASM("NOP");
    __ASM("NOP");

    GPIOE->AFSEL |= 0x20;             // Enable alternate function on PE5
    GPIOE->DEN &= ~0x20;              // Disable digital function on PE5
    GPIOE->AMSEL |= 0x20;             // Enable analog mode on PE5

    ADC0->ACTSS &= ~0x08;             // Disable Sample Sequencer 3
    ADC0->EMUX &= ~0xF000;            // Configure for software trigger
    ADC0->SSMUX3 = 8;                 // Set channel AIN8 (PE5)
    ADC0->SSCTL3 = 0x06;              // Set END0 and IE0 bits
    ADC0->ACTSS |= 0x08;              // Enable Sample Sequencer 3
}


// Read ADC value
uint16_t ADC_Read(void) {
    ADC0->PSSI = 0x08;                // Start conversion on Sequencer 3
    while ((ADC0->RIS & 0x08) == 0);  // Wait for conversion to complete
    result = ADC0->SSFIFO3 & 0xFFF;  // Read 12-bit result
    ADC0->ISC = 0x08;                 // Clear completion flag
    return result;
}


float take_ADC_value(void){
		adcValue = ADC_Read();        // Read ADC value
		voltage = (adcValue / 4095.0f) * 3.3f;
		return voltage;
}
// ADC FUNCTIONS FOR PE4 TRIMPOT
void ADC_Init_PE4(void) {
    SYSCTL->RCGCGPIO |= 0x10;         // Enable clock for GPIO Port E
    __ASM("NOP");                     // Allow time for clock to stabilize
    __ASM("NOP");
    __ASM("NOP");

    SYSCTL->RCGCADC |= 0x02;          // Enable clock for ADC1
    __ASM("NOP");
    __ASM("NOP");
    __ASM("NOP");

    GPIOE->AFSEL |= 0x10;             // Enable alternate function on PE4
    GPIOE->DEN &= ~0x10;              // Disable digital function on PE4
    GPIOE->AMSEL |= 0x10;             // Enable analog mode on PE4

    ADC1->ACTSS &= ~0x08;             // Disable Sample Sequencer 3
    ADC1->EMUX &= ~0xF000;            // Configure for software trigger
    ADC1->SSMUX3 = 9;                 // Set channel AIN9 (PE4)
    ADC1->SSCTL3 = 0x06;              // Set END0 and IE0 bits
    ADC1->ACTSS |= 0x08;              // Enable Sample Sequencer 3
}


// Read ADC value for PE4
uint16_t ADC_Read_PE4(void) {
    ADC1->PSSI = 0x08;                // Start conversion on Sequencer 3
    while ((ADC1->RIS & 0x08) == 0);  // Wait for conversion to complete
    uint16_t result_th = ADC1->SSFIFO3 & 0xFFF;  // Read 12-bit result
    ADC1->ISC = 0x08;                 // Clear completion flag
    return result_th;
}


// Calculate voltage (threshold) from ADC value on PE4
float calculate_threshold_PE4(void) {
    adcValue_th = ADC_Read_PE4();  // Read ADC value
    voltage_th = (adcValue_th / 4095.0f) * 3.3f;  // Convert to voltage
		//voltage_th = (0.28*voltage_th)/0.32;
    return voltage_th;
}


// HELPER FUNCTIONS

// A delay in ms
void delay_ms(float ms) {
    for (int i = 0; i < ms; i++) {
        for (int j = 0; j < 3180; j++) {}  // Roughly 1 ms delay
    }
}

// Function to format distance with a dot as the decimal separator 
void format_distance(float value, char *buffer) {
	int integer_part = (int)value; 
	int decimal_part = (int)((value - integer_part) * 100); // 2 decimal places 
	snprintf(buffer, 20, "%d.%02d", integer_part, decimal_part);
}






// ------------------ Motor & Sensor

// Function to initialize SysTick
/*
void SysTick_Init(void) {
    SysTick->CTRL = 0;             // Disable SysTick during setup
    SysTick->LOAD = 39999;      // Set reload value
    SysTick->VAL = 0;              // Clear current value
    SysTick->CTRL = 0x07;          // Enable SysTick with interrupts
}
*/
// Function to initialize Port B
void PortE_Init(void) {
    SYSCTL->RCGCGPIO |= 0x10; // Enable clock for Port E

    GPIOE->LOCK = 0x4C4F434B; // Unlock Port E
    //GPIOB->CR |= 0xF0;        // Allow changes to PE3-PE0

    GPIOE->DIR |= 0x0F;   // Set PE0-PE3 as outputs
    GPIOE->AFSEL &= ~0x0F; // Disable alternate functions
    GPIOE->DEN |= 0x0F;    // Enable digital I/O
    GPIOE->AMSEL &= ~0xFF; // Disable analog functionality
    GPIOE->PCTL &= ~0xFFFFFFFF; // Configure as GPIO
}

// Function to set motor step
void setStep(uint8_t step_no) {
		//step_no = (step_no - 1) % 4 + 1;
	
    GPIOE->DATA &= ~0x0F; // Clear all output pins (PE0-PE3)

    switch (step_no) {
        case 1: GPIOE->DATA |= 0x01; break; // {1, 0, 0, 0}
        case 2: GPIOE->DATA |= 0x02; break; // {0, 1, 0, 0}
        case 3: GPIOE->DATA |= 0x04; break; // {0, 0, 1, 0}
        case 4: GPIOE->DATA |= 0x08; break; // {0, 0, 0, 1}
        default: break;
    }
}

// GPIO initialization for TRIG pin PB0
void TRIG_Init(void) {
    SYSCTL->RCGCGPIO |= 0x02; // Enable clock for GPIOB
    __ASM("NOP");
    __ASM("NOP");
    __ASM("NOP");

    GPIOB->DIR |= 0x01;       // Set PB0 as output
    GPIOB->DEN |= 0x01;       // Enable digital function on PB0
}

// TIMER2 initialization for input capture on PF4
void TIMER2_Init(void) {
    SYSCTL->RCGCGPIO |= 0x20; // Enable clock for GPIOF
		__ASM("NOP");
    __ASM("NOP");
    __ASM("NOP");
    while ((SYSCTL->PRGPIO & 0x20) == 0); // Wait for GPIOF to be ready

    GPIOF->DIR &= ~0x10;      // Set PF4 as input
    GPIOF->AFSEL |= 0x10;     // Enable alternate function on PF4
    GPIOF->PCTL = (GPIOF->PCTL & 0xFFF0FFFF) | 0x00070000; // Set PF4 as T2CCP0
    GPIOF->DEN |= 0x10;       // Enable digital function on PF4

    SYSCTL->RCGCTIMER |= 0x04; // Enable clock for Timer2
    __ASM("NOP");
    __ASM("NOP");
    __ASM("NOP");
    TIMER2->CTL &= ~0x01;      // Disable Timer2A during setup
    TIMER2->CFG = 0x04;        // Configure for 16-bit mode
    TIMER2->TAMR = 0x17;       // Capture mode, edge-time, count up
    TIMER2->CTL |= 0x0C;       // Capture on both edges (rising and falling)
    TIMER2->IMR |= 0x04;       // Enable capture event interrupt for Timer2A

    NVIC_EnableIRQ(TIMER2A_IRQn); // Enable Timer2A interrupt in NVIC
    NVIC_SetPriority(TIMER2A_IRQn, 4); // Set interrupt priority to 1

    TIMER2->CTL |= 0x01;       // Enable Timer2A
}


// Function to send a >10us pulse to TRIG pin
void send_trigger_pulse(void) {
    GPIOB->DATA &= ~0x01;  // Set TRIG pin (PB0) low
    delay_ms(0.5);            // Wait for 2 ms
    GPIOB->DATA |= 0x01;   // Set TRIG pin (PB0) high
    delay_ms(0.01);            // Wait for > 10 us (1 ms)
    GPIOB->DATA &= ~0x01;  // Set TRIG pin (PB0) low
}

void TIMER2A_Handler(void) {
    if (TIMER2->MIS & 0x04) { // Capture event occurred
        uint32_t currentEdge = TIMER2->TAR;
        TIMER2->ICR = 0x04; // Clear interrupt flag

        if (risingEdge == 0) {
            risingEdge = currentEdge; // Record rising edge timestamp
        } else {
            fallingEdge = currentEdge; // Record falling edge timestamp
            if (fallingEdge > risingEdge) {
                pulse_Width_motor = fallingEdge - risingEdge;
            } else {
                pulse_Width_motor = (0xFFFF - risingEdge) + fallingEdge; // Handle overflow
            }
            risingEdge = 0; // Reset for next measurement
        }
    }
}

// SysTick Handler
void Motor_Handler(void) {
    
    //int stepCount = 0;

    if (stepCount < 1024) {
			setStep(step);
			step = (step % 4) + 1; // Increment step, wrap around to 1	
			angle = (stepCount * 180) / 1024;
			stepCount++;
    }	

    else if ((stepCount>= 1024) & (stepCount<2049)){
				//if (stepCount<2049){
				step = (step - 2 + 4) % 4 + 1; // Decrement step, wrap around to 4
				setStep(step);
				angle = 180 - ((stepCount - 1024) * 180 / 1024);
				stepCount++;
				//}
		}
		else {
        //TIMER2->CTL &= ~0x01; // Disable Timer2A
				//TIMER2->ICR = 0x04;   // Clear interrupt flag
				count_case = 3;
				distance_count = 0;
				int dist_sum = 0;	
				int dist_sum_count = 0;
				for(int i=0; i<1000; i++){
					if (buffer_distance[i] != 0){
						dist_sum+=buffer_distance[i];
						dist_sum_count++;
					}
				}
				distance = dist_sum/dist_sum_count;
				
				__asm("NOP");
				__asm("NOP");
				__asm("NOP");
				SysTick_Init();
				__asm("NOP");
				__asm("NOP");
    }
}




// Function to calculate and display the distance
float alg_motor_sensor(void) {
    // Trigger ultrasonic sensor
    send_trigger_pulse();

    // Wait a short time for the echo to return
    delay_ms(1);
		
    // If a valid pulse width is measured, calculate the distance
    if (pulse_Width_motor < 94118 & pulse_Width_motor > 0) {	
				pulse_Width_motor = pulse_Width_motor / 16;  // Convert to microseconds
        distance = (pulse_Width_motor * 0.34) / 2;
				buffer_distance[distance_count] = distance;
				distance_count++;
				last_angle = angle;
				
				
    } 	
		return buffer_distance[1000];
}










// ------------------- NOKIA 5510 ----------------------
int GetPixelCoordinates(float angle, float distance, int *x, int *y) {
    // Horizontal mapping (angle to x)
		
		float new_dist= (distance*40)/1000;
		*x = (new_dist*cos(last_angle*0.0174533))+42;
		*y = 47 - (new_dist*sin(last_angle*0.0174533)); 
		
    return 0;  // Success
}



// -----------------------------------------------------
#define SCREENW     84
#define SCREENH     48

#define DC                      (*((volatile uint32_t *)0x40004100))
#define DC_COMMAND              0
#define DC_DATA                 0x40
#define RESET                   (*((volatile uint32_t *)0x40004200))
#define RESET_LOW               0
#define RESET_HIGH              0x80
#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40004400))
#define GPIO_PORTA_AFSEL_R      (*((volatile uint32_t *)0x40004420))
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile uint32_t *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile uint32_t *)0x4000452C))
#define SSI0_CR0_R              (*((volatile uint32_t *)0x40008000))
#define SSI0_CR1_R              (*((volatile uint32_t *)0x40008004))
#define SSI0_DR_R               (*((volatile uint32_t *)0x40008008))
#define SSI0_SR_R               (*((volatile uint32_t *)0x4000800C))
#define SSI0_CPSR_R             (*((volatile uint32_t *)0x40008010))
#define SSI0_CC_R               (*((volatile uint32_t *)0x40008FC8))
#define SSI_CR0_SCR_M           0x0000FF00  // SSI Serial Clock Rate
#define SSI_CR0_SPH             0x00000080  // SSI Serial Clock Phase
#define SSI_CR0_SPO             0x00000040  // SSI Serial Clock Polarity
#define SSI_CR0_FRF_M           0x00000030  // SSI Frame Format Select
#define SSI_CR0_FRF_MOTO        0x00000000  // Freescale SPI Frame Format
#define SSI_CR0_DSS_M           0x0000000F  // SSI Data Size Select
#define SSI_CR0_DSS_8           0x00000007  // 8-bit data
#define SSI_CR1_MS              0x00000004  // SSI Master/Slave Select
#define SSI_CR1_SSE             0x00000002  // SSI Synchronous Serial Port
                                            // Enable
#define SSI_SR_BSY              0x00000010  // SSI Busy Bit
#define SSI_SR_TNF              0x00000002  // SSI Transmit FIFO Not Full
#define SSI_CPSR_CPSDVSR_M      0x000000FF  // SSI Clock Prescale Divisor
#define SSI_CC_CS_M             0x0000000F  // SSI Baud Clock Source
#define SSI_CC_CS_SYSPLL        0x00000000  // Either the system clock (if the
                                            // PLL bypass is in effect) or the
                                            // PLL output (default)
#define SYSCTL_RCGC1_R          (*((volatile uint32_t *)0x400FE104))
#define SYSCTL_RCGC2_R          (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // port A Clock Gating Control


enum typeOfWrite{
  COMMAND,                              // the transmission is an LCD command
  DATA                                  // the transmission is data
};


// This table contains the hex values that represent pixels
// for a font that is 5 pixels wide and 8 pixels high
static const uint8_t ASCII[][5] = {
  {0x00, 0x00, 0x00, 0x00, 0x00} // 20
  ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
  ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
  ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
  ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
  ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
  ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
  ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
  ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
  ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
  ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
  ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
  ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
  ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
  ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
  ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
  ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
  ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
  ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
  ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
  ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
  ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
  ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
  ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
  ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
  ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
  ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
  ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
  ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
  ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
  ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
  ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
  ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
  ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
  ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
  ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
  ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
  ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
  ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
  ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
  ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
  ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
  ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
  ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
  ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
  ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
  ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
  ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
  ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
  ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
  ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
  ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
  ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
  ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
  ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
  ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
  ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
  ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
  ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
  ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
  ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c '\'
  ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
  ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
  ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
  ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
  ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
  ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
  ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
  ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
  ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
  ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
  ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
  ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
  ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
  ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
  ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
  ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
  ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
  ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
  ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
  ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
  ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
  ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
  ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
  ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
  ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
  ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
  ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
  ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
  ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
  ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
  ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
  ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
  ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
  ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ~
 // ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f DEL
  ,{0x1f, 0x24, 0x7c, 0x24, 0x1f} // 7f UT sign
};

void static lcdwrite(enum typeOfWrite type, uint8_t message){
  if(type == COMMAND){
                                        // wait until SSI0 not busy/transmit FIFO empty
    while((SSI0_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
    DC = DC_COMMAND;
    SSI0_DR_R = message;                // command out
                                        // wait until SSI0 not busy/transmit FIFO empty
    while((SSI0_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
  } else{
    while((SSI0_SR_R&SSI_SR_TNF)==0){}; // wait until transmit FIFO not full
    DC = DC_DATA;
    SSI0_DR_R = message;                // data out
  }
}
void static lcddatawrite(uint8_t data){
  while((SSI0_SR_R&0x00000002)==0){}; // wait until transmit FIFO not full
  DC = DC_DATA;
  SSI0_DR_R = data;                // data out
}

//********Nokia5110_Init*****************
// Initialize Nokia 5110 48x84 LCD 

void Nokia5110_Init(void){
  volatile uint32_t delay;
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_SSI0;  // activate SSI0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  delay = SYSCTL_RCGC2_R;               // allow time to finish activating
  GPIO_PORTA_DIR_R |= 0xC0;             // make PA6,7 out
  GPIO_PORTA_AFSEL_R |= 0x2C;           // enable alt funct on PA2,3,5
  GPIO_PORTA_AFSEL_R &= ~0xC0;          // disable alt funct on PA6,7
  GPIO_PORTA_DEN_R |= 0xEC;             // enable digital I/O on PA2,3,5,6,7
                                        // configure PA2,3,5 as SSI
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFF0F00FF)+0x00202200;
                                        // configure PA6,7 as GPIO
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0x00FFFFFF)+0x00000000;
  GPIO_PORTA_AMSEL_R &= ~0xEC;          // disable analog functionality on PA2,3,5,6,7
  SSI0_CR1_R &= ~SSI_CR1_SSE;           // disable SSI
  SSI0_CR1_R &= ~SSI_CR1_MS;            // master mode
                                        // configure for system clock/PLL baud clock source
  SSI0_CC_R = (SSI0_CC_R&~SSI_CC_CS_M)+SSI_CC_CS_SYSPLL;
                                        // clock divider for 3.33 MHz SSIClk (16 MHz PLL/16)
                                        // SysClk/(CPSDVSR*(1+SCR))
                                        // 50/(16*(1+0)) = 3.125 MHz (slower than 4 MHz)
  SSI0_CPSR_R = (SSI0_CPSR_R&~SSI_CPSR_CPSDVSR_M)+6; // must be even number
  SSI0_CR0_R &= ~(SSI_CR0_SCR_M |       // SCR = 0 (3.125 Mbps data rate)
                  SSI_CR0_SPH |         // SPH = 0
                  SSI_CR0_SPO);         // SPO = 0
                                        // FRF = Freescale format
  SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_FRF_M)+SSI_CR0_FRF_MOTO;
                                        // DSS = 8-bit data
  SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_DSS_M)+SSI_CR0_DSS_8;
  SSI0_CR1_R |= SSI_CR1_SSE;            // enable SSI

  RESET = RESET_LOW;                    // reset the LCD to a known state
  for(delay=0; delay<10; delay=delay+1);// delay minimum 100 ns
  RESET = RESET_HIGH;                   // negative logic

  lcdwrite(COMMAND, 0x21);              // chip active; horizontal addressing mode (V = 0); use extended instruction set (H = 1)
                                        // set LCD Vop (contrast), which may require some tweaking:
  lcdwrite(COMMAND, CONTRAST);          // try 0xB1 (for 3.3V red SparkFun), 0xB8 (for 3.3V blue SparkFun), 0xBF if your display is too dark, or 0x80 to 0xFF if experimenting
  lcdwrite(COMMAND, 0x04);              // set temp coefficient
  lcdwrite(COMMAND, 0x14);              // LCD bias mode 1:48: try 0x13 or 0x14

  lcdwrite(COMMAND, 0x20);              // we must send 0x20 before modifying the display control mode
  lcdwrite(COMMAND, 0x0C);              // set display control to normal mode: 0x0D for inverse
}

//********Nokia5110_OutChar*****************
// Print a character to the Nokia 5110 48x84 LCD.  
void Nokia5110_OutChar(char data){int i;
  lcddatawrite(0x00);        // blank vertical line padding
  for(i=0; i<5; i=i+1){
    lcddatawrite(ASCII[data - 0x20][i]);
  }
  lcddatawrite(0x00);        // blank vertical line padding
}

//********Nokia5110_OutString*****************
// Print a string of characters to the Nokia 5110 48x84 LCD.

void Nokia5110_OutString(char *ptr){
  while(*ptr){
    Nokia5110_OutChar((unsigned char)*ptr);
    ptr = ptr + 1;
  }
}

//********Nokia5110_OutUDec*****************
// Output a 16-bit number in unsigned decimal format with a
// fixed size of five right-justified digits of output.
void Nokia5110_OutUDec(float n){
    // Get the integer part of the number
    uint16_t integer_part = (uint16_t)n;
    // Get the fractional part (4 decimal places)
    uint32_t fractional_part = (uint32_t)((n - integer_part) * 10000);

    // Display the integer part
    if (integer_part < 10) {
        Nokia5110_OutChar(integer_part + '0');
    } else if (integer_part < 100) {
        Nokia5110_OutChar(integer_part / 10 + '0');
        Nokia5110_OutChar(integer_part % 10 + '0');
    } else if (integer_part < 1000) {
        Nokia5110_OutChar(integer_part / 100 + '0');
        integer_part = integer_part % 100;
        Nokia5110_OutChar(integer_part / 10 + '0');
        Nokia5110_OutChar(integer_part % 10 + '0');
    } else if (integer_part < 10000) {
        Nokia5110_OutChar(integer_part / 1000 + '0');
        integer_part = integer_part % 1000;
        Nokia5110_OutChar(integer_part / 100 + '0');
        integer_part = integer_part % 100;
        Nokia5110_OutChar(integer_part / 10 + '0');
        Nokia5110_OutChar(integer_part % 10 + '0');
    } else {
        Nokia5110_OutChar(integer_part / 10000 + '0');
        integer_part = integer_part % 10000;
        Nokia5110_OutChar(integer_part / 1000 + '0');
        integer_part = integer_part % 1000;
        Nokia5110_OutChar(integer_part / 100 + '0');
        integer_part = integer_part % 100;
        Nokia5110_OutChar(integer_part / 10 + '0');
        Nokia5110_OutChar(integer_part % 10 + '0');
    }

    // Display the decimal point
    Nokia5110_OutChar('.');

    // Display the fractional part (4 decimal places)
    Nokia5110_OutChar(fractional_part / 1000 + '0');  // First decimal place
    fractional_part = fractional_part % 1000;
    Nokia5110_OutChar(fractional_part / 100 + '0');   // Second decimal place
    fractional_part = fractional_part % 100;
    Nokia5110_OutChar(fractional_part / 10 + '0');    // Third decimal place
    fractional_part = fractional_part % 10;
    Nokia5110_OutChar(fractional_part + '0');         // Fourth decimal place
}

//********Nokia5110_SetCursor*****************
// Move the cursor to the desired X- and Y-position.  The
// next character will be printed here.  X=0 is the leftmost
// column.  Y=0 is the top row.
// inputs: newX  new X-position of the cursor (0<=newX<=11)
//         newY  new Y-position of the cursor (0<=newY<=5)
// outputs: none
void Nokia5110_SetCursor(uint8_t newX, uint8_t newY){
  if((newX > 11) || (newY > 5)){        // bad input
    return;                             // do nothing
  }
  // multiply newX by 7 because each character is 7 columns wide
  lcdwrite(COMMAND, 0x80|(newX*7));     // setting bit 7 updates X-position
  lcdwrite(COMMAND, 0x40|newY);         // setting bit 6 updates Y-position
}

//********Nokia5110_Clear*****************
// Clear the LCD by writing zeros to the entire screen and
// reset the cursor to (0,0) (top left corner of screen).
void Nokia5110_Clear(void){
  int i;
  for(i=0; i<(MAX_X*MAX_Y/8); i=i+1){
    lcddatawrite(0x00);
  }
  Nokia5110_SetCursor(0, 0);
}

//********Nokia5110_DrawFullImage*****************
// Fill the whole screen by drawing a 48x84 bitmap image.
// inputs: ptr  pointer to 504 byte bitmap
void Nokia5110_DrawFullImage(const uint8_t *ptr){
  int i;
  Nokia5110_SetCursor(0, 0);
  for(i=0; i<(MAX_X*MAX_Y/8); i=i+1){
    lcddatawrite(ptr[i]);
  }
}
uint8_t Screen[SCREENW*SCREENH/8]; // buffer stores the next image to be printed on the screen

//********Nokia5110_PrintBMP*****************
// Bitmaps defined above were created for the LM3S1968 or
// LM3S8962's 4-bit grayscale OLED display.  They also
// still contain their header data and may contain padding
// to preserve 4-byte alignment.  This function takes a
// bitmap in the previously described format and puts its
// image data in the proper location in the buffer so the
// image will appear on the screen after the next call to
//   Nokia5110_DisplayBuffer();
// The interface and operation of this process is modeled
// after RIT128x96x4_BMP(x, y, image);
// inputs: xpos      horizontal position of bottom left corner of image, columns from the left edge
//                     must be less than 84
//                     0 is on the left; 82 is near the right
//         ypos      vertical position of bottom left corner of image, rows from the top edge
//                     must be less than 48
//                     2 is near the top; 47 is at the bottom
//         ptr       pointer to a 16 color BMP image
//         threshold grayscale colors above this number make corresponding pixel 'on'
//                     0 to 14
//                     0 is fine for ships, explosions, projectiles, and bunkers
// outputs: none
void Nokia5110_PrintBMP(uint8_t xpos, uint8_t ypos, const uint8_t *ptr, uint8_t threshold){
  int32_t width = ptr[18], height = ptr[22], i, j;
  uint16_t screenx, screeny;
  uint8_t mask;
  // check for clipping
  if((height <= 0) ||              // bitmap is unexpectedly encoded in top-to-bottom pixel order
     ((width%2) != 0) ||           // must be even number of columns
     ((xpos + width) > SCREENW) || // right side cut off
     (ypos < (height - 1)) ||      // top cut off
     (ypos > SCREENH))           { // bottom cut off
    return;
  }
  if(threshold > 14){
    threshold = 14;             // only full 'on' turns pixel on
  }
  // bitmaps are encoded backwards, so start at the bottom left corner of the image
  screeny = ypos/8;
  screenx = xpos + SCREENW*screeny;
  mask = ypos%8;                // row 0 to 7
  mask = 0x01<<mask;            // now stores a mask 0x01 to 0x80
  j = ptr[10];                  // byte 10 contains the offset where image data can be found
  for(i=1; i<=(width*height/2); i=i+1){
    // the left pixel is in the upper 4 bits
    if(((ptr[j]>>4)&0xF) > threshold){
      Screen[screenx] |= mask;
    } else{
      Screen[screenx] &= ~mask;
    }
    screenx = screenx + 1;
    // the right pixel is in the lower 4 bits
    if((ptr[j]&0xF) > threshold){
      Screen[screenx] |= mask;
    } else{
      Screen[screenx] &= ~mask;
    }
    screenx = screenx + 1;
    j = j + 1;
    if((i%(width/2)) == 0){     // at the end of a row
      if(mask > 0x01){
        mask = mask>>1;
      } else{
        mask = 0x80;
        screeny = screeny - 1;
      }
      screenx = xpos + SCREENW*screeny;
      // bitmaps are 32-bit word aligned
      switch((width/2)%4){      // skip any padding
        case 0: j = j + 0; break;
        case 1: j = j + 3; break;
        case 2: j = j + 2; break;
        case 3: j = j + 1; break;
      }
    }
  }
}
// There is a buffer in RAM that holds one screen
// This routine clears this buffer
void Nokia5110_ClearBuffer(void){int i;
  for(i=0; i<SCREENW*SCREENH/8; i=i+1){
    Screen[i] = 0;              // clear buffer
  }
}

//********Nokia5110_DisplayBuffer*****************
// Fill the whole screen by drawing a 48x84 screen image.
// inputs: none
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_DisplayBuffer(void){
  Nokia5110_DrawFullImage(Screen);
}

const unsigned char Masks[8]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
//------------Nokia5110_ClrPxl------------
// Clear the Image pixel at (i, j), turning it dark.
// Input: i  the row index  (0 to 47 in this case),    y-coordinate
//        j  the column index  (0 to 83 in this case), x-coordinate
// Output: none		
void Nokia5110_ClrPxl(uint32_t i, uint32_t j){
  Screen[84*(i>>3) + j] &= ~Masks[i&0x07];
}
//------------Nokia5110_SetPxl------------
// Set the Image pixel at (i, j), turning it on.
// Input: i  the row index  (0 to 47 in this case),    y-coordinate
//        j  the column index  (0 to 83 in this case), x-coordinate
// Output: none		
void Nokia5110_SetPxl(uint32_t i, uint32_t j){
 // Calculate the byte in the screen buffer and the bit position
    uint16_t byteIndex = ((y / 8)-1) * SCREEN_WIDTH + x; // y / 8 determines the row in the buffer
    uint8_t bitMask = 1 << (7-(y % 8));                  // y % 8 determines the bit within the byte

    // Set the pixel in the screen buffer
    cartesian[byteIndex] |= bitMask;
		Nokia5110_DrawFullImage(cartesian);
}