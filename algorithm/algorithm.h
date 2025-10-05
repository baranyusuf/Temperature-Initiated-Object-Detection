#include "TM4C123GH6PM.h"  
#include <stdint.h>

void GPIO_Init_for_Analog_Comparator(void); // PC6 PC7
void DeepSleep_Init(void);
void AnalogComparator_Init(void); //COMP0
void Button_Init(void); //PF0 (SW2)
void LED_Init(void);  //RED_LED   ==> PF1   BLUE_LED  ==> PF2  GREEN_LED ==> PF3


void pulse_init(void);
void TIMER2B_Handler(void);

void Set_LED_Color(uint16_t distance_cm);
void GPIO_Init_for_Analog_Comparator(void);
void DeepSleep_Init(void);
void AnalogComparator_Init(void);
void Button_Init(void);
void COMP0_Handler(void);
void LED_Init(void);
void Set_LED_Color(uint16_t distance_cm);

void ADC_Init(void);
uint16_t ADC_Read(void);
float take_ADC_value(void);
void ADC_Init_PE4(void);
uint16_t ADC_Read_PE4(void);
float calculate_threshold_PE4(void);

extern int count_case;
extern float avg_sum;
extern int start_motor;
void delay_ms(float ms);
extern void OutStr(const char *str);
void format_distance(float distance, char* buffer);

// Global Variables
extern int16_t result; // Define initially.
extern int16_t adcValue;   // Variable to store ADC result
extern float voltage;

extern int LOW;  
extern int HIGH; 
extern volatile int run_time;

extern float result_threshold;

// Define step rates in steps per second
extern uint8_t step ;        // Initial step
extern uint32_t risingEdge, fallingEdge, pulseWidth;
extern float distance;
extern char formatted_distance[20];
extern volatile uint32_t *recorded_value;

// Define bit masks for RGB LEDs
#define RED_LED   0x02 // PF1
#define BLUE_LED  0x04 // PF2
#define GREEN_LED 0x08 // PF3

// ------------ Digital Sensor -----------
void I2C3_Init(void);
void I2C_busy(void);
char I2C3_Write_Multiple(int slave_address, char register_address, int bytes_count, char data);
char I2C3_Read_Multiple(int slave_address, char register_address, int bytes_count, char* array);
int Read_IC(void);

void PortC_Init(void);
void Set_PC2_High(void);
void Set_PC2_Low(void);
void LED_Init(void);
void Set_LED_Color(uint16_t distance_cm);
void Turn_Off(void);

// -------------------- KEYPAD -------------
void GPIO_Init_for_Analog_Comparator(void); // PC6 PC7
void DeepSleep_Init(void);
void AnalogComparator_Init(void); //COMP0
extern void keypad(void);
extern void GPIO_B_INITIALIZE(void);
extern void DELAY200(void);
void GPIOB_Interrupt_Init(void);
void process_keypad_input();
void Button_Init(void);
//void TIMER2B_Handler(void);
//void pulse_init(void);

//--------------------- Motor & Sensor & LCD -------------------
void SysTick_Init(void);
void TIMER2_Init(void);
void TRIG_Init(void);
void send_trigger_pulse(void);
float alg_motor_sensor(void);
void PortE_Init(void);
void setStep(uint8_t step_no);
void Motor_Handler(void);

extern int stepCount;
int GetPixelCoordinates(float angle, float distance, int *x, int *y);

//--------------------- LCD5110 & PLL --------------------

// Blue Nokia 5110
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Reset         (RST, pin 1) connected to PA7
// SSI0Fss       (CE,  pin 2) connected to PA3
// Data/Command  (DC,  pin 3) connected to PA6
// SSI0Tx        (Din, pin 4) connected to PA5
// SSI0Clk       (Clk, pin 5) connected to PA2
// 3.3V          (Vcc, pin 6) power
// back light    (BL,  pin 7) not connected
// Ground        (Gnd, pin 8) ground

// Maximum dimensions of the LCD, although the pixels are
// numbered from zero to (MAX-1).  Address may automatically
// be incremented after each transmission.
#define MAX_X                   84
#define MAX_Y                   48

// Contrast value 0xB1 looks good on red SparkFun
// and 0xB8 looks good on blue Nokia 5110.
// Adjust this from 0xA0 (lighter) to 0xCF (darker) for your display.
#define CONTRAST                0xB1


//********Nokia5110_Init*****************
void Nokia5110_Init(void);

//********Nokia5110_OutChar*****************
// Print a character to the Nokia 5110 48x84 LCD.  
void Nokia5110_OutChar(char data);

//********Nokia5110_OutString*****************
// Print a string of characters to the Nokia 5110 48x84 LCD.
// The string will automatically wrap, so padding spaces may
// be needed to make the output look optimal.
// inputs: ptr  pointer to NULL-terminated ASCII string
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutString(char *ptr);

//********Nokia5110_OutUDec*****************
// Output a 16-bit number in unsigned decimal format with a
// fixed size of five right-justified digits of output.
// Inputs: n  16-bit unsigned number
// Outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutUDec(float n); ////////////CHANGED

//********Nokia5110_SetCursor*****************
// Move the cursor to the desired X- and Y-position.  The
// next character will be printed here.  X=0 is the leftmost
// column.  Y=0 is the top row.
// inputs: newX  new X-position of the cursor (0<=newX<=11)
//         newY  new Y-position of the cursor (0<=newY<=5)
// outputs: none
void Nokia5110_SetCursor(uint8_t newX, uint8_t newY);

//********Nokia5110_Clear*****************
// Clear the LCD by writing zeros to the entire screen and
// reset the cursor to (0,0) (top left corner of screen).
// inputs: none
// outputs: none
void Nokia5110_Clear(void);

//********Nokia5110_DrawFullImage*****************
// Fill the whole screen by drawing a 48x84 bitmap image.
// inputs: ptr  pointer to 504 byte bitmap
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_DrawFullImage(const uint8_t *ptr);

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
void Nokia5110_PrintBMP(uint8_t xpos, uint8_t ypos, const uint8_t *ptr, uint8_t threshold);

// There is a buffer in RAM that holds one screen
// This routine clears this buffer
void Nokia5110_ClearBuffer(void);

//********Nokia5110_DisplayBuffer*****************
// Fill the whole screen by drawing a 48x84 screen image.
// inputs: none
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_DisplayBuffer(void);

//------------Nokia5110_ClrPxl------------
// Clear the Image pixel at (i, j), turning it dark.
// Input: i  the row index  (0 to 47 in this case),    y-coordinate
//        j  the column index  (0 to 83 in this case), x-coordinate
// Output: none		
void Nokia5110_ClrPxl(uint32_t i, uint32_t j);

//------------Nokia5110_SetPxl------------
// Set the Image pixel at (i, j), turning it on.
// Input: i  the row index  (0 to 47 in this case),    y-coordinate
//        j  the column index  (0 to 83 in this case), x-coordinate
// Output: none		
void Nokia5110_SetPxl(uint32_t i, uint32_t j);



