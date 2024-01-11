/* ========================================
 * Author: Diego Brandjes
 * Student Number: 500831945
 * IR-Sensor Braking System
 * ========================================
*/

#include "project.h"
#include <stdio.h>
#include <u8x8.h>
#define MINIMUM 7
#define BRAKE_DISTANCE 15
#define BLINK_FREQUENCY 10

uint16_t distanceCM;
u8x8_t EM2Display;
uint8_t psoc_gpio_and_delay_callback(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr); 
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

void uart(int input){    
    char output[20];
    sprintf(output, "%d\n\r", input);
    UART_1_PutString(output);
}

void I2C(char *msg, int i){   
    u8x8_DrawString(&EM2Display, 1, i, msg);
}

void FlashLed(int duration) {
 
    int toggle = 0;
    for (int i = 0; i < BLINK_FREQUENCY; i++){
        
        toggle = !toggle;
        Led_Write(toggle);
        CyDelay(duration/BLINK_FREQUENCY);       
    }
}

int readIRSensor(void) {
    
    char conv[20];
    char string[14];

    ADC_SAR_1_StartConvert();
    ADC_SAR_1_IsEndConversion(ADC_SAR_1_WAIT_FOR_RESULT);
    distanceCM = (2400/(ADC_SAR_1_GetResult16() - 20))* 2.54;
    
    if(distanceCM == MINIMUM){
        I2C("  Too  Close", 4);
        PWM_1_WriteCompare(0);           
        FlashLed(100);
        
    } else if (distanceCM <= BRAKE_DISTANCE){
        I2C("   Braking     ", 4);
        PWM_1_WriteCompare(120);           
        FlashLed(500);
        
    } else{    
        sprintf(conv, "    %d Cm    ", distanceCM);    
            for (uint32_t i = 0; i < sizeof (string); i++){
                string[i] = conv[i];         
            }           
        I2C(string, 4);
        CyDelay(50);        
    }    
    
    PWM_1_WriteCompare(200);           
    uart(distanceCM);
    
    return distanceCM;
}

void initComp() {
    
    u8x8_Setup(&EM2Display, u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_i2c, 
        u8x8_byte_hw_i2c, psoc_gpio_and_delay_callback);
    
    ADC_SAR_1_Start();
    UART_1_Start();
    I2C_1_Start();
    PWM_1_Start();
    
    u8x8_InitDisplay(&EM2Display);
    u8x8_SetPowerSave(&EM2Display, 0);
    u8x8_ClearDisplay(&EM2Display);
    u8x8_SetFont(&EM2Display, u8x8_font_pxplusibmcgathin_f);     
}


int main(void){
    
    CyGlobalIntEnable; 
    initComp();
    
    I2C("   Distance", 1);
       
    for(;;){
        
        readIRSensor();
        
    }
}


/*
PSoC I2C Hardware Interface for U8X8
*/
uint8_t psoc_gpio_and_delay_callback(u8x8_t *u8x8, uint8_t msg,
uint8_t arg_int, void *arg_ptr) {

(void) u8x8; 
(void) arg_ptr;

switch(msg) {

    case U8X8_MSG_GPIO_AND_DELAY_INIT:
    break;

    case U8X8_MSG_DELAY_MILLI:
    CyDelay(arg_int); break;

    case U8X8_MSG_DELAY_10MICRO:
    CyDelayUs(10); break;

    case U8X8_MSG_DELAY_100NANO:
    CyDelayCycles(100); break;

    }

    return 1;

}

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg,
uint8_t arg_int, void *arg_ptr) {

uint8_t *data; switch(msg) {
case U8X8_MSG_BYTE_SEND:
data = (uint8_t *)arg_ptr;

while(arg_int > 0) { 
    (void)I2C_1_MasterWriteByte(*data); data++;
    arg_int--;
}

break;

case U8X8_MSG_BYTE_INIT:
break;

case U8X8_MSG_BYTE_SET_DC:
break;

case U8X8_MSG_BYTE_START_TRANSFER:
(void)I2C_1_MasterSendStart(u8x8_GetI2CAddress(u8x8) >> 1,
I2C_1_WRITE_XFER_MODE);
break;

case U8X8_MSG_BYTE_END_TRANSFER:
(void)I2C_1_MasterSendStop(); 
break;

default: return 0;
    }

return 1;

}

/* [] END OF FILE */
