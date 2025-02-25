//do not change the order of the following 3 definitions
#define FCY 12800000UL 
#define BUFFER_SIZE 256
#include <stdio.h>
#include <libpic30.h>

#include <p33Fxxxx.h>
#include "types.h"
#include "uart.h"
#include "crc16.h"
#include "lab3.h"
#include "lcd.h"
#include "timer.h"

// Primary (XT, HS, EC) Oscillator without PLL
_FOSCSEL(FNOSC_PRIPLL);
// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystanl
_FOSC(OSCIOFNC_ON & POSCMD_XT);
// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);
// Disable Code Protection
_FGS(GCP_OFF);   

#define TIMEOUT_MS 1000  // Timeout in milliseconds (1 sec)


volatile uint8_t interrupt_flag;
char buffer[BUFFER_SIZE]; 
uint16_t crc;

void __attribute__((__interrupt__)) _T2Interrupt(void) {
    interrupt_flag = 1; // Set the interrupt flag to indicate time out 
    TMR2 = 0x00; // reset timer
    IFS0bits.T2IF = 0; // clear the flag
}

// flushes the UART recv buffer
void flush() {
    uint8_t temp = 0;
    start_timer2();
    interrupt_flag = 0;
    while(interrupt_flag==0){ //waits for the timeout
        while(uart2_recv(&temp) != 0 && interrupt_flag==0){} // Read all data received before timeout
    }
    stop_timer2();
}

// Resets all elements in the buffer to \0
void reset_buffer()
{
    int i;
    for(i=0; i < BUFFER_SIZE; i++){
        buffer[i] = '\0';
    }
}

// updates LCD with message
void print_lcd_updated(uint8_t fail_count, uint16_t crc) {
    lcd_clear();
    lcd_locate(0, 0);
    lcd_printf("Fail attempts: %d", fail_count);
    lcd_locate(0, 1);
    lcd_printf("CRC value: %x\r", crc);
    lcd_locate(0, 2);
    lcd_printf("message: %s", buffer);
}

// Updates LCD when there is no valid message
void print_lcd_empty(uint8_t fail_count) {
    lcd_clear();
    lcd_locate(0, 0);
    lcd_printf("Fail attempts: %d", fail_count);
    lcd_locate(0, 1);
    lcd_printf("CRC value:\r");
    lcd_locate(0, 2);
    lcd_printf("message:\r");
}


int8_t receive_message(){
   // uint8_t msg_length = 0;  // reset msg_length
    uint16_t crc_recv = 0;
    uint8_t incoming_data;
    uint8_t index; // index from buffer
    
        
    // busy-waiting for the starting byte
    if(uart2_recv_start_byte() == -1) {
        return -1;
    }
    
    // start the timeout timer
    interrupt_flag = 0;
    start_timer2();
    
    // Read the 2-Byte CRC value
    volatile uint8_t crc_high=0;
    while(uart2_recv(&crc_high) != 0 && interrupt_flag == 0){}
    volatile uint8_t crc_low=0;
    while(uart2_recv(&crc_low) != 0 && interrupt_flag == 0){}
    
    // Read the message length byte
    uint8_t msg_length=0;
    while(uart2_recv(&msg_length)!=0 && interrupt_flag == 0){}
    
    crc_recv = (crc_high<<8)|(crc_low);

    // store message in buffer
    index = 0;
    crc = 0;             
    while(index<msg_length && interrupt_flag == 0){
        while(uart2_recv(&incoming_data) != 0 && interrupt_flag == 0){}
        buffer[index] = incoming_data; // store received dat
        crc = crc_update(crc, incoming_data); // Updates the CRC
        index++;
    }
    stop_timer2();
    
    // check client crc and server crc
    if(crc == crc_recv && interrupt_flag == 0) {
        return 0;
    }
    return -1;
}


int main(void)
{	
    // init UART
    uint16_t baud = 9600;
    uart2_init(baud);
    
    set_timer2(); // 1s interrupt

    // init LCD
    lcd_initialize();
    lcd_clear();
    print_lcd_empty(0);
    
    // count failure times
    uint8_t fail_count = 0;  


    while(1) {
        // check ACK
        if(receive_message() == 0){
            
            print_lcd_updated(fail_count, crc);       
            fail_count = 0;
            uart2_send_8(MSG_ACK);
        } 
        else {
         // uart2_flush();
            flush();
            fail_count++;
            print_lcd_updated(fail_count, crc);
            uart2_send_8(MSG_NACK);
        }
        reset_buffer();
        
    }
     
}
