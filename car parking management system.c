#include "msp.h"
#include <stdio.h>
#include <stdbool.h>

#define IR_SENSOR_PIN BIT5
#define IR_SENSOR2_PIN BIT4
#define SERVO_CONTROL_PIN BIT0
#define LED_PIN BIT0
#define SERVO_PIN BIT7
#define PARKING_FULL_PIN BIT0

void configureIRSensor();
void configureServoMotor();
void delayMs(int n);
void servoRotateClockwise();
void servoRotateCounterClockwise();
void UART_Init(void);
void UART_TransmitChar(unsigned char data);
void UART_TransmitString(const char* str);
void itoa(int value, char* str, int base);
void configureSevenSegment();
void displayNumber(int num);

int availableSpaces = 2; // Set initial available spaces
char str[3];
char report[50];

void configureIRSensor() {
    // Configure IR sensor pins as inputs with pull-up resistors
    P1->DIR &= ~IR_SENSOR_PIN;
    P1->REN |= IR_SENSOR_PIN;
    P1->OUT |= IR_SENSOR_PIN;

    P4->DIR &= ~IR_SENSOR2_PIN;
    P4->REN |= IR_SENSOR2_PIN;
    P4->OUT |= IR_SENSOR2_PIN;
}

void configureServoMotor() {
    // Configure servo motor control pin as output
    P3->DIR |= SERVO_CONTROL_PIN;
}

void configureSevenSegment() {
    // Configure GPIO pins for 7-segment LED display
    // Adjust the pin assignments based on your specific hardware setup
    P2->DIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6;
}

void displayNumber(int num) {
    // Define the 7-segment LED display patterns for digits 0-9
    uint8_t patterns[10] = {
        0x3F, // 0
        0x06, // 1
        0x5B, // 2
        0x4F, // 3
        0x66, // 4
        0x6D, // 5
        0x7D, // 6
        0x07, // 7
        0x7F, // 8
        0x6F  // 9
    };

    // Display the specified number on the 7-segment LED
    P2->OUT = patterns[num % 10];
}

void delayMs(int n) {
    int i, j;
    for (j = 0; j < n; j++)
        for (i = 750; i > 0; i--); // Delay of 1 ms
}

void servoRotateClockwise() {
    // Rotate the servo motor clockwise by setting the control pin high for a specific duration
    P3->OUT |= SERVO_CONTROL_PIN;
    delayMs(2); // Adjust the delay to control the rotation angle
    P3->OUT &= ~SERVO_CONTROL_PIN;
}

void servoRotateCounterClockwise() {
    // Rotate the servo motor counter-clockwise by setting the control pin high for a specific duration
    P3->OUT |= SERVO_CONTROL_PIN;
    delayMs(1); // Adjust the delay to control the rotation angle
    P3->OUT &= ~SERVO_CONTROL_PIN;
}

void UART_Init(void) {
    EUSCI_A0->CTLW0 |= 1; // Put in reset mode to configure UART
    EUSCI_A0->MCTLW = 0;  // Disable oversampling
    EUSCI_A0->CTLW0 = 0x0081; // 00 - 1 stop bit, No Parity, 8-bit data, Asynchronous Mode, First LSB Then MSB, SMCLK
                              // 81 -  Enabled EUSCI_A0 logic held in reset state
    EUSCI_A0->BRW = 26; // 3000000/115200 = 26
    P1->SEL0 |= 0x0C;  // Configure functionality of P1.2, P1.3 as UART Pins
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 &= ~1; // Take UART out of reset mode
}

void UART_TransmitChar(unsigned char data) {
    while (!(EUSCI_A0->IFG & 0x02)); // Wait until TX buffer is ready
    EUSCI_A0->TXBUF = data; // Send character
}

void UART_TransmitString(const char* str) {
    while (*str) {
        UART_TransmitChar(*str);
        str++;
    }
}

void itoa(int value, char* str, int base) {
    sprintf(str, "%d", value);
}

int main(void) {
    __disable_irq();

    // Configure IR sensor pins with pull-up resistors
    configureIRSensor();

    // Configure servo motor control pin as output
    configureServoMotor();

    // Configure LED output pin
    P1->DIR |= LED_PIN;

    // Initialize the 7-segment display
    configureSevenSegment();

    UART_Init();

    // Enable interrupts
    __enable_irq();

    while (1) {
        {
    if (((P1->IN & IR_SENSOR_PIN) == 0) && ((P4->IN & IR_SENSOR2_PIN) == 0)) {
        // Both IR sensors detect an object
        P1->OUT ^= LED_PIN; // Toggle LED state
        servoRotateClockwise();
        availableSpaces = 0;
        delayMs(500);
			UART_TransmitString("NO spaces\n");
    } else if (((P1->IN & IR_SENSOR_PIN) == 0) && ((P4->IN & IR_SENSOR2_PIN) != 0)) {
        // Only IR sensor 1 detects an object
        P1->OUT &= ~LED_PIN; // Turn off LED
        servoRotateCounterClockwise();
        availableSpaces = 1;
        delayMs(500);
        UART_TransmitString("Available spaces: 1 (Left)\n");
    } else if (((P1->IN & IR_SENSOR_PIN) != 0) && ((P4->IN & IR_SENSOR2_PIN) == 0)) {
        // Only IR sensor 2 detects an object
        P1->OUT &= ~LED_PIN; // Turn off LED
        servoRotateCounterClockwise();
        availableSpaces = 1;
        delayMs(500);
        UART_TransmitString("Available spaces: 1 (Right)\n");
    } else {
        // Neither IR sensor detects an object
        P1->OUT &= ~LED_PIN; // Turn off LED
        servoRotateCounterClockwise();
        availableSpaces = 2;
        delayMs(500);
			UART_TransmitString("FREE SPACE\n");
    }

        // Update 7-segment display with available spaces
        displayNumber(availableSpaces);

        // Transmit report over UART
        
        UART_TransmitString(report);

        delayMs(2000);
    }
}}