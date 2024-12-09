#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart.h"
#include <string.h>  
#include <avr/interrupt.h> // For remote curtian control
#include "light_ws2812.h" //Light ws2812 refer to https://github.com/cpldcpu/light_ws2812/tree/master
#define NUM_LEDS 30  // LED 

#define F_CPU 16000000UL 
#define BAUD 9600
#define BAUD_PRESCALER ((F_CPU / (16UL * BAUD)) - 1)

#define CS      PB2
#define MOSI    PB3
#define MISO    PB4
#define SCK     PB5

// Base address for TWI0
#define TWI0_BASE 0xB8
#define TWBR0 (*(volatile uint8_t *)(TWI0_BASE))
#define TWSR0 (*(volatile uint8_t *)(TWI0_BASE + 0x01))
#define TWDR0 (*(volatile uint8_t *)(TWI0_BASE + 0x03))
#define TWCR0 (*(volatile uint8_t *)(TWI0_BASE + 0x04))

#define BH1750_ADDRESS 0x23
#define BH1750_POWER_ON 0x01
#define BH1750_CONTINUOUS_HIGH_RES_MODE 0x10

// Base address for TWI1
#define TWI1_BASE 0xD8
#define TWBR1 (*(volatile uint8_t *)(TWI1_BASE))
#define TWSR1 (*(volatile uint8_t *)(TWI1_BASE + 0x01))
#define TWDR1 (*(volatile uint8_t *)(TWI1_BASE + 0x03))
#define TWCR1 (*(volatile uint8_t *)(TWI1_BASE + 0x04))

#define MPU6050_ADDRESS 0x68
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1 0x6B

#define LED_PIN PB1 //earthquake LED connected to PB1
// curtain Motor
#define MOTOR_PIN1 PC2  // Connected to IN1 of H-bridge
#define MOTOR_PIN2 PC3  // Connected to IN2 of H-bridge

//#define MOTOR_PIN PD0 // Earthquke Motor
#define CURTAIN PD1
#define DOOR PD0
#define BUZZER_PIN PD3   // Earthquke Buzzer connected to PD3
#define SERVO_MIN 64    // Duty cycle for 0? (1ms pulse width, ~5%)
#define SERVO_MAX 256   // Duty cycle for 180? (2ms pulse width, ~10%)

//volatile uint8_t curtain_action = 0; // 0: Stop, 1: open curtain, 2: close curtain
//volatile uint8_t first_curtain_state = 0; // init curtain state on app is closed

void GPIO_init() { // for if curtain auto
    DDRD &= ~(1 << DOOR);    // Configure PD2 (INT0 pin) as input
}

//Earthquke Buzzer
void pwm_init_timer2() {
    //non-inverting mode
    TCCR2A |= (1 << WGM20) | (1 << WGM21);  // Fast PWM mode
    TCCR2A |= (1 << COM2B1);                // Non-reverse mode
    //TCCR2B |= (1 << CS21) | (1 << CS20);    // Prescaler 64, firstly do not set this
    OCR2A = 49; // 16MHz/(2*64*2.5kHZ) - 1 = 49 -- prescaler is 64
    OCR2B = 2;  //5% duty cycle
    // Configure BUZZER_PIN as output
    DDRD |= (1 << BUZZER_PIN);
}

void PWM_init() {
    // Set Timer0 to Fast PWM mode, non-inverting mode
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
    TCCR0B = (1 << CS01) | (1 << CS00); // Set prescaler to 64
    DDRD |= (1 << PD6); // Set PD6 (OC0A) as output
}

//// Function to set servo motor angle
void set_servo_angle(uint8_t angle) {
    // Map angle (0-180�) to duty cycle (SERVO_MIN to SERVO_MAX)
    uint8_t duty = SERVO_MIN + (angle * (SERVO_MAX - SERVO_MIN) / 180);
    OCR0A = duty; // Update duty cycle for PWM
}

void buzzer_on() {
    //start timer2 
    TCCR2B |= (1 << CS21) | (1 << CS20);    // Prescaler 64
}

void buzzer_off() {
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
}

// Motor for curtain
void setup_motor() {
    // Set motor control pins as output
    DDRC |= (1 << MOTOR_PIN1) | (1 << MOTOR_PIN2);
    
    // Ensure motor is stopped initially
    PORTC &= ~(1 << MOTOR_PIN1);
    PORTC &= ~(1 << MOTOR_PIN2);
}

void motor_forward() {
    // Set MOTOR_PIN1 HIGH and MOTOR_PIN2 LOW
    PORTC |= (1 << MOTOR_PIN1);
    PORTC &= ~(1 << MOTOR_PIN2);
}

void motor_backward() {
    // Set MOTOR_PIN1 LOW and MOTOR_PIN2 HIGH
    PORTC &= ~(1 << MOTOR_PIN1);
    PORTC |= (1 << MOTOR_PIN2);
}

void motor_stop() {
    // Set both pins LOW to stop the motor
    PORTC &= ~(1 << MOTOR_PIN1);
    PORTC &= ~(1 << MOTOR_PIN2);
}


void led_on() {
    DDRB |= (1 << LED_PIN);
    PORTB |= (1 << LED_PIN); // Set LIGHT_PIN high to turn on the light
}

void led_off() {
    PORTB &= ~(1 << LED_PIN); // Set LIGHT_PIN low to turn off the light
}

// Initialize TWI0
void TWI0_Init(void) {
    TWBR0 = 32; // Set SCL frequency ~100kHz
    TWSR0 = 0x00; // Prescaler factor is 1
    TWCR0 = (1 << TWEN); // Enable TWI0
}

// Generate a START condition for TWI0
void TWI0_Start(void) {
    TWCR0 = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR0 & (1 << TWINT)));
}

// Generate a STOP condition for TWI0
void TWI0_Stop(void) {
    TWCR0 = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
    _delay_us(10);
}

// Write a byte of data to TWI0
void TWI0_Write(uint8_t data) {
    TWDR0 = data;
    TWCR0 = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR0 & (1 << TWINT)));
}

// Read a byte of data from TWI0 with ACK
uint8_t TWI0_Read_ACK(void) {
    TWCR0 = (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR0 & (1 << TWINT)));
    return TWDR0;
}

// Read a byte of data from TWI0 without ACK
uint8_t TWI0_Read_NACK(void) {
    TWCR0 = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR0 & (1 << TWINT)));
    return TWDR0;
}

// Initialize BH1750 sensor
void BH1750_Init(void) {
    TWI0_Start();
    TWI0_Write(BH1750_ADDRESS << 1);
    TWI0_Write(BH1750_POWER_ON);
    TWI0_Stop();

    _delay_ms(10);

    TWI0_Start();
    TWI0_Write(BH1750_ADDRESS << 1);
    TWI0_Write(BH1750_CONTINUOUS_HIGH_RES_MODE);
    TWI0_Stop();
}

// Read light level from BH1750 sensor
uint16_t BH1750_ReadLightLevel(void) {
    uint16_t lux = 0;

    TWI0_Start();
    TWI0_Write((BH1750_ADDRESS << 1) | 1);
    lux = (TWI0_Read_ACK() << 8);
    lux |= TWI0_Read_NACK();
    TWI0_Stop();

    return lux / 1.2; // Convert raw data to lux
}

// Initialize TWI1
void TWI1_Init(void) {
    TWBR1 = 32; // Set SCL frequency ~100kHz
    TWSR1 = 0x00; // Prescaler factor is 1
    TWCR1 = (1 << TWEN); // Enable TWI1
}

// Generate a START condition for TWI1
void TWI1_Start(void) {
    TWCR1 = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR1 & (1 << TWINT)));
}

// Generate a STOP condition for TWI1
void TWI1_Stop(void) {
    TWCR1 = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
    _delay_us(10);
}

// Write a byte of data to TWI1
void TWI1_Write(uint8_t data) {
    TWDR1 = data;
    TWCR1 = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR1 & (1 << TWINT)));
}

// Read a byte of data from TWI1 with ACK
uint8_t TWI1_Read_ACK(void) {
    TWCR1 = (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR1 & (1 << TWINT)));
    return TWDR1;
}

// Read a byte of data from TWI1 without ACK
uint8_t TWI1_Read_NACK(void) {
    TWCR1 = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR1 & (1 << TWINT)));
    return TWDR1;
}

// Initialize MPU6050 sensor
void MPU6050_Init(void) {
    TWI1_Start();
    TWI1_Write(MPU6050_ADDRESS << 1);
    TWI1_Write(PWR_MGMT_1);
    TWI1_Write(0x00); // Wake up MPU6050
    TWI1_Stop();
}

// Read a 16-bit register value from MPU6050
int16_t MPU6050_ReadRegister16(uint8_t reg) {
    int16_t value;

    TWI1_Start();
    TWI1_Write(MPU6050_ADDRESS << 1);
    TWI1_Write(reg);
    TWI1_Start();
    TWI1_Write((MPU6050_ADDRESS << 1) | 1);
    value = (TWI1_Read_ACK() << 8);
    value |= TWI1_Read_NACK();
    TWI1_Stop();

    return value;
}

// Convert raw acceleration data to g
float calculate_acceleration(int16_t raw) {
    return raw / 16384.0;
}
void SPI_init(void) {
    // Set MOSI, SCK and CS as input, MISO as output
    DDRB &= ~(1 << CS) | (1 << MOSI) | (1 << SCK);
    DDRB |= (1 << MISO);
    
    // Enable SPI, set as slave
    SPCR0 |= (1 << SPE);
    SPCR0 &= ~(1 << MSTR) | (1 << CPOL) | (1 << CPHA);
}
void SPI_transmit(uint8_t data)
{
    /* Start transmission */
    SPDR0 = data;
    /* Wait for transmission complete */
    while(!(SPSR0 & (1<<SPIF)));
}
uint8_t SPI_receive()
{
    // transmit dummy byte to receive
    SPDR0 = 0xFF;

    // Wait for reception complete
    while(!(SPSR0 & (1 << SPIF)));

    // return Data Register
    return SPDR0;
}

void SPI_transmitJSON(const char* sensor, float value) {
    char jsonBuffer[64];
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"sensor\":\"%s\",\"value\":%.2f}", sensor, value);
    UART_putstring(jsonBuffer);

    // Start transmission
    // Wait for CS to go low
    while(PINB & (1 << CS));

    // Send each byte of the JSON string
    char* ptr = jsonBuffer;
    while (*ptr) {
        SPI_transmit(*ptr);
        _delay_us(10);  // Add delay between bytes
        ptr++;
    }

    // Send null terminator
     _delay_ms(100); 
    SPI_transmit('\0');
     // Delay before next transmission
    UART_putstring("sent\n");
}

void SPI_receiveJSON() {
    char buffer[64];
    uint8_t idx = 0;
    char sensor[16];
    float value = 0.0;
    
    // Wait for CS to go low (active)
    while(PINB & (1 << CS));
    
    // Receive JSON string
    while(1) {
        char received = SPI_receive();
        if(received == '\0') break;
        
        if(idx < sizeof(buffer)-1) {
            buffer[idx++] = received;
        }
    }
    buffer[idx] = '\0';
    
    // Parse JSON string manually
    char* ptr = buffer;
    
    // Find "sensor" field
    ptr = strstr(buffer, "\"sensor\":\"");
    if(ptr) {
        ptr += 10;  // Skip "sensor":"
        uint8_t i = 0;
        while(*ptr != '\"' && i < sizeof(sensor)-1) {
            sensor[i++] = *ptr++;
        }
        sensor[i] = '\0';
    }
    
    // Find "value" field
    ptr = strstr(buffer, "\"value\":");
    if(ptr) {
        ptr += 8;  // Skip "value":
        value = atof(ptr);
    }
    
    // Print parsed values via UART
    char me[40];
    sprintf(me,"{\"sensor\":\"%s\",\"value\":%.2f}\n", sensor, value);
    UART_putstring("received:");
    UART_putstring(me);
    
    // Wait for CS to go high (inactive)
    while(!(PINB & (1 << CS)));
}



int main(void) {
    SPI_init();
    pwm_init_timer2(); // EQ Buzzer

    // DDRD |= (1 << MOTOR_PIN); // Earthquke Motor PD0 Output
    //LED Line setup
    struct cRGB led[NUM_LEDS];
    ws2812_setleds(led, 1);

    //  LED line
    for (uint8_t i = 0; i < NUM_LEDS; i++) {
        led[i].r = 0;
        led[i].g = 0;
        led[i].b = 0;
    }
    
    char buffer[64];

    UART_init(BAUD_PRESCALER); // Initialize UART communication
    TWI0_Init(); // Initialize TWI0
    TWI1_Init(); // Initialize TWI1

    BH1750_Init(); // Initialize BH1750
    MPU6050_Init(); // Initialize MPU6050
    
    setup_motor();  // Initialize curtain motor pins
    GPIO_init();// For remotely curtain control  
    PWM_init();//Servo motor init
//    sei();        // Enable global interrupts

//    UART_putstring("Sensors Initialized\r\n");
    int curtain_last_state = 0; //0 : curtain's last state is cloesd; 1 : open
    
    //int Auto = 1; // 1 :Curtain automaticlly controlled by Light sensor; 0 :control by hand remotely

    while (1) {
		    

   
        // Read acceleration data from MPU6050
        int16_t accel_x = MPU6050_ReadRegister16(ACCEL_XOUT_H);
        int16_t accel_y = MPU6050_ReadRegister16(ACCEL_XOUT_H + 2);
        int16_t accel_z = MPU6050_ReadRegister16(ACCEL_XOUT_H + 4);

        float accel_x_g = calculate_acceleration(accel_x);
        float accel_y_g = calculate_acceleration(accel_y);
        float accel_z_g = calculate_acceleration(accel_z);

        float total_acceleration = sqrt(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);

//        snprintf(buffer, sizeof(buffer), "Accel Total: %.2f g\r\n", total_acceleration);
        UART_putstring(buffer);
        //SPI_transmitJSON("ds/Gyroscope", total_acceleration);

        // Control logic for earthquake detection
        if (total_acceleration > 1.50) {
            UART_putstring("Earthquake detected!\r\n");
            //PORTD |= (1 << MOTOR_PIN);// Earthquake alarm
            buzzer_on(); //EQ Buzzer
            led_on();                // Turn the LED on
            _delay_ms(10000);        // Keep the LED on
            //PORTD &= ~(1 << MOTOR_PIN); // Stop Earthquake alarm
            led_off();               // Turn the LED off
            buzzer_off();     //EQ Buzzer       
            
        }
               
        // Control logic for light and curtain
//        if(PIND & (1 << CURTAIN)){
//            motor_forward();
//            _delay_ms(5000);  
//            motor_stop();
//            SPI_transmitJSON("ds/Curtain", 1); 
//            
//        }
//        else if(!(PIND & (1 << CURTAIN))){
//            motor_backward();
//            _delay_ms(5000);  
//            motor_stop();  
//            SPI_transmitJSON("ds/Curtain", 0); 
           uint16_t lux = BH1750_ReadLightLevel();
           snprintf(buffer, sizeof(buffer), "Lux: %u\r\n", lux);
            UART_putstring(buffer);
            if(lux >= 200 && lux <= 3000){
                motor_forward(); //Curtain open 
                _delay_ms(5000); 
                motor_stop();   
                SPI_transmitJSON("ds/Curtain", 1);
            }
            else if(lux >= 0 && lux < 200){
                motor_backward();//Curtain closed
                _delay_ms(5000);   
                motor_stop();  
                SPI_transmitJSON("ds/Curtain", 0); 
                }
            
        
        
        if(PIND & (1 << DOOR)){
            _delay_ms(50); // delay
            if(PIND & (1 << DOOR)){
                set_servo_angle(900);
                SPI_transmitJSON("ds/Door", 1);
                _delay_ms(100); // Delay 1 second
            }
        }
        else if (!(PIND & (1 << DOOR))) {
            set_servo_angle(0);
            SPI_transmitJSON("ds/Door", 0);
            _delay_ms(100); // Delay 1 second
            
        }

        
        
        //LedLine Automaticlly
        // Read light level from BH1750

       if (lux >= 0 && lux < 10 ) {
            UART_putstring("MAX LEDLINE \r\n");
            
            // Update LED line mode    
            ws2812_setleds(led, 30);//read/sleep mode
            _delay_ms(1000);  

        }  else if (lux >= 10 && lux < 200  ) { // Auto == 1 &&
            UART_putstring("MID LEDLINE\r\n");
            ws2812_setleds(led, 16);//music/relax mode
            _delay_ms(1000);
        }  else if (lux >= 200 && lux <= 3000) { // Auto == 1 &&
            UART_putstring("Least LEDLINE\r\n");

            // Update LED line mode    
            ws2812_setleds(led, 3);
            _delay_ms(100);  
        } 
        
    }

}