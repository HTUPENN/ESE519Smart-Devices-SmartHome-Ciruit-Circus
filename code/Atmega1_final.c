#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "uart.h"
#include <string.h>
#include <avr/interrupt.h>

// DHT
#define DHT11_PIN PC1  // Define PC1 as the DHT11 data pin

// Define PWM duty cycle range for SG90 servo motor (1ms to 2ms)
#define SERVO_MIN 64    // Duty cycle for 0° (1ms pulse width, ~5%)
#define SERVO_MAX 256   // Duty cycle for 180° (2ms pulse width, ~10%)

// HCSR501
#define PIR_SENSOR_PIN PINB1   // PIR sensor output connected to PB1
#define BUZZER_PIN     PD3   // Buzzer connected to PD3
#define LIGHT_PIN      PD2   // Light (LED) connected to PB3

// Fans
#define MOTOR_PIN PC5      // Define PB0 as the motor control pin

//SPI
#define CS      PB2
#define MOSI    PB3
#define MISO    PB4
#define SCK     PB5

// Remote control
#define FAN PD0
#define WINDOW PD1

volatile uint8_t spi_flag = 0; // 0 for recive, 1 for continuous
////Motor For Automatic Window
//// Initialize Timer0 for PWM generation
void PWM_init() {
    // Set Timer0 to Fast PWM mode, non-inverting mode
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
    TCCR0B = (1 << CS01) | (1 << CS00); // Set prescaler to 64
    DDRD |= (1 << PD6); // Set PD6 (OC0A) as output
}

//// Function to set servo motor angle
void set_servo_angle(uint8_t angle) {
    // Map angle (0-180°) to duty cycle (SERVO_MIN to SERVO_MAX)
    uint8_t duty = SERVO_MIN + (angle * (SERVO_MAX - SERVO_MIN) / 180);
    OCR0A = duty; // Update duty cycle for PWM
}


//DHT11
void DHT11_Request() {
    // Set the DHT11_PIN as output
    DDRC |= (1 << DHT11_PIN);
    // Pull the pin low for at least 18ms
    PORTC &= ~(1 << DHT11_PIN);
    _delay_ms(18);
    // Pull the pin high for 20-40us
    PORTC |= (1 << DHT11_PIN);
    _delay_us(30);
    // Set the pin as input to read data
    DDRC &= ~(1 << DHT11_PIN);
}

uint8_t DHT11_Response() {
    _delay_us(40);
    if (!(PINC & (1 << DHT11_PIN))) {  // Wait for low signal
        _delay_us(80);
        if (PINC & (1 << DHT11_PIN)) {  // Wait for high signal
            _delay_us(40);
            return 1;  // Sensor responded
        }
    }
    return 0;  // No response
}

uint8_t DHT11_ReadByte() {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        while (!(PINC & (1 << DHT11_PIN)));  // Wait for the pin to go high
        _delay_us(30);
        if (PINC & (1 << DHT11_PIN))  // If pin is high after 30us, it's a '1'
            data = (data << 1) | 1;
        else  // Otherwise, it's a '0'
            data = (data << 1);
        while (PINC & (1 << DHT11_PIN));  // Wait for the pin to go low
    }
    return data;
}


void setup_HCSR502() {
    // Configure PIR_SENSOR_PIN as input
    DDRB &= ~(1 << PIR_SENSOR_PIN);
}

// ADC-MQ-9B
void ADC_init() {
    ADMUX = (1<<REFS0);  // Set Vref to AVcc
    ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);  // ADC prescaler of 128
    ADCSRA |= (1<<ADEN);  // Enable ADC
}

uint16_t ADC_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);  // Select channel
    ADCSRA |= (1<<ADSC);  // Start conversion
    while(ADCSRA & (1<<ADSC));  // Wait for conversion
    return ADC;
}

// Fans
void setup_fans(){
    DDRB |= (1 << MOTOR_PIN); // Configure MOTOR_PIN (PB0) as output
    PORTB &= ~(1 << MOTOR_PIN); // Ensure motor is off initially
}

void motor_control(uint8_t state) {
    if (state==1) {
        PORTC |= (1 << MOTOR_PIN);  // Set HIGH  // Turn on motor
    } else if(state == 0) {
        PORTC &= ~(1 << MOTOR_PIN);
          // Turn off motor
    }
}

// Buzzer
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

void buzzer_on() {
    //start timer2 
    TCCR2B |= (1 << CS21) | (1 << CS20);    // Prescaler 64
}

void buzzer_off() {
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
}

void light_on() {
    DDRD |= (1 << LIGHT_PIN);
    PORTD |= (1 << LIGHT_PIN); // Set LIGHT_PIN high to turn on the light
}

void light_off() {
    PORTD &= ~(1 << LIGHT_PIN); // Set LIGHT_PIN low to turn off the light
}

//spi
void SPI_init(void) {
    // Set MOSI, SCK and CS as input, MISO as output
    DDRB &= ~(1 << CS) | (1 << MOSI) | (1 << SCK);
    DDRB |= (1 << MISO);
    
    
    // Enable SPI, set as slave
    SPCR0 |= (1 << SPE);
    SPCR0 &= ~(1 << MSTR) | (1 << CPOL) | (1 << CPHA);
}

void Remote_control(void){
    // Window
    DDRD &= ~(1 << FAN) | (1 << WINDOW);
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT16) | (1 << PCINT17);
}

ISR(PCINT2_vect){
    // Fan
    if (PIND & (1 << FAN)) { // Check if GPIO is high
        _delay_ms(50);
        if(PIND & (1 << FAN)){
            motor_control(1);  // Turn on motor
        }
    }
    else if (!(PIND & (1 << FAN))) { // Check if GPIO is low
        _delay_ms(50);
        if(!(PIND & (1 << FAN))){
            motor_control(0);  // Turn off motor
        }
    }
    
    // Window
    if (PIND & (1 << WINDOW)) { // Check if GPIO is high
        _delay_ms(50);
        if(PIND & (1 << WINDOW)){
            set_servo_angle(0);
        }
    }
    else if (!(PIND & (1 << WINDOW))) { // Check if GPIO is low
        _delay_ms(50);
        if(!(PIND & (1 << WINDOW))){
            set_servo_angle(900);
        }
    }
    
    
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
    UART_putstring("received command:");
    UART_putstring(me);
    
    // Wait for CS to go high (inactive)
    while(!(PINB & (1 << CS)));
}


int main(void) {
    UART_init(BAUD_PRESCALER);  // Initialize UART
    SPI_init();
    setup_HCSR502();
    setup_fans();
    pwm_init_timer2();
    PWM_init(); // Initialize PWM
    ADC_init();
    Remote_control();
    sei(); // Enable all interrupts
    
    uint16_t mq9b_value;
    char String[50];
    uint8_t humidity_int, humidity_dec, temp_int, temp_dec, checksum;
    float temperature, humidity;
    uint8_t move_detect, fan_state, window_state;
    while (1) {
        //DHT11
        DHT11_Request();  // Send start signal to DHT11
        if (DHT11_Response()) {  // Wait for DHT11 response
            // Read 40 bits of data
            humidity_int = DHT11_ReadByte();
            humidity_dec = DHT11_ReadByte();
            temp_int = DHT11_ReadByte();
            temp_dec = DHT11_ReadByte();
            checksum = DHT11_ReadByte();

            // Verify checksum
            if ((humidity_int + humidity_dec + temp_int + temp_dec) == checksum) {
                temperature = temp_int + temp_dec * 0.1;
                humidity = humidity_int + humidity_dec * 0.1;
                sprintf(String, "Temperature: %.1fC\r\nHumidity: %.1f\r\n",
                        temperature, humidity);
                UART_putstring(String);
                
            } else {
                sprintf(String, "Checksum error!\r\n");
                UART_putstring(String);
            }
        } else {
            sprintf(String, "DHT11 no response.\r\n");
            UART_putstring(String);
        }
        
        //MQ9B
        mq9b_value = ADC_read(4);  // Read from ADC4 (PC4)
        sprintf(String,"The mq9b_value is %u \n", mq9b_value);
        UART_putstring(String);
        
        //_delay_ms(2000);  // Wait 2 seconds before the next reading
        
        // Control
        // Automatic Drying Rack
        // Sunny, window out
        if (humidity<80){
            window_state = 1;
            UART_putstring("Sunny, window open\r\n");
            set_servo_angle(0);
            _delay_ms(1000); // Delay 1 second
        }
        // Evening, drying rack back
        if (humidity>=80){
            window_state = 0;
            UART_putstring("Rainy, window close\r\n");
            set_servo_angle(900);
            _delay_ms(1000); // Delay 1 second
        }
        
        
        //Motion Detector
        if (PINB & (1 << PIR_SENSOR_PIN)) {
            move_detect = 1;
            sprintf(String, "Motion detected!\n");
            UART_putstring(String);
            light_on();
            buzzer_on();
            _delay_ms(2000); // Keep buzzer on for 500ms
        } else {
            move_detect = 0;
            sprintf(String, "No motion detected!\n");
            UART_putstring(String);
            light_off();
            buzzer_off();
        }
        
        // Control fans based on mq-9B sensor value
        if (mq9b_value > 500) {
            motor_control(1);  // Turn on motor
            fan_state = 1;
            UART_putstring("Motor ON\r\n");
            buzzer_on();
            _delay_ms(1000);
        } else {
            fan_state = 0;
            motor_control(0);  // Turn off motor
            UART_putstring("Motor OFF\r\n");
            buzzer_off();
        }
        
        // SPI
        SPI_transmitJSON("ds/Temperature", temperature);
        SPI_transmitJSON("ds/Humidity", humidity);
        SPI_transmitJSON("ds/Mq9b", mq9b_value);
        SPI_transmitJSON("ds/Fan", fan_state);
        SPI_transmitJSON("ds/Window", window_state);
        
    }
}
