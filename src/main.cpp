/********************************************************************************
	Includes
********************************************************************************/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>

#include "../nrf24l01/RF24.h"
#include "../atmega328/mtimer.h"
#include "../common/util.h"
#include "../BME280/BME280.h"

extern "C" {
#include "../atmega328/usart.h"
}

/********************************************************************************
	Macros and Defines
********************************************************************************/
#define CHANNEL                 110
#define SENSOR_DATA_KEY         100
#define SENSOR_ID               2

#define EVENT_START             0
#define EVENT_TEMPERATURE       1
#define EVENT_HUMIDITY          2
#define EVENT_PRESSURE          3
#define EVENT_BATTERY           4

#define SENSOR_SEND_CYCLES      3600 / 8 // 1 cycle each 8 seconds, send data each hour (3600 seconds)
#define BATTERY_SEND_CYCLES     24

/********************************************************************************
	Function Prototypes
********************************************************************************/
void initTimer2();
void sensorLoop();
void initGPIO();
void initRadio();
uint16_t adc_read(uint8_t adcx);
void sendSensorData();
void sendData(uint8_t event, uint8_t data_high, uint8_t data_low);
void powerUpAVR();
void powerDownAVR();

/********************************************************************************
	Global Variables
********************************************************************************/
volatile uint16_t timer2_count = SENSOR_SEND_CYCLES;
volatile uint8_t battery_count = BATTERY_SEND_CYCLES;

RF24 radio;
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

BME280 bme;

/********************************************************************************
	Interrupt Service
********************************************************************************/
ISR(USART_RX_vect)
{
	handle_usart_interrupt();
}

ISR(TIMER1_OVF_vect)
{
	incrementOvf();
}

ISR(TIMER2_OVF_vect)
{
	_NOP();
}

/********************************************************************************
	Main
********************************************************************************/
int main(void) {
    // initialize code
	usart_init();

    // init GPIO
    initGPIO();

    // enable interrupts
    sei();

    // Configure Sleep Mode - Power-save
    SMCR = (0<<SM2)|(1<<SM1)|(1<<SM0)|(0<<SE);

	// Output initialization log
    printf("Start...");
    printf(CONSOLE_PREFIX);

    // Init Radio Module
    initRadio();

    // Init Timer 2
    initTimer2();

    if (!bme.begin()) {
    	printf("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    powerDownAVR();

	// main loop
    while (1) {
    	// main usart loop
    	//usart_check_loop();

    	sensorLoop();
    }
}

/********************************************************************************
	Functions
********************************************************************************/
void sensorLoop() {
	// each hour send the data
	if (++timer2_count >= SENSOR_SEND_CYCLES) {
		timer2_count = 0;

		powerUpAVR();
		sendSensorData();
		powerDownAVR();

		// Wait a little before going to sleep again
	    _delay_ms(10);
	}

	_delay_ms(5);

	// Sleep mode to save battery, Timer 2 will wake up once each 8 seconds
	sleep_mode();
}

void initTimer2() {
    // Init timer2
    //Disable timer2 interrupts
    TIMSK2  = 0;

    //Enable asynchronous mode
    ASSR  = (1<<AS2);
    //set initial counter value
    TCNT2=0;
    //set prescaler 1024
    TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS00);
    //wait for registers update
    while (ASSR & ((1<<TCN2UB)|(1<<TCR2BUB)));
    //clear interrupt flags
    TIFR2  = (1<<TOV2);
    //enable TOV2 interrupt
    TIMSK2  = (1<<TOIE2);
}

void initRadio() {
    radio.begin();
    radio.setRetries(15,15);
    radio.setPayloadSize(8);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setChannel(CHANNEL);

    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);

    radio.printDetails();

	_delay_ms(10);

	// need to flush tx buffer, fixed the issue with packet shift...
	radio.stopListening();

    // Send init token
	uint8_t data[] = {SENSOR_DATA_KEY, SENSOR_ID, EVENT_START, 0, 0};
	radio.write(data, 5);

	_delay_ms(10);
}

void handle_usart_cmd(char *cmd, char *args) {
	if (strcmp(cmd, "test") == 0) {
		printf("\n TEST [%s]", args);
	}

	if (strcmp(cmd, "send") == 0) {
		sendSensorData();
	}
}

void initGPIO() {
    _in(DDD3, DDRD); // INT1 input - not used yet in this device...
    _in(DDC0, DDRC); // Analog input 0
}

uint16_t adc_read(uint8_t adcx) {
	/* adcx is the analog pin we want to use.  ADMUX's first few bits are
	 * the binary representations of the numbers of the pins so we can
	 * just 'OR' the pin's number with ADMUX to select that pin.
	 * We first zero the four bits by setting ADMUX equal to its higher
	 * four bits. */
	ADMUX = adcx;
	ADMUX |= (1<<REFS1)|(1<<REFS0)|(0<<ADLAR);

	_delay_us(300);

	/* This starts the conversion. */
	ADCSRA |= _BV(ADSC);

	/* This is an idle loop that just wait around until the conversion
	 * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
	 * set above, to see if it is still set.  This bit is automatically
	 * reset (zeroed) when the conversion is ready so if we do this in
	 * a loop the loop will just go until the conversion is ready. */
	while ( (ADCSRA & _BV(ADSC)) );

	/* Finally, we return the converted value to the calling function. */
	return ADC;
}

void sendSensorData() {
	int16_t t = (int16_t) (bme.readTemperature() * 100.00f);
	int16_t h = (int16_t) (bme.readHumidity() * 100.00f);
	int16_t p = (int16_t) (bme.readPressure() / 100.0F);

    uint8_t t_low = (uint8_t) t;
    uint8_t t_high = (uint8_t) (t>>8);

    uint8_t h_low = (uint8_t) h;
    uint8_t h_high = (uint8_t) (h>>8);

    uint8_t p_low = (uint8_t) p;
    uint8_t p_high = (uint8_t) (p>>8);

    // Send temperature via NRF24L01 transceiver
    sendData(EVENT_TEMPERATURE, t_high, t_low);

    // Send humidity via NRF24L01 transceiver
    sendData(EVENT_HUMIDITY, h_high, h_low);

    // Send pressure via NRF24L01 transceiver
    sendData(EVENT_PRESSURE, p_high, p_low);

    if (++battery_count >= BATTERY_SEND_CYCLES) {
        // Enable the ADC
        ADCSRA |= _BV(ADEN);
        _delay_ms(5);

    	battery_count = 0;

    	uint16_t mux0Value = adc_read(MUX0);

        uint8_t b_low = (uint8_t) mux0Value;
        uint8_t b_high = (uint8_t) (mux0Value>>8);

        // Send battery level via NRF24L01 transceiver
        sendData(EVENT_BATTERY, b_high, b_low);

        // Disable ADC
        ADCSRA &= ~_BV(ADEN);
    }
}

void sendData(uint8_t event, uint8_t data_high, uint8_t data_low) {
	uint8_t data[] = {SENSOR_DATA_KEY, SENSOR_ID, event, data_high, data_low};
	radio.write(data, 5);

	_delay_ms(10);

	// need to flush tx buffer, fixed the issue with packet shift...
	radio.stopListening();

	// give time to receiver to process the data
	_delay_ms(200);
}

void powerUpAVR() {
	_on(PC0, PORTC); // enable pull-up resistor for ADC battery level

	_out(DDC5, DDRC); // SCL as output
	_out(DDC4, DDRC); // SDA as output

	radio.powerUp();
	bme.normalMode();
}

void powerDownAVR() {
	_off(PC0, PORTC); // disable pull-up resistor for ADC battery level

	_in(DDC5, DDRC); // SCL as input
	_in(DDC4, DDRC); // SDA as input

	radio.powerDown();
	bme.sleepMode();
}
