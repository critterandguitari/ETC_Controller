//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"

extern "C" {
#include "uart.h"
#include "stm32f0xx.h"
#include "Timer.h"
#include "BlinkLed.h"
#include "ssd1306.h"
#include "midi.h"
}

#include "OSC/OSCMessage.h"
#include "SLIPEncodedSerial.h"
#include "OSC/SimpleWriter.h"

// MIDI buffers
extern uint8_t uart1_recv_buf[];
extern uint16_t uart1_recv_buf_head;
extern uint16_t uart1_recv_buf_tail;

// then the MIDI stuff gets packed in this blob
extern uint8_t midi_blob[23];  // 5 bytes CC, 16 bytes for note states, 1 byte sync number, 1 byte program
uint8_t midi_blob_sent = 1;  // flag so midi blob only goes out 1 / frame

// ADC DMA stuff
#define ADC1_DR_Address    0x40012440
__IO uint16_t RegularConvData_Tab[9];

// keys and knobs
uint8_t keyValuesRaw[10];
uint8_t keyValues[4][10];
uint8_t keyValuesLast[10];
uint32_t knobValues[6];

// OSC stuff
SLIPEncodedSerial slip;
SimpleWriter oscBuf;

// for outputting to screen
uint8_t spi_out_buf[130];
uint8_t spi_out_buf_remaining = 0;
uint8_t spi_out_buf_index = 0;

// OLED frame
extern uint8_t pix_buf[];

//// hardware init
static void ADC_Config(void);
static void DMA_Config(void);
void hardwareInit(void);

// OSC callbacks
// ok so heres whats going on:
// ETC sends a newFrame message before rendering a new frame.
// then we wait 20ms or so to allow MIDI to accumulate before sending it back
// this will arrive just in time for the next frame
void ledControl(OSCMessage &msg);
void shutdown(OSCMessage &msg);
void newFrame(OSCMessage &msg);
// end OSC callbacks

// for sending OSC back (knobs and MIDI,  the keys and fs get sent when they change on poll)
void sendKnobs(void);
void sendMIDI(void);

/// scan keys
uint32_t scanKeys();
void remapKeys();
void checkForKeyEvent();
void updateKnobs() ;

//foot
void checkFootSwitch (void) ;

// checks midi flags and packs data into the midi_blob
void packMIDI(void);

int main(int argc, char* argv[]) {

	OSCMessage msgIn;

	blink_led_init();
	blink_led_off();

	timer_start();

	// flash leds while power stabilizes
	// before initializing ADC
	stopwatchStart();
	while (stopwatchReport() < 500){ AUX_LED_GREEN_ON; }
	stopwatchStart();
	while (stopwatchReport() < 500){ AUX_LED_GREEN_ON; }
	stopwatchStart();
	while (stopwatchReport() < 500){ AUX_LED_GREEN_ON; }
	stopwatchStart();

	AUX_LED_RED_OFF;
	AUX_LED_GREEN_OFF;
	AUX_LED_BLUE_OFF;

	uart2_init();

	hardwareInit();

	midi_init(1);

	int progress = 0;

	stopwatchStart();

	// blue while ETC booting
	AUX_LED_GREEN_ON;

	// waiting for /ready command
	while (1) {
		if (slip.recvMessage()) {
			msgIn.fill(slip.decodedBuf, slip.decodedLength);
			if (!msgIn.hasError()) {
				// wait for start message so we aren't sending stuff during boot
				if (msgIn.fullMatch("/ready", 0)) {
					msgIn.empty();
					break;
				}
				msgIn.empty();
			} else {   // just empty it if there was an error
				msgIn.empty();
			}
		}
		// after 15 seconds, something is wrong with bootup, switch LED to error
		if (stopwatchReport() > 1500) {
			stopwatchStart();
			if (progress < 99)
				progress++;
			else {
				AUX_LED_GREEN_OFF;
				AUX_LED_RED_ON;
			}
		}

	} // waiting for /ready command

	stopwatchStart();   // used to check encoder only 1 per 5 ms

	while (1) {

		// check for midi,  new midi stuff gets put in the midi_blob
		while (uart1_recv_buf_tail != uart1_recv_buf_head) {
			uint8_t tmp8 = uart1_recv_buf[uart1_recv_buf_tail++];
			uart1_recv_buf_tail %= UART1_BUFFER_SIZE;
			recvByte(tmp8);
		} // gettin MIDI bytes


		if (slip.recvMessage()) {
			// fill the message and dispatch it

			msgIn.fill(slip.decodedBuf, slip.decodedLength);

			// dispatch it
			if (!msgIn.hasError()) {
				msgIn.dispatch("/led", ledControl, 0);
				msgIn.dispatch("/shutdown", shutdown, 0);
				msgIn.dispatch("/nf", newFrame, 0);
				msgIn.empty();
			} else {   // just empty it if there was an error
				msgIn.empty();
			}
		}

		// every time mux gets back to 0 key scan is complete
		scanKeys();
		checkForKeyEvent(); // and send em out if we got em

		// also check about the foot switch
		checkFootSwitch();

		// get the values from DMA
		updateKnobs();

		// the /nf (newFrame) osc message restarts stop watch
		// after 25 ms towards the end of the frame, send the midi_blob back
		if (stopwatchReport() > 250){
			if (!midi_blob_sent) {
				sendMIDI();
				sendKnobs();
				midi_blob_sent = 1;
			}
		}

	} // Infinite loop, never return.
}

void hardwareInit(void){

	/* DMA configuration */
	DMA_Config();

	/* ADC1 configuration */
	ADC_Config();

	// key lines
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// foot switch
	/*RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);*/

}

//// ADC DMA stuff
static void ADC_Config(void) {
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	/* ADC1 DeInit */
	ADC_DeInit(ADC1);

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Configure  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);

	/* Configure the ADC1 in continuous mode withe a resolution equal to 12 bits  */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_10b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Convert the ADC1 Channel11 and channel10 with 55.5 Cycles as sampling time */
	ADC_ChannelConfig(ADC1, ADC_Channel_14, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_15, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_8, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_9, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_55_5Cycles);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* ADC DMA request in circular mode */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable the ADC peripheral */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait the ADRDY flag */
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY))
		;

	/* ADC1 regular Software Start Conv */
	ADC_StartOfConversion(ADC1);
}

/**
 * @brief  DMA channel1 configuration
 * @param  None
 * @retval None
 */
static void DMA_Config(void) {
	DMA_InitTypeDef DMA_InitStructure;
	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) RegularConvData_Tab;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 6;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);

}

//// end ADC DMA

// OSC callbacks
void newFrame(OSCMessage &msg){
	midi_blob_sent = 0;
	stopwatchStart();  // start timer on new frame, when it gets to 25 ms
}

void ledControl(OSCMessage &msg) {

	blink_led_on();

	int stat;

	// digitalWrite(ledPin, LOW);
	if (msg.isInt(0)) {
		stat = msg.getInt(0);

		stat %= 8;

		if (stat == 0) {
			AUX_LED_RED_OFF;
			AUX_LED_GREEN_OFF;
			AUX_LED_BLUE_OFF;
		}
		if (stat == 1) {
			AUX_LED_RED_OFF;
			AUX_LED_GREEN_OFF;
			AUX_LED_BLUE_ON;
		}
		if (stat == 2) {
			AUX_LED_RED_OFF;
			AUX_LED_GREEN_ON;
			AUX_LED_BLUE_OFF;
		}
		if (stat == 3) {
			AUX_LED_RED_OFF;
			AUX_LED_GREEN_ON;
			AUX_LED_BLUE_ON;
		}
		if (stat == 4) {
			AUX_LED_RED_ON;
			AUX_LED_GREEN_OFF;
			AUX_LED_BLUE_OFF;
		}
		if (stat == 5) {
			AUX_LED_RED_ON;
			AUX_LED_GREEN_OFF;
			AUX_LED_BLUE_ON;
		}
		if (stat == 6) {
			AUX_LED_RED_ON;
			AUX_LED_GREEN_ON;
			AUX_LED_BLUE_OFF;
		}
		if (stat == 7) {
			AUX_LED_RED_ON;
			AUX_LED_GREEN_ON;
			AUX_LED_BLUE_ON;
		}
	}
}

void shutdown(OSCMessage &msg) {

	int i;
	char progressStr[20];
	int len = 0;
	int progress = 0;

	// clear screen
	for (i = 0; i < 1024; i++) {
		pix_buf[i] = 0;
	}

	stopwatchStart();
	while (progress < 99) {
		if (stopwatchReport() > 500) {
			stopwatchStart();
			progress++;
			// LED shutdown color here
		}
	}

	// LED off, shutdown complete

	for (;;)
		;  // endless loop here
}

// end OSC callbacks

// sending MIDI blob back
void sendMIDI(void) {

	OSCMessage msgMIDI("/mblob"); // blob of midi crap

	msgMIDI.add(midi_blob, 23);

	msgMIDI.send(oscBuf);
	slip.sendMessage(oscBuf.buffer, oscBuf.length);
	msgMIDI.empty();
}

// sending knob values back
void sendKnobs(void) {

	OSCMessage msgKnobs("/knobs");

	uint32_t i;
	for (i = 0; i < 6; i++) {
		msgKnobs.add((int32_t) knobValues[i]);
	}

	msgKnobs.send(oscBuf);
	slip.sendMessage(oscBuf.buffer, oscBuf.length);
	msgKnobs.empty();
}

/// scan keys
uint32_t scanKeys() {

		keyValuesRaw[0] = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)) ? 0 : 100; // k1, SD
		keyValuesRaw[1] = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)) ? 0 : 100; // k2, PM
		keyValuesRaw[2] = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)) ? 0 : 100; // k3, NM

		keyValuesRaw[3] = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9)) ? 0 : 100; // k4, OSD
		keyValuesRaw[4] = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10)) ? 0 : 100; // k5, PP
		keyValuesRaw[5] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)) ? 0 : 100; // k6, NP

		keyValuesRaw[6] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)) ? 0 : 100; // k7, SP
		keyValuesRaw[7] = (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)) ? 0 : 100; // k8, CLR
		keyValuesRaw[8] = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)) ? 0 : 100; // 9, AC
		keyValuesRaw[9] = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)) ? 0 : 100; // 10, PIC

}

void remapKeys() {
	static uint32_t cycleCount = 0;

	keyValues[cycleCount][0] = keyValuesRaw[0];
	keyValues[cycleCount][1] = keyValuesRaw[1];
	keyValues[cycleCount][2] = keyValuesRaw[2];
	keyValues[cycleCount][3] = keyValuesRaw[3];
	keyValues[cycleCount][4] = keyValuesRaw[4];
	keyValues[cycleCount][5] = keyValuesRaw[5];
	keyValues[cycleCount][6] = keyValuesRaw[6];
	keyValues[cycleCount][7] = keyValuesRaw[7];
	keyValues[cycleCount][8] = keyValuesRaw[8];
	keyValues[cycleCount][9] = keyValuesRaw[9];


	cycleCount++;
	cycleCount &= 0x3;  // i between 0-3
}



void checkForKeyEvent() {
	remapKeys();
	uint32_t i;

	for (i = 0; i < 10; i++) {
		if ((keyValues[0][i]) && (keyValues[1][i]) && (keyValues[2][i])
				&& (keyValues[3][i])) {

			if (!keyValuesLast[i]) {
				OSCMessage msgKey("/key");

				msgKey.add((int32_t) i);
				msgKey.add((int32_t) 100);

				msgKey.send(oscBuf);
				slip.sendMessage(oscBuf.buffer, oscBuf.length);

				msgKey.empty(); // free space occupied by message
				keyValuesLast[i] = 100;
			}
		}
		if ((!keyValues[0][i]) && (!keyValues[1][i]) && (!keyValues[2][i])
				&& (!keyValues[3][i])) {
			if (keyValuesLast[i]) {
				OSCMessage msgKey("/key");

				msgKey.add((int32_t) i);
				msgKey.add((int32_t) 0);

				msgKey.send(oscBuf);
				slip.sendMessage(oscBuf.buffer, oscBuf.length);

				msgKey.empty();
				keyValuesLast[i] = 0;
			}
		}
	}
}

/// end keys

void updateKnobs() {

	// see if a new conversion is ready
	if ((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == SET) {
		DMA_ClearFlag(DMA1_FLAG_TC1);

		knobValues[4] = RegularConvData_Tab[0];
		knobValues[0] = RegularConvData_Tab[1];
		knobValues[1] = RegularConvData_Tab[2];
		knobValues[3] = RegularConvData_Tab[3];
		knobValues[2] = RegularConvData_Tab[4];
		knobValues[5] = RegularConvData_Tab[5];
	}
}

// check for foot switch change
void checkFootSwitch (void) {
	static uint8_t foot_last = 0;
	static uint8_t foot_debounce[2] = {0, 0};
	static uint8_t foot_debounce_count = 0;

	if (knobValues[5] < 100) foot_debounce[foot_debounce_count] = 0;
	if (knobValues[5] > 900) foot_debounce[foot_debounce_count] = 1;

	foot_debounce_count++;
	foot_debounce_count &= 1;
	//only proceed if debounced (same for 2 times)
	if (foot_debounce[0] == foot_debounce[1]){
		if ((knobValues[5] < 100) && foot_last){
			foot_last = 0;
			// send press
			OSCMessage msgEncoder("/fs");
			msgEncoder.add(1);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
		}
		if ((knobValues[5] > 900) && !foot_last){
			foot_last = 1;
			// send press
			OSCMessage msgEncoder("/fs");
			msgEncoder.add(0);
			msgEncoder.send(oscBuf);
			slip.sendMessage(oscBuf.buffer, oscBuf.length);
		}
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
