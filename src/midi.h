/*
 * midi.h
 *
 *  Created on: Jul 25, 2012
 *      Author: owen
 */

#ifndef MIDI_H_
#define MIDI_H_
#include "stm32f0xx.h"

/*************** MIDI **********************/
/* Private Receive Parameters */
// The channel this Midi instance receives data for (0 means all channels)
int channelIn_;

// These are for keeping track of partial Midi messages as bytes come in
int recvMode_;
int recvByteCount_;
int recvEvent_;
int recvArg0_;
int recvBytesNeeded_;
int lastStatusSent_;

/* Private Send Parameters */

// This controls whether every Midi message gets a command byte sent with it
//  (Midi can just send a single command byte and then stream events without
//  sending the command each time)
uint8_t sendFullCommands_;


/* Internal functions */

void midi_init(uint8_t ch);
// Handle decoding incoming MIDI traffic a byte at a time -- remembers
//  what it needs to from one call to the next.
void recvByte(int byte);
void put_char(uint8_t c);

// Set this parameter to anything other than 0 to cause every MIDI update to
//  include a copy of the current event (e.g. for every note off to include
//  the NOTE OFF event) -- by default, if an event is the same type of event as
//  the last one sent, the event byte isn't sent; just the new note/velocity
static const unsigned int PARAM_SEND_FULL_COMMANDS = 0x1000;

// Use this parameter to update the channel the MIDI code is looking for messages
//  to.  0 means "all channels".
static const unsigned int PARAM_CHANNEL_IN         = 0x1001;


// Changes the updateable parameters (params are PARAM_SEND_FULL_COMMANDS or
//  PARAM_CHANNEL_IN -- see above for what they mean)
void setParam(unsigned int param, unsigned int val);

// Get current values for the user updateable parameters (params, etc same as above)
unsigned int getParam(unsigned int param);


// Call these to send MIDI messages of the given types
void sendNoteOff(unsigned int channel, unsigned int note, unsigned int velocity);
void sendNoteOn(unsigned int channel, unsigned int note, unsigned int velocity);
void sendVelocityChange(unsigned int channel, unsigned int note, unsigned int velocity);
void sendControlChange(unsigned int channel, unsigned int controller, unsigned int value);
void sendProgramChange(unsigned int channel, unsigned int program);
void sendAfterTouch(unsigned int channel, unsigned int velocity);
void sendPitchChange(unsigned int pitch);
void sendSongPosition(unsigned int position);
void sendSongSelect(unsigned int song);
void sendTuneRequest(void);
void sendSync(void);
void sendStart(void);
void sendContinue(void);
void sendStop(void);
void sendActiveSense(void);
void sendReset(void);

void handleNoteOff(unsigned int channel, unsigned int note, unsigned int velocity);
void handleNoteOn(unsigned int channel, unsigned int note, unsigned int velocity);
void handleVelocityChange(unsigned int channel, unsigned int note, unsigned int velocity);
void handleControlChange(unsigned int channel, unsigned int controller, unsigned int value);
void handleProgramChange(unsigned int channel, unsigned int program);
void handleAfterTouch(unsigned int channel, unsigned int velocity);
void handlePitchChange(unsigned int pitch);
void handleSongPosition(unsigned int position);
void handleSongSelect(unsigned int song);
void handleTuneRequest(void);
void handleSync(void);
void handleStart(void);
void handleContinue(void);
void handleStop(void);
void handleActiveSense(void);
void handleReset(void);

// This is used for tracking when we're processing a proprietary stream of data
//  The assigned value is arbitrary; just for internal use.
#define MODE_PROPRIETARY 0xff


//  midi status message types
#define STATUS_EVENT_NOTE_OFF 0x80
#define STATUS_EVENT_NOTE_ON 0x90
#define STATUS_EVENT_VELOCITY_CHANGE 0xA0
#define STATUS_EVENT_CONTROL_CHANGE 0xB0
#define STATUS_EVENT_PROGRAM_CHANGE 0xC0
#define STATUS_AFTER_TOUCH 0xD0
#define STATUS_PITCH_CHANGE 0xE0
#define STATUS_START_PROPRIETARY 0xF0
#define STATUS_SONG_POSITION 0xF2
#define STATUS_SONG_SELECT 0xF3
#define STATUS_TUNE_REQUEST 0xF6
#define STATUS_END_PROPRIETARY 0xF7
#define STATUS_SYNC 0xF8
#define STATUS_START 0xFA
#define STATUS_CONTINUE 0xFB
#define STATUS_STOP 0xFC
#define STATUS_ACTIVE_SENSE 0xFE
#define STATUS_RESET 0xFF



#endif /* MIDI_H_ */
