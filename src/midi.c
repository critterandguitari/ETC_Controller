/*
 * midi.c
 *
 *  Created on: Jul 25, 2012
 *      Author: owen
 */

#include "midi.h"
#include "uart.h"

int note_on_flag = 0;

void recvByte(int byte) {
        int tmp;
    int channel;
    int bigval;           /*  temp 14-bit value for pitch, song pos */


    if (recvMode_ & MODE_PROPRIETARY
      && byte != STATUS_END_PROPRIETARY)
    {
        /* If proprietary handling compiled in, just pass all data received
         *  after a START_PROPRIETARY event to proprietary_decode
         *  until get an END_PROPRIETARY event
         */

#ifdef CONFIG_MIDI_PROPRIETARY
        proprietaryDecode(byte);
#endif

        return;
    }

    if (byte & 0x80) {

        /* All < 0xf0 events get at least 1 arg byte so
         *  it's ok to mask off the low 4 bits to figure
         *  out how to handle the event for < 0xf0 events.
         */

        tmp = byte;

        if (tmp < 0xf0)
            tmp &= 0xf0;

        switch (tmp) {
            /* These status events take 2 bytes as arguments */
            case STATUS_EVENT_NOTE_OFF:
            case STATUS_EVENT_NOTE_ON:
            case STATUS_EVENT_VELOCITY_CHANGE:
            case STATUS_EVENT_CONTROL_CHANGE:
            case STATUS_PITCH_CHANGE:
            case STATUS_SONG_POSITION:
                recvBytesNeeded_ = 2;
                recvByteCount_ = 0;
                recvEvent_ = byte;
                break;

            /* 1 byte arguments */
            case STATUS_EVENT_PROGRAM_CHANGE:
            case STATUS_AFTER_TOUCH:
            case STATUS_SONG_SELECT:
                recvBytesNeeded_ = 1;
                recvByteCount_ = 0;
                recvEvent_ = byte;
                return;

            /* No arguments ( > 0xf0 events) */
            case STATUS_START_PROPRIETARY:
                recvMode_ |= MODE_PROPRIETARY;

#ifdef CONFIG_MIDI_PROPRIETARY
                proprietaryDecodeStart();
#endif

                break;
            case STATUS_END_PROPRIETARY:
                recvMode_ &= ~MODE_PROPRIETARY;

#ifdef CONFIG_MIDI_PROPRIETARY
                proprietaryDecodeEnd();
#endif

                break;
            case STATUS_TUNE_REQUEST:
                handleTuneRequest();
                break;
            case STATUS_SYNC:
                handleSync();
                break;
            case STATUS_START:
                handleStart();
                break;
            case STATUS_CONTINUE:
                handleContinue();
                break;
            case STATUS_STOP:
                handleStop();
                break;
            case STATUS_ACTIVE_SENSE:
                handleActiveSense();
                break;
            case STATUS_RESET:
                handleReset();
                break;
        }

        return;
    }

    if (++recvByteCount_ == recvBytesNeeded_) {
        /* Copy out the channel (if applicable; in some cases this will be meaningless,
         *  but in those cases the value will be ignored)
         */
        channel = (recvEvent_ & 0x0f) + 1;

        tmp = recvEvent_;
        if (tmp < 0xf0) {
            tmp &= 0xf0;
        }

        /* See if this event matches our MIDI channel
         *  (or we're accepting for all channels)
         */
        if (!channelIn_
             || (channel == channelIn_)
             || (tmp >= 0xf0))
        {
            switch (tmp) {
                case STATUS_EVENT_NOTE_ON:
                    /* If velocity is 0, it's actually a note off & should fall thru
                     *  to the note off case
                     */
                    if (byte) {
                        handleNoteOn(channel, recvArg0_, byte);
                        break;
                    }

                case STATUS_EVENT_NOTE_OFF:
                    handleNoteOff(channel, recvArg0_, byte);
                    break;
                case STATUS_EVENT_VELOCITY_CHANGE:
                    handleVelocityChange(channel, recvArg0_, byte);
                    break;
                case STATUS_EVENT_CONTROL_CHANGE:
                    handleControlChange(channel, recvArg0_, byte);
                    break;
                case STATUS_EVENT_PROGRAM_CHANGE:
                    handleProgramChange(channel, byte);
                    break;
                case STATUS_AFTER_TOUCH:
                    handleAfterTouch(channel, byte);
                    break;
                case STATUS_PITCH_CHANGE:
                    bigval = (byte << 7) | recvArg0_;
                    handlePitchChange(bigval);
                    break;
                case STATUS_SONG_POSITION:
                    bigval = (byte << 7) | recvArg0_;
                    handleSongPosition(bigval);
                    break;
                case STATUS_SONG_SELECT:
                    handleSongSelect(byte);
                    break;
            }
        }

        /* Just reset the byte count; keep the same event -- might get more messages
            trailing from current event.
         */
        recvByteCount_ = 0;
    }

    recvArg0_ = byte;
}


// Send Midi NOTE OFF message to a given channel, with note 0-127 and velocity 0-127
void sendNoteOff(unsigned int channel, unsigned int note, unsigned int velocity)
{

	channel = channelIn_;   // use the input channel
    int status = STATUS_EVENT_NOTE_OFF | ((channel - 1) & 0x0f);


    if (sendFullCommands_ || (lastStatusSent_ != status)) {
        put_char(status);
    }

    put_char(note & 0x7f);
    put_char(velocity & 0x7f);
}


// Send Midi NOTE ON message to a given channel, with note 0-127 and velocity 0-127
void sendNoteOn(unsigned int channel, unsigned int note, unsigned int velocity)
{
	channel = channelIn_;  // use the input channel
    int status = STATUS_EVENT_NOTE_ON | ((channel - 1) & 0x0f);

    if (sendFullCommands_ || (lastStatusSent_ != status)) {
        put_char(status);
    }

    put_char(note & 0x7f);
    put_char(velocity & 0x7f);
}


// Send a Midi VELOCITY CHANGE message to a given channel, with given note 0-127,
//  and new velocity 0-127
void sendVelocityChange(unsigned int channel, unsigned int note, unsigned int velocity)
{
    int status = STATUS_EVENT_VELOCITY_CHANGE | ((channel - 1) & 0x0f);


    if (sendFullCommands_ || (lastStatusSent_ != status)) {
        put_char(status);
    }

    put_char(note & 0x7f);
    put_char(velocity & 0x7f);
}


// Send a Midi CC message to a given channel, as a given controller 0-127, with given
//  value 0-127
void sendControlChange(unsigned int channel, unsigned int controller, unsigned int value)
{
    int status = STATUS_EVENT_CONTROL_CHANGE | ((channel - 1) & 0x0f);


    if (sendFullCommands_ || (lastStatusSent_ != status)) {
        put_char(status);
    }

    put_char(controller & 0x7f);
    put_char(value & 0x7f);
}


// Send a Midi PROGRAM CHANGE message to given channel, with program ID 0-127
void sendProgramChange(unsigned int channel, unsigned int program)
{
    int status = STATUS_EVENT_PROGRAM_CHANGE | ((channel - 1) & 0x0f);


    if (sendFullCommands_ || (lastStatusSent_ != status)) {
        put_char(status);
    }

    put_char(program & 0x7f);
}


// Send a Midi AFTER TOUCH message to given channel, with velocity 0-127
void sendAfterTouch(unsigned int channel, unsigned int velocity)
{
    int status = STATUS_AFTER_TOUCH | ((channel - 1) & 0x0f);


    if (sendFullCommands_ || (lastStatusSent_ != status)) {
        put_char(status);
    }

    put_char(velocity & 0x7f);
}


// Send a Midi PITCH CHANGE message, with a 14-bit pitch (always for all channels)
void sendPitchChange(unsigned int pitch)
{
    put_char(STATUS_PITCH_CHANGE);
    put_char(pitch & 0x7f);
    put_char((pitch >> 7) & 0x7f);
}


// Send a Midi SONG POSITION message, with a 14-bit position (always for all channels)
void sendSongPosition(unsigned int position)
{
    put_char(STATUS_SONG_POSITION);
    put_char(position & 0x7f);
    put_char((position >> 7) & 0x7f);
}


// Send a Midi SONG SELECT message, with a song ID of 0-127 (always for all channels)
void sendSongSelect(unsigned int song)
{
    put_char(STATUS_SONG_SELECT);
    put_char(song & 0x7f);
}


// Send a Midi TUNE REQUEST message (TUNE REQUEST is always for all channels)
void sendTuneRequest(void)
{
    put_char(STATUS_TUNE_REQUEST);
}


// Send a Midi SYNC message (SYNC is always for all channels)
void sendSync(void)
{
    put_char(STATUS_SYNC);
}


// Send a Midi START message (START is always for all channels)
void sendStart(void)
{
    put_char(STATUS_START);
}


// Send a Midi CONTINUE message (CONTINUE is always for all channels)
void sendContinue(void)
{
    put_char(STATUS_CONTINUE);
}


// Send a Midi STOP message (STOP is always for all channels)
void sendStop(void)
{
    put_char(STATUS_STOP);
}


// Send a Midi ACTIVE SENSE message (ACTIVE SENSE is always for all channels)
void sendActiveSense(void)
{
    put_char(STATUS_ACTIVE_SENSE);
}


// Send a Midi RESET message (RESET is always for all channels)
void sendReset(void)
{
    put_char(STATUS_RESET);
}


void midi_init(uint8_t ch)
{
    /* Not in proprietary stream */
    recvMode_ = 0;
    /* No bytes recevied */
    recvByteCount_ = 0;
    /* Not processing an event */
    recvEvent_ = 0;
    /* No arguments to the event we haven't received */
    recvArg0_ = 0;
    /* Not waiting for bytes to complete a message */
    recvBytesNeeded_ = 0;
    // There was no last event.
    lastStatusSent_ = 0;
    // Don't send the extra bytes; just send deltas
    sendFullCommands_ = 0;

    /* Listening to all channels (set to 0)*/
    channelIn_ = ch;
}

// Set (package-specific) parameters for the Midi instance
void setParam(unsigned int param, unsigned int val)
{
    if (param == PARAM_SEND_FULL_COMMANDS) {
        if (val) {
            sendFullCommands_ = 1;
        } else {
            sendFullCommands_ = 0;
        }
    } else if (param == PARAM_CHANNEL_IN) {
        channelIn_ = val;
    }
}

// Get (package-specific) parameters for the Midi instance
unsigned int getParam(unsigned int param)
{
    if (param == PARAM_SEND_FULL_COMMANDS) {
        return sendFullCommands_;
    } else if (param == PARAM_CHANNEL_IN) {
        return channelIn_;
    }

    return 0;
}

//  MIDI Callbacks
void handleNoteOff(unsigned int channel, unsigned int note, unsigned int velocity) {

}

void handleNoteOn(unsigned int channel, unsigned int note, unsigned int velocity) {
	note_on_flag = 1;
}

void handleSync(void) {

}

void handleStart(void) {

}

void handleStop(void) {

}


void handleVelocityChange(unsigned int channel, unsigned int note, unsigned int velocity) {}

//float32_t midi_cc[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void handleControlChange(unsigned int channel, unsigned int controller, unsigned int value) {
	//midi_cc[(controller - 10) & 0x7] = (float32_t)value / 127.f;

}

void handleProgramChange(unsigned int channel, unsigned int program) {}
void handleAfterTouch(unsigned int channel, unsigned int velocity) {}
void handlePitchChange(unsigned int pitch) {}
void handleSongPosition(unsigned int position) {}
void handleSongSelect(unsigned int song) {}
void handleTuneRequest(void) {}
void handleContinue(void) {}
void handleActiveSense(void) {}
void handleReset(void) {}


