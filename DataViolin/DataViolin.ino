/*
	DataViolin - a project by Jon Rose
	
*/

#include <FlexiTimer2.h>
#include <SPI.h>
#include <SdFat.h>
#include <MD_MIDIFile.h>

#define	USE_MIDI
#ifdef USE_MIDI // set up for direct MIDI serial output
#define	DEBUG(x)
#define	DEBUGX(x)
#else // don't use MIDI to allow printing debug statements
#define	DEBUG(x)	Serial.print(x)
#define	DEBUGX(x)	Serial.print(x, HEX)
#endif

#define		SD_SELECT_PIN  21
bool 		SD_available = false;
SdFat		SD;
MD_MIDIFile SMF;


#define LEDpin	2
#define SELpin0 A0
#define SELpin1 A1
#define SELpin2 A2
#define SELpin3 A3
#define POTpin	A6
#define TRUpin	A8
#define TRLpin	A9

#define T_LEFT	0
#define T_RIGHT	1
#define B_LEFT	0
#define B_RIGHT	1

// IDs of PWM channels controlling the solenoids and motors
#define L_BOW			0
#define L_FRET_1		1
#define L_FRET_2		2
#define L_FRET_3		3
#define L_FRET_4		4
#define L_FRET_5		5
#define L_FRET_6		6
#define L_FRET_7		7
#define L_FRET_8		8
#define L_FRET_9		9
#define L_FRET_10		10
#define L_FRET_11		11
#define L_FRET_12		12
#define L_MOTOR			13
#define R_BOW			14
#define R_FRET_1		15
#define R_FRET_2		16
#define R_FRET_3		17
#define R_FRET_4		18
#define R_FRET_5		19
#define R_FRET_6		20
#define R_FRET_7		21
#define R_FRET_8		22
#define R_FRET_9		23
#define R_FRET_10		24
#define R_FRET_11		25
#define R_FRET_12		26
#define R_MOTOR			27
#define NUM_CHAN			28
#define NUM_CHAN_PER_BOARD	14
#define FRET_NUM	 		12

#define TIME_GRAN			10
#define PWM_RESOLUTION		11
#define MIN_PWM				0
//#define MAX_PWM				2047
#define MAX_PWM				1638	// reduce maximum to account for 15V supply (but reenable 2047 when we introduce overshoot)
#define LINE_SCALE			5		// extra bits to increase resolution when calculating incs. Basically we're going from 11 bit to 16 bit
#define MIN_PWM_LINE		(MIN_PWM << LINE_SCALE)
#define MAX_PWM_LINE		(MAX_PWM << LINE_SCALE)


int	err;
int pgm, tru, trl, pot;
int speed = 100;

int PWMpin [NUM_CHAN];
int PWMteensy [NUM_CHAN];
int PWMboard [NUM_CHAN];
long currentPWM [NUM_CHAN];
long targetPWM [NUM_CHAN];
long incPWM [NUM_CHAN];
long ticksPWM [NUM_CHAN];

// ====================================================================================================================================
// setup ==============================================================================================================================
// ====================================================================================================================================
void setup() {
	int i;
	
	// Serial
	Serial.begin (57600);
	Serial1.begin (57600);
	Serial2.begin (57600);
	Serial3.begin (57600);
	
	analogWriteFrequency (5, 23437);	// also changes frequency for pins 6,9,10,20,21,22,23 on Teensy 3.1 (Timer 0)
	analogWriteFrequency (3, 23437);	// also changes frequency for pin 4 on Teensy 3.1 (Timer 1)
	analogWriteResolution (11);

	// MIDI
#ifdef USE_MIDI
	usbMIDI.setHandleNoteOff(OnNoteOff);
	usbMIDI.setHandleNoteOn(OnNoteOn);
	usbMIDI.setHandleControlChange(OnControlChange);
#endif

	// MIDI file player
	if (!SD.begin (SD_SELECT_PIN, SPI_FULL_SPEED)) {	// first, check for card
    	DEBUG("\nSD init fail!");
		SD_available = false;
	//	while (true) ;	we don't block if there's no card, we just set a flag
	}
	else {
		// Initialize MIDIFile
		SMF.begin (&SD);
		SMF.setMidiHandler (midiFileCallback);
		SMF.setSysexHandler (sysexFileCallback);
		SMF.setFilename ("SEQ.MID");
		err = SMF.load();
		if (err != -1) {	// second, check for file
			DEBUG("\nSMF load Error ");
			DEBUG(err);
			SD_available = false;
		}
		else {
			SD_available = true;
			SMF.looping (true);
		}
	}

	// pin modes
	initPWMassignments ();
	
	pinMode (LEDpin, OUTPUT);
	digitalWrite (LEDpin, HIGH);
	pinMode (SELpin0, INPUT_PULLUP);
	pinMode (SELpin1, INPUT_PULLUP);
	pinMode (SELpin2, INPUT_PULLUP);
	pinMode (SELpin3, INPUT_PULLUP);
	for (i=0; i<NUM_CHAN; i++) {
		if ((PWMteensy[i] == T_LEFT) && (PWMboard[i] == B_LEFT)) {
			pinMode (PWMpin[i], OUTPUT);
		}
	}

	// init
	resetPWM ();
	updatePWM ();
	
	// interrupts
	FlexiTimer2::set (TIME_GRAN, 1.0/1000, tick); // call every 10ms (10x 1ms)
	FlexiTimer2::start();

}



// PWM ==================================================================
void initPWMassignments () {
	// P I N						T E E N S Y								B O A R D
	PWMpin [L_BOW]		=  5;		PWMteensy [L_BOW]	  	= T_LEFT;		PWMboard [L_BOW]		= B_LEFT;
	
	PWMpin [L_FRET_1 ]	=  6;		PWMteensy [L_FRET_1 ]	= T_LEFT;		PWMboard [L_FRET_1 ]	= B_LEFT;
	PWMpin [L_FRET_2 ]	=  3;		PWMteensy [L_FRET_2 ]	= T_LEFT;		PWMboard [L_FRET_2 ]	= B_LEFT;
	PWMpin [L_FRET_3 ]	= 20;		PWMteensy [L_FRET_3 ]	= T_RIGHT;		PWMboard [L_FRET_3 ]	= B_LEFT;
	PWMpin [L_FRET_4 ]	= 21;		PWMteensy [L_FRET_4 ]	= T_RIGHT;		PWMboard [L_FRET_4 ]	= B_LEFT;
	PWMpin [L_FRET_5 ]	= 22;		PWMteensy [L_FRET_5 ]	= T_RIGHT;		PWMboard [L_FRET_5 ]	= B_LEFT;
	PWMpin [L_FRET_6 ]	= 23;		PWMteensy [L_FRET_6 ]	= T_RIGHT;		PWMboard [L_FRET_6 ]	= B_LEFT;
	PWMpin [L_FRET_7 ]	= 10;		PWMteensy [L_FRET_7 ]	= T_RIGHT;		PWMboard [L_FRET_7 ]	= B_LEFT;
	PWMpin [L_FRET_8 ]	=  9;		PWMteensy [L_FRET_8 ]	= T_RIGHT;		PWMboard [L_FRET_8 ]	= B_LEFT;
	PWMpin [L_FRET_9 ]	=  6;		PWMteensy [L_FRET_9 ]	= T_RIGHT;		PWMboard [L_FRET_9 ]	= B_LEFT;
	PWMpin [L_FRET_10]	=  5;		PWMteensy [L_FRET_10]	= T_RIGHT;		PWMboard [L_FRET_10]	= B_LEFT;
	PWMpin [L_FRET_11]	=  4;		PWMteensy [L_FRET_11]	= T_RIGHT;		PWMboard [L_FRET_11]	= B_LEFT;
	PWMpin [L_FRET_12]	=  3;		PWMteensy [L_FRET_12]	= T_RIGHT;		PWMboard [L_FRET_12]	= B_LEFT;
                                                                                                
	PWMpin [L_MOTOR]	=  4;		PWMteensy [L_MOTOR]	 	= T_LEFT;		PWMboard [L_MOTOR]		= B_LEFT;
	                                                                                            
                                                                                                
	PWMpin [R_MOTOR]	=  5;		PWMteensy [R_MOTOR]		= T_RIGHT;		PWMboard [R_MOTOR]		= B_RIGHT;
	                                                                                            
	PWMpin [R_FRET_1 ]	=  6;		PWMteensy [R_FRET_1 ]	= T_RIGHT;		PWMboard [R_FRET_1 ]	= B_RIGHT;
	PWMpin [R_FRET_2 ]	=  3;		PWMteensy [R_FRET_2 ]	= T_RIGHT;		PWMboard [R_FRET_2 ]	= B_RIGHT;
	PWMpin [R_FRET_3 ]	= 20;		PWMteensy [R_FRET_3 ]	= T_LEFT;		PWMboard [R_FRET_3 ]	= B_RIGHT;
	PWMpin [R_FRET_4 ]	= 21;		PWMteensy [R_FRET_4 ]	= T_LEFT;		PWMboard [R_FRET_4 ]	= B_RIGHT;
	PWMpin [R_FRET_5 ]	= 22;		PWMteensy [R_FRET_5 ]	= T_LEFT;		PWMboard [R_FRET_5 ]	= B_RIGHT;
	PWMpin [R_FRET_6 ]	= 23;		PWMteensy [R_FRET_6 ]	= T_LEFT;		PWMboard [R_FRET_6 ]	= B_RIGHT;
	PWMpin [R_FRET_7 ]	= 10;		PWMteensy [R_FRET_7 ]	= T_LEFT;		PWMboard [R_FRET_7 ]	= B_RIGHT;
	PWMpin [R_FRET_8 ]	=  9;		PWMteensy [R_FRET_8 ]	= T_LEFT;		PWMboard [R_FRET_8 ]	= B_RIGHT;
	PWMpin [R_FRET_9 ]	=  6;		PWMteensy [R_FRET_9 ]	= T_LEFT;		PWMboard [R_FRET_9 ]	= B_RIGHT;
	PWMpin [R_FRET_10]	=  5;		PWMteensy [R_FRET_10]	= T_LEFT;		PWMboard [R_FRET_10]	= B_RIGHT;
	PWMpin [R_FRET_11]	=  4;		PWMteensy [R_FRET_11]	= T_LEFT;		PWMboard [R_FRET_11]	= B_RIGHT;
	PWMpin [R_FRET_12]	=  3;		PWMteensy [R_FRET_12]	= T_LEFT;		PWMboard [R_FRET_12]	= B_RIGHT;

	PWMpin [R_BOW]		=  4;		PWMteensy [R_BOW]		= T_RIGHT;		PWMboard [R_BOW]		= B_RIGHT;
}

void resetPWM () {
	int i;
	for (i=0; i<NUM_CHAN; i++) {
		currentPWM [NUM_CHAN] = 0;
		incPWM [NUM_CHAN] = 0;
		ticksPWM [NUM_CHAN] = 0;
	}
}

void updatePWM () {
	int i;
	int iLR;
	int iR;
	
	// first half handled locally
	for (i=0; i<NUM_CHAN; i++) {
		int cp = (currentPWM[i] >> LINE_SCALE);
		int pin = PWMpin[i];
		byte addr = (0xC0 | (pin & 0x1F));
		byte val	= (0x80 | (byte) (cp >> 7));
		byte val2 = (0x00 | (byte) (cp & 0x7F));

		if (PWMboard[i] == B_LEFT) {
			if (PWMteensy[i] == T_LEFT) {
				// local: set PWM
				analogWrite (pin, cp);
			}
			else {
				// remote: send via Serial
				Serial1.write (addr);
				Serial1.write (val);
				Serial1.write (val2);
			}
		}
		else {
			if (PWMteensy[i] == T_LEFT) {
				Serial3.write (addr);
				Serial3.write (val);
				Serial3.write (val2);
			}
			else {
				Serial2.write (addr);
				Serial2.write (val);
				Serial2.write (val2);
			}
		}
	}
}

void linePWM (int i, long target, long attack, long delay) {
//	Serial.println (i);
//	Serial.println (target);
//	Serial.println (attack);
//	Serial.println (delay);

	if (attack <= 0) {
		targetPWM[i] = target;
		currentPWM[i] = target;
		ticksPWM[i] = 0;
		incPWM[i] = 0;
	}
	else {
		targetPWM[i] = target;
		ticksPWM[i] = attack / TIME_GRAN;
		incPWM[i] = (target - currentPWM[i]) / ticksPWM[i];
	}
}

void tickPWM () {
	int i;
	for (i=0; i<NUM_CHAN; i++) {
		if (ticksPWM[i] > 0) {
			currentPWM[i] += incPWM[i];
			ticksPWM[i]--;
		}
	}
	
	updatePWM ();
}
// END PWM ===============================================================
//
// Notes ===================================================================
bool note_playing_left_flag = false;
bool note_playing_right_flag = false;
int last_played_note;

void PlayNote (int ID, int velocity) {
	linePWM (ID, MAX_PWM_LINE, 100, 0);
	last_played_note = ID;
	
//	delay (200);	roll delay into linePWM function, so we don't block
	
	if (PWMboard [ID] == B_LEFT) {
		if (note_playing_left_flag) {
			// if a note is already playing we don't have to push down the bow again (aka legato)
		}
		else {
			linePWM (L_MOTOR, MAX_PWM_LINE, 100, 0);
			linePWM (L_BOW, MAX_PWM_LINE, 100, 0);
		}
		note_playing_left_flag = true;
	}
	else {
		if (note_playing_right_flag) {
		}
		else {
			linePWM (R_MOTOR, MAX_PWM_LINE, 100, 0);
			linePWM (R_BOW, MAX_PWM_LINE, 100, 0);
		}
		note_playing_right_flag = true;
	}	
}

void StopNote (int ID, int velocity) {
	linePWM (ID, MIN_PWM_LINE, 100, 0);
	
	if (last_played_note == ID) {	// simple scheme to only release after the last played note has been released. We might need to buffer all notes and check if any is still being held
	//	delay (200);

		if (PWMboard [ID] == B_LEFT) {
			linePWM (L_MOTOR, MIN_PWM_LINE, 100, 0);
			linePWM (L_BOW, MIN_PWM_LINE, 100, 0);
			note_playing_left_flag = false;
		}
		else {
			linePWM (R_MOTOR, MIN_PWM_LINE, 100, 0);
			linePWM (R_BOW, MIN_PWM_LINE, 100, 0);
			note_playing_right_flag = false;
		}			
	}
}


//
// Sensors ===============================================================
int checkSelector () {
	int s;
	s =		 !digitalRead (SELpin0) + 
			2 * !digitalRead (SELpin1) + 
			4 * !digitalRead (SELpin2) + 
			8 * !digitalRead (SELpin3);
	return s;
}

void checkAnalog () {
	tru = analogRead (TRUpin);
	trl = analogRead (TRLpin);
	pot = analogRead (POTpin);
	
	if (pgm == 2) {
		linePWM (L_MOTOR, map (trl, 0, 1023, 0, MAX_PWM_LINE), 10, 0);
	}
		
}
// END Sensors ============================================================
//
// MIDI ===================================================================
int midiCtrlIn;

void checkMIDI () {
#ifdef USE_MIDI
	usbMIDI.read(); // USB MIDI receive
#endif
}
void OnNoteOn(byte channel, byte note, byte velocity) {
	int ch;

	if ((note >= 60) && (note <= 71)) {
		ch = note - 60 + L_FRET_1;
	}
	else if ((note >= 72) && (note <= 83)) {
		ch = note - 72 + R_FRET_1;
	}
	else return;
 
//	linePWM (ch, MAX_PWM_LINE, 0, 0);
//	linePWM (ch, MAX_PWM_LINE, (127-velocity) * 10,0);
	PlayNote (ch, 0);
}

void OnNoteOff(byte channel, byte note, byte velocity) {
	int ch;
	
	if ((note >= 60) && (note <= 71)) {
		ch = note - 60 + L_FRET_1;
	}
	else if ((note >= 72) && (note <= 83)) {
		ch = note - 72 + R_FRET_1;
	}
	else return;
	
//	linePWM (ch, MIN_PWM_LINE, 0, 0);
	StopNote (ch, 0);
}

void OnControlChange(byte channel, byte control, byte value) {
	midiCtrlIn = value;
}
// END MIDI ===============================================================

// ====================================================================================
// MAIN LOOP - MAIN TICK
// loop is called as fast as possible
// tick is called at regular intervals (1ms)

// ====================================================================================
void tick()
{
//	DEBUG("tick");

	tickPWM ();	
	checkAnalog ();
}

void loop () {

	pgm = checkSelector ();
	pgm = 0;

	switch (pgm) {
	case 0:
		// MAIN: listen to MIDI, play notes
		checkMIDI ();
		break;
	case 1:
		// MIDI file player: play from SD card (maybe more than one song, maybe use second selector on right board to select this)
		playMIDIfile ();
		break;
	case 2:
		// host-assisted adjust & test mode: set PWM tables and other parameters, instrument returns MIDI diagnostics
		break;
	case 3:
		// standalone test mode
		speed = pot + 50;
		break;
	case 4:
		// sequence test: play a simple sequence
		fretWithMotorSequence ();
		fretSequence ();
		break;
	default:
		speed = 50;
		break;
	}
//	blinkLED (speed);
}

// programs ========================================================================
void fretSequence () {
	static int i=1;
	int iOn, iOff;
	
	iOn = i;
	iOff = i-1;
	if (iOff < 1) {
		iOff = FRET_NUM;
	}
		
	linePWM (iOff, MIN_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
	delay (map (pot, 0, 1023, 100, 30));
	linePWM (iOn, MAX_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
	delay (map (pot, 0, 1023, 2000, 500));

	i++;
	if (i>FRET_NUM) {
		i = 1;
	}
}

// ========================================================================
void fretWithMotorSequence () {
	static int i=1;
	int iOn, iOff;
	
	iOn = i;
	iOff = i-1;
	if (iOff < 1) {
		iOff = FRET_NUM;
	}
		
/*	linePWM (iOff, MIN_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
	delay (map (pot, 0, 1023, 100, 30));
	linePWM (iOn, MAX_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
	linePWM (L_BOW, MAX_PWM_LINE, 10, 0);
	delay (map (pot, 0, 1023, 2000, 500));
	linePWM (L_BOW, MIN_PWM_LINE, 10, 0);
*/

	linePWM (iOn, MAX_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
	delay (map (pot, 0, 1023, 100, 30));
	linePWM (iOff, MIN_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
	delay (map (pot, 0, 1023, 600, 130));
	linePWM (L_BOW, MAX_PWM_LINE, 10, 0);

	delay (map (pot, 0, 1023, 1000, 250));
	linePWM (L_BOW, MIN_PWM_LINE, 10, 0);
	delay (map (pot, 0, 1023, 1000, 250));
	


	i++;
	if (i>FRET_NUM) {
		i = 1;
	}
}






// ========================================================================
//	MIDI file player functions
// ========================================================================

void midiFileCallback(midi_event *pev)
// Called by the MIDIFile library when a file event needs to be processed
// thru the midi communications interface.
// This callback is set up in the setup() function.
{
	byte	noteNum, velocity;
	byte	cmd = (pev->data[0] & 0xF0);	// not sure if we really need to blank the channel bits or wether the library does this for us
	
	if (cmd == 0x90) {
		noteNum		= pev->data[1];
		velocity	= pev->data[2];
		
		if (velocity != 0) {
			digitalWrite (LEDpin, HIGH);
		//	linePWM (noteNum - 0x30 + 1, MAX_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
			linePWM (noteNum - 0x30 + L_FRET_1, MAX_PWM_LINE, 0, 0);
		}
		else {
			digitalWrite (LEDpin, LOW);			
		//	linePWM (noteNum - 0x30 + 1, MIN_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
			linePWM (noteNum - 0x30 + L_FRET_1, MIN_PWM_LINE, 0, 0);
		}
	}
	
/*#if USE_MIDI
	if ((pev->data[0] >= 0x80) && (pev->data[0] <= 0xe0))
	{
		Serial.write(pev->data[0] | pev->channel);
		Serial.write(&pev->data[1], pev->size-1);
	}
	else
		Serial.write(pev->data, pev->size);
#endif*/
  DEBUG("\nM T");
  DEBUG(pev->track);
  DEBUG(":  Ch ");
  DEBUG(pev->channel+1);
  DEBUG(" Data ");
  for (uint8_t i=0; i<pev->size; i++)
  {
	DEBUGX(pev->data[i]);
    DEBUG(' ');
  }
}

void sysexFileCallback(sysex_event *pev)
// Called by the MIDIFile library when a system Exclusive (sysex) file event needs 
// to be processed through the midi communications interface. Most sysex events cannot 
// really be processed, so we just ignore it here.
// This callback is set up in the setup() function.
{
  DEBUG("\nS T");
  DEBUG(pev->track);
  DEBUG(": Data ");
  for (uint8_t i=0; i<pev->size; i++)
  {
    DEBUGX(pev->data[i]);
	DEBUG(' ');
  }
}

void midiSilence(void)
// Turn everything off on every channel.
// Some midi files are badly behaved and leave notes hanging, so between songs turn
// off all the notes and sound
{
	midi_event	ev;

	// All sound off
	// When All Sound Off is received all oscillators will turn off, and their volume
	// envelopes are set to zero as soon as possible.
	ev.size = 0;
	ev.data[ev.size++] = 0xb0;
	ev.data[ev.size++] = 120;
	ev.data[ev.size++] = 0;

	for (ev.channel = 0; ev.channel < 16; ev.channel++)
		midiFileCallback(&ev);
}

void tickMetronome(void)
// flash a LED to the beat
{
	static uint32_t	lastBeatTime = 0;
	static boolean	inBeat = false;
	uint16_t	beatTime;

	beatTime = 60000/SMF.getTempo();		// msec/beat = ((60sec/min)*(1000 ms/sec))/(beats/min)
	if (!inBeat)
	{
		if ((millis() - lastBeatTime) >= beatTime)
		{
			lastBeatTime = millis();
			inBeat = true;
		}
	}
	else
	{
		if ((millis() - lastBeatTime) >= 100)	// keep the flash on for 100ms only
		{
			inBeat = false;
		}
	}

}

void playMIDIfile (void)
{
	if (!SMF.isEOF()) {
		SMF.getNextEvent();
	}
	
/*	// play the file
	if (!SMF.isEOF())
	{
		if (SMF.getNextEvent())
		tickMetronome();
	}

	// done with this one
//	SMF.close();
//	midiSilence();
*/
}


