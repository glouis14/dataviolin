/*
	DataViolin - a project by Jon Rose
	
	to do:
	- more hierarchical structures for all the parameters
	- combine ADSR concept with planned overshoot functionality
*/

#include <FlexiTimer2.h>
#include <SPI.h>
#include <SdFat.h>
#include <MD_MIDIFile.h>
#include <EEPROM.h>

#define DEBUG_ENABLE
#define	USE_MIDI

#ifdef DEBUG_ENABLE
#ifdef USE_MIDI // set up for direct MIDI serial output
#define	DEBUG(x)		SX_STRING(x)
#define DEBUGN(x)		SX_NUM(x)
#define DEBUGN_LN(x)	SX_NUM_LN(x)
#define	DEBUGX(x)
#else // don't use MIDI to allow printing debug statements
#define	DEBUG(x)		Serial.print(x)
#define	DEBUGN(x)		Serial.print(x)
#define	DEBUGN_LN(x)	Serial.println(x)
#define	DEBUGX(x)		Serial.print(x, HEX)
#endif
#else
#define	DEBUG(x)
#define	DEBUGX(x)
#define	DEBUGN(x)
#define	DEBUGN_LN(x)
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
#define L_OPEN			0
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
#define R_OPEN			14
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
#define OVER_PWM			2047
#define LINE_SCALE			5		// extra bits to increase resolution when calculating incs. Basically we're going from 11 bit to 16 bit
#define MIN_PWM_LINE		(MIN_PWM << LINE_SCALE)
#define MAX_PWM_LINE		(MAX_PWM << LINE_SCALE)
#define OVER_PWM_LINE		(OVER_PWM << LINE_SCALE)

typedef struct {
	long	c;		// current
	long	min;
	long	max;
} t_valRange;
int randomRangeStruct (t_valRange *v);
int initStruct (t_valRange *v, long vi);
	
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
long delayPWM [NUM_CHAN];
bool fretDownFlags [NUM_CHAN];
int fretBowCompensation [NUM_CHAN];

int latest_note_ID;

bool ignoreChannelFlag = true;
bool localRandomization = true;

// ====================================================================================================================================
// setup ==============================================================================================================================
// ====================================================================================================================================
void setup() {
	int i;

	// need this one earlier for debugging
	pinMode (LEDpin, OUTPUT);
//	digitalWrite (LEDpin, HIGH);
	
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
	usbMIDI.setHandleProgramChange(OnProgramChange);
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
			DEBUGN(err);
			SD_available = false;
		}
		else {
			SD_available = true;
			SMF.looping (true);
		}
	}

	// pin modes
	initPWMassignments ();
	
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
	
	initArticulation ();
	readParamFromEEPROM ();
	
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
	                                                                                            
                                                                                                
	PWMpin [R_MOTOR]	=  4;		PWMteensy [R_MOTOR]		= T_RIGHT;		PWMboard [R_MOTOR]		= B_RIGHT;
	                                                                                            
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

	PWMpin [R_BOW]		=  5;		PWMteensy [R_BOW]		= T_RIGHT;		PWMboard [R_BOW]		= B_RIGHT;
}

void resetPWM () {
	int i;
	for (i=0; i<NUM_CHAN; i++) {
		currentPWM [NUM_CHAN] = 0;
		incPWM [NUM_CHAN] = 0;
		ticksPWM [NUM_CHAN] = 0;
		delayPWM [NUM_CHAN] = 0;
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

void linePWM (int i, long target, long attack, long delayed) {
//	Serial.println (i);
//	Serial.println (target);
//	Serial.println (attack);
//	Serial.println (delayed);

	delayPWM[i] = delayed / TIME_GRAN;
	targetPWM[i] = target;
	ticksPWM[i] = attack / TIME_GRAN;
	if (ticksPWM[i] == 0) {
		incPWM[i] = 0;
		// we can't set the current value to the target value immediately (like before we introduced the delay). 
		// instead in the tick() routine when delayPWM has reached zero and ticksPWM is zero, we set the current to the target
	}
	else {
		incPWM[i] = (target - currentPWM[i]) / ticksPWM[i];		
	}
}

void tickPWM () {
	int i;
	for (i=0; i<NUM_CHAN; i++) {
		if (delayPWM[i] > 0) {
			delayPWM[i]--;
		}
		else {
			if (ticksPWM[i] > 0) {
				currentPWM[i] += incPWM[i];
				ticksPWM[i]--;
			}
			else {
				currentPWM[i] = targetPWM[i];
			}			
		}
	}
	
	updatePWM ();
}
// END PWM ===============================================================
//
// Notes ===================================================================
t_valRange	noteOnFretDelay;
t_valRange	noteOnBowDelay;
t_valRange	noteOnMotorDelay;
t_valRange	noteOffFretDelay;
t_valRange	noteOffBowDelay;
t_valRange	noteOffMotorDelay;

t_valRange	motorSpeed_L;
t_valRange	motorSpeed_R;
t_valRange	bowPressure_L;
t_valRange	bowPressure_R;
t_valRange	fretPressure;

t_valRange	noteOnFretAttack;
t_valRange	noteOnBowAttack;
t_valRange	noteOnMotorAttack;
t_valRange	noteOffFretRelease;
t_valRange	noteOffBowRelease;
t_valRange	noteOffMotorRelease;

t_valRange	noteOnFretDecay;
t_valRange	noteOnBowDecay;
t_valRange	noteOnMotorDecay;
t_valRange	noteOnFretSustain;
t_valRange	noteOnBowSustain;
t_valRange	noteOnMotorSustain;



bool note_playing_flag_L = false;
bool note_playing_flag_R = false;
int last_played_note_L;
int last_played_note_R;

void PlayNote (int ID, int velocity) {
	long peak, sustain;
	
	// bring the fret down
	if ((noteOnFretSustain.c == 127) || (noteOnFretDecay.c == 0)) {		// no sustain means only one line segment
		sustain = (MAX_PWM_LINE * fretPressure.c) / 127;

		if ((ID != L_OPEN) && (ID != R_OPEN)) {
			linePWM (ID, sustain, noteOnFretAttack.c, noteOnFretDelay.c);
		}		
	}
	else {
		// sustain is interpreted a bit differently:
		//		the delta to 127 is used as an overshoot for the initial attack, while sustain sets the main volume
		//		i.e. we don't lower the sustain phase but raise the attack peak
		long overshoot = 127 - noteOnFretSustain.c;
		
		sustain = (MAX_PWM_LINE *  fretPressure.c             ) / 127;
		peak    = (MAX_PWM_LINE * (fretPressure.c + overshoot)) / 127;
		
		if (peak > OVER_PWM_LINE)	peak = OVER_PWM_LINE;
		
		if ((ID != L_OPEN) && (ID != R_OPEN)) {
			linePWM (ID, peak, 		noteOnFretAttack.c, 	noteOnFretDelay.c);
			linePWM (ID, sustain, 	noteOnFretDecay.c, 		noteOnFretAttack.c + noteOnFretDelay.c);
		}		
	}
		
	fretDownFlags [ID] = true;

	// start the engine and engage
	if (PWMboard [ID] == B_LEFT) {
		last_played_note_L = ID;
		
		if (note_playing_flag_L) {
			// if a note is already playing we don't have to push down the bow again (aka legato)
		}
		else {
		//	linePWM (L_BOW, (MAX_PWM_LINE * bowPressure_L.c) / 127, noteOnBowAttack.c, noteOnBowDelay.c);
			linePWM (L_BOW, (MAX_PWM_LINE * bowPressure_L.c * fretBowCompensation [ID]) / 8001, noteOnBowAttack.c, noteOnBowDelay.c);
			//               16 bit         7 bit             7 bit = 30 bit              127*63 (13 bit)
		
			linePWM (L_MOTOR, (MAX_PWM_LINE * motorSpeed_L.c) / 127, noteOnMotorAttack.c, noteOnMotorDelay.c);
		}
		note_playing_flag_L = true;
	}
	else {
		last_played_note_R = ID;

		if (note_playing_flag_R) {
		}
		else {
		//	linePWM (R_BOW, (MAX_PWM_LINE * bowPressure_R.c) / 127, noteOnBowAttack.c, noteOnBowDelay.c);
			linePWM (R_BOW, (MAX_PWM_LINE * bowPressure_R.c * fretBowCompensation [ID]) / 8001, noteOnBowAttack.c, noteOnBowDelay.c);

			linePWM (R_MOTOR, (MAX_PWM_LINE * motorSpeed_R.c) / 127, noteOnMotorAttack.c, noteOnMotorDelay.c);
		}
		note_playing_flag_R = true;
	}	
}

void StopNote (int ID, int velocity) {
	int i, num;
	
	if ((ID != L_OPEN) && (ID != R_OPEN)) {
		linePWM (ID, MIN_PWM_LINE, noteOffFretRelease.c, noteOffFretDelay.c);
	}
	
	fretDownFlags [ID] = false;
	
	num = 0;
	for (i=L_FRET_1; i<=L_FRET_12; i++) {
		if (fretDownFlags [i]) {
			num++;
		}
	}

//	if (last_played_note_L == ID) {	// simple scheme to only release after the last played note has been released. We might need to buffer all notes and check if any is still being held
		// we should keep an array of notes held down and only release the box after all frets have been released
	if (num == 0) {
	//	if (PWMboard [ID] == B_LEFT) {
			linePWM (L_BOW, MIN_PWM_LINE, noteOffBowRelease.c, noteOffBowDelay.c);
			linePWM (L_MOTOR, MIN_PWM_LINE, noteOffMotorRelease.c, noteOffMotorDelay.c);
			note_playing_flag_L = false;
	//	}
	}
	

	// same procedure for right string
	num = 0;
	for (i=R_FRET_1; i<=R_FRET_12; i++) {
		if (fretDownFlags [i]) {
			num++;
		}
	}

//	if (last_played_note_R == ID) {
	if (num == 0) {
	//	if (PWMboard [ID] == B_RIGHT) {
			linePWM (R_BOW, MIN_PWM_LINE, noteOffBowRelease.c, noteOffBowDelay.c);
			linePWM (R_MOTOR, MIN_PWM_LINE, noteOffMotorRelease.c, noteOffMotorDelay.c);
			note_playing_flag_R = false;
	//	}			
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
#define MIDI_NOTE_OPEN_STRING_L	60
#define MIDI_NOTE_OPEN_STRING_R	84

void checkMIDI () {
	int ret;
#ifdef USE_MIDI
	if (usbMIDI.read()) { // USB MIDI receive
	//	digitalWrite (LEDpin, HIGH);
	//	digitalWrite (LEDpin, LOW);	
		ret = usbMIDI.getType();
		if (ret == 7) {
			OnSysex ();
		}
	}
#endif
}
void OnNoteOn (byte channel, byte note, byte velocity) {
	int ch;

	if ((channel == 1) || (ignoreChannelFlag)) {
		if ((note >= MIDI_NOTE_OPEN_STRING_L) && (note <= MIDI_NOTE_OPEN_STRING_L + 12)) {
			ch = note - MIDI_NOTE_OPEN_STRING_L + L_OPEN;
		}		
	}
	if ((channel == 2) || (ignoreChannelFlag)) {
		if ((note >= MIDI_NOTE_OPEN_STRING_R) && (note <= MIDI_NOTE_OPEN_STRING_R + 12)) {
			ch = note - MIDI_NOTE_OPEN_STRING_R + R_OPEN;
		}		
	}

	if (localRandomization) {
		RandomizeArticulationOn ();
	}
	PlayNote (ch, 0);
	latest_note_ID = ch;
}

void OnNoteOff (byte channel, byte note, byte velocity) {
	int ch;
	
	if ((channel == 1) || (ignoreChannelFlag)) {
		if ((note >= MIDI_NOTE_OPEN_STRING_L) && (note <= MIDI_NOTE_OPEN_STRING_L + 12)) {
			ch = note - MIDI_NOTE_OPEN_STRING_L + L_OPEN;
		}
	}
	if ((channel == 2) || (ignoreChannelFlag)) {
		if ((note >= MIDI_NOTE_OPEN_STRING_R) && (note <= MIDI_NOTE_OPEN_STRING_R + 12)) {
			ch = note - MIDI_NOTE_OPEN_STRING_R + R_OPEN;
		}		
	}
	
	if (localRandomization) {
		RandomizeArticulationOff ();
	}
	StopNote (ch, 0);
}

void OnControlChange (byte channel, byte control, byte value) {
	// calling all 16 (currently) control changes separately takes 170µs to process
//	digitalWrite (LEDpin, HIGH);
	
	switch (control) {
	case 0: 	noteOnFretDelay.c = value * 10;			break;
	case 1:		noteOnBowDelay.c = value * 10;			break;
	case 2:		noteOnMotorDelay.c = value * 10;		break;
	case 3:		noteOffFretDelay.c = value * 10;		break;
	case 4:		noteOffBowDelay.c = value * 10;			break;
	case 5:		noteOffMotorDelay.c = value * 10;		break;
		
	case 10:	motorSpeed_L.c = value;		linePWM (L_MOTOR, (MAX_PWM_LINE * motorSpeed_L.c) / 127, 0, 0);			break;	// don't just adjust the parameter, apply immediately
	case 11:	motorSpeed_R.c = value;		linePWM (R_MOTOR, (MAX_PWM_LINE * motorSpeed_R.c) / 127, 0, 0);			break;
	case 12:	bowPressure_L.c = value;	linePWM (L_BOW, (MAX_PWM_LINE * bowPressure_L.c) / 127, 0, 0);			break;
	case 13:	bowPressure_R.c = value;	linePWM (R_BOW, (MAX_PWM_LINE * bowPressure_R.c) / 127, 0, 0);			break;
	case 14:	fretPressure.c = value;		linePWM (latest_note_ID, (MAX_PWM_LINE * fretPressure.c) / 127, 0, 0);	break;

	case 20:	noteOnFretAttack.c = value * 10;		break;
	case 21:	noteOnBowAttack.c = value * 10;			break;
	case 22:	noteOnMotorAttack.c = value * 10;		break;
	case 23:	noteOffFretRelease.c = value * 10;		break;
	case 24:	noteOffBowRelease.c = value * 10;		break;
	case 25:	noteOffMotorRelease.c = value * 10;		break;
		
	// min/max for the above
	case 40:	noteOnFretDelay.min = value * 10;		break;
	case 41:	noteOnBowDelay.min = value * 10;		break;
	case 42:	noteOnMotorDelay.min = value * 10;		break;
	case 43:	noteOffFretDelay.min = value * 10;		break;
	case 44:	noteOffBowDelay.min = value * 10;		break;
	case 45:	noteOffMotorDelay.min = value * 10;		break;
	case 50:	noteOnFretDelay.max = value * 10;		break;
	case 51:	noteOnBowDelay.max = value * 10;		break;
	case 52:	noteOnMotorDelay.max = value * 10;		break;
	case 53:	noteOffFretDelay.max = value * 10;		break;
	case 54:	noteOffBowDelay.max = value * 10;		break;
	case 55:	noteOffMotorDelay.max = value * 10;		break;
		
	case 60:	motorSpeed_L.min = value;				break;
	case 61:	motorSpeed_R.min = value;				break;
	case 62:	bowPressure_L.min = value;				break;
	case 63:	bowPressure_R.min = value;				break;
	case 64:	fretPressure.min = value;				break;
	case 70:	motorSpeed_L.max = value;				break;
	case 71:	motorSpeed_R.max = value;				break;
	case 72:	bowPressure_L.max = value;				break;
	case 73:	bowPressure_R.max = value;				break;
	case 74:	fretPressure.max = value;				break;

	case 80:	noteOnFretAttack.min = value * 10;		break;
	case 81:	noteOnBowAttack.min = value * 10;		break;
	case 82:	noteOnMotorAttack.min = value * 10;		break;
	case 83:	noteOffFretRelease.min = value * 10;	break;
	case 84:	noteOffBowRelease.min = value * 10;		break;
	case 85:	noteOffMotorRelease.min = value * 10;	break;
	case 90:	noteOnFretAttack.max = value * 10;		break;
	case 91:	noteOnBowAttack.max = value * 10;		break;
	case 92:	noteOnMotorAttack.max = value * 10;		break;
	case 93:	noteOffFretRelease.max = value * 10;	break;
	case 94:	noteOffBowRelease.max = value * 10;		break;
	case 95:	noteOffMotorRelease.max = value * 10;	break;
		

	case 127:	localRandomization = value;		break;
	}
	
//	digitalWrite (LEDpin, LOW);	
}

void OnProgramChange (byte channel, byte program) {
	if ((channel == 7) && (program == 73)) {
	//	digitalWrite (LEDpin, HIGH);
		writeParamToEEPROM ();
	}
	if ((channel == 11) && (program == 94)) {
	//	digitalWrite (LEDpin, LOW);
		readParamFromEEPROM ();
	}
}

void OnSysex () {
	int i, num;
	byte *sxBuf;
	byte channel, note, velocity;
	int mode;
	
#ifdef USE_MIDI	
	num = usbMIDI.getData1();
	if (num != 22) return;
	
	sxBuf = usbMIDI.getSysExArray();
	if (sxBuf [0] != 240) return;
#else
	return;
#endif

	note 		= sxBuf [1];
	velocity	= sxBuf [2];
	channel		= sxBuf [3];
	mode		= sxBuf [4];
	
	if (mode == 1) {
		if (velocity != 0) {	// Note On
			noteOnFretDelay.c		= sxBuf [5] * 10;
			noteOnBowDelay.c 		= sxBuf [6] * 10;
			noteOnMotorDelay.c		= sxBuf [7] * 10;
			
			noteOnFretAttack.c		= sxBuf [11] * 10;
			noteOnBowAttack.c		= sxBuf [12] * 10;
			noteOnMotorAttack.c		= sxBuf [13] * 10;
			
			fretPressure.c			= sxBuf [19];
			
			if ((note >= MIDI_NOTE_OPEN_STRING_L) && (note <= MIDI_NOTE_OPEN_STRING_L + 12)) {
				motorSpeed_L.c		= sxBuf [17];
				bowPressure_L.c		= sxBuf [18];
			}
			else if (channel == 2) {
				motorSpeed_R.c		= sxBuf [17];
				bowPressure_R.c		= sxBuf [18];
			}

			OnNoteOn (channel, note, velocity);
		}
		else {	// Note Off
			noteOffFretDelay.c		= sxBuf [8] * 10;
			noteOffBowDelay.c 		= sxBuf [9] * 10;
			noteOffMotorDelay.c		= sxBuf [10] * 10;
			
			noteOffFretRelease.c	= sxBuf [14] * 10;
			noteOffBowRelease.c		= sxBuf [15] * 10;
			noteOffMotorRelease.c	= sxBuf [16] * 10;

			OnNoteOff (channel, note, velocity);
		}
	}	
}
// END MIDI ===============================================================

void initArticulation () {
	int i;
	
	initStruct (&noteOnFretDelay, 0);
	initStruct (&noteOnBowDelay, 0);
	initStruct (&noteOnMotorDelay, 0);
	initStruct (&noteOffFretDelay, 0);
	initStruct (&noteOffBowDelay, 0);
	initStruct (&noteOffMotorDelay, 0);

	initStruct (&motorSpeed_L, 127);
	initStruct (&motorSpeed_R, 127);
	initStruct (&bowPressure_L, 127);
	initStruct (&bowPressure_R, 127);
	initStruct (&fretPressure, 127);

	initStruct (&noteOnFretAttack, 0);
	initStruct (&noteOnBowAttack, 0);
	initStruct (&noteOnMotorAttack, 0);
	initStruct (&noteOffFretRelease, 0);
	initStruct (&noteOffBowRelease, 0);
	initStruct (&noteOffMotorRelease, 0);
	
	// could set all to 127, but this better illustrates which values matter
	for (i=L_FRET_1; i<=L_FRET_12; i++) {
		fretBowCompensation [i] = 63;	// not 127 - we want to be able to increase!
	}
	for (i=R_FRET_1; i<=R_FRET_12; i++) {
		fretBowCompensation [i] = 63;
	}
}

void RandomizeArticulationOn () {
	randomRangeStruct (&noteOnFretDelay);
//	noteOnFretDelay.c = random (noteOnFretDelay.min, noteOnFretDelay.max + 1);
//	noteOnFretDelay.c = random (500, 1000);
	randomRangeStruct (&noteOnBowDelay);
	randomRangeStruct (&noteOnMotorDelay);

	randomRangeStruct (&motorSpeed_L);
	randomRangeStruct (&motorSpeed_R);
	randomRangeStruct (&bowPressure_L);
	randomRangeStruct (&bowPressure_R);
	randomRangeStruct (&fretPressure);

	randomRangeStruct (&noteOnFretAttack);
	randomRangeStruct (&noteOnBowAttack);
	randomRangeStruct (&noteOnMotorAttack);
}

void RandomizeArticulationOff () {
	randomRangeStruct (&noteOffFretDelay);
	randomRangeStruct (&noteOffBowDelay);
	randomRangeStruct (&noteOffMotorDelay);

	randomRangeStruct (&noteOffFretRelease);
	randomRangeStruct (&noteOffBowRelease);
	randomRangeStruct (&noteOffMotorRelease);	
}

int randomRangeStruct (t_valRange *v) {
	v->c = random (v->min, v->max+1);	// make maximum inclusive
}

int initStruct (t_valRange *v, long vi) {
	v->c = vi;
	v->min = vi;
	v->max = vi;
}


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
void loopt () {		// test loop
	SX_STRING ("hello ");
	SX_NUM_LN (404);
	delay (1000);
}
void loop () {
//	loopt ();	return;

	pgm = checkSelector ();
//	pgm = 0;

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
			OnNoteOn (1, noteNum, velocity);
		//	linePWM (noteNum - 0x30 + 1, MAX_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
		//	linePWM (noteNum - 0x30 + L_FRET_1, MAX_PWM_LINE, 0, 0);
		}
		else {
			digitalWrite (LEDpin, LOW);			
			OnNoteOff (1, noteNum, velocity);
		//	linePWM (noteNum - 0x30 + 1, MIN_PWM_LINE, map (pot, 0, 1023, 500, 100), 0);
		//	linePWM (noteNum - 0x30 + L_FRET_1, MIN_PWM_LINE, 0, 0);
		}
	}
	else if (cmd == 0x80) {		// e.g. Finale creates "proper" NoteOff commands instead of NoteOns with a velocity of 0
		noteNum		= pev->data[1];
		velocity	= pev->data[2];

		digitalWrite (LEDpin, LOW);			
		OnNoteOff (1, noteNum, velocity);
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
/*  DEBUG("\nM T");
  DEBUG(pev->track);
  DEBUG(":  Ch ");
  DEBUG(pev->channel+1);
  DEBUG(" Data ");
  for (uint8_t i=0; i<pev->size; i++)
  {
	DEBUGX(pev->data[i]);
    DEBUG(' ');
  }
*/
}

void sysexFileCallback(sysex_event *pev)
// Called by the MIDIFile library when a system Exclusive (sysex) file event needs 
// to be processed through the midi communications interface. Most sysex events cannot 
// really be processed, so we just ignore it here.
// This callback is set up in the setup() function.
{
/*  DEBUG("\nS T");
  DEBUG(pev->track);
  DEBUG(": Data ");
  for (uint8_t i=0; i<pev->size; i++)
  {
    DEBUGX(pev->data[i]);
	DEBUG(' ');
  }
*/
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



// ========================================================================
//	EEPROM
// ========================================================================

void writeParamToEEPROM () {
	unsigned char c;
	
//	digitalWrite (LEDpin, HIGH);
//	delay (1000);
//	digitalWrite (LEDpin, LOW);
	
	EEPROM.write (0, 1);
	EEPROM.write (1, 2);
	EEPROM.write (2, 3);
	EEPROM.write (3, 4);
	
	EEPROM.write ( 4, noteOnFretDelay.min >> 8);		EEPROM.write ( 5, noteOnFretDelay.min & 0xFF);
	EEPROM.write ( 6, noteOnBowDelay.min >> 8);			EEPROM.write ( 7, noteOnBowDelay.min & 0xFF);
	EEPROM.write ( 8, noteOnMotorDelay.min >> 8);		EEPROM.write ( 9, noteOnMotorDelay.min & 0xFF);
	EEPROM.write (10, noteOffFretDelay.min >> 8);		EEPROM.write (11, noteOffFretDelay.min & 0xFF);
	EEPROM.write (12, noteOffBowDelay.min >> 8);		EEPROM.write (13, noteOffBowDelay.min & 0xFF);
	EEPROM.write (14, noteOffMotorDelay.min >> 8);		EEPROM.write (15, noteOffMotorDelay.min & 0xFF);
	
	EEPROM.write (16, noteOnFretDelay.max >> 8);		EEPROM.write (17, noteOnFretDelay.max & 0xFF);  
	EEPROM.write (18, noteOnBowDelay.max >> 8);			EEPROM.write (19, noteOnBowDelay.max & 0xFF);   
	EEPROM.write (20, noteOnMotorDelay.max >> 8);		EEPROM.write (21, noteOnMotorDelay.max & 0xFF); 
	EEPROM.write (22, noteOffFretDelay.max >> 8);		EEPROM.write (23, noteOffFretDelay.max & 0xFF); 
	EEPROM.write (24, noteOffBowDelay.max >> 8);		EEPROM.write (25, noteOffBowDelay.max & 0xFF);  
	EEPROM.write (26, noteOffMotorDelay.max >> 8);		EEPROM.write (27, noteOffMotorDelay.max & 0xFF);
	
	
	EEPROM.write (28, motorSpeed_L.min >> 8);			EEPROM.write (29, motorSpeed_L.min & 0xFF);
	EEPROM.write (30, motorSpeed_R.min >> 8);			EEPROM.write (31, motorSpeed_R.min & 0xFF);
	EEPROM.write (32, bowPressure_L.min >> 8);			EEPROM.write (33, bowPressure_L.min & 0xFF);
	EEPROM.write (34, bowPressure_R.min >> 8);			EEPROM.write (35, bowPressure_R.min & 0xFF);
	EEPROM.write (36, fretPressure.min >> 8);			EEPROM.write (37, fretPressure.min & 0xFF);

	EEPROM.write (38, motorSpeed_L.max >> 8);			EEPROM.write (39, motorSpeed_L.max & 0xFF);
	EEPROM.write (40, motorSpeed_R.max >> 8);			EEPROM.write (41, motorSpeed_R.max & 0xFF);
	EEPROM.write (42, bowPressure_L.max >> 8);			EEPROM.write (43, bowPressure_L.max & 0xFF);
	EEPROM.write (44, bowPressure_R.max >> 8);			EEPROM.write (45, bowPressure_R.max & 0xFF);
	EEPROM.write (46, fretPressure.max >> 8);			EEPROM.write (47, fretPressure.max & 0xFF);


	EEPROM.write (48, noteOnFretAttack.min >> 8);		EEPROM.write (49, noteOnFretAttack.min & 0xFF);
	EEPROM.write (50, noteOnBowAttack.min >> 8);		EEPROM.write (51, noteOnBowAttack.min & 0xFF);
	EEPROM.write (52, noteOnMotorAttack.min >> 8);		EEPROM.write (53, noteOnMotorAttack.min & 0xFF);
	EEPROM.write (54, noteOffFretRelease.min >> 8);		EEPROM.write (55, noteOffFretRelease.min & 0xFF);
	EEPROM.write (56, noteOffBowRelease.min >> 8);		EEPROM.write (57, noteOffBowRelease.min & 0xFF);
	EEPROM.write (58, noteOffMotorRelease.min >> 8);	EEPROM.write (59, noteOffMotorRelease.min & 0xFF);
	
	EEPROM.write (60, noteOnFretAttack.max >> 8);		EEPROM.write (61, noteOnFretAttack.max & 0xFF);
	EEPROM.write (62, noteOnBowAttack.max >> 8);		EEPROM.write (63, noteOnBowAttack.max & 0xFF);
	EEPROM.write (64, noteOnMotorAttack.max >> 8);		EEPROM.write (65, noteOnMotorAttack.max & 0xFF);
	EEPROM.write (66, noteOffFretRelease.max >> 8);		EEPROM.write (67, noteOffFretRelease.max & 0xFF);
	EEPROM.write (68, noteOffBowRelease.max >> 8);		EEPROM.write (69, noteOffBowRelease.max & 0xFF);
	EEPROM.write (70, noteOffMotorRelease.max >> 8);	EEPROM.write (71, noteOffMotorRelease.max & 0xFF);
}


long mergeEEPROMbytes (int i) {
	return ((((long) EEPROM.read (i)) << 8) + (long) EEPROM.read (i+1));
}

void readParamFromEEPROM () {
	if (EEPROM.read (0) != 1)	return;
	if (EEPROM.read (1) != 2)	return;
	if (EEPROM.read (2) != 3)	return;
	if (EEPROM.read (3) != 4)	return;
	
	noteOnFretDelay.min 		= mergeEEPROMbytes (4);
	noteOnBowDelay.min 			= mergeEEPROMbytes (6);
	noteOnMotorDelay.min 		= mergeEEPROMbytes (8);
	noteOffFretDelay.min 		= mergeEEPROMbytes (10);
	noteOffBowDelay.min 		= mergeEEPROMbytes (12);
	noteOffMotorDelay.min 		= mergeEEPROMbytes (14);
 
	noteOnFretDelay.max 		= mergeEEPROMbytes (16);
	noteOnBowDelay.max 			= mergeEEPROMbytes (18);
	noteOnMotorDelay.max 		= mergeEEPROMbytes (20);
	noteOffFretDelay.max 		= mergeEEPROMbytes (22);
	noteOffBowDelay.max 		= mergeEEPROMbytes (24);
	noteOffMotorDelay.max 		= mergeEEPROMbytes (26);


	motorSpeed_L.min			= mergeEEPROMbytes (28);
	motorSpeed_R.min			= mergeEEPROMbytes (30);
	bowPressure_L.min			= mergeEEPROMbytes (32);
	bowPressure_R.min			= mergeEEPROMbytes (34);
	fretPressure.min			= mergeEEPROMbytes (36);

	motorSpeed_L.max			= mergeEEPROMbytes (38);
	motorSpeed_R.max			= mergeEEPROMbytes (40);
	bowPressure_L.max			= mergeEEPROMbytes (42);
	bowPressure_R.max			= mergeEEPROMbytes (44);
	fretPressure.max			= mergeEEPROMbytes (46);
	

	noteOnFretAttack.min		= mergeEEPROMbytes (48);
	noteOnBowAttack.min			= mergeEEPROMbytes (50);
	noteOnMotorAttack.min		= mergeEEPROMbytes (52);
	noteOffFretRelease.min		= mergeEEPROMbytes (54);
	noteOffBowRelease.min		= mergeEEPROMbytes (56);
	noteOffMotorRelease.min		= mergeEEPROMbytes (58);

	noteOnFretAttack.max		= mergeEEPROMbytes (60);
	noteOnBowAttack.max			= mergeEEPROMbytes (62);
	noteOnMotorAttack.max		= mergeEEPROMbytes (64);
	noteOffFretRelease.max		= mergeEEPROMbytes (66);
	noteOffBowRelease.max		= mergeEEPROMbytes (68);
	noteOffMotorRelease.max		= mergeEEPROMbytes (70);
	
	
	for (i=0; i<NUM_CHAN; i++) {
		fretBowCompensation [i] = mergeEEPROMbytes (72 + i*2);		
	}	
}



// ========================================================================
//	DEBUG
// ========================================================================
#ifdef DEBUG_ENABLE
#ifdef USE_MIDI
char num_buf [5];
byte sx_buf [256];
#endif
#endif


void SX_STRING (char *s) {
	int i, l;

	l = strlen (s);
		
	sx_buf [0] = 240;
	for (i=0; i<l; i++) {
		sx_buf[i+1] = s[i];
	}
	sx_buf [i+1] = 247;
	usbMIDI.sendSysEx (l+2, sx_buf);
}

void SX_NUM (long n) {
	sprintf (num_buf, "%ld", n);
	num_buf [4] = 0;	// make sure string is 0-terminated
	SX_STRING (num_buf);
}


void SX_NUM_LN (long n) {
	sprintf (num_buf, "%ld\n", n);
	num_buf [5] = 0;	// make sure string is 0-terminated
	SX_STRING (num_buf);
}

