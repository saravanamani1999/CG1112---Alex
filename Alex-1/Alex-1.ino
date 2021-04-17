#include <buffer.h>

#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <avr/sleep.h>



#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b11110001


typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
}TDirection;


volatile TDirection dir = STOP;

#define S0 A5
#define S1 A4
#define S2 A2
#define S3 A3
#define VCC 7
#define sensorOut A0
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0


// change this to make the song slower or faster
int tempo = 180;

// change this to whichever pin you want to use
int buzzer = 11;

// notes of the moledy followed by the duration.
// a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
// !!negative numbers are used to represent dotted notes,
// so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
int melody[] = {

  // Nokia Ringtone 
  // Score available at https://musescore.com/user/29944637/scores/5266155
  
  NOTE_E5, 8, NOTE_D5, 8, NOTE_FS4, 4, NOTE_GS4, 4, 
  NOTE_CS5, 8, NOTE_B4, 8, NOTE_D4, 4, NOTE_E4, 4, 
  NOTE_B4, 8, NOTE_A4, 8, NOTE_CS4, 4, NOTE_E4, 4,
  NOTE_A4, 2, 
};

// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;

// this calculates the duration of a whole note in ms
int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;


/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      183

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          21.3

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  11  // Right forward pin
#define RR                  10 // Right reverse pin

const int offset = 5;
TBuffer *buf;
/*
 *    Alex's State Variables
 */

#define PI                  3.141592654
#define ALEX_LENGTH         21.2
#define ALEX_BREADTH        14.3

float AlexDiagonal = 0.0;
float AlexCirc = 0.0;



// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;


// Store the revolutions on Alex's left
// and right wheels
float leftRevs;
float rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

unsigned long diff_l;
unsigned long diff_r;
unsigned long ticks_l;
unsigned long ticks_r;


/*
 * 
 * Alex Communication Routines.
 * 
 */

 
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}
void WDT_off(void){
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0x00;
}

void setupPowerSaving()
{
  WDT_off();
  
  PRR |= PRR_TWI_MASK;

  PRR |= PRR_SPI_MASK;

  ADCSRA &= ~ADCSRA_ADC_MASK;

  PRR |= PRR_ADC_MASK;

  SMCR &= SMCR_IDLE_MODE_MASK;

  DDRB |= 0b00100000;

  PORTB &= 0b11011111;
}

void putArduinoToIdle()
{
  PRR |= (PRR_TIMER0_MASK | PRR_TIMER1_MASK | PRR_TIMER2_MASK);

  SMCR |= SMCR_SLEEP_ENABLE_MASK;

  sleep_cpu();

  SMCR &= ~SMCR_SLEEP_ENABLE_MASK;

  PRR &= ~(PRR_TIMER0_MASK | PRR_TIMER1_MASK | PRR_TIMER2_MASK);
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);
  
}

void sendColour()
{
  TPacket colourPacket;
  colourPacket.packetType = PACKET_TYPE_RESPONSE;
  colourPacket.command = RESP_COLOUR;
  colourPacket.params[0] = colourSensing();  

  if(colourSensing()){
    sendMessage("red");
  }

  else{
    sendMessage("green");
  }
  
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}


void dbprint(char *format, ... ) {
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer); 
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

bool colourSensing() {

  int redCount = 0;
  int greenCount = 0;
  int red,green,i;
  
  for(i = 0;i < 40; i++) {


    PORTC &= 0b11110011;
    
    delay(10);

    red = pulseIn(sensorOut, LOW);
    

    PORTC |= 0b00001100;
    delay(10);

    green = pulseIn(sensorOut,LOW);

    if(red < green) { 
      redCount++;
    } 
    else {    
      greenCount++;
    }
    
  }
  if( redCount > greenCount) {
    return true;
  }

  else {
    return false;
  }
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD  &= 0b11110011;
  PORTD |= 0b00001100;
  
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD){
    leftForwardTicks++;
  }
  else if (dir == BACKWARD){
    leftReverseTicks++;
  }
  else if (dir == LEFT){
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT){
    leftForwardTicksTurns++;
  }

  if (dir == FORWARD){
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARD){
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
}

void rightISR()
{
  
  if (dir == FORWARD){
    rightForwardTicks++;
  }
  else if (dir == BACKWARD){
    rightReverseTicks++;
  }
  else if (dir == RIGHT){
    rightReverseTicksTurns++;
  }
  else if (dir == LEFT){
    rightForwardTicksTurns++;
  }
 
  
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
//    UBRR0L = 103;
//    UBRR0H = 0;

//    UCSR0C = 0b00100100;
//    UCSR0A = 0;
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
//  UCSR0B = 0b00011000;
//  initBuffer(buf,1000);
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(unsigned char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();
//  while(dataAvailable(buf)){
//    readBuffer(buf,buffer + count++);
//  }

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(unsigned const char *buffer, int len)
{
  Serial.write(buffer, len);

//  for (int i = 0; i < len;i++){
//    writeBuffer(buf,buffer + i);
//  }
}

/*
 * Alex's motor drivers.
 * 
 */

 ISR(TIMER0_COMPA_vect)
 {
  
 }

 
 ISR(TIMER0_COMPB_vect)
 {
  
 }
 
 ISR(TIMER1_COMPB_vect)
 {
  
 }
 
 ISR(TIMER2_COMPA_vect)
 {
  
 }

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
   
   DDRD |= 0b01100000;
   DDRB |= 0b00001100;

   TIMSK0 |= 0b00000110;
   TIMSK1 |= 0b00000100;
   TIMSK2 |= 0b00000010;

   TCNT0 = 0;
   TCNT1 = 0;
   TCNT2 = 0;
   TCCR0A = 0b11110001;
   TCCR1A = 0b00110001;
   TCCR2A = 0b11000001;
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
   OCR0A = 0;
   OCR0B = 0;
   OCR1B = 0;
   OCR2A = 0;

  
  TCCR0B |= 0b00000001;
  
  TCCR1B |= 0b00000001;
  
  TCCR2B |= 0b00000001;
  
  
}



// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  clearCounters();
  if(dist > 0)    
    deltaDist = dist;
  else
    deltaDist = 9999999;
  newDist = forwardDist + deltaDist;


  
  dir = FORWARD;
  //int val = pwmVal(speed);
  int power_l = pwmVal(speed);
  int power_r = pwmVal(speed);



  leftForwardTicks = 0;
  rightForwardTicks = 0;

  unsigned long leftForwardTicks_prev = leftForwardTicks;
  unsigned long rightForwardTicks_prev = rightForwardTicks;

  leftRevs = (dist)/WHEEL_CIRC;
  rightRevs = (dist)/WHEEL_CIRC;
  unsigned long targetCount_l = leftRevs*COUNTS_PER_REV;
  unsigned long targetCount_r = rightRevs*COUNTS_PER_REV;

  while ((leftForwardTicks < targetCount_l) && (rightForwardTicks < targetCount_r)){

    ticks_l = leftForwardTicks;
    ticks_r = rightForwardTicks;

    
    OCR0A = power_l;
    OCR0B = 0;
    OCR2A = 0;
    OCR1B = power_r; 

    diff_l = ticks_l - leftForwardTicks_prev;
    diff_r = ticks_r - rightForwardTicks_prev;

    leftForwardTicks_prev = ticks_l;
    rightForwardTicks_prev = ticks_r;

    if (diff_l > diff_r){
      power_l -= offset;
      power_r += offset;
    }

    if (diff_r > diff_l){
      power_l += offset;
      power_r -= offset;
    }
  }
  

}
  
 

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  clearCounters();

  
  dir = BACKWARD;
 
  int power_l = pwmVal(speed);
  int power_r = pwmVal(speed);

  if(dist > 0)    
    deltaDist = dist;
  else
    deltaDist = 9999999;
  newDist = forwardDist + deltaDist;


  leftReverseTicks = 0;
  rightReverseTicks = 0;

  unsigned long leftReverseTicks_prev = leftReverseTicks;
  unsigned long rightReverseTicks_prev = rightReverseTicks;

  leftRevs = (dist)/(WHEEL_CIRC);
  rightRevs = (dist)/(WHEEL_CIRC);
  unsigned long targetCount_l = leftRevs*COUNTS_PER_REV;
  unsigned long targetCount_r = rightRevs*COUNTS_PER_REV;

  while ((leftReverseTicks < targetCount_l) && (rightReverseTicks < targetCount_r)){

    ticks_l = leftReverseTicks;
    ticks_r = rightReverseTicks;
    
    OCR0A = 0;
    OCR0B = power_l;
    OCR2A = power_r;
    OCR1B = 0;



    diff_l = ticks_l - leftReverseTicks_prev;
    diff_r = ticks_r - rightReverseTicks_prev;

    leftReverseTicks_prev = ticks_l;
    rightReverseTicks_prev = ticks_r;

    if (diff_l > diff_r){
      power_l -= offset;
      power_r += offset;
    }

    if (diff_r > diff_l){
      power_l += offset;
      power_r -= offset;
    }
  }
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
unsigned long computeDeltaTicks(float ang) {
 
  unsigned long ticks = (unsigned long)((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
  
}
void left(float ang, float speed)
{
  //clearOneCounter();
  dir = LEFT;
  int val = pwmVal(speed);

  if (ang > 0) {
    deltaTicks = computeDeltaTicks(ang);
  }
  else {
    deltaTicks = 9999999;
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.

    OCR0A = 0;
    OCR0B = val;
    OCR2A = 0;
    OCR1B = val;
  
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  //clearOneCounter();
  dir = RIGHT;
  int val = pwmVal(speed);
  if (ang > 0) {
    deltaTicks = computeDeltaTicks(ang);
  }
  else {
    deltaTicks = 9999999;
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
      

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.

    OCR0A = val;
    OCR0B = 0;
    OCR2A = val;
    OCR1B = 0;
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  OCR0A = 0;
  OCR0B = 0;
  OCR2A = 0;
  OCR1B = 0;
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void victory(){
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, melody[thisNote], noteDuration * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(buzzer);
  }
}

void handleCommand(TPacket *command) 
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_AUTO_FORWARD:
        sendOK();
        forward(20,70);
      break;
      
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_AUTO_REVERSE:
        sendOK();
        reverse(20,70);
      break;
      
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_AUTO_RIGHT:
        sendOK();
        right(16,100);
      break;
      
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_AUTO_LEFT:
        sendOK();
        left(16,100);
      break;

    case COMMAND_STOP:
        sendOK();
        stop();
      break;


    case COMMAND_GET_STATS:
        sendStatus();
      break;


    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
        
      break;

    case COMMAND_GET_COLOUR:                                                                                                                                                                                                                                            
        sendColour();     
      break;
      
    case COMMAND_GET_TONE:                                                                                                                                                                                                                                            
        victory();     
      break;

    /*
     * Implement code for other commands here.
     * 
     */
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}



void setup() {
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
  // put your setup code here, to run once:
  

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupPowerSaving();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupColourModule();
  sei();
}

void setupColourModule(){


  DDRC |= 0b00111100;
  DDRC &= 0b11111110;

  PORTC |= 0b00100000;
  PORTC &= 0b11101111;
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

 //forward(0, 100);
 //
 //left(0,100);
 //reverse(0,50);
 //right(0,50);

// Uncomment the code below for Week 9 Studio 2


 // put your main code here, to run repeatedly:
 
 TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 

  if(deltaDist > 0) {
    if(dir == FORWARD)
    {
      if(forwardDist >= newDist)
      {
        deltaDist=0;
        newDist=0;
        stop();
      }
    }
    else if(dir == BACKWARD){
      if(reverseDist >= newDist){
        deltaDist=0;
        newDist=0;
        stop();
      }
    }
    

    else if(dir == STOP){
      deltaDist=0;
      newDist=0;
      stop();
    }
  }

  if (deltaTicks > 0)
  {
    if(dir == LEFT)
    {
      if(leftReverseTicksTurns >= targetTicks)
      {
        deltaDist=0;
        targetTicks=0;
        stop();
      }
    }
    else if(dir == RIGHT){
      if(rightReverseTicksTurns >= targetTicks){
        deltaDist=0;
        targetTicks=0;
        stop();
      }

    }
    else if(dir == STOP){
      deltaDist=0;
      newDist=0;
      stop();
    }
  }

      
      
      
}
