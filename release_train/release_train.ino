#include <SPI.h>

#include <EthernetUdp.h>
#include <Dns.h>
#include <Dhcp.h>
#include <Ethernet.h>
#include <EthernetServer.h>
#include <EthernetClient.h>

struct InputState {
  boolean switchState;
  boolean section1;
  boolean section2;
  boolean section3;
};

class State {
  public:
    virtual State* getNextState(InputState input) = 0;
    virtual void enter() = 0;
    virtual ~State();
};

State::~State() {}

class AtRest : public State {
  public:
    virtual State* getNextState(InputState input);
    virtual void enter();
};

class GoingForwards : public State {
  public:
    virtual State* getNextState(InputState input);
    virtual void enter();
};

class GoingBackwards : public State {
  public:
    virtual State* getNextState(InputState input);
    virtual void enter();   
};

State* AtRest::getNextState(InputState input)
{
  if(input.switchState)
    return new GoingForwards();
  return NULL;
}

State* GoingForwards::getNextState(InputState input)
{
  if(input.section3)
    return new GoingBackwards();
  return NULL;
}

State* GoingBackwards::getNextState(InputState input)
{
  if(input.section1)
    return new AtRest();
  return NULL;
}

void AtRest::enter() {
  digitalWrite(8, LOW);
  digitalWrite(11, LOW);  
}

void GoingForwards::enter() {
  digitalWrite(8, LOW);
  digitalWrite(11, HIGH);  
}

void GoingBackwards::enter() {
  digitalWrite(8, HIGH);
  digitalWrite(11, LOW);  
}

State *currentState;

void setup() {  
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);
  // put your setup code here, to run once:

  setPwmFrequency(9, 256);
  Serial.begin(9600);
  Serial.println("Starting");
  currentState = new AtRest();
  currentState->enter();
}


void loop() {
  // put your main code here, to run repeatedly:
  doPwm(9, A2, A3);

  InputState state = readInputs();

  State * newState = currentState->getNextState(state);
  if(newState == NULL) return;

  newState->enter();
  delete currentState;
  currentState = newState;  
}

struct InputState readInputs()
{
  bool sect1 = digitalRead(5) == LOW;
  bool sect2 = digitalRead(6) == LOW;
  bool sect3 = digitalRead(4) == LOW;
  bool button = digitalRead(3) == HIGH;
  
  InputState state = {button, sect1, sect2, sect3};

  return state;
}

// 1 second's worth of PWM
void doPwm(int pin, int speedPin, int trimmerPin)
{
  int val = analogRead(speedPin);
  int rangeBottom = analogRead(trimmerPin);
  int bottom = map(rangeBottom, 0, 1023, 0, 255);
  int speed = map(val, 0, 1023, bottom, 225);
  if(speed == bottom) speed = 0;
  analogWrite(pin, speed);
}

/**
 * Divides a given PWM pin frequency by a divisor.
 *
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 *
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 *
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 *
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = (TCCR0B & 0b11111000) | mode;
    } else {
      TCCR1B = (TCCR1B & 0b11111000) | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = (TCCR2B & 0b11111000) | mode;
  }
}

