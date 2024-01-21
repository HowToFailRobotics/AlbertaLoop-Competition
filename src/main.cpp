#include <Arduino.h>
#include "IRremote.h"
#include <avr/sleep.h>

#define TASK1 0
#define TASK2 1
#define TASK3 2
#define TASK4 3
#define NUM_STATES 4

// States
#define FORWARD 1
#define STOP    2
#define REVERSE 3
#define LEFT 4
#define RIGHT 5

#define PROXIMITY_INPUT  7

bool timer_ready = false;
int state = TASK1;
int motor_state = 2;
int next_motor_state = 2;


// Motor 1
int dir1PinA = 2;
int dir2PinA = 3;
int speedPinA = 10; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int dir1PinB = 4; // Direction pin for Motor 2
int dir2PinB = 5; // Alternate direction pin for Motor 2 (if needed)

// IR Receiver
int receiver = 11; // Signal Pin of IR receiver to Arduino Digital Pin 11
IRrecv irrecv(receiver);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'
unsigned long key_value = 0;

void init_timer()
{
//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624 / 100;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

void init_power_save()
{
  SMCR |= (1 << SM1) | (1 << SM0);
  SMCR |= (1 << SE);
}

void init_idle()
{
  SMCR &= ~(1 << SM2) & ~(1 << SM1) & ~(1 << SM0);
  SMCR |= (1 << SE);
}

void translateIR() // takes action based on IR code received

// describing Remote IR codes 
{
  if (results.value == 0xFFFFFFFF) {return;}
  Serial.println("Hex value:");
  Serial.println(results.value, HEX);
  switch(results.value)

  {
  case 0xFFA25D: Serial.println("POWER"); break;
  case 0xFFE21D: Serial.println("FUNC/STOP"); break;
  case 0xFF629D: Serial.println("VOL+"); break;
  
  case 0xFF22DD:
    Serial.println("FAST BACK");
    Serial.println(motor_state);
    if(motor_state == FORWARD)
      next_motor_state = STOP;
    else
      next_motor_state = REVERSE;
    break;

  case 0xFF02FD:
    Serial.println("PAUSE");
    next_motor_state = STOP;
    break;

  case 0xFFC23D:
    Serial.println("FAST FORWARD");
    if(motor_state == REVERSE)
      next_motor_state = STOP;
    else
      next_motor_state = FORWARD;
    break;

  case 0xFF906F:
    Serial.println("RIGHT");
    if(motor_state != STOP)
      next_motor_state = STOP;
    else
      next_motor_state = RIGHT;
    break;

  case 0xFFE01F:
    Serial.println("LEFT");
    if(motor_state != STOP)
      next_motor_state = STOP;
    else
      next_motor_state = LEFT;
    break;

  // case 0xFFE01F: Serial.println("DOWN");    break;
  case 0xFFA857: Serial.println("VOL-");    break;
  // case 0xFF906F: Serial.println("UP");    break;
  case 0xFF9867: Serial.println("EQ");    break;
  case 0xFFB04F: Serial.println("ST/REPT");    break;
  case 0xFF6897: Serial.println("0");    break;
  case 0xFF30CF: Serial.println("1");    break;
  // case 0xFF18E7: Serial.println("2");    break;
  case 0xFF7A85: Serial.println("3");    break;
  case 0xFF10EF: Serial.println("4");    break;
  // case 0xFF38C7: Serial.println("5");    break;
  case 0xFF5AA5: Serial.println("6");    break;
  case 0xFF42BD: Serial.println("7");    break;
  // case 0xFF4AB5: Serial.println("8");    break;
  case 0xFF52AD: Serial.println("9");    break;
  case 0xFFFFFFFF: Serial.println(" REPEAT");break;  

  default: 
    Serial.println(" other button   ");

  }// End Case
  key_value = results.value;
  // delay(500); // Do not get immediate repeat


} //END translateIR


void motor_forward()
{
  Serial.println("starting motor");
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, HIGH);
  // analogWrite(speedPinA, 50);

  // Motor 2
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, HIGH);
}

void motor_reverse()
{
  Serial.println("reversing motor");
  digitalWrite(dir1PinA, HIGH);
  digitalWrite(dir2PinA, LOW);

  digitalWrite(dir1PinB, HIGH);
  digitalWrite(dir2PinB, LOW);
}

void motor1_forward() {
    digitalWrite(dir1PinA, LOW);
    digitalWrite(dir2PinA, HIGH);
}

void motor1_reverse() {
    digitalWrite(dir1PinA, HIGH);
    digitalWrite(dir2PinA, LOW);
}

void motor2_forward() {
    digitalWrite(dir1PinB, LOW);
    digitalWrite(dir2PinB, HIGH);
}

void motor2_reverse() {
    digitalWrite(dir1PinB, HIGH);
    digitalWrite(dir2PinB, LOW);
}

void motor_stop()
{
  // Serial.println("stopping motor");
  digitalWrite(dir1PinA, LOW);
  digitalWrite(dir2PinA, LOW);
  analogWrite(speedPinA, 0);
  digitalWrite(dir1PinB, LOW);
  digitalWrite(dir2PinB, LOW);
}



void task1()
{
  // Serial.println("Task 1");
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
    irrecv.resume(); // receive the next value
    translateIR(); 
    // Serial.println("recieved signal");
  }  
}

void task2()
{
  // Serial.println("Task 2");
  int sensorValue = digitalRead(7);
  // Serial.println(sensorValue);
}

void task3()
{
  // Serial.println("Task 3");
  switch(motor_state)
  {
    case FORWARD:
      motor1_forward();
      motor2_forward();
    break;
    case REVERSE:
      motor1_reverse();
      motor2_reverse();
    break;
    case RIGHT:
      motor1_reverse();
      motor2_forward();
    break;
    case LEFT:
      motor1_forward();
      motor2_reverse();
    break;
    case STOP:
      motor_stop();
    break;
    default:
    break;
  }
  if (next_motor_state != motor_state) {
    motor_state = next_motor_state;
  }
}




void setup() {

  // Motor 1
  pinMode(dir1PinA,OUTPUT);
  pinMode(dir2PinA,OUTPUT);

  // Setup for Motor 2
  pinMode(dir1PinB, OUTPUT);
  pinMode(dir2PinB, OUTPUT);

  // Proximity
  pinMode(PROXIMITY_INPUT, INPUT);

  

  irrecv.enableIRIn(); // Start the receiver


  sei();//allow interrupts
  init_timer();
  init_power_save();
  Serial.begin(115200);

}


void atmega_sleep()
{
  sleep_enable();
  sleep_cpu();
  sleep_disable();
}

void loop() {

  if(timer_ready)
  {
    // Serial.print("State = ");
    // Serial.println(state);
    switch(state)
    {
      case TASK1: 
        task1();
      break;
      case TASK2:
        task2();
      break;
      case TASK3:
        task1();
      break;
      case TASK4:
        task3();
      break;
      default: break;
    }
    state++;

    if(state%(NUM_STATES) == 0) state = TASK1;
    timer_ready = false;
  }

}

// timer1
ISR(TIMER1_COMPA_vect) {
// process the timer1 overflow here
  timer_ready = true;
}