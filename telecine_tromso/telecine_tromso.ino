#include <inttypes.h>

/* pins circuito
    enab
    dir
    step
    vent
    sensor
    boton1
    boton2
    led
    pot
*/
const int enabPin = 5;
const int dirPin = 6;
const int stepPin = 7;
const int vent = 8;
const int sensor = 9;
const int button1 = 10;
const int button2 = 11;
const int led = 12;
const int pot = A0;

bool home_position = false;
bool first_round = false;

#define auto_reset              0
#define start_moving_forward    1
#define start_moving_backward   2
#define keep_moving             3
#define start_braking           4
#define keep_braking            5
#define auto_end                6
#define frame                   7
#define led2                    8

static uint8_t state = start_braking;
static uint8_t is_manual = 0;

int move_motor(void){
  
  for (int i = 0; i < 38; i++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(50);
      }
}

int advance_constant(int x){
 
  int  val = digitalRead(sensor);
  int counter = 0;
  while (counter < x){
    for (int i = 0; i < 6400; i ++){ 
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(50);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(50);   
  }
  Serial.write('0' + val);
  digitalWrite(led, HIGH);
  delayMicroseconds(500);
  digitalWrite(led, LOW);
  counter += 1;
  Serial.println(counter);
  
}

}
int serial_monitor(void){

  byte gate[8]; //array used to make an average of the values; 0 means the shutter is in front of the sensor
  int sum_gate = 0;
  int val = 0;

    if (state == start_moving_forward){
      for (int i = 0; i < 8; i ++){
      val = digitalRead(sensor);
      gate[i] = val;
      }  
      for (int i = 0; i < 8; i ++){ 
        sum_gate += gate[i];
        Serial.println("This is the value of the gate");
        Serial.println(gate[i]);
      }
      Serial.println(sum_gate);
      
      if (sum_gate == 8){
        digitalWrite(led, HIGH);
        delayMicroseconds(1000);
        digitalWrite(led, LOW);
        delayMicroseconds(1000);
      }else{
        sum_gate = 0;
      }
    }
}
    
int blinking_led(void){
  for (int i = 0; i < 50; i ++){
    digitalWrite(led, HIGH);
    delay(200);
    digitalWrite(led, LOW);
    delayMicroseconds(200);
  }

  state = auto_end;
}

int homing(void){
  
  digitalWrite(dirPin, LOW);
  int val = 0;
  bool gate_closed = true; //boolean related roughly to the shutter in front of the sensor
  bool center_not_found = true; //boolean related precisely to the sensor being at the end of the shutter
  byte gate[8]; //array used to make an average of the values; 0 means the shutter is in front of the sensor
  int sum_gate = 0;

    while (gate_closed == true && center_not_found == true){
      for (int i = 0; i < 8; i ++){
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(50);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(50);
      val = digitalRead(sensor);
      gate[i] = val;
      }  
      for (int i = 0; i < 8; i ++){ 
        sum_gate += gate[i];
        Serial.println(gate[i]);
      }
      Serial.println(sum_gate);
      
      if (sum_gate == 0){
        gate_closed = false;
        continue;
      }else{
        sum_gate = 0;
        continue;
      }
    }
   
   if (gate_closed == false){
      val = digitalRead(sensor);
      
      while (center_not_found == true){
        Serial.println("buscando el 8");
        for (int i = 0; i < 8; i ++){   
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(50);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(50);        
        val = digitalRead(sensor);
        gate[i] = val;
      }
      for (int i = 0; i < 8; i ++){ 
        sum_gate += gate[i];
        Serial.println(gate[i]);
      }
      Serial.println(sum_gate);
      if (sum_gate == 8){
        center_not_found = false;
        continue;
      }else{
        sum_gate = 0;
        continue;
      }  
    }
  }

    digitalWrite(dirPin, HIGH);
    for (int i = 0; i < 65; i++){
      move_motor();
      }
}

void advance_frame(int x){

  int  val = digitalRead(sensor);
  int counter = 0;
  digitalWrite(dirPin, LOW);
  while (counter < x){
    for (int i = 0; i < 6400; i ++){ 
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(50);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(50);   
  }
  Serial.write('0' + val);
  digitalWrite(led, HIGH);
  delayMicroseconds(500);
  digitalWrite(led, LOW);
  counter += 1;
  Serial.println(counter);
}
  state = auto_end;
}
 

ISR(TIMER1_OVF_vect)
{
   
  switch ( state )
  {
    case auto_reset:
      homing();
      Serial.println("Homing executed");
      state = auto_end;
      Serial.println(state);
      break;

    case start_moving_forward:
      homing();
      digitalWrite(dirPin, LOW);
      delayMicroseconds(1000000);
      goto START_MOVING;
    
    case start_moving_backward:
      homing();
      digitalWrite(dirPin, HIGH);
      delayMicroseconds(1000000);
      goto START_MOVING;

START_MOVING:
      state = keep_moving;
      break;
    
    case keep_moving:
      home_position = false;
      move_motor();
      state = keep_moving;
      break;

    case start_braking:
      digitalWrite(stepPin, LOW);
      state = keep_braking;
      break;
   
    case keep_braking:
      digitalWrite(stepPin, LOW);
      state = auto_end;
      break;
      
    case auto_end:
      home_position = false;
      break;

    case frame:
      advance_frame(20);
      break;

    case led2:
      blinking_led();
      break;
  }

  /* buttons */
  if ( digitalRead(button1) ) /* button 1 */
  {
    if ( is_manual == 0 )
    {
      is_manual = 1;
      homing();
      delayMicroseconds(500000);
      Serial.println("Aqui");
      state = start_moving_forward;
    }
  }
  else if ( digitalRead(button2) ) /* button 2 */
  {
    if ( is_manual == 0 )
    {
      homing();
      delayMicroseconds(500000);
      is_manual = 1;
      state = start_moving_backward;
    }
  }
  else if ( is_manual )
  {
    /* was manual, start brake */
    is_manual = 0;
    state = start_braking;
  }
}


/*ISR(TIMER2_OVF_vect)
{
  //blinking_led();
}

*/
int main(void) {

  pinMode(enabPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(vent, OUTPUT);
  pinMode(sensor, INPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(led, OUTPUT);
  pinMode(A0, INPUT);

  digitalWrite(enabPin, LOW);
  digitalWrite(dirPin, HIGH);

  Serial.begin(115200);

cli ();
TCCR1A = (0 << COM1A1) | (0 << COM1A0) /* disconnected */
       | (0 << COM1B1) | (0 << COM1B0) /* disconnected */
       | (0 << WGM11 ) | (1 << WGM10 );
TCCR1B = (0 << FOC1A ) | (0 << FOC1B )
       | (1 << WGM12 )
       | (1 << CS12  ) | (0 << CS11  ) | (0 << CS10 ); /* 256 prescaler */
TCNT1  = 0;
OCR1A  = 0;
OCR1B  = 0;
/*

TCCR2A = (0 << COM2A1) | (0 << COM2A0) 
       | (0 << COM2B1) | (0 << COM2B0) 
       | (1 << WGM21 ) | (1 << WGM20 );
TCCR2B = (0 << FOC2A ) | (0 << FOC2B )
       | (1 << WGM22 )
       | (0 << CS22  ) | (0 << CS21  ); 

TIMSK2 |= (1 << TOIE2);

TCNT2 = 0;
OCR2A = 0;
OCR2B = 0;
*/


TIMSK1 |= (1<<TOIE1);


GTCCR = 0;

sei();


  while (1)
  {
  
    if ( Serial.available() )
    {
      uint8_t code = Serial.read();
     Serial.println(code);
      switch ( code )
      {
        case '0':
          state = auto_reset;
          break;
        case '1':
          state = start_moving_forward;
          break;
        case '2':
          state = start_moving_backward;
          break;
        case '4':
          state = start_braking;
          break;
        case '5':
          state = frame;
          break;
        case '6':
          state = led2;
          break;
      
      }
    }
  }
}
/*

/*void loop() {
  homing();
  }*/
