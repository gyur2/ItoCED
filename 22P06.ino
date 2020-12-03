#include <Servo.h>

#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10 

#define _DIST_TARGET 175  
#define _DIST_MIN 87
#define _DIST_MAX 400

#define _DUTY_MIN 850 
#define _DUTY_NEU 1550
#define _DUTY_MAX 2000

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30

#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 30   
#define _INTERVAL_SERIAL 100 

#define _DIST_ALPHA 0.4
#define DELAY_MICROS 1500
#define _KP 2 //0.0   

Servo myservo; 


float dist_target; // location to send the ball
float dist_raw, dist_ema, dist_min, dist_max; 

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;  
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr, duty_neutral;   

// PID variables
float error_curr, error_prev, control, pterm; //, dterm, iterm; 
float filter_dist, samples_num;

void setup() {
  last_sampling_time_serial = 0;
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  myservo.attach(PIN_SERVO);

  dist_min = _DIST_MIN;               
  dist_max= _DIST_MAX; 
  dist_target = _DIST_TARGET;
  dist_ema = 0;
  samples_num= 3;
  duty_curr = _DUTY_NEU;
  duty_neutral = _DUTY_NEU;
  
  myservo.writeMicroseconds(duty_neutral);
// initialize serial port
  delay(50);
  
  Serial.begin(57600);
  
  //event_dist = event_servo = event_serial = false;
  duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
}

void loop() {
  /*
  unsigned long time_curr = millis();
   if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  */
  event_serial = true; event_dist = true; event_servo = true;
  
  if(event_dist) {
    event_dist = false;  
    // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered(); 

    // PID control logi
    error_curr = dist_target - dist_ema; 
    pterm =  error_curr; 
    control = _KP*pterm; 

  // duty_target = f(duty_neutral, control)
    duty_target = duty_neutral + control; 

    if (duty_target > _DUTY_MAX) {
      duty_target = _DUTY_MAX;     
    } else if (duty_target < _DUTY_MIN) { 
      duty_target = _DUTY_MIN;       
    } 
  }
  
  if(event_servo) {
    event_servo = false;

    // adjust duty_curr toward duty_target by duty_chg_per_interval 
    if(duty_target > duty_curr) {
        duty_curr += duty_chg_per_interval;
        if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    myservo.writeMicroseconds(duty_curr);

    last_sampling_time_serial += _INTERVAL_SERIAL;
    last_sampling_time_servo += _INTERVAL_SERVO;
    last_sampling_time_dist += _INTERVAL_DIST;
  }

  //float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
    if(event_serial) {
      event_serial = false;     
      Serial.print("dist_ir:");
      Serial.print(dist_raw);
      Serial.print(",pterm:");
      Serial.print(map(pterm,-1000,1000,510,610));
      //Serial.print(",dterm:");
      //Serial.print(map(dterm,-1000,1000,510,610));
      Serial.print(",duty_target:");
      Serial.print(map(duty_target,1000,2000,410,510));
      Serial.print(",duty_curr:");
      Serial.print(map(duty_curr,1000,2000,410,510));
      Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void)
{ // return value unit: mm
  float val; 
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val; 
}
float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float ir_distance_filtered(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // ema 필터 추가
  dist_ema = _DIST_ALPHA * lowestReading + (1-_DIST_ALPHA)*dist_ema;
  return dist_ema;
}
