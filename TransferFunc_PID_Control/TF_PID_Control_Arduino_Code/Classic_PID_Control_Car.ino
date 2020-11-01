//------- header file and setup macro constant
//These define's must be placed at the beginning before #include "TimerInterrupt.h"
#define TIMER_INTERRUPT_DEBUG      0

#define USE_TIMER_1     false
#define USE_TIMER_2     true
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#include "TimerInterrupt.h"
#include "TimerOne.h"
#include "AsyncSonarLib.h"
#include <PID_v1.h>




//------ PIN and HW definition

// hardwire connect pin 5&6&9, pin 7&4
#define PWMSPEED_PIN 9
#define DIRECTION_PIN 4
#define DIRECTION_PIN_B 7
#define POTENTIAL_METER_PIN A1
#define SONAR_PIN A0




// system control constant
#define Sync_TIMER_INTERVAL_MS    10   //sampling period
#define Sonar_Sampling_per_PID_CYCLE 1  // sonar sampling before dO pid control
#define PID_TIMER_INTERVAL_MS 10   //PID sampling period: it should equal to Sync_TIMER_INTERVAL_MS*Sonar_Sampling_per_PID_CYCLE
#define PWM_Frequency_uS 5000

#define PID_OUTPUT_MAPPING_OFFSET  12  //offset value for motor duty cycle mapping, the mininum value corresponding to the least cycle that drive on motor
#define STOPTHREATHOLD 1  // if the duty cycle is below this value,  we turn it off in order to avoid vibration.  it refer to PID output vluae
#define NumberofDistanceFilter 10 // the distance check filter. in order to find the right location target



// system control

// 0.552V (-PI/2) 1.625(0) 2.698(+PI/2)
#define ANGLE_MAPPING_VALUE1  180 // Resistance value corresponding to start angle
#define ANGLE_MAPPING_VALUE2  900 // Resistance value corresponding to end angle
#define ANGLE_MAPPING_RESULT1  -0.5// -1/6 PI , start angle unit in pi
#define ANGLE_MAPPING_RESULT2  0.5//  1/6 PI ,  end angle unit in pi






// virable
volatile bool system_start_flag = false; // flag to turn on everything.  configured by serial command START / STOP
volatile float pwm_duty_cycle = 0;
volatile int pwm_direction = 0;
volatile float sonar_distance = 0;
volatile float angle = 0;
volatile bool update_flag = false, update_sonar_flag=false, change_PID_K_flag=false;


int sonar_sampling_counter =0;


//filter


float sonar_distance_record[NumberofDistanceFilter];




//debug
const int indicator_LED_PIN =  LED_BUILTIN;// the number of the LED pin
bool DEBUG_FLAG=0;

// -----------PID control ----
double current_sonar_distance,target_sonar_distance, PID_output_value;
double Kp1=0.2,Ki1=0,Kd1=0.01;
double Kp2=0.05,Ki2=0,Kd2=0.005;   // for future implementation if need  .

PID myPID(&current_sonar_distance, &PID_output_value, &target_sonar_distance,Kp1,Ki1,Kd1,REVERSE);

// ---- PID control func ---

void PID_initial()
{
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100+PID_OUTPUT_MAPPING_OFFSET,100-PID_OUTPUT_MAPPING_OFFSET);
  myPID.SetSampleTime(PID_TIMER_INTERVAL_MS);
}
// ---- Sub function ------




void serial_com_setup()
{

  Serial.begin(9600);
  debug_print("\nStarting");

}


//--------------------------
void Angle_initial()
{

}

void Angle_checking()
{

  angle = float(map(analogRead(POTENTIAL_METER_PIN), ANGLE_MAPPING_VALUE1, ANGLE_MAPPING_VALUE2, ANGLE_MAPPING_RESULT1*1000, ANGLE_MAPPING_RESULT2*1000))/1000;

}






//------------------------------



void PWM_initial()
{

  Timer1.initialize(PWM_Frequency_uS); // initialize timer1
  // put your setup code here, to run once:
  pinMode(PWMSPEED_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(DIRECTION_PIN_B, OUTPUT);

}



void PWM_set(float dutyCycle, int dir)
{
   digitalWrite(DIRECTION_PIN, dir);
   digitalWrite(DIRECTION_PIN_B, dir);
   Timer1.pwm(PWMSPEED_PIN,(int)(dutyCycle * 1023 /100));
}



// -----------------------------
// --- AsyncSonar ---
// Sonar check call back

void Sonar_Ping_Recieved(AsyncSonar& sonar)
{

  //debug_print("check sona");
  sonar_distance=sonar.GetMeasureMM();
  sonar.Start();
}

// Sonar check timeout callback
void Sonar_Ping_TimeOut(AsyncSonar& sonar)
{
       // debug_print("TimeOut");
        sonar.Start();
}


AsyncSonar sonar_check(SONAR_PIN, Sonar_Ping_Recieved, Sonar_Ping_TimeOut);


void Sonar_Ping_initial()
{
  //sonar.SetTemperatureCorrection(28);  // optional
  sonar_check.Start(10); // start in 100ms
}






//  ------------------------------

void Distance_detection_setup()
{
}







//--------------------------



void Sync_initial()
{
  ITimer2.init();

  if (ITimer2.attachInterruptInterval(Sync_TIMER_INTERVAL_MS,Sync_interrupt_handler))  //add paramter here for call back func if need, follow original code
    debug_print("Sync sampling timer setup is done");
  else
    debug_print("Can not set up timer for sampling");

}

void Sync_interrupt_handler()
{
    //debug_print("sampling at "+  String(millis())+"ms" );
    if (system_start_flag){
      update_sonar_flag=true;
      sonar_sampling_counter+=1;
    }

    if (sonar_sampling_counter==Sonar_Sampling_per_PID_CYCLE){
      sonar_sampling_counter =0;
      update_flag = true;
    }

}



void Sync_update()
{
    // Angle_checking();    //check angle value, here we comment out temperary.

     //sonar_check.Update(1500)  ;

    //debug_print("angle :"+String(angle)+" distance:"+String(sonar_distance));



    //anti collision provention
    if ((current_sonar_distance< 50 ) and (system_start_flag) and (millis()>50)){
        debug_indicator_led(1);
        PWM_set(0,0);
        system_start_flag=false;
        debug_print("anti colllision stop : "+String(current_sonar_distance));

    }

    // if system is in start state and target_sonar_distance is not 0.
    myPID.SetTunings(Kp1, Ki1, Kd1);
    if (change_PID_K_flag)
    {
        if (abs(current_sonar_distance-target_sonar_distance)<180)
        {
            myPID.SetTunings(Kp2, Ki2, Kd2);
        }
    }


    if((system_start_flag)and (target_sonar_distance))
    {
      myPID.Compute();
      float k= abs(PID_output_value*1000)/1000+PID_OUTPUT_MAPPING_OFFSET;


     if (abs(PID_output_value)<STOPTHREATHOLD){
       k=0;
     }

      if (PID_output_value>0){
        PWM_set(k,0);
      }
      else{
        PWM_set(k,1);
      }
      Serial.println(String(target_sonar_distance)+" "+String(current_sonar_distance)+"  "+String(sonar_distance)+" "+String(PID_output_value)+"  "+String(k));

    }



}




void Upper_Stream_Comman_handler()
{

  String content=Serial.readString();
  debug_print(content);

  content.trim();
  //debug_print(content);
  if (content.equals("start")){
    system_start_flag = true;
  }


  if (content.equals("stop")){
    system_start_flag = false;
    PWM_set(0, 0);
  }

  if (content.equals("debug_on")){
    DEBUG_FLAG = true;
    Serial.println("debug mode on");
  }

  if (content.equals("debug_off")){
    DEBUG_FLAG = false;
    Serial.println("debug mode off");
  }


  String cmd=content.substring(0,1);
  float cmd_content=content.substring(1).toFloat();

  //set up target distance eg:t500
  if (cmd=="t"){
    target_sonar_distance = cmd_content;

    //decide whether the coming movement need dynamic change K value.
    // set the level at 400, the reason is only above 400, my car will build up enough speed to require adjsut K
    if (abs(target_sonar_distance-current_sonar_distance)<400){
      change_PID_K_flag =false;
    }
    else{
      change_PID_K_flag =true;
    }

  }


}




void debug_indicator_initial()
{
  pinMode(indicator_LED_PIN,OUTPUT);

}

void debug_indicator_led(bool status)
{
  digitalWrite(indicator_LED_PIN,status);
}






void debug_print(String content)
{
  if (DEBUG_FLAG){
    Serial.println(content);
  }
}



float sonar_distance_filter()
{
  float temp_total_distance = 0;
  temp_total_distance += sonar_distance;
  for (int i=NumberofDistanceFilter-1;i>0;i--){
    sonar_distance_record[i]=sonar_distance_record[i-1];
    temp_total_distance+=sonar_distance_record[i];
  }
  sonar_distance_record[0]=sonar_distance;
  return temp_total_distance/NumberofDistanceFilter;
}


void setup()
{

  serial_com_setup();
  Sonar_Ping_initial();
  sonar_check.Update();

  PWM_initial();
  PID_initial();
  //from here the system will start its sync sampling and actions.
  Sync_initial();

  //debug subfuncs
  debug_indicator_initial();


}

void loop()
{

  // 1. process upper stream command
  if (Serial.available() > 0) {
    Upper_Stream_Comman_handler();
  }

 // 2. keep sampling , but do it outside call back func, in order to save accurate counter accumulation.
   if (update_sonar_flag)
   {

     update_sonar_flag=false;

     sonar_check.Update();

     current_sonar_distance=sonar_distance_filter();

   }


   if (update_flag)
   {
      Sync_update();
      update_flag=false;
   }


}
