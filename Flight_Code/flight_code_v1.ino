#include<math.h>
#include<Servo.h>

//
// ATTENTION !!!!!!!!!
//
#define HITL true //IF FLYING, THIS SHOULD BE FALSE
//
// ATTENTION !!!!!!!!!
//

float TIMESTEP = 0.05; // s
float MASS = 0.650; // kg
float GRAVITY = -9.80; // m/s/s
float TARGET_APOGEE = 254.508; //  m
float DRY_MASS = 0.650; // kg
float DRY_WEIGHT = DRY_MASS * GRAVITY; // N

float SEARCH_FOR = 100.0f; // milliseconds, how long to stay in loop looking for ideal cd before saving state and cycling through rest of state machine
float OLD_DATA = 1.0f; // seconds, any sensor data older than this is discarded
int ARMING_DELAY = 5.0f; // how long, in seconds, between when the flight computer is turned on and when it should be ready on the launchpad
float TAKEOFF_THRESHOLD = 10.0f; // m/s,  when we go faster than this, computer thinks we have tacken off
float ACCELERATION_SAMPLE_TIME = 0.5f; // seconds, how long between acceleration samples when detecting motor burnout 
float BURNOUT_ACCELERATION_THRESHOLD = -8.0f; // m/s/s, when acceleration is more negative than this, computer thinks motor has burned out

float MIN_CD = 0.000944f;
float MAX_CD = 0.00521f;

float MAX_ERROR = 1.0f; // m

float VELOCITY_DELTA_T = 0.1f; //how many seconds between velocity update cycles

Servo airbrakes;
float deploy_angle = 0.0f;

float apogee_finder(float v, float a, float d){
    float cd = d;
    while (v > 0){
        float drag_force = -(v * v) * cd;
        float acceleration = (drag_force + DRY_WEIGHT)/MASS;
        a += v * TIMESTEP;
        v += acceleration * TIMESTEP;
    }
    
    return (a);
}

bool still_running = false;
float max_cd = MAX_CD;
float min_cd = MIN_CD;
float cd = 0.0f;
float v_running;
float a_running;
float target_alt;
float alt_finder(){
    float a = -0.05;
    //float b = TARGET_APOGEE;
    return a * (a_running - TARGET_APOGEE) + TARGET_APOGEE;
}
float cd_finder(float velocity, float altitude){
    if(!still_running){
        max_cd = MAX_CD;
        min_cd = MIN_CD;
        cd = 0.0f;
        v_running = velocity;
        a_running = altitude;
        target_alt = alt_finder();
    }
    float t = millis();
    //int i = 0;
    
    if(apogee_finder(v_running, a_running, min_cd) < target_alt){
        return min_cd;
    }
    else if (apogee_finder(v_running, a_running, max_cd) > target_alt){
        return max_cd;
    }
    while(true){
        //i ++;
        cd = (min_cd + max_cd)/2;
        //Serial.println("????");
        
        float alt = apogee_finder(v_running, a_running, cd);
        //Serial.println(alt);
        if (abs(alt-target_alt) < MAX_ERROR){
            still_running = false;
            //Serial.println("converged");
            break;
        }
        if (alt < target_alt){
            max_cd = cd;
        }
        else{
            min_cd = cd;
        }

        if(millis() - t > SEARCH_FOR){ // make sure not to get hung up in this loop
            still_running = true;
            //Serial.println("pass through");
            return deploy_angle;
            break;
        }
    }
    //Serial.println(i);
    return cd;
}

//----------------------------------------------------------
//                  TIMEKEEPER
//----------------------------------------------------------
//all times in seconds

float time; // how long the computer has been powered on, seconds

bool has_taken_off = false; //self explanetory
float liftoff = 0; //time of liftoff
float flight_time = 0; // time since liftoff

float previous_time = 0; // used to calculate delta time
float delta_time = 0; // measured in SECONDS


void timekeeper(){ // flight version of timekeeper
    time = millis() / 1000.0f;
    delta_time = time - previous_time;
    previous_time = time;

    if(has_taken_off){
        flight_time = time - liftoff;
    }
}


//---------------------------------------
//              Sensor Refresh
//---------------------------------------

float altitude = 0.0f;
float previous_altitude = 0.0f;
float previous_vel_time = 0.0f;

float previous_altitudes[5];
//float altitude_times[20];
int altitude_index = 0;
float velocity = 0.0f;
#if HITL // NOT flight software
    void sensor_refresh(){
        
        if(Serial.available()){
            //Serial.print("hi");
            
            //previous_altitudes[altitude_index] = Serial.readStringUntil('\t').toFloat();
            altitude = Serial.readStringUntil(':').toFloat();
            
            // use average of recent altitudes to smooth out sensor noise
            float tot = 0;
            for(int i = 0; i < 5; i ++){
                tot += previous_altitudes[i];
            }
            //altitude = tot/5;  
            
            altitude_index ++;
            if(altitude_index > 5){
                altitude_index = 0;
            }

            //calculate velocity
            if(time - previous_vel_time > VELOCITY_DELTA_T){
                velocity = (altitude - previous_altitude) / (time - previous_vel_time);
                previous_altitude = altitude;
                previous_vel_time = time;
                //Serial.println(altitude);
            }


            while(Serial.available()){
                Serial.read();
            }
            //Serial.println(altitude);
        }
        
    }
#else // flight software
    void sensor_refreah(){
        
    }
#endif

//-----------------------------------------------
//              DATA LOGGER!
//-----------------------------------------------

float log_frequency = 0.05f; // how often data is logged, seconds
float last_log_time = 0.0f;

#if HITL
    void data_log(){
        if(time - last_log_time > log_frequency){
            Serial.println(deploy_angle);

            last_log_time = time;
        }
    }
#else
    void data_log(){
        
    }
#endif

//-----------------------------------------------
//              STATE MACHINE!
//-----------------------------------------------

int state = 0;

//---------------
float began_waiting_for_arming = 0.0f;

void wait_for_arming(){
    if(began_waiting_for_arming == 0.0f){
        began_waiting_for_arming = time;
    }
    if(time - began_waiting_for_arming > ARMING_DELAY){

        //for(int i = 0; i < 20; i ++){     --old, unnecesary code
        //    altitude_times[i] = time;
        //}

        state ++;
        //Serial.println(state);
    }
}


//----------------

float previous_velocity = 0.0f;// used for burnout detection

void wait_for_ignition(){
    if(velocity > TAKEOFF_THRESHOLD){
        previous_velocity = velocity;
        state ++;
        //Serial.println(state);
    }
}

//==============================================

float acceleration_time = 0.0f; // time since last acceleration measurement
float acceleration = 0.0f;

void wait_for_burnout(){
    acceleration_time += delta_time;
    if(acceleration_time > ACCELERATION_SAMPLE_TIME){
        acceleration = (velocity - previous_velocity) / acceleration_time;
        acceleration_time = 0.0f;
        previous_velocity = velocity;
    }
    if(acceleration < BURNOUT_ACCELERATION_THRESHOLD){
        state ++;
        //Serial.println(state);
    }
}

//==============================================



void apogee_correction(){
    float a = -0.00000407762;
    float b = 1.54521;
    float c = -0.0009436;
    //Serial.println(velocity);
    //Serial.println(altitude);
    float cd = -cd_finder(velocity, altitude);
    //Serial.println(cd);
    deploy_angle = pow((cd - c) / a, 1/b);
    //airbrakes.write(deploy_angle);
    //Serial.println(deploy_angle);
    //cd = a * pow(deploy_angle, b) + c
}

//==============================================

void setup(){
    Serial.begin(115200);
    Serial.setTimeout(1);
    Serial.println("testing...");
    /*
    Serial.println(cd_finder(84.0, 43.0));

    float a = -0.00000407762;
    float b = 1.54521;
    float c = -0.0009436;
    float cd = -cd_finder(84.0, 43.0);
    Serial.println(pow((cd - c) / a, 1/b));
    */

    //airbrakes.attach(9);
    delay(1000);
}



void loop(){
    timekeeper();
    sensor_refresh();
    data_log();
    if(state == 0){
        wait_for_arming();        
    }
    else if(state == 1){
        wait_for_ignition();
    }
    else if (state == 2){
        wait_for_burnout();
    }
    else if(state == 3){
        apogee_correction();
    }
    delay(5);
}