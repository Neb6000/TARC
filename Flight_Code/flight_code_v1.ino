#include<math.h>
#include<Servo.h>

//
// ATTENTION !!!!!!!!!
//
#define HITL true //IF FLYING, THIS SHOULD BE FALSE
//
// ATTENTION !!!!!!!!!
//

float TIMESTEP = 0.1; // s
float MASS = 0.650; // kg
float GRAVITY = -9.80; // m/s/s
float TARGET_APOGEE = 254.508; //  m

float OLD_DATA = 1.0f; // seconds, any sensor data older than this is discarded
int ARMING_DELAY = 10.0f; // how long, in seconds, between when the flight computer is turned on and when it should be ready on the launchpad
float TAKEOFF_THRESHOLD = 10.0f; // m/s,  when we go faster than this, computer thinks we have tacken off
float ACCELERATION_SAMPLE_TIME = 0.5f; // seconds, how long between acceleration samples when detecting motor burnout 
float BURNOUT_ACCELERATION_THRESHOLD = -8.0f; // m/s/s, when acceleration is more negative than this, computer thinks motor has burned out

float MIN_CD = 0.000944f;
float MAX_CD = 0.00521f;

float MAX_ERROR = 1.0f; // m

Servo airbrakes;

float apogee_finder(float v, float a, float d){
    float weight = MASS * GRAVITY;
    float cd = d;
    while (v > 0){
        float drag_force = -(v * v) * cd;
        float acceleration = (drag_force + weight)/MASS;
        a += v * TIMESTEP;
        v += acceleration * TIMESTEP;
    }
    return (a);
}


float cd_finder(float velocity, float altitude){
    float max_cd = MAX_CD;
    float min_cd = MIN_CD;
    float cd = 0.0f;
    //int i = 0;
    while(true){
        //i ++;
        cd = (min_cd + max_cd)/2;
        //Serial.println("????");
        float alt = apogee_finder(velocity, altitude, cd);
        //Serial.println(alt);
        if (abs(alt-TARGET_APOGEE) < MAX_ERROR){
            break;
        }
        if (alt < TARGET_APOGEE){
            max_cd = cd;
        }
        else{
            min_cd = cd;
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
float previous_altitudes[20];
float altitude_times[20];
int altitude_index = 0;
float velocity = 0.0f;
#if HITL // NOT flight software
    void sensor_refresh(){
        
        if(Serial.available()){
            //Serial.print("hi");
            previous_altitudes[altitude_index] = Serial.readStringUntil('\t').toFloat();
            altitude = previous_altitudes[altitude_index];
            //Serial.println(altitude);
            altitude_times[altitude_index] = time;
            //int i = altitude_index;
            int i_next;
            if(altitude_index == 19){
                i_next = 0;
            }
            else{
                i_next = altitude_index + 1;
            }
            //use change in altitude from between most current and oldest stored altitude value to calculate velocity
            velocity = (previous_altitudes[altitude_index] - previous_altitudes[i_next]) / (altitude_times[altitude_index] - altitude_times[i_next]);
            
            /*              DEPRECIATED
            int tot = 0;
            int i = altitude_index;
            int i_next;
            int initial = i;
            velocity = 0.0;
            while(!(tot > 0 && i == initial)){ //shitty averaging code
                if(i > 0){
                    i_next = i - 1;
                } else{
                    i_next = 9;
                }
                if(time - altitude_times[i] < OLD_DATA && time - altitude_times[i_next] < OLD_DATA){
                    tot ++;
                    velocity += (previous_altitudes[i] - previous_altitudes[i_next]) / (altitude_times[i] - altitude_times[i_next]);
                }
                i --;
                if(i < 0){
                    i = 9;
                }
            }
            */

            //Serial.println(tot);
            //velocity = velocity / tot; // Average all velocities of recent data points

            altitude_index ++;
            if(altitude_index > 19){
                altitude_index = 0;
            }
            while(Serial.available()){
                Serial.read();
            }
            Serial.println(altitude);
        }
        
    }
#else // flight software
    void sensor_refreah(){
        
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
        for(int i = 0; i < 20; i ++){
            altitude_times[i] = time;
        }

        state ++;
        Serial.println(state);
    }
}


//----------------

float previous_velocity = 0.0f;// used for burnout detection

void wait_for_ignition(){
    if(velocity > TAKEOFF_THRESHOLD){
        previous_velocity = velocity;
        state ++;
        Serial.println(state);
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
        Serial.println(state);
    }
}

//==============================================

float deploy_angle = 0.0f;

void apogee_correction(){
    float a = -0.00000407762;
    float b = 1.54521;
    float c = -0.0009436;
    Serial.println(velocity);
    Serial.println(altitude);
    float cd = -cd_finder(velocity, altitude);
    Serial.println(cd);
    deploy_angle = pow((cd - c) / a, 1/b);
    //airbrakes.write(deploy_angle);
    Serial.println(deploy_angle);
    //cd = a * pow(deploy_angle, b) + c
}

//==============================================

void setup(){
    Serial.begin(230400);
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
}