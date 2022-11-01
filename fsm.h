# include "Arduino.h"
// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
# ifndef _FSM_H
# define _FSM_H
# define LED_PIN 13 
# define BUZZER_PIN 6

// Define frequency of updates for our linesensors, PID and motors.
# define LINE_SENSOR_UPDATE 10 // absolute minimum is 8 milliseconds here as that is about the max time the line sensor update function can take
# define PID_UPDATE         20
# define MOTOR_UPDATE       30
# define LOST_LIMIT  1500// Initiate return to start after 1.5 seconds of lost line.



# include "linesensor.h"
# include "kinematics.h"
# include "pid.h"

LineSensor_c linesensors;
Kinematics_c kinematics;

// two instances of PID class, one for each wheel
PID_c speed_pid_left; 
PID_c speed_pid_right;


// Class for our Finite State Machine
class FSM_c {
  public:

    // need these bad boys as whole class variables.
    unsigned long linesensors_ts = 0; 
    unsigned long motor_ts = 0;
    float e_line = 0.0; // initial value for or error from line variable
    int lost_line_count = 0; // we want to know how long the robot has been off line force trigger of return to start.

    //**** PID variables ****
    // I use PID only for straight line control.
    unsigned long pid_ts = 0; // timestamp
    float average_left_speed; // low pass filter of speed, left
    float average_right_speed; // low pass filter of speed, right

    long count_left_last = count_wheel_left; // for difference in encoder counts, set initial values.
    long count_right_last = count_wheel_right;
    float demand = 0.3; // global demand speed variable, encoder counts per ms
    float pwm_left; // our fixed speed values.
    float pwm_right;
    //***********************
    

    // Constructor, must exist.
    FSM_c() {
    }

    // This function calls updates for: Linesensors, PID, and Robot State
    int update_state(int state){

      // Update kinematics (has timing built in, we should refactor the motors and line sensors to contain their own timing too.)
      kinematics.update();

      // Record the time of this execution of loop for coming calucations ( _ts = "time-stamp" )
      unsigned long current_ts;
      current_ts = millis();

      // Run our line sensor update every 100ms (10hz). Tracking time for the line sensor (ls)
      unsigned long elapsed_t;
      elapsed_t = current_ts - linesensors_ts;


      // Linesensore Update
      if( elapsed_t > LINE_SENSOR_UPDATE ) {

        // run our line sensor read function
        e_line = linesensors.activate_LS();
        // record when the line sensors were run
        linesensors_ts = millis();
      }



      // PID Update: calculating speed estimate
      elapsed_t = current_ts - pid_ts; // current time minus prevour time.
      if ( elapsed_t > PID_UPDATE ) { // run every 20 ms

        long diff_left_count;
        long diff_right_count;
        float left_speed;
        float right_speed;

        diff_left_count = count_wheel_left - count_left_last; // get the difference in counts
        count_left_last = count_wheel_left; // update the previous count to current.

        diff_right_count = count_wheel_right - count_right_last; // ditto
        count_right_last = count_wheel_right;

        left_speed = (float)diff_left_count/(float)elapsed_t; // difference in counts devided by elapsed time (counts per ms).
        right_speed = (float)diff_right_count/(float)elapsed_t; 

        // we find an average speed which we weight 70% compared to the current of 30%.
        average_left_speed = (0.7*average_left_speed) + (0.3*left_speed);
        average_right_speed = (0.7*average_right_speed) + (0.3*right_speed);

        pwm_left = speed_pid_left.update(demand, average_left_speed); 
        pwm_right = speed_pid_right.update(demand, average_right_speed);

        pid_ts = millis(); // update the timestamp

        // Serial.print("average left speed: ");
        // Serial.println(average_left_speed);
        // Serial.print("average right speed: ");
        // Serial.println(average_right_speed);
        // Serial.print("demand: ");
        // Serial.println(demand);
      }


      // Robot State Update: Update what the motors are doing.
      elapsed_t = current_ts - motor_ts;
      if( elapsed_t > MOTOR_UPDATE ) {

        // STATE 5: ROBOT REACHED HOME
        if (state == 5){ // if you've reached home, stay home.
          state = 5;
        }

        // STATE 4: CONTINUE RETURN TO START
        else if (state == 4){ // for MVP, we do not attempt return to start.
          state = 4;
        }

        // STATE 3 OR 4: DISCERN WHETHER LINE LOST OR AT TRACK END
        // e.g 50 counts of 30 millis = 1500 so this should trigger on 51st count.
        else if(lost_line_count*MOTOR_UPDATE > LOST_LIMIT){ // check return to start first. If lost line has run consecutively for more than LOST LIMIT then you assume we have lost the line.
          Serial.print(lost_line_count);

          // If travelled less than 300mm in x direction, you haven't yet reached end of track -> turn around.
          if(abs(kinematics.X_pos) < 300){ // if robot has moved less than 30cm along the track.
            float desired_angle = kinematics.Theta + 3.14159; //  as the robot always goes right at the start (turning angle -ve), desired angle will be opp direction so add pi
            speed_pid_left.reset();
            speed_pid_right.reset();
            while(kinematics.Theta < (desired_angle - 0.2)){ // rotate until you reach desired angle, minus constant added to correct error. remove when you can.
              kinematics.update(); // update theta
              motors.setMotorPower(-pwm_left, pwm_right); // rotate toward desired angle - only works as pwm doesn't update in while loop.
            }
            motors.setMotorPower(0, 0); // stop the robot
            lost_line_count = 0; // reset the lost line count
            state = 3; // return to lost line state
          }
 
          else{ // if you've gone far enough then you must be at track end.
          motors.setMotorPower(0, 0); // stop the robot
          delay(2000); // stop for two seconds to show you recognise you are at track end.
          state = 4; //lost line count is reset only when line found so return to start should remain the state indefinitely.
          }
        }

        // STATE 0: INITIAL STATE, JOINING LINE.
        else if(abs(e_line) < 0.07 && state == 0){ // CHANGE ME FOR DIFFERENT SURFACES using a higher threshold than before to see the line to prevent it thinking it finds the line before it does and thus getting in a spin about being lost or finding.
          state = 0; // still joining line
        }
        
        // STATE 1: ERROR HIGH ENOUGH TO INITIATE JOIN LINE
        else if(state == 0){
          state = 1; // join line turn initialise
        }

        // STATE 3: LINE LOST
        else if(abs(e_line) < 0.06) { // if error drops low enough, you've lost the line 
          state = 3; // line lost
          lost_line_count = lost_line_count + 1; // increment by one for each time you run lost line consecutively.
        }

        // STATE 2: ON LINE
        else{ // Assume you are on line
          state = 2;
          lost_line_count = 0; // robot found line up so reset lost line count.
        }
      
        // print the current state
        // Serial.print("CURRENT STATE: ");
        // Serial.print(state);
        // Serial.print("\n");
    
        // Update the time stamp.
        motor_ts = millis();

      }

      return(state);
    }

    // STATE 0: INITIAL STATE.
    void search_for_line(){
      // if state = initial, run this
      // go forward (error is low)
      motors.setMotorPower(pwm_left,pwm_right);
      digitalWrite(LED_PIN, false); // light off means not on the line.
    }

    // STATE 1: JOINING LINE
    int join_line(){
      digitalWrite(LED_PIN, true);
      // line found, turn on the spot to line up.
      while (abs(kinematics.Theta) < 40*(3.14/180)){ // TURN ON THE SPOT TILL at 40 degrees, Allows our robot to get lined up enough for on line arc to take over.
          kinematics.update(); // need updated theta values.
          motors.setMotorPower(25, -20); // turn right for left sensor sees first
      }
      return(2); // once out of while loop, assume on line.
    }

    // STATE 2: ON THE LINE
    void on_line(){
      // if state = on line, run this
      digitalWrite(LED_PIN, true); // error is small enough that we regard motor as "on line" but not so small that it cannot see line at all. Light on indicates this.

      // turn if not lined up, else go straight.
      if ( abs(e_line) >0.20){ // SHARP TURNS - HIGHER ERROR!
        if(e_line > 0){ //+ve = turn left
          motors.setMotorPower( 0 , e_line*(105) ); // MAX ERROR ABOUT 0.5 IN PRACTICE, 0.5 X 100 = 50, MAX TURN SPEED
        }  
        else{
          motors.setMotorPower( e_line*(-105), 0 ); // MAX ERROR ABOUT 0.5 IN PRACTICE, 0.5 X 100 = 50, MAX TURN SPEED
        }
      }
      // GENTLE TURNS - LOWER ERROR! CHANGED TO ARCING RATHER THAN TURNING FOR SMOOTHNESS.
      if ( abs(e_line) >0.1){ // regarding +- 0.1 as seeing the line but not lined up. uneven motor values used to allow for weaker right motor I have noticed.
        if(e_line > 0){ // +ve error, arc left
          motors.setMotorPower( 22 , e_line*(250) ); // arc right proportionally to error. // difference to offset more powerful Left motor while calibration not in.
      }   // above, proportional wheel speed range will be 25 - 75 pwm
        else{ // -ve error, arc right
          motors.setMotorPower(e_line*(-250) , 23 ); // arc right proportionally to error. // difference to offset more powerful Left motor while calibration not in.
      }   // above, proportional wheel speed range will be 25 - 75 pwm
      
      }
      


      else{ // straight on line
      motors.setMotorPower(22,22); // Not using straight line pid as resetting the pid every time the line is lost/found creates jerky line following.
      }
    }
    

    // STATE 3: LINE HAS BEEN LOST
    void lost_line(){
      // if state = lost line, run this
      // go straight until you trigger return to start.
      motors.setMotorPower(pwm_left,pwm_right); // go forward slowly
      digitalWrite(LED_PIN, false); // light off means not on the line.
      bool buzz = false;
      for(int i = 0; i < 2 ; i++){ // buzz if you lose the line!
        if (buzz == false) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzz = true;
        }
        else{
        digitalWrite(BUZZER_PIN, LOW); 
        }       
      }      
    }


    // STATE 4: RETURN TO START
    int return_to_start(){ 
      digitalWrite(LED_PIN, true); 
      kinematics.update(); // need to keep updating position each loop!

      // calculate theta home -> angle to return to start in a straight line.
      if(kinematics.Theta_Home == 0){

        kinematics.Theta_Home = 3.14159 + atan(kinematics.Y_pos/kinematics.X_pos);
        Serial.println(kinematics.Theta_Home);

        if(kinematics.Theta_Home > 3.14){ // Conditions to keep theta home -180 < theta_home < 180
          kinematics.Theta_Home = kinematics.Theta_Home - (2*3.14159);
        }
        if(kinematics.Theta_Home < -3.14){
          kinematics.Theta_Home = kinematics.Theta_Home + (2*3.14159);
        }
      }

      // if theta does not equal theta home (within some error), slowly rotate.
      if (kinematics.Theta > kinematics.Theta_Home + 0.20){ // rotate until theta within about degrees of theta home (0.2 rads) - 12 degree leighway combats the overshoot.
        motors.setMotorPower(21,-21); // slowly rotate in -ve direction
        return(4); // keep spinnin'
      }

      if (kinematics.Theta < kinematics.Theta_Home - 0.20){ // rotate until theta within about degrees of theta home (0.2 rads) - 12 degree leighway combats the overshoot.
      motors.setMotorPower(-21,22); // slowly rotate in +ve direction
      return(4); // keep spinnin'
      }

      else{ // Robot lined up, now head home
        speed_pid_left.reset();
        speed_pid_right.reset();
        unsigned long time_to_home = millis() + 15000; // takes about 15 seconds to get home
        unsigned long current_time = millis();
        while(current_time < time_to_home){
          current_time = millis(); // update current time
          update_state(4); // need to keep updating average speeds!
          kinematics.update(); // need to keep updating position each loop!
          motors.setMotorPower(pwm_left,pwm_right); // go in a straight line.
        }
        Serial.println("HOME!");
        // Once you're home, stop.
        motors.setMotorPower(0, 0);
        return(5); // our home state
      }


      
       
    }

    void home(){
      digitalWrite(LED_PIN, false); 
      motors.setMotorPower(0, 0);
    }

};



#endif