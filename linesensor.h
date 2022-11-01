#include "Arduino.h"
// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H


// define our 5 sensor pins and our IR emittor pin
#define LS_LEFTEST_PIN A11
#define LS_LEFT_PIN A0
#define LS_CENTRE_PIN A2
#define LS_RIGHT_PIN A3
#define LS_RIGHTEST_PIN A4
#define EMIT_IR_PIN 11
// define the total number of sensors - basically a placeholder for int 5
# define NUMBER_OF_LS_PINS 5




// Class to operate the linesensor(s).
class LineSensor_c {
  public:
  
  // Constructor, must exist.
  LineSensor_c() {
    
  } 


  // define our array of 3 pins, number of entries (3) must match number of sensors we are readng (3). pin_list[3] tells the compiler we will access an integer stored in 3 
  // consecutive locations in memory. index from 0 - 2 for range of 3 ofc.
  int light_sensor_list[NUMBER_OF_LS_PINS] = {LS_LEFTEST_PIN ,LS_LEFT_PIN, LS_CENTRE_PIN, LS_RIGHT_PIN, LS_RIGHTEST_PIN}; 
  int light_sensor; // integer for indexing our light sensor lists. e.g for light sensor in light sensor list.



  // put your setup code here, to run once in void setup.
  void initialise() {

    // Set the five IR sensor pins as inputs
    pinMode(LS_LEFT_PIN, INPUT);
    pinMode(LS_CENTRE_PIN, INPUT);
    pinMode(LS_RIGHT_PIN, INPUT);
    pinMode(LS_LEFTEST_PIN, INPUT);
    pinMode(LS_RIGHTEST_PIN, INPUT);
    enable_IR_LED();
  }



  // function to enable the IR LED.
  void enable_IR_LED(){
    // Set emit pin as an output with HIGH - this is the setting to use it for the line sensors rather than bump detectors
    pinMode(EMIT_IR_PIN, OUTPUT); // OUTPUT means IR emittor pin is ON. INPUT means it is off.
    digitalWrite(EMIT_IR_PIN, HIGH); // HIGH for line sensors, LOW for bumpers. 
  }


  // function to disable the IR LED
  void disable_IR_LED(){
    // Set emit pin as an output with HIGH - this is the setting to use it for the line sensors rather than bump detectors
    pinMode(EMIT_IR_PIN, INPUT); // OUTPUT means IR emittor pin is ON. INPUT means it is off.
    digitalWrite(EMIT_IR_PIN, HIGH); // HIGH for line sensors, LOW for bumpers. 
  }



  // Function to charge the capacitor in each light sensor.
  void charge_Capacitor( int light_sensor_list[], int light_sensor ){
    // temporarily to output and HIGH
    pinMode( light_sensor_list[light_sensor], OUTPUT );
    digitalWrite( light_sensor_list[light_sensor], HIGH );
    // Tiny delay for capacitor to charge.
    delayMicroseconds(10);
    //  Turn input pin back to an input
    pinMode( light_sensor_list[light_sensor], INPUT );
  }



  // Function to activate our light sensors, this contains all the overall timing measures.
  float activate_LS(){

    // start the timer for hte process
    unsigned long whole_process_time_start; 
    whole_process_time_start = micros();

    // run the function to read the sensors
    float e_line_to_main = readLineSensor();

    // mark the end time
    unsigned long whole_process_time_end;
    whole_process_time_end = micros();

    // print elapsed time for whole process
    // Serial.print(", whole process took: ");
    // Serial.print(whole_process_time_end - whole_process_time_start); // whole process time calculation
    // Serial.print("uS");
    // Serial.print("\n");
    // Serial.print("\n");

    return(e_line_to_main);
  }



  // Function to read the line sensors and discern how long they take to discharge.
  float readLineSensor() {



    // for loop to charge all our pins. need to charge 'em up before we look at discharge time.
    for(light_sensor = 0; light_sensor < NUMBER_OF_LS_PINS; light_sensor++){
    
      // charge current capacitor.
      charge_Capacitor(light_sensor_list, light_sensor); 
    }


    // DEFINE OUR VARIABLES
    // Places to store microsecond count
    unsigned long start_time; // start time
    // currently seeing approx 500us on white surface, 2800us on black surface, >3000us suspended in air.
    unsigned long timeout = 3000; // if it takes longer than 3000 microseconds to read all three sensors, time out the while loop (don't get stuck in loop).


    float sensor_read[NUMBER_OF_LS_PINS] = {0, 0, 0, 0, 0}; // list to store our 5 sensor times, one from each sensor. Give initial values of 0 for to allow us to only record earliest discharge value. must be a float as later we want decimals in normalised values.
    int remaining = NUMBER_OF_LS_PINS; // variable to track the number of sensors you have yet to check on.

    start_time = micros(); // Get your start time! Outside while loop as we want the same start time for each sensor!

    while( remaining > 0 ){

      unsigned long current_time = micros(); // get current time

      unsigned long elapsed_time = current_time - start_time; // get elapsed time

      // now we read each of our sensors in a "for loop", increasing by one each time, going through our light sensors list.
      for(light_sensor = 0; light_sensor < NUMBER_OF_LS_PINS; light_sensor++){

        // check if current pin has gone to low
        if( digitalRead( light_sensor_list[light_sensor]) == LOW){
          // Check if this sensor has had a previous value of elapsed time stored before. We only want to store the EARLIEST i.e, value hasn't yet been changed from 0.
          if(sensor_read[light_sensor] == 0){
            // set the discharge time to elapsed time!
            sensor_read[light_sensor] = elapsed_time;
            // let 'em know one less remaining!
            remaining = remaining - 1;
          }
        }
      }

      // check if elapsed time has reached your timeout
      if( elapsed_time >= timeout){
        // if so, set remaining to zero to force end of looping
        remaining = 0;
        // lets us clearly see if timeout has occured. This is defined as a timeout to read all five sensors.
        // Serial.print("THAT'S A TIMEOUT BUDDY! -> (> ");
        // Serial.print(timeout);
        // Serial.print(" )");

      }
    }



    // Print output.
    // Serial.print("line sensors (L to R): " );
    // Serial.print( sensor_read[0] ); // lets us see time for leftest sensor to reach LOW.
    // Serial.print(", ");
    // Serial.print( sensor_read[1] ); // lets us see time for left sensor to reach LOW.
    // Serial.print(", ");
    // Serial.print( sensor_read[2] ); // lets us see time for centre sensor to reach LOW.
    // Serial.print(", ");
    // Serial.print( sensor_read[3] ); // lets us see time for right sensor to reach LOW.
    // Serial.print(", ");
    // Serial.print( sensor_read[4] ); // lets us see time for rightest sensor to reach LOW.
    //Serial.print("\n");

    // Condition to prevent robot thinking line is lost if error is very low from being perfectly lined up on line
    // This is the case if middle sensor discharge time is high but those either side are are low.
    if(sensor_read[2] > 1500 && sensor_read[1] < 1000 && sensor_read[3] < 1000){ 
      // Serial.print("\n");
      // Serial.println(0.09);
      return 0.09; //correct value to identify it as on line plus override the join line function.
    }

    // ************************************************  
    //  Add weighted line following!
    float Sensor_Summation = 0;
    //Serial.println(Sensor_Summation);
    for(light_sensor = 0; light_sensor < NUMBER_OF_LS_PINS; light_sensor++){
      Sensor_Summation += sensor_read[light_sensor];
    }
    //Serial.println(Sensor_Summation);

    // summation is about 2600uS on all white, up to 9000uS on all black.

    // now to normalise,
    // Equation to normalise each part is individual value)/ original summation = percentage value from that sensor.

    for(light_sensor = 0; light_sensor < NUMBER_OF_LS_PINS; light_sensor++){
      sensor_read[light_sensor] = sensor_read[light_sensor]/Sensor_Summation;
    }
    
      // Print output.
      // Serial.print("\n");
      // Serial.print("Normalised line sensors (L to R): " );
      // Serial.print( sensor_read[0] ); // lets us see time for leftest sensor to reach LOW.
      // Serial.print(", ");
      // Serial.print( sensor_read[1] ); // lets us see time for left sensor to reach LOW.
      // Serial.print(", ");
      // Serial.print( sensor_read[2] ); // lets us see time for centre sensor to reach LOW.
      // Serial.print(", ");
      // Serial.print( sensor_read[3] ); // lets us see time for right sensor to reach LOW.
      // Serial.print(", ");
      // Serial.print( sensor_read[4] ); // lets us see time for rightest sensor to reach LOW.
      // Serial.print("\n");



    // Now we have normalised dischage times, where the highest ones indicate the darkest surface a.k.a maximum line. 
    // we update paul's eqn to account for 5 sensors: for left: w_left = L_leftest + L_Left. the 0.5L_Centre always cancels so I'm going to ignore that.

    // error from line = e_line = weighting_left - weighting_right = L_leftest + L_Left - L_rightest - L_right. IN MY CODE WE TAKE LEFT ERROR AS +VE, RIGHT AS -VE.
    float e_line = (sensor_read[0] + sensor_read[1] - sensor_read[3] -sensor_read[4]); // sensor_read[2] is centre, we can ignore that. 
    // Serial.print("\n");
    // Serial.print("e line: ");
    // Serial.println(e_line);


    return(e_line); // return the e_line value when we run our function.
  } 
  };



#endif
