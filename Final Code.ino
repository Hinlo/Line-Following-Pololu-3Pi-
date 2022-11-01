# define LED_PIN 13  // Pin to activate the orange LED
# define BUZZER_PIN 6 // Pin to activate the buzzer
// Define our states 
# define STATE_INITIAL 0
# define STATE_JOIN_LINE 1
# define STATE_ON_LINE 2        
# define STATE_LOST_LINE 3
# define STATE_RETURN_TO_START 4 
# define STATE_HOME 5     

// Our FSM and Kinematics classes call the remaining classes so don't need to be called again here.
# include "fsm.h"
FSM_c fsm;

int state; // global state int to set under differing conditions.
int previous_state; // used to check if state has changed for PID reset


void setup() {
  // put your setup code here, to run once
  // Start Serial, send debug text
  Serial.begin(9600);
  delay(5000);
  Serial.println("***RESET***");

  // Set LED and buzzer pin as an output
  pinMode( LED_PIN, OUTPUT );
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialise linesensors GPIO
  linesensors.initialise();

  // initialise encoders before motors.
  setupEncoder0(); 
  setupEncoder1();

  // Initialise motor GPIO
  motors.initialise();

  state = 0; // set state to join line.

  // pid set up
  // setup k_proportional, k_integral , k_differential - these are the system gains used to manipulate the error signal e_line.
  speed_pid_left.initialise(100, 0.5, -100);
  speed_pid_right.initialise(100, 0.5, -100);

  // reset PID before you use it.
  speed_pid_left.reset();
  speed_pid_right.reset();
}

void loop(){ 
  
  // Get our state value from the finite state machine.
  state = fsm.update_state(state);
  // If the state has changed to "on line", reset the PID instances. State only resets after coming back from lost line or search for line.
  if(state != previous_state && state !=2){
    speed_pid_left.reset(); // reset the pid controllers
    speed_pid_right.reset();
    previous_state = state; // update previous state
  }


  // Choose behaviour based on current state.
  if(state == STATE_INITIAL){
    fsm.search_for_line();
  }

  else if(state == STATE_JOIN_LINE){
    state = fsm.join_line();
  }

  else if(state == STATE_ON_LINE){
    fsm.on_line();
  }

  else if(state == STATE_LOST_LINE){
    fsm.lost_line();
  }

  else if(state == STATE_RETURN_TO_START){
    state = fsm.return_to_start();
  }
  else if(state == STATE_HOME){
    fsm.home();
  }


}






