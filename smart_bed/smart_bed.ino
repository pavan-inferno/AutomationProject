//This firmware is to be run on Adafruit Bluefruit M0 board. 
// This device contains following accesories: 2 pumps(pressure, vacuum), 3 3-way solenoids, 96 1-way solenoids, 96 analog pressure sensors, 1 RGB LED, 1 momentary button
// Output devices are all controlled by MC33996 which is an SPI controlled 16 low side switch IC: A0, A1, A2....  A6.
// A0 controls: pump_p, pump_v,V1, V2, V3, R_led, G_led, B_led, Aux1, Aux2... Aux8 (Aux are free ports with switchable 24V power)
// A1,A2..A6 each control 16 solenoids each
// Input devices are all read by ADG731 which is an SPI controlled 32 to 1 switch IC: U1, U2, U3
// U1, U2, U3 each are connected to 32 pressure sensors each
// outputs from U1, U2, U3 are voltage divided to 3.3V from 5V and connected to A0, A1, A2(analog pins on the microcontroller)
// 

#include <SPI.h>              // Requisite for NxpDe... and AbpSensor...
#include "NxpDemuxsw.h"       // Custom class developed 16 Demux switch from Nxp
#include "AnalogADG731.h"

//  #define DEBUG              // Toggles debug messages are shown in serial port monitor 
#define BAUD_RATE 2000000             // Baud rate of serial link
boolean is_binary = false;            // Toggles between binary and ascii for outbound packets (can be changed through serial monitor)
boolean stream = true;                // Toggles whether the data is continuously transmitted (can be changed through serial monitor)
boolean inject_setpressures = false;  // Toggles between transmitting current pressure and set pressure map from the GUI  (can be changed through serial monitor)
boolean inject_checkboxes =false;     // Toggles between transmitting current pressure and checkboxes from the GUI  (can be changed through serial monitor)

//Pin definitions (check PCB to make sure these are right)

//Chip select pins of demux devices A0,A1,A2...A6
#define ACCMUX_PIN 9
#define SOLMUX1_PIN 6
#define SOLMUX2_PIN 5
#define SOLMUX3_PIN 21
#define SOLMUX4_PIN 20
#define SOLMUX5_PIN 17
#define SOLMUX6_PIN 18

//Chip select pins for analog multiplexer devices U1,U2,U3
#define SENSMUX1_PIN 12
#define SENSMUX2_PIN 11
#define SENSMUX3_PIN 10

//Analog pins on the microcontroller connected to devices U1,U2,U3
#define PSENS1_PIN A0
#define PSENS2_PIN A1
#define PSENS3_PIN A2

// Digital pin on the microcontroller connected to the user switch (momentary)
#define RGB_SW_PIN 1

#define MAX_PRESSURE_IN_KPA 20.0        // Maximum permitted pressure in any volume 
#define PACKET_STARTA 'a'               // Starting character of outbound ASCII packet
#define PACKET_STARTB 'b'               // Starting character of outbound binary packet
#define PACKET_STOP 'z'                 // Ending character of outbound packet
#define NUMBER_OF_AIRCELLS 96           // Number of aircells connected to the controller

//create demux objects corresponding to each demux chip A0-A6
NxpDemuxsw accmux(ACCMUX_PIN);
NxpDemuxsw solmux1(SOLMUX1_PIN);
NxpDemuxsw solmux2(SOLMUX2_PIN);
NxpDemuxsw solmux3(SOLMUX3_PIN);
NxpDemuxsw solmux4(SOLMUX4_PIN);
NxpDemuxsw solmux5(SOLMUX5_PIN);
NxpDemuxsw solmux6(SOLMUX6_PIN);

//create a temporary pointer to one of the above variables
NxpDemuxsw* solmux_pntr;

// create ADG731 object for handling analog multiplexers U1,U2,U3
AnalogADG731 sensmux1(SENSMUX1_PIN);
AnalogADG731 sensmux2(SENSMUX2_PIN);
AnalogADG731 sensmux3(SENSMUX3_PIN);

float pressure_val [NUMBER_OF_AIRCELLS];      // Pressure array of aircells in kPa
uint8_t controller_state = 0;                 // State of controller (0 - Idle, 1-48 Busy)
uint8_t num_flags =0;                         // Number of aircells selected by the GUI
float tol = 0.1;                              // Pressure tolerance in kPa
int packet_size=0;

//Proportional valve finer control (only used in single aircell control)
float init_diff;                              // Initial difference in current and set pressure
float factor;                                 // Nondimensional factor which determines the proportional valve opening
float powthresh=2.0;                          // Power threshold which determines the proportional valve opening
float maxthresh=100.0,minthresh=50.0;         // Max and min values which determines the proportional valve opening
int aircell_no = 1;                           //  Aircell no. selected for single pressure regulation  


unsigned long equalize_timeoutMS=5000,regulate_timeoutMS=10000, offload_timeoutMS = 5000;         //  Default timeouts for equalize and regulate operations
unsigned long equalize_start_time,regulate_start_time, offload_start_time;                  //  Starting times in uC clock (millis()) when equalize and regulate operations start
unsigned long single_timeout_MS = 4000;                                 //  Default timeout for single aircell regulation
boolean equalize_command_given = false,regulate_command_given = false, offload_command_given = false;  //  Flags to indicate whether the command has been given and active

// Mapping of aircells to control hardware (After making new connections, make sure you edit only this part) 
//Default mapping which cab be used to reassign numbers after connecting the seat cushion to the controller
//uint8_t real_no[NUMBER_OF_AIRCELLS] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48};
uint8_t real_no[NUMBER_OF_AIRCELLS];


unsigned long time_now, time_prev, t_start;       // Temporary variables to measure time elapsed between successive iterations of loop()
float freq;                                       // Frequency of loop() iteration in Hz
byte *bpointer = (byte *)&freq;                   // Pointer to frequency above

int temp_int;                       //  Temporary variable for extracting preceding integer
float temp_float;                   //  Temporary variable for extracting preceding floating point number
float pv_percent;                   //  Temporary variable for calulating proportional valve opening %

byte readbuffer[250];               //  Byte array for storing arduino serial input buffer
byte ser_byte;                      //  temporary byte of inbound serial buffer
byte mask=1;                        //  8 bit mask used for bitwise decoding operations
int num_of_bytes;                   //  Length of inbound serial packet

boolean aircell_flags[NUMBER_OF_AIRCELLS];    //  Stores the decoded selection of checkboxes from the GUI
boolean regulate_flags[NUMBER_OF_AIRCELLS] ;  //  Stores the checkbox selection when regualtion operation is started
uint16_t percentpressure;                     //  Temporary variable for calculating scaled pressure in the sensor range (0,62.5)
float setpressure[NUMBER_OF_AIRCELLS];        //  Stores the selected setpressure values from the GUI

boolean ascending = true;                     //  Flag determining whether the aircells will be regulated from lowest to highest pressure or otherwise
uint8_t sorted_indices[NUMBER_OF_AIRCELLS];   //  Array with indices sorted according to an ascending or desceding order of setpressure
int abort_signal;
//uint8_t pressure_bias[NUMBER_OF_AIRCELLS] = {0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0};
float pressure_bias[NUMBER_OF_AIRCELLS] = {55,48,51,50,51,50,51,49,51,48,49,48,48,50,52,50,48,50,50,48,50,50,50,50,50,50,49,51,50,51,50,50,51,50,51,50,50,50,50,50,50,48,48,49,49,50,48,50,51,48,49,51,52,50,53,50,49,47,49,49,51,48,50,45,50,50,51,50,50,52,50,49,49,50,50,52,48,54,50,50,51,49,49,50,50,49,49,50,50,45,48,50,51,48,49,50};

// Initialization code
void setup() {
  // Initialize serial port
  Serial.begin(BAUD_RATE);
  // Set pin modes for all accesories
  pinMode(RGB_SW_PIN,INPUT);        // RGBLED button

//  Set default LED color
  setLEDColor('g');
    
//    Turn off all devices explicitly : solenoids, sensors, accessories(pump and valves)
  accmux.switch_all_off();
  solmux1.switch_all_off();
  solmux2.switch_all_off();
  solmux3.switch_all_off();
  solmux4.switch_all_off();
  solmux5.switch_all_off();
  solmux6.switch_all_off();
  

  for (int i = 0; i < NUMBER_OF_AIRCELLS; i++) // The code inside the loop is run for all the 62 bubbles
  {
    sorted_indices[i] = i;                      // Initialize the indices array
    setpressure[i] = 2.0;      // modify the initialized setPressure array
    aircell_flags[i] = true; // Making aircell flag true will skip that aircell while regulation
    real_no[i] = i+1;
  }
  // aircell_flags[0]=false;
  // aircell_flags[1]=false;
  // aircell_flags[2]=false;
  // aircell_flags[3]=false;
  rest_state();
}

void loop()
{     
  if (micros() != 0)
    freq = (1000000.0 / (micros() - time_prev));    // Frequency calculation: reciprocal of time interval
  time_now = micros();  
  
//Check serial buffer for packets starting with 'b' and ending with char(10) or newline character
  if (Serial.available()) // Condition to check if there is data available in the serial port
  {
    ser_byte = Serial.read();
    if (ser_byte == 'b')    // Checking if the first byte is 'b'
    {                                                                  
      num_of_bytes = Serial.readBytesUntil(char(10), readbuffer, 240); // Read bytes from the read buffer until newline character
    }
  }

//  Smaller messages
  if (num_of_bytes<100)
  {
// - (b0 - b5)Test individual devices in accmux by giving only numbers preceded by 'b'
    if (isDigit((char)readbuffer[0]))
    {
      temp_int = String((char*)readbuffer).toInt();
      toggle_acc(temp_int);
    }
// - Single character commands
    switch ((char)readbuffer[0]) 
    {
// - Print all pressures in kPa
      case 'p':
          readpressure();
          printpressure();
          break;
// - Turn fill mode on
      case 'x':
          controlled_fill(100.0);
          break;
// - Turn pump off
      case 'y':
          pump_off();
          break;
// - Toggles the stream
      case 'q':
          stream = !stream;
          break;
// - Toggles the binary mode of packet transmission to the GUI from uC
      case 'm':
          is_binary = !is_binary;
          break;
// - Go to rest state
      case '*':
        rest_state();
        break;
// - Exhaust all volumes naturally
      case '-':
        natural_exhaust();
        break;
// - Exhaust all volumes with pump
      case 'e':
        exhaust_on();
        break;
// - Turn given solenoid number(1-96) on
      case 's': 
        temp_int  = String((char*)readbuffer).substring(1).toInt();
        if (temp_int<97 && temp_int>0){
          Serial.println("Solenoid ON");
          turn_on_solenoid(temp_int);
        }
        break;
// - Turn given solenoid number(1-96) off
      case 'c': //Solenoid Off
        temp_int  = String((char*)readbuffer).substring(1).toInt();
        if (temp_int<NUMBER_OF_AIRCELLS+1 && temp_int>0){
          Serial.println("Solenoid OFF");
          turn_off_solenoid(temp_int);
        }
        break;
// - given aircell number for go to pressure 
      case 'n': 
        temp_int = String((char*)readbuffer).substring(1).toInt();
        if (temp_int>0 && temp_int<NUMBER_OF_AIRCELLS+1)
          aircell_no = temp_int;
        break;
// - given timeout for go to pressure 
      case 't': 
        temp_int = String((char*)readbuffer).substring(1).toInt();
        single_timeout_MS = temp_int;
        break;
// - given tolerance for go to pressure  in psi
      case 'T': 
        temp_float = String((char*)readbuffer).substring(1).toFloat();
        tol = temp_float/6.89;
        break;
// - take 1 air cell to given pressure
      case 'g': 
        temp_float  = String((char*)readbuffer).substring(1).toFloat();
        if (temp_float<20.0 && temp_float>=0.0)
          go_to_pressure(temp_float/6.85,single_timeout_MS);
        break;
// - take all air cells to the pressures given by GUI (one by one)
      case 'd':
        go_to_pressure_sequential();
        break;
// - equalize the aircells given by the GUI
      case 'r':
        temp_int = String((char*)readbuffer).substring(1).toInt();
        if (isDigit((char)readbuffer[1]))
          equalize_timeoutMS = temp_int;
        equalize();          
        break;
// - reset the controller 
      case '!':
        reset();
        break;
// - take aircells selected by the GUI to given pressures 
      case 'f':
        temp_int = String((char*)readbuffer).substring(1).toInt();
        if(temp_int==1)
          ascending = true;
        else
          ascending = false;
        pressure_sorted_regulate(1000);
        break;
// - optimized regulate for all aircells selected
      case 'o':
        // temp_int = String((char*)readbuffer).substring(1).toInt();
        // if(temp_int==1)
        //   ascending = true;
        // else
        //   ascending = false;
        temp_int = String((char*)readbuffer).substring(1).toInt();
        if(temp_int<100000)
          regulate_timeoutMS = temp_int;
        optimized_regulate();
        break;
// - Print aircell flags
      case 'a':
        print_aircell_flags();
        break;
// - Change setpressure and print them
      case 'b':
        temp_float  = String((char*)readbuffer).substring(1).toFloat();
        Serial.println("Set pressures");
        for (int i=0; i<NUMBER_OF_AIRCELLS; i++)
        {
          setpressure[i] = temp_float;
          Serial.print(setpressure[i]);
          Serial.print(',');
        }
        break;
//  Inject selected checkboxes from the GUI into pressure values (debug move)
      case 'z':
        inject_checkboxes = !inject_checkboxes;
        break;
      case '#':
        inject_setpressures = !inject_setpressures;
      case '%':
        temp_int = String((char*)readbuffer).substring(1).toInt();
        if (isDigit((char)readbuffer[1]))
          offload_timeoutMS = temp_int;
        offload();
    }

  }
//  Long packets checked for commdanded pressure 
    /*
   * Check whether the number of bytes in the readbuffer is 132
   * The division of the data is 124 bytes for 62 pressure values (2 bytes each)
   * The last 8 bytes is for the offloading flags
   */
  if (num_of_bytes == 208 && readbuffer[208] == 0 )
  {
    for (int i = 0; i < NUMBER_OF_AIRCELLS; i++)
    {
      percentpressure = (uint16_t)readbuffer[2 * i + 1] << 8; // Rearranging the byte values between the LSB and MSB
      percentpressure |= (uint16_t)readbuffer[2 * i];
      setpressure[i] = percentpressure*14.50/1023.0;
    }
    int bytenum = 192; // For offloading flags, start from byte 124 (i.e. after the 62 pressure values)
    mask = 2;
    for (int counter = 0; counter < 48; counter++) // Iterate through the bit mask
    {
      if (!(readbuffer[bytenum] & mask)) // if bitwise AND resolves to true
        aircell_flags[counter] = true;
      else
        aircell_flags[counter] = false;

      mask <<= 1;    // Left shift the mask value
      if (mask == 0) // The succeeding shift after 1000 0000 --> 0000 0000 (i.e after 8 shifts)
      {
        bytenum++;
        mask = 1;
      }
    }

    bytenum = 200; // For offloading flags, start from byte 124 (i.e. after the 62 pressure values)
    mask = 2;
    for (int counter = 48; counter < NUMBER_OF_AIRCELLS; counter++) // Iterate through the bit mask
    {
      if (!(readbuffer[bytenum] & mask)) // if bitwise AND resolves to true
        aircell_flags[counter] = true;
      else
        aircell_flags[counter] = false;

      mask <<= 1;    // Left shift the mask value
      if (mask == 0) // The succeeding shift after 1000 0000 --> 0000 0000 (i.e after 8 shifts)
      {
        bytenum++;
        mask = 1;
      }
    }
  }
  
  memset(readbuffer, 0, sizeof(readbuffer)); // Clearing the readbuffer using memset
  packet_size = num_of_bytes;
  num_of_bytes = 0;

if(!digitalRead(RGB_SW_PIN))
abort_signal = !abort_signal;
if ((abort_signal)&&(!digitalRead(RGB_SW_PIN)))
{
  equalize_command_given = false;
  offload_command_given = false;
  regulate_command_given = false;
  rest_state();
  controller_state = 0;
  setLEDColor('r');
}
if ((!abort_signal)&&(!digitalRead(RGB_SW_PIN)))
{
  rest_state();
}
Serial.println(abort_signal);
// Equalize subroutine (passive)
  if (equalize_command_given)
  {
    controller_state = 100;
//    If the equalization times out
    if ( millis()-equalize_start_time>equalize_timeoutMS)
      {
      rest_state();
      equalize_command_given = false;
      controller_state = 0;
      }
//      Else do nothing
  }
  // Offload subroutine (passive)
  if (offload_command_given)
  {
    controller_state = 200;
    if (millis()-offload_start_time>offload_timeoutMS)
    {
      rest_state();
      offload_command_given = false;
      controller_state = 0;
    }
  }
  // Regulate subroutine (active)
  if (regulate_command_given)
  {
//    If the regulation times out
    if ( millis()-regulate_start_time>regulate_timeoutMS )
    {
    rest_state();
    regulate_command_given = false;
    controller_state=0;
    }
//    If the regulate operation continues
    else
    {
      if (!regulate_flags[sorted_indices[controller_state-1]])
      {
        if (pressure_val[sorted_indices[controller_state-1]]>setpressure[sorted_indices[controller_state-1]]+tol)
        { 
          controlled_exhaust(100.0);
          // Serial.println("exhausting");
        }
        else if (pressure_val[sorted_indices[controller_state-1]]<setpressure[sorted_indices[controller_state-1]]-tol)
        {
          controlled_fill(100.0);
          // Serial.println("Filling");
        }
        else
        {
          // Serial.print("Captured:");
          // Serial.println(controller_state);
          turn_off_solenoid(sorted_indices[controller_state-1]+1);
          controller_state++;
        }
      }
      else
      {
        controller_state++;
      }
      
    }
    if (controller_state==NUMBER_OF_AIRCELLS+1)
    {
      // Serial.println("Finished operation !");
      regulate_command_given = false;
      rest_state();
      controller_state=0;
    }
  }
  // packet injection to check interpretation of  checkbox values
  if (inject_checkboxes)
    {
      for (int i=0; i<NUMBER_OF_AIRCELLS; i++)
      {
        pressure_val[i] = float(aircell_flags[i]);
      }
    }
  else
  {
    readpressure();
  }
  // Print pressure values if stream is toggled on
  if (stream)
  {
    printpressure();
  }
  time_prev = time_now; 
}
//set the color of RGB LED according to the given character [x- off, b - blue, r -red, g - green, c - cyan(b and g), m - magenta(r+b), y - yellow(r+g), w - (b+r+g)] 
void setLEDColor (char colour)
{
  switch (colour)
  {
    case 'x' :
    accmux.turn_pin_off(5);
    accmux.turn_pin_off(6);
    accmux.turn_pin_off(7);
    break;
    case 'b' :
    accmux.turn_pin_off(5);
    accmux.turn_pin_off(6);
    accmux.turn_pin_on(7);
    break;
    case 'r' :
    accmux.turn_pin_on(5);
    accmux.turn_pin_off(6);
    accmux.turn_pin_off(7);
    break;
    case 'g' :
    accmux.turn_pin_off(5);
    accmux.turn_pin_on(6);
    accmux.turn_pin_off(7);
    break;
    case 'c' :
    accmux.turn_pin_off(5);
    accmux.turn_pin_on(6);
    accmux.turn_pin_on(7);
    break;
    case 'm' :
    accmux.turn_pin_on(5);
    accmux.turn_pin_off(6);
    accmux.turn_pin_on(7);
    break;
    case 'y' :
    accmux.turn_pin_on(5);
    accmux.turn_pin_on(6);
    accmux.turn_pin_off(7);
    break;
    case 'w' :
    accmux.turn_pin_on(5);
    accmux.turn_pin_on(6);
    accmux.turn_pin_on(7);
    break;
  }

}
// Toggle the given device connected to accmux
void toggle_acc (int sel_pin)
{
  if (sel_pin<16 && sel_pin>=0)
    {
      accmux.toggle_pin(sel_pin);
      #ifdef DEBUG
        Serial.print("Toggled: ");
        Serial.print(temp_int);
        Serial.print(" ");
        for( int i=0; i<16;i++)
          Serial.print(accmux.states[i]); 
        Serial.println();
      #endif
    }
}

//convert the given percentage into a integer value in (0-65535)
int mapfloat(float x)
{
  if (x<0.0)
    x=0;
  if (x>100.0)
    x=0;
  return int(6553.55*x);
}
// Turn pump on
void pump_on()
{
  accmux.turn_pin_on(0);
  #ifdef DEBUG
    Serial.println(" Pump On ");
  #endif
}
// Turn pump off
void pump_off()
  {
    accmux.turn_pin_off(0);
    #ifdef DEBUG
    Serial.println(" Pump Off ");
    #endif
  }
// Turn vacuum on
void vacuum_on()
{
  accmux.turn_pin_on(1);
  #ifdef DEBUG
    Serial.println(" Vacuum On ");
  #endif
}
// Turn vacuum off
void vacuum_off()
  {
    accmux.turn_pin_off(1);
    #ifdef DEBUG
    Serial.println(" Vacuum Off ");
    #endif
  }
//Turn valve 1 on
void valve1_on()
 {
    accmux.turn_pin_on(2);
    #ifdef DEBUG
      Serial.print(" Valve1 On");   
    #endif
 }
//Turn valve 1 off
void valve1_off()
 {
    accmux.turn_pin_off(2);
    #ifdef DEBUG
      Serial.print(" Valve1 Off");
    #endif
 }
//Turn valve 2 on
void valve2_on()
 {
    accmux.turn_pin_on(3);
    #ifdef DEBUG
      Serial.print(" Valve2 On");
    #endif
 }
//Turn valve 2 off
void valve2_off()
 {
    accmux.turn_pin_off(3);
    #ifdef DEBUG
      Serial.print(" Valve2 Off");
    #endif
 }
 //Turn valve 3 on
void valve3_on()
 {
    accmux.turn_pin_on(4);
    #ifdef DEBUG
      Serial.print(" Valve3 On");
    #endif
 }
//Turn valve 3 off
void valve3_off()
 {
    accmux.turn_pin_off(4);
    #ifdef DEBUG
      Serial.print(" Valve3 Off");
    #endif
 }
//Turn a given solenoid on(1-96)
//Note all the transformations to match sensor number with corresponding solenoids number
void turn_on_solenoid(uint8_t Solpin)
{
  Solpin = Solpin -1;
//  Solpin = real_no[Solpin]-1;
  if(Solpin<16)
  {
      solmux6.turn_pin_on(Solpin);
  }
  else if(Solpin<32)
    {
        solmux5.turn_pin_on(Solpin-16);
    }
    else if(Solpin<48)
      {
          solmux4.turn_pin_on(Solpin-32);
      }
      else if(Solpin<64)
        {
            solmux3.turn_pin_on(Solpin-48);
        }
        else if(Solpin<80)
          {
              solmux2.turn_pin_on(Solpin-64);
          }
          else if(Solpin<96)
          {
              solmux1.turn_pin_on(16-Solpin+80-1);
          }
      
}
//Turn a given solenoid off(1-96)
void turn_off_solenoid(uint8_t Solpin)
{
  Solpin = Solpin -1;
  if(Solpin<16)
  {
      solmux6.turn_pin_off(Solpin);
  }
  else if(Solpin<32)
    {
        solmux5.turn_pin_off(Solpin-16);
    }
    else if(Solpin<48)
      {
          solmux4.turn_pin_off(Solpin-32);
      }
      else if(Solpin<64)
        {
            solmux3.turn_pin_off(Solpin-48);
        }
        else if(Solpin<80)
          {
              solmux2.turn_pin_off(Solpin-64);
          }
          else if(Solpin<96)
          {
              solmux1.turn_pin_off(16-Solpin+80-1);
          }
}
//Controlled fill for given %
void controlled_fill(float val)
{   
    pump_on();        // turn pump on
    vacuum_off();
    valve1_on();      // setup fill mode
    valve2_on();     // setup fill mode
    valve3_off();

    // analogWrite(A0,mapfloat(val));
    #ifdef DEBUG
      Serial.print("Controlled Fill ");
    #endif
}
//Controlled exhaust for given %
void controlled_exhaust(float val)
{  
    pump_off();                  // turn pump off
    vacuum_on();
    valve1_off();                // setup exhaust mode
    valve2_off();                // setup exhaust mode   
    valve3_on(); 
    #ifdef DEBUG
      Serial.print("Controlled Exhaust ");
    #endif
}
//Put the controller in rest state
void rest_state()
{ 
    pump_off();                  // turn pump off
    vacuum_off();
    valve1_off();                // setup rest mode
    valve2_off();                // setup rest mode
    valve3_off();                // setup rest mode

//    Turn off all solenoids
    for(int i=1;i<=NUMBER_OF_AIRCELLS;i++)
    {
    turn_off_solenoid(i);
    }
    #ifdef DEBUG
      Serial.print("Rest");
    #endif
    setLEDColor('g');
}
//Exhaust all volumes
void exhaust_on()
 {
  solmux1.switch_all_on();
  solmux2.switch_all_on();
  solmux3.switch_all_on();
  solmux4.switch_all_on();
  solmux5.switch_all_on();
  solmux6.switch_all_on();
  controlled_exhaust(100.0);
  #ifdef DEBUG
    Serial.println("Exhaust On");   
  #endif
 }

//Natural exhaust all volumes without pumps
void natural_exhaust()
 {
  solmux1.switch_all_on();
  solmux2.switch_all_on();
  solmux3.switch_all_on();
  solmux4.switch_all_on();
  solmux5.switch_all_on();
  solmux6.switch_all_on();
  pump_off();                  // turn pump off
  vacuum_off();
  valve1_off();                // setup exhaust mode
  valve2_off();                // setup exhaust mode   
  valve3_off(); 
  #ifdef DEBUG
    Serial.println("Natural Exhaust On");   
  #endif
 }
//Read the internal pressures of all volumes
void readpressure()
{
  for(int i=0;i<NUMBER_OF_AIRCELLS;i++)
     {
      if (inject_setpressures)
        pressure_val[i] = setpressure[i];
      else
      {
        if (i<32){
          sensmux1.select(i+1);
//          delay(10);
          pressure_val[i] = 0.019261*(analogRead(PSENS1_PIN)-pressure_bias[i]);
        }else if(i<64){
          sensmux2.select(i-32+1);
//          delay(10);
          pressure_val[i] = 0.019261*(analogRead(PSENS2_PIN)-pressure_bias[i]);
        }
        else{
          sensmux3.select(i-64+1);          
//          delay(10);
          pressure_val[i] = 0.019261*(analogRead(PSENS3_PIN)-pressure_bias[i]);
        }
      }
     }
}
//Prints the current internal pressure map to serial port
void printpressure()
{
  if (is_binary)
  {
    Serial.print(PACKET_STARTB);            // The first character is 'b'
    Serial.write((byte *)pressure_val,sizeof(pressure_val));  // The 48 pressure values are displayed as 2-byte values
    Serial.write(bpointer, sizeof(float)); // The frequency (as stored in bpointer) is a float value (4 bytes)
    Serial.write(controller_state);
    Serial.println(PACKET_STOP); // The last character is 'z'
  }
  else{
    Serial.print(PACKET_STARTA); // The first character is 'a'
    for (int i=0; i<NUMBER_OF_AIRCELLS; i++)
    {
      Serial.print(pressure_val[i]);
      Serial.print(',');
    }
    Serial.print((int)freq); // Frequency is displayed after all the pressure values
    Serial.print(',');
    // Serial.print(packet_size);
    Serial.print(controller_state);
    Serial.println(PACKET_STOP); // The last character is 'z'
//    Serial.println();
  }
}
//only for testing on one volume say aircell 1 (by default)
//use n command to change it to whatever aircell no
void go_to_pressure(float givenpressure, uint32_t timeout)
{
  controller_state = aircell_no;
  readpressure();
  init_diff =  abs(givenpressure-pressure_val[aircell_no-1]);
  uint32_t time_started = millis(); 
  while((abs(givenpressure-pressure_val[aircell_no-1])>tol) && (millis()-time_started < timeout))
  {
     readpressure();
     turn_on_solenoid(aircell_no);
    factor = abs(givenpressure-pressure_val[aircell_no-1])/init_diff;
    if (factor >1)
      factor = 1.0;
    if (factor <0)
      factor = 0.0;
    factor = pow(factor,powthresh);
    // pv_percent = maxthresh*factor + minthresh*(1-factor);
    pv_percent = maxthresh;
    if (pressure_val[aircell_no-1] > givenpressure +tol)
    {
      controlled_exhaust(pv_percent);
    }
    if (pressure_val[aircell_no-1] < givenpressure -tol)
    {
      controlled_fill(pv_percent);
    }
    printpressure();
  }
  rest_state();
  controller_state = 0;
}
//   Sorting the indices in the bottom up approach
void sort_indices_bottom_up(float a[], uint8_t size, uint8_t indices[])
{
  for (int i = 0; i < (size - 1); i++)
  {
    for (int o = 0; o < (size - (i + 1)); o++)
    {
      if (a[indices[o]] > a[indices[o + 1]])
      {
        uint8_t t = indices[o];
        indices[o] = indices[o + 1];
        indices[o + 1] = t;
      }
    }
  }
}
//   Sorting the indices in the top down approach
void sort_indices_top_down(float a[], uint8_t size, uint8_t indices[])
{
  for (int i = 0; i < (size - 1); i++)
  {
    for (int o = 0; o < (size - (i + 1)); o++)
    {
      if (a[indices[o]] < a[indices[o + 1]])
      {
        uint8_t t = indices[o];
        indices[o] = indices[o + 1];
        indices[o + 1] = t;
      }
    }
  }
}
// Equalize the pressures in given aircells(given by GUI)
void equalize()
{
  rest_state();
//  Open all selected bubbles to the manifold
  for (int i = 0; i < NUMBER_OF_AIRCELLS; i++)                                 // For all the 62 bubbles
  {
    if (!aircell_flags[i])
    {
      turn_on_solenoid(i+1);
    }
  }
// Record when the operation started
  equalize_start_time = millis();
  equalize_command_given = true;
  setLEDColor('y');

}
// Reset the microcontroller state
void reset()
{
  rest_state();
  go_to_cmdmode();
}
//Go to app mode
void go_to_appmode()
{
  is_binary = true;
  stream = true;
}
//Go to cmd mode
void go_to_cmdmode()
{
  is_binary = false;
  stream = false;
}
//regulates to given pressure map either in ascending or descending order 
void pressure_sorted_regulate(long int timeoutinMS)
{
  #ifdef DEBUG
    Serial.println("Sorted regulation started");
  #endif
  if (ascending)
    sort_indices_bottom_up(setpressure, NUMBER_OF_AIRCELLS, sorted_indices); // sort the indices of the pressures in bottom up approach
  else
    sort_indices_top_down(setpressure, NUMBER_OF_AIRCELLS, sorted_indices); // sort the indices of the pressures in top down approach

  long int time_started = millis();
  for (int i = 0; i < NUMBER_OF_AIRCELLS; i++)
  {
    if (!aircell_flags[sorted_indices[i]])
    {
      controller_state = sorted_indices[i] + 1;
      readpressure();
      #ifdef DEBUG
        Serial.print("Regulating with aircell:");
        Serial.print(sorted_indices[i] + 1);
      #endif 
      while ((abs(pressure_val[sorted_indices[i]] - setpressure[sorted_indices[i]]) > tol) && (millis()-time_started<timeoutinMS))
      {
        turn_on_solenoid(sorted_indices[i]+1);
        if (pressure_val[sorted_indices[i]] > setpressure[sorted_indices[i]])
          controlled_exhaust(100.0);
        else
          controlled_fill(100.0);
        readpressure();
        printpressure();
      }
      #ifdef DEBUG
        Serial.println("Done");
      #endif 
      turn_off_solenoid(sorted_indices[i]+1);
    }
  }
  rest_state();
  controller_state = 0;
}
//prints aircell_flags
void print_aircell_flags()
{
  for(int i=0; i<NUMBER_OF_AIRCELLS; i++)
  {
    if (aircell_flags[i])
      Serial.print('1');
    else
      Serial.print('0');
  }
  Serial.println();
}
//regulates to given pressure map one by one
void go_to_pressure_sequential()
{
  for (int i=0; i<NUMBER_OF_AIRCELLS; i++)
  {
    if(!aircell_flags[i])
    {
      aircell_no = i+1;
      go_to_pressure(setpressure[i],single_timeout_MS);
    }
  }
}
// bottom up or top down regulation initialization
void optimized_regulate()
{
  if (ascending)
    sort_indices_bottom_up(setpressure, NUMBER_OF_AIRCELLS, sorted_indices); // sort the indices of the pressures in bottom up approach
  else
    sort_indices_top_down(setpressure, NUMBER_OF_AIRCELLS, sorted_indices); // sort the indices of the pressures in top down approach
  num_flags = 1;
  // Open all solenoids
  for(int i=0; i<NUMBER_OF_AIRCELLS; i++)
  {
    regulate_flags[i] = aircell_flags[i];
    if (!regulate_flags[sorted_indices[i]])
    {
      num_flags++;
      turn_on_solenoid(sorted_indices[i]+1);
    }
  }
  setLEDColor('b');
  regulate_start_time = millis();
  regulate_command_given = true;
  controller_state = 1;
}

void offload()
{
  for (int i=0; i<NUMBER_OF_AIRCELLS; i++)
  {
    if(!aircell_flags[i])
    {
      turn_on_solenoid(i+1);
    }
    else
    {
      turn_off_solenoid(i+1);
    }
  }
  controlled_exhaust(100.0);
  offload_start_time = millis();
  offload_command_given = true;
}
