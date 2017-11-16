/*
 *
 *  Developed by Hassan Albalawi (April 2016), Hassan@ksa.us.com
 *
 *  >>>> THIS CODE USED TO STREAM OpenBCI V3_32 DATA TO iPhone App "Alpha Waves" <<<<
 *
 *   *** WHAT DOES IT DO ***
 *
 *  This sketch will stream data from Channel #1 only,
 *
 *
 *
 *
 *   *** DEPENDENCIES ***
 *   In order to make it work, you have to setup the Rfdunio with the attached skecth
 *   (refer to OpenBCI documentary to know how to upload a code to Rfdunio on the board)
 *
 *
 *   *** Credits ***
 *
 *  This sketch uses Biquad filter library, which was originally devloped by:
 *  http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/
 *  Then, it was modified by Chip Audette to be used on OpenBCI 8-bit board.
 *  Then, I modified it to be used on OpenBCI 32-bit Board
 *
 *  The method of calucalting the Alpha power was inspired by the code developed by Chip Aduette here:
 *  https://github.com/chipaudette/EEGHacker/blob/master/Arduino/OBCI_V2_AlphaDetector/
 *
 *  The core of this code is a modified version of the OpenBCI 32-bit original code
 */


#include <DSPI.h>
#include <EEPROM.h>
#include "OpenBCI_32_BLE.h"

//------------------------------------------------------------------------------
//  << OpenBCI BUSINESS >>
boolean is_running = false;    // this flag is set in serialEvent on reciept of ascii prompt
OpenBCI_32_BLE OBCI; //Uses SPI bus and pins to say data is ready.

// these are used to change individual channel settings from PC
char currentChannelToSet;    // keep track of what channel we're loading settings for
boolean getChannelSettings = false; // used to receive channel settings command
int channelSettingsCounter; // used to retrieve channel settings from serial port
int leadOffSettingsCounter;
boolean getLeadOffSettings = false;

// these are all subject to the radio requirements: 31byte max packet length (maxPacketLength - 1 for packet checkSum)
#define OUTPUT_NOTHING (0)  // quiet
#define OUTPUT_BINARY (1)  // normal transfer mode
#define OUTPUT_BINARY_SYNTHETIC (2)  // needs portage
int outputType;

//------------------------------------------------------------------------------
//  << LIS3DH Accelerometer Business >>
//  LIS3DH_SS on pin 5 defined in OpenBCI library
volatile boolean auxAvailable = false;
volatile boolean addAccel = false;
boolean useAccelOnly = false;
//------------------------------------------------------------------------------
//  << PUT FILTER STUFF HERE >>
#define MAX_N_CHANNELS (8)   //how many channels are available in hardware
#define N_EEG_CHANNELS (1)
//Design filters  (This BIQUAD class requires ~6K of program space!  Ouch.)
//For frequency response of these filters: http://www.earlevel.com/main/2010/12/20/biquad-calculator/
#include "Biquad.h"   //modified from this source code:  http://www.earlevel.com/main/2012/11/26/biquad-c-source-code/

// Stop DC filter
#define SAMPLE_RATE_HZ (250.0)  //default setting for OpenBCI
#define FILTER_Q (0.5)        //critically damped is 0.707 (Butterworth)
#define PEAK_GAIN_DB (0.0) //we don't want any gain in the passband
#define HP_CUTOFF_HZ (0.5)  //set the desired cutoff for the highpass filter
Biquad stopDC_filter1(bq_type_highpass, HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad stopDC_filter2(bq_type_highpass, HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, PEAK_GAIN_DB); //one for each channel because the object 
Biquad stopDC_filter3(bq_type_highpass, HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, PEAK_GAIN_DB); //one for each channel because the object 
Biquad stopDC_filter4(bq_type_highpass, HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, PEAK_GAIN_DB); //one for each channel because the object
Biquad stopDC_filter5(bq_type_highpass, HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, PEAK_GAIN_DB); //one for each channel because the object 
Biquad stopDC_filter6(bq_type_highpass, HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, PEAK_GAIN_DB); //one for each channel because the object 
Biquad stopDC_filter7(bq_type_highpass, HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, PEAK_GAIN_DB); //one for each channel because the object 
Biquad stopDC_filter8(bq_type_highpass, HP_CUTOFF_HZ / SAMPLE_RATE_HZ, FILTER_Q, PEAK_GAIN_DB); //one for each channel because the object 

// Notch Filter
#define NOTCH_FREQ_HZ (50.0)      // Make sure you select the right power line frequency; set it to 60Hz if you're in the U.S.
#define NOTCH_Q (4.0)              //pretty sharp notch
Biquad notch_filter1(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter2(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter3(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter4(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter5(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter6(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter7(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter8(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter9(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter10(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter11(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter12(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter13(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter14(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter15(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad notch_filter16(bq_type_notch, NOTCH_FREQ_HZ / SAMPLE_RATE_HZ, NOTCH_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states

// Design signal detection filter

//////////////////////////// Start Power Filters //////////////////////////////////
#define BP_Q (2.0f) //somewhat steep slope

// ALPHA High Power (10 - 11.75Hz)
#define AHP_FREQ_HZ (10.0f)  //focus on High Alpha waves
Biquad AHP_bandpass_filter1(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter2(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter3(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter4(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter5(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter6(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter7(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter8(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter9(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter10(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter11(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter12(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter13(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter14(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter15(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states
Biquad AHP_bandpass_filter16(bq_type_bandpass, AHP_FREQ_HZ / SAMPLE_RATE_HZ, BP_Q, PEAK_GAIN_DB); //one for each channel because the object maintains the filter states

Biquad *AHP_bp1,*AHP_bp2,*AHP_bp3,*AHP_bp4,*AHP_bp5,*AHP_bp6,*AHP_bp7,*AHP_bp8,*AHP_bp9,*AHP_bp10,*AHP_bp11,*AHP_bp12,*AHP_bp13,*AHP_bp14,*AHP_bp15,*AHP_bp16;
////////////////////////////// End Power Filters ////////////////////

#define MICROVOLTS_PER_COUNT (0.02235174f)  //Nov 10,2013...assumes gain of 24, includes mystery factor of 2... = 4.5/24/(2^24) *  2


//define some output pins
int BUZZER_OUTPUT_PIN = 13;
int Red_OUTPUT_PIN  = 12;

boolean AlphaDetector = true;  //enable or disable as you'd like..
boolean NoBase = false; // If you would like to establish a base instead of using the standard based (was set experimently)
boolean cancelNoise = true; // Do you want to cancel noise by Beta-wave method?
boolean DetectNoise = false; // Send error code if you recieved a lot of noises
boolean UseStandardBase = true; // Used if you don't establish a base
boolean printAlpha = false; // used to print Alpha value instead of counter value when true
//------------------------------------------------------------------------------

int LED = 11;  // blue LED alias
int PGCpin = 12;  // PGC pin goes high when PIC is in bootloader mode
//------------------------------------------------------------------------------

void setup(void) {

  Serial0.begin(9600);  // using hardware uart number 0
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);   // blue LED
  pinMode(PGCpin, OUTPUT);
  digitalWrite(PGCpin, LOW); // used to tell RFduino if we are in bootloader mode

  startFromScratch();

  //setup other pins

  // RGB Pins
  pinMode(Red_OUTPUT_PIN, OUTPUT);
  pinMode(BUZZER_OUTPUT_PIN, OUTPUT);

  // on then off to make sure they are working properly
  digitalWrite(Red_OUTPUT_PIN, HIGH);
  delay(500);
  digitalWrite(Red_OUTPUT_PIN, LOW);
  digitalWrite(BUZZER_OUTPUT_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_OUTPUT_PIN, LOW);
}

int prev_LED_val = 0, LED_val = 0;

void loop() {


  if (is_running) {

    while (!(OBCI.isDataAvailable())) {
    }   // wait for DRDY pin...

    OBCI.updateChannelData(); // get the fresh ADS results

    if (AlphaDetector)  Run_AlphaDetector();
  }
  eventSerial();
}


// some variables to help find 'burger protocol' commands
int plusCounter = 0;
char testChar;
unsigned long commandTimer;

void eventSerial() {
  if (Serial0.available() > 0) {
    char inChar = (char)Serial0.read();

    getCommand(inChar);
  }
}


void getCommand(char token) {
  switch (token) {
    //TURN CHANNELS ON/OFF COMMANDS
    case '1':
      changeChannelState_maintainRunningState(1, DEACTIVATE);
      break;
    case '2':
      changeChannelState_maintainRunningState(2, DEACTIVATE);
      break;
    case '3':
      changeChannelState_maintainRunningState(3, DEACTIVATE);
      break;
    case '4':
      changeChannelState_maintainRunningState(4, DEACTIVATE);
      break;
    case '5':
      changeChannelState_maintainRunningState(5, DEACTIVATE);
      break;
    case '6':
      changeChannelState_maintainRunningState(6, DEACTIVATE);
      break;
    case '7':
      changeChannelState_maintainRunningState(7, DEACTIVATE);
      break;
    case '8':
      changeChannelState_maintainRunningState(8, DEACTIVATE);
      break;
    case '!':
      changeChannelState_maintainRunningState(1, ACTIVATE);
      break;
    case '@':
      changeChannelState_maintainRunningState(2, ACTIVATE);
      break;
    case '#':
      changeChannelState_maintainRunningState(3, ACTIVATE);
      break;
    case '$':
      changeChannelState_maintainRunningState(4, ACTIVATE);
      break;
    case '%':
      changeChannelState_maintainRunningState(5, ACTIVATE);
      break;
    case '^':
      changeChannelState_maintainRunningState(6, ACTIVATE);
      break;
    case '&':
      changeChannelState_maintainRunningState(7, ACTIVATE);
      break;
    case '*':
      changeChannelState_maintainRunningState(8, ACTIVATE);
      break;
    case '0':
    case '-':
    case '=':
    case 'p':
    case '[':
    case ']':
    case'A':
      DetectNoise = false;
      //    Serial0.println("DetectNoise is False");
      break;
    case'S':
      DetectNoise = true;
      //    Serial0.println("DetectNoise is True");
      break;
    case'F':
      UseStandardBase = false;
      //    Serial0.println("UseStandardBase is False");
      break;
    case'G':
      UseStandardBase = true;
      //    Serial0.println("UseStandardBase is True");
      break;
    case'H':
      cancelNoise = false;
      //    Serial0.println("cancelNoise is False");
      break;
    case'J':
      cancelNoise = true;
      //    Serial0.println("cancelNoise is True");
      break;
    case'K':
    case'L':
    case 'a':
    case 'h':
    case 'j':
    // CHANNEL SETTING COMMANDS
    case 'x':
    case 'X':
    case 'd':
      // used to test the connection
      digitalWrite(LED, HIGH);
      break;
    case 'D':
    case 'c':
      // used to test the connection
      digitalWrite(LED, LOW);
      break;
    case 'C':
    case 'z':
    case 'Z':
    // STREAM DATA AND FILTER COMMANDS
    case 'v':
      //      startFromScratch();
      break;
    case 'n':
    case 'N':
    case 'b':  // stream data
      startRunning(OUTPUT_BINARY);
      break;
    case 's':  // stop streaming data
      OBCI.disable_accel();
      stopRunning();
      break;
    case 'f':
      printAlpha = true;
      break;
    case 'g':
      printAlpha = false;
      break;
    case '?':
    default:
      break;
  }
}// end of getCommand

boolean stopRunning(void) {
  if (is_running == true) {
    OBCI.stopStreaming();                    // stop the data acquisition  //
    is_running = false;
  }
  return is_running;
}

boolean startRunning(int OUT_TYPE) {
  if (is_running == false) {
    outputType = OUT_TYPE;
    OBCI.startStreaming();
    is_running = true;
  }
  return is_running;
}

int changeChannelState_maintainRunningState(int chan, int start)
{
  boolean is_running_when_called = is_running;
  int cur_outputType = outputType;

  //must stop running to change channel settings
  stopRunning();
  if (start == true) {
    //    Serial0.print("Activating channel ");
    //    Serial0.println(chan);
    OBCI.activateChannel(chan);
  }
  else {
    //    Serial0.print("Deactivating channel ");
    //    Serial0.println(chan);
    OBCI.deactivateChannel(chan);
  }
  //restart, if it was running before
  if (is_running_when_called == true) {
    startRunning(cur_outputType);
  }
}
//
void startFromScratch() {
  delay(1000);
  //  Serial0.print("OpenBCI V3 32bit Board\nSetting ADS1299 Channel Values\n");
  OBCI.useAccel = false;  // option to add accelerometer dat to stream
  OBCI.useAux = false;    // option to add user data to stream not implimented yet
  OBCI.initialize();
  OBCI.configureLeadOffDetection(LOFF_MAG_6NA, LOFF_FREQ_31p2HZ);
  //  Serial0.print("ADS1299 Device ID: 0x");
  //  Serial0.println(OBCI.ADS_getDeviceID(),HEX);
  //  Serial0.print("LIS3DH Device ID: 0x");
  //  Serial0.println(OBCI.LIS3DH_getDeviceID(),HEX);
  //  sendEOT();
}

// DO FILTER STUFF HERE IF YOU LIKE
int ind_count = 1;
float AHP_tmp1[250],AHP_tmp2[250],AHP_tmp3[250],AHP_tmp4[250],AHP_tmp5[250],AHP_tmp6[250],AHP_tmp7[250],AHP_tmp8[250];
float Diff_H1 = 0,Diff_H2 = 0,Diff_H3 = 0,Diff_H4 = 0,Diff_H5 = 0,Diff_H6 = 0,Diff_H7 = 0,Diff_H8 = 0;
float AHP1 = 0,AHP2 = 0,AHP3 = 0,AHP4 = 0,AHP5 = 0,AHP6 = 0,AHP7 = 0,AHP8 = 0;

void Run_AlphaDetector(int cn) {
  float val, AHP_val;

  val = (float) OBCI.channelDataInt[0];
  val = stopDC_filter1.process(val);
  val = notch_filter1.process(val);     //apply 50Hz notch filter
  val = notch_filter2.process(val);
  OBCI.channelDataInt[0] = (long) val;
  float val_common = val;
  AHP_val = val_common;
  AHP_bp1 = &AHP_bandpass_filter1;
  AHP_bp2 = &AHP_bandpass_filter2;
  AHP_val = AHP_bp1->process(AHP_val);    //apply bandpass filter
  AHP_val = AHP_bp2->process(AHP_val);    //do it again to make it even tighter
  AHP_tmp1[ind_count] = AHP_val * AHP_val;

  
  val = (float) OBCI.channelDataInt[1];
  val = stopDC_filter2.process(val);
  val = notch_filter3.process(val);     //apply 50Hz notch filter
  val = notch_filter4.process(val);
  OBCI.channelDataInt[1] = (long) val;
  float val_common = val;    
  AHP_val = val_common;
  AHP_bp1 = &AHP_bandpass_filter3;
  AHP_bp2 = &AHP_bandpass_filter4;
  AHP_val = AHP_bp3->process(AHP_val);    //apply bandpass filter
  AHP_val = AHP_bp4->process(AHP_val);    //do it again to make it even tighter
  AHP_tmp2[ind_count] = AHP_val * AHP_val;
  
  
  val = (float) OBCI.channelDataInt[2];
  val = stopDC_filter3.process(val);
  val = notch_filter5.process(val);     //apply 50Hz notch filter
  val = notch_filter6.process(val);
  OBCI.channelDataInt[2] = (long) val;
  float val_common = val;
  AHP_val = val_common;
  AHP_bp1 = &AHP_bandpass_filter5;
  AHP_bp2 = &AHP_bandpass_filter6;
  AHP_val = AHP_bp5->process(AHP_val);    //apply bandpass filter
  AHP_val = AHP_bp6->process(AHP_val);    //do it again to make it even tighter
  AHP_tmp3[ind_count] = AHP_val * AHP_val;
  
  
  val = (float) OBCI.channelDataInt[3];
  val = stopDC_filter4.process(val);
  val = notch_filter7.process(val);     //apply 50Hz notch filter
  val = notch_filter8.process(val);
  OBCI.channelDataInt[3] = (long) val;
  float val_common = val;
  AHP_val = val_common;
  AHP_bp1 = &AHP_bandpass_filter7;
  AHP_bp2 = &AHP_bandpass_filter8;
  AHP_val = AHP_bp7->process(AHP_val);    //apply bandpass filter
  AHP_val = AHP_bp8->process(AHP_val);    //do it again to make it even tighter   
  AHP_tmp4[ind_count] = AHP_val * AHP_val;
  
  
  val = (float) OBCI.channelDataInt[4];
  val = stopDC_filter5.process(val);
  val = notch_filter9.process(val);     //apply 50Hz notch filter
  val = notch_filter10.process(val);
  OBCI.channelDataInt[5] = (long) val;
  float val_common = val;
  AHP_val = val_common;
  AHP_bp1 = &AHP_bandpass_filter9;
  AHP_bp2 = &AHP_bandpass_filter10;
  AHP_val = AHP_bp9->process(AHP_val);    //apply bandpass filter
  AHP_val = AHP_bp10->process(AHP_val);    //do it again to make it even tighter
  AHP_tmp5[ind_count] = AHP_val * AHP_val;
  
  
  val = (float) OBCI.channelDataInt[5];
  val = stopDC_filter6.process(val);
  val = notch_filter11.process(val);     //apply 50Hz notch filter
  val = notch_filter12.process(val);
  OBCI.channelDataInt[6] = (long) val;
  float val_common = val;
  AHP_val = val_common;
  AHP_bp1 = &AHP_bandpass_filter11;
  AHP_bp2 = &AHP_bandpass_filter12;
  AHP_val = AHP_bp11->process(AHP_val);    //apply bandpass filter
  AHP_val = AHP_bp12->process(AHP_val);    //do it again to make it even tighter
  AHP_tmp6[ind_count] = AHP_val * AHP_val;
  
  
  val = (float) OBCI.channelDataInt[6];
  val = stopDC_filter7.process(val);
  val = notch_filter13.process(val);     //apply 50Hz notch filter
  val = notch_filter14.process(val);
  OBCI.channelDataInt[6] = (long) val;
  float val_common = val;
  AHP_val = val_common;
  AHP_bp1 = &AHP_bandpass_filter13;
  AHP_bp2 = &AHP_bandpass_filter14;
  AHP_val = AHP_bp13->process(AHP_val);    //apply bandpass filter
  AHP_val = AHP_bp14->process(AHP_val);    //do it again to make it even tighter  
  AHP_tmp7[ind_count] = AHP_val * AHP_val;
  
  
  val = (float) OBCI.channelDataInt[7];
  val = stopDC_filter8.process(val);
  val = notch_filter15.process(val);     //apply 50Hz notch filter
  val = notch_filter16.process(val);
  OBCI.channelDataInt[7] = (long) val;
  float val_common = val;
  AHP_val = val_common;
  AHP_bp1 = &AHP_bandpass_filter15;
  AHP_bp2 = &AHP_bandpass_filter16;
  AHP_val = AHP_bp15->process(AHP_val);    //apply bandpass filter
  AHP_val = AHP_bp16->process(AHP_val);    //do it again to make it even tighter
  AHP_tmp8[ind_count] = AHP_val * AHP_val;
  

  if (ind_count == 250) { // if we reached a full second

    // Calculate RMS value from 250 values (One Second)
    for (int i = 1; i < 251; i++) {
       AHP1 = AHP1 + AHP_tmp1[i];
       AHP2 = AHP2 + AHP_tmp2[i];
       AHP3 = AHP3 + AHP_tmp3[i];
       AHP4 = AHP4 + AHP_tmp4[i];
       AHP5 = AHP5 + AHP_tmp5[i];
       AHP6 = AHP6 + AHP_tmp6[i];
       AHP7 = AHP7 + AHP_tmp7[i];
       AHP8 = AHP8 + AHP_tmp8[i];
    }


    AHP1 =  sqrt(abs(AHP1 / 250.0f));
    AHP2 =  sqrt(abs(AHP2 / 250.0f));
    AHP3 =  sqrt(abs(AHP3 / 250.0f));
    AHP4 =  sqrt(abs(AHP4 / 250.0f));
    AHP5 =  sqrt(abs(AHP5 / 250.0f));
    AHP6 =  sqrt(abs(AHP6 / 250.0f));
    AHP7 =  sqrt(abs(AHP7 / 250.0f));
    AHP8 =  sqrt(abs(AHP8 / 250.0f));


    Diff_H1 = (AHP1 * MICROVOLTS_PER_COUNT); 
    Diff_H2 = (AHP2 * MICROVOLTS_PER_COUNT); 
    Diff_H3 = (AHP3 * MICROVOLTS_PER_COUNT); 
    Diff_H4 = (AHP4 * MICROVOLTS_PER_COUNT); 
    Diff_H5 = (AHP5 * MICROVOLTS_PER_COUNT); 
    Diff_H6 = (AHP6 * MICROVOLTS_PER_COUNT); 
    Diff_H7 = (AHP7 * MICROVOLTS_PER_COUNT); 
    Diff_H8 = (AHP8 * MICROVOLTS_PER_COUNT); 
    
    //--------- End: Update Counter ----------//
    ind_count = 1;
    if (printAlpha == true) {
      Serial0.write((float) Diff_H1);
      Serial0.write((float) Diff_H2);
      Serial0.write((float) Diff_H3);
      Serial0.write((float) Diff_H4);
      Serial0.write((float) Diff_H5);
      Serial0.write((float) Diff_H6);
      Serial0.write((float) Diff_H7);
      Serial0.write((float) Diff_H8);
    } else {
      Serial0.write((float) counter);
    }
  } // End of 250samples loop

  OBCI.update24bitData();
  int_count++;
  
  //---------------- END: Update Counter & Act on data ------------------------//
}
