/********************************************************************************************
 * * Name:        Benjamin Hillen
 * * Date:        9 May 2020
 * * Description: Performs audio functions for the DK Music Box. Users may play a pre-
 * *              programmed song from the SD card, record 20 seconds of their own audio,
 * *              and listen to the primary tones in their recorded audio. Designed to run
 * *              on the Arduino UNO board, #define statements may be changed to allow the
 * *              use of an Arduino MEGA to improve audio quality
********************************************************************************************/
//Possible Improvements:
//Improve mapping search algorithm from linear to binary search for better speed
//Stop feature for record() and playRecording()
//Think about implementing volume control using potentiometer for Song 1 and Song 2
//Improve method for reading button presses, possibly using bounce

#include <arduinoFFT.h>
#include "SD.h"
#define SD_CHIPSELECT 10 //SS select pin 10 for UNO
#include "TMRpcm.h"
#include "SPI.h"

TMRpcm music;
arduinoFFT fft = arduinoFFT();

/*SD Card Pins - UNO
 * CS 10
 * SCK 13
 * MOSI 11
 * MISO 12
 */
#define SPEAKER_PIN 9 //pin 9 for UNO speaker output according to TMRpcm library
#define SD_CHIPSELECT 10 //SS select pin 10 for UNO

#define AUDIO_IN A0 //ADC UNO input
//#define AUDIO_OUT 5 //PWM recorded audio output
#define MOTOR 3 //PWM DK motor pin
#define STOP_PIN 2 //stop button pin

#define SONG1_PIN 6 
#define SONG2_PIN 7

#define RECORD_PIN 4
#define PLAY_RECORDING_PIN 8

#define maxRecordTime 20000000 //Maximum recording duration in microseconds
#define numFreq 240 //number of frequencies in the recording array, more = better signal reconstruction, 390
#define windowDelay 25682 //time between taking a window, 25682 is the maximum to maintain 0.02 second accuracy

/*                    THE GREAT MEMORY CONUNDRUM
//This is what is causing my dynamic memory problems, this int array is huge eats 6250 bytes,
//but the maximum dynamic memory I have on UNO is 2048, meaning the array would have to be
//shrunk to 414 ints.

//arduinoFFT lets me take a maximum of 256 samples per window, equating to a total of
//0.0256 seconds per window. This would result in 781 windows over 20 seconds, leaving me
//with 782 ints, which is 1562 bytes. This decreases audio quality by 400% but would let
//me fit the code on the UNO. Upgrading hardware to an Arduino MEGA would
//allow for the maximum audio quality possible

//The above fix brought us down from 7470 bytes of dynamic memory to 2782 bytes. We still 
//exceed the limit by 734 bytes (35%) and I currently have no idea how to improve that.

//THE SOLUTION: Taking 781 frequencies results in 33 notes per second, which is almost 
//indistinguishable from a lower rate. I can reduce the size of the array by adding a 
//small delay inbetween sampling windows, resulting in less memory consumption at the cost
//of reduced resolution. 

//The current array size allows for 19.5 notes to be played per second over 20 seconds.
//This equates to a maximum error in note duration of 0.02 seconds.
*/
int rFreq[numFreq]; //holds the recorded dominant frequencies

/***********************************************************************
 * *                          setup()
 * * param:
 * * output:
 * * pre:
 * * post:        audio input, speaker, and motor pins are initialized,
 * *              SD card is accessed and Serial port is open at 115200
 * * description: initializes pins and the SD card used in loop() and
 * *              other functions
************************************************************************/
void setup() {
  pinMode(AUDIO_IN, INPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(STOP_PIN, INPUT);
  
  pinMode(SONG1_PIN, INPUT);
  pinMode(SONG2_PIN, INPUT);
  pinMode(RECORD_PIN, INPUT);
  pinMode(PLAY_RECORDING_PIN, INPUT);
  
  music.speakerPin = SPEAKER_PIN;

  digitalWrite(SPEAKER_PIN, LOW);
  
  Serial.begin(115200);

  SD.begin(SD_CHIPSELECT);

  pinMode(SD_CHIPSELECT, OUTPUT);

  music.setVolume(0);

  music.CSPin = SD_CHIPSELECT;

  Serial.begin(115200);
  if(!SD.begin(SD_CHIPSELECT))
  {
    return;
  }
}

/*********************************************************************
 * *                        loop()
 * * param:
 * * output:
 * * pre:           initialized SD card
 * * post:  
 * * description:   monitors buttons for user choice, plays music and
 * *                records audio using custom functions
*********************************************************************/
void loop(){
  //Stop spinning DK
  analogWrite(MOTOR, LOW);
  
  //Song 1
  if (digitalRead(SONG1_PIN) == HIGH){
    bool keepPlaying = true;

    analogWrite(MOTOR, 255);

    music.setVolume(5);

    music.play("song1.wav");

    while (keepPlaying){
      keepPlaying = music.isPlaying();
      
      if (digitalRead(STOP_PIN) == HIGH){
        music.disable();
        break;
      }
    }
  }
  
  //Song 2
  if (digitalRead(SONG2_PIN) == HIGH){
    bool keepPlaying = true;

    analogWrite(MOTOR, 255);

    music.setVolume(5);

    music.play("song2.wav");

    while (keepPlaying){
      keepPlaying = music.isPlaying();
      
      if (digitalRead(STOP_PIN) == HIGH){
        music.disable();
        break;
      }
    }
  }
  
  //Play Recording
  if (digitalRead(PLAY_RECORDING_PIN) == HIGH){
    //playRecording();
    bool keepPlaying = true;

    analogWrite(MOTOR, 255);

    music.setVolume(5);

    music.play("songR.wav");

    while (keepPlaying){
      keepPlaying = music.isPlaying();
      
      if (digitalRead(STOP_PIN) == HIGH){
        music.disable();
        break;
      }
    }
  }
  
  //Record Music
  if (digitalRead(RECORD_PIN) == HIGH){
    //record();
    unsigned long start_record = micros;

    music.startRecording("songR.wav",8000,A0);

    while(micros() - start_record < 20000000){}

    music.stopRecording("songR.wav");
  }
}

/*************************************************************
 * *                    playRecording()
 * * param:
 * * output: 
 * * pre:         rFreq array is populated with frequencies
 * * post:  
 * * description: iterates through rFreq and playes piano
 * *              notes using tone on the arduino
*************************************************************/
void playRecording(){
  //start spinning DK
  analogWrite(MOTOR, 255);
  
  //loop through rFreq and play each frequency as a tone
  for(int i = 0; i < numFreq; i++){
    unsigned long start_note = micros(); //
    //Serial.println(rFreq[i]);
    
    tone(SPEAKER_PIN, rFreq[i]);
    
    if(digitalRead(STOP_PIN) == HIGH){
      return;
    }
    
    while(micros() - start_note < 71428){} //
  }
}

/******************************************************************
 * *                        record()
 * * param:
 * * output:
 * * pre:         AUDIO_IN pin has been initialized
 * * post:        rFreq holds dominant notes computed from input
 * *              audio signal
 * * description: Takes windows of audio samples, uses FFT to
 * *              compute the dominant frequency of the window,
 * *              stores that frequency in an array, and maps
 * *              each frequency to a note on a piano with map()
******************************************************************/
void record(){
  const uint16_t SAMPLES = 256; //Max samples per window is 256
  unsigned long start_samp;

  
  const double SAMP_FREQ = 10000; //Max sampling frequency is 10 kHz on Uno

  //Sampling period in micro seconds
  unsigned int SAMP_PERIOD = (1000000*(1.0/SAMP_FREQ));

  //arrays to hold window sampling values
  double sampArr[SAMPLES];
  double imag[SAMPLES];
  
  unsigned long rStart = micros();
  int j = 0;

  //Serial.println("----------------------------Entering FFT Loop----------------------------");
  //record for 20 seconds
  while(micros() - rStart < maxRecordTime){
    if(digitalRead(STOP_PIN) == HIGH){
      break;
    }
    
    start_samp = micros();

    //Sample signal at AUDIO_IN pin
    for(int i=0; i<SAMPLES; i++){
      sampArr[i] = analogRead(AUDIO_IN);
      imag[i] = 0;

      while(micros() - start_samp < SAMP_PERIOD){}

      start_samp += SAMP_PERIOD;
    }

    //compute FFT
    fft.Windowing(sampArr, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    fft.Compute(sampArr, imag, SAMPLES, FFT_FORWARD);
    fft.ComplexToMagnitude(sampArr, imag, SAMPLES);
    
    //find the dominant frequency of that window
    double x = fft.MajorPeak(sampArr, SAMPLES, SAMP_FREQ);
    
    //store dominant frequency as an int
    rFreq[j] = (int) x;
    j++;

    start_samp = micros();

    //wait for the specified time delay to keep rFreq from consuming too much memory
    while(micros() - start_samp < windowDelay){}
  }

  //Serial.println("----------------------------Entering Mapping Loop----------------------------");
  //loop through and map dominant frequency to closest piano key with map function
  for(int i = 0; i < j; i++)
  {
    int fix = rFreq[i];
    rFreq[i] = freqMap(fix);
  }
}

/********************************************************************************
 * *                              freqMap(int)
 * * param:       value
 * * output:      int
 * * pre:         value is a non-negative frequency
 * * post:        value has been mapped to a piano frequency within +- 4%
 * * description: takes a frequency and calculates an acceptable window of
 * *              frequencies within 4% of the original, then linearly searches
 * *              through the lookup array piano to find the first frequency
 * *              within the acceptable window, which value is mapped to and
 * *              returned
*********************************************************************************/
int freqMap(int value){
  const int size = 69;
  const double accuracy = 0.04;
  
  //array of piano frequencies
  const int piano[size] = {0, 104, 110, 117, 123, 131, 139, 147, 156, 165, 175, 185, 196,
                           208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415,
                           440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880,
                           932, 988, 1046, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 
                           1760, 1865, 1976, 2093, 2218, 2350, 2489, 2637, 2794, 2960, 3136, 
                           3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978};
  
  //calculate window of acceptable mapping
  int error = value * accuracy;
  int highBound = value + error;
  int lowBound = value - error;

  //map value to a rest if it is below hearing threshold
  if (value < 20){
    value = 0;
    //Serial.println("Mapped to Rest");
    return value;
  }
  
  //map value to lowest possible note if it is under 100 Hz bounds
  else if (value < 100 && value >= 20){
    value = 104;
    //Serial.println("Mapped to Lowest Note");
    return value;
  }
  
  //map value to highest possible note if it exceeds bounds
  else if (value > 4978){
    value = 4978;
    //Serial.println("Mapped to Highest Note");
    return value;
  }

  //could improve this to a binary search
  //linear search lookup table for first note within acceptable mapping
  else{
        for (int i = 0; i < size; i++){
          if(piano[i] <= highBound && piano[i] >= lowBound){
            value = piano[i];
            //Serial.println("Mapped to ");
            //Serial.print(value);
            return value;
          }
        }
  }
}
