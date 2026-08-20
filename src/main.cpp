#include <Arduino.h>
#include "arduinoFFT.h"
#include <PDM.h>

#define DEBUG 0                 
#define SAMPLES 1024       
#define SAMPLING_FREQ 16000

double INVERSE_PERIOD = (double) SAMPLING_FREQ / SAMPLES;

int16_t recording_buf[SAMPLES];
volatile uint8_t recording = 0;
volatile static bool record_ready = false;
volatile int samples_received = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();

  while (bytesAvailable > 0) {
    int bytesToRead = min(bytesAvailable, (SAMPLES - samples_received) * (int)sizeof(int16_t));

    PDM.read(&recording_buf[samples_received], bytesToRead);

    samples_received += bytesToRead / sizeof(int16_t);
    bytesAvailable -= bytesToRead;

    if (samples_received >= SAMPLES) {
      record_ready = true;
      samples_received = 0;
      break;
    }
  }
}

ArduinoFFT<double> FFT = ArduinoFFT<double>();
double vReal[SAMPLES];
double vImag[SAMPLES];

const char* noteNames[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};



String findNearestNote(double frequency) {
  if (frequency < 20) return "Low";
  
  double A4 = 440.0;
  double C0 = A4 * pow(2, -4.75); 
  double halfSteps = 12 * log2(frequency / C0);
  int roundedHalfSteps = round(halfSteps);
  
  if (roundedHalfSteps < 0) roundedHalfSteps = 0;
  
  int octave = roundedHalfSteps / 12;
  int noteIndex = roundedHalfSteps % 12;
  
  return String(noteNames[noteIndex]) + String(octave);
}

int findNearestFreqCollection(double frequency) {
  if (frequency < 20) return 0;
  
  double A4 = 440.0;
  double C0 = A4 * pow(2, -4.75); 
  double halfSteps = 12 * log2(frequency / C0);
  int roundedHalfSteps = round(halfSteps);
  
  if (roundedHalfSteps < 0) roundedHalfSteps = 0;
  
  int octave = roundedHalfSteps / 12;
  int noteIndex = (roundedHalfSteps % 12 ) + 1;
  
  return (octave * 100) + noteIndex; // for example C1 would be 100 + 1 = 101. A4 would be 410. Basically think of Base-12.
}

double freqToCents(double frequency) {
  double A4 = 440.0;
  double C0 = A4 * pow(2, -4.75);
  double halfSteps = 12 * log2(frequency / C0);
  double cents = (halfSteps - round(halfSteps)) * 100;
  return cents;
}

double getPeakMagnitude(double targetFreq) {
  int center = round(targetFreq / INVERSE_PERIOD);

  if (center < 2 || center >= (SAMPLES /2) - 2) return 0;

  double bestMagnitude = 0;

  for (int i = center - 2; i <= center + 2; i++) {

    if (vReal[i] > vReal[i - 1] && vReal[i] > vReal[i + 1]) {
      if (vReal[i] > bestMagnitude)
        bestMagnitude = vReal[i];
    }
  }

  return bestMagnitude;
}

double refineFrequency(double frequency) {
  int center = round(frequency / INVERSE_PERIOD);

  if (center < 2 || center >= (SAMPLES / 2) - 1)
    return frequency;

  int bestIndex = center;

  for (int i = center - 2; i <= center + 2; i++) {
    if (vReal[i] > vReal[bestIndex]) {
      bestIndex = i;
    }
  }

  double alpha = vReal[bestIndex - 1];
  double beta  = vReal[bestIndex];
  double gamma = vReal[bestIndex + 1];

  double denominator = alpha - 2 * beta + gamma;

  if (abs(denominator) < 0.0001)
    return bestIndex * INVERSE_PERIOD;

  double offset = (alpha - gamma) / (2 * denominator);

  return (bestIndex + offset) * INVERSE_PERIOD;
}


double findFundamental() {
  double bestFrequency = 0;
  double bestScore = 0;

  double minFreq = 196.00;   // G3
  double maxFreq = 3520.00;  // A7

  for (double candidate = minFreq; candidate <= maxFreq; candidate *= pow(2.0, 1.0 / 12.0)) {

    double score = 0;
    for (int harmonic = 1; harmonic <= 5; harmonic++) {

      double harmonicFreq = candidate * harmonic;
      if (harmonicFreq >= SAMPLING_FREQ / 2) break;

      double magnitude = getPeakMagnitude(harmonicFreq);

      if (harmonic == 1)
        score += magnitude * 2.0 * (1 / candidate);
      else
        score += (magnitude / harmonic) * (1 / candidate);
      }

    if (score > bestScore) {
      bestScore = score;
      bestFrequency = candidate;
    }
  }

  return bestFrequency;
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {delay(10);}

  PDM.onReceive(onPDMdata);

  PDM.setGain(20);

  if (!PDM.begin(1, 16000)) {
    Serial.println("microphone not starting");
    while(1);
  }

  Serial.println("Mic started");
}

void loop() { 
  if (record_ready) {
    
    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] = recording_buf[i];
      vImag[i] = 0.0;
    }
    
    double mean = 0;
    for (int i = 0; i < SAMPLES; i++) {
      mean += vReal[i];
    }
    mean /= SAMPLES;

    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] -= mean;
    }
    
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HANN, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);
    
    double maxMagnitude = 0;
    uint16_t maxIndex = 0;

    String lastFT = "";
    double magSum = 0;
    
    // Bin 0 is DC componet.
    for (uint16_t i = 2; i < (SAMPLES / 2); i++) {
      if (vReal[i] > maxMagnitude) {
        maxMagnitude = vReal[i];
        maxIndex = i;
      }
      /*
      Explain to future Divy: 
        - frequecy is related to index in vReal[] (*Inverse_Period and adjusted by a parabolic factor)
        - magnitude is the element in vReal[]

        This for-loop finds basically the most dominate frequency (dominate by highest magitude) and 
      assigns that the 'correct' freq we need to find. However this system fails to account for overtones and harmonics,
      giving it a chance that the frequency is the wrong octave.
      */
      //*
      //Serial for overtone data.
      
      String frequencyTwelve = findNearestNote(i * INVERSE_PERIOD);
      double mag = vReal[i];
      int currentNote = findNearestFreqCollection(i * INVERSE_PERIOD);

      if (currentNote >= 307 && currentNote <= 611){
        if(lastFT != frequencyTwelve && lastFT != ""){

          Serial.print(lastFT);
          Serial.print(",");
          Serial.print(magSum , 2);
          Serial.print(";");

          magSum = mag;
          lastFT = frequencyTwelve;

        }else {
          magSum += mag;
        }

        lastFT = frequencyTwelve;
      }


      if (i == (SAMPLES / 2) - 1) {
        Serial.print(lastFT);
        Serial.print(",");
        Serial.println(magSum , 2);
      }

      //*/

    }
    
    double fundamentalCandidate = findFundamental();
    double interpolatedFreq = refineFrequency(fundamentalCandidate);

    String noteName = findNearestNote(interpolatedFreq);
    double noteCents = freqToCents(interpolatedFreq);

    Serial.print("N,");
    
    //*
    if (maxMagnitude > 1600) {
      Serial.print(noteName + ",");    
      Serial.println(noteCents);    
    } else {
      Serial.print("Quiet,");
      Serial.println("0");
    }//*/
    
    record_ready = false;
  }
  
}

