#include <ADC.h>
#include <ADC_util.h>

ADC *adc = new ADC(); // adc object

const uint32_t analogReadBitDepth = 12;
const uint32_t analogReadMax = (1 << analogReadBitDepth);
const uint32_t analogReadAveragingNum = 32;
const uint32_t analogReadPin = A0; // ADC0 or ADC1

const uint32_t interruptPin = 0;
volatile uint32_t triggerReset = true; // start true so initialize histogram & stats
uint32_t millisEarliestNextInterrupt = 0;

void interruptPressed() {
  
  // wait before new interrupt (debounce)
  uint32_t millisRead = millis();
  if (millisRead <= millisEarliestNextInterrupt)
    return;  
  millisEarliestNextInterrupt = millisRead + 200;

  triggerReset = true;
}

void setup() {  
  pinMode(analogReadPin, INPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptPressed, FALLING);
  
  Serial.begin(12000000);//115200);
  Serial.print("analog analogReadBitDepth is: ");
  Serial.println(analogReadBitDepth);
  Serial.print("analog analogReadAveragingNum is: ");
  Serial.println(analogReadAveragingNum);

  adc->adc0->setAveraging(analogReadAveragingNum); // set number of averages
  adc->adc0->setResolution(analogReadBitDepth); // set bits of resolution

  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  /* For Teensy 4:
  VERY_LOW_SPEED: is the lowest possible sampling speed (+22 ADCK, 24 in total).
  LOW_SPEED adds +18 ADCK, 20 in total.
  LOW_MED_SPEED adds +14, 16 in total.
  MED_SPEED adds +10, 12 in total.
  MED_HIGH_SPEED adds +6 ADCK, 8 in total.
  HIGH_SPEED adds +4 ADCK, 6 in total.
  HIGH_VERY_HIGH_SPEED adds +2 ADCK, 4 in total
  VERY_HIGH_SPEED is the highest possible sampling speed (0 ADCK added, 2 in total).
  */

  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  /* For Teensy 4:
  VERY_LOW_SPEED is guaranteed to be the lowest possible speed within specs (higher than 4 MHz).
  LOW_SPEED is equal to VERY_LOW_SPEED
  MED_SPEED is always >= ADC_LOW_SPEED and <= ADC_HIGH_SPEED.
  HIGH_SPEED is guaranteed to be the highest possible speed within specs (lower or eq than 40 MHz).
  VERY_HIGH_SPEED is equal to HIGH_SPEED
  */

  adc->adc0->wait_for_cal(); // waits until calibration is finished and writes the corresponding registers
}

uint32_t measurmentHistogram[analogReadMax];
uint32_t runNumber = 0;
uint32_t minimum;
uint32_t maximum;
uint32_t nMeasurements;
uint32_t measurement;
uint32_t millisStartTimestamp;
uint64_t summation;

void loop() {

  if (triggerReset) {
    triggerReset = false;
    
    summation = 0;
    minimum = (1 << analogReadBitDepth);
    maximum = 0;
    nMeasurements = 0;
    runNumber += 1;
    
    // reset histogram array
    for( uint32_t i = 0; i < analogReadMax; i++ ) {
      measurmentHistogram[i] = 0;
    }
  
    millisStartTimestamp = millis();
  }

  uint32_t microsStartPrintFrame = micros();
  uint32_t nMeasurementsPerPrintFrame = 1000;

  // take measurements for a while
  for( uint32_t i=0; i<nMeasurementsPerPrintFrame; i++ ) {
    measurement = adc->adc0->analogRead(analogReadPin);
    summation += measurement;
    measurmentHistogram[measurement] += 1;
    
    if( measurement < minimum )
      minimum = measurement;
      
    if( measurement > maximum )
      maximum = measurement;
  }
  nMeasurements += nMeasurementsPerPrintFrame;
  
  uint32_t microsPrintFrameDuration = micros() - microsStartPrintFrame;

  // end of taking measurements, now time to print summary statistics
  Serial.print("run #");
  Serial.print(runNumber);
  Serial.print(" cumulative histogram after ");
  Serial.print((float) millis() / 1000);
  Serial.print(" seconds and ");
  Serial.print(nMeasurements);
  Serial.print(" measurements (");
  Serial.print((float) nMeasurementsPerPrintFrame * 1000.0f / microsPrintFrameDuration);
  Serial.print(" kHz sampling rate)");
  Serial.println();

  float mean = (float) summation / nMeasurements;
  float sumofsquares = 0;
  
  for( uint32_t i=minimum; i<= maximum; i++) {
    float differenceFromMean = (float) i - mean;
    sumofsquares += (float) measurmentHistogram[i] * (differenceFromMean * differenceFromMean);
    Serial.print("bin[");
    Serial.print(i);
    Serial.print("] = ");
    printRightJustifiedUnsignedInt(measurmentHistogram[i]);
    float percentOfTotal = (float) measurmentHistogram[i] * 100.0f / nMeasurements;
    Serial.print(' ');
    for( int bars = measurmentHistogram[i] * 100 / nMeasurements; bars >= 0; bars-- ) {
      Serial.write('=');
    }
    Serial.print(' ');
    Serial.print(percentOfTotal);
    Serial.println('%');
  }
  Serial.println("normalized scale:       0%       10%       20%       30%       40%       50%       60%       70%       80%       90%      100%");
  
  Serial.print("mean:   ");
  Serial.println(mean, 6);

  float variance = sumofsquares / (float) nMeasurements;  
  Serial.print("var:    ");
  Serial.println(variance, 6);

  float standardDeviation = sqrt(variance);
  Serial.print("stdDev: ");
  Serial.println(standardDeviation, 6);

  Serial.println();
  Serial.println();
}

void printRightJustifiedUnsignedInt(uint32_t value) {
  const int32_t maxDigits = 10;
  uint32_t digits[maxDigits];
  int32_t digitIndex = 0;
  while( digitIndex < maxDigits ) {
    digits[digitIndex] = value % 10;
    value = value / 10;

    if( value == 0 ) {
      for( int32_t digitsLeft = digitIndex + 1; digitsLeft < maxDigits; digitsLeft++ ) {
        Serial.write(' ');
      }
      break;
    }

    digitIndex++;
  }

  while (digitIndex >= 0) {
    Serial.print(digits[digitIndex]);
    digitIndex--;
  }
}
