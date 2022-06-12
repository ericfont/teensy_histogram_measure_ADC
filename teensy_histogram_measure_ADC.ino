#include <ADC.h>
#include <ADC_util.h>

ADC *adc = new ADC(); // adc object

const uint32_t analogReadBitDepth0 = 10;
const uint32_t analogReadBitDepth1 = 10;
const uint32_t analogReadMax0 = (1 << analogReadBitDepth0);
const uint32_t analogReadMax1 = (1 << analogReadBitDepth1);
const uint32_t analogReadAveragingNum0 = 1;
const uint32_t analogReadAveragingNum1 = 1;
const uint32_t analogReadPin0 = A0; // ADC0 or ADC1 // pin 14
const uint32_t analogReadPin1 = A1; // ADC0 or ADC1 // pin 15

const uint32_t nMillisecondsDelayPerPrintFrame = 1000; // number of milliseconds to run measurements before printing result

const enum ADC_settings::ADC_CONVERSION_SPEED conversionSpeeds[2] = {ADC_CONVERSION_SPEED::VERY_HIGH_SPEED, ADC_CONVERSION_SPEED::VERY_HIGH_SPEED};
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
  
const enum ADC_settings::ADC_SAMPLING_SPEED samplingSpeeds[2] = {ADC_SAMPLING_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED};
  /* For Teensy 4:
  VERY_LOW_SPEED is guaranteed to be the lowest possible speed within specs (higher than 4 MHz).
  LOW_SPEED is equal to VERY_LOW_SPEED
  MED_SPEED is always >= ADC_LOW_SPEED and <= ADC_HIGH_SPEED.
  HIGH_SPEED is guaranteed to be the highest possible speed within specs (lower or eq than 40 MHz).
  VERY_HIGH_SPEED is equal to HIGH_SPEED
  */

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
  pinMode(analogReadPin0, INPUT_DISABLE);
  pinMode(analogReadPin1, INPUT_DISABLE);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptPressed, FALLING);
  
  Serial.begin(12000000);//115200);

  adc->adc0->setAveraging(analogReadAveragingNum0); // set number of averages
  adc->adc1->setAveraging(analogReadAveragingNum1); // set number of averages
  
  adc->adc0->setResolution(analogReadBitDepth0); // set bits of resolution
  adc->adc1->setResolution(analogReadBitDepth1); // set bits of resolution

  adc->adc0->setConversionSpeed(conversionSpeeds[0]); // change the conversion speed
  adc->adc1->setConversionSpeed(conversionSpeeds[1]); // change the conversion speed

  adc->adc0->setSamplingSpeed(samplingSpeeds[0]); // change the sampling speed
  adc->adc1->setSamplingSpeed(samplingSpeeds[1]); // change the sampling speed

  adc->adc0->wait_for_cal(); // waits until calibration is finished and writes the corresponding registers
  adc->adc1->wait_for_cal(); // waits until calibration is finished and writes the corresponding registers
}

volatile uint64_t measurementHistogram0[analogReadMax0];
volatile uint64_t measurementHistogram1[analogReadMax1];

uint64_t nMeasurements0;
uint64_t nMeasurements1;

uint32_t runNumber = 0;
uint32_t millisStartTimestamp;
volatile uint32_t nMeasurements0PerPrintFrame;
volatile uint32_t nMeasurements1PerPrintFrame;

void loop() {

  if (triggerReset) {
    triggerReset = false;
    
    nMeasurements0 = 0;
    nMeasurements1 = 0;
    
    runNumber += 1;
    
    // reset histogram arrays
    for( uint32_t i = 0; i < analogReadMax0; i++ ) {
      measurementHistogram0[i] = 0;
    }
    for( uint32_t i = 0; i < analogReadMax1; i++ ) {
      measurementHistogram1[i] = 0;
    }
  
    millisStartTimestamp = millis();
  }

  nMeasurements0PerPrintFrame = 0;
  nMeasurements1PerPrintFrame = 0;
  
  adc->adc0->enableInterrupts(adc0_isr);
  adc->adc1->enableInterrupts(adc1_isr);
  
  uint32_t microsPrintFrameStartTime = micros();
  
  adc->adc0->startContinuous(analogReadPin0);
  adc->adc1->startContinuous(analogReadPin1);
  
  delay (nMillisecondsDelayPerPrintFrame); // take measurements for a while
  
  adc->adc0->stopContinuous();
  adc->adc1->stopContinuous();
  
  uint32_t microsPrintFrameDuration = micros() - microsPrintFrameStartTime;
  
  adc->adc0->disableInterrupts();
  adc->adc1->disableInterrupts();  
  
  nMeasurements0 += (uint64_t) nMeasurements0PerPrintFrame;
  nMeasurements1 += (uint64_t) nMeasurements1PerPrintFrame;  

  // end of taking measurements, now time to print summary statistics
  Serial.print("run #");
  Serial.print(runNumber);
  Serial.print(" starting at ");
  Serial.print(microsPrintFrameStartTime / 1000000.0f);
  Serial.print(" seconds for ");
  Serial.print(microsPrintFrameDuration);
  Serial.print(" microseconds, producing cumulative histogram.\n");
  Serial.println();

  // calculate stats
  for( uint32_t adc_number = 0; adc_number < 2; adc_number++ ) {
    volatile uint64_t *measurementHistogram = ( (adc_number == 0) ? measurementHistogram0 : measurementHistogram1);
    uint64_t nMeasurementsPerPrintFrame = ( (adc_number == 0) ? nMeasurements0PerPrintFrame : nMeasurements1PerPrintFrame);
    uint64_t nMeasurements = ( (adc_number == 0) ? nMeasurements0 : nMeasurements1);
    uint32_t analogReadMax = ( (adc_number == 0) ? analogReadMax0 : analogReadMax1);
    uint32_t analogReadBitDepth = ( (adc_number == 0) ? analogReadBitDepth0 : analogReadBitDepth1);
    uint32_t analogReadAveragingNum = ( (adc_number == 0) ? analogReadAveragingNum0 : analogReadAveragingNum1);
    float microsPerSample = (float) nMeasurementsPerPrintFrame / 1000000.0f;

    Serial.print("ADC");
    Serial.print(adc_number);
    Serial.print(": ");
    Serial.print("BitDepth=");
    Serial.print(analogReadBitDepth);
    Serial.print(", averaging=");
    Serial.print(analogReadAveragingNum);
    Serial.print(", ADC_CONVERSION_SPEED=");
    Serial.print((uint8_t) conversionSpeeds[adc_number]);
    Serial.print(", ADC_SAMPLING_SPEED=");
    Serial.print((uint8_t) samplingSpeeds[adc_number]);
    Serial.print(", at rate of ");
    Serial.print(microsPerSample, 6);
    Serial.print(" microsPerSample or ");
    Serial.print(1000.0f / microsPerSample);
    Serial.println(" kHz.");
  
    uint32_t minimum_index = (1 << analogReadBitDepth);    
    uint32_t maximum_index = 0;
    uint32_t mode_index = 0;
    uint64_t mode_value = 0;
    float summation = 0;
    
    for( uint32_t i = 0; i < analogReadMax; i++) {
      if( measurementHistogram[i] > 0 ) {        
        summation += (float)(measurementHistogram[i] * i) / analogReadMax;
    
        if( i < minimum_index )
          minimum_index = i;
          
        if( i > maximum_index )
          maximum_index = i;

        if( measurementHistogram[i] > mode_value ) {
          mode_value = measurementHistogram[i];
          mode_index = i;
        }        
      }
    }
    float mean = (float) summation / nMeasurements;

    Serial.println("bin    [index]     count    percent of total measurements");
    
    float sumofsquares = 0;
      
    for( uint32_t i = 0; i < analogReadMax; i++) {
      float bin = (float) i / analogReadMax;
      float differenceFromMean = bin - mean;
      sumofsquares += (float) measurementHistogram[i] * (differenceFromMean * differenceFromMean);

      if( (i + 16 > mode_index) && (i <= mode_index + 16) ) {
        Serial.print(bin, 4);
        Serial.print(" [");
        Serial.print(i);
        Serial.print("]: ");
        printRightJustifiedUnsignedInt(measurementHistogram[i]);
        if( measurementHistogram[i] > 0 ) {
          float percentOfTotal = (float) measurementHistogram[i] * 100.0f / nMeasurements;
          Serial.print(' ');
          for( int bars = (int64_t) measurementHistogram[i] * 100 / nMeasurements; bars >= 0; bars-- ) {
            Serial.write('=');
          }
          Serial.print(' ');
          Serial.print(percentOfTotal);
          Serial.print('%');
        }
        Serial.println();
      }
    }
        
    Serial.println("Summary:");
    Serial.print("range:  ");
    Serial.print((float) (maximum_index + 1 - minimum_index) / analogReadMax, 9);
    Serial.print(" (contained in ");
    Serial.print(maximum_index - minimum_index);
    Serial.print(" bins, from index [");
    Serial.print(minimum_index);
    Serial.print("] to [");
    Serial.print(maximum_index);
    Serial.println("])");
    
    Serial.print("mode:   ");
    Serial.println(mode_index / (float) analogReadMax, 9);
    
    Serial.print("mean:   ");
    Serial.println(mean, 9);
  
    float variance = sumofsquares / (float) nMeasurements;  
    Serial.print("var:    ");
    Serial.println(variance, 9);
  
    float standardDeviation = sqrt(variance);
    Serial.print("stdDev: ");
    Serial.println(standardDeviation, 9);
  
    Serial.println();
  }
  triggerReset = true;
}

void adc0_isr(void) {
  nMeasurements0PerPrintFrame++;
  int measurement = adc->adc0->analogReadContinuous();
  measurementHistogram0[measurement] += 1;
}

void adc1_isr(void) {
  nMeasurements1PerPrintFrame++;
  int measurement = adc->adc1->analogReadContinuous();
  measurementHistogram1[measurement] += 1;
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
