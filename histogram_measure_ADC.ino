const uint32_t analogReadBitDepth = 12;
const uint32_t analogReadMax = (1 << analogReadBitDepth);
const uint32_t analogReadAveragingNum = 32;
const uint32_t analogReadPin = 14;

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
  analogReadResolution(analogReadBitDepth);
  analogReadAveraging(analogReadAveragingNum);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptPressed, FALLING);
  
  Serial.begin(12000000);//115200);
  Serial.print("analog analogReadBitDepth is: ");
  Serial.println(analogReadBitDepth);
  Serial.print("analog analogReadAveragingNum is: ");
  Serial.println(analogReadAveragingNum);
}

uint32_t measurmentHistogram[analogReadMax];
uint32_t runNumber = 0;
uint64_t summation;
uint32_t minimum;
uint32_t maximum;
uint32_t nMeasurements;
uint32_t measurement;
uint32_t millisStartTimestamp;

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

  static uint32_t millisPerPrintFrame = 500;
  uint32_t millisNextPrintFrame = millis() + millisPerPrintFrame;
  uint32_t nMeasurementsThisPrintFrame = 0;

  // take measurements for a while
  while( !triggerReset && millis() < millisNextPrintFrame) {
    measurement = analogRead(analogReadPin);
    measurmentHistogram[measurement] += 1;
    summation += measurement;
    
    if( measurement < minimum )
      minimum = measurement;
      
    if( measurement > maximum )
      maximum = measurement;

    nMeasurementsThisPrintFrame++;
  }
  nMeasurements += nMeasurementsThisPrintFrame;
  
  uint32_t millisDuration = millis() - millisStartTimestamp;

  // end of taking measurements, now time to print summary statistics
  Serial.print("run #");
  Serial.print(runNumber);
  Serial.print(" cumulative histogram after ");
  Serial.print((float) millisDuration / 1000);
  Serial.print(" seconds and ");
  Serial.print(nMeasurements);
  Serial.print(" measurements (");
  Serial.print((float) nMeasurementsThisPrintFrame / millisPerPrintFrame);
  Serial.print(" kHz sampling rate)");
  Serial.println();

  for( uint32_t i=minimum; i<= maximum; i++) {
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
  
  float mean = (float) summation / nMeasurements;
  Serial.print("mean: ");
  Serial.println(mean, 6);

  Serial.println();
  Serial.println();
}

void printRightJustifiedUnsignedInt(uint32_t value) {
  const uint32_t maxDigits = 10;
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