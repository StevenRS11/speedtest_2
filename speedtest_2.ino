#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define GATE1_PIN 4  // First photogate digital input pin
#define GATE2_PIN 5  // Second photogate digital input pin
#define DISTANCE_INCHES 2.0  // Distance between photogates in inches
#define TIMEOUT_US 1000000  // 1 second timeout in microseconds

#define SDA_PIN 8  // Change this to your preferred SDA pin
#define SCL_PIN 9  // Change this to your preferred SCL pin
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


volatile unsigned long timeGate1 = 0;
volatile unsigned long timeGate2 = 0;
volatile bool gate1Triggered = false;
volatile bool gate2Triggered = false;
volatile int lastTriggeredGate = 0;  // 1 = Gate 1 first, 2 = Gate 2 first
volatile int gate1TriggerCount = 0;
volatile int gate2TriggerCount = 0;

float incomingSpeed = -1;
float outgoingSpeed = -1;

void IRAM_ATTR gate1Interrupt() {
    unsigned long currentTime = micros();

    if (gate1Triggered) {
        gate1TriggerCount++;
        Serial.println("WARNING: Gate 1 triggered multiple times before Gate 2!");
        resetSavedSpeeds();
        return;  // Ignore repeated triggers
    }

    // First activation
    detachInterrupt(digitalPinToInterrupt(GATE1_PIN));  // Disable interrupt
    timeGate1 = currentTime;
    gate1Triggered = true;
    lastTriggeredGate = 1;
    gate1TriggerCount++;
}

void IRAM_ATTR gate2Interrupt() {
    unsigned long currentTime = micros();

    if (gate2Triggered) {
        gate2TriggerCount++;
        Serial.println("WARNING: Gate 2 triggered multiple times before Gate 1!");
        resetSavedSpeeds();
        return;  // Ignore repeated triggers
    }

    // First activation
    detachInterrupt(digitalPinToInterrupt(GATE2_PIN));  // Disable interrupt
    timeGate2 = currentTime;
    gate2Triggered = true;
    lastTriggeredGate = 2;
    gate2TriggerCount++;
}





void resetMeasurement() {
    gate1Triggered = false;
    gate2Triggered = false;
    timeGate1 = 0;
    timeGate2 = 0;
    lastTriggeredGate = 0;
    gate1TriggerCount = 0;
    gate2TriggerCount = 0;


    // Re-enable interrupts
    attachInterrupt(digitalPinToInterrupt(GATE1_PIN), gate1Interrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(GATE2_PIN), gate2Interrupt, RISING);
}

void resetSavedSpeeds(){
      incomingSpeed = -1;
      outgoingSpeed = -1;
}

void printSavedSpeeds(){
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(incomingSpeed);
  display.println(outgoingSpeed);

  display.display();      // Show initial text
  resetSavedSpeeds();
}

void setup() {
    Serial.begin(115200);
    pinMode(GATE1_PIN, INPUT);
    pinMode(GATE2_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(GATE1_PIN), gate1Interrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(GATE2_PIN), gate2Interrupt, RISING);

    Serial.println("Bidirectional Photogate Speed Measurement Ready...");

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }else
  {
    Serial.println("Screen Found");
  }


}



void loop() {
    if (gate1Triggered || gate2Triggered) {
        // Check for timeout
        if ((gate1Triggered && micros() - timeGate1 > TIMEOUT_US) ||
            (gate2Triggered && micros() - timeGate2 > TIMEOUT_US)) {
            Serial.println("Timeout occurred, resetting measurement...");
            resetSavedSpeeds();
            resetMeasurement();
        }
    }

    if (gate1Triggered && gate2Triggered) {
        unsigned long timeDiff = abs((long)(timeGate2 - timeGate1)); // Time difference in microseconds

        if (timeDiff > 0) {
            float timeSeconds = timeDiff / 1e6;  // Convert microseconds to seconds
            float speedInchesPerSec = DISTANCE_INCHES / timeSeconds;
            float speedMPH = speedInchesPerSec * 0.0568182;  // Conversion factor: 1 in/sec = 0.0568182 mph

            // Determine direction
            Serial.print("Speed: ");
            Serial.print(speedMPH);
            Serial.print(" MPH ");
            if (lastTriggeredGate == 1) {
                Serial.println("(Direction: Gate 1 → Gate 2)");
                incomingSpeed = speedMPH;
            } else if (lastTriggeredGate == 2) {
              outgoingSpeed = speedMPH;
                Serial.println("(Direction: Gate 2 → Gate 1)");
            }

            resetMeasurement(); // Reset after successful measurement
        }
    }
    if(incomingSpeed != -1 && outgoingSpeed != 0){
      printSavedSpeeds();
      resetSavedSpeeds();
    }
    

}
