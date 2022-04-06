#include <Arduino.h>
#include <SensirionI2CSfmSf06.h>
#include <Wire.h>

SensirionI2CSfmSf06 sfmSf06;

int flowPinMp = 2;    //This is the input pin on the Arduino
int flowPinMs = 3;    //This is the input pin on the Arduino

double flowRateMp;    //This is the value we intend to calculate.
volatile int countMp; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.

double flowRateMs;    //This is the value we intend to calculate.
volatile int countMs; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.

volatile int HMp, HMs;

void setup() {
  // put your setup code here, to run once:
  pinMode(flowPinMp, INPUT);           //Sets the pin as an input
  pinMode(flowPinMs, INPUT);           //Sets the pin as an input
  attachInterrupt(0, FlowMp, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"
  attachInterrupt(1, FlowMs, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"
  Serial.begin(115200);  //Start Serial

    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    Wire.begin();

    uint16_t error;
    char errorMessage[256];

    /**
     * select the proper i2c address for your sensor
     * see datasheet of your sensor
     *
     */
    sfmSf06.begin(Wire, ADDR_SFM3119);

    error = sfmSf06.stopContinuousMeasurement();

    if (error) {
        Serial.print("Error trying to execute stopContinuousMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    uint32_t productIdentifier;
    uint8_t serialNumber[8];
    uint8_t serialNumberSize = 8;

    error = sfmSf06.readProductIdentifier(productIdentifier, serialNumber,
                                          serialNumberSize);

    if (error) {
        Serial.print("Error trying to execute readProductIdentifier(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Product Identifer:");
        Serial.println(productIdentifier);
        Serial.print("Serial Number:");
        Serial.print("0x");
        for (size_t i = 0; i < serialNumberSize; i++) {
            uint8_t value = serialNumber[i];
            Serial.print(value < 16 ? "0" : "");
            Serial.print(value, HEX);
        }
        Serial.println();
    }

    sfmSf06.startO2ContinuousMeasurement();
    
}
void loop() {
  // put your main code here, to run repeatedly:
  countMp = 0;      // Reset the counter so we start counting from 0 again
  countMs = 0;      // Reset the counter so we start counting from 0 again
  HMp=0;
  HMs=0;
  uint16_t error;
    char errorMessage[256];
    float flow = 0;
    float temperature = 0;
    uint16_t status = 0;
  Serial.println("1");
  interrupts();   //Enables interrupts on the Arduino
  delay (200);   //Wait 1 second
  HMp = countMp;
  HMs = countMs;
  error = sfmSf06.readMeasurementData(flow, temperature, status);
  if (!error) {
        Serial.print(flow);
        Serial.print("\t");
        Serial.println(temperature);
    }

  Serial.print(HMp);         //Print the variable flowRate to Serial
  Serial.print("\t");
  Serial.println(flowRateMs);
}

void FlowMp()
{
   countMp++; //Every time this function is called, increment "count" by 1
}

void FlowMs()
{
   countMs++; //Every time this function is called, increment "count" by 1
}
