#include <OneWire.h>
#include <DallasTemperature.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>

// Mutex Handle for Pressure Sensor
SemaphoreHandle_t pressure_mutex = NULL;

void writeIntIntoEEPROM(int address, unsigned long number)
{ 
  EEPROM.write(address, (number >> 24) & 0xFF);
  EEPROM.write(address + 1, (number >> 16) & 0xFF);
  EEPROM.write(address + 2, (number >> 8) & 0xFF);
  EEPROM.write(address + 3, number & 0xFF);
}

unsigned long readIntFromEEPROM(int address)
{
  return (((long)EEPROM.read(address) << 24) + ((long)EEPROM.read(address + 1) << 16) + ((long)EEPROM.read(address + 2) << 8) + ((long)EEPROM.read(address + 3)));
}

void manageHeatingElement(bool &active, bool &cycleActive, unsigned long &cycleStartTime, int elementPin, float temperature, int elementNumber, unsigned long &elementOnTime, int addr); 
void HeatingElementControl(void *pvParameters); 
void BoosterPumpControl(void *pvParameters);
void CirculationPumpControl(void *pvParameters); 

// GPIO pins configuration
#define TEMPERATURE_PIN 4         // Pin for temperature sensor
#define PRESSURE_SENSOR_PIN 35     // Pin for pressure sensor
#define BOOSTER_PUMP_PIN 32        // Pin for booster pump
#define CIRCULATION_PUMP_PIN 14    // Pin for circulation pump
#define ELEMENT_1_PIN 27           // Pin for heating element 1
#define ELEMENT_2_PIN 26           // Pin for heating element 2
#define ELEMENT_3_PIN 25           // Pin for heating element 3
#define ELEMENT_4_PIN 33           // Pin for heating element 4

// Setup a oneWire instance to communicate with OneWire devices 
OneWire oneWire(TEMPERATURE_PIN);

// Pass oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// Temperature and pressure Thresholds 
const float MAX_TEMPERATURE = 60.0;
const float FINAL_MAX_TEMPERATURE = 65.0;
const float MAX_PRESSURE_C_PUMP = 2;
const float MIN_PRESSURE_C_PUMP = 1.75;
const float MAX_PRESSURE_B_PUMP = 1;
const float MIN_PRESSURE_B_PUMP = 0.5;

const int adress1 = 0;
const int adress2 = 4;
const int adress3 = 8;
const int adress4 = 12;

// Time durations configuration
const unsigned long C_PUMP_CYCLE_DURATION = 60 * 60 * 1000;   // 1 hour in milliseconds
const unsigned long ACTIVE_DURATION = 45 * 60 * 1000;         // 45 minutes in milliseconds
const unsigned long REST_DURATION = 15 * 60 * 1000;           // 15 minutes in milliseconds
const unsigned long CHECK_DURATION = 10 * 60 * 1000;          // 10 minutes in milliseconds

unsigned long Element1onTime = 0;
unsigned long Element2onTime = 0;
unsigned long Element3onTime = 0;
unsigned long Element4onTime = 0;

// Flags to track heating element 
bool element1Active = false;
bool element2Active = false;
bool element3Active = false;
bool element4Active = false;
bool element1CycleActive = false;
bool element2CycleActive = false;
bool element3CycleActive = false;
bool element4CycleActive = false;

int currentElement = 1;   // Indicates the currently active element
int rawPressure = 0;
String pressureVal;

//Heating Elements numbers
int element1Number = 1;
int element2Number = 2;
int element3Number = 3;
int element4Number = 4;

// Status Flags
bool firstElementCheck = true;
bool allElementsActive = false;
bool circulationPumpCheck = false;
bool circulationPumpFirstTime = true;
bool boosterPumpActive = false;

// Cycle time variables
unsigned long element1CycleStartTime = 0;
unsigned long element2CycleStartTime = 0;
unsigned long element3CycleStartTime = 0;
unsigned long element4CycleStartTime = 0;
unsigned long pumpCheckTime = 0;
unsigned long lastCheckTime = 0;

void setup() 
{
  Serial.begin(115200);
  EEPROM.begin(16);
  sensors.begin();

  SerialBT.begin("ESP32test");  //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  Element1onTime = readIntFromEEPROM(adress1);
  Element2onTime = readIntFromEEPROM(adress2);
  Element3onTime = readIntFromEEPROM(adress3);
  Element4onTime = readIntFromEEPROM(adress4);  

  // Configure output pins
  pinMode(ELEMENT_1_PIN, OUTPUT);
  pinMode(ELEMENT_2_PIN, OUTPUT);
  pinMode(ELEMENT_3_PIN, OUTPUT);
  pinMode(ELEMENT_4_PIN, OUTPUT);
  pinMode(BOOSTER_PUMP_PIN, OUTPUT);
  pinMode(CIRCULATION_PUMP_PIN, OUTPUT);

  // Configure input pins
  pinMode(TEMPERATURE_PIN, INPUT);
  pinMode(PRESSURE_SENSOR_PIN, INPUT);

  // Turn off all heating elements initially
  digitalWrite(ELEMENT_1_PIN, LOW);
  digitalWrite(ELEMENT_2_PIN, LOW);
  digitalWrite(ELEMENT_3_PIN, LOW);
  digitalWrite(ELEMENT_4_PIN, LOW);

  // Start Circulation pump at the beginning
  if (!circulationPumpCheck)
  {
    digitalWrite(CIRCULATION_PUMP_PIN, HIGH);
    circulationPumpCheck = true;
    pumpCheckTime = millis();
  }
  
  // Create mutex for pressure sensor
  pressure_mutex = xSemaphoreCreateMutex();
  if (pressure_mutex != NULL) 
  {
    Serial.println("Pressure Sensor Mutex created");
  }

  // Create freertos tasks
  xTaskCreate(BoosterPumpControl, "BoosterPumpControlTask", 4096, NULL, 1, NULL);
  xTaskCreate(HeatingElementControl, "HeatingElementControlTask", 4096, NULL, 2, NULL);
  xTaskCreate(CirculationPumpControl, "CirculationPumpControlTask", 4096, NULL, 1, NULL);
}
void loop()
{}

// Task to control the heating elements
void HeatingElementControl(void *pvParameters) 
{
  String previousTemp = "-127.0", secondPreviousTemp = "-127.0";
  for (;;) 
  {
    String element1time, element2time, element3time, element4time;
    Serial.print("Element 1 total onTime: ");
    Serial.print(Element1onTime);
    Serial.println(" mins");
    element1time = (String)Element1onTime;

    Serial.print("Element 2 total onTime: ");
    Serial.print(Element2onTime);
    Serial.println(" mins");
    element2time = (String)Element2onTime;
   
    Serial.print("Element 3 total onTime: ");
    Serial.print(Element3onTime);
    Serial.println(" mins");
    element3time = (String)Element3onTime;
    
    Serial.print("Element 4 total onTime: ");
    Serial.print(Element4onTime);
    Serial.println(" mins");
    element4time = (String)Element4onTime;

    String element1TimeHeading = "\nElement 1 total on time: ";
    String element2TimeHeading = "\nElement 2 total on time: ";
    String element3TimeHeading = "\nElement 3 total on time: ";
    String element4TimeHeading = "\nElement 4 total on time: ";
    String temp_heading = "\nTemperature: ";
    String currentTemperature;  // Variable to store the current temperature
  
    sensors.requestTemperatures();
    currentTemperature = sensors.getTempCByIndex(0);  // Get the current temperature

    if (currentTemperature.toFloat() == -127.0)
    {
      printf("\n\rFalse Reading!");
      if (secondPreviousTemp.toFloat() != -127.0)
      {
        currentTemperature = secondPreviousTemp;
      }
      if (previousTemp.toFloat() != -127.0)
      {
        currentTemperature = previousTemp;
      }
    }

    Serial.println("\n\rTemperature: ");
    Serial.println(currentTemperature);

    secondPreviousTemp = previousTemp;
    previousTemp = currentTemperature;

    if (SerialBT.available()) 
    {
      SerialBT.write((const uint8_t*)element1TimeHeading.c_str(),element1TimeHeading.length());
      SerialBT.write((const uint8_t*)element1time.c_str(),element1time.length());
      SerialBT.write((const uint8_t*)element2TimeHeading.c_str(),element2TimeHeading.length());
      SerialBT.write((const uint8_t*)element2time.c_str(),element2time.length());
      SerialBT.write((const uint8_t*)element3TimeHeading.c_str(),element3TimeHeading.length());
      SerialBT.write((const uint8_t*)element3time.c_str(),element3time.length());
      SerialBT.write((const uint8_t*)element4TimeHeading.c_str(),element4TimeHeading.length());
      SerialBT.write((const uint8_t*)element4time.c_str(),element4time.length());
      SerialBT.write((const uint8_t*)temp_heading.c_str(),temp_heading.length());
      SerialBT.write((const uint8_t*)currentTemperature.c_str(),currentTemperature.length());
      SerialBT.flush();
    }

    unsigned long currentTime = millis();  // Get the current time in milliseconds

    // Check if 10 minutes have passed, to check the temperature to turn on the next element or if it's the first element turn
    if (currentTime - lastCheckTime >= CHECK_DURATION || firstElementCheck) 
    {
      lastCheckTime = currentTime;  // Update the last check time

      // Check if the current temperature is below the specified limit and if all elements are active
      if (currentTemperature.toFloat() < MAX_TEMPERATURE && (!element1CycleActive || !element2CycleActive || !element3CycleActive || !element4CycleActive)) 
      {
        if (Element1onTime <= Element2onTime && Element1onTime <= Element3onTime && Element1onTime <= Element4onTime && !element1CycleActive) 
        {
          digitalWrite(ELEMENT_1_PIN, HIGH);  // Turn on heating element 1
          firstElementCheck = false;  // Mark the first element check as done
          element1Active = true;  // Mark element 1 as active
          element1CycleStartTime = millis();  // Record the cycle start time
          element1CycleActive = true;  // Mark element 1 cycle as active
          Serial.println("Element 1 Active!");
          
        } 
        else if (Element2onTime <= Element3onTime && Element2onTime <= Element4onTime && !element2CycleActive) 
        {
          digitalWrite(ELEMENT_2_PIN, HIGH);  // Turn on heating element 2
          firstElementCheck = false;  // Mark the first element check as done
          element2Active = true;  // Mark element 2 as active
          element2CycleStartTime = millis();  // Record the cycle start time
          element2CycleActive = true;  // Mark element 2 cycle as active
          Serial.println("Element 2 Active!");
          
        } 
        else if (element3RunningTime <= element4RunningTime) 
        {
          turnOnElement(HEATING_ELEMENT_3_PIN, element3RunningTime);
        } 
        else 
        {
          turnOnElement(HEATING_ELEMENT_4_PIN, element4RunningTime);
        }

        // Activate the appropriate heating element turn by turn based on the current element using switch case statement
        switch (currentElement) 
        {
          case 1:
            {
              digitalWrite(ELEMENT_1_PIN, HIGH);  // Turn on heating element 1
              firstElementCheck = false;  // Mark the first element check as done
              element1Active = true;  // Mark element 1 as active
              element1CycleStartTime = millis();  // Record the cycle start time
              element1CycleActive = true;  // Mark element 1 cycle as active
              Serial.println("Element 1 Active!");
              break;
            }
          case 2:
            {
              digitalWrite(ELEMENT_2_PIN, HIGH);  // Turn on heating element 2
              firstElementCheck = false;  // Mark the first element check as done
              element2Active = true;  // Mark element 2 as active
              element2CycleStartTime = millis();  // Record the cycle start time
              element2CycleActive = true;  // Mark element 2 cycle as active
              Serial.println("Element 2 Active!");
              break;
            }
          case 3:
            {
              digitalWrite(ELEMENT_3_PIN, HIGH);  // Turn on heating element 3
              firstElementCheck = false;  // Mark the first element check as done
              element3Active = true;  // Mark element 3 as active
              element3CycleStartTime = millis();  // Record the cycle start time
              element3CycleActive = true;  // Mark element 3 cycle as active
              Serial.println("Element 3 Active!");
              break;
            }
          case 4:
            {
              digitalWrite(ELEMENT_4_PIN, HIGH);  // Turn on heating element 4
              firstElementCheck = false;  // Mark the first element check as done
              element4Active = true;  // Mark element 4 as active
              element4CycleStartTime = millis();  // Record the cycle start time
              element4CycleActive = true;  // Mark element 4 cycle as active
              // allElementsActive = true;  // Mark all elements as active
              Serial.println("Element 4 Active!");
              break;
            }
        }

        currentElement = (currentElement % 4) + 1;  // Move to the next element for the next cycle
      }
    }
    // Manage heating elements based on their status and current temperature
    manageHeatingElement(element1Active, element1CycleActive, element1CycleStartTime, ELEMENT_1_PIN, currentTemperature.toFloat(), element1Number, Element1onTime, adress1);
    manageHeatingElement(element2Active, element2CycleActive, element2CycleStartTime, ELEMENT_2_PIN, currentTemperature.toFloat(), element2Number, Element2onTime, adress2);
    manageHeatingElement(element3Active, element3CycleActive, element3CycleStartTime, ELEMENT_3_PIN, currentTemperature.toFloat(), element3Number, Element3onTime, adress3);
    manageHeatingElement(element4Active, element4CycleActive, element4CycleStartTime, ELEMENT_4_PIN, currentTemperature.toFloat(), element4Number, Element4onTime, adress4);

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay to prevent tight loops
  }
}

// Task to control the booster pump
void BoosterPumpControl(void *pvParameters) 
{
  String pres_heading = "\nPressure: ";
  for (;;)
  {
    if (xSemaphoreTake(pressure_mutex, portMAX_DELAY) == pdTRUE)
    {
      // rawPressure = analogRead(PRESSURE_SENSOR_PIN);  // Read raw pressure value
      // pressureVal = map(rawPressure, 0, 3840, 0, 400);  // Map raw value to pressure range
      float sum;
      String average;
      for (int i=0; i<5; i++)
      {
        rawPressure = analogRead(PRESSURE_SENSOR_PIN);  // Read raw pressure value
        sum += map(rawPressure, 0, 3900, 0, 400);  // Map raw value to pressure range
      }
      average = (sum / 5);
      sum = 0;
      pressureVal = average;
      pressureVal = pressureVal.toFloat() / 100;  // Convert pressure value to desired range
      if (pressureVal.toFloat() < 0)
      {
        pressureVal = "0.0";
      }
      else if (pressureVal.toFloat() > 4)
      {
        pressureVal = "4.0";
      }
      Serial.println("\n\rPressure: ");
      Serial.println(pressureVal);
      if (SerialBT.available()) 
      {
        SerialBT.write((const uint8_t*)pres_heading.c_str(),pres_heading.length());
        SerialBT.write((const uint8_t*)pressureVal.c_str(),pressureVal.length());
        SerialBT.flush();
      }
      
      xSemaphoreGive(pressure_mutex);
    }
    
    // Conditions to make booster pump inactive, if the pump currently active and below the specified pressure limit
    if (boosterPumpActive && pressureVal.toFloat() >= MAX_PRESSURE_B_PUMP)
    {
      digitalWrite(BOOSTER_PUMP_PIN, LOW);  // Turn off the booster pump
      boosterPumpActive = false;  // Mark the booster pump as inactive
    }
    // Conditions to make booster pump active, if the pump currently inactive and above the specified pressure limit
    else if (!boosterPumpActive && pressureVal.toFloat() < MIN_PRESSURE_B_PUMP)
    {
      digitalWrite(BOOSTER_PUMP_PIN, HIGH);  // Turn on the booster pump
      boosterPumpActive = true;  // Mark the booster pump as active
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay to prevent tight loops
  }
}

// Task to control the circulation pump
void CirculationPumpControl(void *pvParameters) 
{
  for (;;)
  {
    if (xSemaphoreTake(pressure_mutex, portMAX_DELAY) == pdTRUE)
    {
      // rawPressure = analogRead(PRESSURE_SENSOR_PIN);  // Read raw pressure value
      // pressureVal = map(rawPressure, 0, 3850, 0, 400);  // Map raw value to pressure range
      float sum;
      String average;
      for (int i=0; i<5; i++)
      {
        rawPressure = analogRead(PRESSURE_SENSOR_PIN);  // Read raw pressure value
        sum += map(rawPressure, 0, 3900, 0, 400);  // Map raw value to pressure range
      }
      average = (sum / 5);
      sum = 0;
      pressureVal = average;
      pressureVal = pressureVal.toFloat() / 100;  // Convert pressure value to desired range
      if (pressureVal.toFloat() < 0)
      {
        pressureVal = "0.0";
      }
      else if (pressureVal.toFloat() > 4)
      {
        pressureVal = "4.0";
      }
      Serial.println("\n\rPressure: ");
      Serial.println(pressureVal);
      xSemaphoreGive(pressure_mutex);
    }

    unsigned long CURRENT_TIME = millis(); // Get the current time in milliseconds
    
    // Conditions to make circulation pump inactive

    // If Circulation pump currently active
    if (circulationPumpCheck)
    { 
      // If circulation is active for specified duration of 1 hour
      if ((CURRENT_TIME - pumpCheckTime) >= C_PUMP_CYCLE_DURATION)
      {
        // If pressure value is above the specified pressure limit for circulation pump
        if (pressureVal.toFloat() >= MAX_PRESSURE_C_PUMP)
        {
          digitalWrite(CIRCULATION_PUMP_PIN, LOW);  // Turn off the circulation pump
          circulationPumpCheck = false;  // Mark the circulation pump check as inactive
          pumpCheckTime = CURRENT_TIME;  // Record the pump check time
        }
      }
    }
    // Conditions to make circulation pump active. if circulation pump currently inactive
    else
    {
      // If circulation is inactive for specified duration of 1 hour or if pressure valve is below the specified pressure limit for circulation pump
      if ((CURRENT_TIME - pumpCheckTime) >= C_PUMP_CYCLE_DURATION || pressureVal.toFloat() < MIN_PRESSURE_C_PUMP)
      {
        digitalWrite(CIRCULATION_PUMP_PIN, HIGH);  // Turn on the circulation pump
        circulationPumpCheck = true;  // Mark the circulation pump check as active
        pumpCheckTime = CURRENT_TIME;  // Record the pump check time
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay to prevent tight loops
  }
}

void manageHeatingElement(bool &active, bool &cycleActive, unsigned long &cycleStartTime, int elementPin, float temperature, int elementNumber, unsigned long &elementOnTime, int addr) 
{
  unsigned long elapsedTime = millis() - cycleStartTime;  // Calculate elapsed time since cycle start
  
  if (temperature < MAX_TEMPERATURE)  // Check if temperature is below the maximum limit
  {
    if (!active)  // If the respective element is not currently active
    {
      // Condition for rest time check of respective element, if rest time of 15 minutes for the particular heating element is completed and for that cycle of element have to be active
      if (elapsedTime >= REST_DURATION && cycleActive) 
      {
        Serial.print("REST_DURATION elapsedTime: ");
        Serial.println(elapsedTime);
        Serial.print("Pin: ");
        Serial.println(elementPin);
        cycleStartTime = millis();  // Start a new cycle
        active = true;  // Mark the element as active
        digitalWrite(elementPin, HIGH);  // Turn on the heating element
      }
    } 
    else  // If the element is currently active
    {
      // Condition for active time check of respective element, if active time of 45 minutes for the particular heating element is completed and for that cycle of element have to be active
      if (elapsedTime >= ACTIVE_DURATION && cycleActive) 
      {
        Serial.print("ACTIVE_DURATION elapsedTime: ");
        Serial.println(elapsedTime);
        Serial.print("Pin: ");
        Serial.println(elementPin);
        elementOnTime += round((double) elapsedTime/60000);
        Serial.print("Element ");
        Serial.print(elementNumber);
        Serial.print(" onTime: ");
        Serial.println(elementOnTime);
        writeIntIntoEEPROM(addr,elementOnTime);
        EEPROM.commit();
        cycleStartTime = millis();  // Start a new cycle
        active = false;  // Mark the element as inactive
        digitalWrite(elementPin, LOW);  // Turn off the heating element
      }
    }
  } 
  else if (temperature >= MAX_TEMPERATURE && temperature < FINAL_MAX_TEMPERATURE)  // If temperature exceeds the maximum limit
  {
    // If the respective element is active and it active duration of 45 minutes has completed
    if (active && elapsedTime >= ACTIVE_DURATION) 
    {
      elementOnTime += round((double) elapsedTime/60000);
      writeIntIntoEEPROM(addr,elementOnTime);
      EEPROM.commit();
      active = false;  // Mark the element as inactive
      cycleActive = false;  // Mark the cycle as inactive
      digitalWrite(elementPin, LOW);  // Turn off the heating element
      firstElementCheck = true;  // Allow the first element check again
      currentElement = (elementNumber % 4) + 1;  // increment the current element 
      // allElementsActive = false;  // Mark all elements as inactive
      Serial.print("ACTIVE_DURATION elapsedTime @60*C: ");
      Serial.println(elapsedTime);
      Serial.print("Element Number: ");
      Serial.println(elementNumber);
    } 
    else if (!active)
    {
      cycleActive = false;
    }
  }
  else if (temperature >= FINAL_MAX_TEMPERATURE)
  {
    if (active)
    {
      elementOnTime += round((double) elapsedTime/60000);
      writeIntIntoEEPROM(addr,elementOnTime);
      EEPROM.commit();
      active = false;  // Mark the element as inactive
      cycleActive = false;  // Mark the cycle as inactive
      digitalWrite(elementPin, LOW);  // Turn off the heating element
      firstElementCheck = true;  // Allow the first element check again
      currentElement = (elementNumber % 4) + 1;  // Reset the current element to the first one
      // allElementsActive = false;  // Mark all elements as inactive
      Serial.print("Pin Low @65*C: ");
      Serial.println(elementPin);
    }
    else if (!active)
    {
      cycleActive = false;
    }
  }
}