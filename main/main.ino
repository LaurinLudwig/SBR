#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>

// neopixel debug signals
// slow purple blink  - SSD1306 init failed
// slow red    blink  - MPU6050 init failed
// slow orange blink  - Hoverserial init failed
// fast green  blink  - all inits passed


// ################ Hardware and Communication Definitions ################
// defines for neopixel
#define PIN         13
#define NUMPIXELS   1

// defines for SSD1306
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
#define MAX_OLD_MESSAGES ((SCREEN_HEIGHT - 16) / 8) // Die unterste Zeile ist für die neueste Nachricht reserviert
#define MAX_MESSAGE_LENGTH (SCREEN_WIDTH / 6) // Da jeder Charakter etwa 6 Pixel breit ist
// Array zur Speicherung der alten Nachrichten
String messages[MAX_OLD_MESSAGES];
int messageCount = 0;

// defines for hoverserial
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      250         // [-] Maximum speed for testing
#define SPEED_STEP          10  
#define TXD_PIN 17
#define RXD_PIN 16

// defines for Serial Monitor
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)

// Global variables for hoverserial
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// PID Loop Variables
float Kp = 25;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.0; // Derivative gain

float setPoint = -1.0;   // Desired angle (upright position)
float integral = 0;   // Integral term
float previousError = 0; // Previous error for derivative calculation

unsigned long previousTime = 0;

Adafruit_MPU6050 mpu;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HardwareSerial HoverSerial(1);

// ########################## SETUP ##########################
void setup() {
  // init Serial Monitor
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // init neopixel
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(50, 50, 50));
  pixels.show();
  delay(200);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  Serial.println("neopixel init passed");

  // init SSD1306
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed!"));
    while(1){
      pixels.setPixelColor(0, pixels.Color(100, 0, 100));
      pixels.show();
      delay(500);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
      delay(500);
    }
  }
  Serial.println("ssd1306 init passed");
  display.clearDisplay();
  display.drawPixel(10, 10, SSD1306_WHITE);
  display.display();
  delay(500);
  display.clearDisplay();
  display.display();

  // init mpu
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) {
      pixels.setPixelColor(0, pixels.Color(100, 0, 0));
      pixels.show();
      delay(500);
      pixels.setPixelColor(0, pixels.Color(100, 0, 0));
      pixels.show();
      delay(500);
    }
  }
  Serial.println("mpu init passed");
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  // init hoverserial
  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD_PIN, TXD_PIN);
  if(!HoverSerial){
    Serial.println("Failed to initialize Hoverserial connection!");
    while(1){
      pixels.setPixelColor(0, pixels.Color(100, 50, 0));
      pixels.show();
      delay(500);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
      delay(500);
    }
  }
  Serial.println("Hoverserial init passed");

  // all inits passed
  Serial.println("--------- all inits passed ---------");
  pixels.setPixelColor(0, pixels.Color(0, 150, 0));
  pixels.show();
  delay(200);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  delay(200);
  pixels.setPixelColor(0, pixels.Color(0, 150, 0));
  pixels.show();
  delay(200);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  delay(200);
}

// ########################## Functions ##########################

// Hoverserial Functions
// SEND HOVERSERIAL
void Send(int16_t uSteer, int16_t uSpeed){
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// RECEIVE HOVERSERIAL
void Receive(){
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            //Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas * -1);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            //Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            //Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            //Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
            Serial.println();
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

// SSD1306 Functions
// Add a new Message to the Display
void addMessage(String newMessage) {
  // Kürze die Nachricht, wenn sie zu lang ist
  if (newMessage.length() > MAX_MESSAGE_LENGTH) {
    newMessage = newMessage.substring(0, MAX_MESSAGE_LENGTH);
  }

  // Wenn die maximale Anzahl der alten Nachrichten erreicht ist, verschiebe die Nachrichten nach oben
  if (messageCount >= MAX_OLD_MESSAGES) {
    for (int i = 0; i < MAX_OLD_MESSAGES - 1; i++) {
      messages[i] = messages[i + 1];
    }
    messages[MAX_OLD_MESSAGES - 1] = newMessage;
  } else {
    messages[messageCount] = newMessage;
    messageCount++;
  }

  // Zeige die Nachrichten auf dem Display an
  displayMessages();
}

// Display the Messages
void displayMessages() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Zeichne die alten Nachrichten bis zur Linie
  for (int i = 0; i < messageCount - 1; i++) {
    display.setCursor(0, i * 8); // Setze den Cursor auf die entsprechende Zeile
    display.println(messages[i]);
    if ((i + 1) * 8 >= SCREEN_HEIGHT - 16) break; // Stoppe, wenn die Linie erreicht ist
  }

  // Zeichne die Trennlinie
  display.drawLine(0, SCREEN_HEIGHT - 17, SCREEN_WIDTH, SCREEN_HEIGHT - 17, SSD1306_WHITE);

  // Zeichne die neue Nachricht nur unter der Linie
  if (messageCount > 0) {
    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.println(messages[messageCount - 1]);
  }

  display.display();
}

// Neopixel Functions
//Blink
void Blink(int times = 1, uint32_t color = pixels.Color(255, 255, 255), int interval = 500) {
  for (int i = 0; i < times; i++) {
    pixels.setPixelColor(0, color);
    pixels.show();
    delay(interval);

    pixels.setPixelColor(0, color);
    pixels.show();
    delay(interval);
  }
}

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate the error (current angle - desired angle)
  float error = a.acceleration.z - setPoint;

  // Calculate integral term
  integral += error;

  // Calculate derivative term
  float derivative = (error - previousError);

  // Calculate the output from the PID formula
  float output = (Kp * error + Ki * integral + Kd * derivative) * 1; // Scale the output

  // Save the current error as the previous error for the next loop
  previousError = error;

  // Send the output as speed command to the motors
  Send(0, (int16_t)output);

  // Receive feedback
  Receive();

  // Display the current speed on the OLED
  addMessage(String((int16_t)output));

  // Wait for the next loop
  //delay(5);
  unsigned long timeNow = millis();
  while (timeNow < previousTime + 10){
    delay(1);
    timeNow = millis();
  }
  previousTime = timeNow;
}
