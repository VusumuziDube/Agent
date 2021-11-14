#include <string.h>
#include <aJSON.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Specify the ID that is on the ARUCO tag of the robot.
//#define AGENT_ID    0x01 // For Robot 1
#define AGENT_ID    0x02 // For Robot 2
//#define AGENT_ID    0x05 // For Robot 3
///////////////////////////////////////////////////
//#include <I2Cdev.h>
//#include <MPU6050_6Axis_MotionApps20.h>
//
//// MPU control/status vars
//MPU6050 mpu;
//bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer
//
//// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
//    mpuInterrupt = true;
//}
///////////////////////////////////////////////////

#define PWMA   6           //Left Motor Speed pin (ENA)
#define AIN2   A0          //Motor-L forward (IN2).
#define AIN1   A1          //Motor-L backward (IN1)
#define PWMB   5           //Right Motor Speed pin (ENB)
#define BIN1   A2          //Motor-R forward (IN3)
#define BIN2   A3          //Motor-R backward (IN4)
#define PIN    7
#define ECHO   4
#define TRIG   3

//JSON
aJsonStream serial_stream(&Serial);

//==============================================
#define OLED_RESET 9
#define OLED_SA0   8
Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);


#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

static const unsigned char PROGMEM logo16_glcd_bmp[] =
{
  //  B00000000, B11000000,
  //  B00000001, B11000000,
  //  B00000001, B11000000,
  //  B00000011, B11100000,
  //  B11110011, B11100000,
  //  B11111110, B11111000,
  //  B01111110, B11111111,
  //  B00110011, B10011111,
  //  B00011111, B11111100,
  //  B00001101, B01110000,
  //  B00011011, B10100000,
  //  B00111111, B11100000,
  //  B00111111, B11110000,
  //  B01111100, B11110000,
  //  B01110000, B01110000,
  //  B00000000, B00110000
  0x00, 0x00, 0x00, 0x20, 0x00, 0x60, 0x00, 0xE0, 0x00, 0xE0, 0xEE, 0xCC, 0x7E, 0xCE, 0x7F, 0xDE,
  0x7F, 0xFF, 0x3F, 0xFB, 0x3B, 0x33, 0x11, 0x27, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0C, 0x00, 0x00
};
#define SSD1306_LCDHEIGHT 64
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void testdrawline();
void testdrawbitmap(const uint8_t *bitmap, uint8_t w, uint8_t h);
void testdrawchar(void);
void testdrawcircle(void);
void testfillrect(void);
void testdrawrect(void);
void testdrawroundrect(void);
void testfillroundrect(void);
void testdrawtriangle(void);
void testscrolltext(void);
void testfilltriangle(void);
//==============================================

#define beep_on  PCF8574Write(0xDF & PCF8574Read())
#define beep_off PCF8574Write(0x20 | PCF8574Read())
#define Addr  0x20

char comdata[128] = "";
int Speed = 40;
int distance = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int buzzer = 0;
byte LED1[3];
byte LED2[3];
byte LED3[3];
byte LED4[3];
byte LINE[5];
 
byte value;


uint16_t i, j;
unsigned long lasttime = 0;
unsigned long lastping = 0;
byte connection = 0;
byte flag = 1;
Adafruit_NeoPixel RGB = Adafruit_NeoPixel(4, PIN, NEO_GRB + NEO_KHZ800);

void PCF8574Write(byte data);
byte PCF8574Read();
int Distance_test();
void forward();
void backward();
void right();
void left();
void stop();
bool pair();

void readSensors();

uint32_t Wheel(byte WheelPos);

void setup() {
  // put your setup code here, to run once:
  //StaticJsonDocument<32> doc;
  delay(1500); // wait for Wi-Fi Module to boot up
  Serial.begin(115200);
  while(!Serial); 
  
  RGB.begin();
  RGB.show(); // Initialize all pixels to 'off'

  Wire.begin();


  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(ECHO, INPUT);    // Define the ultrasonic echo input pin
  pinMode(TRIG, OUTPUT);   // Define the ultrasonic trigger input pin
  analogWrite(PWMA, Speed);
  analogWrite(PWMB, Speed);

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  // display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //  display.clearDisplay();
  //  display.setTextSize(2);
  //  display.setTextColor(WHITE);
  //  display.fillRect(0, 0, display.width(), 16, WHITE);
  //  display.setCursor(15,20);
  //  RGB.show();
  //  display.print("Agent: ");
  //  display.println(AGENT_ID);
  //  display.display();


  Serial.println("Agent");
  while(!pair()){
    //beep_on;
    delay(500); // wait to pair with the server
  }
  
  pack();
  stop();
}

bool pair(){
  //do {
    readLine();
    Serial.print("{\"LINK\":");
    Serial.print(AGENT_ID);
    //Serial.print(comdata);
    Serial.print(", \"State\":");
    Serial.print(strstr(comdata, "LINK") != 0);
    Serial.println("}"); 
  //} while (strstr(comdata, "LINK") == 0);
  return strstr(comdata, "LINK");
}

byte CLEAR = 0;
byte ptr = 0;
bool readLine() {
  if (CLEAR){
      ptr = 0;
      comdata[ptr] = '\0';
      CLEAR = 0;
  }
  while (Serial.available() > 0)
  {
    char result = Serial.read();
    comdata[ptr++] = result;
    comdata[ptr] = '\0';
    Serial.print(result);
    if (result == '\n'){
      CLEAR = 1;
      return 1;
    }
  }
  return 0;
}
//{"LED1":[255,1,255],"LED2":[255,1,255],"LED4":[255,1,255],"LED3":[0,255,0]}
void updateLED(aJsonObject* Result, byte* LED, int target){
  for(int i = 0; i < 3; i++){
    aJsonObject* temp = aJson.getArrayItem(Result, i);
    Serial.println(temp->valueint);
    LED[i] = (byte) temp->valueint;
  }
  RGB.setPixelColor(target, RGB.Color(LED[0], LED[1], LED[2]));
  RGB.show();
}

int command_ptr = 0;
int readCommand(){
  if(readLine()){
    
    if (command_ptr = strstr(comdata, "pack") != 0) {
      return 1;
    }else{ 
      aJsonObject* msg = aJson.parse(comdata);
      aJsonObject* Result = 0;
      if (Result = aJson.getObjectItem(msg, "Buzzer")) {
        buzzer = Result->valueint;
        if(Result->valueint){
          beep_on; //buzzer = 1;
        }else{
          beep_off; //buzzer = 0;
        }
      }
      if (Result = aJson.getObjectItem(msg, "LED1")) {
        updateLED(Result, LED1, 0);
      }
      if (Result = aJson.getObjectItem(msg, "LED2")) {
        updateLED(Result, LED2, 1);
      }
      if (Result = aJson.getObjectItem(msg, "LED3")) {
        updateLED(Result, LED3, 2);
      }
      if (Result = aJson.getObjectItem(msg, "LED4")) {
        updateLED(Result, LED4, 3);
      }
      if (Result = aJson.getObjectItem(msg, "Dir")) {
        // {"Dir":"W"}
        switch(Result->valuestring[0]){
          case 'W': forward();    break;
          case 'S': backward();   break;
          case 'A': left();       break;
          case 'D': right();      break;
          default:  stop();       break;
        }
      }
      if (Result = aJson.getObjectItem(msg, "Speed")) {
        if (strcmp(Result->valuestring, "Low") == 0)       //Low
          Speed = 50;
        else if (strcmp(Result->valuestring, "Medium") == 0)    //Medium
          Speed = 150;
        else if (strcmp(Result->valuestring, "High") == 0)      //High
          Speed = 250;
      }
      return 1;
    }
    return 0; 
  }
}



void loop() {
  // put your main code here, to run repeatedly:
  //readSensors();
  delay(50);
  stop();
  //readSensors();
  pack();
  delay(300);
  Serial.print(comdata);
  if(readCommand()){
    ptr = 0;
    comdata[ptr] = '\0';
  }
    

  delay(50);
  //stop();
  if ((millis() - lasttime > 20) && 0 ) {
    lasttime = millis();
    for (i = 0; i < RGB.numPixels(); i++) {
      RGB.setPixelColor(i, Wheel(((i * 256 / RGB.numPixels()) + j) & 255));
    }
    if (flag)RGB.show();
    if (j++ > 256 * 5) j = 0;
  }
}

void addItem(char* key, int value, bool finish){
  Serial.print("\"");
  Serial.print(key);
  Serial.print("\":");
  Serial.print(value);
  if(!finish) Serial.print(",");
}

void addItemArray(char* key, byte* value, size_t siz, bool finish){
  Serial.print("\"");
  Serial.print(key);
  Serial.print("\":");
  Serial.print("[");
  for(int i; i < siz-1; i++){
    Serial.print(value[i]);
    Serial.print(",");
  }
  Serial.print(value[siz-1]);
  Serial.print("]");
  if(!finish) Serial.print(",");
}

void pack() {
  distance = Distance_test();                 //display distance
  distance = (2 < distance) && (distance < 400) ? distance : -1;
  Serial.print("{");
  addItem("link", AGENT_ID, false);
  addItem("LeftMotor", leftMotorSpeed, false);
  addItem("RightMotor", rightMotorSpeed, false);
  addItem("Ultrasonic", distance, false);
  addItem("Buzzer", buzzer, false);
  addItem("IR_Sensors", PCF8574Read() & 0xC0, false);
  addItem("Joystick", (PCF8574Read() | 0xE0) & 0x3F, false);
  addItemArray("LED1", LED1, 3, false);
  addItemArray("LED2", LED2, 3, false);
  addItemArray("LED3", LED3, 3, false);
  addItemArray("LED4", LED4, 3, false);
  addItemArray("LINE", LINE, 5, true);
  Serial.println("}");
}

void PCF8574Write(byte data)
{
  Wire.beginTransmission(Addr);
  Wire.write(data);
  Wire.endTransmission();
}

byte PCF8574Read()
{
  int data = -1;
  Wire.requestFrom(Addr, 1);
  if (Wire.available()) {
    data = Wire.read();
  }
  return data;
}

int Distance_test()         // Measure the distance
{
  digitalWrite(TRIG, LOW);   // set trig pin low 2μs
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);  // set trig pin 10μs , at last 10us
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);    // set trig pin low
  float Fdistance = pulseIn(ECHO, HIGH);  // Read echo pin high level time(us)
  Fdistance = Fdistance / 58;    //Y m=（X s*344）/2
  // X s=（ 2*Y m）/344 ==》X s=0.0058*Y m ==》cm = us /58
  return (int)Fdistance;
}

void forward()
{
  analogWrite(PWMA, Speed);
  analogWrite(PWMB, Speed);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void backward()
{
  analogWrite(PWMA, Speed);
  analogWrite(PWMB, Speed);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void right()
{
  analogWrite(PWMA, Speed);
  analogWrite(PWMB, Speed);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void left()
{
  analogWrite(PWMA, Speed);
  analogWrite(PWMB, Speed);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void stop()
{
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return RGB.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return RGB.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return RGB.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
