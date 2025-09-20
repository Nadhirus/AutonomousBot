# ZU32U4 Wall-Following + Dead-Reckoning Mapping System  

This project implements a **wall-following and dead-reckoning navigation algorithm** on a **ZU32U4-based robot** to build and transmit a binary occupancy grid map. A companion Python script visualizes the map in real time.  

---

## Features  
- **Wall-Following Navigation:** The robot hugs walls using IR/ultrasonic sensors to systematically explore an area.  
- **Dead Reckoning:** Tracks position based on encoder and heading data to estimate the robot’s location in a 2D grid.  
- **Occupancy Grid Mapping:** Builds a binary map (0 = free, 1 = occupied) of the explored environment.  
- **Real-Time Visualization:** Streams occupancy data to a Python script for live display.  

---

## Hardware  
- **ZU32U4 board** (ATmega32U4-based)  
- Motor driver (L298N or equivalent)  
- Wheel encoders for odometry  
- IR or ultrasonic sensors for wall detection  
- Power source (LiPo or NiMH battery pack)  

---

## Software  
- **Firmware:** Written in C/C++ (Arduino-compatible for ATmega32U4)  
- **Communication:** Serial (USB/UART) sends occupancy map data to the host PC  
- **Visualizer:** Python 3 script using `matplotlib` or `pygame` for rendering the map  


## API to use low level robot functions

```c
// Zumo32U4.h -- general standalone functions
void ledRed(bool on);
void ledGreen(bool on);
void ledYellow(bool on);
bool usbPowerPresent();
uint16_t readBatteryMillivolts();

// Zumo32U4Encoders — quadrature encoders
static void init();
static int16_t getCountsLeft();
static int16_t getCountsRight();
static int16_t getCountsAndResetLeft();
static int16_t getCountsAndResetRight();
static bool checkErrorLeft();
static bool checkErrorRight();

// Zumo32U4IMU — inertial measurement unit
uint8_t getLastError();
bool init();
Zumo32U4IMUType getType();
void enableDefault();
void configureForTurnSensing();
void configureForBalancing();
void configureForFaceUphill();
void writeReg(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t readReg(uint8_t addr, uint8_t reg);
void readAcc();
void readGyro();
void readMag();
void read();
bool accDataReady();
bool gyroDataReady();
bool magDataReady();
// Public attributes (vectors): a, g, m of type vector<int16_t>

// Zumo32U4Motors
void setSpeeds(int16_t leftSpeed, int16_t rightSpeed);
void setLeftSpeed(int16_t speed);
void setRightSpeed(int16_t speed);

// Zumo32U4LineSensors
Zumo32U4LineSensors(uint8_t *pins, uint8_t numSensors, uint8_t emitterPin = SENSOR_LEDON);
void init(uint8_t *pins, uint8_t numSensors, uint16_t timeout = 2000, uint8_t emitterPin = SENSOR_LEDON);
void initThreeSensors(uint8_t emitterPin = SENSOR_LEDON);
void initFiveSensors(uint8_t emitterPin = SENSOR_LEDON);
void read(unsigned int *sensor_values, unsigned char readMode = QTR_EMITTERS_ON);
void emittersOn();
void emittersOff();
void calibrate(unsigned char readMode = QTR_EMITTERS_ON);
void resetCalibration();
void readCalibrated(unsigned int *sensor_values, unsigned char readMode = QTR_EMITTERS_ON);
int readLine(unsigned int *sensor_values, unsigned char readMode = QTR_EMITTERS_ON, unsigned char white_line = 0);

// Zumo32U4ProximitySensors
Zumo32U4ProximitySensors(uint8_t *pins, uint8_t numSensors, uint8_t lineSensorEmitterPin = defaultLineSensorEmitterPin);
void init(uint8_t *pins, uint8_t numSensors, uint8_t lineSensorEmitterPin = defaultLineSensorEmitterPin);
void initThreeSensors(uint8_t lineSensorEmitterPin = defaultLineSensorEmitterPin);
void initFrontSensor(uint8_t lineSensorEmitterPin = defaultLineSensorEmitterPin);
uint8_t getNumSensors() const;
void setPeriod(uint16_t period);
void setBrightnessLevels(uint16_t *levels, uint8_t levelCount);
void setPulseOnTimeUs(uint16_t pulseOnTimeUs);
void setPulseOffTimeUs(uint16_t pulseOffTimeUs);
uint8_t getNumBrightnessLevels() const;
void lineSensorEmittersOff();
void read();
bool readBasic(uint8_t sensorNumber);
bool readBasicLeft();
bool readBasicFront();
bool readBasicRight();
uint8_t countsWithLeftLeds(uint8_t sensorNumber) const;
uint8_t countsWithRightLeds(uint8_t sensorNumber) const;
uint8_t countsLeftWithLeftLeds() const;
uint8_t countsLeftWithRightLeds() const;
uint8_t countsFrontWithLeftLeds() const;
uint8_t countsFrontWithRightLeds() const;
uint8_t countsRightWithLeftLeds() const;
uint8_t countsRightWithRightLeds() const;

// Zumo32U4LCD (Character LCD)
void init();
void reinitialize();
void initPins();
void send(uint8_t data, bool rsValue, bool only4bits);
void clear();
void loadCustomCharacter(const uint8_t *picture, uint8_t number);
void loadCustomCharacter(const char *picture, uint8_t number);
void loadCustomCharacterFromRam(const uint8_t *picture, uint8_t number);
void gotoXY(uint8_t x, uint8_t y);
void setCursor(uint8_t col, uint8_t row);
void noDisplay();
void display();
void noCursor();
void cursor();
void noBlink();
void blink();
void cursorSolid();
void cursorBlinking();
void hideCursor();
void scrollDisplayLeft();
void scrollDisplayRight();
void home();
void leftToRight();
void rightToLeft();
void autoscroll();
void noAutoscroll();
void command(uint8_t cmd);
size_t write(uint8_t c);
size_t write(const uint8_t *buffer, size_t size);

// Zumo32U4OLED (Graphical OLED)
void init();
void reinitialize();
void invert();
void noInvert();
void rotate180();
void noRotate();
void setContrast(uint8_t contrast);
void setLayout8x2();
void setLayout8x2WithGraphics(const uint8_t *graphics);
void setLayout11x4();
void setLayout11x4WithGraphics(const uint8_t *graphics);
void setLayout21x8();
void setLayout21x8WithGraphics(const uint8_t *graphics);
void display();
void displayPartial(uint8_t x, uint8_t y, uint8_t width);
void noAutoDisplay();
uint8_t *getLinePointer(uint8_t line);
void gotoXY(uint8_t x, uint8_t y);
uint8_t getX();
uint8_t getY();
size_t write(const uint8_t *buffer, size_t size);
size_t write(uint8_t d);
void loadCustomCharacterFromRam(const uint8_t *picture, uint8_t number);
void loadCustomCharacter(const uint8_t *picture, uint8_t number);
void loadCustomCharacter(const char *picture, uint8_t number);

// Zumo32U4Buzzer
static void playFrequency(unsigned int freq, unsigned int duration, unsigned char volume);
static void playNote(unsigned char note, unsigned int duration, unsigned char volume);
static void play(const char *sequence);
static void playFromProgramSpace(const char *sequence);
static void playMode(unsigned char mode);
static unsigned char playCheck();
static unsigned char isPlaying();
static void stopPlaying();

// Zumo32U4IRPulses
void start(uint16_t period, uint16_t dutyCycleLeft, uint16_t dutyCycleRight, uint16_t count);
void stop();
```