#ifndef Aardvark_H
#define Aardvark_H
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// Pin constants
extern const uint8_t LED_PIN;
extern const uint8_t MOTOR_A_EN;
extern const uint8_t MOTOR_B_EN;
extern const uint8_t MOTOR_A;
extern const uint8_t MOTOR_B;
extern const uint8_t CURRENT_A;
extern const uint8_t CURRENT_B;
extern const uint8_t DIGITAL;
extern const uint8_t ANALOG;
extern const uint8_t COLOR_PIN;
extern const uint8_t RED;
extern const uint8_t BLUE;
extern const uint8_t CLEAR;
extern const uint8_t GREEN;
extern const uint8_t SCALE_PD;
extern const uint8_t SCALE_2;
extern const uint8_t SCALE_20;
extern const uint8_t SCALE_100;
extern const uint8_t SERVO_POWER;
extern const uint8_t SERVO_0_PIN;
extern const uint8_t SERVO_1_PIN;
extern const uint8_t BATTERY_PIN;
extern const uint8_t PIEZO_PIN;
extern const uint8_t VOLTS;
extern const uint8_t RAW_ADC;

// Real-Time Clock extern constants
extern const uint8_t RTC_ADDRESS;
extern const uint8_t RTC_SEC;
extern const uint8_t RTC_CONTROL;

// EEPROM extern constants
extern const uint8_t EEPROM_ADDRESS;

// Digital thermometer
extern const uint8_t THERM_ADDRESS;
extern const uint8_t THERM_TEMP;
extern const uint8_t THERM_RESO;
extern const uint8_t CELSIUS;
extern const uint8_t FAHRENHEIT;

// I/O expander extern constants
extern const uint8_t IO_ADDRESS;
extern const uint8_t IODIRA;
extern const uint8_t IODIRB;
extern const uint8_t GPIOA;
extern const uint8_t GPIOB;
extern const uint8_t GPPUA;

// Accelerometer extern constants
extern const uint8_t ACCEL_ADDRESS;
extern const uint8_t AXIS_X;
extern const uint8_t AXIS_Y;
extern const uint8_t ACCEL_SHOOK;
extern const uint8_t ACCEL_DETECT;

// Sonar constants
extern const uint8_t CM;
extern const uint8_t INCH;
extern const uint8_t RAW_US;

class Aardvark {
private:
    void sonarTrigger(uint8_t);
    void i2cWrite(uint8_t, uint8_t, uint8_t);
    uint8_t i2cRead(int, uint8_t);

public:
    Aardvark();
    ~Aardvark();
    void init();
    void motorSpeed(uint8_t, int);
    void motorBrake(uint8_t);
    int motorCurrent(uint8_t);
    float getBattery(uint8_t);
    int8_t getTilt(uint8_t);
    unsigned long getColor(uint8_t, uint8_t);
    unsigned long getSonar(uint8_t, uint8_t);
    bool isShook();
    float getTemp(uint8_t);
    void writeEEPROM(unsigned long, uint8_t);
    uint8_t readEEPROM(unsigned long);
    void setAlert(uint8_t);
    void setLed(uint8_t, uint8_t);
    void setServo(uint8_t, uint8_t);
    void setServoPower(uint8_t);
    void setIoDir(uint8_t);
    void writeIo(uint8_t);
    void setIoPullUp(uint8_t);
    uint8_t readIo();
};

#endif
