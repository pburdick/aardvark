#include <Aardvark.h>

// Pin constants
const uint8_t LED_PIN = 4;
const uint8_t MOTOR_A_EN = 12;
const uint8_t MOTOR_B_EN = 13;
const uint8_t MOTOR_A = 0;
const uint8_t MOTOR_B = 1;
const uint8_t CURRENT_A = 24;
const uint8_t CURRENT_B = 25;
const uint8_t DIGITAL = 0;
const uint8_t ANALOG = 1;
const uint8_t COLOR_PIN = 21;
const uint8_t RED = 0;
const uint8_t BLUE = 1;
const uint8_t CLEAR = 2;
const uint8_t GREEN = 3;
const uint8_t SCALE_PD = 0;
const uint8_t SCALE_2 = 1;
const uint8_t SCALE_20 = 2;
const uint8_t SCALE_100 = 3;
const uint8_t SERVO_POWER = 18;
const uint8_t SERVO_0_PIN = 20;
const uint8_t SERVO_1_PIN = 19;
const uint8_t BATTERY_PIN = 26;
const uint8_t PIEZO_PIN = 15;
const uint8_t SONAR_0_PIN = 23;
const uint8_t SONAR_1_PIN = 1;
const uint8_t VOLTS = 0;
const uint8_t RAW_ADC = 1;
const float   VOLT_CONV = 70.83;

// Real-Time Clock constants
const uint8_t RTC_ADDRESS = 0x6F;
const uint8_t RTC_SEC = 0x00;
const uint8_t RTC_CONTROL = 0x07;

// EEPROM constants
const uint8_t EEPROM_ADDRESS = 0x50;

// Digital thermometer
const uint8_t THERM_ADDRESS = 0x48;
const uint8_t THERM_TEMP = 0;
const uint8_t THERM_RESO = 1;
const uint8_t CELSIUS = 0;
const uint8_t FAHRENHEIT = 1;

// I/O expander constants
const uint8_t IO_ADDRESS = 0x20;
const uint8_t IODIRA = 0x00;
const uint8_t IODIRB = 0x01;
const uint8_t GPIOA = 0x12;
const uint8_t GPIOB = 0x13;
const uint8_t GPPUA = 0x0C;

// Accelerometer constants
const uint8_t ACCEL_ADDRESS = 0x15;
const uint8_t AXIS_X = 0x00;
const uint8_t AXIS_Y = 0x01;
const uint8_t ACCEL_SHOOK = 0x02;
const uint8_t ACCEL_DETECT = 0x04;

// Sonar constants
const uint8_t CM = 0;
const uint8_t INCH = 1;
const uint8_t RAW_US = 2;

Servo servo0;
Servo servo1;

//Constructor
Aardvark::Aardvark(){}

//Destructor
Aardvark::~Aardvark(){}

//Public methods
void Aardvark::init(){
    Wire.begin();
    Wire.setClock(400000); //Use fast i2c
    ADCSRA = ADCSRA & 0b11111000; //Increase ADC speed
    ADCSRA = ADCSRA | 0b100; // by changing prescaler to /16

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); //Blink the LED
    delay(100);
    digitalWrite(LED_PIN, LOW);

    //Initialize the realtime clock
    i2cWrite(RTC_ADDRESS, RTC_SEC, 0x80); // Enable the ST bit to start oscillator
    setAlert(1); // Blink the alert LED
    delay(100);
    setAlert(0);

    //Initialize the I/O expander and the motors
    analogWrite(MOTOR_A_EN, 0); //Disable the motor A bridge
    analogWrite(MOTOR_B_EN, 0); //Disable the motor B bridge
    i2cWrite(IO_ADDRESS, IODIRA, 0xFF); // Set port A to all inputs
    i2cWrite(IO_ADDRESS, IODIRB, 0x00); // Set port B to all outputs
    i2cWrite(IO_ADDRESS, GPIOB, 0x00); // Set port B bits to 0

    //Initialize servos
    pinMode(SERVO_POWER, OUTPUT);
    digitalWrite(SERVO_POWER, LOW); //Turn off the 6v power supply
    servo0.attach(SERVO_0_PIN);
    servo1.attach(SERVO_1_PIN);

    //Initialize temperature sensor to 12-bit
    i2cWrite(THERM_ADDRESS, THERM_RESO, byte(0x60));
}

void Aardvark::motorSpeed(uint8_t motor, int speed){
    int speedAbs = abs(speed);
    if (256 > speedAbs && (motor == MOTOR_A || motor == MOTOR_B)){ //Check parameters
        uint8_t ioLatchB = i2cRead(IO_ADDRESS, GPIOB);
        uint8_t mask = 0b00; // Stop mask
        if ( speed < 0 ){ mask = 0b10; } // Change mask to reverse
        if ( speed > 0){ mask = 0b01; } // or Change mask to forward
        if ( motor == MOTOR_A ){
            analogWrite(MOTOR_A_EN, 0); // Disable bridges
            ioLatchB = ioLatchB & 0b11111100; // Clear bits for motor A
            ioLatchB = ioLatchB | mask; // Or in direction
            i2cWrite(IO_ADDRESS, GPIOB, ioLatchB);
            analogWrite(MOTOR_A_EN, speedAbs);
        } else if ( motor == MOTOR_B ){
            analogWrite(MOTOR_B_EN, 0); // Disable bridges
            ioLatchB = ioLatchB & 0b11110011; // Clear bits for motor B
            ioLatchB = ioLatchB | ( mask << 2 ); // Or in direction
            i2cWrite(IO_ADDRESS, GPIOB, ioLatchB);
            analogWrite(MOTOR_B_EN, speedAbs);
        }
    }
}

void Aardvark::motorBrake(uint8_t motor){
    if ( motor == MOTOR_A || motor == MOTOR_B ){ //Check parameters
        int ioLatchB = i2cRead(IO_ADDRESS, GPIOB);
        if ( motor == MOTOR_A ){
            ioLatchB = ioLatchB & 0b11111100; // Clear bits for motor A
            i2cWrite(IO_ADDRESS, GPIOB, ioLatchB);
        } else if ( motor == MOTOR_B ){
            ioLatchB = ioLatchB & 0b11110011; // Clear bits for motor B
            i2cWrite(IO_ADDRESS, GPIOB, ioLatchB);
        }
    }
}

int Aardvark::motorCurrent(uint8_t motor){
    unsigned int motor_current = 0;
    if (motor == MOTOR_A){
        motor_current = analogRead(CURRENT_A);
    } else if (motor == MOTOR_B){
        motor_current = analogRead(CURRENT_B);
    }
    return motor_current;
}

float Aardvark::getBattery(uint8_t mode){
    float value = 0;
    if (mode == VOLTS || mode == RAW_ADC){
        value = analogRead(BATTERY_PIN);
        if (mode == VOLTS){
            value = value / VOLT_CONV;
        }
    }
    return value;
}

float Aardvark::getTemp(uint8_t unit){
    uint8_t tempMSB;
    uint8_t tempLSB;
    uint8_t fraction;
    unsigned int raw;
    float degrees;
    bool negFlag = false;
    Wire.beginTransmission(THERM_ADDRESS);
    Wire.write(byte(0));
    Wire.endTransmission();
    Wire.requestFrom(THERM_ADDRESS, byte(2));
    tempMSB = Wire.read();
    tempLSB = Wire.read();
    raw = tempMSB * 256 + tempLSB;
    if (bitRead(raw, 15) == 1){ //Negative temp--2's complement
        raw = (~raw) + 1;
        negFlag = true;
    }
    fraction = (raw & 0xF0) >> 4;
    degrees = (raw >> 8);
    degrees = degrees + (.0625 * fraction);
    if ( negFlag ){ degrees = -degrees; }
    if (unit == FAHRENHEIT){ degrees = 1.80 * degrees + 32; }
    return degrees;
}

void Aardvark::writeEEPROM(unsigned long address, uint8_t value){
    uint8_t device;
    uint8_t addressMSB;
    uint8_t addressLSB;
    device = EEPROM_ADDRESS | bitRead(address, 16);
    addressMSB = ( address & 0xFF00 ) >> 8;
    addressLSB = ( address & 0x00FF );
    Wire.beginTransmission(device);
    Wire.write(addressMSB);
    Wire.write(addressLSB);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t Aardvark::readEEPROM(unsigned long address){
    uint8_t numBytes;
    int device;
    uint8_t addressMSB;
    uint8_t addressLSB;
    uint8_t value = 0;
    device = EEPROM_ADDRESS | bitRead(address, 16);
    addressMSB = ( address & 0xFF00 ) >> 8;
    addressLSB = ( address & 0x00FF );
    Wire.beginTransmission(device);
    Wire.write(addressMSB);
    Wire.write(addressLSB);
    Wire.endTransmission();
    numBytes = Wire.requestFrom(device, 1);
    if (numBytes == 1) { value = Wire.read(); }
    return value;
}

void Aardvark::setAlert(uint8_t mode){
    switch(mode){
        case 0: // Off
            i2cWrite(RTC_ADDRESS, RTC_CONTROL, 0x80);
            break;
        case 1: // On
            i2cWrite(RTC_ADDRESS, RTC_CONTROL, 0x00);
            break;
        case 2: // Blink
            i2cWrite(RTC_ADDRESS, RTC_CONTROL, 0x40);
    }
}

void Aardvark::setLed(uint8_t value, uint8_t mode){
    if ( mode == DIGITAL && (value == 0 || value == 1) ){
        digitalWrite(LED_PIN, value);
    } else if ( mode == ANALOG && (value >= 0 && value <= 255) ){
        analogWrite(LED_PIN, value);
    }
}

void Aardvark::setServo(uint8_t motor, uint8_t dir){
    if (( motor == 0 || motor == 1 ) && ( dir >= 0 && dir <= 180 )){ // Check parameters
        digitalWrite(SERVO_POWER, 1); //Turn on the 6v servo power
        if (motor == 0){
            servo0.write(dir);
        } else {
            servo1.write(dir);
        }
    }
}

void Aardvark::setServoPower(uint8_t value){
    if (value == 0 || value == 1){
        pinMode(SERVO_POWER, OUTPUT);
        digitalWrite(SERVO_POWER, value);
    }
}

int8_t Aardvark::getTilt(uint8_t axis){
    int8_t tilt = 0;
    if (axis == AXIS_X || axis == AXIS_Y){
        tilt = i2cRead(ACCEL_ADDRESS, axis);
        if ( bitRead(tilt, 7) == 1 ){ //If negative number...
            tilt = -byte(~tilt + 1); //convert 2's complement
        }
    }
    return tilt;
}

bool Aardvark::isShook(){
    bool shake = true;
    uint8_t shakeEvent = i2cRead(ACCEL_ADDRESS, ACCEL_SHOOK);
    shakeEvent = shakeEvent & 0x60;
    if (shakeEvent == 0){ shake = false; }
    return shake;
}

unsigned long Aardvark::getColor(uint8_t color, uint8_t scale){
    unsigned long reading = 0;
    uint8_t mask;
    uint8_t ioLatchB;
    if ((color >=0 && color <= 3) && (scale >=0 && scale <= 3)){
        ioLatchB = i2cRead(IO_ADDRESS, GPIOB);
        ioLatchB = ioLatchB & 0b00001111; //Clear color sensor bits
        mask = ((scale << 2) | color) << 4; //Set up mask
        ioLatchB = ioLatchB | mask;
        i2cWrite(IO_ADDRESS, GPIOB, ioLatchB); // Move to I/O expander
        if (scale != SCALE_PD){
            reading = pulseIn(COLOR_PIN, HIGH);
        }
    }
    return reading;
}

unsigned long Aardvark::getSonar(uint8_t sonar, uint8_t unit){
    unsigned long echo = 0;
    if ((sonar == 0 || sonar == 1) && (unit >= CM && unit <= RAW_US)){ //Check parameters
        uint8_t pin[] = {SONAR_0_PIN, SONAR_1_PIN}; //Map Arduino pins to sonar pins
        sonarTrigger(sonar);
        echo = pulseIn(pin[sonar], HIGH); //and measure the length of the echo return
        if (unit == CM){ //Convert echo time (microseconds) to centimeters
            echo = echo / 29 / 2;
        } else if (unit == INCH){ //Convert echo time to inches
            echo = echo / 74 / 2;
        }
    }
    return echo;
}

// Set port A direction, 1 is in, 0 is out
void Aardvark::setIoDir(uint8_t dir){
    i2cWrite(IO_ADDRESS, IODIRA, dir);
}

void Aardvark::writeIo(uint8_t value){
    i2cWrite(IO_ADDRESS, GPIOA, value);
}

uint8_t Aardvark::readIo(){
    return i2cRead(IO_ADDRESS, GPIOA);
}

void Aardvark::setIoPullUp(uint8_t value){
    i2cWrite(IO_ADDRESS, GPPUA, value);
}

//Private methods
void Aardvark::sonarTrigger(uint8_t sonar){
    // Sonar 0 is on PC7 and sonar 1 is on PB1
    switch(sonar){
        case 0:
            DDRC |= 0b10000000; //Set PC7 to output
            PORTC &= 0b01111111; //Set PC7 to 0
            delayMicroseconds(2); //for 2 microseconds
            PORTC |= 0b10000000; //Set PC7 to 1 for
            delayMicroseconds(5); //5 microsecond pulse
            PORTC &= 0b01111111; //Set PC7 back to 0
            DDRC &= 0b01111111; //and flip PC7 back to input
            break;
        case 1:
            DDRB |= 0b00000010; //Set PB1 to output
            PORTB &= 0b11111101;//Set PB1 to 0
            delayMicroseconds(2);//for 2 microseconds
            PORTB |= 0b00000010;//Set PB1 to 1
            delayMicroseconds(5);//for 5 microsecond pulse
            PORTB &= 0b11111101;//Set PB1 back to 0
            DDRB &= 0b11111101;//and flip PB1 back to input
    }
}

void Aardvark::i2cWrite(uint8_t device, uint8_t address, uint8_t value){
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t Aardvark::i2cRead(int device, uint8_t address){
    uint8_t value = 0;
    uint8_t numBytes;
    Wire.beginTransmission(device);
    Wire.write(address);
    Wire.endTransmission();
    numBytes = Wire.requestFrom(device, 1);
    if (numBytes == 1) { value = Wire.read(); }
    return value;
}
