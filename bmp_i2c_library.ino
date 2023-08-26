#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <LiquidCrystal_I2C.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#define BMP180_ADDR 0x77
#define AC1 0xAA
#define AC2 0xAC
#define AC3 0xAE
#define AC4 0xB0
#define AC5 0xB2
#define AC6 0xB4
#define B1 0xB6
#define B2 0xB8
#define MB 0xBA
#define MD 0xBE
#define MC 0xBC

#define I2C_BITRATE 100000UL
#define I2C_PRESCALER 1

#define TWBR_VALUE (((F_CPU / I2C_BITRATE) - 16) / (2 * I2C_PRESCALER))

short  ac1;
short  ac2;
short  ac3;
unsigned short  ac4;
unsigned short  ac5;
unsigned short  ac6;
short  b1;
short  b2;
short  mb;
short  md;
short  mc;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void I2C_Init() {
    TWBR = TWBR_VALUE;            // I2C clock frequency = 16MHz / (16 + 2 * TWBR * (PrescalerValue))
    TWSR &= ~(1 << TWPS0);
    TWSR &= ~(1 << TWPS1);

    ac1 = i2c_read(AC1);
    ac2 = i2c_read(AC2);
    ac3 = i2c_read(AC3);
    ac4 = i2c_read(AC4);
    ac5 = i2c_read(AC5);
    ac6 = i2c_read(AC6);
    b1 = i2c_read(B1);
    b2 = i2c_read(B2);
    mb = i2c_read(MB);
    md = i2c_read(MD);
    mc = i2c_read(MC);
}

void I2C_Start() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void I2C_Stop() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    _delay_us(50);
}

void I2C_Write(uint8_t data) {
    TWDR = data;                                    // ТИПО ЧТОБЫ В ЭТОТ РЕГИСТР ПИСАТЬ
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t I2C_Read_ACK() {                                // ТИПО ЧТОБЫ C ЭТОГО РЕГИСТРА ЧИТАТЬ, ДОПИШИ БЛЯДЬ
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t I2C_Read_NACK() {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint16_t i2c_read(uint8_t reg){
  uint16_t value;
  I2C_Start();
  I2C_Write(BMP180_ADDR << 1);
  I2C_Write(reg);
  I2C_Start();
  I2C_Write((BMP180_ADDR << 1) | 0x01);
  value = (uint32_t)I2C_Read_ACK() << 8;
  value |= I2C_Read_NACK();
  I2C_Stop();
  return value;
}

void i2c_write(uint8_t reg, uint8_t data){
  I2C_Start();
    I2C_Write(BMP180_ADDR << 1);
    I2C_Write(reg);
    I2C_Write(data);
    I2C_Stop();
}

int32_t computeB5(int32_t UT){

  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1 + (int32_t)md);
  return X1 + X2;
}

uint16_t BMP180_ReadTemperature() {
    uint16_t value;
    // Write 0x2E to Measurment control register (0xF4)
    i2c_write(0xF4, 0x2E);
    _delay_ms(5);
    // Read from Out MSB and Out LSB registers
    value = i2c_read(0xF6);// + i2c_read(0xF6+1);
    
    //Serial.println(value);
    return value;
}

float BMP180_CalculateTemperature() {
    uint32_t ut = BMP180_ReadTemperature();
    int32_t B5 = computeB5(ut);
    float t = (B5+8)>>4;

    t/=10;
    return t;
} 

uint32_t BMP180_ReadPressure(){
  uint32_t value;
  i2c_write(0xF4, 0x34+(3 << 6));
  _delay_ms(26);

  value = i2c_read(0xF6);
  value <<= 8;
  value |= i2c_read(0xF6 + 2);
  value >>= (8 - 3);

  return value;
}

int32_t BMP180_CalculatePressure(){
  int32_t up, ut, B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  ut = BMP180_ReadTemperature();
  up = BMP180_ReadPressure();
  B5 = computeB5(ut);



  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1 * 4 + X3) <<3) + 2) / 4;

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)up - B3) * (uint32_t)(50000UL >> 3);

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + (int32_t)3791) >> 4);
  
  return p;
}

float readAltitude(float sealevelPressure){
  float altitude;

  float pressure = BMP180_CalculatePressure();

  altitude = 44330 * (1.0 - pow(pressure / sealevelPressure, 0.1903));

  return altitude;
}

int main(){
  I2C_Init();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 1);

  float temperature;
  float pressure;

  while(1){
    // Calculate temperature in degrees Celsius
    temperature = BMP180_CalculateTemperature();
    pressure = BMP180_CalculatePressure();
    lcd.clear();
    lcd.print("T: ");
    lcd.print(temperature);
    lcd.setCursor(0, 1);
    lcd.print("P: ");
    lcd.print(pressure);
    _delay_ms(1000);
  }

  return 0;
}

