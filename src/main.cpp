#include <Arduino.h>
#include <Wire.h>
#include "regs.h"

void i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t i2c_read_byte(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.requestFrom(addr, 1);
  uint8_t ret = Wire.read();
  Wire.endTransmission();
  return ret;
}

uint32_t i2c_read_20bit(uint8_t addr, uint8_t start_reg) {
  uint32_t ret = 0;
  uint8_t lsb = i2c_read_byte(addr, start_reg);
  uint8_t middle = i2c_read_byte(addr, start_reg+1);
  uint8_t msb = i2c_read_byte(addr, start_reg+2);
  ret = (uint32_t)lsb;
  ret |= (uint32_t)middle << 8;
  ret |= (uint32_t)msb << 16;
  ret &= 0x000FFFFF; // mask off top 12 bits (they're unused)
  return ret;
}

uint16_t i2c_read_12bit(uint8_t addr, uint8_t start_reg) {
  uint16_t ret;
  uint8_t lsb = i2c_read_byte(addr, start_reg);
  uint8_t msb = i2c_read_byte(addr, start_reg+1);
  ret = (uint32_t)lsb;
  ret |= (uint32_t)msb << 8;
}

void configure_color_sensor(int address) {
  uint8_t mcByte = 0;
  mcByte |= BIT_MAIN_CTRL_LS_EN;
  mcByte |= BIT_MAIN_CTRL_PS_EN;
  mcByte |= BIT_MAIN_CTRL_RGB_MODE;
  i2c_write_byte(I2C_ADDR, REG_MAIN_CTRL, mcByte);
  // Defaults are fine, but you may want to set: PS resolution, PS meas rate, LS resolution, LS meas rate, LS gain
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  configure_color_sensor(I2C_ADDR);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t ir = i2c_read_20bit(I2C_ADDR, REG_LS_DATA_IR0);
  uint32_t r = i2c_read_20bit(I2C_ADDR, REG_LS_DATA_RED0);
  uint32_t g = i2c_read_20bit(I2C_ADDR, REG_LS_DATA_GREEN0);
  uint32_t b = i2c_read_20bit(I2C_ADDR, REG_LS_DATA_BLUE0);
  uint16_t ps = i2c_read_12bit(I2C_ADDR, REG_PS_DATA0);
  bool ps_overflow = (ps >> 11) & 0x01;
  ps = ps & (0xFFFF >> 5); // chop off 12th bit. 

  Serial.print("IR: ");
  Serial.print(ir);
  Serial.print(", R: ");
  Serial.print(r);
  Serial.print(", G: ");
  Serial.print(g);
  Serial.print(", B: ");
  Serial.print(b);
  Serial.print(", PS: ");
  Serial.print(ps);
  Serial.print(", PSO: ");
  Serial.print(ps_overflow);
}