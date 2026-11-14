// Pin and address constants. Usable from Arduino and host g++.
#pragma once
#include <stdint.h>

static constexpr int FSR_COUNT = 6;
static constexpr uint8_t FSR_PINS[FSR_COUNT] = {1, 2, 4, 5, 6, 10};
// Per finger: indices into FSR_PINS.
static constexpr int FINGER_FSR_MAP[3][2] = {{5, 0}, {3, 2}, {4, 1}};
// Printed channel number for each FSR index.
static constexpr int FSR_CHANNEL_NUMBER[FSR_COUNT] = {2, 1, 3, 4, 5, 6};

static constexpr int SERVO_RX = 17;
static constexpr int SERVO_TX = 18;

static constexpr int I2C_SDA = 8;
static constexpr int I2C_SCL = 9;
static constexpr uint32_t I2C_HZ = 100000;

static constexpr int TOF_COUNT = 3;
static constexpr uint8_t TOF_ADDR[TOF_COUNT] = {0x33, 0x30, 0x31};
