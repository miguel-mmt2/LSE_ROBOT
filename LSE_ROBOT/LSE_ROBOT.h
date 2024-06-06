#ifndef LSE_ROBOT_H
#define LSE_ROBOT_H

// ===================================== Required libraries ===================================== 
#include <Arduino.h>

// WiFi Communication
#include <WiFi.h>
#include <WebServer.h>

// SPI Communication
#include <SPI.h>

// I2C Communication
#include <Wire.h>

// Sensor: RFID Tag Reader
#include <MFRC522.h>

// Sensor: Lidar Middle Sensor
#include <VL53L0X.h> 

// Sensor: Gas Sensor
#include <ADS1X15.h>

// ===================================== Class LSE_ROBOT =====================================
class LSE_ROBOT{
    private:
// ------------------------------------- Sensor Pins -------------------------------------
// RFID Tag Reader (MFRC522) - Pins
// static constexpr uint8_t PIN_RFID_MISO = 19;
// static constexpr uint8_t PIN_RFID_MOSI = 23;
// static constexpr uint8_t PIN_RFID_SCLK = 18;
// static constexpr uint8_t PIN_RFID_SS = 5;

static MFRC522 RFID_TAG_READER;

// LIDAR Sensor - Pins
// Sensor 1 - LIDAR (Sharp GP2Y0A21Y) - Pins
static constexpr uint8_t PIN_LIDAR_RIGHT_VP = 36;

// Sensor 2 - LIDAR (VL53L0X-V2) - Pins
static constexpr uint8_t PIN_XSHUT_MIDDLE = 17;
static constexpr uint8_t PIN_LIDAR_MIDDLE_SDA = 32;
static constexpr uint8_t PIN_SCL_MIDDLE = 33;
static constexpr uint8_t I2C_ADDRESS_LIDAR_MIDDLE = 0x29; // I2C Address

static VL53L0X LIDAR_MIDDLE;

       
// Sensor 3 - LIDAR (Sharp GP2Y0A21Y) - Pins
static constexpr uint8_t PIN_LIDAR_LEFT_VP = 34;

// ADC Gas Sensor (ADS115) - Pins
static constexpr uint8_t PIN_ADC_SDA = 21;
static constexpr uint8_t PIN_ADC_SCL = 22;
static constexpr uint8_t I2C_ADDRESS_ADC = 0x48; // I2C Address

static ADS1115 ADC_GAS_SENSOR;



// ------------------------------------- WiFi Communication -------------------------------------
static char* SSID;
static char* PASSWORD;



// ------------------------------------- SPI Communication -------------------------------------
static constexpr uint8_t PIN_VSPI_MISO = 19;
static constexpr uint8_t PIN_VSPI_MOSI = 23;
static constexpr uint8_t PIN_VSPI_SCLK = 18;
static constexpr uint8_t PIN_VSPI_SS = 5;




    public:
// ------------------------------------- Variables -------------------------------------
int Tag_Count;

// ------------------------------------- Setups -------------------------------------
static void Setup_Wifi();
static void Setup_RFID_Tag_Reader();
static void Setup_LIDAR();
static void Setup_ADC_Gas_Sensor();
static void Setup_SPI();

// ------------------------------------- Functions -------------------------------------
// Printing I2C Devices 
static void Get_I2C_Devices();

static uint16_t Get_Distance_Left();
static uint16_t Get_Distance_Middle();
static uint16_t Get_Distance_Right();

static uint16_t Get_ADC_Gas_Sensor_Value();
static void Get_RFID_Tag_Value();

static void DATA_SPI();

static void send_x();
static void send_y();
static void send_theta();

static void received_x();
static void received_y();
static void received_theta();

static void left_sensor();
static void middle_sensor();
static void right_sensor();


// ------------------------------------- Web Server -------------------------------------
static void beginServer();
static void handleRoot();
static void handleSubmit();
static void handleCoordinates();
static String getPage();
};

#endif