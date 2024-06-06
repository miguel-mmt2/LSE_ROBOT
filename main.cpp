#include <LSE_ROBOT.h>

LSE_ROBOT ROBOT;

// ===================================== SETUP() =====================================
void setup() {
  Serial.begin(115200);

  ROBOT.Setup_SPI();

  // ROBOT.Setup_RFID_Tag_Reader();
  ROBOT.Setup_LIDAR();
  // ROBOT.Setup_ADC_Gas_Sensor();

  ROBOT.Setup_Wifi();

  ROBOT.beginServer();
}

// ===================================== LOOP() =====================================
void loop() {
  ROBOT.DATA_SPI();

  // Get ADC Gas Sensor Value
  // ROBOT.Get_ADC_Gas_Sensor_Value();

  // Get RFID Tag Value
  // ROBOT.Get_RFID_Tag_Value();

  // Imprime os valores dos sensores LIDAR
  Serial.printf("LIDAR Right: %d mm\n",   ROBOT.Get_Distance_Left());
  Serial.printf("LIDAR Middle: %d mm\n",   ROBOT.Get_Distance_Middle());
  Serial.printf("LIDAR Left: %d mm\n",   ROBOT.Get_Distance_Right());

  delay(100); // Atualiza a cada 100 ms (ajuste conforme necessário)
}
