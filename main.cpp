#include <Arduino.h>

#include <SPI.h>

#define VSPI_MISO 33
#define VSPI_MOSI 25
#define VSPI_SCLK 26
#define VSPI_SS 27

#define buffersize 20

void Task1code(void *parameter);
void forward();
void backward();
void right();
void left();

typedef union _packet_buffer_t
{
  uint8_t buff[4];
  int32_t value;
} packet_buffer_t;

packet_buffer_t _packet, _packet2, _packet3, _packet4;

TaskHandle_t Task1;

u_int8_t RXBuffer[buffersize];
u_int8_t TXBuffer[buffersize];





void setup()
{
  pinMode(VSPI_SS, OUTPUT);
  pinMode(VSPI_SCLK, OUTPUT);
  pinMode(VSPI_MOSI, OUTPUT);
  pinMode(VSPI_MISO, INPUT);

  SPI.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(1000000);

  Serial.begin(115200);
  Serial.setDebugOutput(true);

  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1",   /* Name of the task */
      10000,     /* Stack size in words */
      NULL,      /* Task input parameter */
      0,         /* Priority of the task */
      &Task1,    /* Task handle. */
      0);        /* Core where the task should run */

  forward();
  backward();
  right();
  left();
      
}

void loop()
{
  // put your main code here, to run repeatedly:
  digitalWrite(VSPI_SS, LOW); // pull SS slow to prep other for transfer
  forward();
  backward();
  right();
  left();
  SPI.transferBytes(TXBuffer, RXBuffer, buffersize);
  digitalWrite(VSPI_SS, HIGH); // pull ss high to signify end of data transfer
  _packet.buff[0] = RXBuffer[5];
  _packet.buff[1] = RXBuffer[6];
  _packet.buff[2] = RXBuffer[7];
  _packet.buff[3] = RXBuffer[8];
  //Serial.printf("Encoder: %d\n", _packet.value);
  delay(500);
  
}

void Task1code(void *parameter)
{
  for (;;)
  {
    Serial.printf("CORE %d: hello world\n", xPortGetCoreID());
    delay(500);
  }
}


void forward() {

  _packet.value = 1;
  TXBuffer[5] = _packet.buff[0];
	TXBuffer[6] = _packet.buff[1];
	TXBuffer[7] = _packet.buff[2];
	TXBuffer[8] = _packet.buff[3];

}

void backward() {

  _packet2.value = 2;
  TXBuffer[9] = _packet2.buff[0];
	TXBuffer[10] = _packet2.buff[1];
	TXBuffer[11] = _packet2.buff[2];
	TXBuffer[12] = _packet2.buff[3];

}

void left() {

  _packet3.value = 3;
  TXBuffer[13] = _packet3.buff[0];
	TXBuffer[14] = _packet3.buff[1];
	TXBuffer[15] = _packet3.buff[2];
	TXBuffer[16] = _packet3.buff[3];

}

void right() {

  _packet4.value = 4;
  TXBuffer[17] = _packet4.buff[0];
	TXBuffer[18] = _packet4.buff[1];
	TXBuffer[19] = _packet4.buff[2];
	TXBuffer[20] = _packet4.buff[3];

}