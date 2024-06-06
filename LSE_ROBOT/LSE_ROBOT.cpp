#include "LSE_ROBOT.h"

#define buffersize 30

typedef union _packet_buffer_t {
  uint8_t buff[4];
  float value;
} packet_buffer_t;

WebServer server(80); // Instancia o servidor na porta 

u_int8_t RXBuffer[buffersize];
u_int8_t TXBuffer[buffersize];

packet_buffer_t _packet, _packet2, _packet3, _packet4, _packetX, _packetY, _packet_theta, _packetX_received, _packetY_received, _packet_theta_received, send_right_sensor, send_left_sensor, send_middle_sensor;
float x_received, y_received, theta_received;

String rightWheelSpeed, leftWheelSpeed, xCoord, yCoord, zCoord, theta;
String currentX = "0", currentY = "0", currentZ = "0", currentTheta = "0";
String pathCoordinates = ""; // Para armazenar o histórico das coordenadas


// ------------------------------------- WiFi Communication -------------------------------------
// char* LSE_ROBOT::SSID = "iPhone de Miguel (5)";
// char* LSE_ROBOT::PASSWORD = "123456789";

// ------------------------------------- Devices -------------------------------------
// VL53L0X LSE_ROBOT::RFID_TAG_READER(PIN_RFID_SS, RDID_RST_PIN);
VL53L0X LSE_ROBOT::LIDAR_MIDDLE;
ADS1115 LSE_ROBOT::ADC_GAS_SENSOR;


// ------------------------------------- Setup WiFi -------------------------------------
void LSE_ROBOT::Setup_Wifi() {
  WiFi.enableSTA(true);

  delay(2000);

  WiFi.begin("iPhone de Miguel (5)", "123456789");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// ------------------------------------- Setup RFID Tag Reader -------------------------------------
void LSE_ROBOT::Setup_RFID_Tag_Reader() {
  SPI.begin();
  LSE_ROBOT::RFID_TAG_READER.PCD_Init();
}

// ------------------------------------- Setup_ADC_Gas_Sensor -------------------------------------
void LSE_ROBOT::Setup_ADC_Gas_Sensor() {
 ADC_GAS_SENSOR.begin();
}

// ------------------------------------- Setup SPI-------------------------------------
void LSE_ROBOT::Setup_SPI() {
  pinMode(PIN_VSPI_SS, OUTPUT);
  pinMode(PIN_VSPI_SCLK, OUTPUT);
  pinMode(PIN_VSPI_MOSI, OUTPUT);
  pinMode(PIN_VSPI_MISO, INPUT);

  digitalWrite(PIN_VSPI_SS, HIGH);
  SPI.begin(PIN_VSPI_SCLK, PIN_VSPI_MISO, PIN_VSPI_MOSI, PIN_VSPI_SS);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  // SPI.setFrequency(1000000);

  Serial.setDebugOutput(true);
}

// ------------------------------------- Setup LIDAR -------------------------------------
void LSE_ROBOT::Setup_LIDAR() {
  // Left LIDAR
  pinMode(PIN_LIDAR_LEFT_VP, INPUT);

  // Right LIDAR
  pinMode(PIN_LIDAR_RIGHT_VP, INPUT);

  // Middle LIDAR
  Wire.begin(PIN_LIDAR_MIDDLE_SDA, PIN_SCL_MIDDLE);

  pinMode(PIN_XSHUT_MIDDLE, OUTPUT);
  digitalWrite(PIN_XSHUT_MIDDLE, LOW);
  delay(10);
  digitalWrite(PIN_XSHUT_MIDDLE, HIGH);
  delay(10);

  LIDAR_MIDDLE.setAddress(I2C_ADDRESS_LIDAR_MIDDLE);
  LIDAR_MIDDLE.init();
  LIDAR_MIDDLE.startContinuous(0);
}

// ------------------------------------- Get I2C Devices -------------------------------------
void LSE_ROBOT::Get_I2C_Devices() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // O comando Wire.beginTransmission() inicia uma transmissão com o dispositivo de endereço especificado.
    Wire.beginTransmission(address);
    error = Wire.endTransmission(); // O comando Wire.endTransmission() termina a transmissão.

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }

  delay(5000); // Espera 5 segundos antes de escanear novamente
}

// ------------------------------------- Get Distance Left -------------------------------------
uint16_t LSE_ROBOT::Get_Distance_Left() {
  uint16_t LIDAR_LEFT_READ_VALUE = analogRead(PIN_LIDAR_LEFT_VP);
  return constrain(map(LIDAR_LEFT_READ_VALUE,0,4095,800,100),100,800);
}

// ------------------------------------- Get Distance Right -------------------------------------
uint16_t LSE_ROBOT::Get_Distance_Right() {
  uint16_t LIDAR_RIGHT_READ_VALUE = analogRead(PIN_LIDAR_RIGHT_VP);
  return constrain(map(LIDAR_RIGHT_READ_VALUE,0,4095,800,100),100,800);
}

// ------------------------------------- Get Distance Middle -------------------------------------
uint16_t LSE_ROBOT::Get_Distance_Middle() {
  uint16_t LIDAR_MIDDLE_READ_VALUE = LIDAR_MIDDLE.readRangeContinuousMillimeters();
  return constrain(LIDAR_MIDDLE_READ_VALUE,0,2600);
}

// ------------------------------------- Get Gas Sensor Value -------------------------------------
// uint16_t LSE_ROBOT::Get_ADC_Gas_Sensor_Value() {
//   uint16_t GAS_SENSOR_READ_VALUE = ADC_GAS_SENSOR.readADC_SingleEndedContinuous(0);
//   return constrain(map(GAS_SENSOR_READ_VALUE,0,4095,0,1000),0,1000);
// }

// ------------------------------------- Get RFID Tag -------------------------------------
void LSE_ROBOT::Get_RFID_Tag_Value() {
  if (RFID_TAG_READER.PICC_IsNewCardPresent() && RFID_TAG_READER.PICC_ReadCardSerial()) {
    Serial.println("Tag RFID detectada!");

    // Obtém o UID da tag
    String tagUID = "";
    for (byte i = 0; i < RFID_TAG_READER.uid.size; i++) {
      tagUID += String(RFID_TAG_READER.uid.uidByte[i] < 0x10 ? "0" : "");
      tagUID += String(RFID_TAG_READER.uid.uidByte[i], HEX);
    }
    Serial.print("UID da tag: ");
    Serial.println(tagUID);

    delay(1000); // Pequeno atraso para evitar leituras múltiplas da mesma tag
  }
}

// ------------------------------------- send_x -------------------------------------
void LSE_ROBOT::send_x() {
  _packetX.value = xCoord.toFloat();
  TXBuffer[5] = _packetX.buff[0];
  TXBuffer[6] = _packetX.buff[1];
  TXBuffer[7] = _packetX.buff[2];
  TXBuffer[8] = _packetX.buff[3];
}

// ------------------------------------- received_x -------------------------------------
void LSE_ROBOT::received_x() {
   _packetX_received.buff[0] = RXBuffer[5];
  _packetX_received.buff[1] = RXBuffer[6];
  _packetX_received.buff[2] = RXBuffer[7];
  _packetX_received.buff[3] = RXBuffer[8];
  x_received = _packetX_received.value;
}



// ------------------------------------- send_y -------------------------------------
void LSE_ROBOT::send_y() {
  _packetY.value = yCoord.toFloat();
  TXBuffer[9] = _packetY.buff[0];
  TXBuffer[10] = _packetY.buff[1];
  TXBuffer[11] = _packetY.buff[2];
  TXBuffer[12] = _packetY.buff[3];
}

// ------------------------------------- received_y -------------------------------------
void LSE_ROBOT::received_y() {
  _packetY_received.buff[0] = RXBuffer[9];
  _packetY_received.buff[1] = RXBuffer[10];
  _packetY_received.buff[2] = RXBuffer[11];
  _packetY_received.buff[3] = RXBuffer[12];
  y_received = _packetY_received.value;
}



// ------------------------------------- send_tehta -------------------------------------
void LSE_ROBOT::send_theta() {
  _packet_theta.value = theta.toFloat();
  TXBuffer[13] = _packet_theta.buff[0];
  TXBuffer[14] = _packet_theta.buff[1];
  TXBuffer[15] = _packet_theta.buff[2];
  TXBuffer[16] = _packet_theta.buff[3];
}

// ------------------------------------- received_theta -------------------------------------
void LSE_ROBOT::received_theta() {
  _packet_theta_received.buff[0] = RXBuffer[13];
  _packet_theta_received.buff[1] = RXBuffer[14];
  _packet_theta_received.buff[2] = RXBuffer[15];
  _packet_theta_received.buff[3] = RXBuffer[16];
  theta_received = _packet_theta_received.value;
}


// ------------------------------------- right_sensor -------------------------------------
void LSE_ROBOT::right_sensor() {
  send_right_sensor.value = Get_Distance_Right();
  TXBuffer[17] = send_right_sensor.buff[0];
  TXBuffer[18] = send_right_sensor.buff[1];
  TXBuffer[19] = send_right_sensor.buff[2];
  TXBuffer[20] = send_right_sensor.buff[3];

}
// ------------------------------------- left_ sensor -------------------------------------
void LSE_ROBOT::left_sensor() {
  send_left_sensor.value = Get_Distance_Left();
  TXBuffer[21] = send_left_sensor.buff[0];
  TXBuffer[22] = send_left_sensor.buff[1];
  TXBuffer[23] = send_left_sensor.buff[2];
  TXBuffer[24] = send_left_sensor.buff[3];
}

// ------------------------------------- middl_sensor -------------------------------------
void LSE_ROBOT::middle_sensor() {
  send_middle_sensor.value = Get_Distance_Middle();
  TXBuffer[25] = send_middle_sensor.buff[0];
  TXBuffer[26] = send_middle_sensor.buff[1];
  TXBuffer[27] = send_middle_sensor.buff[2];
  TXBuffer[28] = send_middle_sensor.buff[3];
}

// ------------------------------------- DATA SPI -------------------------------------
void LSE_ROBOT::DATA_SPI() {
  send_x();
  send_y();
  send_theta();
  right_sensor();
  left_sensor();
  middle_sensor();

  digitalWrite(PIN_VSPI_SS, LOW); // pull SS slow to prep other for transfer
  SPI.transferBytes(TXBuffer, RXBuffer, buffersize);
  digitalWrite(PIN_VSPI_SS, HIGH); // pull ss high to signify end of data transfer

  received_x();
  received_y();
  received_theta();

  Serial.printf("x: %f\n",x_received);
  Serial.printf("y: %f\n",y_received);
  Serial.printf("Theta: %f\n", theta_received);
  server.handleClient();
}

//------------------------------------- Get_Page -------------------------------------
String LSE_ROBOT::getPage() {
  String page = "<html lang='en'><head><meta http-equiv='refresh' content='60'/>";
  page += "<title>ESP32 WebServer - Control</title>";
  page += "<style>";
  page += "body { background-color: #f0f0f0; font-family: Arial, Helvetica, Sans-Serif; color: #333333; padding: 20px; }";
  page += "h1 { color: #0056b3; }";
  page += "form { background: #ffffff; padding: 20px; border-radius: 10px; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); }";
  page += "ul { list-style-type: none; padding: 0; }";
  page += "li { margin-bottom: 10px; }";
  page += "input[type='text'], input[type='number'] { width: 100%; padding: 8px; margin-top: 5px; border: 1px solid #cccccc; border-radius: 5px; }";
  page += "input[type='submit'] { background-color: #0056b3; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; }";
  page += "input[type='submit']:hover { background-color: #003d80; }";
  page += ".output { margin-top: 20px; padding: 20px; background: #e9ecef; border-radius: 10px; }";
  page += ".output h3 { margin-top: 0; }";
  page += "</style>";
  page += "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>";
  page += "<script>";
  page += "window.onload = function() {";
  page += "  var ctx = document.getElementById('chart').getContext('2d');";
  page += "  window.myLine = new Chart(ctx, {";
  page += "    type: 'line',";
  page += "    data: {";
  page += "      datasets: [{";
  page += "        label: 'Path',";
  page += "        data: [],"; // Inicializa o gráfico sem dados
  page += "        borderColor: 'rgba(75, 192, 192, 1)',";
  page += "        tension: 0.1";
  page += "      }]";
  page += "    },";
  page += "    options: { scales: { x: { type: 'linear', position: 'bottom' }, y: { type: 'linear' } } }";
  page += "  });";
  page += "  setInterval(updateCoordinates, 1);"; // Atualiza as coordenadas a cada 1 ms
  page += "};";
  page += "function updateCoordinates() {";
  page += "  fetch('/coordinates').then(response => response.json()).then(data => {";
  page += "    var x = data.x;";
  page += "    var y = data.y;";
  page += "    window.myLine.data.datasets[0].data.push({x: x, y: y});";
  page += "    window.myLine.update();";
  page += "  });";
  page += "}";
  page += "</script>";
  page += "</head><body><h1>ESP32 WebServer</h1>";
  page += "<h3>Robot Control</h3>";
  page += "<form action='/' method='POST'>";
  page += "<ul>";
  
  // Speed Control
  page += "<li>Right Wheel Speed: <input type='text' name='rightWheelSpeed'></li>";
  page += "<li>Left Wheel Speed: <input type='text' name='leftWheelSpeed'></li>";
  
  // Coordinates Control
  page += "<li>X Coordinate: <input type='text' name='xCoord'></li>";
  page += "<li>Y Coordinate: <input type='text' name='yCoord'></li>";

  
  page += "</ul>";
  page += "<input type='submit' value='Submit'>";
  page += "</form>";

  page += "<div class='output'>";
  page += "<h3>Current Coordinates</h3>";
  page += "<p>X: " + currentX + "</p>";
  page += "<p>Y: " + currentY + "</p>";
  page += "</div>";

  page += "<canvas id='chart' width='400' height='400'></canvas>";

  page += "</body></html>";

  return page;
}

// ------------------------------------- handle Root -------------------------------------
void LSE_ROBOT::handleRoot() {
  rightWheelSpeed = server.arg("rightWheelSpeed");
  leftWheelSpeed = server.arg("leftWheelSpeed");
  xCoord = server.arg("xCoord");
  yCoord = server.arg("yCoord");

  // Update current coordinates if values are provided
  if (xCoord != "") {
    currentX = xCoord;
    Serial.print("X Coordinate: ");
    Serial.println(xCoord);
  }

  if (yCoord != "") {
    currentY = yCoord;
    Serial.print("Y Coordinate: ");
    Serial.println(yCoord);
  }


  // Adiciona as novas coordenadas ao histórico
  if (xCoord != "" && yCoord != "") {
    if (pathCoordinates != "") {
      pathCoordinates += ",";
    }
    pathCoordinates += "{x: " + xCoord + ", y: " + yCoord + "}";
  }
  server.send(200, "text/html", getPage());
}




//------------------------------------- Begin Server -------------------------------------
void LSE_ROBOT::beginServer() {
  server.on("/", handleRoot);
  server.on("/coordinates", handleCoordinates); // Novo endpoint para obter coordenadas
  server.begin();
  Serial.println("HTTP server started");
}

//------------------------------------- handle Coordinates -------------------------------------
void LSE_ROBOT::handleCoordinates() {
  String json = "{\"x\": " + String(x_received) + ", \"y\": " + String(y_received) + ", \"theta\": " + String(theta_received) + "}";
  server.send(200, "application/json", json);
}





















