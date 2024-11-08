#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include <OneWire.h>
#include <math.h>
#include <Wire.h>

//////////VARIABLES/////////
int _count = 0;
bool _accidentState = false;
bool _alarmState = false;
bool _updateState = false;
bool _sendSmsState = false;
bool _waitResponseState = false;
String _mensaje;
double _eventSpeed;

int _argHour, _argMinute;
float _internalTemperature;
float _accelerometerErrorX = 0;
float _accelerometerErrorY = 0;
float _gyroscopeErrorX = 0;
float _gyroscopeErrorY = 0;
float _gyroscopeErrorZ = 0;
float _accAngleX = 0;
float _accAngleY = 0;
float _roll;
float _pitch;
float _highestRegisteredRoll = 0;
float _highestRegisteredPitch = 0;
float _xGForce = 0;
float _yGForce = 0;
float _zGForce = 0;
float _readGForceX;
float _readGForceY;
float _readGForceZ;

double _latitude;
double _longitude;
double _speed;
double _highestRegisteredSpeed;
String _course;

float _elapsedTime;
float _currentTime;
float _previousTime;
float gyroAngleX;
float gyroAngleY;
float _GyroX = 0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
float _GyroY = 0;
float _GyroZ = 0;
float _AccX = 0;  // X-axis value //4096 para 8G
float _AccY = 0;  // Y-axis value
float _AccZ = 0;  // Z-axis value
int16_t ax, ay, az;

int mq9SensorValue;
float mq9SensorVoltage;
float rs_gas;
float _gasRatio;

////////////////////////////

const int MPU = 0x68;      // MPU6050 I2C address
const int pinDatosDQ = 9;  //Pin donde se conecta el bus 1-Wire

SoftwareSerial Sim800Serial(4, 5);  //INVERTIR PINES PARA PCB_V3
SoftwareSerial GPSserial(2, 3);
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);
TinyGPSPlus gps;
MPU6050 mpu;

void sendSMS(bool updateState) {

  Serial.println("Envio MSJ");

  String SMSdata;
  Sim800Serial.begin(9600);
  Sim800Serial.print("AT+CMGF=1\r");
  delay(100);
  Sim800Serial.println("AT+CNMI=1,2,0,0,0");
  delay(100);
  Sim800Serial.print("AT+CMGS=\"+543564654911\"\r");
  delay(500);

  if (!updateState) {
    SMSdata = "Accidente vehicular detectado\nLat: " + String(_latitude, 6) + ", Long: " + String(_longitude, 6) + ", Velocidad: " + String(_highestRegisteredSpeed, 2);
  } else {
    SMSdata = "Se modificaron condiciones en el siniestro\nTemperatura: " + String(sensorDS18B20.getTempCByIndex(0), 2) + "°C\nAlta probabilidad de incendio.";
    _waitResponseState = true;
    TIMSK1 &= ~(1 << TOIE1);
    noInterrupts();
  }

  Serial.println(SMSdata);

  Sim800Serial.print(SMSdata);
  delay(500);
  Sim800Serial.print((char)26);
  delay(500);
  Sim800Serial.println();
}

void getArgTime(void) {

  _argHour = gps.time.hour() - 3;
  if (_argHour < 0) {
    _argHour = _argHour + 24;
  }
  _argMinute = gps.time.minute();
}

void temperatureRead(void) {
  sensorDS18B20.requestTemperatures();
  //Serial.println("LEO TEMPERATURA");
  //_internalTemperature = sensorDS18B20.getTempCByIndex(0);
}

void getGasRatio(void) {

  mq9SensorValue = analogRead(A0);
  mq9SensorVoltage = (float)mq9SensorValue / 1024 * 5.0;
  rs_gas = (5.0 - mq9SensorVoltage) / mq9SensorVoltage;

  _gasRatio = rs_gas / 4.5;  //4.5 es el valor medido al aire libre.
}

String courseToCardinal(int course) {
  String directions[8] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
  int index = round((course % 360) / 45);
  return directions[index % 8];
}

void calculateMpuError(void) {
  int c = 0;
  //leo 200 veces valores de acelerometro
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  //AVERIGUAR ESTO
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    float AccX = (Wire.read() << 8 | Wire.read()) / 4096.0;
    float AccY = (Wire.read() << 8 | Wire.read()) / 4096.0;
    float AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0;
    //sumas todas las lecturas
    _accelerometerErrorX = _accelerometerErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    _accelerometerErrorY = _accelerometerErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  //dividis la suma por 200 para obtener el valor de error
  _accelerometerErrorX = _accelerometerErrorX / 200;
  _accelerometerErrorY = _accelerometerErrorY / 200;
  c = 0;

  //leo 200 veces valores del giroscopio
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    float GyroX = Wire.read() << 8 | Wire.read();
    float GyroY = Wire.read() << 8 | Wire.read();
    float GyroZ = Wire.read() << 8 | Wire.read();
    // Sumo todas las lecturas
    _gyroscopeErrorX = _gyroscopeErrorX + (GyroX / 131.0);
    _gyroscopeErrorY = _gyroscopeErrorY + (GyroY / 131.0);
    _gyroscopeErrorZ = _gyroscopeErrorZ + (GyroZ / 131.0);
    c++;
  }

  _gyroscopeErrorX = _gyroscopeErrorX / 200;
  _gyroscopeErrorY = _gyroscopeErrorY / 200;
  _gyroscopeErrorZ = _gyroscopeErrorZ / 200;
}

void readMPU6050Data(void) {

  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  _AccX = (Wire.read() << 8 | Wire.read()) / 4096.0;  // X-axis value //4096 para 8G
  _AccY = (Wire.read() << 8 | Wire.read()) / 4096.0;  // Y-axis value
  _AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0;  // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2))) * 180 / PI) + (_accelerometerErrorX * -1);       // AccErrorX ~(-1.15) See the calculate_IMU_error()custom function for more details
  _accAngleY = (atan(-1 * _AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2))) * 180 / PI) + (_accelerometerErrorY * -1);  // AccErrorY ~(-2.39)

  // === Read gyroscope data === //
  _previousTime = _currentTime;                          // Previous time is stored before the actual time read
  _currentTime = millis();                               // Current time actual time read
  _elapsedTime = (_currentTime - _previousTime) / 1000;  // Divide by 1000 to get seconds

  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);                     // Read 4 registers total, each axis value is stored in 2 registers
  _GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  _GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  _GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  _GyroX = _GyroX + (_gyroscopeErrorX * -1);  // GyroErrorX ~(-2.85)
  _GyroY = _GyroY + (_gyroscopeErrorY * -1);  // GyroErrorY ~(1.81)
  _GyroZ = _GyroZ + (_gyroscopeErrorZ * -1);  // GyroErrorZ ~ (0.99)

  gyroAngleX = gyroAngleX + _GyroX * _elapsedTime;  // deg/s * s = deg
  gyroAngleY = gyroAngleY + _GyroY * _elapsedTime;

  _roll = 0.96 * gyroAngleX + 0.04 * _accAngleX;
  _pitch = 0.96 * gyroAngleY + 0.04 * _accAngleY;

  gyroAngleX = _roll;   //////PROBAR CON Y SIN ESTOS TERMINOS
  gyroAngleY = _pitch;  //////

  if (abs(_roll) >= _highestRegisteredRoll) {
    _highestRegisteredRoll = abs(_roll);
  }

  if (abs(_pitch) >= _highestRegisteredPitch) {
    _highestRegisteredPitch = abs(_pitch);
  }
}

void readGPSData(void) {

  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
    if (gps.location.isUpdated()) {

      _course = courseToCardinal(gps.course.deg());
      getArgTime();

      _latitude = gps.location.lat();
      _longitude = gps.location.lng();
      _speed = gps.speed.kmph();
      /*courseDeg = gps.course.deg();
      day = gps.date.day();
      month = gps.date.month();
      year = gps.date.year();*/

      if (_speed >= _highestRegisteredSpeed) {
        _highestRegisteredSpeed = _speed;
      }
    }
  }
}

void getAxisAcceleration() {

  mpu.getAcceleration(&ax, &ay, &az);

  _readGForceX = abs(ax) / 4096.0;  //factor para convertir raw a G
  _readGForceY = abs(ay) / 4096.0;
  _readGForceZ = abs(az) / 4096.0;

  if (abs(_readGForceX) >= _xGForce) {
    _xGForce = abs(_readGForceX);
  }

  if (abs(_readGForceY) >= _yGForce) {
    _yGForce = abs(_readGForceY);
  }

  if (abs(_readGForceZ) >= _zGForce) {
    _zGForce = abs(_readGForceZ);
  }
}

void (*resetFunc)(void) = 0;

ISR(TIMER1_OVF_vect)  //timer overflow ISR: update counter INTERRUPT PARA MPU6050
{
  /*  ------codigo en la interrupt----- */

  if (!_alarmState) {
    if ((_highestRegisteredPitch >= 60) || (_highestRegisteredRoll >= 60)) {
      _alarmState = true;
      _eventSpeed = _highestRegisteredSpeed;
    } else if ((_xGForce >= 2/*4*/ || _yGForce >= 2/*4*/ || _zGForce >= 2/*4*/) && _speed > 10) {
      _alarmState = true;
      _eventSpeed = _highestRegisteredSpeed;
    } else {
      _xGForce = 0;
      _yGForce = 0;
      _zGForce = 0;
      _highestRegisteredPitch = 0;
      _highestRegisteredRoll = 0;
      _highestRegisteredSpeed = 0;
    }
  } else {
    if (!_accidentState) {
      Serial.println(_count);
      if (_count >= 3) {
        if (_speed <= (_eventSpeed / 2)) {
          _accidentState = true;
          _sendSmsState = true;
        } else {
          _alarmState = false;
          Serial.println("velocidad insuficiente. Reseteo variables.");
          _xGForce = 0;
          _yGForce = 0;
          _zGForce = 0;
          _highestRegisteredPitch = 0;
          _highestRegisteredRoll = 0;
          _highestRegisteredSpeed = 0;
          _count = 0;
        }
      }
      _count++;
    } else {
      Serial.println("Solicitando temperatura y gas");
      if (sensorDS18B20.getTempCByIndex(0) > 30 || _gasRatio <= 0.9) {
        _sendSmsState = true;
        _updateState = true;
      }
    }
  }

  ////// BORRAR /////

  if (_alarmState) {
    Serial.println("Estado de alarma: TRUE");
  } else {
    Serial.println("Estado de alarma: FALSE");
  }

  if (_accidentState) {
    Serial.println("Estado de accidente: TRUE");
  } else {
    Serial.println("Estado de accidente: FALSE");
  }

  Serial.println(_highestRegisteredPitch);
  Serial.println(_highestRegisteredRoll);
  Serial.println(_highestRegisteredSpeed);
  Serial.println(_xGForce);
  Serial.println(_yGForce);
  Serial.println(_zGForce);
  Serial.println(sensorDS18B20.getTempCByIndex(0));

  Serial.println("Fin Interrupt");

  //////////////////
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();                 //inicia la cominación i2c
  Wire.beginTransmission(MPU);  //comienza la comunicación
  Wire.write(0x6B);             //escribis en el registro 6B del acell
  Wire.write(0x00);             //mandas un 0 y reseteas
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  //el registro 1C del aceel es ACCEL_CONFIG
  Wire.write(0x10);  // 00010000 pone el registro en escala completa +/- 8g
  Wire.endTransmission(true);

  GPSserial.begin(9600);

  Serial.begin(9600);

  sensorDS18B20.begin();
  sensorDS18B20.setResolution(9);

  do (calculateMpuError());
  while (isnan(_accelerometerErrorX) && isnan(_accelerometerErrorY));

  sensorDS18B20.requestTemperatures();

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= 0b00000101;  //1024 prescaler
  TIMSK1 |= 0b00000001;  //enable overflow interrupt
  interrupts();

  //Serial.println((String) "acc X error: " + _accelerometerErrorX);
  //Serial.println((String) "acc Y error: " + _accelerometerErrorY);
  Serial.println("INICIO PROGRAMA");
}

void loop() {

  if (_accidentState) {

    if (_sendSmsState) {
      sendSMS(_updateState);
      _sendSmsState = false;
    }
    temperatureRead();
    getGasRatio();
  } else {
    readMPU6050Data();
    getAxisAcceleration();
    readGPSData();
  }

  if (_waitResponseState) {
    delay(500);
    Serial.println("ESPERANDO MSJ DE RESPUESTA");

    while (Sim800Serial.available()) {
      _mensaje = Sim800Serial.readString();
      Serial.println(_mensaje);
    }

    if (_mensaje.indexOf("Recibido") >= 0) {
      Serial.println("MENSAJE RECIBIDO");
      delay(3000);
      resetFunc();
    }
  }
}