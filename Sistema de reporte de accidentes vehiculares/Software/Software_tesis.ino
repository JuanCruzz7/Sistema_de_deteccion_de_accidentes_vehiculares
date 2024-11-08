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
float _GyroX = 0;  
float _GyroY = 0;
float _GyroZ = 0;
float _AccX = 0;
float _AccY = 0;
float _AccZ = 0;
int16_t ax, ay, az;

int mq9SensorValue;
float mq9SensorVoltage;
float rs_gas;
float _gasRatio;

////////////////////////////

const int MPU = 0x68;  
const int pinDatosDQ = 9;

SoftwareSerial Sim800Serial(4, 5);
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
}

void getGasRatio(void) {

  mq9SensorValue = analogRead(A0);
  mq9SensorVoltage = (float)mq9SensorValue / 1024 * 5.0;
  rs_gas = (5.0 - mq9SensorVoltage) / mq9SensorVoltage;

  _gasRatio = rs_gas / 4.5;
}

String courseToCardinal(int course) {
  String directions[8] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
  int index = round((course % 360) / 45);
  return directions[index % 8];
}

void calculateMpuError(void) {
  int c = 0;
  
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  //AVERIGUAR ESTO
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    float AccX = (Wire.read() << 8 | Wire.read()) / 4096.0;
    float AccY = (Wire.read() << 8 | Wire.read()) / 4096.0;
    float AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0;
    
    _accelerometerErrorX = _accelerometerErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    _accelerometerErrorY = _accelerometerErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  
  _accelerometerErrorX = _accelerometerErrorX / 200;
  _accelerometerErrorY = _accelerometerErrorY / 200;
  c = 0;

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

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  _AccX = (Wire.read() << 8 | Wire.read()) / 4096.0;
  _AccY = (Wire.read() << 8 | Wire.read()) / 4096.0;
  _AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0;

  _accAngleX = (atan(_AccY / sqrt(pow(_AccX, 2) + pow(_AccZ, 2))) * 180 / PI) + (_accelerometerErrorX * -1);       // AccErrorX ~(-1.15) See the calculate_IMU_error()custom function for more details
  _accAngleY = (atan(-1 * _AccX / sqrt(pow(_AccY, 2) + pow(_AccZ, 2))) * 180 / PI) + (_accelerometerErrorY * -1);  // AccErrorY ~(-2.39)

  _previousTime = _currentTime;
  _currentTime = millis();
  _elapsedTime = (_currentTime - _previousTime) / 1000;

  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  _GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  _GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  _GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  _GyroX = _GyroX + (_gyroscopeErrorX * -1);
  _GyroY = _GyroY + (_gyroscopeErrorY * -1);
  _GyroZ = _GyroZ + (_gyroscopeErrorZ * -1);

  gyroAngleX = gyroAngleX + _GyroX * _elapsedTime;
  gyroAngleY = gyroAngleY + _GyroY * _elapsedTime;

  _roll = 0.96 * gyroAngleX + 0.04 * _accAngleX;
  _pitch = 0.96 * gyroAngleY + 0.04 * _accAngleY;

  gyroAngleX = _roll;
  gyroAngleY = _pitch;

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

      if (_speed >= _highestRegisteredSpeed) {
        _highestRegisteredSpeed = _speed;
      }
    }
  }
}

void getAxisAcceleration() {

  mpu.getAcceleration(&ax, &ay, &az);

  _readGForceX = abs(ax) / 4096.0;
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

ISR(TIMER1_OVF_vect)
{
  if (!_alarmState) {
    if ((_highestRegisteredPitch >= 60) || (_highestRegisteredRoll >= 60)) {
      _alarmState = true;
      _eventSpeed = _highestRegisteredSpeed;
    } else if ((_xGForce >= 4 || _yGForce >= 4 || _zGForce >= 4) && _speed > 60) {
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
      if (_count >= 3) {
        if (_speed <= (_eventSpeed / 2)) {
          _accidentState = true;
          _sendSmsState = true;
        } else {
          _alarmState = false;
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
      if (sensorDS18B20.getTempCByIndex(0) > 30 || _gasRatio <= 0.9) {
        _sendSmsState = true;
        _updateState = true;
      }
    }
  }
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x10);
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
  TCCR1B |= 0b00000101;
  TIMSK1 |= 0b00000001;
  interrupts();
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

    while (Sim800Serial.available()) {
      _mensaje = Sim800Serial.readString();
      Serial.println(_mensaje);
    }

    if (_mensaje.indexOf("Recibido") >= 0) {
      delay(3000);
      resetFunc();
    }
  }
}