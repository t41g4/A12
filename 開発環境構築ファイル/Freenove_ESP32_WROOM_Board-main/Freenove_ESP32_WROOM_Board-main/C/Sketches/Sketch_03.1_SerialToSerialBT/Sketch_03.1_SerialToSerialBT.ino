#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>


// --------------------
// ピン設定
// --------------------

#define LED_PIN 2

// SDカードCS
#define SD_CS 5


// MPU6050
Adafruit_MPU6050 mpu;


// 測定状態
bool measuring = false;


// 時間管理
unsigned long startTime;
unsigned long lastSampleTime;


// 200Hz
#define SAMPLE_INTERVAL 5


File dataFile;


// --------------------
// BLE設定
// --------------------

#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-1234-abcdefabcdef"



class Callback : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *pCharacteristic) {

    String value = pCharacteristic->getValue();

    Serial.println(value);


    if(value == "START"){

      measuring = true;

      startTime = millis();

      Serial.println("測定開始");


      dataFile =
        SD.open("/vibration.csv", FILE_WRITE);


      if(dataFile){

        dataFile.println(
          "time_ms,accel_x,accel_y,accel_z"
        );

        dataFile.close();

      }

    }



    if(value == "STOP"){

      measuring = false;

      Serial.println("測定終了");

    }

  }

};



// --------------------
// setup
// --------------------

void setup(){

  Serial.begin(115200);


  pinMode(
    LED_PIN,
    OUTPUT
  );


  // MPU6050

  Wire.begin(21,22);


  if(!mpu.begin()){

    Serial.println(
      "MPU6050エラー"
    );

    while(1);

  }


  Serial.println(
    "MPU6050 OK"
  );



  mpu.setAccelerometerRange(
    MPU6050_RANGE_2_G
  );


  mpu.setFilterBandwidth(
    MPU6050_BAND_44_HZ
  );



  // SD

  if(!SD.begin(SD_CS)){

    Serial.println(
      "SDカードエラー"
    );

    while(1);

  }


  Serial.println(
    "SD OK"
  );



  // BLE

  BLEDevice::init(
    "ESP32_Vibration"
  );


  BLEServer *server =
    BLEDevice::createServer();



  BLEService *service =
    server->createService(
      SERVICE_UUID
    );



  BLECharacteristic *characteristic =
    service->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE
    );



  characteristic->setCallbacks(
    new Callback()
  );



  service->start();



  BLEAdvertising *advertising =
    BLEDevice::getAdvertising();


  advertising->start();



  Serial.println(
    "BLE Ready"
  );

}



// --------------------
// loop
// --------------------

void loop(){



  // LED点滅

  static unsigned long ledTime = 0;

  if(measuring){

    if(millis() - ledTime > 300){

      ledTime = millis();

      digitalWrite(
        LED_PIN,
        !digitalRead(LED_PIN)
      );

    }

  }

  else{

    digitalWrite(
      LED_PIN,
      LOW
    );

  }





  // 測定処理

  if(measuring){


    if(millis() - lastSampleTime >= SAMPLE_INTERVAL){


      lastSampleTime = millis();



      sensors_event_t a,g,temp;


      mpu.getEvent(
        &a,
        &g,
        &temp
      );


      unsigned long t =
        millis() - startTime;



      dataFile =
        SD.open(
          "/vibration.csv",
          FILE_APPEND
        );



      if(dataFile){


        dataFile.print(t);
        dataFile.print(",");


        dataFile.print(
          a.acceleration.x
        );

        dataFile.print(",");


        dataFile.print(
          a.acceleration.y
        );


        dataFile.print(",");


        dataFile.println(
          a.acceleration.z
        );


        dataFile.close();

      }



      // 確認表示

      Serial.print(t);
      Serial.print(",");

      Serial.print(
        a.acceleration.x
      );

      Serial.print(",");

      Serial.print(
        a.acceleration.y
      );

      Serial.print(",");

      Serial.println(
        a.acceleration.z
      );


    }

  }

}