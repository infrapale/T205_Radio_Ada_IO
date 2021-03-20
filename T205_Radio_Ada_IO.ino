/***************************************************

T205 RFM69 Relay to Adafruit IO
 
****************************************************/

#include <SPI.h>
#include <Wire.h>
#include "config.h"
#include "AdafruitIO_WiFi.h"
#include <Adafruit_Sensor.h>
#include <bsec.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "ssd1306_feather.h"
#include <RH_RF69.h>
//#define  VILLA_ASTRID 1
#define  LILLA_ASTRID 1
//#define  SIIRTOLA 1
#include <secrets.h>
#include "config.h"
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0   //915.0
#define SEALEVELPRESSURE_HPA (1013.25)
#define DISPLAY_FALLBACK_MILLIS 15000


#define  FLOAT_NO_VALUE 99999.0

typedef struct {
    float water_temp;
    float temperature;
    float pressure;
    float temp_dht22;
    float hum_dht22;
    float ldr1;
    float ldr2;
} DockSensor_value_t;

typedef struct {
    float temperature;     
    float temp_dht22;     
    float hum_dht22;     
    float ldr1;     
    float ldr2;
} OD1_Sensor_value_t;


typedef struct {
    float temperature;     
    float humidity;    
    float pressure;     
    float gas;    
} Tupa_BME680_value_t;

typedef struct {
    AdafruitIO_Feed *water_temp;
    AdafruitIO_Feed *temperature;
    AdafruitIO_Feed *pressure;
    AdafruitIO_Feed *temp_dht22;
    AdafruitIO_Feed *hum_dht22;
    AdafruitIO_Feed *ldr1;
    AdafruitIO_Feed *ldr2;
} DockSensor_AIO_t;

typedef struct {
    AdafruitIO_Feed *temperature;     
    AdafruitIO_Feed *temp_dht22;     
    AdafruitIO_Feed *hum_dht22;     
    AdafruitIO_Feed *ldr1;     
    AdafruitIO_Feed *ldr2;
} OD1_Sensor_AIO_t;

typedef struct {
    AdafruitIO_Feed *temperature;     
    AdafruitIO_Feed *humidity;    
    AdafruitIO_Feed *pressure;     
    AdafruitIO_Feed *gas;    
} Tupa_BME680_AIO_t;

DockSensor_AIO_t     dock_aio;
OD1_Sensor_AIO_t     od1_aio;
Tupa_BME680_AIO_t    tupa_aio;


DockSensor_value_t   dock_sensor;
OD1_Sensor_value_t   od1_sensor;
Tupa_BME680_value_t  tupa_sensor;


AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
Bsec    iaqSensor; // I2C
bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
};


int16_t packetnum = 0;  // packet counter, we increment per xmission

void   BsecInitialize(void);
void   checkIaqSensorStatus(void);
void   RadioRxHandler(void);
String parse_json(String fromStr, char *tag);


/*************************** Sketch Code ************************************/
long int poll_rfm_millis;
long int mqtt_connect_millis;
long int bme_read_millis;
long int display_default_millis;
long int bme_save_millis; 
byte bme_indx;
boolean sd_card_ok;
String yyyy_mm_dd;
//DateTime now;
//File myFile;
uint8_t  aio_state = 0;

void setup() {
    delay(2000);
    ssd1306_setup();
    Serial.begin(115200);
    delay(2000);
    while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
    Serial.println("T205 RFM69 to Adafruit IO Relay");
    row[0] = "T2015 Radio-WiFi";
    
    pinMode(LED, OUTPUT);     
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
  
    Serial.println();
  
    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    
    if (!rf69.init()) {
      Serial.println("RFM69 radio init failed");
      row[2] ="RFM69 failed, stopped";
      display_rows();
      while (1);
    }
    Serial.println("RFM69 radio init OK!");
    row[2]="RFM69 = OK";
    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if (!rf69.setFrequency(RF69_FREQ)) {
      Serial.println("setFrequency failed");
    }
  
    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
  
    uint8_t rfm69_key[] = RFM69_KEY; //exactly the same 16 characters/bytes on all nodes!
    rf69.setEncryptionKey(rfm69_key);

    dock_aio.water_temp   = io.feed("villaastrid.water-temp");
    dock_aio.temperature  = io.feed("villaastrid.dock-temp");
    dock_aio.pressure     = io.feed("villaastrid.dock-pres");
    dock_aio.temp_dht22   = io.feed("villaastrid.docktemp-dht22");
    dock_aio.hum_dht22    = io.feed("villaastrid.dock-hum-dht22");
    dock_aio.ldr1         = io.feed("villaastrid.dock-ldr1");
    dock_aio.ldr2         = io.feed("villaastrid.dock-ldr2");
    
    od1_aio.temperature   = io.feed("villaastrid.outdoor1-temp");
    od1_aio.temp_dht22    = io.feed("villaastrid.outdoor1-temp-dht22");
    od1_aio.hum_dht22     = io.feed("villaastrid.outdoor1-hum-dht22");
    od1_aio.ldr1          = io.feed("villaastrid.outdoor1-ldr1");
    od1_aio.ldr2          = io.feed("villaastrid.outdoor1-ldr2");
    
    tupa_aio.temperature  = io.feed("villaastrid.tupa-temp");
    tupa_aio.humidity     = io.feed("villaastrid.tupa-hum");
    tupa_aio.pressure     = io.feed("villaastrid.tupa-pres");
    tupa_aio.gas          = io.feed("/feeds/villaastrid.tupa-gas");


  // Connect to WiFi access point.
    Serial.println(); Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);
  
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    Serial.println("WiFi connected");
    Serial.println("IP address: "); Serial.println(WiFi.localIP().toString());
    row[1] = "IP:"; row[1].concat(WiFi.localIP().toString());
    Serial.println(row[1]);

    BsecInitialize();
    memset(&dock_sensor,FLOAT_NO_VALUE,sizeof(dock_sensor));
    memset(&od1_sensor,FLOAT_NO_VALUE,sizeof(od1_sensor));
    memset(&tupa_sensor,FLOAT_NO_VALUE,sizeof(tupa_sensor));
    
    
  poll_rfm_millis = millis();
  mqtt_connect_millis = millis();
  display_default_millis =millis();
  bme_read_millis = millis();
  bme_save_millis = millis();
  bme_indx =0;
  //delay(5000);
}


void loop(void) {


    RadioRxHandler();

    if ((millis() - mqtt_connect_millis) > 5000)
    {
        mqtt_connect_millis = millis();
    
        io.run();
        if (aio_state > 17) aio_state = 0;
        switch (aio_state) {
            case 0: 
                aio_state++;
                break;
            case 1: 
                if (! iaqSensor.run()) // If no data is available
                {  
                    checkIaqSensorStatus();
                    aio_state = 6;
                }
                else {
                    tupa_sensor.temperature  = iaqSensor.temperature;
                    tupa_sensor.humidity     = iaqSensor.humidity;
                    tupa_sensor.pressure     = iaqSensor.pressure;
                    tupa_sensor.gas          = iaqSensor.co2Equivalent;               
                    aio_state++;
               }
            case 2: 
                tupa_aio.temperature->save(tupa_sensor.temperature);
                aio_state++;
                break; 
            case 3: 
                tupa_aio.humidity->save(tupa_sensor.humidity);
                aio_state++;
                break; 
            case 4: 
                tupa_aio.pressure->save(tupa_sensor.pressure);
                aio_state++;
                break; 
            case 5: 
                tupa_aio.gas->save(tupa_sensor.gas);
                aio_state++;
                break; 
            case 6:
                if (dock_sensor.water_temp != FLOAT_NO_VALUE) {
                    dock_aio.water_temp->save(dock_sensor.water_temp);
                }
                aio_state++;
                break; 
            case 7:
                if (dock_sensor.temperature != FLOAT_NO_VALUE) {
                    dock_aio.temperature->save(dock_sensor.temperature);
                }    
                aio_state++;
                break; 
            case 8:
                if (dock_sensor.pressure != FLOAT_NO_VALUE) {
                    dock_aio.pressure->save(dock_sensor.pressure);
                }
                aio_state++;
                break; 
            case 9:
                if (dock_sensor.temp_dht22 != FLOAT_NO_VALUE) {
                    dock_aio.temp_dht22->save(dock_sensor.temp_dht22);
                }    
                aio_state++;
                break; 
            case 10:
                if (dock_sensor.hum_dht22 != FLOAT_NO_VALUE) {
                    dock_aio.hum_dht22->save(dock_sensor.hum_dht22);
                }
                aio_state++;
                break; 
            case 11:
                if (dock_sensor.ldr1 != FLOAT_NO_VALUE) {
                    dock_aio.ldr1->save(dock_sensor.ldr1);
                }
                aio_state++;
                break; 
            case 12:
                if (dock_sensor.ldr2 != FLOAT_NO_VALUE) {
                    dock_aio.ldr2->save(dock_sensor.ldr2);
                }
                aio_state++;
                break; 
            case 13:
                if (od1_sensor.temperature != FLOAT_NO_VALUE) {
                    od1_aio.temperature->save(od1_sensor.temperature);
                }
                aio_state++;
                break; 
            case 14:
                if (od1_sensor.temp_dht22 != FLOAT_NO_VALUE) {
                    od1_aio.temp_dht22->save(od1_sensor.temp_dht22);
                }
                aio_state++;
                break; 
            case 15:
                if (od1_sensor.hum_dht22 != FLOAT_NO_VALUE) {
                    od1_aio.hum_dht22->save(od1_sensor.hum_dht22);
                }
                aio_state++;
                break;
            case 16:
                if (od1_sensor.ldr1 != FLOAT_NO_VALUE) {
                    od1_aio.ldr1->save(od1_sensor.ldr1);
                }
                aio_state++;
                break; 
            case 17:
                if (od1_sensor.ldr2 != FLOAT_NO_VALUE) {
                    od1_aio.ldr2->save(od1_sensor.ldr2);
                }
                aio_state++;
                break; 
            
         }                 
    }
}



void RadioRxHandler(void){
   String sensor_zone;
   String sensor_name;
   String sensor_value;
   String inputString;
   float  fsensor_value;

   if (rf69.available()) {
      // Should be a message for us now   
      uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf69.recv(buf, &len)) {
          if (!len) return;
          buf[len] = 0;
          Serial.print("Received [");
          Serial.print(len);
          Serial.print("]: ");
          Serial.println((char*)buf);
          Serial.print("RSSI: ");
          Serial.println(rf69.lastRssi(), DEC);
          inputString = (char*)buf;
          sensor_zone = parse_json(inputString,"{\"Z");
          sensor_name = parse_json(inputString,",\"S");
          sensor_value = parse_json(inputString,",\"V");
          fsensor_value = sensor_value.toFloat();
          row[2] = sensor_zone; row[2].concat(":");
          row[2].concat(sensor_name);row[2].concat(":");
          row[2].concat(sensor_value);
          display_rows();
          Serial.println(sensor_zone); Serial.println(sensor_name); Serial.println(sensor_value);
          if (sensor_zone == "Dock"){
              if (sensor_name == "T_Water")  dock_sensor.water_temp = fsensor_value;
              if (sensor_name == "T_bmp180") dock_sensor.temperature = fsensor_value;
              if (sensor_name == "P_bmp180") dock_sensor.pressure = fsensor_value;
              if (sensor_name == "T_dht22")  dock_sensor.temp_dht22 = fsensor_value;
              if (sensor_name == "H_dht22")  dock_sensor.hum_dht22 = fsensor_value;
              if (sensor_name == "ldr1")     dock_sensor.ldr1 = fsensor_value;
              if (sensor_name == "ldr2")     dock_sensor.ldr2 = fsensor_value;
          }
          if (sensor_zone == "OD_1"){
               if (sensor_name == "Temp")    od1_sensor.temperature = fsensor_value;     
               if (sensor_name == "Temp2")   od1_sensor.temp_dht22 = fsensor_value;     
               if (sensor_name == "Hum")     od1_sensor.hum_dht22 = fsensor_value;     
               if (sensor_name == "Light1")  od1_sensor.ldr1 = fsensor_value;     
               if (sensor_name == "Light2")  od1_sensor.ldr2 = fsensor_value;
          } 
      }     
   }  
}

String parse_json(String fromStr, char *tag){
   int pos1;
   int pos2;
   //Serial.println(fromStr); Serial.println(tag);
   pos1 = fromStr.indexOf(tag);
   if (tag == ",\"V"){
      pos1 = fromStr.indexOf(":",pos1) +1;
      pos2 = fromStr.indexOf(",",pos1);
  }
   else{
      pos1 = fromStr.indexOf(":\"",pos1) +2;
      pos2 = fromStr.indexOf("\"",pos1);
   }
    return(fromStr.substring(pos1,pos2));
}



// Helper function definitions
void BsecInitialize(void){
    String output;
    
    iaqSensor.begin(BME680
    _I2C_ADDR_SECONDARY, Wire);
    output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);
    checkIaqSensorStatus();
    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();
    // Print the header
    output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%], Static IAQ, CO2 equivalent, breath VOC equivalent";
    Serial.println(output);
}

void checkIaqSensorStatus(void)
{
    String output;

    if (iaqSensor.status != BSEC_OK) {
        if (iaqSensor.status < BSEC_OK) {
            output = "BSEC error code : " + String(iaqSensor.status);
            Serial.println(output);
            for (;;)  delay(10);
        } else {
          output = "BSEC warning code : " + String(iaqSensor.status);
          Serial.println(output);
        }
    }
   
    if (iaqSensor.bme680Status != BME680_OK) {
      if (iaqSensor.bme680Status < BME680_OK) {
        output = "BME680 error code : " + String(iaqSensor.bme680Status);
        Serial.println(output);
        for (;;)  delay(10);
      } else {
        output = "BME680 warning code : " + String(iaqSensor.bme680Status);
        Serial.println(output);
      }
    }
}


boolean bme_setup(){
  //if (!bme.begin(0x76)) {
  //  Serial.println("Could not find a valid BME680 sensor, check wiring!");
  //  return(false);
  //}

  /* Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  */
  return(true);
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
