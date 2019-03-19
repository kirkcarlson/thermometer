/*
 * Project MultipleThermometer
 * Description: Read DS18B22 One-wire Thermometers and publish to
 *              particle.io and/or to a local MQTT broker.
 * Author: Kirk Carlson
 * Date: Dec. 20, 2017 - 2019
 *
 * Publishing is controlled with the defines for particlePublishing
 * and mqttPublishing.
 *
 * Note that only 4 message per second are
 * allowed to be published to Particle, with a two second recovery
 * time.
 *
 * light: report abrupt changes
 * temperature: report temperatures as they change
 * four temperatures:
 *   water in
 *   water out
 *   flue
 *   ambient
 *
 * A DHT11 also provides the ambient temperature and humidity.
 *
 * Expect water out to swing the most. Flue may have some fluctuations.
 * water in should be fairly constant although affected by ambient
 * ambient will decrease at night.
 */


//**** LIBRARIES ****
#include <DS18B20.h>
#include <MQTT.h>
#include <Adafruit_DHT.h>
#include "addresses.h"


//**** DEFINES ****

//#define particlePublishing
#ifdef particlePublishing
    #define IF_PARTPUB if(1)
#else
    #define IF_PARTPUB if(0)
#endif

#define mqttPublishing
#ifdef mqttPublishing
    #define IF_MQTTPUB if(1)
#else
    #define IF_MQTTPUB if(0)
#endif

#define DEBUG_MESSAGES
#ifdef DEBUG_MESSAGES
    #define IF_DEBUG if(1)
#else
    #define IF_DEBUG if(0)
#endif

#define DHTTYPE DHT11

// Sensors
//first four are one-wire sensors, one on board and three cables
#define DHT_TEMP 4
#define DHT_HUMID 5
#define LIGHT 6

#define KEEP_ALIVE 60


//**** CONSTANTS ****

const int MAXRETRY = 3;
const int pinLightSensor = A0;
const int pinPower = A5; // regulated power for lightSensor
const int pinOneWire = D0;
const int pinDHT = D1;
const int BUTTON_MANUAL = D2;

const int sizeofTopic = 30;
const int sizeofPayload = 30;
const int sizeofNameCharArray = 2*8 + 1; // 8 bytes to hex characters + null character

const uint32_t sampleTime = 1000;
const uint32_t publishTime = 1 * 60 * 1000; //minimum time between publications
const uint32_t minimumRepeatTime = 500;
const int numberInTemperatureAverage = 3;
const float temperatureReportThreshhold = .5; //degrees C
const float humidityReportThreshhold = 5; //%
const float lightReportThreshhold = 25.0; //units
const int NUM_ONEWIRE_SENSORS = 4; //does not work well if set higher than number installed
const int NUM_SENSORS = NUM_ONEWIRE_SENSORS + 3;
const int pubBurstRecoveryTime = 2000; // milliseconds


//**** CLASSES ****

class Sensor {
    public:
        String nameString;
        char nameCharArray[ sizeofNameCharArray];
        uint8_t address [8];
        float value;
        float lastValueSent;
        uint32_t lastTimeSent;
        uint32_t lastValidTime;
        uint32_t pubDue;

        Sensor( String sName, float sValue, uint32_t lastTime) {
            nameString = sName;
            value = sValue;
            lastValueSent = sValue;
            lastTimeSent = lastTime;
            lastValidTime = lastTime;
            pubDue = lastTime;
        };
};


//**** VARIABLES AND DATA ****

uint32_t lastValidTime[ NUM_ONEWIRE_SENSORS];
float temperatureAverage[ NUM_ONEWIRE_SENSORS];

// setup will initialize Sensor values
Sensor sensors[NUM_SENSORS] = { Sensor("onewire0",0,0),
                                Sensor("onewire1",0,0),
                                Sensor("onewire2",0,0),
                                Sensor("onewire3",0,0),
                                Sensor("DHT_temp",0,0),
                                Sensor("DHT_humid",0,0),
                                Sensor("light",0,0)
                              };

DS18B20          ds18b20( pinOneWire);
DHT              dht ( pinDHT, DHTTYPE);
MQTT             mqttClient( server, 1883, KEEP_ALIVE, receiveMQTT);
//SerialLogHandler logHandler( LOG_LEVEL_WARN);
SerialLogHandler logHandler( LOG_LEVEL_TRACE);


//**** FUNCTIONS ****


#ifdef mqttPublishing
// receive MQTT message
void receiveMQTT(char* topic, byte* payload, unsigned int length) {
    char p[length + 1]; // dynamically allocated??, guess on stack so OK

    memcpy(p, payload, length);
    p[length] = '\0';
}
#endif


// called as:
//  publishFloat ( NODE_NAME, sensors[0].nameString, sensors[0].value)

void publishFloat( String nodeName, String nameString, float value)
{

    char topic [ sizeofTopic];
    char payload [ sizeofPayload];

    (String(nodeName) + "/" + String(nameString)).toCharArray( topic, sizeofTopic);
    String( value,1).toCharArray( payload, sizeofPayload);

    Log.info ( "%s published: %0.2f", topic, value);

    IF_PARTPUB {
        Particle.publish( topic, payload, PRIVATE);
    }
    IF_MQTTPUB {
        mqttClient.publish( topic, payload);
    }
}


// called as:
//  publishInteger ( NODE_NAME, sensorName, sensorValue)

void publishInteger( String nodeName, String nameString, int value)
{
    char topic [ sizeofTopic];
    char payload [ sizeofPayload];

    (String(nodeName) + "/" + String(nameString)).toCharArray( topic, sizeofTopic);
    String( value).toCharArray( payload, sizeofPayload);

    Log.info ( "%s published: %d", topic, value);

    IF_PARTPUB {
        Particle.publish( topic, payload, PRIVATE);
    }
    IF_MQTTPUB {
        mqttClient.publish( topic, payload);
    }
}


double getTemperature(uint8_t addr[8]) {
    double _temp;
    int i = 0;

    do {
        _temp = ds18b20.getTemperature(addr);
    } while (!ds18b20.crcCheck() && MAXRETRY > i++);

    if (i < MAXRETRY) {
        //_temp = ds18b20.convertToFahrenheit(_temp);
    }
    else
    {
        _temp = NAN;
        Log.trace( "One-wire invalid reading");
    }

    return _temp;
}


void checkOWTemperature( int sensorNumber, Sensor *sensor) {
    float temperature = getTemperature( sensor->address);
    uint32_t now = millis();

    if (!isnan(temperature))
    {
        Log.trace( "One-wire sensor %d: %s: %0.2f", sensorNumber,
                sensor->nameCharArray, temperature);
        sensor->value = temperature;
        sensor->lastValidTime = now;
        float change = sensor->lastValueSent - temperature;
        if ( abs( change) > temperatureReportThreshhold || now >= sensor->pubDue) {
            if (now - sensor->lastTimeSent > minimumRepeatTime) {
                publishFloat ( NODE_NAME, sensor->nameString, temperature);
                sensor->lastTimeSent = now;
                sensor->pubDue = now + publishTime;
                sensor->lastValueSent = temperature;
            }
        }
    } else {
        Log.trace( "One-wire sensor bad read: %d: %s", sensorNumber,
                sensor->nameCharArray);
    }
};


void checkLight( Sensor *sensor) {
    uint8_t newLightValue = analogRead( pinLightSensor);
    uint32_t now = millis();

    Log.trace( "Light: %d", newLightValue);
    float change = sensor->lastValueSent - newLightValue;
    if (abs(change) > lightReportThreshhold || now >= sensor->pubDue) {
        if (now - sensor->lastTimeSent > minimumRepeatTime) {
            publishInteger ( NODE_NAME, sensor->nameString, newLightValue);
            sensor->lastTimeSent = now;
            sensor->pubDue = now + publishTime;
            sensor->lastValueSent = newLightValue;
        }
    }
}


void checkDhtHumidity( Sensor *sensor) {
    int change;
    float humidity = dht.getHumidity();
    uint32_t now = millis();

    if (isnan( humidity)) {
        Log.warn( "Failed to read from DHT sensor!");
    } else {
        Log.trace( "DHT Humid: %0.0f%%", humidity);
        change = humidity - sensor->lastValueSent;
        if ( abs( change) > humidityReportThreshhold || now >= sensor->pubDue) {
            if (now - sensor->lastTimeSent > minimumRepeatTime) {
                publishInteger ( NODE_NAME, sensor->nameString, humidity);
                sensor->lastTimeSent = now;
                sensor->pubDue = now + publishTime;
                sensor->lastValueSent = humidity;
            }
        }
    }
}


void checkDhtTemperature( Sensor *sensor) {
    int change;
    float temperature = dht.getTempCelcius();
    uint32_t now = millis();

    if (isnan( temperature)) {
        Log.warn( "Failed to read from DHT sensor!");
    } else {
        Log.trace( "DHT Temp: %0.1f°C", temperature);
        change = temperature - sensor->lastValueSent;
        if ( abs( change) > temperatureReportThreshhold || now >= sensor->pubDue) {
            if (now - sensor->lastTimeSent > minimumRepeatTime) {
                publishFloat ( NODE_NAME, sensor->nameString, temperature);
                sensor->lastTimeSent = now;
                sensor->pubDue = now + publishTime;
                sensor->lastValueSent = temperature;
            }
        }
    }
}


void sensorToCharArray (char *charArray, int sizeofCharArray, uint8_t addr[8])
{
    int result;
    result = snprintf(charArray, sizeofCharArray, "%02X%02X%02X%02X%02X%02X%02X%02X",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]);
    if (result > sizeofCharArray)
    {
        Log.warn( "sensorToCharArray overflowed");
    }
}



void setup() {
    // Local variables
    //uint32_t now = millis();

    // initialize libraries
    dht.begin(); // for the DHT sensor
    ds18b20.resetsearch(); // for the one-wire sensors
    Serial.begin(9600); // for the serial port

    //set up the local switch
    pinMode (BUTTON_MANUAL, INPUT_PULLUP);

    // connect to the MQTT server
    IF_MQTTPUB
    {
        mqttClient.connect(NODE_NAME);

        // publish/subscribe
        if (mqttClient.isConnected()) {
            Log.trace( "MQTT is connected");
        }
        else
        {
            Log.warn( "MQTT did not connect");
        }
    };

    for (int i = 0; i < NUM_ONEWIRE_SENSORS; i++) {   // try to read the sensor addresses
        ds18b20.search(sensors[i].address); // and if available store
        sensorToCharArray( sensors[i].nameCharArray, sizeofNameCharArray,
                sensors[i].address);
        sensors[i].nameString = String( sensors[i].nameCharArray, 16);
    };

    // set up the light sensor
    pinMode( pinLightSensor, INPUT);
    pinMode( pinPower, OUTPUT);
    digitalWrite( pinPower, HIGH);
}



SYSTEM_MODE( MANUAL); // allow manual connect to Particle.io

void loop() {
    if (!WiFi.ready()) {
        //status = "WiFi connecting";
        WiFi.on();
        WiFi.connect();
        //sendHeartbeat();
    }
    if (digitalRead (BUTTON_MANUAL) == LOW) {
        Particle.connect();
    }
    if (Particle.connected()) {
        Particle.process();
    }

#ifdef mqttPublishing
    if (mqttClient.isConnected())
    {
        mqttClient.loop();
    } else {
        Log.trace( "Attempting another MQTT connection");
        mqttClient.connect(NODE_NAME);
    }
#endif

    for (int i = 0; i < NUM_ONEWIRE_SENSORS; i++)
    {
        checkLight( &sensors[ LIGHT]);
        checkOWTemperature( i, &sensors[i]);
    }

    checkLight( &sensors[ LIGHT]);
    checkDhtTemperature( &sensors[DHT_TEMP]);
    checkLight( &sensors[ LIGHT]);
    checkDhtHumidity( &sensors[DHT_HUMID]);
}
