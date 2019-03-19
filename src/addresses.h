/*
 * Project oled
 * Description:  provide addresses for thermometer
 * Author: Kirk Carlson
 * Date: 8 Aug 2018
 */

// This file contains the addressed used by OLED

#define NODE_NAME "therm1"

/**
 * if want to use IP address:
 *   byte server[] = { XXX,XXX,XXX,XXX };
 *   MQTT client(server, 1882, callback);
 * want to use domain name:
 *   MQTT client("iot.eclipse.org", 1882, callback);
 **/

byte server[] = { 192,168,4,1 };
