/*
  xdrv_100_palazzetti.ino - Palazzetti support for Tasmota

  Copyright (C) 2024  Flobul

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifdef USE_PALAZZETTI

#define XDRV_100                  100
#define D_NAME_PALAZZETTI         "Palazzetti"
#define PLZ_BAUDRATE              38400
#define PLZ_UDP_PORT              54549
#define MAX_JTIMER                1024 //spaces, tab, CRLF

#define PLZ_SETTINGS_TIMESYNC     1     // 0=OFF; 1=ON
#define PLZ_SETTINGS_MQTT_TOPIC   0     // 0=RAW value in one topic (ex: TELE/T1 = 19.20);
                                        // 1=JSON in category topic (ex: TELE/TMPS = "{\"INFO\":"DATA":{"T1":23.20}...}");
                                        // 2=RAW value in category topic (ex: TELE/TMPS/T1 = 19.20)
#define PLZ_SETTINGS_INTERVAL     15    // in seconds

#define PLZ_SENDMSG_ENDPOINT       "/cgi-bin/sendmsg.lua"
#define PLZ_SYSCMD_ENDPOINT        "/cgi-bin/syscmd.lua"
// Define Std Text Strings
#define PLZ_ERROR_FILE_NOT_FOUND "PLZ: ERROR File system not ready or file not found"


#include "Palazzetti.h"
#include <TasmotaSerial.h>
#include <time.h>
#include <WiFiUdp.h>
#include <sys/socket.h>
#include <errno.h>
#include "zlib.h"
#include <fstream>
#include <strstream>
#include <base64.hpp>
#include <JsonParser.h>
#if defined(ESP32) && defined(USE_ETHERNET)
#include <ETH.h>
#endif

enum Palazzetti_Commands {
    CMND_PALAZZETTI_SENDMSG,
    CMND_PALAZZETTI_INIT,
    CMND_PALAZZETTI_INTERVAL,
    CMND_PALAZZETTI_SYNCCLOCK,
    CMND_PALAZZETTI_MQTT
};

struct {
    Palazzetti::CommandResult cmdRes;
    char data[1536];  // Buffer global pour construire le JSON ("DATA": {"T1": 10.1,"T2":...})
    char info[256];   // Buffer temporaire pour chaque bloc de JSON ("INFO":{"CMD":...)
    char full[1280];  // Taille ajustée pour contenir les deux ({"INFO":{"CMD":"GET+SETP","DATA":...}})
    char msg[128];  // message lors d'erreur
    char cmd[29];  // cmd envoyée
    time_t ts;
    bool success = false;
} PlzIJson;

struct Palazzetti_Request {
    char *command_data = nullptr;   // to store the whole command received
    bool is_connected = false;      // is stove connected
    bool is_get = false;            // is a get request
    bool is_udp = false;            // is a udp request
    bool is_post = false;           // is a post request
    long last_attempt = 0;          // last retry connection attempt
    long retry_interval = 60000;    // retry connection interval
    uint8_t interval_counter = 0;   // counter in seconds
} PlzRequest;

struct Palazzetti_Settings {
    uint32_t crc32;                 // to detect file changes
    uint16_t version;               // to detect driver function changes
    uint8_t sync_clock;             // to sync time of the stove from tasmota 
    uint8_t interval = 15;          // time between refresh requests
    uint8_t mqtt_topic;             // mqtt topic style
} PlzSettings;

struct Palazzetti_JTimer {
    bool enabled = false;
    bool ecostart_mode_enabled = false;
    bool sync_clock_enabled = false;
    bool off_when_no_match = false;
    bool program_match = false;
    char program_id[64] = "PROGRAM_NOMATCH_OFF";
    char last_edit[32] = "";
} PlzJTimer;

struct Plz {
    uint16_t STATUS = 0;
    uint16_t LSTATUS = 0;
    uint16_t FSTATUS = 0;
    float T1 = 0.0f;
    float T2 = 0.0f;
    float T3 = 0.0f;
    float T4 = 0.0f;
    float T5 = 0.0f;
    uint16_t F1V = 0;
    uint16_t F2V = 0;
    uint16_t F1RPM = 0;
    uint16_t F2L = 0;
    uint16_t F2LF = 0;
    bool isF3SF4SValid = false;
    float F3S = 0.0f;
    float F4S = 0.0f;
    bool isF3LF4LValid = false;
    uint16_t F3L = 0;
    uint16_t F4L = 0;
    bool isPWRValid = false;
    uint16_t IGN = 0;
    uint16_t POWERTIMEh = 0;
    uint16_t POWERTIMEm = 0;
    uint16_t HEATTIMEh = 0;
    uint16_t HEATTIMEm = 0;
    uint16_t SERVICETIMEh = 0;
    uint16_t SERVICETIMEm = 0;
    uint16_t ONTIMEh = 0;
    uint16_t ONTIMEm = 0;
    uint16_t OVERTMPERRORS = 0;
    uint16_t IGNERRORS = 0;
    uint16_t PQT = 0;
    char STOVE_DATETIME[20] = { 0 };
    byte STOVE_WDAY = 0;
    float SETP = 0.0f;
    byte PWR = 0;
    float FDR = 0.0f;
    bool isF2LValid = false;
    uint16_t FANLMINMAX[6] = { 0 };
    byte SLNT = 0;
    uint16_t DPT = 0;
    uint16_t DP = 0;
    byte IN_I01 = 0;
    byte IN_I02 = 0;
    byte IN_I03 = 0;
    byte IN_I04 = 0;
    byte OUT_O01 = 0;
    byte OUT_O02 = 0;
    byte OUT_O03 = 0;
    byte OUT_O04 = 0;
    byte OUT_O05 = 0;
    byte OUT_O06 = 0;
    byte OUT_O07 = 0;
    char SN[28] = { 0 };
    uint16_t MOD = 0;
    uint16_t VER = 0;
    uint16_t CORE = 0;
    char FWDATE[11] = { 0 };
    byte CHRSTATUS = 0;
    float PCHRSETP[6] = {0.0f};
    byte PSTART[6][2] = {{ 0 }};
    byte PSTOP[6][2] = {{ 0 }};
    byte DM[7][3] = {{ 0 }};
    int MBTYPE = 0;
    char APLTS[20] = { 0 };
    uint16_t APLWDAY = 0;
    bool isMFSTATUSValid = false;
    uint16_t MFSTATUS = 0;
    byte PUMP = 0;
    byte IN = 0;
    byte OUT = 0;
    bool isSNValid = false;
    byte SNCHK = 0;
    uint16_t FLUID = 0;
    uint16_t SPLMIN = 0;
    uint16_t SPLMAX = 0;
    byte UICONFIG = 0;
    byte HWTYPE = 0;
    byte DSPTYPE = 0;
    byte DSPFWVER = 0;
    byte CONFIG = 0;
    byte PELLETTYPE = 0;
    uint16_t PSENSTYPE = 0;
    byte PSENSLMAX = 0;
    byte PSENSLTSH = 0;
    byte PSENSLMIN = 0;
    byte MAINTPROBE = 0;
    byte STOVETYPE = 0;
    byte FAN2TYPE = 0;
    byte FAN2MODE = 0;
    byte BLEMBMODE = 0;
    byte BLEDSPMODE = 0;
    byte CHRONOTYPE = 0;
    byte AUTONOMYTYPE = 0;
    byte NOMINALPWR = 0;
    byte params[0x6A] = { 0 };
    uint16_t hiddenParams[0x6F] = { 0 };
    uint16_t ADDR_DATA = 0;
} Plz;

struct Palazzetti_Scenario_Settings {
    int setPoint;
    int setPower;
    int setFan;
    int setFan3;
    int setFan4;
    int setSilent;
};

Palazzetti pala;
TasmotaSerial *plzSerial = nullptr;
WiFiUDP plzUdpServer;
bool plzUdpServerActive = false;
uint8_t pala_rx_pin = NOT_A_PIN;
uint8_t pala_tx_pin = NOT_A_PIN;
uint8_t pala_detect_pin = NOT_A_PIN;
const uint16_t PLZ_SETTINGS_VERSION = 0x0100;
unsigned long last_all_status_refresh = 0;
static char last_stove_datetime[20] = { 0 };
time_t getTimeStamp() {
    return time(nullptr);
}
Palazzetti_Scenario_Settings scenarioSettings[4] = {
    {-1, -1, -1, -1, -1, -1}, // off
    {18, -1, -1, -1, -1, -1}, // economy
    {22, -1, -1, -1, -1, -1}, // comfort
    {26, -1, -1, -1, -1, -1}  // warm
};
const char kScenarioSettings[] PROGMEM = "SET SETP|SET POWR|SET RFAN|SET FN3L|SET FN4L|SET SLNT";
const char* scenarioNames[] = {"off", "economy", "comfort", "warm"};

const char TIME_HOUR_MINUTE[]            PROGMEM = "%02d:%02d";
const char HTTP_PALAZZETTI_TITLE[]       PROGMEM = "{s}%s{m}%s{e}";
const char HTTP_PALAZZETTI_TITLE_S[]     PROGMEM = "{s}%s{m}%s ";
const char HTTP_PALAZZETTI_SIMPLE[]      PROGMEM = "{s}%s{m}%d %s{e}";
const char HTTP_PALAZZETTI_POURCENT[]    PROGMEM = "{s}%s{m}%d " D_UNIT_PERCENT" {e}";
const char HTTP_PALAZZETTI_TIME[]        PROGMEM = "{s}%s{m}%d " D_UNIT_HOUR " %d " D_UNIT_MINUTE "{e}";
const char HTTP_PALAZZETTI_TEMPERATURE[] PROGMEM = "{s}%s{m}%s " D_UNIT_DEGREE "%c{e}";
const char HTTP_PALAZZETTI_THERMOSTAT[]  PROGMEM = "{s}%s{m}%s " D_UNIT_DEGREE "%c ";
const char HTTP_PALAZZETTI_TITLE_D[]     PROGMEM = "{s}%s{m}%d ";
const char HTTP_PALAZZETTI_POWER_BTN[]   PROGMEM = "<button style=\"background:#%s\" onclick=\"fetch('" PLZ_SENDMSG_ENDPOINT "?cmd=%s')\">%s</button>";
const char HTTP_PALAZZETTI_SLIDER_BTN2[] PROGMEM = "<button onclick=\"fetch('" PLZ_SENDMSG_ENDPOINT "?cmd=%s');\" style='width:15%%'>-</button>"
                                                       "<input type='range' min='%d' max='%d' step='1' value='%s' onchange=\"fetch('" PLZ_SENDMSG_ENDPOINT "?cmd=%s+this.value');\"  style='width:69%%'/>"
                                                       "<button onclick=\"fetch('" PLZ_SENDMSG_ENDPOINT "?cmd=%s');\" style='width:15%%'>+</button>";

const char HTTP_PALAZZETTI_SLIDER_BTN[]  PROGMEM = "<td><button onclick=\"fetch('" PLZ_SENDMSG_ENDPOINT "?cmd=%s');\">-</button></td>"
                                                       "<td><input type='range' min='%d' max='%d' step='1' value='%s' onchange=\"fetch('" PLZ_SENDMSG_ENDPOINT "?cmd=%s+this.value');\" /></td>"
                                                       "<td><button onclick=\"fetch('" PLZ_SENDMSG_ENDPOINT "?cmd=%s');\">+</button>{e}";
const char kPalazzetti_Commands[]        PROGMEM = "Sendmsg|Init|Interval|Sync|Mqtt";
const char JSON_STR_TEMPLATE[]           PROGMEM = "\"%s\":\"%s\",";
const char JSON_INT_TEMPLATE[]           PROGMEM = "\"%s\":%d,";
const char JSON_ELE_TEMPLATE[]           PROGMEM = "%d,";
const char JSON_BOOL_TEMPLATE[]          PROGMEM = "\"%s\":%s,";
const char JSON_OBJ_TEMPLATE[]           PROGMEM = "\"%s\":{";
const char JSON_ARRAY_TEMPLATE[]         PROGMEM = "\"%s\":[";
const char JSON_INFO_TEMPLATE[]          PROGMEM = "\"INFO\":{";
const char JSON_OBJ_OPEN[]               PROGMEM = "{";
const char JSON_OBJ_CLOSE[]              PROGMEM = "}";
const char JSON_ARRAY_CLOSE[]            PROGMEM = "]";
const char JSON_COMMA[]                  PROGMEM = ",";
const char JSON_OBJ_CLOSE_COMMA[]        PROGMEM = "},";
const char JSON_CMD[]                    PROGMEM = "CMD";
const char JSON_RSP[]                    PROGMEM = "RSP";
const char JSON_TS[]                     PROGMEM = "TS";
const char JSON_MSG[]                    PROGMEM = "MSG";


#ifdef USE_WEBSERVER

#endif // USE_WEBSERVER

#define D_PLZ_POWER "Puissance"
#define D_PLZ_POWER_ON "Allumer"
#define D_PLZ_POWER_OFF "Éteindre"
#define D_PLZ_SET_POINT "Consigne"
#define D_PLZ_T1 "Température Ambiante / Départ"
#define D_PLZ_T2 "Température Sécurité / Retour"
#define D_PLZ_T3 "Température Fumées"
#define D_PLZ_T4 "Température MultiFire CC"
#define D_PLZ_T5 "Température Ambiante / Accumulateur"
#define D_PLZ_MAIN_FAN "Ventilateur central"
#define D_PLZ_FAN4 "Ventilateur droit"
#define D_PLZ_FAN3 "Ventilateur gauche"
#define D_PLZ_F1RPM "Extracteur fumées"
#define D_PLZ_F1V "Vitesse Ventilation F1V"
#define D_PLZ_F2V "Vitesse Ventilation F2V"
#define D_PLZ_POWERTIME "Durée alimentation électrique"
#define D_PLZ_HEATTIME "Durée de chauffage"
#define D_PLZ_SERVICETIME "Durée depuis dernier entretien"
#define D_PLZ_PQT "Pellets brûlés"
#define D_PLZ_SLNT_ON "Silencieux"
#define D_PLZ_SLNT_OFF "Normal"

#define D_PLZ_STATUS_0 "Eteint"
#define D_PLZ_STATUS_1 "Arrêté par minuterie"
#define D_PLZ_STATUS_2 "Essai d’allumage"
#define D_PLZ_STATUS_3 "Chargement granulés"
#define D_PLZ_STATUS_4 "Allumage"
#define D_PLZ_STATUS_5 "Contrôle combustion"
#define D_PLZ_STATUS_6 "En marche"
#define D_PLZ_STATUS_7 "En marche - Modulation"
#define D_PLZ_STATUS_8 "-"
#define D_PLZ_STATUS_9 "Stand-by"
#define D_PLZ_STATUS_10 "Extinction"
#define D_PLZ_STATUS_11 "Nettoyage brasier"
#define D_PLZ_STATUS_12 "Attente refroidissement"
#define D_PLZ_STATUS_51 "Eco mode"
#define D_PLZ_STATUS_241 "Erreur Nettoyage"
#define D_PLZ_STATUS_243 "Erreur Grille"
#define D_PLZ_STATUS_244 "NTC2 ALARM"
#define D_PLZ_STATUS_245 "NTC3 ALARM"
#define D_PLZ_STATUS_247 "Erreur Porte"
#define D_PLZ_STATUS_248 "Erreur Dépression"
#define D_PLZ_STATUS_249 "NTC1 ALARM"
#define D_PLZ_STATUS_250 "TC1 ALARM"
#define D_PLZ_STATUS_252 "Erreur évacuation Fumées"
#define D_PLZ_STATUS_253 "Pas de pellets"
#define D_PLZ_STATUS_501 "Éteint"
#define D_PLZ_STATUS_502 "Allumage"
#define D_PLZ_STATUS_503 "Contrôle combustion"
#define D_PLZ_STATUS_504 "En marche"
#define D_PLZ_STATUS_505 "Bûches terminées"
#define D_PLZ_STATUS_506 "Refroidissement"
#define D_PLZ_STATUS_507 "Nettoyage brasier"
#define D_PLZ_STATUS_509 "Allumage avec granulés"
#define D_PLZ_STATUS_1239 "Porte ouverte"
#define D_PLZ_STATUS_1240 "Température trop haute"
#define D_PLZ_STATUS_1241 "Avertissement nettoyage"
#define D_PLZ_STATUS_1243 "Erreur combustion"
#define D_PLZ_STATUS_1244 "Erreur sonde granulés ou eau de retour"
#define D_PLZ_STATUS_1245 "Erreur T05 sonde non raccordée ou bien défectueuse"
#define D_PLZ_STATUS_1247 "Porte ou couvercle de charge ouverts"
#define D_PLZ_STATUS_1248 "Erreur pressostat de sécurité"
#define D_PLZ_STATUS_1249 "Mauvais fonctionnement sonde principale"
#define D_PLZ_STATUS_1250 "Mauvais fonctionnement sonde fumées"
#define D_PLZ_STATUS_1252 "Température trop haute des fumées en sortie"
#define D_PLZ_STATUS_1253 "Granulés terminés ou non allumage"
#define D_PLZ_STATUS_1508 "Erreur générique - Voir Notice"
#define D_PLZ_STATUS_UNKNOWN "Erreur inconnu"
#define D_PLZ_STOVETYPE_UNKNOWN "Inconnu";
#define D_PLZ_STOVETYPE_AIR "Circulation d'air";
#define D_PLZ_STOVETYPE_WATER "Hydraulique";
#define D_PLZ_STOVETYPE_MULTIFIRE_AIR "Mixte bois/granulés air";
#define D_PLZ_STOVETYPE_MULTIFIRE_IDRO "Mixte bois/granulés hydraulique";
#define D_PLZ_STOVETYPE_KITCHEN_AIR "Cuisinière";
#define D_PLZ_STOVETYPE_KITCHEN_IDRO "Cuisinière hydrolique";
#define D_PLZ_HIGH "HAUT";
#define D_PLZ_PROP "Proportionel";

const char* commandResultToString() {
    switch (PlzIJson.cmdRes) {
        case Palazzetti::CommandResult::OK:
            return "OK";
        case Palazzetti::CommandResult::ERROR:
            return "ERROR";
        case Palazzetti::CommandResult::COMMUNICATION_ERROR:
            return "COMM_ERROR";
        case Palazzetti::CommandResult::BUSY:
            return "BUSY";
        case Palazzetti::CommandResult::UNSUPPORTED:
            return "UNSUPPORTED";
        case Palazzetti::CommandResult::PARSER_ERROR:
            return "PARSER_ERROR";
        default:
            return "UNKNOWN";
    }
}

const char* getPlzStatusMessage(uint16_t statusCode) {
    switch (statusCode) {
        case 0:   return D_PLZ_STATUS_0;
        case 1:   return D_PLZ_STATUS_1;
        case 2:   return D_PLZ_STATUS_2;
        case 3:   return D_PLZ_STATUS_3;
        case 4:   return D_PLZ_STATUS_4;
        case 5:   return D_PLZ_STATUS_5;
        case 6:   return D_PLZ_STATUS_6;
        case 7:   return D_PLZ_STATUS_7;
        case 8:   return D_PLZ_STATUS_8;
        case 9:   return D_PLZ_STATUS_9;
        case 10:  return D_PLZ_STATUS_10;
        case 11:  return D_PLZ_STATUS_11;
        case 12:  return D_PLZ_STATUS_12;
        case 51:  return D_PLZ_STATUS_51;
        case 241: return D_PLZ_STATUS_241;
        case 243: return D_PLZ_STATUS_243;
        case 244: return D_PLZ_STATUS_244;
        case 245: return D_PLZ_STATUS_245;
        case 247: return D_PLZ_STATUS_247;
        case 248: return D_PLZ_STATUS_248;
        case 249: return D_PLZ_STATUS_249;
        case 250: return D_PLZ_STATUS_250;
        case 252: return D_PLZ_STATUS_252;
        case 253: return D_PLZ_STATUS_253;
        case 501: return D_PLZ_STATUS_501;
        case 502: return D_PLZ_STATUS_502;
        case 503: return D_PLZ_STATUS_503;
        case 504: return D_PLZ_STATUS_504;
        case 505: return D_PLZ_STATUS_505;
        case 506: return D_PLZ_STATUS_506;
        case 507: return D_PLZ_STATUS_507;
        case 509: return D_PLZ_STATUS_509;
        case 1239: return D_PLZ_STATUS_1239;
        case 1240: return D_PLZ_STATUS_1240;
        case 1241: return D_PLZ_STATUS_1241;
        case 1243: return D_PLZ_STATUS_1243;
        case 1244: return D_PLZ_STATUS_1244;
        case 1245: return D_PLZ_STATUS_1245;
        case 1247: return D_PLZ_STATUS_1247;
        case 1248: return D_PLZ_STATUS_1248;
        case 1249: return D_PLZ_STATUS_1249;
        case 1250: return D_PLZ_STATUS_1250;
        case 1252: return D_PLZ_STATUS_1252;
        case 1253: return D_PLZ_STATUS_1253;
        case 1508: return D_PLZ_STATUS_1508;
        default:  return D_PLZ_STATUS_UNKNOWN;
    }
}

const char* getPlzStoveType(uint16_t stoveType) {
    switch (stoveType) {
        case 0:   return D_PLZ_STOVETYPE_UNKNOWN;
        case 1:   return D_PLZ_STOVETYPE_AIR;
        case 2:   return D_PLZ_STOVETYPE_WATER;
        case 3:   return D_PLZ_STOVETYPE_MULTIFIRE_AIR;
        case 4:   return D_PLZ_STOVETYPE_MULTIFIRE_IDRO;
        case 5:   return D_PLZ_STOVETYPE_KITCHEN_AIR;
        case 6:   return D_PLZ_STOVETYPE_KITCHEN_IDRO;
        default:  return D_PLZ_STOVETYPE_UNKNOWN;
    }
}

const char* getPlzFanStatus(uint16_t fanCode) {
    switch (fanCode) {
        case 0:   return D_OFF;
        case 1:   return "1";
        case 2:   return "2";
        case 3:   return "3";
        case 4:   return "4";
        case 5:   return "5";
        case 6:   return D_PLZ_HIGH;
        case 7:   return D_AUTO;
        case 8:   return D_PLZ_PROP;
        default:  return "Unknown";
    }
}

bool checkStrtolError(const char* str, char* endptr) {
    return (str == endptr || errno != 0) ? true : false;
}

bool isValidFileType(const char* fileType) {
    return (strcmp(fileType, "CSV") == 0 || strcmp(fileType, "JSON") == 0);
}

int compress(const std::string str, std::string outstring, int compressionlevel)
{
  z_stream zs;                        // z_stream is zlib's control structure
  memset(&zs, 0, sizeof(zs));

  if (deflateInit2(&zs,compressionlevel,Z_DEFLATED, 9, 1, Z_FIXED) != Z_OK)
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Output buffer fucked !"));

  zs.next_in = (Bytef*)str.data();
  zs.avail_in = str.size();           // set the z_stream's input

  int ret;
  char outbuffer[256];


  // retrieve the compressed bytes blockwise
  do {
    zs.next_out = reinterpret_cast<Bytef*>(outbuffer);
    zs.avail_out = sizeof(outbuffer);

    ret = deflate(&zs, Z_FINISH);
    ESP_LOGD("TAG","hey");
    if (outstring.size() < zs.total_out) {
      // append the block to the output string
      outstring.append(outbuffer,
                       zs.total_out - outstring.size());
    }
  } while (ret == Z_OK);

  deflateEnd(&zs);

  if (ret != Z_STREAM_END) {          // an error occurred that was not EOF
                AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Output buffer too small!"));

  }

  return outstring.size() > 0;
}

char* PlzEncodeGzip(const char* data, size_t data_len, size_t* encoded_len) {
    z_stream deflate_s;
	deflate_s.zalloc = Z_NULL;
	deflate_s.zfree = Z_NULL;
	deflate_s.opaque = Z_NULL;
	deflate_s.avail_in = 0;
	deflate_s.next_in = Z_NULL;
    int ret = -1;


    ret = deflateInit2(&deflate_s, -1, Z_DEFLATED, MAX_WBITS + 15, 2, Z_DEFAULT_STRATEGY);
    if (ret != Z_OK) {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: deflateInit2 failed with error code: %d"), ret);
        return NULL;
    } else {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: deflateInit2 success with code: %d"), ret);
    }
 
    unsigned char* out = (unsigned char*)malloc(MAX_JTIMER);
    if (out == NULL) {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Failed to allocate output buffer!"));
        deflateEnd(&deflate_s);
        return NULL;
    }

    deflate_s.avail_in = data_len;
    deflate_s.next_in = (unsigned char*)data;
    deflate_s.avail_out = MAX_JTIMER;
    deflate_s.next_out = out;

    while ((ret = deflate(&deflate_s, Z_FINISH)) != Z_STREAM_END) {
        if (ret != Z_OK) {
            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: deflate failed: %d"), ret);
            free(out);
            deflateEnd(&deflate_s);
            return NULL;
        }
        if (deflate_s.avail_out == 0) {
            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Output buffer too small!"));
            free(out);
            deflateEnd(&deflate_s);
            return NULL;
        }
    }

    if (deflateEnd(&deflate_s) != Z_OK) {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: deflateEnd failed!"));
        free(out);
        return NULL;
    }

    size_t compressed_len = MAX_JTIMER - deflate_s.avail_out;

    unsigned char* base64_encoded = (unsigned char*)malloc(compressed_len * 4 / 3 + 4);
    if (base64_encoded == NULL) {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Failed to allocate base64 buffer!"));
        free(out);
        return NULL;
    }

    size_t base64_len = encode_base64(out, compressed_len, base64_encoded);
    if (base64_len == 0) {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Base64 encoding failed!"));
        free(out);
        free(base64_encoded);
        return NULL;
    }

    free(out);
    *encoded_len = base64_len;
    return (char*)base64_encoded;
}

// Fonction pour décoder en Base64 et décompresser en GZIP
char *PlzDecodeGunzip(const char *input, size_t input_len, size_t *output_len) {
    uint8 *decoded_data = (uint8 *)malloc(input_len);
    size_t decoded_len = decode_base64((uint8 *)input, decoded_data);
    if (decoded_len == 0) {
        AddLog(LOG_LEVEL_INFO, PSTR("Base64 decode failed!"));
        free(decoded_data);
        return NULL;
    }

    z_stream gzip_decomp_stream;
    memset(&gzip_decomp_stream, 0, sizeof(gzip_decomp_stream));

    gzip_decomp_stream.next_in = decoded_data;
    gzip_decomp_stream.avail_in = decoded_len;
    uint8 *pUncomp = (uint8 *)malloc(input_len * 2); // Taille estimée pour la décompression
    gzip_decomp_stream.next_out = pUncomp;
    gzip_decomp_stream.avail_out = input_len * 2;
    int initInflate = inflateInit2(&gzip_decomp_stream, MAX_WBITS + 32);
    if (initInflate != Z_OK) {
        AddLog(LOG_LEVEL_INFO, PSTR("GZIP decompression initialization failed %d!"), initInflate);
        free(decoded_data);
        free(pUncomp);
        return NULL;
    }

    if (inflate(&gzip_decomp_stream, Z_FINISH) != Z_STREAM_END) {
        AddLog(LOG_LEVEL_INFO, PSTR("GZIP decompression failed!"));
        inflateEnd(&gzip_decomp_stream);
        free(decoded_data);
        free(pUncomp);
        return NULL;
    }

    inflateEnd(&gzip_decomp_stream);
    *output_len = gzip_decomp_stream.total_out;
    free(decoded_data);
    return (char *)pUncomp;
}

void PlzJSONAddStr(const char* key, const char* value) {
    char temp[64];
    snprintf_P(temp, sizeof(temp), JSON_STR_TEMPLATE, key, value);
    strcat_P(PlzIJson.data, temp);
}

void PlzJSONAddInt(const char* key, int value) {
    char temp[32];
    snprintf_P(temp, sizeof(temp), JSON_INT_TEMPLATE, key, value);
    strcat_P(PlzIJson.data, temp);
}

void PlzJSONAddIntArr(int value) {
    char temp[32];
    snprintf_P(temp, sizeof(temp), JSON_ELE_TEMPLATE, value);
    strcat_P(PlzIJson.data, temp);
}

void PlzJSONAddBool(const char* key, bool value) {
    char temp[32];
    snprintf_P(temp, sizeof(temp), JSON_BOOL_TEMPLATE, key, value ? PSTR("true") : PSTR("false"));
    strcat_P(PlzIJson.data, temp);
}

void PlzJSONAddFloat(const char *key, float value) {//hack float
    char temp[32];
    String formattedValue = String(value, 2);
    snprintf(temp, sizeof(temp), JSON_BOOL_TEMPLATE, key, formattedValue.c_str());
    strcat_P(PlzIJson.data, temp);
}

void PlzJSONAddObj(const char* data) {
    char temp[256];
    snprintf_P(temp, sizeof(temp), JSON_OBJ_TEMPLATE, data);
    strcat_P(PlzIJson.data, temp);
}

void PlzJSONAddArray(const char* data) {
    char temp[256];
    snprintf_P(temp, sizeof(temp), JSON_ARRAY_TEMPLATE, data);
    strcat_P(PlzIJson.data, temp);
}

void PlzJSONAddArray(const char* key, uint16_t* values, int count) {
    char temp[128];
    int offset = 0;
    offset += snprintf_P(temp, sizeof(temp), PSTR("\"%s\":["), key);
    for (int i = 0; i < count; ++i) {
        if (i > 0) {
            offset += snprintf_P(temp + offset, sizeof(temp) - offset, JSON_COMMA);
        }
        offset += snprintf_P(temp + offset, sizeof(temp) - offset, PSTR("%d"), values[i]);
    }
    offset += snprintf_P(temp + offset, sizeof(temp) - offset, PSTR("]"));
    strcat_P(PlzIJson.data, temp);
}

void PlzJSONAddCSV(const char* key, int value) {
    char line[50];
    snprintf(line, sizeof(line), "%s;%d\r\n", key, value);
    strncat(PlzIJson.data, line, sizeof(PlzIJson.data) - strlen(PlzIJson.data) - 1);
}

void PlzJSONMerge() {
    snprintf_P(PlzIJson.info, sizeof(PlzIJson.info), JSON_INFO_TEMPLATE);

    char temp[256];
    snprintf_P(temp, sizeof(temp), JSON_STR_TEMPLATE, JSON_CMD, PlzIJson.cmd);
    strcat_P(PlzIJson.info, temp);

    snprintf_P(temp, sizeof(temp), JSON_STR_TEMPLATE, JSON_RSP, commandResultToString());
    strcat_P(PlzIJson.info, temp);

    snprintf_P(temp, sizeof(temp), JSON_INT_TEMPLATE, JSON_TS, getTimeStamp());
    strcat_P(PlzIJson.info, temp);

    if (PlzIJson.msg[0] != 0/* && strcmp(commandResultToString(), "OK") != 0*/) {        
        snprintf_P(temp, sizeof(temp), JSON_STR_TEMPLATE, JSON_MSG, PlzIJson.msg);
        strcat_P(PlzIJson.info, temp);
    }

    PlzJSONRemoveComma(PlzIJson.info);
    PlzJSONRemoveComma(PlzIJson.data);

    strcat_P(PlzIJson.info, JSON_OBJ_CLOSE_COMMA);

    snprintf_P(PlzIJson.full, sizeof(PlzIJson.full), PSTR("{%s%s}"), PlzIJson.info, PlzIJson.data);
}

void PlzJSONOpenObj() {
    strcat_P(PlzIJson.data, JSON_OBJ_OPEN);
}

void PlzJSONCloseObj() {
    PlzJSONRemoveComma(PlzIJson.data);
    strcat_P(PlzIJson.data, JSON_OBJ_CLOSE);
}

void PlzJSONCloseArray() {
    PlzJSONRemoveComma(PlzIJson.data);
    strcat_P(PlzIJson.data, JSON_ARRAY_CLOSE);
}

void PlzJSONAddComma() {
    strcat_P(PlzIJson.data, JSON_COMMA);
}

void PlzJSONRemoveComma(char* json) {
    int len = strlen(json);
    if (len > 0 && json[len - 1] == ',') {
        json[len - 1] = '\0';
    }
}

/* ======================================================================
Function: PlzDrvInit
Purpose : Tasmota core driver init
Input   : -
Output  : -
Comments: -
====================================================================== */
void PlzDrvInit(void)
{
    if (PinUsed(GPIO_PALAZZETTI_RX) && PinUsed(GPIO_PALAZZETTI_TX)) {
        pala_rx_pin = Pin(GPIO_PALAZZETTI_RX);
        pala_tx_pin = Pin(GPIO_PALAZZETTI_TX);
        pala_detect_pin = Pin(GPIO_PALAZZETTI_DETECT);
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Pin Rx used %d, Tx %d and Det %d"), pala_rx_pin, pala_tx_pin, pala_detect_pin);
        memset(&Plz, 0, sizeof(Plz));
    }
}

/* ======================================================================
Function: PlzInit
Purpose : Tasmota core device init
Input   : -
Output  : -
Comments: -
====================================================================== */
void PlzInit(void)
{
    if (!PinUsed(GPIO_PALAZZETTI_RX) || !PinUsed(GPIO_PALAZZETTI_TX)) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Pin Rx used %d and Tx %d"), GPIO_PALAZZETTI_RX, GPIO_PALAZZETTI_TX);
        return;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: RX on GPIO%d, TX on GPIO%d, Det on GPIO%d, baudrate %d"), pala_rx_pin, pala_tx_pin, pala_detect_pin, PLZ_BAUDRATE);
    
    if (!plzSerial) {
#ifdef ESP8266
        plzSerial = new TasmotaSerial(pala_rx_pin, pala_tx_pin, 2, 0);
#endif // ESP8266
#ifdef ESP32
        plzSerial = new TasmotaSerial(pala_rx_pin, pala_tx_pin, 1, 0);
#endif  // ESP32
    }

    PlzInitJSON();
    memset(&Plz, 0, sizeof(Plz));
/*
#ifdef ESP8266
    uint8_t hwDetectPin = 5; //GPIO5
#endif // ESP8266
#ifdef ESP32
    uint8_t hwDetectPin = 22; //GPIO22
#endif*/
    int hwVersion = PlzDetectHardWare();
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Hardware detected %d"), hwVersion);

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Initializing serial..."));
    PlzIJson.cmdRes = pala.initialize(
        std::bind(&PlzOpenSerial, std::placeholders::_1),
        std::bind(&PlzCloseSerial),
        std::bind(&PlzSelectSerial, std::placeholders::_1),
        std::bind(&PlzReadSerial, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlzWriteSerial, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlzDrainSerial),
        std::bind(&PlzFlushSerial),
        std::bind(&PlzUSleep, std::placeholders::_1),
        hwVersion == 1 // detect Hw and check pin HIGH
    );

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Connecting to Palazzetti stove..."));
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Palazzetti connected"));
        PlzRequest.is_connected = true;
        PlzIJson.cmdRes = pala.getSN(&Plz.SN);
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande response Plz.SN=%s"), Plz.SN);
       
        int32_t diff = UpdateDevicesPresent(1); // claim a power device slot
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande UpdateDevicesPresent %d"), diff);

        PlzUpdate();
    } else {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Palazzetti stove connection failed"));
    }

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Shutting down UDP Server..."));
    if (plzUdpServerActive) {
        plzUdpServer.clear();
        plzUdpServer.stop();
        plzUdpServerActive = false;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Initializing UDP Server..."));
    if (TasmotaGlobal.restart_flag || TasmotaGlobal.global_state.network_down) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Tasmota restarted or network down"));
        return;
    }
    if (!plzUdpServerActive) {
        if (!plzUdpServer.begin(PLZ_UDP_PORT)) {
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: UDP server is down"));
            return;
        }
        plzUdpServerActive = true;
        AddLog(LOG_LEVEL_DEBUG_MORE, "PLZ: UDP Listener Started: Normal Scheme");
    }
}

int PlzDetectHardWare() {
    pinMode(pala_detect_pin, INPUT_PULLUP);
    delay(1);
    return (digitalRead(pala_detect_pin) == HIGH) ? 1 : 2;
}

/* ======================================================================
Function: PlzSaveBeforeRestart
Purpose : 
Input   : -
Output  : -
Comments: -
====================================================================== */
void PlzSaveBeforeRestart()
{
    if (PinUsed(GPIO_PALAZZETTI_TX)) {
        pinMode(Pin(GPIO_PALAZZETTI_TX), OUTPUT);
        digitalWrite(Pin(GPIO_PALAZZETTI_TX), HIGH);
    }
}

/* ======================================================================
Function: PlzOpenSerial
Purpose : Ouverture du port série
====================================================================== */
int PlzOpenSerial(uint32_t baudrate)
{
    if (plzSerial && plzSerial->begin(baudrate)) {
#ifdef ESP8266
        if (plzSerial->hardwareSerial() ) {
            ClaimSerial();

            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: using hardware serial"));
        } else {
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: using software serial"));
        }
#endif // ESP8266
#ifdef ESP32
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: using hardserial %d"), plzSerial->getUart());
#endif // ESP32
    } else {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Serial initialization failed"));
        //return -1;
    }
    return 0;
}

/* ======================================================================
Function: PlzCloseSerial
Purpose : Fermeture du port série
====================================================================== */
void PlzCloseSerial()
{
    plzSerial->end();
    pinMode(GPIO_PALAZZETTI_TX, OUTPUT);
    digitalWrite(GPIO_PALAZZETTI_TX, HIGH);
}

/* ======================================================================
Function: PlzSelectSerial
Purpose : Sélection du port série
====================================================================== */
int PlzSelectSerial(unsigned long timeout)
{
    size_t avail;
    unsigned long startmillis = millis();
    while ((avail = plzSerial->available()) == 0 && (startmillis + timeout) > millis())
        ;

    return avail;
}

/* ======================================================================
Function: PlzReadSerial
Purpose : Lecture sur le port série
====================================================================== */
size_t PlzReadSerial(void *buf, size_t count)
{
    return plzSerial->readBytes((char *)buf, count);
}

/* ======================================================================
Function: PlzWriteSerial
Purpose : Écriture sur le port série
====================================================================== */
size_t PlzWriteSerial(const void *buf, size_t count)
{
    return plzSerial->write((const uint8_t *)buf, count);
}

/* ======================================================================
Function: PlzDrainSerial
Purpose : Vider le buffer
====================================================================== */
int PlzDrainSerial()
{
    plzSerial->flush();
    return 0;
}

/* ======================================================================
Function: PlzFlushSerial
Purpose : Vider le buffer de réception et envoi
====================================================================== */
int PlzFlushSerial()
{
    plzSerial->flush();
    while (plzSerial->read() != -1)
        ;
    return 0;
}

/* ======================================================================
Function: PlzUSleep
Purpose : Pause en microsecondes
====================================================================== */
void PlzUSleep(unsigned long usecond)
{
    delayMicroseconds(usecond);
}

void PlzHandleUdpRequest() {
    if (plzUdpServerActive) {
        int packetSize = plzUdpServer.parsePacket();
        if (packetSize <= 0) return;

        String strData;
        strData.reserve(packetSize + 1);

        int bufferByte;
        while ((bufferByte = plzUdpServer.read()) >= 0) {
            strData += (char)bufferByte;
        }

        PlzRequest.is_udp = true;

        if (strData.endsWith("bridge?")) {
            PlzRequest.command_data = (char*)malloc(strlen("GET STDT") + 1);
            if (PlzRequest.command_data != nullptr) {
                strcpy(PlzRequest.command_data, "GET STDT");
                PlzParser();
                free(PlzRequest.command_data);
                PlzRequest.command_data = nullptr;
            }
        } else if (strData.endsWith("bridge?GET ALLS")) {
            PlzRequest.command_data = (char*)malloc(strlen("GET ALLS") + 1);
            if (PlzRequest.command_data != nullptr) {
                strcpy(PlzRequest.command_data, "GET ALLS");
                PlzParser();
                free(PlzRequest.command_data);
                PlzRequest.command_data = nullptr;
            }
        } else {
            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Unrecognized UDP command received"));
        }

        if (PlzIJson.full[0] != '\0') {
            plzUdpServer.beginPacket(plzUdpServer.remoteIP(), plzUdpServer.remotePort());
            size_t length = strlen(PlzIJson.full);
            size_t bytesSent = plzUdpServer.write((const uint8_t *)PlzIJson.full, length);

            if (bytesSent == length) {
                AddLog(LOG_LEVEL_INFO, PSTR("PLZ: UDP message sent successfully: %s"), PlzIJson.full);
            } else {
                AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Failed to send UDP message"));
            }
            plzUdpServer.endPacket();
        } else {
            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: JSON message is empty"));
        }
    } else {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: UDP server is not initialized"));
        PlzInit();
    }
}

/* ======================================================================
Function: PlzShow
Purpose : Display Palazzetti infos on WEB Interface
====================================================================== */
void PlzShow(bool json) 
{
    if (json) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: PlzShow json"));
        //WSContentSend_P(PSTR("{\"data\": \""), currentAllStatus.c_str(), PSTR("}"));
        return;
    }
#ifdef USE_WEBSERVER
    else {
        // {t}            = <table style='width:100%%'>
        // {s}            = <tr><th>
        // {m}            = </th><td style='width:20px;white-space:nowrap'>
        // {e}            = </td></tr>
        // HTTP_TABLE100  = <table style='width:100%%'>

        bool hasSetPoint    = (Plz.SETP != 0);
        bool hasPower       = (Plz.STOVETYPE != 8);
        bool hasSwitch      = (Plz.STOVETYPE != 7 && Plz.STOVETYPE != 8);
        bool hasFirstFan    = (Plz.FAN2TYPE > 1);
        bool hasSecondFan   = (Plz.FAN2TYPE > 2); //hasZeroSpeedFan
        bool hasThirdFan    = (Plz.FAN2TYPE > 3);
        bool hasRemoteProbe = ((Plz.BLEMBMODE == 7 || Plz.BLEMBMODE == 17) && (Plz.BLEDSPMODE == 7 || Plz.BLEDSPMODE == 17));
        bool hasBurningTime = (Plz.MBTYPE == 0);
        bool isMicronova    = (Plz.MBTYPE == 0 || Plz.MBTYPE == 1 || Plz.MBTYPE == 5 || Plz.MBTYPE == 6 || Plz.MBTYPE == 7);
        bool isATech        = (Plz.MBTYPE == 10 || Plz.MBTYPE == 11 || Plz.MBTYPE == 12 || Plz.MBTYPE == 13 || Plz.MBTYPE == 14
                            || Plz.MBTYPE == 15 || Plz.MBTYPE == 100);
        bool isChronoBox    = (Plz.CHRSTATUS == 1); // 0=App ; 1=Pala
        bool isFan3ASwitch  = (Plz.FANLMINMAX[2] == 0 && Plz.FANLMINMAX[3] == 1);
        bool isFan4ASwitch  = (Plz.FANLMINMAX[4] == 0 && Plz.FANLMINMAX[5] == 1);
        bool hasFanAuto     = (Plz.FAN2MODE == 2 || Plz.FAN2MODE == 3); // F2L==> 0=OFF ; 7=AUTO
        bool hasFanHigh     = (Plz.FAN2MODE == 3); // 6=HIGH
        bool hasFanProp     = (Plz.FAN2MODE == 4); // 8=PROPORTIONAL
        bool isIdroType     = (Plz.STOVETYPE == 2 || Plz.STOVETYPE == 4 || Plz.STOVETYPE == 6);
        bool hasError       = (Plz.LSTATUS >= 1000);
        bool isStopped      = (Plz.STATUS == 0 || Plz.LSTATUS == 1);
        bool isStarted      = (Plz.LSTATUS == 0 || Plz.LSTATUS == 1 || Plz.LSTATUS == 6 || Plz.LSTATUS == 7 || Plz.LSTATUS == 9 
                            || Plz.LSTATUS == 11 || Plz.LSTATUS == 12 || Plz.LSTATUS == 51 || Plz.LSTATUS == 501 || Plz.LSTATUS == 504
                            || Plz.LSTATUS == 505 || Plz.LSTATUS == 506 || Plz.LSTATUS == 507);

        /*if (hasSwitch) {
            if (isStopped) { 
                WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "1bcc4d", "CMD+ON", D_ON);
            } else {
                if (isStarted) { 
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "ff3333", "CMD+OFF", D_OFF);
                } else {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "ffc233", "CMD+OFF", D_OFF);
                }
            }
        }*/

        WSContentSend_P("{t}");
        WSContentSend_PD(HTTP_PALAZZETTI_TITLE, "Statut", getPlzStatusMessage(Plz.STATUS));

        if (Plz.F2L == 0) {
            WSContentSend_P(HTTP_PALAZZETTI_TITLE_S, "Silence", "");
            WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "ff3333", "SET+SLNT+0", D_PLZ_SLNT_OFF);
            WSContentSend_P("{e}");
        } else {
            WSContentSend_P(HTTP_PALAZZETTI_TITLE_S, "Silence", "");
            WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "1bcc4d", "SET+SLNT+1", D_PLZ_SLNT_ON);
            WSContentSend_P("{e}");
        }
        WSContentSend_P("</table>");

        if (hasSetPoint) {
            WSContentSend_P("{t}");
            WSContentSend_PD(HTTP_PALAZZETTI_THERMOSTAT, D_PLZ_SET_POINT, String(Plz.SETP, 2).c_str(), D_UNIT_CELSIUS[0]);
            WSContentSend_P("{e}");
            WSContentSend_P("</table>");
            WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN2, "SET+STPD", Plz.SPLMIN, Plz.SPLMAX, String(Plz.SETP, 2).c_str(), "SET+SETP", "SET+STPU");
            WSContentSend_P("{t}");
            WSContentSend_PD(HTTP_PALAZZETTI_TITLE_D, D_PLZ_POWER, Plz.PWR);
            WSContentSend_P("{e}");
            WSContentSend_P("</table>");
            WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN2, "SET+PWRD", 1, 5, String(Plz.PWR).c_str(), "SET+POWR", "SET+PWRU");
        }

        if (hasFirstFan) {
            WSContentSend_P("{t}");
            WSContentSend_PD(HTTP_PALAZZETTI_TITLE_S, D_PLZ_MAIN_FAN, getPlzFanStatus(Plz.F2L));
            WSContentSend_P("{e}");
            WSContentSend_P("</table>");
            int maxValue = (hasFanProp ? 8 : (hasFanAuto ? 7 : Plz.FANLMINMAX[1]));
            WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN2, "SET+FN2D", Plz.FANLMINMAX[0], maxValue, String(Plz.F2L).c_str(), "SET+RFAN", "SET+FN2U");
        }

        if (hasSecondFan) {
            WSContentSend_P("{t}");
            WSContentSend_PD(HTTP_PALAZZETTI_TITLE_D, D_PLZ_FAN4, Plz.F4L);
            WSContentSend_P("{e}");
            WSContentSend_P("</table>");
            char cmndOff4[10];
            char cmndOn4[10];
            snprintf_P(cmndOff4, sizeof(cmndOff4), PSTR("SET+FN4L+%d"), Plz.FANLMINMAX[4]);
            snprintf_P(cmndOn4, sizeof(cmndOn4), PSTR("SET+FN4L+%d"), Plz.FANLMINMAX[5]);
            if (isFan4ASwitch) {
                if (Plz.F4L == 0) {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "1bcc4d", cmndOff4, D_ON);
                } else {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "ff3333", cmndOn4, D_OFF);
                }
            } else {
                WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN, cmndOff4, Plz.FANLMINMAX[4], Plz.FANLMINMAX[5], Plz.F4L, "SET+FN4L", cmndOn4);
            }
        }

        if (hasThirdFan) {
            WSContentSend_P("{t}");
            WSContentSend_PD(HTTP_PALAZZETTI_TITLE_D, D_PLZ_FAN3, Plz.F3L);
            WSContentSend_P("{e}");
            WSContentSend_P("</table>");
            char cmndOff3[10];
            char cmndOn3[10];
            snprintf_P(cmndOff3, sizeof(cmndOff3), PSTR("SET+FN3L+%d"), Plz.FANLMINMAX[2]);
            snprintf_P(cmndOn3, sizeof(cmndOn3), PSTR("SET+FN3L+%d"), Plz.FANLMINMAX[3]);
            if (isFan3ASwitch) {
                if (Plz.F3L == 0) {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "1bcc4d", cmndOff3, D_ON);
                } else {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "ff3333", cmndOn3, D_OFF);
                }
            } else {
                WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN, cmndOff3, Plz.FANLMINMAX[2], Plz.FANLMINMAX[3], Plz.F3L, "SET+FN3L", cmndOn3);
            }
        }
        WSContentSend_P("</table>{t}");

        WSContentSend_PD(HTTP_PALAZZETTI_TEMPERATURE, D_PLZ_T1, String(Plz.T1).c_str(), D_UNIT_CELSIUS[0]);
        WSContentSend_PD(HTTP_PALAZZETTI_TEMPERATURE, D_PLZ_T2, String(Plz.T2).c_str(), D_UNIT_CELSIUS[0]);
        WSContentSend_PD(HTTP_PALAZZETTI_TEMPERATURE, D_PLZ_T3, String(Plz.T3).c_str(), D_UNIT_CELSIUS[0]);

        WSContentSend_PD(HTTP_PALAZZETTI_SIMPLE, D_PLZ_F1RPM, Plz.F1RPM, "trs/min");
        WSContentSend_PD(HTTP_PALAZZETTI_POURCENT, D_PLZ_F1V, Plz.F1V);
        WSContentSend_PD(HTTP_PALAZZETTI_POURCENT, D_PLZ_F2V, Plz.F2V);

        WSContentSend_PD(HTTP_PALAZZETTI_TIME, D_PLZ_POWERTIME, Plz.POWERTIMEh, Plz.POWERTIMEm);
        WSContentSend_PD(HTTP_PALAZZETTI_TIME, D_PLZ_HEATTIME, Plz.HEATTIMEh, Plz.HEATTIMEm);
        WSContentSend_PD(HTTP_PALAZZETTI_SIMPLE, D_PLZ_SERVICETIME, Plz.SERVICETIMEh, D_UNIT_HOUR);
        WSContentSend_PD(HTTP_PALAZZETTI_SIMPLE, D_PLZ_PQT, Plz.PQT, D_UNIT_KILOGRAM);

        if (Plz.OVERTMPERRORS > 0) {
            WSContentSend_P(PSTR("<div style='color:red;'>Erreur de surchauffe détectée</div>"));
        }
        if (Plz.IGNERRORS > 0) {
            WSContentSend_P(PSTR("<div style='color:red;'>Allumages ratés</div>"));
        }
        if (Plz.LSTATUS >= 1000) {
            WSContentSend_P(PSTR("<div style='color:red;'>Erreur poêle</div>"));
        }
        
        if (Plz.IGN == 1) {
            //WSContentSend_P(PSTR("<div>Brûleur en marche</div>"));
        } else {
            //WSContentSend_P(PSTR("<div>Brûleur éteint</div>"));
        }

        WSContentSend_P(PSTR("</table>"));

    }
#endif  // USE_WEBSERVER
}

void PlzSendmsgGetRequestHandler(void) {
    if (!HttpCheckPriviledgedAccess()) {
        return;
    }
    PlzRequest.is_get = true;
    char tempCmd[30]; //dev in progress, move to 30 once finished

    WebGetArg(PSTR("cmd"), tempCmd, sizeof(tempCmd));
    if (strlen(tempCmd) > 0) {
        PlzRequest.command_data = (char*)malloc(strlen(tempCmd) + 1);
        if (PlzRequest.command_data != nullptr) {
            strcpy(PlzRequest.command_data, tempCmd);
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: SENDMSG received with cmd: %s"), PlzRequest.command_data);
            if (PlzParser()) {
                AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Command PlzParser successful"));
            } else {
                AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Command failed"));
            }
            free(PlzRequest.command_data);
            PlzRequest.command_data = nullptr;
        }
    }

    WebGetArg(PSTR("command"), tempCmd, sizeof(tempCmd));
    if (strlen(tempCmd) > 0) {
        PlzRequest.command_data = (char*)malloc(strlen(tempCmd) + 1);
        if (PlzRequest.command_data != nullptr) {
            strcpy(PlzRequest.command_data, tempCmd);
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: SENDMSG received with command: %s"), PlzRequest.command_data);
            if (PlzParser()) {
                AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Command PlzParser successful"));
            } else {
                AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Command failed"));
            }
            free(PlzRequest.command_data);
            PlzRequest.command_data = nullptr;
        }
    }
}

void PlzSendmsgPostRequestHandler(void) {
    if (!HttpCheckPriviledgedAccess()) {
        return;
    }
    PlzRequest.is_get = true;
    PlzRequest.is_post = true;
    char event[500];
    strlcpy(event, Webserver->arg(0).c_str(), sizeof(event));
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: SENDMSG POST received with event: %s"), event);
  //PLZ: SENDMSG POST received with event: {"mac":"C8:C9:A3:37:96:E6","command":"GET ALLS","source_properties":{"nuboj_compatibility_mode":false}}
    JsonParser parser(event);
    JsonParserObject root = parser.getRootObject();
    if (!root.isValid()) {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Failed to parse JSON"));
        return;
    }
    JsonParserToken mac = root[PSTR("mac")];
    if (mac.isNull() || strcasecmp(mac.getStr(), WiFi.macAddress().c_str()) != 0) {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Wrong MAC address %s"), mac.getStr());
        return;
    }
    JsonParserToken command = root[PSTR("command")];
    if (command.isNull()) {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: No command found in JSON"));
        return;
    }
    size_t cmdLength = strlen(command.getStr());
    PlzRequest.command_data = (char*)malloc(cmdLength + 1);
    if (PlzRequest.command_data == nullptr) {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Memory allocation for command_data failed"));
        return;
    }

    strlcpy(PlzRequest.command_data, command.getStr(), cmdLength + 1);
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: SENDMSG POST received with cmd: %s"), PlzRequest.command_data);

    if (PlzParser()) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Command PlzParser successful"));
    } else {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Command failed"));
    }

    free(PlzRequest.command_data);
    PlzRequest.command_data = nullptr;
}

void PlzSyscmdRequestHandler(void) {
    if (!HttpCheckPriviledgedAccess()) {
        return;
    }
    PlzRequest.is_get = true;
    char cmd[29];
    WebGetArg(PSTR("cmd"), cmd, sizeof(cmd));

    if (strlen(cmd) > 0) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: SENDMSG received with command: %s"), cmd);
        if (PlzSyscmdCmd(cmd)) {
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzSyscmdCmd successful"));
        } else {
            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Commande échouée:"));
        }
    }
}

void PlzSyncClockWithStove(void) {
    static char last_stove_datetime[32] = { 0 };
    char formatted_stove_datetime[32] = { 0 };

    if (strlen(Plz.STOVE_DATETIME) >= sizeof(formatted_stove_datetime) - 1) {
        return;
    }

    snprintf(formatted_stove_datetime, sizeof(formatted_stove_datetime), "%s", Plz.STOVE_DATETIME);
    char *space_pos = strchr(formatted_stove_datetime, ' ');
    if (space_pos) {
        *space_pos = 'T';
    }

    if (strcmp(formatted_stove_datetime, last_stove_datetime) != 0) {
        strncpy(last_stove_datetime, formatted_stove_datetime, sizeof(last_stove_datetime));

        String tasmota_time = GetDateAndTime(DT_LOCAL);

        int stove_year, stove_month, stove_day, stove_hour, stove_minute, stove_second;

        if (sscanf(formatted_stove_datetime, "%d-%d-%dT%d:%d:%d",
                   &stove_year, &stove_month, &stove_day,
                   &stove_hour, &stove_minute, &stove_second) == 6) {
            int tasmota_year, tasmota_month, tasmota_day, tasmota_hour, tasmota_minute, tasmota_second;

            sscanf(tasmota_time.c_str(), "%d-%d-%dT%d:%d:%d",
                   &tasmota_year, &tasmota_month, &tasmota_day,
                   &tasmota_hour, &tasmota_minute, &tasmota_second);

            int stove_total_minutes = stove_hour * 60 + stove_minute;
            int tasmota_total_minutes = tasmota_hour * 60 + tasmota_minute;
            int diff_minutes = abs(stove_total_minutes - tasmota_total_minutes);

            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Diff_minutes: %d"), diff_minutes);

            if (diff_minutes >= 2) {
                char cmdBuffer[64];
                snprintf(cmdBuffer, sizeof(cmdBuffer), "Palazzetti sendmsg SET TIME %s", tasmota_time.c_str());
                AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Sending SET+TIME+%s"), tasmota_time.c_str());

                ExecuteCommand(cmdBuffer, SRC_WEBCONSOLE);
            }
        }
    }
}

void PlzUpdate(void)
{
    if (++PlzRequest.interval_counter < PlzSettings.interval) return;
    PlzRequest.interval_counter = 0;
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: update"));

    const char* cmdList[] = {
        "GET STDT", "GET TIME", "GET SETP", "GET POWR",
        "GET DPRS", "GET STAT", "GET TMPS", "GET FAND",
        "GET CNTR"
    };

    for (const char* cmd : cmdList) {
        PlzRequest.command_data = (char*)malloc(strlen(cmd) + 1);
        if (PlzRequest.command_data == nullptr) {
            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Memory allocation for command_data failed"));
            break;
        }

        strcpy(PlzRequest.command_data, cmd);

        if (!PlzParser()) {
            AddLog(LOG_LEVEL_ERROR, PSTR("Command failed: %s"), PlzRequest.command_data);
            free(PlzRequest.command_data);
            PlzRequest.command_data = nullptr;
            break;
        }

        free(PlzRequest.command_data);
        PlzRequest.command_data = nullptr;
    }
}

void PlzGetAllHiddenParameters(const char *cmnd) {
    char fileType[5];
    strncpy(fileType, cmnd + 5, 4);
    fileType[4] = '\0'; 
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande getAllHiddenParameters=%s"), fileType);
    if (!isValidFileType(fileType)) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Unknown filetype: %s"), fileType);
        PlzIJson.cmdRes = Palazzetti::CommandResult::UNSUPPORTED;
        WSSend(200, CT_APP_JSON, PlzIJson.full);
        return;
    }
    PlzIJson.cmdRes = pala.getAllHiddenParameters(&Plz.hiddenParams);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char cmdParm[5];
        strncpy(cmdParm, cmnd, 4);
        cmdParm[4] = '\0';
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande CommandResult=%s"), fileType);
        PlzSendParametersResponse(fileType, Plz.hiddenParams, 0x6F, cmdParm);
    }
}

void PlzGetAllParameters(const char *cmnd) {
    char fileType[5];
    strncpy(fileType, cmnd + 5, 4);
    fileType[4] = '\0'; 
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande getAllParameters=%s"), fileType);
    if (!isValidFileType(fileType)) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Unknown filetype: %s"), fileType);
        PlzIJson.cmdRes = Palazzetti::CommandResult::UNSUPPORTED;
        WSSend(200, CT_APP_JSON, PlzIJson.full);
        return;
    }
    PlzIJson.cmdRes = pala.getAllParameters(&Plz.params);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char cmdParm[5];
        strncpy(cmdParm, cmnd, 4);
        cmdParm[4] = '\0';
        PlzSendParametersResponse(fileType, Plz.params, 0x6A, cmdParm);
    }
}

void PlzGetAllStatus() {
    unsigned long currentMillis = millis();
    bool refreshStatus = ((currentMillis - last_all_status_refresh) > 15000UL);
    PlzIJson.cmdRes = pala.getAllStatus(refreshStatus, &Plz.MBTYPE, &Plz.MOD, &Plz.VER, &Plz.CORE, &Plz.FWDATE, &Plz.APLTS, &Plz.APLWDAY, &Plz.CHRSTATUS, &Plz.STATUS, &Plz.LSTATUS, &Plz.isMFSTATUSValid, &Plz.MFSTATUS, &Plz.SETP, &Plz.PUMP, &Plz.PQT, &Plz.F1V, &Plz.F1RPM, &Plz.F2L, &Plz.F2LF, &Plz.FANLMINMAX, &Plz.F2V, &Plz.isF3LF4LValid, &Plz.F3L, &Plz.F4L, &Plz.PWR, &Plz.FDR, &Plz.DPT, &Plz.DP, &Plz.IN, &Plz.OUT, &Plz.T1, &Plz.T2, &Plz.T3, &Plz.T4, &Plz.T5, &Plz.isSNValid, &Plz.SN);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        if (refreshStatus) {
            last_all_status_refresh = currentMillis;
        }
        PlzJSONAddInt("MBTYPE", Plz.MBTYPE);
        PlzJSONAddStr("MAC", WiFi.macAddress().c_str());
        PlzJSONAddInt("MOD", Plz.MOD);
        PlzJSONAddInt("VER", Plz.VER);
        PlzJSONAddInt("CORE", Plz.CORE);
        PlzJSONAddStr("FWDATE", Plz.FWDATE);
        PlzJSONAddStr("APLTS", Plz.APLTS);
        PlzJSONAddInt("APLWDAY", Plz.APLWDAY);
        PlzJSONAddInt("CHRSTATUS", Plz.CHRSTATUS);
        PlzJSONAddInt("STATUS", Plz.STATUS);
        PlzJSONAddInt("LSTATUS", Plz.LSTATUS);
        if (Plz.isMFSTATUSValid) {
            PlzJSONAddInt("MFSTATUS", Plz.MFSTATUS);
        }
        PlzJSONAddFloat("SETP", Plz.SETP);
        PlzJSONAddInt("PUMP", Plz.PUMP);
        PlzJSONAddInt("PQT", Plz.PQT);
        PlzJSONAddInt("F1V", Plz.F1V);
        PlzJSONAddInt("F1RPM", Plz.F1RPM);
        PlzJSONAddInt("F2L", Plz.F2L);
        PlzJSONAddInt("F2LF", Plz.F2LF);
        PlzJSONAddArray("FANLMINMAX", Plz.FANLMINMAX, 6);
        PlzJSONAddComma();
        PlzJSONAddInt("F2V", Plz.F2V);
        if (Plz.isF3LF4LValid) {
            PlzJSONAddInt("F3L", Plz.F3L);
            PlzJSONAddInt("F4L", Plz.F4L);
        }
        PlzJSONAddInt("PWR", Plz.PWR);
        PlzJSONAddFloat("FDR", Plz.FDR);
        PlzJSONAddInt("DPT", Plz.DPT);
        PlzJSONAddInt("DP", Plz.DP);
        PlzJSONAddInt("IN", Plz.IN);
        PlzJSONAddInt("OUT", Plz.OUT);
        PlzJSONAddFloat("T1", Plz.T1);
        PlzJSONAddFloat("T2", Plz.T2);
        PlzJSONAddFloat("T3", Plz.T3);
        PlzJSONAddFloat("T4", Plz.T4);
        PlzJSONAddFloat("T5", Plz.T5);
        PlzJSONAddInt("EFLAGS", 0);
        if (Plz.isSNValid) {
            //JSONAddStr("SN", Plz.SN);
            PlzJSONAddStr("SN", "LT201629480580256025776");
        }
    }
}

void PlzGetAllTemps() {
    PlzIJson.cmdRes = pala.getAllTemps(&Plz.T1, &Plz.T2, &Plz.T3, &Plz.T4, &Plz.T5);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddFloat("T1", Plz.T1);
        PlzJSONAddFloat("T2", Plz.T2);
        PlzJSONAddFloat("T3", Plz.T3);
        PlzJSONAddFloat("T4", Plz.T4);
        PlzJSONAddFloat("T5", Plz.T5);
    }
}

void PlzGetChronoData() {
    PlzIJson.cmdRes = pala.getChronoData(&Plz.CHRSTATUS, &Plz.PCHRSETP, &Plz.PSTART, &Plz.PSTOP, &Plz.DM);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("CHRSTATUS", Plz.CHRSTATUS);
        PlzJSONAddObj("Programs");
        for (byte i = 0; i < 6; i++) {
            PlzJSONAddObj(String("P" + String(i + 1)).c_str());
            PlzJSONAddFloat("CHRSETP", Plz.PCHRSETP[i]);
            char time[6] = {'0', '0', ':', '0', '0', 0};
            time[0] = Plz.PSTART[i][0] / 10 + '0';
            time[1] = Plz.PSTART[i][0] % 10 + '0';
            time[3] = Plz.PSTART[i][1] / 10 + '0';
            time[4] = Plz.PSTART[i][1] % 10 + '0';
            PlzJSONAddStr("START", time);
            time[0] = Plz.PSTOP[i][0] / 10 + '0';
            time[1] = Plz.PSTOP[i][0] % 10 + '0';
            time[3] = Plz.PSTOP[i][1] / 10 + '0';
            time[4] = Plz.PSTOP[i][1] % 10 + '0';
            PlzJSONAddStr("STOP", time);
            PlzJSONCloseObj();
            PlzJSONAddComma();
        }
        PlzJSONCloseObj();
        PlzJSONAddComma();
        PlzJSONAddObj("Days");
        for (byte dayNumber = 0; dayNumber < 7; dayNumber++) {
            PlzJSONAddObj(String("D" + String(dayNumber + 1)).c_str());
            for (byte memoryNumber = 0; memoryNumber < 3; memoryNumber++) {
                char memoryName[3] = {'M', (char)(memoryNumber + '1'), 0};
                PlzJSONAddStr(memoryName, Plz.DM[dayNumber][memoryNumber] ? String("P" + String(Plz.DM[dayNumber][memoryNumber])).c_str() : "OFF");
            }
            PlzJSONCloseObj();
            PlzJSONAddComma();
        }
        PlzJSONCloseObj();
    }
}

void PlzGetJTimer() {
    // Obtenez les données de chrono depuis pala.getChronoData
    PlzIJson.cmdRes = pala.getChronoData(&Plz.CHRSTATUS, &Plz.PCHRSETP, &Plz.PSTART, &Plz.PSTOP, &Plz.DM);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char jsonBuffer[MAX_JTIMER] = { 0 }; // Buffer pour stocker le JSON
        size_t offset = 0;

        // Commencer à construire le JSON
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "{");
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"last_program\": \"%s\",", PlzJTimer.program_id);
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"curr_timezone\": %s,", "true");
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"curr_timezone_country\": \"%s\",", String(Settings->timezone).c_str());
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"enabled\": %s,", Settings->flag3.timers_enable ? "true" : "false");
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"off_when_nomatch\": %s,", PlzJTimer.off_when_no_match ? "true" : "false");
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"sync_clock_enabled\": %s,", PlzSettings.sync_clock ? "true" : "false");
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"last_edit\": \"%s\",", PlzJTimer.last_edit);

        // Scenarios
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"scenarios\": {");
        for (int i = 0; i < 4; ++i) {
            if (i > 0) {
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, ",");
            }
            offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"%s\": {\"settings\": {", scenarioNames[i]);
            if (scenarioSettings[i].setPoint != -1) {
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"SET SETP\": %d", scenarioSettings[i].setPoint);
            } else if (scenarioSettings[i].setPower != -1) {
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"SET POWR\": %d", scenarioSettings[i].setPower);
            } else if (scenarioSettings[i].setFan != -1) {
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"SET RFAN\": %d", scenarioSettings[i].setFan);
            } else if (scenarioSettings[i].setFan3 != -1) {
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"SET FN3L\": %d", scenarioSettings[i].setFan3);
            } else if (scenarioSettings[i].setFan4 != -1) {
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"SET FN4L\": %d", scenarioSettings[i].setFan4);
            } else if (scenarioSettings[i].setSilent != -1) {
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"SET SLNT\": %d", scenarioSettings[i].setSilent);
            }
            offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "}}");
        }
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "},");

        // Scheduler
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"scheduler\": {");
        bool firstEntry = true;

        for (int day = 0; day <= 6; day++) {
            Timer start_timer = Settings->timer[day];
            Timer end_timer = Settings->timer[day + 7];
            if (start_timer.arm && end_timer.arm) {
                if (start_timer.time > end_timer.time) continue;
                if (!firstEntry) {
                    offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, ",");
                }
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"%d\": [{", day - 1);
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"id\": 1,");
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"start_time\": %d,", start_timer.time);
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"end_time\": %d,", end_timer.time);
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"scenario\": \"economy\","); // search scenario in rule data
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "\"settings\": {}");
                offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "}]");
                firstEntry = false;
            }
        }
        offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "}}"); // Close scheduler
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Final Encoded Timer jsonBuffer: %s"), jsonBuffer);

        size_t output_len;
        char *encodedData = PlzEncodeGzip(jsonBuffer, strlen(jsonBuffer), &output_len);
        if (encodedData) {
            char finalJson[1024];
            snprintf_P(finalJson, sizeof(finalJson), JSON_STR_TEMPLATE, "TIMER", encodedData);
            strcat_P(PlzIJson.data, finalJson);
            free(encodedData);
        } else {
            snprintf_P(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Failed to encode datas"));
            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Failed to encode JSON"));
        }
    }
}


void PlzSetJTimer(const char *encoded_data) {
    //const char *encoded_data = "H4sIAAAAAAAAA8WSvQqDMBSFZ/MUJXMG/yjFvXuh3UqRVK9V0ARiRIrk3XuToKW0g1MdAuE799xDDplIQEHwewslzXYVb3tgiGRV5WMNIhey47qoUdNqcFJf4LxqZI9sIoGbpZm9oQZaN+KB0mQQGGZ1KCRueX7PODuS8/Gyw3NCEh0sensL2VVS6VXeOP70jlx164z7xUicF99YQzm0oLyfhjS7+kWNrSlifqnmSue66QAZTcKQeg6iXGiULnguzuK5E/artsDc3AOiTVLjTVKTTVLTf6fi/yLmBTyZldd0AwAA";
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Compressed JSON preview: %s"), encoded_data);
    size_t encoded_len = strlen(encoded_data);
    size_t decoded_len;
    char *decoded_data = PlzDecodeGunzip(encoded_data, encoded_len, &decoded_len);

    if (!decoded_data) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Decoding failed!"));
        return;
    }
    decoded_data[decoded_len] = '\0';
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Decompressed JSON preview: %s"), decoded_data);

    JsonParser parser(decoded_data);
    JsonParserObject root = parser.getRootObject();
    if (!root) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: JSON not valid: %s"), decoded_data);
        return;
    }

    Settings->flag3.timers_enable = root.getBool("enabled", false);
    if (Settings->flag3.timers_enable) {
        if (Plz.CHRSTATUS) {
            PlzSetChronoStatus("0"); // CHRD timer off
        }
    } else {
        //return; //do I have to return? otherwise timers are emptied
        // if I return, error message is sent to the App
    }

    PlzJTimer.sync_clock_enabled = root.getBool("sync_clock_enabled", false);
    if (PlzJTimer.sync_clock_enabled != PlzSettings.sync_clock) {
        PlzSettings.sync_clock = PlzJTimer.sync_clock_enabled;
        PlzSettingsSave();  // Sauvegarder les settings mis à jour
        PlzSettingsPublish();
    }

    PlzJTimer.ecostart_mode_enabled = root.getBool("ecostart_mode_enabled", false); //flag_modalita_ecostart
    PlzJTimer.off_when_no_match = root.getBool("off_when_nomatch", true);
    snprintf(PlzJTimer.last_edit, sizeof(PlzJTimer.last_edit), "%s", root.getStr("last_edit"));
    const char* appl_id = root.getStr("applid");
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: ecostart_mode_enabled: %d, last_edit: %s, off_when_no_match: %d, appl_id: %d"),
        PlzJTimer.ecostart_mode_enabled, PlzJTimer.last_edit, PlzJTimer.off_when_no_match, appl_id);

    JsonParserToken scenariosToken = root["scenarios"];
    if (scenariosToken.isObject()) {
        JsonParserObject scenariosObj = scenariosToken.getObject();
        for (int i = 1; i < 4; ++i) {
            JsonParserToken scenarioToken = scenariosObj[scenarioNames[i]];
            if (scenarioToken.isObject()) {
                JsonParserObject scenarioObj = scenarioToken.getObject();
                JsonParserToken settingsToken = scenarioObj["settings"];
                if (settingsToken.isObject()) {
                    JsonParserObject settingsObj = settingsToken.getObject();
                    scenarioSettings[i].setPoint = settingsObj.getInt("SET SETP", -1);
                    scenarioSettings[i].setPower = settingsObj.getInt("SET POWR", -1);
                    scenarioSettings[i].setFan = settingsObj.getInt("SET RFAN", -1);
                    scenarioSettings[i].setFan3 = settingsObj.getInt("SET FN3L", -1);
                    scenarioSettings[i].setFan4 = settingsObj.getInt("SET FN4L", -1);
                    scenarioSettings[i].setSilent = settingsObj.getInt("SET SLNT", -1);

                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Scenario: %s, Setpoint: %d, setPower: %d, setFan: %d, setFan3: %d, setFan4: %d, setSilent: %d"),
                        scenarioNames[i], scenarioSettings[i].setPoint, scenarioSettings[i].setPower, scenarioSettings[i].setFan,
                        scenarioSettings[i].setFan3, scenarioSettings[i].setFan4, scenarioSettings[i].setSilent);
                }
            }
        }
    }

    // Configurer les timers et règles
    JsonParserToken schedulerToken = root["scheduler"];
    if (schedulerToken.isObject()) {
        JsonParserObject schedulerObj = schedulerToken.getObject();
        char ruleBuffer[MAX_RULE_SIZE] = { 0 };
        Settings->flag3.timers_enable = 1;
        for (int day = 0; day <= 6; ++day) {
            int timer_day = day + 1;
            char dayKey[2];
            snprintf(dayKey, sizeof(dayKey), "%d", day);
            if (schedulerObj[dayKey].isArray()) {
                        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: day exists  %d"), day);
                JsonParserArray dayArray = schedulerObj[dayKey].getArray();
                for (int i = 0; i < dayArray.size(); ++i) {
                    JsonParserObject entry = dayArray[i].getObject();
                    const char* scenario = entry.getStr("scenario");
                    int startTime = entry.getInt("start_time", 0);
                    int endTime = entry.getInt("end_time", 0);

                    char ruleStartCmd[43];
                    snprintf(ruleStartCmd, sizeof(ruleStartCmd), "ON Rules#Timer%d DO Backlog Mem1 %s ; ", timer_day, scenario);
                    strcat(ruleBuffer, ruleStartCmd);
                        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: ruleBuffer1 %s"), ruleBuffer);

                    int scenarioIdx = PlzGetScenarioIndex(scenario);
                    if (scenarioIdx < 0) {
                        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Invalid scenario name: %s"), scenario);
                        continue; // Ignorer cet élément si le scénario est introuvable
                    }
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: PlzGetScenarioIndex scenarioIdx: %d"), scenarioIdx);

                    int values[] = {
                        scenarioSettings[scenarioIdx].setPoint, scenarioSettings[scenarioIdx].setPower, 
                        scenarioSettings[scenarioIdx].setFan, scenarioSettings[scenarioIdx].setFan3, 
                        scenarioSettings[scenarioIdx].setFan4, scenarioSettings[scenarioIdx].setSilent
                    };

                    int last_prog_value = -1;
                    if (scenarioIdx == 0) {
                        strcat(ruleBuffer, " Palazzetti sendmsg CMD OFF; ");
                    } else if (scenarioIdx > 0 && scenarioIdx < 4) {
                        char commandBuffer[18];
                        char tempRule[32];
                        for (int j = 0; j < 6; j++) {
                            if (values[j] != -1) {
                                GetTextIndexed(commandBuffer, sizeof(commandBuffer), j, kScenarioSettings);
                                snprintf(tempRule, sizeof(tempRule), "Palazzetti sendmsg %s %d ; ", commandBuffer, values[j]);
                                last_prog_value = values[j];
                                strcat(ruleBuffer, tempRule);
                            }
                        }
                        //if lastCmd already sent ?
                        strcat(ruleBuffer, " Palazzetti sendmsg CMD ON;");
                        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Day: %d, Scenario: %s, Start: %d, End: %d"), day, scenario, startTime, endTime);

                        snprintf(PlzJTimer.program_id, sizeof(PlzJTimer.program_id), "%d_%d_%s_%d", day, startTime, scenario, last_prog_value);
                        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Applied Program ID: %s"), PlzJTimer.program_id);
                    }
                    strcat(ruleBuffer, " ENDON");

                    char ruleEndCmd[80];
                    snprintf(ruleEndCmd, sizeof(ruleEndCmd), "; ON Rules#Timer%d DO Backlog Mem1 OFF; Palazzetti sendmsg CMD OFF; ENDON ", day + 8, day + 8);
                    strcat(ruleBuffer, ruleEndCmd);

                    // Configurer directement les timers
                    Settings->timer[day].arm = 1;//PlzJTimer.enabled ? 1 : 0;
                    Settings->timer[day].mode = 0;
                    Settings->timer[day].window = 2; //timer.lib.lua#192
                    Settings->timer[day].time = startTime;
                    Settings->timer[day].days = 1 << timer_day;
                    Settings->timer[day].repeat = 1;
                    Settings->timer[day].device = 1;
                    Settings->timer[day].power = 3;
                    Response_P(PSTR("{"));
                    PrepShowTimer(day);
                    ResponseJsonEnd();

                    Settings->timer[day + 7].arm = 1;//PlzJTimer.enabled ? 1 : 0;
                    Settings->timer[day + 7].mode = 0;
                    Settings->timer[day + 7].window = 2; //timer.lib.lua#192
                    Settings->timer[day + 7].time = endTime;
                    Settings->timer[day + 7].days = 1 << timer_day;
                    Settings->timer[day + 7].repeat = 1;
                    Settings->timer[day + 7].device = 1;
                    Settings->timer[day + 7].power = 3;
                    Response_P(PSTR("{"));
                    PrepShowTimer(day);
                    ResponseJsonEnd();

                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Day: %d, Scenario: %s, Start: %d, End: %d"), day, scenario, startTime, endTime);
                }
            } else {
                // ici il faut supprimer les timers autres que ceux saisis
                Settings->timer[day].data = 0;
                Settings->timer[day + 7].data = 0;
            }
        }
        char cmnd[32];
        snprintf_P(cmnd, sizeof(cmnd), PSTR("Rule3 %s"), ruleBuffer);
        ExecuteCommand(cmnd, SRC_WEBCONSOLE);
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Rule stored: %s"), ruleBuffer);
        snprintf(PlzIJson.cmd, sizeof(PlzIJson.cmd), PSTR("%s"), "settimer");
    }

    PlzIJson.cmdRes = Palazzetti::CommandResult::OK;
    snprintf_P(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("settimer OK"));
}

void PlzGetCounters() {
    PlzIJson.cmdRes = pala.getCounters(&Plz.IGN, &Plz.POWERTIMEh, &Plz.POWERTIMEm, &Plz.HEATTIMEh, &Plz.HEATTIMEm, &Plz.SERVICETIMEh, &Plz.SERVICETIMEm, &Plz.ONTIMEh, &Plz.ONTIMEm, &Plz.OVERTMPERRORS, &Plz.IGNERRORS, &Plz.PQT);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("IGN", Plz.IGN);
        char powerTimeStr[8];
        snprintf_P(powerTimeStr, sizeof(powerTimeStr), TIME_HOUR_MINUTE, Plz.POWERTIMEh, Plz.POWERTIMEm);
        PlzJSONAddStr("POWERTIME", powerTimeStr);
        char heatTimeStr[8];
        snprintf_P(heatTimeStr, sizeof(heatTimeStr), TIME_HOUR_MINUTE, Plz.HEATTIMEh, Plz.HEATTIMEm);
        PlzJSONAddStr("HEATTIME", heatTimeStr);
        char serviceTimeStr[8];
        snprintf_P(serviceTimeStr, sizeof(serviceTimeStr), TIME_HOUR_MINUTE, Plz.SERVICETIMEh, Plz.SERVICETIMEm);
        PlzJSONAddStr("SERVICETIME", serviceTimeStr);
        char ontimeTimeStr[8];
        snprintf_P(ontimeTimeStr, sizeof(ontimeTimeStr), TIME_HOUR_MINUTE, Plz.ONTIMEh, Plz.ONTIMEm);
        PlzJSONAddStr("ONTIME", ontimeTimeStr);
        PlzJSONAddInt("OVERTMPERRORS", Plz.OVERTMPERRORS);
        PlzJSONAddInt("IGNERRORS", Plz.IGNERRORS);
        PlzJSONAddInt("PQT", Plz.PQT);
    }
}

void PlzGetDateTime() {
    PlzIJson.cmdRes = pala.getDateTime(&Plz.STOVE_DATETIME, &Plz.STOVE_WDAY);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddStr("STOVE_DATETIME", Plz.STOVE_DATETIME);
        PlzJSONAddInt("STOVE_WDAY", Plz.STOVE_WDAY);
    }
}

void PlzGetDPressData() {
    PlzIJson.cmdRes = pala.getDPressData(&Plz.DPT, &Plz.DP);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("DPT", Plz.DPT);
        PlzJSONAddInt("DP", Plz.DP);
    }
}

void PlzGetFanData() {
    PlzIJson.cmdRes = pala.getFanData(&Plz.F1V, &Plz.F2V, &Plz.F1RPM, &Plz.F2L, &Plz.F2LF, &Plz.isF3SF4SValid, &Plz.F3S, &Plz.F4S, &Plz.isF3LF4LValid, &Plz.F3L, &Plz.F4L);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("F1V", Plz.F1V);
        PlzJSONAddInt("F2V", Plz.F2V);
        PlzJSONAddInt("F1RPM", Plz.F1RPM);
        PlzJSONAddInt("F2L", Plz.F2L);
        PlzJSONAddInt("F2LF", Plz.F2LF);
        if (Plz.isF3SF4SValid) {
            PlzJSONAddFloat("F3S", Plz.F3S);
            PlzJSONAddFloat("F4S", Plz.F4S);
        }
        if (Plz.isF3LF4LValid) {
            PlzJSONAddInt("F3L", Plz.F3L);
            PlzJSONAddInt("F4L", Plz.F4L);
        }
    }
}

void PlzGetHParamListData(const char *getHiddenParam, const char *prefix) {
    char* endptr3 = nullptr;
    errno = 0;
    int getHiddenParamIndex = strtol(getHiddenParam, &endptr3, 10);
    if (checkStrtolError(getHiddenParam, endptr3)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }
    uint16_t getHiddenParamValue;
    PlzIJson.cmdRes = pala.getHiddenParameter(getHiddenParamIndex, &getHiddenParamValue);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[8];
        snprintf(key, sizeof(key), PSTR("%s%d"), prefix, getHiddenParamIndex);
        PlzJSONAddInt(key, getHiddenParamValue);
    }
}

void PlzGetIO() {
    PlzIJson.cmdRes = pala.getIO(&Plz.IN_I01, &Plz.IN_I02, &Plz.IN_I03, &Plz.IN_I04, &Plz.OUT_O01, &Plz.OUT_O02, &Plz.OUT_O03, &Plz.OUT_O04, &Plz.OUT_O05, &Plz.OUT_O06, &Plz.OUT_O07);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("IN_I01", Plz.IN_I01);
        PlzJSONAddInt("IN_I02", Plz.IN_I02);
        PlzJSONAddInt("IN_I03", Plz.IN_I03);
        PlzJSONAddInt("IN_I04", Plz.IN_I04);
        PlzJSONAddInt("OUT_O01", Plz.OUT_O01);
        PlzJSONAddInt("OUT_O02", Plz.OUT_O02);
        PlzJSONAddInt("OUT_O03", Plz.OUT_O03);
        PlzJSONAddInt("OUT_O04", Plz.OUT_O04);
        PlzJSONAddInt("OUT_O05", Plz.OUT_O05);
        PlzJSONAddInt("OUT_O06", Plz.OUT_O06);
        PlzJSONAddInt("OUT_O07", Plz.OUT_O07);
    }
}

void PlzGetLabel() {
    PlzIJson.cmdRes = Palazzetti::CommandResult::OK;
    PlzJSONAddStr("LABEL", NetworkHostname());
}

void PlzGetLimMaxListData() {
}

void PlzGetLimMinListData() {
}

void PlzGetModelVersion() {
    PlzIJson.cmdRes = pala.getModelVersion(&Plz.MOD, &Plz.VER, &Plz.CORE, &Plz.FWDATE);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("MOD", Plz.MOD);
        PlzJSONAddInt("VER", Plz.VER);
        PlzJSONAddInt("CORE", Plz.CORE);
        PlzJSONAddStr("FWDATE", Plz.FWDATE);
    }
}

void PlzGetParamListData(const char *getParam, const char *prefix) {
    char* endptr0 = nullptr;
    errno = 0;
    int getParamIndex = strtol(getParam, &endptr0, 10);
    if (checkStrtolError(getParam, endptr0)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }

    byte getParamValue;
    PlzIJson.cmdRes = pala.getParameter(getParamIndex, &getParamValue);
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande getParameter param=%d value=%d"), getParamIndex, getParamValue);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[8];
        snprintf(key, sizeof(key), PSTR("%s%d"), prefix, getParamIndex);
        PlzJSONAddInt(key, getParamValue);
    }
}

void PlzGetPower() {
    PlzIJson.cmdRes = pala.getPower(&Plz.PWR, &Plz.FDR);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("PWR", Plz.PWR);
        PlzJSONAddFloat("FDR", Plz.FDR);
    }
}

    // Fonction pour obtenir l'index du scénario
int PlzGetScenarioIndex(const char* scenario) {
    for (int i = 0; i < 4; ++i) {
        if (strcmp(scenarioNames[i], scenario) == 0) {
            return i;
        }
    }
    return -1;
}

void PlzGetSetPoint() {
    PlzIJson.cmdRes = pala.getSetPoint(&Plz.SETP);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddFloat("SETP", Plz.SETP);
    }
}

void PlzGetSN() {
    PlzIJson.cmdRes = pala.getSN(&Plz.SN);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddStr("SN", "LT201629480580256025776");
        //JSONAddStr("SERN", Plz.SN);
    }
}

void PlzGetStaticData() {
    PlzIJson.cmdRes = pala.getStaticData(&Plz.SN, &Plz.SNCHK, &Plz.MBTYPE, &Plz.MOD, &Plz.VER, &Plz.CORE, &Plz.FWDATE, &Plz.FLUID, &Plz.SPLMIN, &Plz.SPLMAX, &Plz.UICONFIG, &Plz.HWTYPE, &Plz.DSPTYPE, &Plz.DSPFWVER, &Plz.CONFIG, &Plz.PELLETTYPE, &Plz.PSENSTYPE, &Plz.PSENSLMAX, &Plz.PSENSLTSH, &Plz.PSENSLMIN, &Plz.MAINTPROBE, &Plz.STOVETYPE, &Plz.FAN2TYPE, &Plz.FAN2MODE, &Plz.BLEMBMODE, &Plz.BLEDSPMODE, &Plz.CHRONOTYPE, &Plz.AUTONOMYTYPE, &Plz.NOMINALPWR);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddStr("LABEL", NetworkHostname());
        PlzJSONAddStr("GWDEVICE", "wlan0");
        PlzJSONAddStr("MAC", WiFi.macAddress().c_str());
        PlzJSONAddStr("GATEWAY", WiFi.gatewayIP().toString().c_str());
        PlzJSONAddStr("DNS", ("[" + WiFi.dnsIP().toString() + "]").c_str());
        PlzJSONAddStr("WMAC", WiFi.macAddress().c_str());
        PlzJSONAddStr("WMODE", (WiFi.getMode() & WIFI_STA) ? "sta" : "ap");
        PlzJSONAddStr("WADR", WiFi.localIP().toString().c_str());
        PlzJSONAddStr("WGW", WiFi.gatewayIP().toString().c_str());
        PlzJSONAddStr("WENC", "psk2");
        PlzJSONAddStr("WPWR", WiFi.RSSI() + " dBm");
        PlzJSONAddStr("WSSID", WiFi.SSID().c_str());
        PlzJSONAddStr("WPR", WiFi.isConnected() ? "dhcp" : "static");
        PlzJSONAddStr("WMSK", WiFi.subnetMask().toString().c_str());
        IPAddress broadcast;
        for (int i = 0; i < 4; i++) {
            broadcast[i] = WiFi.localIP()[i] | ~WiFi.subnetMask()[i];
        }
        PlzJSONAddStr("WBCST", broadcast.toString().c_str());
        PlzJSONAddInt("WCH", WiFi.channel());

#if defined(ESP32) && defined(USE_ETHERNET)
        PlzJSONAddStr("EPR", "dhcp");
        PlzJSONAddStr("EGW", ETH.gatewayIP().toString().c_str());
        PlzJSONAddStr("EMSK", ETH.subnetMask().toString().c_str());
        PlzJSONAddStr("EADR", ETH.localIP().toString().c_str());
        PlzJSONAddStr("EMAC", ETH.macAddress().c_str());
        PlzJSONAddStr("ECBL", ETH.linkUp() ? "up" : "down");
        PlzJSONAddStr("EBCST", ETH.broadcastIP().toString().c_str());
#else
        PlzJSONAddStr("EPR", "dhcp");
        PlzJSONAddStr("EGW", "0.0.0.0");
        PlzJSONAddStr("EMSK", "0.0.0.0");
        PlzJSONAddStr("EADR", "0.0.0.0");
        PlzJSONAddStr("EMAC", WiFi.macAddress().c_str());
        PlzJSONAddStr("ECBL", "down");
        PlzJSONAddStr("EBCST", "");
#endif

        bool internetConnected = (WiFi.status() == WL_CONNECTED);
        PlzJSONAddInt("APLCONN", 1);
        PlzJSONAddInt("ICONN", internetConnected ? 1 : 0);
        PlzJSONAddStr("CBTYPE", "miniembplug");
        PlzJSONAddStr("sendmsg", "2.1.2 2018-03-28 10:19:09");
        PlzJSONAddStr("plzbridge", "2.2.1 2022-10-24 11:13:21");
        PlzJSONAddStr("SYSTEM", "2.5.3 2021-10-08 10:30:20 (657c8cf)");
        PlzJSONAddBool("CLOUD_ENABLED", true);

        PlzJSONAddStr("SN", "LT201629480580256025776");
        //JSONAddStr("SN", Plz.SN);
        PlzJSONAddInt("SNCHK", Plz.SNCHK);
        PlzJSONAddInt("MBTYPE", Plz.MBTYPE);
        PlzJSONAddInt("MOD", Plz.MOD);
        PlzJSONAddInt("VER", Plz.VER);
        PlzJSONAddInt("CORE", Plz.CORE);
        PlzJSONAddStr("FWDATE", Plz.FWDATE);
        PlzJSONAddInt("FLUID", Plz.FLUID);
        PlzJSONAddInt("SPLMIN", Plz.SPLMIN);
        PlzJSONAddInt("SPLMAX", Plz.SPLMAX);
        PlzJSONAddInt("UICONFIG", Plz.UICONFIG);
        PlzJSONAddInt("HWTYPE", Plz.HWTYPE);
        PlzJSONAddInt("DSPTYPE", Plz.DSPTYPE);
        PlzJSONAddInt("DSPFWVER", Plz.DSPFWVER);
        PlzJSONAddInt("CONFIG", Plz.CONFIG);
        PlzJSONAddInt("PELLETTYPE", Plz.PELLETTYPE);
        PlzJSONAddInt("PSENSTYPE", Plz.PSENSTYPE);
        PlzJSONAddInt("PSENSLMAX", Plz.PSENSLMAX);
        PlzJSONAddInt("PSENSLTSH", Plz.PSENSLTSH);
        PlzJSONAddInt("PSENSLMIN", Plz.PSENSLMIN);
        PlzJSONAddInt("MAINTPROBE", Plz.MAINTPROBE);
        PlzJSONAddInt("STOVETYPE", Plz.STOVETYPE);
        PlzJSONAddInt("FAN2TYPE", Plz.FAN2TYPE);
        PlzJSONAddInt("FAN2MODE", Plz.FAN2MODE);
        PlzJSONAddInt("BLEMBMODE", Plz.BLEMBMODE);
        PlzJSONAddInt("BLEDSPMODE", Plz.BLEDSPMODE);
        PlzJSONAddInt("CHRONOTYPE", Plz.CHRONOTYPE);
        PlzJSONAddInt("AUTONOMYTYPE", Plz.AUTONOMYTYPE);
        PlzJSONAddInt("NOMINALPWR", Plz.NOMINALPWR);
    }
}

void PlzGetStatus() {
    PlzIJson.cmdRes = pala.getStatus(&Plz.STATUS, &Plz.LSTATUS, &Plz.FSTATUS);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("STATUS", Plz.STATUS);
        PlzJSONAddInt("LSTATUS", Plz.LSTATUS);
        PlzJSONAddInt("FSTATUS", Plz.FSTATUS);
    }
}

void PlzInitJSON() {
    PlzIJson.cmdRes = Palazzetti::CommandResult::ERROR;
    memset(PlzIJson.data, 0, sizeof(PlzIJson.data));
    memset(PlzIJson.info, 0, sizeof(PlzIJson.info));
    memset(PlzIJson.full, 0, sizeof(PlzIJson.full));
    memset(PlzIJson.msg, 0, sizeof(PlzIJson.msg));
    memset(PlzIJson.cmd, 0, sizeof(PlzIJson.cmd));
    PlzIJson.ts = 0;
    PlzIJson.success = false;
}

bool PlzParser() {
    const char* cmd = PlzRequest.command_data;
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzParser cmd=%s"), cmd);

    char cmdCopy[1024];
    snprintf(cmdCopy, sizeof(cmdCopy), "%s", cmd);
    char* start = cmdCopy;
    while (*start == ' ') {
        start++;
    }
    if (!PlzRequest.is_post) {
        for (char &c : cmdCopy) c = toupper(c);
    }
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzParser filtered cmd=%s"), cmdCopy);

    PlzInitJSON();
    snprintf(PlzIJson.cmd, sizeof(PlzIJson.cmd), PSTR("%s"), cmdCopy);
    PlzIJson.cmdRes = Palazzetti::CommandResult::COMMUNICATION_ERROR;

    if (PlzRequest.is_get || PlzRequest.is_udp) {
        PlzJSONAddObj("DATA");
    }
    if (strncasecmp(cmdCopy, "GET ", 4) == 0) {
        PlzIJson.success = PlzParserGET(cmdCopy + 4);
    } else if (strncasecmp(cmdCopy, "SET ", 4) == 0) {
        PlzIJson.success = PlzParserSET(cmdCopy + 4);
    } else if (strncasecmp(cmdCopy, "CMD ", 4) == 0) {
        PlzIJson.success = PlzParserCMD(cmdCopy + 4);
    } else if (strncasecmp(cmdCopy, "BKP ", 4) == 0) {
        PlzIJson.success = PlzParserBKP(cmdCopy + 4);
    } else if (strncasecmp(cmdCopy, "EXT ", 4) == 0) {
        PlzIJson.success = PlzParserEXT(cmdCopy + 4);
    } else {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf_P(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Unknown command received: %s"), cmdCopy);
    }

    if (PlzIJson.success) {
        PlzRequest.is_connected = true;
    }

    if (PlzRequest.is_get || PlzRequest.is_udp || PlzRequest.is_post) {
        if (!PlzIJson.success) {
            PlzJSONAddBool("NODATA", !PlzIJson.success);
            if (!PlzIJson.msg) {
                snprintf_P(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Stove communication failed"));
            }
        }
        PlzJSONCloseObj();
    }

    if (PlzRequest.is_get || PlzRequest.is_udp) {
        PlzJSONAddComma();
        PlzJSONAddBool("SUCCESS", PlzIJson.success);
        PlzJSONRemoveComma(PlzIJson.data);

        if (strncasecmp(cmdCopy, "BKP ", 4) == 0) {
            char attachment[100];
            snprintf_P(attachment, sizeof(attachment), PSTR("attachment; filename=%s.%s"), cmdCopy + 4, cmdCopy + 9);
            Webserver->sendHeader(F("Content-Disposition"), attachment);
            
            const char* parmStart = strstr(PlzIJson.data, "\"DATA\":{"); // Pour CSV
            if (parmStart != nullptr) {
                parmStart += 8;
                const char* parmEnd = strstr(parmStart, ",\"SUCCESS\"");

                if (parmEnd != nullptr) {
                    int startIndex = parmStart - PlzIJson.data;
                    int endIndex = parmEnd - PlzIJson.data;

                    if (strncmp(cmdCopy + 9, "CSV", 3) == 0) {
                        WSSend(200, CT_PLAIN, String(PlzIJson.data).substring(startIndex, endIndex - 1));
                        snprintf_P(PlzIJson.data, sizeof(PlzIJson.data), PSTR("\"%s\""), RemoveSpace(PlzIJson.data));
                    } else if (strncmp(cmdCopy + 9, "JSON", 4) == 0) {
                        WSSend(200, CT_APP_JSON, String(PlzIJson.data).substring(startIndex - 1, endIndex));
                    }
                } else {
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: BKP end data not found"));
                }
            } else {
                AddLog(LOG_LEVEL_INFO, PSTR("PLZ: BKP start data not found"));
            }
        }

        PlzIJson.ts = getTimeStamp();
        PlzJSONMerge();
        if (strncasecmp(cmdCopy, "BKP ", 4) != 0) {
            WSSend(200, CT_APP_JSON, PlzIJson.full);
        }
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: PlzParser1 Commande response=%s"), PlzIJson.full);
        PlzRequest.is_get = false;
        PlzRequest.is_udp = false;
    }

    if (PlzSettings.mqtt_topic == 1) {
        PlzIJson.ts = getTimeStamp();
        PlzJSONMerge();
    
        ResponseClear();
        Response_P(PSTR("%s"), PlzIJson.full);
        MqttPublishPrefixTopicRulesProcess_P(TELE, cmdCopy);
    } else if (PlzSettings.mqtt_topic == 0 || PlzSettings.mqtt_topic == 2) {
        char *keyStart = strstr(PlzIJson.data, "{");
        while (keyStart) {
            keyStart = strchr(keyStart, '"');
            if (!keyStart) break;

            char* keyEnd = strchr(keyStart + 1, '"');
            if (!keyEnd) break;
            *keyEnd = '\0';

            char *valueStart = strchr(keyEnd + 1, ':') + 1;
            if (!valueStart) break;
            while (*valueStart == ' ') valueStart++;

            char *valueEnd;
            if (*valueStart == '"') {
                valueStart++;
                valueEnd = strchr(valueStart, '"');
            } else if (*valueStart == '[') {
                valueStart++;
                valueEnd = strchr(valueStart, ']'); 
            } else {
                valueEnd = valueStart;
                while (*valueEnd && *valueEnd != ',' && *valueEnd != '}') valueEnd++;
            }
            if (!valueEnd) break;
            *valueEnd = '\0';

            char mqttTopic[128];
            if (PlzSettings.mqtt_topic == 0) {
                snprintf(mqttTopic, sizeof(mqttTopic), "%s", keyStart + 1);
            } else if (PlzSettings.mqtt_topic == 2) {
                snprintf(mqttTopic, sizeof(mqttTopic), "%s/%s", cmdCopy, keyStart + 1);
            }
               
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Publishing MQTT: topic=%s, valueS=%s, localCmd=%s, keyStart+1=%s"), mqttTopic, valueStart, cmdCopy, keyStart + 1);
            Response_P(PSTR("%s"), valueStart);
            MqttPublishPrefixTopicRulesProcess_P(TELE, mqttTopic);

            keyStart = valueEnd + 1;
        }
        Response_P(PSTR("%s"), PlzIJson.cmd);
        if (PlzSettings.mqtt_topic == 0) {
            MqttPublishPrefixTopicRulesProcess_P(TELE, JSON_CMD);
        } else if (PlzSettings.mqtt_topic == 2) {
            MqttPublishPrefixTopicRulesProcess_P(TELE, "INFO/CMD");
        }
        if (PlzIJson.ts > 0) {
            Response_P(PSTR("%u"), PlzIJson.ts);
            if (PlzSettings.mqtt_topic == 0) {
                MqttPublishPrefixTopicRulesProcess_P(TELE, JSON_TS);
            } else if (PlzSettings.mqtt_topic == 2) {
                MqttPublishPrefixTopicRulesProcess_P(TELE, "INFO/TS");
            }
        }
        Response_P(PSTR("%s"), commandResultToString());
        if (PlzSettings.mqtt_topic == 0) {
            MqttPublishPrefixTopicRulesProcess_P(TELE, JSON_RSP);
        } else if (PlzSettings.mqtt_topic == 2) {
            MqttPublishPrefixTopicRulesProcess_P(TELE, "INFO/RSP");
        }
    } 
    return PlzIJson.success;
}

bool PlzParserGET(char* cmnd) {
    PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande reçue cmnd=%s"), cmnd);
    if (strncmp(cmnd, "ALLS", 4) == 0 || strncmp(cmnd, "LALL", 4) == 0 || strncmp(cmnd, "JALL", 4) == 0) {
        PlzGetAllStatus();
    } else if (strncmp(cmnd, "TMPS", 4) == 0) {
        PlzGetAllTemps();
    } else if (strncmp(cmnd, "IOPT", 4) == 0) {
        PlzGetIO();
    } else if (strncmp(cmnd, "DPRS", 4) == 0) {
        PlzGetDPressData();
    } else if (strncmp(cmnd, "FAND", 4) == 0) {
        PlzGetFanData();
    } else if (strncmp(cmnd, "SETP", 4) == 0) {
        PlzGetSetPoint();
    } else if (strncmp(cmnd, "STAT", 4) == 0) {
        PlzGetStatus();
    } else if (strncmp(cmnd, "TIME", 4) == 0) {
        PlzGetDateTime();
    } else if (strncmp(cmnd, "MDVE", 4) == 0) {
        PlzGetModelVersion();
    } else if (strncmp(cmnd, "CHRD", 4) == 0) {
        PlzGetChronoData();
    } else if (strncmp(cmnd, "JTMR", 4) == 0) {
        PlzGetJTimer();
    //} else if (strncmp(cmnd, "CSET", 4) == 0) {
    } else if (strncmp(cmnd, "STDT", 4) == 0 || strncmp(cmnd, "LSTD", 4) == 0) {
        PlzGetStaticData();
        strcpy(cmnd, "STDT");
    } else if (strncmp(cmnd, "SERN", 4) == 0) {
        PlzGetSN();
    } else if (strncmp(cmnd, "LABL", 4) == 0) {
        PlzGetLabel();
    } else if (strncmp(cmnd, "CUNT", 4) == 0 || strncmp(cmnd, "CNTR", 4) == 0) {
        PlzGetCounters();
        strcpy(cmnd, "CNTR");
    } else if (strncmp(cmnd, "POWR", 4) == 0) {
        PlzGetPower();
    } else if (strncmp(cmnd, "PARM", 4) == 0) {
        PlzGetParamListData(cmnd + 5, "PAR");
    } else if (strncmp(cmnd, "HPAR", 4) == 0) {
        PlzGetHParamListData(cmnd + 5, "HPAR");
    }
    return (PlzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

bool PlzParserSET(char* cmnd) {
    if (strncmp(cmnd, "RFAN", 4) == 0) {
        PlzSetRoomFan(cmnd + 5);
        strcpy(cmnd, "FAND");
    } else if (strncmp(cmnd, "FN2D", 4) == 0) {
        PlzSetRoomFanDown();
        strcpy(cmnd, "FAND");
    } else if (strncmp(cmnd, "FN2U", 4) == 0) {
        PlzSetRoomFanUp();
        strcpy(cmnd, "FAND");
    } else if (strncmp(cmnd, "FN3L", 4) == 0) {
        PlzSetRoomFan3(cmnd + 5);
        strcpy(cmnd, "FAND");
    } else if (strncmp(cmnd, "FN4L", 4) == 0) {
        PlzSetRoomFan4(cmnd + 5);
        strcpy(cmnd, "FAND");
    } else if (strncmp(cmnd, "JTMR", 4) == 0) {
        PlzSetJTimer(cmnd + 5);
        PlzJSONAddBool("NODATA", (PlzIJson.cmdRes == Palazzetti::CommandResult::OK));
    } else if (strncmp(cmnd, "SLNT", 4) == 0) {
        PlzSetSilentMode(cmnd + 5);
    } else if (strncmp(cmnd, "POWR", 4) == 0) {
        PlzSetPower(cmnd + 5);
    } else if (strncmp(cmnd, "PWRD", 4) == 0) {
        PlzSetPowerDown();
        strcpy(cmnd, "POWR");
    } else if (strncmp(cmnd, "PWRU", 4) == 0) {
        PlzSetPowerUp();
        strcpy(cmnd, "POWR");
    } else if (strncmp(cmnd, "SETP", 4) == 0) {
        PlzSetSetPoint(cmnd + 5);
    } else if (strncmp(cmnd, "STPD", 4) == 0) {
        PlzSetSetPointDown();
        strcpy(cmnd, "SETP");
    } else if (strncmp(cmnd, "STPU", 4) == 0) {
        PlzSetSetPointUp();
        strcpy(cmnd, "SETP");
    } else if (strncmp(cmnd, "STPF", 4) == 0) {
        PlzSetSetPointFloat(cmnd + 5);
        strcpy(cmnd, "SETP");
    } else if (strncmp(cmnd, "TIME", 4) == 0) {
        PlzSetDateTime(cmnd + 5);
        strcpy(cmnd, "TIME");
    } else if (strncmp(cmnd, "CPRD", 4) == 0) {
        PlzSetChronoPrg(cmnd + 5);
        strcpy(cmnd, "CHRSTATUS");
    } else if (strncmp(cmnd, "CSTH", 4) == 0) {
        PlzSetChronoStartHH(cmnd + 5);
        strcpy(cmnd, "CHRSTATUS");
    } else if (strncmp(cmnd, "CSTM", 4) == 0) {
        PlzSetChronoStartMM(cmnd + 5);
        strcpy(cmnd, "CHRSTATUS");
    } else if (strncmp(cmnd, "CSPH", 4) == 0) {
        PlzSetChronoStopHH(cmnd + 5);
        strcpy(cmnd, "CHRSTATUS");
    } else if (strncmp(cmnd, "CSPM", 4) == 0) {
        PlzSetChronoStopMM(cmnd + 5);
        strcpy(cmnd, "CHRSTATUS");
    } else if (strncmp(cmnd, "CDAY", 4) == 0) {
        PlzSetChronoDay(cmnd + 5);
        strcpy(cmnd, "CHRSTATUS");
    } else if (strncmp(cmnd, "CSET", 4) == 0) {
        PlzSetChronoSetpoint(cmnd + 5);
        strcpy(cmnd, "CHRSTATUS");
    } else if (strncmp(cmnd, "CSST", 4) == 0) {
        PlzSetChronoStatus(cmnd + 5);
        strcpy(cmnd, "CHRSTATUS");
    //} else if (strncmp(cmnd, "CDYD", 4) == 0) {
    } else if (strncmp(cmnd, "PARM", 4) == 0) {
        PlzSetParameter(cmnd + 5, "PAR");
    } else if (strncmp(cmnd, "HPAR", 4) == 0) {
        PlzSetHiddenParameter(cmnd + 5, "HPAR");
    } else if (strncmp(cmnd, "LABL", 4) == 0) {
        PlzSetLabel(cmnd + 5);
    } else if (strncmp(cmnd, "LMAX", 4) == 0) {
    //EXT+ADRD+813F+0
    } else if (strncmp(cmnd, "LMIN", 4) == 0) {
    //EXT+ADRD+80D5+0
    } else if (strncmp(cmnd, "SERN", 4) == 0) {
        PlzSetSN(cmnd + 5);
     }

    return (PlzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

bool PlzParserCMD(char* cmnd) {
    if (strncmp(cmnd, "ON", 2) == 0) {
        PlzSetSwitchOn();
        strcpy(cmnd, "STAT");
    } else if (strncmp(cmnd, "OFF", 3) == 0) {
        PlzSetSwitchOff();
        strcpy(cmnd, "STAT");
    }
    return (PlzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

bool PlzParserBKP(char* cmnd) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande reçue cmnd BKP=%s"), cmnd);
    if (strncmp(cmnd, "PARM", 4) == 0) {
        PlzGetAllParameters(cmnd);
    } else if (strncmp(cmnd, "HPAR", 4) == 0) {
        PlzGetAllHiddenParameters(cmnd);
    }
    return (PlzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

bool PlzParserEXT(char* cmnd) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande reçue cmnd EXT=%s"), cmnd);
    if (strncmp(cmnd, "STDT", 4) == 0) {
        //pala.iUpdateStaticData(); private method
    } else if (strncmp(cmnd, "ADWR", 4) == 0) {
        PlzWriteData(cmnd + 5);
    } else if (strncmp(cmnd, "ADRD", 4) == 0) {
        PlzReadData(cmnd + 5);
    } else if (strncmp(cmnd, "HPRL", 4) == 0) {
        PlzGetHParamListData(cmnd + 5, "HP");
    } else if (strncmp(cmnd, "PARL", 4) == 0) {
        PlzGetParamListData(cmnd + 5, "P");
    } else if (strncmp(cmnd, "LMXL", 4) == 0) {
        PlzGetLimMaxListData();
    } else if (strncmp(cmnd, "LMNL", 4) == 0) {
        PlzGetLimMinListData();
    }
    //
    //LMNL
    return (PlzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

void PlzReadData(const char *cmd) {
    char* endptr = nullptr;
    errno = 0;
    int hex = strtol(cmd, &endptr, 16);
    int value = strtol(cmd + 5, &endptr, 10);

    if (checkStrtolError(cmd, endptr) || checkStrtolError(cmd + 5, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing key or value PlzReadData"));
        return;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzReadData hex=%d, value=%d"), hex, value);

    PlzIJson.cmdRes = pala.readData(hex, value, &Plz.ADDR_DATA);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[10];
        snprintf_P(key, sizeof(key), PSTR("ADDR_%04X"), hex % 0x10000);
        PlzJSONAddStr("DATATYPE", (value > 0 ? "WORD" : "BYTE"));
        PlzJSONAddInt(key, Plz.ADDR_DATA);
    }
}

void PlzSetChronoDay(const char* cmd) {
    char* endptr = nullptr;
    errno = 0;
    int day = strtol(cmd, &endptr, 10);
    int memory = strtol(cmd + 2, &endptr, 10);
    int program = strtol(cmd + 4, &endptr, 10);

    if (checkStrtolError(cmd, endptr) || checkStrtolError(cmd + 2, endptr) || checkStrtolError(cmd + 4, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing day or program data for setChronoDay"));
        return;
    }

    PlzIJson.cmdRes = pala.setChronoDay(day, memory, program);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char dayName[3] = {'D', static_cast<char>(day + '0'), '\0'};
        char memoryName[3] = {'M', static_cast<char>(memory + '0'), '\0'};
        char programName[3] = {'P', static_cast<char>(program + '0'), '\0'};

        PlzJSONAddObj(dayName);
        PlzJSONAddObj(memoryName);
        if (program) {
            PlzJSONAddStr("Program", programName);
        } else {
            PlzJSONAddStr("Program", "OFF");
        }
        PlzJSONCloseObj(); // Close memory object
        PlzJSONCloseObj(); // Close day object
    }
}

void PlzSetChronoPrg(const char* cmd) {
    char* endptr = nullptr;
    errno = 0;
    int program = strtol(cmd, &endptr, 10);
    int setpoint = strtol(cmd + 2, &endptr, 10);
    int startHour = strtol(cmd + 5, &endptr, 10);
    int startMinute = strtol(cmd + 8, &endptr, 10);
    int stopHour = strtol(cmd + 11, &endptr, 10);
    int stopMinute = strtol(cmd + 14, &endptr, 10);

    if (checkStrtolError(cmd, endptr) || checkStrtolError(cmd + 3, endptr) ||
        checkStrtolError(cmd + 8, endptr) || checkStrtolError(cmd + 12, endptr) ||
        checkStrtolError(cmd + 15, endptr) || checkStrtolError(cmd + 18, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing chrono program data for setChronoPrg"));
        return;
    }

    PlzIJson.cmdRes = pala.setChronoPrg(program, setpoint, startHour, startMinute, stopHour, stopMinute);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char programName[3] = {'P', static_cast<char>(program + '0'), '\0'};
        float chrsetp = static_cast<float>(setpoint) / 10.0f; // Assuming setpoint is in tenths

        char startTime[6];
        snprintf_P(startTime, sizeof(startTime), TIME_HOUR_MINUTE, startHour, startMinute);

        char stopTime[6];
        snprintf_P(stopTime, sizeof(stopTime), TIME_HOUR_MINUTE, stopHour, stopMinute);

        PlzJSONAddObj(programName);
        PlzJSONAddFloat("CHRSETP", chrsetp);
        PlzJSONAddStr("START", startTime);
        PlzJSONAddStr("STOP", stopTime);
        PlzJSONCloseObj(); // Close program object
    }
}

void PlzSetChronoSetpoint(const char* cmd) {
    char* endptr = nullptr;
    errno = 0;
    int programNumber = strtol(cmd, &endptr, 10);
    int setpoint = strtol(cmd + 2, &endptr, 10);

    if (checkStrtolError(cmd, endptr) || checkStrtolError(cmd + 3, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing setpoint for setChronoSetpoint"));
        return;
    }

    PlzIJson.cmdRes = pala.setChronoSetpoint(programNumber, setpoint);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddStr("Result", "OK");
    }
}

void PlzSetChronoStartHH(const char* cmd) {
    char* endptr = nullptr;
    errno = 0;
    int programNumber = strtol(cmd, &endptr, 10);
    int startHour = strtol(cmd + 2, &endptr, 10);
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande getParameter programNumber=%d, startMinute=%d"), programNumber, startHour);
return;
//ESP-PLZ: Commande getParameter programNumber=8, startMinute=8

    if (checkStrtolError(cmd, endptr) || checkStrtolError(cmd + 2, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing start hour for setChronoStartHH"));
        return;
    }
    PlzIJson.cmdRes = pala.setChronoStartHH(programNumber, startHour);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddStr("Result", "OK");
    }
}

void PlzSetChronoStartMM(const char* cmd) {
    char* endptr = nullptr;
    errno = 0;
    int programNumber = strtol(cmd, &endptr, 10);
    int startMinute = strtol(cmd + 2, &endptr, 10);

    if (checkStrtolError(cmd, endptr) || checkStrtolError(cmd + 2, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing start minute for setChronoStartMM"));
        return;
    }

    PlzIJson.cmdRes = pala.setChronoStartMM(programNumber, startMinute);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddStr("Result", "OK");
    }
}

void PlzSetChronoStatus(const char* cmd) {
    char* endptr = nullptr;
    errno = 0;
    int status = strtol(cmd, &endptr, 10);

    if (checkStrtolError(cmd, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing status for setChronoStatus"));
        return;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzSetChronoStatus cmdStr=%d"), status);

    PlzIJson.cmdRes = pala.setChronoStatus(status, &Plz.CHRSTATUS);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("CHRSTATUS", Plz.CHRSTATUS);
    }
}

void PlzSetChronoStopHH(const char* cmd) {
    char* endptr = nullptr;
    errno = 0;
    int programNumber = strtol(cmd, &endptr, 10);
    int stopHour = strtol(cmd + 2, &endptr, 10);

    if (checkStrtolError(cmd, endptr) || checkStrtolError(cmd + 2, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing stop hour for setChronoStopHH"));
        return;
    }

    PlzIJson.cmdRes = pala.setChronoStopHH(programNumber, stopHour);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddStr("Result", "OK");
    }
}

void PlzSetChronoStopMM(const char* cmd) {
    char* endptr = nullptr;
    errno = 0;
    int programNumber = strtol(cmd, &endptr, 10);
    int stopMinute = strtol(cmd + 2, &endptr, 10);

    if (checkStrtolError(cmd, endptr) || checkStrtolError(cmd + 2, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing stop minute for setChronoStopMM"));
        return;
    }

    PlzIJson.cmdRes = pala.setChronoStopMM(programNumber, stopMinute);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddStr("Result", "OK");
    }
}

void PlzSetDateTime(const char* cmd) {
    char* endptr = nullptr;
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande setDateTime cmd=%s"), cmd);
    int year = strtol(cmd, &endptr, 10); // Extraction de l'année
    errno = 0;
    if (checkStrtolError(cmd, endptr)) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing Year"));
        return;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande setDateTime year=%d"), year);
    
    int month = strtol(endptr + 1, &endptr, 10); // Extraction du mois
    errno = 0;
    if (checkStrtolError(endptr + 1, endptr)) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing Month"));
        return;
    }
    
    int day = strtol(endptr + 1, &endptr, 10); // Extraction du jour
    errno = 0;
    if (checkStrtolError(endptr + 1, endptr)) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing Day"));
        return;
    }
    
    int hour = strtol(endptr + 1, &endptr, 10); // Extraction de l'heure
    errno = 0;
    if (checkStrtolError(endptr + 1, endptr)) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing Hour"));
        return;
    }
    
    int minute = strtol(endptr + 1, &endptr, 10); // Extraction de la minute
    errno = 0;
    if (checkStrtolError(endptr + 1, endptr)) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing Minute"));
        return;
    }
    
    int second = strtol(endptr + 1, nullptr, 10); // Extraction de la seconde
    errno = 0;
    if (checkStrtolError(endptr + 1, nullptr)) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing Second"));
        return;
    }

    // Log des informations d'entrée
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: setDateTime year=%d, month=%d, day=%d, hour=%d, minute=%d, second=%d"), year, month, day, hour, minute, second);

    if (year < 2000 || year > 2099) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Incorrect Year"));
        return;
    }
    if (month < 1 || month > 12) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Incorrect Month"));
        return;
    }
    bool isLeapYear = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
    int maxDay = (month == 2) ? (isLeapYear ? 29 : 28) : (31 - (month - 1) % 7 % 2);
    if (day < 1 || day > maxDay) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Incorrect Day"));
        return;
    }
    if (hour < 0 || hour > 23) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Incorrect Hour"));
        return;
    }
    if (minute < 0 || minute > 59) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Incorrect Minute"));
        return;
    }
    if (second < 0 || second > 59) {
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Incorrect Second"));
        return;
    }

    PlzIJson.cmdRes = pala.setDateTime(year, month, day, hour, minute, second, &Plz.STOVE_DATETIME, &Plz.STOVE_WDAY);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddStr("STOVE_DATETIME", Plz.STOVE_DATETIME);
        PlzJSONAddInt("STOVE_WDAY", Plz.STOVE_WDAY);
    }
}

void PlzSetHiddenParameter(const char *setHiddenParam, const char *prefix) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande setHiddenParameter cmdStr=%s"), setHiddenParam);
    char* endptr4 = nullptr;
    errno = 0;
    int setHiddenParamIndex = strtol(setHiddenParam, &endptr4, 10);
    if (checkStrtolError(setHiddenParam, endptr4)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }

    char* endptr5 = nullptr;
    errno = 0;
    int setHiddenParamValue = strtol(endptr4, &endptr5, 10);
    if (checkStrtolError(endptr4, endptr5)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande setHiddenParameter index=%d value=%d"), setHiddenParamIndex, setHiddenParamValue);
    PlzIJson.cmdRes = pala.setHiddenParameter(setHiddenParamIndex, setHiddenParamValue);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[8];
        snprintf(key, sizeof(key), "%s%d", prefix, setHiddenParamIndex);
        PlzJSONAddInt(key, setHiddenParamValue);
    }
}

void PlzSetLabel(const char *setLabelParam) {
    if (strlen(setLabelParam) < 33) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::OK;
        PlzJSONAddStr("LABEL", setLabelParam);
        SettingsUpdateText(SET_HOSTNAME, setLabelParam);
        TasmotaGlobal.restart_flag = 2;
    } else {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Label too long"));
    }
}

void PlzSetParameter(const char *setParam, const char *prefix) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande setParameter cmdStr=%s"), setParam);
    char* endptr1 = nullptr;
    errno = 0;
    int setParamIndex = strtol(setParam, &endptr1, 10);
    if (checkStrtolError(setParam, endptr1)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }

    char* endptr2 = nullptr;
    errno = 0;
    int setParamValue = strtol(endptr1, &endptr2, 10);
    if (checkStrtolError(endptr1, endptr2)) {
    //int32_t setParamValue = strtol(endptr1, &endptr2, 10);
    //if (setParamValue == 0 == errno == 0) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande setParameter index=%d value=%d"), setParamIndex, setParamValue);
    PlzIJson.cmdRes = pala.setParameter(setParamIndex, setParamValue);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[8];
        snprintf(key, sizeof(key), "%s%d", prefix, setParamIndex);
        PlzJSONAddInt(key, setParamValue);
    }
}

void PlzSetPower(const char *setPowerLevel) {
    char* endptr = nullptr;
    errno = 0;
    int powerLevel = strtol(setPowerLevel, &endptr, 10);
    if (checkStrtolError(setPowerLevel, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }

    PlzIJson.cmdRes = pala.setPower(static_cast<byte>(powerLevel), &Plz.PWR, &Plz.isF2LValid, &Plz.F2L, &Plz.FANLMINMAX);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("PWR", Plz.PWR);
        if (Plz.isF2LValid) {
            PlzJSONAddInt("F2L", Plz.F2L);
        }
        PlzJSONAddArray("FANLMINMAX", Plz.FANLMINMAX, 6);
    }
}

bool PlzSetPowerBtn(void)
{
    bool status = false;
    bool hasSwitch = (Plz.STOVETYPE != 7 && Plz.STOVETYPE != 8);
    bool isStarted = (Plz.LSTATUS == 0 || Plz.LSTATUS == 1 || Plz.LSTATUS == 6 || Plz.LSTATUS == 7 || Plz.LSTATUS == 9
                    || Plz.LSTATUS == 11 || Plz.LSTATUS == 12 || Plz.LSTATUS == 51 || Plz.LSTATUS == 501 || Plz.LSTATUS == 504
                    || Plz.LSTATUS == 505 || Plz.LSTATUS == 506 || Plz.LSTATUS == 507);

    AddLog(LOG_LEVEL_INFO, PSTR("activedevice %d"), TasmotaGlobal.active_device);

    if (XdrvMailbox.payload != SRC_SWITCH && plzSerial && hasSwitch) {  // ignore to prevent loop from pushing state from faceplate interaction
        if (TasmotaGlobal.power != isStarted) {
            char cmdBuffer[32] = { 0 };
            if (TasmotaGlobal.power) {
                snprintf(cmdBuffer, sizeof(cmdBuffer), "Palazzetti sendmsg CMD ON");
                ExecuteCommand(cmdBuffer, SRC_WEBCONSOLE);
            } else {
                snprintf(cmdBuffer, sizeof(cmdBuffer), "Palazzetti sendmsg CMD OFF");
                ExecuteCommand(cmdBuffer, SRC_WEBCONSOLE);
            }
            status = true;
        }
        AddLog(LOG_LEVEL_INFO, PSTR("bitRead %d"),(bitRead(XdrvMailbox.index, TasmotaGlobal.active_device-1) ^ bitRead(TasmotaGlobal.rel_inverted, TasmotaGlobal.active_device-1)));
    }
    return status;
}

void PlzSetPowerDown() {
    PlzIJson.cmdRes = pala.setPowerDown(&Plz.PWR, &Plz.isF2LValid, &Plz.F2L, &Plz.FANLMINMAX);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("PWR", Plz.PWR);
        if (Plz.isF2LValid) {
            PlzJSONAddInt("F2L", Plz.F2L);
        }
        PlzJSONAddArray("FANLMINMAX", Plz.FANLMINMAX, 6);
    }
}

void PlzSetPowerUp() {
    PlzIJson.cmdRes = pala.setPowerUp(&Plz.PWR, &Plz.isF2LValid, &Plz.F2L, &Plz.FANLMINMAX);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("PWR", Plz.PWR);
        if (Plz.isF2LValid) {
            PlzJSONAddInt("F2L", Plz.F2L);
        }
        PlzJSONAddArray("FANLMINMAX", Plz.FANLMINMAX, 6);
    }
}

void PlzSetRoomFan(const char *setRoomFan) {
    char* endptr = nullptr;
    errno = 0;
    int fanLevel = strtol(setRoomFan, &endptr, 10); // Extraction de la valeur de fanLevel

    if (checkStrtolError(setRoomFan, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing fan level for setRoomFan"));
        return;
    }

    PlzIJson.cmdRes = pala.setRoomFan(fanLevel, &Plz.isPWRValid, &Plz.PWR, &Plz.F2L, &Plz.F2LF);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        if (Plz.isPWRValid) {
            PlzJSONAddInt("PWR", Plz.PWR);
        }
        PlzJSONAddInt("F2L", Plz.F2L);
        PlzJSONAddInt("F2LF", Plz.F2LF);
    }
}

void PlzSetRoomFan3(const char *cmd) {
    char* endptr = nullptr;
    errno = 0;
    int fanLevel = strtol(cmd, &endptr, 10); // Extraction de la valeur de fanLevel

    if (checkStrtolError(cmd, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing fan level for setRoomFan3"));
        return;
    }

    PlzIJson.cmdRes = pala.setRoomFan3(fanLevel, &Plz.F3L);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("F3L", Plz.F3L);
    }
}

void PlzSetRoomFan4(const char *cmd) {
    char* endptr = nullptr;
    errno = 0;
    int fanLevel = strtol(cmd, &endptr, 10); // Extraction de la valeur de fanLevel

    if (checkStrtolError(cmd, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing fan level for setRoomFan4"));
        return;
    }

    PlzIJson.cmdRes = pala.setRoomFan4(fanLevel, &Plz.F4L);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("F4L", Plz.F4L);
    }
}

void PlzSetRoomFanDown() {
    PlzIJson.cmdRes = pala.setRoomFanDown(&Plz.isPWRValid, &Plz.PWR, &Plz.F2L, &Plz.F2LF);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        if (Plz.isPWRValid) {
            PlzJSONAddInt("PWR", Plz.PWR);
        }
        PlzJSONAddInt("F2L", Plz.F2L);
        PlzJSONAddInt("F2LF", Plz.F2LF);
    }
}

void PlzSetRoomFanUp() {
    PlzIJson.cmdRes = pala.setRoomFanUp(&Plz.isPWRValid, &Plz.PWR, &Plz.F2L, &Plz.F2LF);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        if (Plz.isPWRValid) {
            PlzJSONAddInt("PWR", Plz.PWR);
        }
        PlzJSONAddInt("F2L", Plz.F2L);
        PlzJSONAddInt("F2LF", Plz.F2LF);
    }
}

void PlzSetSetPoint(const char *setSetPoint) {
    char* endptr5 = nullptr;
    errno = 0;
    int setSetPointValue = strtol(setSetPoint, &endptr5, 10);
    if (checkStrtolError(setSetPoint, endptr5)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }

    PlzIJson.cmdRes = pala.setSetpoint(static_cast<byte>(setSetPointValue), &Plz.SETP);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddFloat("SETP", Plz.SETP);
    }
}

void PlzSetSetPointDown() {
    PlzIJson.cmdRes = pala.setSetPointDown(&Plz.SETP);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddFloat("SETP", Plz.SETP);
    }
}

void PlzSetSetPointFloat(const char *setSetPointFloat) {
    char* endptr = nullptr;
    errno = 0;
    int intPart = strtol(setSetPointFloat, &endptr, 10);
    if (checkStrtolError(setSetPointFloat, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }
    errno = 0;
    int decPart = strtol(endptr + 1, &endptr, 10);
    if (checkStrtolError(endptr + 1, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }

    while (decPart >= 100) decPart /= 10;
    if (decPart < 10) decPart *= 10;
    decPart = (decPart + 10) / 20 * 20;
    if (decPart >= 100) {
        intPart++;
        decPart -= 100;
    }
    float setPointFloat = intPart + decPart / 100.0f;

    PlzIJson.cmdRes = pala.setSetpoint(setPointFloat, &Plz.SETP);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddFloat("SETP", Plz.SETP);
    }
}

void PlzSetSetPointUp() {
    PlzIJson.cmdRes = pala.setSetPointUp(&Plz.SETP);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddFloat("SETP", Plz.SETP);
    }
}

void PlzSetSilentMode(const char *cmd) {
    char* endptr = nullptr;
    errno = 0;
    int silentMode = strtol(cmd, &endptr, 10);
    if (checkStrtolError(cmd, endptr)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        return;
    }

    PlzIJson.cmdRes = pala.setSilentMode(silentMode, &Plz.SLNT, &Plz.PWR, &Plz.F2L, &Plz.F2LF, &Plz.isF3LF4LValid, &Plz.F3L, &Plz.F4L);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("SLNT", Plz.SLNT);
        PlzJSONAddInt("PWR", Plz.PWR);
        PlzJSONAddInt("F2L", Plz.F2L);
        PlzJSONAddInt("F2LF", Plz.F2LF);
        if (Plz.isF3LF4LValid) {
            PlzJSONAddInt("F3L", Plz.F3L);
            PlzJSONAddInt("F4L", Plz.F4L);
        }
    }
}

void PlzSetSN(const char *cmd) {
    //PlzIJson.cmdRes = pala.setSN(&Plz.SN);
    PlzJSONAddStr("SN", "LT201629480580256025776");
    PlzIJson.cmdRes = Palazzetti::CommandResult::OK;
}

void PlzSetSwitchOff() {
    PlzIJson.cmdRes = pala.switchOff(&Plz.STATUS, &Plz.LSTATUS, &Plz.FSTATUS);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("STATUS", Plz.STATUS);
        PlzJSONAddInt("LSTATUS", Plz.LSTATUS);
        PlzJSONAddInt("FSTATUS", Plz.FSTATUS);
    }
}

void PlzSetSwitchOn() {
    PlzIJson.cmdRes = pala.switchOn(&Plz.STATUS, &Plz.LSTATUS, &Plz.FSTATUS);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        PlzJSONAddInt("STATUS", Plz.STATUS);
        PlzJSONAddInt("LSTATUS", Plz.LSTATUS);
        PlzJSONAddInt("FSTATUS", Plz.FSTATUS);
    }
}

void PlzSendParametersResponse(char* fileType, const void* params, size_t paramCount, const char* paramType) {

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzSendParametersResponse=%s"), fileType);
    if (strcmp(fileType, "CSV") == 0 ) {
        char header[50];
        snprintf(header, sizeof(header), "%s;VALUE\r\n", paramType);
        strncat(PlzIJson.data, header, sizeof(PlzIJson.data) - strlen(PlzIJson.data) - 1);

        for (size_t i = 0; i < paramCount; i++) {
            PlzJSONAddCSV(String(i).c_str(), ((const byte*)params)[i]);
        }
    } else if (strcmp(fileType, "JSON") == 0) {
        PlzJSONAddArray(paramType);
        for (size_t i = 0; i < paramCount; i++) {
            PlzJSONAddIntArr(((const byte*)params)[i]);
        }
        PlzJSONCloseArray();
    }
}

bool PlzSyscmdCmd(const char* cmd) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzSyscmdCmd cmd=%s"), cmd);
    //wifiscan
    //listbledev
    //nwdata
    //netdata
    //settz &tz=
    //setsystemclock datetime
    char cmndType[4] = { 0 };
    char cmnd[25] = { 0 };

    const char* separator = strchr(cmd, ' ');
    if (!separator) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzParser no separator found"));
        return false;
    }

    strncpy(cmndType, cmd, separator - cmd);
    cmndType[3] = '\0';
    strncpy(cmnd, separator + 1, sizeof(cmnd) - 1);

    // Convertir en majuscules
    for (char &c : cmndType) c = toupper(c);
    for (char &c : cmnd) c = toupper(c);
    PlzIJson.success = false;

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzSyscmdCmd cmndType=%s"), cmndType);
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzSyscmdCmd cmnd=%s"), cmnd);

    if (strcasecmp(cmndType, "SETTZ") == 0) {
        //PlzIJson.success = PlzParserGET(cmnd);
    } else if (strcasecmp(cmndType, "SETTZ") == 0) {
    }
    return false;
}

void PlzWriteData(const char *cmd) {
    char* endptr1 = nullptr;
    errno = 0;
    int hex = strtol(cmd, &endptr1, 16);
    if (checkStrtolError(cmd, endptr1)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing hex value PlzWriteData"));
        return;
    }

    char* endptr2 = nullptr;
    errno = 0;
    int value = strtol(endptr1, &endptr2, 10);
    if (checkStrtolError(endptr1, endptr2)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing value PlzWriteData"));
        return;
    }

    char* endptr3 = nullptr;
    errno = 0;
    int mode = strtol(endptr2, &endptr3, 10);
    if (checkStrtolError(endptr2, endptr3)) {
        PlzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf(PlzIJson.msg, sizeof(PlzIJson.msg), PSTR("Error parsing mode value PlzWriteData"));
        return;
    }

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande PlzWriteData hex=%d, value=%d, mode=%d"), hex, value, mode);
    return;
    PlzIJson.cmdRes = pala.writeData(hex, value, mode);
    if (PlzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[10];
        snprintf_P(key, sizeof(key), PSTR("ADDR_%04X"), hex % 0x10000);
        PlzJSONAddStr("DATATYPE", (value > 0 ? "WORD" : "BYTE"));
        PlzJSONAddInt(key, value % 0x10000);
    }
}

bool PlzCmd(void) {
    char command[CMDSZ];
    char *arg_part = NULL;
    char *cmd_part = NULL;

    bool serviced = false;
    uint8_t disp_len = strlen(D_NAME_PALAZZETTI);

    if (!strncasecmp_P(XdrvMailbox.topic, PSTR(D_NAME_PALAZZETTI), disp_len)) {

        if (XdrvMailbox.data_len > 0) {
            cmd_part = strtok_r(XdrvMailbox.data, " ", &arg_part);
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Cmd %s"), cmd_part);
            //AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Cmd arg_part=%s"), arg_part);

            uint8_t command_code = GetCommandCode(command, sizeof(command), cmd_part, kPalazzetti_Commands);
            uint8_t newSetting = 255;

            //AddLog(LOG_LEVEL_INFO, PSTR("PLZ: What cmd code=%d cmd=%s arg=%s"), command_code, cmd_part, arg_part);
            switch (command_code) {
                case CMND_PALAZZETTI_SENDMSG:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Sendmsg cmd %s"), arg_part);
                    if (PlzRequest.is_connected && *arg_part != '\0') {
                        PlzRequest.command_data = (char*)malloc(strlen(arg_part) + 1);
                        if (PlzRequest.command_data != nullptr) {
                            strcpy(PlzRequest.command_data, arg_part);
                            serviced = PlzParser();
                            free(PlzRequest.command_data);
                            PlzRequest.command_data = nullptr;
                        } else {
                            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Memory allocation for command_data failed"));
                            //serviced = false;
                        }
                    } else {
                        serviced = true;
                    }
                    break;
                case CMND_PALAZZETTI_INIT:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Init cmd"));
                    serviced = true;
                    PlzInit();
                    break;
                case CMND_PALAZZETTI_INTERVAL:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Interval cmd=%s"), arg_part);
                    if (PlzSettings.interval > atoi(arg_part)) {
                        itoa(PlzSettings.interval, arg_part, 10);
                    }
                    PlzSettings.interval = atoi(arg_part);
                    PlzSettingsSave();
                    PlzSettingsPublish();
                    serviced = true;
                    break;
                case CMND_PALAZZETTI_SYNCCLOCK:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Timesync cmd"));
                    if (strcmp(arg_part, "0") == 0 || strcmp(arg_part, "off") == 0) {
                        newSetting = 0;  // Désactiver
                    } else if (strcmp(arg_part, "1") == 0 || strcmp(arg_part, "on") == 0) {
                        newSetting = 1;  // Activer
                    }
                    
                    if (newSetting != 255) {
                        PlzSettings.sync_clock = newSetting;
                        PlzSettingsSave();  // Sauvegarder les settings mis à jour
                        PlzSettingsPublish();
                    } else {
                        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Invalid sync_clock argument: %s"), arg_part);
                        PlzSettingsPublish();
                    }

                    serviced = true;
                    break;
                case CMND_PALAZZETTI_MQTT:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: mqtt"));
                    
                    if (strcmp(arg_part, "0") == 0 || strcmp(arg_part, "raw") == 0) {
                        newSetting = 0;
                    } else if (strcmp(arg_part, "1") == 0 || strcmp(arg_part, "json") == 0) {
                        newSetting = 1;
                    } else if (strcmp(arg_part, "2") == 0 || strcmp(arg_part, "rawtopic") == 0) {
                        newSetting = 2;
                    }
                    
                    if (newSetting != 255) {
                        PlzSettings.mqtt_topic = newSetting;
                        PlzSettingsSave();
                        PlzSettingsPublish();
                    } else {
                        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Invalid MQTT argument: %s"), arg_part);
                        PlzSettingsPublish();
                    }
                    serviced = true;
                    break;
                default:
                    PlzDisplayCmdHelp();
                    break;
            }
        } else {
            PlzDisplayCmdHelp();
        }
    }
    return serviced;
}

void PlzDisplayCmdHelp(void) {
    AddLog(LOG_LEVEL_INFO, PSTR("Palazzetti commands :"));
    AddLog(LOG_LEVEL_INFO, PSTR("    sendmsg [GET/SET/CMD/BKP/EXT] [] []   Send commands to Palazzetti"));
    AddLog(LOG_LEVEL_INFO, PSTR("    init                                  Initialize serial connection"));
    AddLog(LOG_LEVEL_INFO, PSTR("    interval=%d                        Refresh informations interval"), PlzSettings.interval);
    AddLog(LOG_LEVEL_INFO, PSTR("    sync=%d                            Sync Palazzetti time from Tasmota"), PlzSettings.sync_clock);
    AddLog(LOG_LEVEL_INFO, PSTR("    mqtt=%d                             MQTT topic usage"), PlzSettings.mqtt_topic);
}

void PlzSettingsDefault(void) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: " D_USE_DEFAULTS));
    PlzSettings.sync_clock = PLZ_SETTINGS_TIMESYNC;
    PlzSettings.mqtt_topic = PLZ_SETTINGS_MQTT_TOPIC;
    PlzSettings.interval = PLZ_SETTINGS_INTERVAL;
    PlzSettings.version = PLZ_SETTINGS_VERSION;
}

bool PlzSettingsRestore(void) {
    XdrvMailbox.data  = (char*)&PlzSettings;
    XdrvMailbox.index = sizeof(PlzSettings);
    return true;
}

void PlzSettingsLoad(bool erase) {
    // Called from FUNC_PRE_INIT once at restart
    // Called from FUNC_RESET_SETTINGS

    memset(&PlzSettings, 0x00, sizeof(PlzSettings));
    // Init default values in case file is not found
    PlzSettingsDefault();

    char filename[20];
    // Use for drivers:
    snprintf_P(filename, sizeof(filename), PSTR(TASM_FILE_DRIVER), XDRV_100);

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: About to load settings from file %s"), filename);

#ifdef USE_UFILESYS
    if (erase) {
        // Use defaults
        TfsDeleteFile(filename);  
    }
    else if (TfsLoadFile(filename, (uint8_t*)&PlzSettings, sizeof(PlzSettings))) {
        // Fix possible setting deltas
        
    } else {
        // File system not ready: No flash space reserved for file system
        AddLog(LOG_LEVEL_INFO, PSTR(PLZ_ERROR_FILE_NOT_FOUND));
    }
#else
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Error, file system not enabled"));
#endif  // USE_UFILESYS
}

void PlzSettingsSave(void) {
  // Called from FUNC_SAVE_SETTINGS every SaveData second and at restart
  uint32_t crc32 = GetCfgCrc32((uint8_t*)&PlzSettings +4, sizeof(PlzSettings) -4);  // Skip crc32
  if (crc32 != PlzSettings.crc32 && PlzSettings.version > 0) {
    PlzSettings.crc32 = crc32;
    
    char filename[20];
    snprintf_P(filename, sizeof(filename), PSTR(TASM_FILE_DRIVER), XDRV_100);

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: About to save settings to file %s"), filename);

#ifdef USE_UFILESYS
    if (!TfsSaveFile(filename, (const uint8_t*)&PlzSettings, sizeof(PlzSettings))) {
        // File system not ready: No flash space reserved for file system
        AddLog(LOG_LEVEL_INFO, PLZ_ERROR_FILE_NOT_FOUND);
    }
#else
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Error, file system not enabled"));
#endif  // USE_UFILESYS
    }
}

void PlzSettingsPublish(void) {
	Response_P(PSTR("{\"SyncClock\":\"%s\""), PlzSettings.sync_clock ? "on" : "off");
    const char *topic = PlzSettings.mqtt_topic == 0 ? "raw" : PlzSettings.mqtt_topic == 1 ? "json" : PlzSettings.mqtt_topic == 2 ? "rawtopic" : "N/A";
	ResponseAppend_P(PSTR(",\"MQTTTopicName\":\"%s\""), topic);
	ResponseAppend_P(PSTR(",\"MQTTTopic\":%d"), PlzSettings.mqtt_topic);
	ResponseAppend_P(PSTR(",\"Interval\":%d"), PlzSettings.interval);
	ResponseAppend_P(PSTR(",\"Version\":%d}"), PlzSettings.version);
	MqttPublishPrefixTopicRulesProcess_P(TELE, PSTR("PlzSettings"));
}

// Initialisation
bool Xdrv100(uint32_t function) {
    bool result = false;
    switch (function)
    {
        case FUNC_INIT:
            PlzInit();  // Initialiser le contrôle
            break;
        case FUNC_SAVE_BEFORE_RESTART:
            if (PlzRequest.is_connected) { PlzSaveBeforeRestart(); }
            break;
        case FUNC_RESTORE_SETTINGS:
            result = PlzSettingsRestore();
            break;
        case FUNC_RESET_SETTINGS:
            PlzSettingsLoad(1);
            break;
        case FUNC_SAVE_SETTINGS:
            PlzSettingsSave();
            break;
        case FUNC_AFTER_TELEPERIOD:
            PlzSettingsPublish();
            break;
        case FUNC_PRE_INIT:
            PlzSettingsLoad(0);
            PlzDrvInit();
            break; 
        case FUNC_EVERY_SECOND:
            if (PlzRequest.is_connected) {
                PlzUpdate();
                PlzHandleUdpRequest();
                if (PlzSettings.sync_clock && RtcTime.valid && Plz.STOVE_DATETIME[0] != '\0') {
                    PlzSyncClockWithStove();
                }
            } else {
                unsigned long currentTime = millis();
                if (currentTime - PlzRequest.last_attempt >= PlzRequest.retry_interval) {
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Tentative de reconnexion..."));
                    PlzInit();
                    PlzRequest.last_attempt = currentTime;
                }
            }
            break;
        case FUNC_COMMAND:
            result = PlzCmd();
            break;
        /*case FUNC_JSON_APPEND:
            if (PlzRequest.is_connected) { PlzShow(1); }
            break;*/
        case FUNC_SET_DEVICE_POWER:
            PlzSetPowerBtn();
            break;
    #ifdef USE_WEBSERVER
        case FUNC_WEB_ADD_HANDLER:
            WebServer_on(PLZ_SENDMSG_ENDPOINT, PlzSendmsgGetRequestHandler, HTTP_GET);
            WebServer_on(PLZ_SENDMSG_ENDPOINT, PlzSendmsgPostRequestHandler, HTTP_POST);
            WebServer_on(PLZ_SYSCMD_ENDPOINT, PlzSyscmdRequestHandler, HTTP_GET);
            break;
        case FUNC_WEB_GET_ARG:
            PlzSendmsgGetRequestHandler();
            break;
        case FUNC_WEB_SENSOR: // maybe to display last command sent
            if (PlzRequest.is_connected) { PlzShow(0); }
            break;
    #endif  // USE_WEBSERVER
        case FUNC_ACTIVE:
            result = true;
            break;
    }
    return result;
}
#endif //USE_PALAZZETTI