#include "SD_state_machine.h"

// Define timeout time in milliseconds,0 (example: 2000ms = 2s)
const long timeoutTime = 3000;
bool mounted = false;
char file_name[20]; 
File dataFile;
/* Debug Variables */
boolean savingBlink = false;
boolean saveFlag = false;

char ok_msg[] = "OK";
char error_msg[] = "ERRO";

void pinConfig()
{
  // Pins
  pinMode(EMBEDDED_LED, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
  //pinMode(CAN_INTERRUPT, INPUT_PULLUP);
  // pinMode(MODEM_RST, OUTPUT);
  // digitalWrite(MODEM_RST, HIGH);
  
  return;
}

bool start_SD_device()
{
  do { /*Serial.println("Mount SD...");*/ } while(!sdConfig() && millis() < timeoutTime);

  if(!mounted)
  {
    //Serial.println("SD mounted error!!");
    return false;
  } else {
    sdSave(true);
    setup_SD_ticker();
  }

  return mounted;
}

bool sdConfig()
{
  if(!SD.begin(SD_CS)) 
    return false;

  File root;
  root = SD.open("/");
  int num_files = countFiles(root);
  sprintf(file_name, "/%s%d.csv", "data", num_files + 1);
  mounted = true;

  return true;
}

int countFiles(File dir)
{
  int fileCountOnSD = 0; // for counting files
  for(;;)
  {
    File entry = dir.openNextFile();
    
    // no more files
    if(!entry)
      break;
    
    // for each file count it
    fileCountOnSD++;
    entry.close();
  }

  return fileCountOnSD - 1;
}

void sdSave(bool set, bluetooth bluetooth_packet)
{
  //bluetooth sd_save_data = update_packet_sd_save();

  dataFile = SD.open(file_name, FILE_APPEND);

  if(dataFile)
  {
    //dataFile.println(packetToString(set));
    dataFile.close();
    savingBlink = !savingBlink;
    digitalWrite(DEBUG_LED, savingBlink);
    bluetooth_packet.check_sd = 1;
  } else {
    digitalWrite(DEBUG_LED, LOW);
    //Serial.println(F("falha no save"));
    bluetooth_packet.check_sd = 0;
  }
  Serial.println("Into the function");
  Serial.print("bluetooth_packet.check_sd --> ");
  Serial.println(bluetooth_packet.check_sd);
}

String packetToString(bool err)
{
  mqtt_packet_t SD_data = update_packet();
  
  String dataString = "";
    if(err)
    {
      dataString += "ACCX";
      dataString += ",";
      dataString += "ACCY";
      dataString += ",";
      dataString += "ACCZ";
      dataString += ",";
      dataString += "DPSX";
      dataString += ",";
      dataString += "DPSY";
      dataString += ",";
      dataString += "DPSZ";
      dataString += ",";
      dataString += "ROLL";
      dataString += ",";
      dataString += "PITCH";
      dataString += ",";

      dataString += "RPM";
      dataString += ",";
      dataString += "VEL";
      dataString += ",";
      dataString += "TEMP_MOTOR";
      dataString += ",";
      dataString += "SOC";
      dataString += ",";
      dataString += "TEMP_CVT";
      dataString += ",";
      //dataString += "FUEL_LEVEL";
      //dataString += ",";
      dataString += "VOLT";
      dataString += ",";
      dataString += "CURRENT";
      dataString += ",";
      dataString += "FLAGS";
      dataString += ",";
      dataString += "LATITUDE";
      dataString += ",";
      dataString += "LONGITUDE";
      dataString += ",";
      dataString += "TIMESTAMP";
    }
    
    else
    {
      // imu
      dataString += String((SD_data.imu_acc.acc_x*0.061)/1000);
      dataString += ",";
      dataString += String((SD_data.imu_acc.acc_y*0.061)/1000);
      dataString += ",";
      dataString += String((SD_data.imu_acc.acc_z*0.061)/1000);
      dataString += ",";
      dataString += String(SD_data.imu_dps.dps_x);
      dataString += ",";
      dataString += String(SD_data.imu_dps.dps_y);
      dataString += ",";
      dataString += String(SD_data.imu_dps.dps_z);
      dataString += ",";
      dataString += String(SD_data.Angle.Roll);
      dataString += ",";
      dataString += String(SD_data.Angle.Pitch);

      dataString += ",";
      dataString += String(SD_data.rpm);
      dataString += ",";
      dataString += String(SD_data.speed);
      dataString += ",";
      dataString += String(SD_data.temperature);
      dataString += ",";
      dataString += String(SD_data.SOC);
      dataString += ",";
      dataString += String(SD_data.cvt);
      dataString += ",";
      //dataString += String(SD_data.fuel);
      //dataString += ",";
      dataString += String(SD_data.volt);
      dataString += ",";
      dataString += String(SD_data.current);
      dataString += ",";
      dataString += String(SD_data.flags);
      dataString += ",";
      dataString += String(SD_data.latitude);
      dataString += ",";
      dataString += String(SD_data.longitude);
      dataString += ",";
      dataString += String(SD_data.timestamp);
    }

  return dataString;
}

void Check_SD_for_storage(bluetooth bluetooth_packet)
{
  if(saveFlag && mounted)
  {
    sdSave(false, bluetooth_packet);
    saveFlag = false;    
  }
}

/* Ticker routine */
Ticker ticker40Hz;

void setup_SD_ticker()
{ 
  ticker40Hz.attach(0.025f, ticker40HzISR);
}

void ticker40HzISR()
{
  saveFlag = true; // set to true when the ECU store the CAN datas
}