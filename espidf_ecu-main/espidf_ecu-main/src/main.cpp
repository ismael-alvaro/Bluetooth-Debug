#include <Arduino.h>
/* CAN Libraries */
#include <CAN.h>
/* Libraries of SD and Conectivity state Machine */
#include <SD_state_machine.h>
#include <CON_state_machine.h>

String ok = "OK";
String erro = "ERRO";

TaskHandle_t SDlogging = NULL, ConectivityState = NULL;
bool _sd = false; // flag to check if SD module compile
uint8_t _sot = DISCONNECTED;

bluetooth bluetooth_packet;

/* States Machines */
void SdStateMachine(void *pvParameters);
void ConnStateMachine(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  pinConfig(); // Hardware and Interrupt Config

  /* CAN-BUS Initialize */
  if (!CAN_start_device())
  {
    bluetooth_packet.can_bus_init = 0;
  }
  else
  {
    bluetooth_packet.can_bus_init = 1;
  }

  send_can_bus_init(bluetooth_packet);

  /* Tasks */
  // This state machine is responsible for the Basic CAN logging
  xTaskCreatePinnedToCore(SdStateMachine, "SDStateMachine", 4096, NULL, 5, &SDlogging, 0);
  // This state machine is responsible for the GPRS connection
  xTaskCreatePinnedToCore(ConnStateMachine, "ConnectivityStateMachine", 4096, NULL, 5, &ConectivityState, 1);
}

void loop() { /**/ }

/* SD State Machine */
void SdStateMachine(void *pvParameters)
{
  _sd = start_SD_device();

  Serial.print("_sd -> "); Serial.println(_sd);

  if (_sd)
  {
    bluetooth_packet.sd_start = 1;
  }
  else
  {
    bluetooth_packet.sd_start = 0;
  }

  send_sd_start(bluetooth_packet);

  /* For synchronization between ECU and panel */
  Send_SOT_msg(_sot);

  while (1)
  {
    Check_SD_for_storage();

    send_check_sd(bluetooth_packet);

    vTaskDelay((_sd ? 1 : 100));
  }
}

/* Connectivity State Machine */
void ConnStateMachine(void *pvParameters)
{
  _sot = Initialize_GSM();

  if ((_sot & 0x04) == ERROR_CONECTION)
  { // enable the error bit
    Send_SOT_msg(_sot);
    vTaskDelay(DELAY_ERROR(_sot));
    bluetooth_packet.internet_modem = 0;
  }
  else
  {
    bluetooth_packet.internet_modem = 1;
  }

  send_internet_modem(bluetooth_packet);

  Send_SOT_msg(_sot);

  while (1)
  {
    if (!Check_mqtt_client_conection())
    {
      _sot == CONNECTED ? _sot = DISCONNECTED : 0; // disable online flag
      Send_SOT_msg(_sot);
      gsmReconnect(_sot);
      Send_SOT_msg(_sot);
      bluetooth_packet.mqtt_client_connection = 0;
    }
    else
    {
      bluetooth_packet.mqtt_client_connection = 1;
    }

    send_client_connection(bluetooth_packet);

    Send_msg_MQTT();

    vTaskDelay(1);
  }
}
