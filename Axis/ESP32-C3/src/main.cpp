#include <Arduino.h>
#include <ESPmDNS.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <WiFi.h>

#include "packet.h"

constexpr const char* Default_SSID = WIFI_SSID;
constexpr const char* Default_Password = WIFI_PASSWORD;
constexpr const char Default_Hostname[] = "SpinningLED-";
constexpr uint16_t port = 26425;

bool softAP = false;

constexpr uint8_t SPI_MISO_Pin = 4;
constexpr uint8_t SPI_MOSI_Pin = 0;
constexpr uint8_t SPI_SCK_Pin = 1;
constexpr uint8_t SPI_SS_Pin = 5;
// This should probably increase to 15 ~ 20 Mbps
// STM32F411 SPI should be able to handle about 24 Mbps (64 MHz core) to 30 Mbps (80 MHz core).
const SPISettings SPI_Settings(16000000, SPI_MSBFIRST, SPI_MODE0);

constexpr uint8_t LED_Pin = 7;
Adafruit_NeoPixel pixels(1, LED_Pin, NEO_GRB + NEO_KHZ800);

WiFiServer server;
WiFiClient client;

// Packets should be limited to some extend, but also large enough for SPI transmission
// Image size (frame) is about 400*114*2 = 91200 bytes = ~89 KB
//     lines  bytes
//     1       228
//     2       456
//     4       912
//     6      1368
//     8      1824 <-- this looks OK for SPI transmissions
// With 15 fps this is 1336 KB/s -> ~ 12 Mbit/sec (!!!)
// Adding overhead we are talking about 15 Mbit/sec
//
// TODO:
// Not sure whether to do this over TCP (more overhead and limits) or UDP (async, faster,
// lower overhead, but no reliable transmission). Can also combine both methods, with
// status going through TCP and streaming data through UDP.
//
union {
  axis_info_packet packet;
  char buffer[sizeof(axis_info_packet) + 8];
} device_data;
bool device_data_valid = false;
unsigned long device_data_last_update = 0;

constexpr size_t client_buffer_size = 2048;
union {
  uint8_t buffer[client_buffer_size];
  axis_packet packet;
} client_buffer;
size_t client_buffer_pos = 0;
static_assert(client_buffer_size >= AXIS_PACKET_MAX_PACKET_LENGTH);

//union {
//  uint8_t buffer[AXIS_PACKET_MAX_PACKET_LENGTH];
//  axis_packet packet;
//} spi_receive_buffer;

static void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

static void setLedColorWifi() {
  if (softAP)
    setLedColor(0, 165, 165);
  else
    setLedColor(0, 0, 255);
}

void setup() {
  Serial.begin(115200);

  pixels.setBrightness(30);
  setLedColor(255, 0, 0);

  SPI.begin(SPI_SCK_Pin, SPI_MISO_Pin, SPI_MOSI_Pin, SPI_SS_Pin);
  SPI.setHwCs(true);

  char hostname[sizeof(Default_Hostname) + 7] = { 0 };
  {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    sprintf(hostname, "%s%02X%02X%02X", Default_Hostname, mac[3], mac[4], mac[5]);
  }

  WiFi.setHostname(hostname);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setSleep(WIFI_PS_NONE); // improve latency by disabling power save
  WiFi.setAutoReconnect(true);
  setLedColor(165, 165, 0);
  WiFi.begin(Default_SSID, Default_Password);
  while(WiFi.status() != WL_CONNECTED && WiFi.status() != WL_CONNECT_FAILED) {
    if (WiFi.status() == WL_CONNECTION_LOST) {
      // Retry
      WiFi.begin(Default_SSID, Default_Password);
      Serial.print("r");
    } else {
      Serial.print(".");
    }
    delay(500);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECT_FAILED) {
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(hostname);
    softAP = true;
  }

  delay(1000); // Allow platformio serial monitor to attach
  Serial.println("SpinningLED ESP32-C3");
  Serial.print("Hostname: "); Serial.println(WiFi.getHostname());
  if (softAP) {
    Serial.print("WIFI SoftAP: "); Serial.println(hostname);
    Serial.print("IP: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.print("WIFI Network: "); Serial.println(Default_SSID);
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  }
  setLedColorWifi();

  MDNS.begin(hostname);
  MDNS.addService("spinningled", "tcp", port);

  server.begin(port);
}

bool isInfoPacket(const axis_packet_header& header, size_t packetsize) {
  return header.type == AXIS_PACKET_TYPE_INFO
         && header.payload_length >= sizeof(axis_info_data)
         && (size_t(header.payload_length) + sizeof(device_data.packet.header)) <= packetsize;
}

bool was_connected = false;

bool checkForDeviceData() {
  unsigned long now = micros();
  if ((now - device_data_last_update) > 2000000ul
    || (!device_data_valid && (now - device_data_last_update) > 100000ul)
  ) {
    // Every 2 seconds (or 100ms if not valid)
    memset(device_data.buffer, 0, sizeof(device_data));
    SPI.beginTransaction(SPI_Settings);
    SPI.transfer(device_data.buffer, sizeof(device_data));
    SPI.endTransaction();

    device_data_valid = isInfoPacket(device_data.packet.header, sizeof(device_data));
    device_data_last_update = now;
    if (!device_data_valid) {
      Serial.println("Invalid info packet via SPI");
    }

    return device_data_valid;
  } else {
    return false;
  }
}

void loop() {
  if (!client.connected()) {
    if (was_connected) {
      Serial.println("Disconnected");
      setLedColorWifi();
      was_connected = false;
    }
    client = server.available();
    client_buffer_pos = 0;
    if (client.fd() != -1 && client.connected()) {
      Serial.print("Connection from ");
      Serial.println(client.remoteIP());
      setLedColor(0, 255, 0);

      if (device_data_valid) {
        client.write(device_data.buffer, sizeof(device_data.packet.header) + device_data.packet.header.payload_length);
      }
      was_connected = true;
    } else {
      checkForDeviceData();
    }
  } else {
    // We are connected. Check for data and process this...
    int size = client.read(client_buffer.buffer + client_buffer_pos, client_buffer_size - client_buffer_pos);
    client_buffer_pos += size;

//    if (size != 0) {
//      Serial.print("Received ");
//      Serial.println(size);
//    }

    if (client_buffer_pos >= sizeof(client_buffer.packet.header)) {
      size_t packet_length = (sizeof(client_buffer.packet.header) + client_buffer.packet.header.payload_length);
      if (client_buffer.packet.header.payload_length >= AXIS_PACKET_MAX_PAYLOAD_LENGTH) {
        Serial.println("Disconnect due to too large framing");
        client.stop();
      } else if (client_buffer_pos >= packet_length) {
        // TODO: Should we extend the packet to a minimum size?
        //       We will not do this but just stick to the size.

        //Serial.println("sent");

        // If we received a packet, sent it over SPI
        SPI.beginTransaction(SPI_Settings);
        SPI.transfer(client_buffer.buffer, packet_length);
        SPI.endTransaction();

        if (client_buffer.packet.header.type != AXIS_PACKET_TYPE_DUMMY
          && client_buffer.packet.header.payload_length <= packet_length) {

          size_t received_packet_length = client_buffer.packet.header.payload_length + sizeof(client_buffer.packet.header);
          bool send_to_client = true;

          if (isInfoPacket(client_buffer.packet.header, packet_length)) {
            if (memcmp(client_buffer.buffer, device_data.buffer, received_packet_length) == 0) {
              memcpy(client_buffer.buffer, device_data.buffer, received_packet_length);
              send_to_client = false;
            }
          }

          // We received a full packet, so sent it to the client
          // That is, if that's useful to do...
          if (send_to_client) {
            client.write(client_buffer.buffer, received_packet_length);
            // Register packet has been exchanged
            device_data_last_update = micros();
          }
        }

        // Process next packet (in next loop invoke)
        client_buffer_pos -= packet_length;
        memmove(client_buffer.buffer, client_buffer.buffer + packet_length, client_buffer_pos);
      }
    } else {
      if (checkForDeviceData()) {
        // Sent it to the client...
        client.write(device_data.buffer, sizeof(device_data.packet.header) + device_data.packet.header.payload_length);
      }
    }
  }
}
