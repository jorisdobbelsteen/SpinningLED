//
// Created by Joris on 07/01/2024.
//

#ifndef AXIS_COMMUNICATION_PACKET_H
#define AXIS_COMMUNICATION_PACKET_H

#include <stdint.h>

#define AXIS_PACKET_MAX_PACKET_LENGTH 1368
#define AXIS_PACKET_MAX_PAYLOAD_LENGTH (AXIS_PACKET_MAX_PACKET_LENGTH - sizeof(axis_packet_header))

// only C23 or C++11 allow specifying the type here
typedef enum axis_packet_type_t {
  AXIS_PACKET_TYPE_DUMMY            = 0,
  // Server to Client
  AXIS_PACKET_TYPE_INFO             = 0x1,
  // Client to Server
  AXIS_PACKET_TYPE_SET_PROGRAM      = 0x801,
  AXIS_PACKET_TYPE_PIXEL_DATA_START = 0x4000,
  AXIS_PACKET_TYPE_PIXEL_DATA_END   = 0x7FFF,
} axis_packet_type_t;

typedef enum axis_program_t {
  AXIS_PROGRAM_HUE = 1,
  AXIS_PROGRAM_RGB = 2,
  AXIS_PROGRAM_INTERLACE = 3,
  AXIS_PROGRAM_IMAGE_RGB565 = 16,
} axis_program_t;

typedef struct axis_packet_header {
  uint16_t type;
  uint16_t payload_length; // size in bytes excluding the header
} axis_packet_header;

typedef struct axis_info_data {
  uint16_t width;
  uint16_t height;
  uint16_t current_program;
} axis_info_data;

typedef struct axis_set_program_data {
  uint16_t program;
} axis_set_program_data;

typedef struct axis_packet {
  struct axis_packet_header header;
  union {
    struct axis_info_data info_data;
    struct axis_set_program_data program_set_data;
    uint16_t pixel_data[AXIS_PACKET_MAX_PAYLOAD_LENGTH];
  };
} axis_packet;

//#ifdef __cplusplus
//template<typename T>
//struct axis_packet_t {
//  axis_packet_header header;
//  T data;
//};
//#endif

// specialized definitions
typedef struct axis_info_packet {
  axis_packet_header header;
  struct axis_info_data info_data;
} axis_info_packet;

#endif //ESP32_C3_PACKET_H
