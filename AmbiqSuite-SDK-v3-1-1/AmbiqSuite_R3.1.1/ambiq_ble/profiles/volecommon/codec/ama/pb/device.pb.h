//*****************************************************************************
//
//  device.pb.h
//! @file
//!
//! @brief Auto-generated (see below).
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk_3_1_1-10cda4b5e0 of the AmbiqSuite Development Package.
//
//*****************************************************************************
/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.1 at Fri Nov 09 16:58:28 2018. */

#ifndef PB_DEVICE_PB_H_INCLUDED
#define PB_DEVICE_PB_H_INCLUDED
#include <pb.h>

#include "common.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _ConnectionStatus
{
    ConnectionStatus_CONNECTION_STATUS_UNKNOWN = 0,
    ConnectionStatus_CONNECTION_STATUS_CONNECTED = 1,
    ConnectionStatus_CONNECTION_STATUS_DISCONNECTED = 2
} ConnectionStatus;
#define _ConnectionStatus_MIN ConnectionStatus_CONNECTION_STATUS_UNKNOWN
#define _ConnectionStatus_MAX ConnectionStatus_CONNECTION_STATUS_DISCONNECTED
#define _ConnectionStatus_ARRAYSIZE ((ConnectionStatus)(ConnectionStatus_CONNECTION_STATUS_DISCONNECTED + 1))

typedef enum _DevicePresence
{
    DevicePresence_DEVICE_PRESENCE_UNKNOWN = 0,
    DevicePresence_DEVICE_PRESENCE_ACTIVE = 1,
    DevicePresence_DEVICE_PRESENCE_INACTIVE = 2,
    DevicePresence_DEVICE_PRESENCE_ACCESSIBLE = 3
} DevicePresence;
#define _DevicePresence_MIN DevicePresence_DEVICE_PRESENCE_UNKNOWN
#define _DevicePresence_MAX DevicePresence_DEVICE_PRESENCE_ACCESSIBLE
#define _DevicePresence_ARRAYSIZE ((DevicePresence)(DevicePresence_DEVICE_PRESENCE_ACCESSIBLE + 1))

typedef enum _DeviceBattery_Status
{
    DeviceBattery_Status_UNKNOWN = 0,
    DeviceBattery_Status_CHARGING = 1,
    DeviceBattery_Status_DISCHARGING = 2,
    DeviceBattery_Status_FULL = 3
} DeviceBattery_Status;
#define _DeviceBattery_Status_MIN DeviceBattery_Status_UNKNOWN
#define _DeviceBattery_Status_MAX DeviceBattery_Status_FULL
#define _DeviceBattery_Status_ARRAYSIZE ((DeviceBattery_Status)(DeviceBattery_Status_FULL + 1))

/* Struct definitions */
typedef struct _GetDeviceConfiguration
{
    char dummy_field;
/* @@protoc_insertion_point(struct:GetDeviceConfiguration) */
} GetDeviceConfiguration;

typedef struct _StartSetup
{
    char dummy_field;
/* @@protoc_insertion_point(struct:StartSetup) */
} StartSetup;

typedef struct _UpdateDeviceInformation
{
    pb_callback_t name;
/* @@protoc_insertion_point(struct:UpdateDeviceInformation) */
} UpdateDeviceInformation;

typedef struct _CompleteSetup
{
    ErrorCode error_code;
/* @@protoc_insertion_point(struct:CompleteSetup) */
} CompleteSetup;

typedef struct _DeviceBattery
{
    uint32_t level;
    uint32_t scale;
    DeviceBattery_Status status;
/* @@protoc_insertion_point(struct:DeviceBattery) */
} DeviceBattery;

typedef struct _DeviceConfiguration
{
    bool needs_assistant_override;
    bool needs_setup;
/* @@protoc_insertion_point(struct:DeviceConfiguration) */
} DeviceConfiguration;

typedef struct _DeviceStatus
{
    ConnectionStatus link;
    ConnectionStatus nfmi;
    DevicePresence presence;
/* @@protoc_insertion_point(struct:DeviceStatus) */
} DeviceStatus;

typedef struct _GetDeviceInformation
{
    uint32_t device_id;
/* @@protoc_insertion_point(struct:GetDeviceInformation) */
} GetDeviceInformation;

typedef struct _OverrideAssistant
{
    ErrorCode error_code;
/* @@protoc_insertion_point(struct:OverrideAssistant) */
} OverrideAssistant;

typedef struct _DeviceInformation
{
    char serial_number[20];
    char name[16];
    pb_size_t supported_transports_count;
    Transport supported_transports[4];
    char device_type[14];
    uint32_t device_id;
    DeviceBattery battery;
    DeviceStatus status;
    uint32_t product_color;
/* @@protoc_insertion_point(struct:DeviceInformation) */
} DeviceInformation;

typedef struct _NotifyDeviceConfiguration
{
    DeviceConfiguration device_configuration;
/* @@protoc_insertion_point(struct:NotifyDeviceConfiguration) */
} NotifyDeviceConfiguration;

typedef struct _NotifyDeviceInformation
{
    DeviceInformation device_information;
/* @@protoc_insertion_point(struct:NotifyDeviceInformation) */
} NotifyDeviceInformation;

/* Default values for struct fields */

/* Initializer values for message structs */
#define DeviceBattery_init_default               {0, 0, _DeviceBattery_Status_MIN}
#define DeviceStatus_init_default                {_ConnectionStatus_MIN, _ConnectionStatus_MIN, _DevicePresence_MIN}
#define DeviceInformation_init_default           {"", "", 0, {_Transport_MIN, _Transport_MIN, _Transport_MIN, _Transport_MIN}, "", 0, DeviceBattery_init_default, DeviceStatus_init_default, 0}
#define GetDeviceInformation_init_default        {0}
#define DeviceConfiguration_init_default         {0, 0}
#define GetDeviceConfiguration_init_default      {0}
#define OverrideAssistant_init_default           {_ErrorCode_MIN}
#define StartSetup_init_default                  {0}
#define CompleteSetup_init_default               {_ErrorCode_MIN}
#define NotifyDeviceConfiguration_init_default   {DeviceConfiguration_init_default}
#define UpdateDeviceInformation_init_default     {{{NULL}, NULL}}
#define NotifyDeviceInformation_init_default     {DeviceInformation_init_default}
#define DeviceBattery_init_zero                  {0, 0, _DeviceBattery_Status_MIN}
#define DeviceStatus_init_zero                   {_ConnectionStatus_MIN, _ConnectionStatus_MIN, _DevicePresence_MIN}
#define DeviceInformation_init_zero              {"", "", 0, {_Transport_MIN, _Transport_MIN, _Transport_MIN, _Transport_MIN}, "", 0, DeviceBattery_init_zero, DeviceStatus_init_zero, 0}
#define GetDeviceInformation_init_zero           {0}
#define DeviceConfiguration_init_zero            {0, 0}
#define GetDeviceConfiguration_init_zero         {0}
#define OverrideAssistant_init_zero              {_ErrorCode_MIN}
#define StartSetup_init_zero                     {0}
#define CompleteSetup_init_zero                  {_ErrorCode_MIN}
#define NotifyDeviceConfiguration_init_zero      {DeviceConfiguration_init_zero}
#define UpdateDeviceInformation_init_zero        {{{NULL}, NULL}}
#define NotifyDeviceInformation_init_zero        {DeviceInformation_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define UpdateDeviceInformation_name_tag         1
#define CompleteSetup_error_code_tag             1
#define DeviceBattery_level_tag                  1
#define DeviceBattery_scale_tag                  2
#define DeviceBattery_status_tag                 3
#define DeviceConfiguration_needs_assistant_override_tag 1
#define DeviceConfiguration_needs_setup_tag      2
#define DeviceStatus_link_tag                    1
#define DeviceStatus_nfmi_tag                    2
#define DeviceStatus_presence_tag                3
#define GetDeviceInformation_device_id_tag       1
#define OverrideAssistant_error_code_tag         1
#define DeviceInformation_serial_number_tag      1
#define DeviceInformation_name_tag               2
#define DeviceInformation_supported_transports_tag 3
#define DeviceInformation_device_type_tag        4
#define DeviceInformation_device_id_tag          5
#define DeviceInformation_battery_tag            6
#define DeviceInformation_status_tag             7
#define DeviceInformation_product_color_tag      8
#define NotifyDeviceConfiguration_device_configuration_tag 1
#define NotifyDeviceInformation_device_information_tag 1

/* Struct field encoding specification for nanopb */
extern const pb_field_t DeviceBattery_fields[4];
extern const pb_field_t DeviceStatus_fields[4];
extern const pb_field_t DeviceInformation_fields[9];
extern const pb_field_t GetDeviceInformation_fields[2];
extern const pb_field_t DeviceConfiguration_fields[3];
extern const pb_field_t GetDeviceConfiguration_fields[1];
extern const pb_field_t OverrideAssistant_fields[2];
extern const pb_field_t StartSetup_fields[1];
extern const pb_field_t CompleteSetup_fields[2];
extern const pb_field_t NotifyDeviceConfiguration_fields[2];
extern const pb_field_t UpdateDeviceInformation_fields[2];
extern const pb_field_t NotifyDeviceInformation_fields[2];

/* Maximum encoded size of messages (where known) */
#define DeviceBattery_size                       14
#define DeviceStatus_size                        6
#define DeviceInformation_size                   100
#define GetDeviceInformation_size                6
#define DeviceConfiguration_size                 4
#define GetDeviceConfiguration_size              0
#define OverrideAssistant_size                   2
#define StartSetup_size                          0
#define CompleteSetup_size                       2
#define NotifyDeviceConfiguration_size           6
/* UpdateDeviceInformation_size depends on runtime parameters */
#define NotifyDeviceInformation_size             102

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define DEVICE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
