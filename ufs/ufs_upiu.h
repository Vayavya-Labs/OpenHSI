/*******************************************************************************
Copyright (c) 2020, Vayavya Labs Pvt. Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Vayavya Labs Pvt. Ltd. nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL VAYAVYA LABS PVT. LTD. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/


/** @file ufs_upiu.h
  * 
  * @brief  Header files lists datatype
  *
  */

#pragma once 


/* AllocAndClearMem & mem_free */ 
/* needs to be changed based on env */

#define NO_DMA_ALLOC 0x00
#define NO_ALIGN 0x00

/* SCSI Command Type */
#define SCSI_TYPE 0x00

/* UFS UPIU flag */
#define CMD_FLAG_NONE 0x0
#define CMD_FROM_HOST 0x20
#define CMD_FROM_DEVICE 0x40

/*Return Status*/
#define FAILURE -1

/*Flag value in NOP_IN resonse*/
#define OVERFLOW_FLAG 0x40
#define UNDERFLOW_FLAG 0x20
#define MISMATCH_ERROR_FLAG 0X10

/*Purge operation Status*/
#define UFS_PURGE_STATUS 0x6
#define UFS_PURGE_STATUS_IDLE 0X0
#define UFS_PURGE_STATUS_IN_PROGRESS 0X1
#define UFS_PURGE_STATUS_STOPPED_PREMATURELY 0X2
#define UFS_PURGE_STATUS_COMPLETED_SUCCESSFULLY 0X3
#define UFS_PURGE_STATUS_LOGI_UNIT_QUEUE_NOT_EMPTY 0X4
#define UFS_PURGE_STATUS_GENERAL_FAILURE 0X5

/* Unit descriptor offset of bprovision type */
#define B_PROVISION_TYPE 0x17

#define LOGICAL_BLK_SIZE 0x1000

/*UM attribute*/
#define UFS_MIN_UM_AREA_SIZE 0x12

/* \ enum ufs_query_type
 * UFS Protocol Information Units (UPIU) Query request type (opcode)
 * Reference 10.7.8 JEDEC 220B
 */
typedef enum ufs_query_type {
    UFS_QUERY_NOP       = 0x00,
    UFS_QUERY_READ_DESC     = 0x01,
    UFS_QUERY_WRITE_DESC    = 0x02,
    UFS_QUERY_READ_ATTR     = 0x03,
    UFS_QUERY_WRITE_ATTR    = 0x04,
    UFS_QUERY_READ_FLAG     = 0x05,
    UFS_QUERY_SET_FLAG  = 0x06,
    UFS_QUERY_CLEAR_FLAG    = 0x07,
    UFS_QUERY_TOGGLE_FLAG   = 0x08,
    UFS_QUERY_ERROR     = 0xFF
}ufs_query_type;

/* \enum ufs_query_response
 * UFS Protocol Information Units (UPIU) Query Response code
 * Reference 10.7.9.3 JEDEC 220C
 */
typedef enum ufs_query_response {
    UFS_QUERY_SUCCESS       = 0x00,
    UFS_PARAM_NOT_READABLE      = 0xF6,
    UFS_PARAM_NOT_WRITEABLE     = 0xF7,
    UFS_PARAM_ALREADY_WRITTEN   = 0xF8,
    UFS_INVALID_LENGTH      = 0xF9,
    UFS_INVALID_VALUE       = 0xFA,
    UFS_INVALID_SELECTOR        = 0xFB,
    UFS_INVALID_INDEX       = 0xFC,
    UFS_INVALID_IDN         = 0xFD,
    UFS_INVALID_OPCODE      = 0xFE,
    UFS_INVALID_FAILURE         = 0xFF
}ufs_query_response;

/* \enum ufs_query_function
 * UFS Protocol Information Units (UPIU) Query function
 * Reference 10.7.8 JEDEC 220B
 */
typedef enum ufs_query_function {
    UFS_RESERVED_QUERY_FUNCTION = 0,
    UFS_STANDARD_READ_REQUEST   = 0x1,
    UFS_STANDARD_WRITE_REQUEST  = 0x81
}ufs_query_function;

/* Flag ID - Flag idn for query requests p-362*/
typedef enum flags_id {
    FDEVICE_INIT     = 0x1,
    FPERMANANT_WPEN  = 0x2,
    FPOWERON_WPEN    = 0x3,
    FBG_OPSEN        = 0x4,
    FPURGE_OPSEN     = 0x6,
    FPHYRES_REMOVAL  = 0x8,
    FBUSY_RTC        = 0x9
}flags_id;

/* Descriptor Identification value - Descriptor idn for Query requests p-311*/
enum desc_idn {
    IDN_DEVICE        = 0x0,
    IDN_CONFIGURAION  = 0x1,
    IDN_UNIT          = 0x2,
    IDN_INTERCONNECT  = 0x4,
    IDN_STRING        = 0x5,
    IDN_GEOMETRY      = 0x7,
    IDN_POWER         = 0x8,
    IDN_DEVICE_HELTH  = 0X9,
};

/* \enum ufs_upiu_task_mgmt_request
 * UFS Task Management Service Request types
 * Reference Table 10.27 JEDEC 220C
 */
typedef enum ufs_upiu_task_mgmt_request {
    UFS_UPIU_TASK_MGMT_ABORT_TASK       = 0x1,
    UFS_UPIU_TASK_MGMT_ABORT_TASK_SET   = 0x2,
    UFS_UPIU_TASK_MGMT_CLEAR_TASK_SET   = 0x4,
    UFS_UPIU_TASK_MGMT_LUN_RESET        = 0x8,
    UFS_UPIU_TASK_MGMT_QUERY_TASK       = 0x80,
    UFS_UPIU_TASK_MGMT_QUERY_TASK_SET   = 0x81,
}ufs_upiu_task_mgmt_request;

/* \enum ufs_upiu_task_mgmt_response
 * UFS Task Management Service Response types
 * Reference Table 10.27 JEDEC 220C
 */
typedef enum ufs_upiu_task_mgmt_response {
    UFS_UPIU_TASK_MGMT_FUNCTION_COMPLETE    = 0x00,
    UFS_UPIU_TASK_MGMT_FUNCTION_NOT_SUPP    = 0x04,
    UFS_UPIU_TASK_MGMT_FUNCTION_FAILED  = 0x05,
    UFS_UPIU_TASK_MGMT_FUNCTION_SUCCEEDED   = 0x08,
    UFS_UPIU_TASK_MGMT_INCORRECT_LUN    = 0x09
}ufs_upiu_task_mgmt_response;

/* \enum ufs_resp_upiu_response
 * UFS Protocol Information Units (UPIU) Response UPIU's Response code
 * Table 10-8 — UTP Response Values JEDEC 220C
 */
typedef enum ufs_resp_upiu_response {
    UFS_TARGET_SUCCESS = 0x0,
    UFS_TARGET_FAILURE = 0x1,
}ufs_resp_upiu_response;


/* \enum ufs_scsi_command_opcode
 * UFS SCSI command opcode
 * Reference 12.3 JEDEC 220B
 */
enum ufs_scsi_command_opcode {
    UFS_SCSI_FORMAT_UNIT        = 0x04,
    UFS_SCSI_INQUIRY        = 0x12,
    UFS_SCSI_MODE_SELECT_10     = 0x55,
    UFS_SCSI_MODE_SENSE_10      = 0x5A,
    UFS_SCSI_PREFETCH_10        = 0x34,
    UFS_SCSI_READ_6         = 0x08,
    UFS_SCSI_READ_10        = 0x28,
    UFS_SCSI_READ_BUFFER        = 0x3C,
    UFS_SCSI_READ_CAPACITY_10   = 0x25,
    UFS_SCSI_REPORT_LUNS        = 0xA0,
    UFS_SCSI_REQ_SENSE      = 0x03,
    UFS_SCSI_SECURITY_PROTOCOL_IN   = 0xA2,
    UFS_SCSI_SECURITY_PROTOCOL_OUT  = 0xB5,
    UFS_SCSI_START_STOP         = 0x1B,
    UFS_SCSI_TEST_UNIT_READY    = 0x00,
    UFS_SCSI_WRITE_6        = 0x0A,
    UFS_SCSI_WRITE_10       = 0x2A,
    UFS_SCSI_WRITE_BUFFER       = 0x3B,

    UFS_SCSI_PREFETCH_16        = 0x90,
    UFS_SCSI_READ_16        = 0x88,
    UFS_SCSI_READ_CAPACITY_16   = 0x9E,
    UFS_SCSI_SEND_DIAGNOSTIC    = 0x1D,
    UFS_SCSI_SYNCHRONIZE_CACHE_10   = 0x35,
    UFS_SCSI_SYNCHRONIZE_CACHE_16   = 0x91,
    UFS_SCSI_UNMAP          = 0x42,
    UFS_SCSI_VERIFY_10      = 0x2F,
    UFS_SCSI_WRITE_16       = 0x8A,
    UFS_SCSI_COMMAND_OPCODE_ERROR   = 0xFF
};

/* \enum ufs_scsi_sense_key
 * UFS SCSI command status sense key
 * Reference Table 10-18 JEDEC 220C
 */
enum ufs_scsi_sense_key {
    UFS_SCSI_SENSE_NO_SENSE     = 0x00,
    UFS_SCSI_SENSE_RECOVERED_ERROR  = 0x01,
    UFS_SCSI_SENSE_NOT_READY    = 0x02,
    UFS_SCSI_SENSE_MEDIUM_ERROR     = 0x03,
    UFS_SCSI_SENSE_HARDWARE_ERROR   = 0x04,
    UFS_SCSI_SENSE_ILLEGAL_REQUEST  = 0x05,
    UFS_SCSI_SENSE_UNIT_ATTENTION   = 0x06,
    UFS_SCSI_SENSE_DATA_PROTECT     = 0x07,
    UFS_SCSI_SENSE_BLANK_CHECK  = 0x08,
    UFS_SCSI_SENSE_VENDOR_SPECIFIC  = 0x09,
    UFS_SCSI_SENSE_COPY_ABORTED     = 0x0A,
    UFS_SCSI_SENSE_ABORTED_COMMAND  = 0x0B,
    UFS_SCSI_SENSE_VOLUME_OVERFLOW  = 0x0D,
    UFS_SCSI_SENSE_MISCOMPARE   = 0x0E,
    UFS_SCSI_SENSE_ERROR        = 0xFF
};

/* \enum ufs_scsi_command_status
 * UFS SCSI command status values
 * Reference Table 10-15 JEDEC 220C
 */
enum ufs_scsi_command_status {
    UFS_SCSI_GOOD           = 0x00,
    UFS_SCSI_CHECK_CONDITION    = 0x02,
    UFS_SCSI_CONDITION_MET      = 0x04,     //Not Applicable
    UFS_SCSI_BUSY           = 0x08,
    UFS_SCSI_RESERVATION_CONFLICT   = 0x18,     //Optional
    UFS_SCSI_TASK_SET_FULL      = 0x28,
    UFS_SCSI_ACA_ACTIVE         = 0x30,     //Not Applicable
    UFS_SCSI_TASK_ABORTED       = 0x40,         //Not Applicable 
    UFS_SCSI_COMMAND_STATUS_ERROR   = 0xFF
};

/* enum ufs_upiu_type
 * UFS Protocol Information Units (UPIU) type (transaction code/type)
 * Reference 10.5 JEDEC 220B
 */
typedef enum ufs_upiu_type {
    UFS_UPIU_NOP_OUT    = 0x00,
    UFS_UPIU_NOP_IN     = 0x20,
    UFS_UPIU_COMMAND    = 0x01,
    UFS_UPIU_RESPONSE   = 0x21,
    UFS_UPIU_TASK_MGMT_REQ  = 0x04,
    UFS_UPIU_TASK_MGMT_RSP  = 0x24,
    UFS_UPIU_QUERY_REQ  = 0x16,
    UFS_UPIU_QUERY_RSP  = 0x36,
    UFS_UPIU_DATA_OUT   = 0x02,
    UFS_UPIU_DATA_IN    = 0x22,
    UFS_UPIU_RTT        = 0x31,
    UFS_UPIU_REJECT     = 0x3F,
    UFS_UPIU_TYPE_ERROR     = 0xFF
}ufs_upiu_type;


struct ufs_nop_upiu {
    enum ufs_upiu_type transaction_type;        // Type of request or response contained within the UPIU

    unsigned char flags;            // Flags
    unsigned char reserved_2;       // Reserved
    unsigned char task_tag;         // Unique Task Tag for series of transactions
    unsigned char reserved_4;       // Reserved
    unsigned char reserved_5;       // Reserved
    unsigned char response;         // Response of success or failure. Reserved in UPIU from Initiator to Target.
    unsigned char reserved_7;       // Reserved
    unsigned char ehs_length;       // Not used in this standard, always set to zero.
    unsigned char dev_info;         // Device level information by specific UFS functionality in RESPONSE UPIU.
    unsigned short data_seg_length;     // Number of valid bytes within the Data Segment of the UPIU
    unsigned int reserved_12_15;        // Reserved
    unsigned int reserved_16_19;        // Reserved
    unsigned int reserved_20_23;        // Reserved
    unsigned int reserved_24_27;        // Reserved
    unsigned int reserved_28_31;        // Reserved
};

/* \struct ufs_command_upiu
 * Command UFS Protocol Information Units (UPIU) representing a SCSI command service
 * Reference 10.7.1 JEDEC 220B
 */
struct ufs_command_upiu {
    enum ufs_upiu_type transaction_type;        // Type of request or response contained within the UPIU
    unsigned char flags;            // Flags
    unsigned char lun;          // Logical Unit Number to which request is targeted
    unsigned char task_tag;         // Unique Task Tag for series of transactions
    unsigned char command_set;      // Initiator ID and type of command that is in the CDB field
    unsigned char reserved_5;       // Reserved
    unsigned char reserved_6;       // Reserved
    unsigned char reserved_7;       // Reserved
    unsigned char ehs_length;       // Not used in this standard, always set to zero.
    unsigned char reserved_9;       // Reserved
    unsigned short data_seg_length;     // Number of valid bytes within the Data Segment of the UPIU

    unsigned int expected_data_length;  // Expected number of bytes to be transferred
    unsigned char cdb[16];      //cdb blocks is an array of 16 byte which holds the value of scsi commamds
};


/* \struct ufs_response_upiu
 * Response UFS Protocol Information Units (UPIU) indicates command and device level
 * status resulting from the successful or failed execution of a command
 * Reference 10.7.2 JEDEC 220B
 */
struct ufs_response_upiu {
    enum ufs_upiu_type transaction_type;        // Type of request or response contained within the UPIU
    unsigned char flags;            // Flags
    unsigned char lun;          // Logical Unit Number to which request is targeted
    unsigned char task_tag;         // Unique Task Tag for series of transactions
    unsigned char command_set;      // Initiator ID and type of command that is in the CDB field
    unsigned char reserved_5;       // RESERVED
    unsigned char response;         // Response of success or failure. Reserved in UPIU from Initiator to Target.
    unsigned char status;           // SCSI status or opcode specific status or reserved.
    unsigned char ehs_length;       // Not used in this standard, always set to zero.
    unsigned char dev_info;         // Device level information by specific UFS functionality in RESPONSE UPIU.
    unsigned short data_seg_length;     // Number of valid bytes within the Data Segment of the UPIU
    unsigned int residual_transfer_cnt; // Number of bytes not transferred because of data under/overflow. Reserved for rest.
    unsigned int reserved_16_19;        // RESERVED
    unsigned int reserved_20_23;        // RESERVED
    unsigned int reserved_24_27;        // RESERVED
    unsigned int reserved_28_31;        // RESERVED
    unsigned short sense_data_length;   // Number of valid Sense Data bytes that follow
    unsigned char validbit_response_code;   // Fixed format sense data response
    unsigned char reserved_sense_1;     // RESERVED
    unsigned char sense_key;        // General SCSI error code for previous command
    unsigned int reserved_sense_3_6;    // RESERVED
    unsigned char additional_sense_len; // Length in bytes of additional sense information
    unsigned int reserved_sense_8_11;   // RESERVED
    unsigned char asc;          // ADDITIONAL SENSE CODE
    unsigned char ascq;         // ADDITIONAL SENSE CODE QUALIFIER
    unsigned char fruc;         // FIELD REPLACEABLE UNIT CODE
    unsigned short reserved_sense_15_16;    // RESERVED
    unsigned char reserved_sense_17;    // RESERVED
};

/* \struct ufs_query_req_upiu
 * Query Request UFS Protocol Information Units (UPIU) to read and write parametric data
 * Reference 10.7.8 JEDEC 220B
 */
struct ufs_query_req_upiu {
    enum ufs_upiu_type transaction_type;        // Type of request or response contained within the UPIU
    unsigned char flags;            // Flags
    unsigned char reserved_2;       // Reserved
    unsigned char task_tag;         // Unique Task Tag for series of transactions
    unsigned char reserved_4;       // Reserved
    unsigned char function;         // Type of Query function 
    unsigned char reserved_6;       // Reserved
    unsigned char reserved_7;       // Reserved
    unsigned char ehs_length;       // Not used in this standard, always set to zero.
    unsigned char reserved_9;       // Reserved
    unsigned short data_seg_length;     // Number of valid bytes within the Data Segment of the UPIU
    enum ufs_query_type opcode;         // Query request type/opcode
    unsigned char identification;       // Identification for a particular descriptor/attribute/flag
    unsigned char index;            // Index for a particular descriptor/attribute
    unsigned char selector;         // Identification for a particular descriptor/attribute
    unsigned char osf_3;            // Reserved
    unsigned char osf_4;            // Reserved
    unsigned short length;          // Length of descriptor to read/write. Reserved for attributes/flags
    unsigned int value;         // Value to write to attribute. Reserved for rest.
    unsigned int osf_7;         // Reserved
    unsigned char data[8];          // Data to be written into descriptor in case of write descriptor
};

/* \struct ufs_query_rsp_upiu
 * Query Response UFS Protocol Information Units (UPIU) to return parametric data in
 * case of read descriptor/attribute/flag query request, or to provide response to
 * write flag descriptor/attribute query request or set/clear/toggle flag query request.
 * Reference 10.7.9 JEDEC 220B
 */
struct ufs_query_rsp_upiu {
    enum ufs_upiu_type transaction_type;        // Type of request or response contained within the UPIU
    unsigned char flags;            // Flags
    unsigned char reserved_2;       // Reserved
    unsigned char task_tag;         // Unique Task Tag for series of transactions
    unsigned char reserved_4;       // Reserved
    unsigned char function;         // Query function 
    unsigned char response;         // Query Response
    unsigned char reserved_7;       // Reserved
    unsigned char ehs_length;       // Not used in this standard, always set to zero.
    unsigned char dev_info;         // Device level information by specific UFS functionality in RESPONSE UPIU.
    unsigned short data_seg_length;     // Number of valid bytes within the Data Segment of the UPIU
    enum ufs_query_type opcode;         // Query request type/opcode
    unsigned char identification;       // Identification for a particular descriptor/attribute/flag
    unsigned char index;            // Index for a particular descriptor/attribute
    unsigned char selector;         // Identification for a particular descriptor/attribute
    unsigned char osf_3;            // Reserved
    unsigned char osf_4;            // Reserved
    unsigned short length;          // Length of descriptor to read/write. Reserved for attributes/flags
    unsigned int value;         // Value read/write from/to attribute. Value of Flag. Reserved for descriptors.
    unsigned int osf_7;         // Reserved
    unsigned char data[8];  // Data to be read from descriptor in case of read descriptor
};

/* \struct ufs_task_mgmt_req_upiu
 * Task Management Request UFS Protocol Information Units (UPIU) is used to manage
 * the execution of one or more tasks within the Target device
 * Reference 10.7.6 JEDEC 220B
 */
struct ufs_task_mgmt_req_upiu {
    enum ufs_upiu_type transaction_type;        // Type of request or response contained within the UPIU
    unsigned char flags;            // Flags
    unsigned char lun;          // Logical Unit Number to which request is targeted
    unsigned char task_tag;         // Unique Task Tag for series of transactions
    unsigned char iid;          // Initiator ID and type of command that is in the CDB field
    unsigned char function;         // Task management function
    unsigned char reserved_6;       // Reserved
    unsigned char reserved_7;       // Reserved
    unsigned char ehs_length;       // Not used in this standard, always set to zero.
    unsigned char reserved_9;       // Reserved
    unsigned short data_seg_length;     // Number of valid bytes within the Data Segment of the UPIU
    unsigned int input_param_1;         // LSB contains LUN of the logical unit
    unsigned int input_param_2;         // LSB Task Tag of the task/command
    unsigned int input_param_3;         // Bits [3:0] contain IID of the task/command
    unsigned int reserved_24_27;        // RESERVED
    unsigned int reserved_28_31;        // RESERVED
};

/* \struct ufs_task_mgmt_rsp_upiu
 * Task Management Response UFS Protocol Information Units (UPIU) - response to Task Management Request
 * Reference 10.7.7 JEDEC 220B
 */
struct ufs_task_mgmt_rsp_upiu {
    enum ufs_upiu_type transaction_type;        // Type of request or response contained within the UPIU
    unsigned char flags;            // Flags
    unsigned char lun;          // Logical Unit Number to which request is targeted
    unsigned char task_tag;         // Unique Task Tag for series of transactions
    unsigned char iid;          // Initiator ID and type of command that is in the CDB field
    unsigned char reserved_5;       // Reserved
    unsigned char response;         // Response of success or failure. Reserved in UPIU from Initiator to Target.
    unsigned char reserved_7;       // Reserved
    unsigned char ehs_length;       // Not used in this standard, always set to zero.
    unsigned char reserved_9;       // Reserved
    unsigned short data_seg_length;     // Number of valid bytes within the Data Segment of the UPIU
    unsigned int output_param_1;        // LSB contains Task Management Service Response
    unsigned int output_param_2;        // Reserved
    unsigned int reserved_20_23;        // RESERVED
    unsigned int reserved_24_27;        // RESERVED
    unsigned int reserved_28_31;        // RESERVED
};


/* \struct ufs_unmap_block_desc
 * Block descriptor for Prameter list of SCSI UNAMP Command
 * Reference 11.3.26.2 JEDEC 220B
 */
struct ufs_unmap_block_desc {
    unsigned long long unmap_block_addr; //Unmap block address
    unsigned int logical_block; // Number of logical blocks
    unsigned int reserved_12_15; // Reserved
};

/* \struct ufs_unmap_param_list
 * Prameter list for SCSI UNAMP Command 
 * Reference 11.3.26.1 JEDEC 220B
 */
struct ufs_unmap_param_list {
    unsigned short unmap_data_len; // unmap data length
    unsigned short unmap_block_desc_len; // unmap bock descriptor data length
    unsigned int reserved_4_7; // Reserved
    struct ufs_unmap_block_desc unmap_block_desc[64]; //array of struct ufs_unmap_block_desc
};

//RPMB Macros
#define RPMB_PROGRAM_KEY        0x1    /* Program RPMB Authentication Key */
#define RPMB_GET_WRITE_COUNTER  0x2    /* Read RPMB write counter */
#define RPMB_WRITE_DATA         0x3    /* Write data to RPMB partition */
#define RPMB_READ_DATA          0x4    /* Read data from RPMB partition */
#define RPMB_RESULT_READ        0x5    /* Read result request  (Internal) */

extern const unsigned char KEY[];
extern const unsigned char NONCE[];
#if 0
//RPMB Authentication Programming Key
const unsigned char KEY[] =
{
    0x42, 0xac, 0xe0, 0xf9, 0x7a, 0x7c,
    0x81, 0x7d, 0x1a, 0x96, 0x2d, 0xee,
    0xbf, 0xf7, 0xe, 0x19, 0xb4, 0x86,
    0x86, 0xd3, 0x23, 0x87, 0x40, 0xeb,
    0x2e, 0xa4, 0x2e, 0x84, 0x53, 0x19,
    0x82, 0xea
};

//RPMB Nonce 
const unsigned char NONCE[] =
{
    0x12, 0x1c, 0xe1, 0xe9, 0x6c, 0x9c,
    0x21, 0x1d, 0x3b, 0x32, 0x45, 0xef,
    0xbc, 0x17, 0x0e, 0x2a
};
#endif
//RPMB Enum
/* \enum rpmb_op_result - rpmb operation results*/

enum rpmb_result {
    RPMB_ERR_OK      = 0x0000,  /*operation successful*/
    RPMB_ERR_GENERAL = 0x0001,  /*general failure*/
    RPMB_ERR_AUTH    = 0x0002,  /*mac doesn't match or ac calculation failure*/
    RPMB_ERR_COUNTER = 0x0003,  /*counter value didn`t match*/
    RPMB_ERR_ADDRESS = 0x0004,  /*address out of range*/
    RPMB_ERR_WRITE   = 0x0005,  /*data,counter or result write failure*/
    RPMB_ERR_READ    = 0x0006,  /*data,counter or result read failure*/
    RPMB_ERR_NO_KEY  = 0x0007,  /*autherntication key not yet programmed*/
    RPMB_ERR_COUNTER_EXPIRED = 0x0080 /*Counter expired*/
};
    
