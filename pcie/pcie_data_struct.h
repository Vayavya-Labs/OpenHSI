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

#pragma once

/* PCIe Physical device driver ref */
#include <dwc_pcie_rc_yapphdr.h>

#include "stdint.h"

/* The values below have to be updated with the values appropriate
 * for the environment. */
#define PCIE_GIC400V2M_MSI_SETSPI_NS  0x40
#define PCIE_MSIX_PBA_ENTRY_SIZE      4

#define MSI_BASE_ADDR 0x82010000
#define MSI_SPI_START_NUM 0xA00DA
#define MSI_SPI_COUNT 0x1

#ifdef PLATFORM64
typedef uint64_t pcieRcAddr_t;
#else
typedef uint32_t pcieRcAddr_t;
#endif

typedef pcieRcAddr_t pcieRootcompBaseAddr_t;
typedef int status_t;

/* Access types */
typedef enum pcieAccessType {
	PCIE_TRANSLTED_ACCESS_NONE = 0,
	PCIE_TRANSLTED_ACCESS_WO,
	PCIE_TRANSLTED_ACCESS_RO,
	PCIE_TRANSLTED_ACCESS_RW
} pcieAccessType_t;

typedef enum pcieAddressWidth {
	addr32_e,
	addr64_e
} pcieAddressWidth_t;

/*
 * Page Request Service message that will be sent from endpoint to root
 * complex.
 * */
typedef struct pcieRcPrsPacket {
	/*
	 * Fmt=<31:29> Type=<28:24> Res3=<23> TC=<22:20> Res2=<19> Attr2=<18>
	 * Res2=<17:16> TD=<15> EP=<14> Attr1=<12:13> Res1=<11:10> Length=<9:0>
	 * */
	uint32_t dw0;
	/*
	 * reqId=<31:16> tag=<15:7> msgCode=<7:0>
	 * */
	uint32_t dw1;
	uint32_t dw2; /* Upper address 63:32 */
	/* PageAdderss=<31:12> PRGI=<11:3> L=<2> WR=<1:0>*/
	uint32_t dw3;
} pcieRcPrsPacket_t;

#define PCIE_GET_PRS_REQ_FMT(ptr) \
	((ptr->dw0 >> 29) & 0x7)

#define PCIE_GET_PRS_REQ_ID(ptr) \
	((ptr->dw1 >> 16) & 0xFFFF)

#define PCIE_GET_PRS_REQ_L(ptr) \
	((ptr->dw3 >> 2) & 0x1)

#define PCIE_GET_PRS_REQ_PRGI(ptr) \
	((ptr->dw3 >> 3) & 0x1FF)

#define PCIE_GET_PRS_REQ_ADDR_LOW(ptr) \
	((ptr->dw3 >> 20) & 0x1FF)


typedef struct pcieRcPrgResponsePacket {
	/*
	 * Fmt=<31:29> Type=<28:24> Res3=<23> TC=<22:20> Res2=<19> Attr2=<18>
	 * Res2=<17:16> TD=<15> EP=<14> Attr1=<12:13> Res1=<11:10> Length=<9:0>
	 * */
	uint32_t dw0;
	/*
	 * reqId=<31:16> tag=<15:7> msgCode=<7:0>
	 * */
	uint32_t dw1;
	/* destinationDevId=<31:16> responseCode=<15:12> res1=<11:9>
	 * pageRequestGroupIndex=<8:0>
	 * */
	uint32_t dw2;
	uint32_t dw3; /* Reserved */
} pcieRcPrgResponsePacket_t;

#define SET_PRG_RESPONSE_PRGI(ptr, prgi) \
	ptr->dw2 &= ~(0x1FF); \
ptr->dw2 |= (0x1FF & prgi)

#define SET_PRG_RESPONSE_RESPONSE_CODE(ptr, code) \
	ptr->dw2 &= ~(0xF); \
ptr->dw2 |= ((0xF & code) << 12)

#define SET_PRG_RESPONSE_DEST_ID(ptr, id) \
	ptr->dw2 &= ~(0xFF << 16); \
ptr->dw2 |= (id << 16)

/*
 * Response codes for Page Request Group Response
 * */
typedef enum pcieRcPrgResponseCode {
	pcieRcPrgReqSuccess = 0,
	pcieRcPrgReqInvalid,
	pcieRcPrgReqUnused_1,
	pcieRcPrgReqUnused_2,
	pcieRcPrgReqResponsfailEure
} pcieRcPrgResponseCode_t;

#define PREPARE_COMMON_PRG_RESPONSE_FIELDS(ptr) \
	ptr->dw0 &= ~(0x3FF);\
	ptr->dw0 &= ~(0x1F << 24);\
	ptr->dw0 != ~(0x12 << 24);\
	ptr->dw1 &= ~(0xFF);\
	ptr->dw1 |= 0x05

enum { NoInt_e, Legacy_e, MSI_e, MSIX_e };

#define BITS_PER_INTR 32
#define BIT_WORD(nr)  ((nr) / BITS_PER_INTR)
#define BITMAP_INITIAL_WORD_MASK(start) (~0UL << ((start) % BITS_PER_INTR))
#define BITMAP_WORD_MASK(nbits) \
	(\
	((nbits) % BITS_PER_INTR) ? \
	(1UL<<((nbits) % BITS_PER_INTR))-1 : ~0UL \
	)

/* translation regions */
typedef enum pcieRcRegionType {
	PCIE_RC_MEM_REGION = 0,
	PCIE_RC_MESSAGE_REGION = 0x10,
	PCIE_RC_IO_REGION = 2,
	PCIE_RC_CONFIG_REGION
} pcieRcRegionType_t;

/** BAR Aperture Coding */
typedef enum BarApertureSize {
	APERTURE_SIZE_4K = 5 /* BAR Aperture size of 4K */
} pcie_BarApertureSize_t;

typedef enum pcieCfgDataSize {
	UNIT_8_SZ = 1,
	UNIT_16_SZ = 2,
	UNIT_32_SZ = 4,
	UNIT_64_SZ = 8,
} pcieCfgDataSize_t;

/* Structure for configuring BAR address
*/
typedef struct pcieBarResourcesInfo {
	uint64_t pcieAddr;    /* To track holes in PCIe addr space */
	uint32_t baseAddrHdw; /* Translated BAR address high */
	uint32_t baseAddrLdw; /* Translated BAR address low */
	unsigned long size;   /* Size of BAR */
} pcieBarResourcesInfo_t;

struct pcieDev;

typedef struct msixEntries {
	uint32_t vector;
	uint16_t entry;
} pcieMsixEntries_t;

struct pcieMsiMsg {
	uint32_t addrLo;  /* low 32 bits of msi message address */
	uint32_t addrHi;  /* high 32 bits of msi message address */
	uint32_t data;    /* 16 bits of msi message data */
};

struct pcieMsiInfo {
	struct {
		uint8_t maxMsgs : 3;   /* log2 number of messages */
		uint8_t addrIs_64 : 1; /* Address size: 0 = 32bit 1 = 64bit */
		uint8_t maskBit : 1;   /* mask-pending bit supported ? */
		uint8_t isMsix : 1;
		uint16_t entryNo;      /* The entry for this vector */
	} pcieMsiAttrib;
	uint8_t pos;             /* Location of the msi capability */
	uint16_t defaultIrq;
	uint16_t irq;
	uint8_t maskPos;         /* position masking - depends on whether
														* addressing is 64 bit or 32 bit. */
	uint32_t masked;          /* mask bits */
	struct pcieMsiMsg msg;            
	pcieRcAddr_t msixTblBase; /* MSIX vector table base addr */
	struct pcieMsiInfo *next; /* For MSIX when we will have more than on
						 * IRQ */
	struct pcieDev *pcieDevice;
};

struct pcieMsiDesc {
	uint32_t nvecUsed;            /* number of messages */
	struct pcieMsiInfo *info;
};

enum {noHpuEvent_e = 0, hotPlug_e = 33, hotUnplug_e = 34};
enum {enumeration_e, reEnum_e};
enum {pluggedIn_e, unplugged_e};
enum {pass_e, fail_e};

typedef struct pcieCfg {
	uint16_t pcieVendId;
	uint16_t pcieDevId;
	uint32_t pcieDevVendId;/* Device and Vendor ID of IP, hash table key */
	struct pcieBarResourcesInfo *pcieRegBaseAddr;
	uint8_t pcieNumOfBars;
	uint8_t pcieIsRootcomp;
	/* BDF info */
	uint8_t pcieBusNum;
	uint8_t pcieDevNum;
	uint8_t pcieFunNum;
} pcieCfg_t;

typedef enum {
	PCIE_BAR_PROP_32_MEM_NOPREFETCH = 0,
	PCIE_BAR_PROP_32_IO,
	PCIE_BAR_PROP_32_MEM_PREFETCH,
	PCIE_BAR_PROP_64_MEM_NOPREFETCH,
	PCIE_BAR_PROP_64_IO,
	PCIE_BAR_PROP_64_MEM_PREFETCH,
	BAR_DISABLE,
} pcieBarProperty_t;

/*
 * This structure exposes configurable fields of PCIE capability structure to
 * user
 * */
typedef struct pcieCap {
	uint32_t pcieDevCtl;    /* Offset 0x8*/
	uint32_t pcieLinkCtl;   /* Offset 0x10 */
	uint32_t pcieSlotCtl;   /* Offset 0x18 */
	uint32_t pcieRootCtl;   /* Offset 0x1C */
	uint32_t pcieDevCtrl_2; /* Offset 0x28 */
	uint32_t pcieLinkCtrl_2;/* Offset 0x30 */
	uint32_t pcieSlotCtrl_2;/* Offset 0x38 */
} pcieCap_t;

typedef struct pcieRcAddrSizePair {
	unsigned long addr; /* AXI wrapper address */
	unsigned long size; /* Size of AXI address space */
} pcieRcAddrSizePair_t;

/* Inbound address translation properties */
typedef struct pcieRcInboundReg {
	unsigned long addr;       /* Base PCIE address */
	unsigned long size;       /* Address space size */
	pcieRcRegionType_t type;  /* Region type */
	uint8_t barNum;           /* Bar Number */
} pcieRcInboundReg_t;

/* outbound address translation properties */
typedef struct pcieRcOutboundReg {
	unsigned long addr;      /* Translated PCIE address */
	unsigned long size;      /* Address space size */
	pcieRcRegionType_t type; /* Region type */
} pcieRcOutboundReg_t;

#define PCIE_NUM_OB_AXI_TRNSLTN_REGNS 4
#define PCIE_NUM_RP_IB_TRNSLTN_REGNS  2
#define PCIE_MSI_MSIX_SUPPORT_EN      1
#define PCIE_PRE_CONFIG_EP_MEM_SIZE   4096

/* 
 * structure for passing translation region settings
 * */
typedef struct rcIpAddrXlateParams {
	pcieRcOutboundReg_t obReg[PCIE_NUM_OB_AXI_TRNSLTN_REGNS];
	pcieRcAddrSizePair_t axiReg[PCIE_NUM_OB_AXI_TRNSLTN_REGNS];
	pcieRcAddrSizePair_t ibReg[PCIE_NUM_RP_IB_TRNSLTN_REGNS];
	pcieRcInboundReg_t ibRegProp[PCIE_NUM_RP_IB_TRNSLTN_REGNS];
	uint8_t msgRegion;
	uint8_t numObRegs;
	uint8_t numIbRegs;
} pcieRcIpAddrXlateParams_t;

/*
 * This structure exposes configurable parameters of root complex IP.
 * Typically configurable IP, will have address translation configurations,
 * root complex configuration space tuning and setting of link parameters.
 * */
typedef struct rcIpConfigParams {
	pcieRcIpAddrXlateParams_t *rcAddrXlate; 
	pcieRcAddr_t rcLinkSettingsCfgSpaceTuningBase;
	pcieRcAddr_t addrXlationBase;/* Base addr of address translation registers*/
	pcieRcAddr_t rcCfgBase;      /* Base addr RC configuration space */
	pcieRcAddr_t dmaBase;        /* Base addr of RC DMA register */
} rcIpCfgParms_t;

/*
 * BAR property, address and size
 * */
typedef struct barProperties {
	uint32_t addr;
	unsigned long size;
	pcieBarProperty_t barProp;
} barProperties_t;

typedef struct pcieRcCfgParm {
	unsigned long barPoolStart;       /* PCIe address space start */
	unsigned long barPoolStartCpuAddr;/* CPU address space start which will be
																				translated to PCIe address space */
	unsigned long barPoolSz;   /* PCIe address space range */
	uint8_t numRdDmaChnls;     /* Number of Read DMA channels */
	uint8_t numWrDmaChnls;     /* Number of  Write DMA channels */
	uint32_t * rpBase;         /* Root port controller base address */
	barProperties_t barProp[2];/* Root port BAR properties */
	pcieCap_t *pcieCapConfig;  /* PCIe express capability structure */
	unsigned long pcieAxiAddr; /* PCIe address for EP configuration access */
	unsigned long *pcieRcRegCfgAddr;/*PCIe address for RC reg/configuration access*/
	uint8_t rcIntrMask;        /* RC interrupt mask */
	uint8_t numObRegions;      /* Number of Outbound Regions */
	uint8_t numIbRegions;      /* Number of Inbound Regions */
	rcIpCfgParms_t *ipParams;  /*IP specific configuration struct rcIpCfgParms_t*/
} pcieRootcompConfigHandle_t;

/* MSIX resource info for new implementation */
typedef struct msixInfo {
	uint16_t irqNum;
	uint32_t data;
	uint32_t addrLow;
	uint32_t addrHigh;
} msixInfo_t;

typedef struct pcieDev {
	void *cfg;    /* configuration of this pcieDevice, pcieCfg_t */
	pcieRootcompConfigHandle_t *pcieRcCfg;/* HW specific info and function
															pointers for various protocol specific APIs */
	uint32_t enabledIntType; /*type of interrupt enabled by driver:
															legacy, msi or msix */
	struct pcieDev *originalPtr; /* used to keep track of the original pointer in
										* the tree when a request for pcieDevice context is made. */
	uint8_t inUse; /* Indicates whether this pcieDevice instance is advertised */
	uint8_t pcieCap; /* offset of pcieCap in capability list */
	uint8_t msiCap;  /* offset of msiCap in capability list */
	uint8_t msixCap; /* offset of msixCap in capability list */
	uint8_t pmCap;   /* offset of pmCap in capability list */
	uint16_t aerCap; /* Offset to AER capability in extended PCIe capability */
	uint32_t vendorSpecCap; /* Offset to Vendor Specific Extended Capability */
#ifdef ENBL_AER_HANDLING
	pcieChannelState_t errorState;
	pcieAerHdrLogRegs_t tlp; /* TLP Header */
#endif /* ENBL_AER_HANDLING */

#ifdef PCIE_MSI_MSIX_SUPPORT_EN
	struct pcieMsiDesc desc; /* Information related to MSI/MSI-X */
	uint8_t numMsiIntrs;     /* Number of MSI IRQs allocated by user */
	uint8_t *msiIrqsList;    /* List of MSI IRQs assigned by system */
	msixInfo_t *msixInfoList;
	uint16_t numMsixVecs;
#endif /* PCIE_MSI_MSIX_SUPPORT_EN */
} pcieEndpointHandle_t;

typedef struct hashElement {
	pcieEndpointHandle_t *pcieDevHandle;
	uint8_t pluggedState;
	uint16_t instance;
	void *next;             /* Pointer to next pcieDevice */
} hashElement_t;

/* PCIe Message type: Message Code << 8 | Message Routing << 5 */
typedef enum {
	/* Power Management */
	PM_ACTV_STT_NACK_e = 0x1480,
	PM_PME_e = 0x1800,
	PM_TURN_OFF_e = 0x1960,
} pcieMsgType_t;

typedef struct pcieMsg {
	uint32_t format;        /* Fmt field of header */
	uint32_t messageType;  /* Type field of header */
	uint32_t trafficClass; /* TC field of header */
	uint32_t isIdOrder;   /* Attr field of byte1 of header */
	uint32_t tlpHints;     /* TH field of header */
	uint32_t tlpDigest;    /* TD field of header */
	uint32_t poisonedData; /* EP field of header */
	uint32_t attr;          /* ATTR field of byte2 of header */
	uint32_t addressType;  /* AT field of header */
	uint32_t length;        /* Length field of header */
	uint32_t reqId;        /* Requester ID field of header */
	uint32_t tag;           /* TAG field of header */
	uint32_t messageCode;  /* msgCode field of header */
	uint32_t addrHigh;     /* higher address field */
	uint32_t addrLow;      /* lower address field */
	uint64_t data;    /* msg data  */
	uint32_t routingMethod;/* routing subfield of message type */ 
} pcieMsg_t;

enum { root_e, bus_e, dev_e, fun_e };

typedef enum { MM_e, IO_e } axsType_t;

struct pcieFabTree;
typedef struct pcieFabTree pcieFabricTree_t;

typedef struct{
	pcieFabricTree_t *sib;
	pcieFabricTree_t *chld;
} pcieLink_t;

struct pcieFabTree {
	pcieEndpointHandle_t *pcieDevice;
	union {
		uint8_t bus;
		uint8_t dev;
		uint8_t fun;
	} info;
	uint8_t nodeType;
	char* indent;
	struct pcieFabTree *bus;
	union {
		pcieLink_t bus;
		pcieLink_t dev;
		pcieLink_t fun;
	} next;
	struct pcieFabTree *nextRp;  /* For use when we have multiple RCs */
};

typedef struct pcieHotPuInfo {
	pcieEndpointHandle_t *pcieDevice;
	pcieRootcompBaseAddr_t pcieAddr;
	pcieRootcompBaseAddr_t cpuAddr;
	uint8_t bus;
	uint8_t dev;
	uint8_t fun;
	uint32_t size;
	struct pcieHotPuInfo *next;
} pcieSlotsInfo_t;

typedef struct pcieHotPuInfoWrap {
	uint8_t mpss;
	pcieSlotsInfo_t *start;
	pcieSlotsInfo_t *end;
} pcieSlots_t;

#define INVALID_VAL ((pcieSlots_t *)~0L)

/** Type 1 access for RP */
typedef enum
{
	PCIE_TYPE1_DISBLD_ALL = 0,          /* Disabled */
	PCIE_TYPE1_PF_32_BIT_IO_DISBLD = 1, /* 32 bit prefetchable, io disabled */
	PCIE_TYPE1_PF_64_BIT_IO_DISBLD = 2, /* 64 bit prefetchable, io disabled */
	PCIE_TYPE1_PF_DISBLD_IO_16_BIT = 3, /* disabled prefetchable, 16 bit io */
	PCIE_TYPE1_PF_32_BIT_IO_16_BIT = 4, /* 32 bit prefetchable, 16 bit io */
	PCIE_TYPE1_PF_64_BIT_IO_16_BIT = 5, /* 64 bit prefetchable, 16 bit io */
	PCIE_TYPE1_PF_DISBLD_IO_32_BIT = 6, /* disabled prefetchable, 32 bit io */
	PCIE_TYPE1_PF_32_BIT_IO_32_BIT = 7, /* 32 bit prefetchable, 32 bit io */
	PCIE_TYPE1_PF_64_BIT_IO_32_BIT = 8, /* 64 bit prefetchable, 32 bit io */
} pcieType1Config_t;

/** Enable or Disable */
typedef enum
{
	PCIE_DIS_PARM = 0, /* Disable the associated parameter */
	PCIE_EN_PARM = 1,  /* Enable the associated parameter */
} pcieEnOrDis_t;

typedef struct {
	pcieSlots_t *pInfo;  /* wrapper for list of empty pluggable slots*/
	pcieSlots_t *uInfo;  /* wrapper for list of un-pluggable slots*/
	pcieSlotsInfo_t *curEvent;
	uint8_t pcieEventType;
} pcihotPlug_e_t;

typedef struct {
	pcieRootcompBaseAddr_t *pcieRcBaseAddr; /* RC base address */
	pcieRootcompConfigHandle_t *pcieRcCfg;  /* Root complex configuration structure */
	pcieEndpointHandle_t *pcieDevice;       /* Endpoint pcieDevice handle */
	uint8_t pcieReenumerationStatus;        /* enumeration or re-enumeration */
	uint8_t pcieGetMemDetailStatus;
	pcihotPlug_e_t *hotPlug;
	struct dwc_pcie_rc_prv_data *prv_data;
} pcieRootcompDataHandle_t;

/* TLP Packet types */
typedef enum pcieRequestTypes {
	pcieAtsRequest = 0,       /* Macro for ATS request */
	pcieAtsResponseHeader,    /* Macro for ATS response header */
	pcieAtsResponsePayload_32,/* Macro for ATS response 32 bit payload */
	pcieAtsResponsePayload_64 /* Macro for ATS response 64 bit payload */
} pcieRequestTypes_t;

/* ATS response completion statue enumeration */
#define PCIE_ATS_RESPONSE_SUCCESS   0x0
#define PCIE_ATS_RESPONSE_UR        0x1
#define PCIE_ATS_RESPONSE_CRS       0x2
#define PCIE_ATS_RESPONSE_CA        0x4

#define PCIE_ATS_STU_SIZE 4096

#define PCIE_ATS_STU_DW_SIZE 1024

#define PCIE_ATS_SINGLE_COMPLETION_PACKET 1 /* To indicate single STU support */

#ifndef PCIE_ATS_SINGLE_COMPLETION_PACKET
#error "PCIE_ATS: More than single translation request are not supported\n"
#endif

/* pcieAtsRequest will be used to get ATS request details for generic 64bit
 * format.
 * Depending on FMT bit we will prepare response packet for 32bit/64bit
 * */
typedef struct pcieAtsRequestPacket {
	/* Res6<31> Fmt<29:30> Type<24:28> Res5<23:23> TC<20:22> Res4<19>  Res3<18>
	 * Res2<16:17> TD<15:15> EP<14:14> Res1<12:13> AT <10:11> LENGTH <0:9> */
	uint32_t dw_0;
	uint32_t dw_1; /* reqId<16:31> Tag<8:15> Last DW BE <4:7> 1st DW <0:3>*/
	uint32_t dw_2; /* untranslatedAddressUpper <0:31> */
	uint32_t dw_3; /* untranslatedAddressLower <12:31> Res7<1:11> NW<0:0> */
} pcieAtsRequestPacket_t;

#define PCIE_ATS_REQ_ID_GET(ptr) ((ptr->dw_1 & 0xffff0000) >> 16)
#define PCIE_ATS_REQ_FMT_GET(ptr) ((ptr->dw_0 & 0x60000000) >> 30)  
#define PCIE_ATS_REQ_LENGTH_GET(ptr) ((ptr->dw_0 & 0x000003ff))
#define PCIE_ATS_REQ_TC_GET(ptr) ((ptr->dw_0 & 0x00700000) >> 20)
#define PCIE_ATS_REQ_UNTRANSLATED_ADDR_GET(ptr) (ptr->dw_3) 

/* ATS response structure that will be populated for sending to endpoint */
typedef struct pcieAtsResponse {
	/* Res7=<31:31> FMT=<29:30> TYPE=<24:28> Res6=<23:23> TC=<20:22>
	 * Res5=<19:19> Res4=<18:18> Res3=<17:16> TD=<15:15> EP=<14:14> Res2=<12:13>
	 * Res1=<11:10> Length=<9:0> */
	uint32_t dw0;
	/* completerId=<31:16> complStatus=<15:13>  bcm=<12:12>
	 * byteCount=<0:11> */
	uint32_t dw1;
	uint32_t dw2; /* reqId=<31:16> tag=<15:8> Res8=<7:7> lowAddr=<6:0>*/
} pcieAtsResposePacket_t;

#define PCIE_ATS_RESPONSE_TC_SET(ptr, val) \
	ptr->dw0 &= ~(0x00700000);\
ptr->dw0 |= val << 20

#define PCIE_ATS_RESPONSE_LOW_ADDR_SET(ptr, val) \
	ptr->dw2 &= ~(0x0000007f);\
ptr->dw2 |= val << 0

#define PCIE_ATS_RESPONSE_COMPLETER_ID_SET(ptr, val) \
	ptr->dw1 &= ~(0xffff0000);\
ptr->dw1 |= val << 16 

#define PCIE_ATS_RESPONSE_CMPL_STATUS_SET(ptr, val) \
	ptr->dw1 &= ~(0x0000e000);\
ptr->dw1 |= val << 13 

#define PCIE_ATS_RESPONSE_BYTE_COUNT_SET(ptr, val) \
	ptr->dw1 &= ~(0x00000fff);\
ptr->dw1 |= val << 0

#define PCIE_ATS_RESPONSE_LENGTH_SET(ptr, val) \
	ptr->dw0 &= ~(0x000003ff);\
ptr->dw0 |= val << 0

#define PCIE_ATS_RESPONSE_FMT_SET(ptr, val) \
	ptr->dw0 &= ~(0x60000000);\
ptr->dw0 |= val << 29 

#define PCIE_PREPARE_COMMON_ATS_RESPONSE_FIELDS(ptr, fun_req_id)\
	ptr->dw0 &= ~(0xf1000000);\
ptr->dw0 |= (0xa << 24);\
ptr->dw1 &= ~(0xffff << 16);\
ptr->dw1 |= funReqId << 16

/* ATS response data. This is packet is relevant for 64bit. But however, we
 * will make use of the same for 32bit using FMT bit in pcieAtsResponse
 * */
typedef struct pcieAtsResponseData {
	uint32_t dw0; /* translatedAddress [63:32] */
	/* 
	 * translatedAddress[31:12]=<31:12> S=<11:11> N=<10:10> Res1=<9:3> U=<2:2>
	 * W=<1:1> R=<0:0>*/
	uint32_t dw1;
} pcieAtsResponseData_t;

#define PCIE_ATS_RESPONSE_DATA_S_32_SET(ptr, val)\
	ptr->dw1 &= ~(0x00000800);\
ptr->dw1 |= val << 11 

#define PCIE_ATS_RESPONSE_DATA_N_32_SET(ptr, val)\
	ptr->dw1 &= ~(0x00000400);\
ptr->dw1 |= val << 10 

#define PCIE_ATS_RESPONSE_DATA_U_32_SET(ptr, val)\
	ptr->dw1 &= ~(0x00000004);\
ptr->dw1 |= val << 2 

#define PCIE_ATS_RESPONSE_DATA_W_32_SET(ptr, val)\
	ptr->dw1 &= ~(0x00000002);\
ptr->dw1 |= val << 1 

#define PCIE_ATS_RESPONSE_DATA_R_32_SET(ptr, val)\
	ptr->dw1 &= ~(0x00000001);\
ptr->dw1 |= val

#define PCIE_ATS_RESPONSE_DATA_ADDR_32_SET(ptr, val)\
	ptr->dw1 &= ~(0xfffff000);\
ptr->dw1 |= val << 12 

#define HTBL_SIZE 3

#define ERRORINVAL (-1)

#define PCIE_DEV_TYPE_UPSTREAM_PORT_OF_PCIE_SWITCH 0x5
#define PCIE_DEV_TYPE_DOWNSTREAM_PORT_OF_PCIE_SWITCH 0x6
#define ONE_MB_ALIGN 0xFFFFF
#define SET_BME_MSE_ISE 7

/* Message Signalled Interrupts registers */
#define PCIE_MSI_FLAGS         2      /* Message Control */
#define PCIE_MSI_FLAGS_EN      0x0001 /* MSI feature enabled */
#define PCIE_MSI_FLAGS_QMASK   0x000e /* Maximum queue size available */
#define PCIE_MSI_FLAGS_QSIZE   0x0070 /* Message queue size configured */
#define PCIE_MSI_FLAGS_64BIT   0x0080 /* 64-bit addresses allowed */
#define PCIE_MSI_FLAGS_MASKBIT 0x0100 /* Per-vector masking capable */
#define PCIE_MSI_RFU           3      /* Rest of capability flags */
#define PCIE_MSI_ADDRESS_LO    4      /* Lower 32 bits */
#define PCIE_MSI_ADDRESS_HI    8 /*Upper 32 bits (if PCI_MSI_FLAGS_64BIT set)*/
#define PCIE_MSI_DATA_32       8 /*16 bits of data for 32-bit pcie_devices */
#define PCIE_MSI_MASK_32       12/*Mask bits register for 32-bit pcie_devices*/
#define PCIE_MSI_PENDING_32    16/*Pending intrs for 32-bit pcie_devices */
#define PCIE_MSI_DATA_64       12/*16 bits of data for 64-bit pcie_devices */
#define PCIE_MSI_MASK_64       16/*Mask bits register for 64-bit pcie_devices*/
#define PCIE_MSI_PENDING_64    20/* Pending intrs for 64-bit pcie_devices */

/* Meassage Signalled Interrupt multiple message enable constants */
#define PCIE_NUM_MSG_REQUESTED_1    0
#define PCIE_NUM_MSG_REQUESTED_2    1
#define PCIE_NUM_MSG_REQUESTED_4    2
#define PCIE_NUM_MSG_REQUESTED_8    3
#define PCIE_NUM_MSG_REQUESTED_16   4
#define PCIE_NUM_MSG_REQUESTED_32   5

/* MSI-X registers */
#define PCIE_MSIX_FLAGS         2          /* Message Control */
#define PCIE_MSIX_FLAGS_QSIZE   0x07FF     /* Table size */
#define PCIE_MSIX_FLAGS_MASKALL 0x4000     /* Mask all vectors for this function */
#define PCIE_MSIX_FLAGS_EN      0x8000     /* MSI-X enable */
#define PCIE_MSIX_TABLE         4          /* Table offset */
#define PCIE_MSIX_TABLE_BIR     0x00000007 /* BAR index */
#define PCIE_MSIX_TABLE_OFFSET  0xfffffff8 /* Offset into specified BAR */
#define PCIE_MSIX_PBA           8          /* Pending Bit Array offset */
#define PCIE_MSIX_PBA_BIR       0x00000007 /* BAR index */
#define PCIE_MSIX_PBA_OFFSET    0xfffffff8 /* Offset into specified BAR */
#define PCIE_CAP_MSIX_SIZEOF    12         /* size of MSIX registers */

/* MSI-X Table entry format */
#define PCIE_MSIX_ENTRY_SIZE         16
#define PCIE_MSIX_ENTRY_LOWER_ADDR   0
#define PCIE_MSIX_ENTRY_UPPER_ADDR   4
#define PCIE_MSIX_ENTRY_DATA         8
#define PCIE_MSIX_ENTRY_VECTOR_CTRL  12
#define PCIE_MSIX_ENTRY_CTRL_MASKBIT 1

#define PCI_EXP_DEV_CAP                   0x04
#define PCI_EXP_DEV_CONTROL_STATUS        0x08
#define PCI_EXP_LINK_CAPS                 0x0C
#define PCI_EXP_LINK_CONTROL_AND_STATUS   0x10
#define PCI_EXP_SLOT_CAPS                 0x14
#define PCI_EXP_SLOT_CONTROL_STATUS       0x18
#define PCI_EXP_ROOT_CONTROL_CAP          0x1C
#define PCI_EXP_ROOT_STATUS               0x20
#define PCI_EXP_DEV_CAP_2                 0x24
#define PCI_EXP_DEV_CONTROL_STATUS_2      0x28
#define PCI_EXP_LINK_CAPS_2               0x2C
#define PCI_EXP_LINK_CONTROL_AND_STATUS_2 0x30
#define PCI_EXP_SLOT_CAP_2                0x34
#define PCI_EXP_SLOT_CONTROL_STATUS_2     0x38

#define PCIE_SLOT_IMPLEMENTED_MASK        0x100
#define PCIE_SLOT_IMPLEMENTED_SHIFT       0x8
#define PCIE_HOT_PLUG_SLOT                0x128
#define PCIE_HOT_PLUG_CAPABLE_MASK        0x40
#define PCIE_PRESENCE_DETECT_CHANGED      0x80000
#define PCIE_PRESENCE_DETECT              0x400000

#define DMA_ALLOC	0
#define NO_DMA_ALLOC	1
#define  NO_ALIGN 0

