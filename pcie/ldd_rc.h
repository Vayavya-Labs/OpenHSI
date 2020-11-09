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

/** @file ldd_rc.h
  * 
  * @brief File captures required interface to access the PCIe host controller
  *  driver 
*/

#ifndef PCIE_LDD_H
#define PCIE_LDD_H

#include <stdlib.h>
#include "pcie_data_struct.h"
#include "pcie_lib.h"
#include "dwc_pcie_rc_yapphdr.h"
#include "dwc_pcie_rc_yheader.h"
#include "env.h"

typedef enum
{
	E_READ = 0,
	E_WRITE
} readWriteFlag;

/**
 * \brief Initialize RC 
 *
 * \param  *pdata             Pointer to RC private data structure of PCIe stack
 * \return 0 on success and -1 on failure
 */
status_t pcieRcInit(pcieRootcompDataHandle_t *pdata);

/**
 * \brief Configure RC
 *
 * \param  *pdata             Pointer to RC private data structure of PCIe stack

 * \param  pcieAddr           PCIe Address
 * \param  cpuAddr            CPU Address
 * \param  epCfgAddr          EP configuration Address
 * \param  rcCfgAddr          RC configuration Address
 * \param  addrSpaceSize      Address space size
 * \param  numRdDmaChnls      Number of read DMA channels
 * \param  numWrDmaChnls      Number of write DMA channels
 * \param  numObRegions       Number of outbound regions
 * \param  numIbRegions       Number of inbound regions
 * \return 0 on success -1 on failure
 */
status_t pcieRcCfg(pcieRootcompDataHandle_t *pdata, 
		unsigned long pcieAddr,
		unsigned long cpuAddr, 
		unsigned long epCfgAddr, 
		unsigned long rcCfgAddr,
		unsigned long addrSpaceSize, 
		unsigned char numRdDmaChnls, 
		unsigned char numWrDmaChnls,
		unsigned char numObRegions, 
		unsigned char numIbRegions);

/**
 * \brief Configure RC ATU inbound region 
 *
 * \param  *pdata    Pointer to RC private data structure of PCIe stack
 * \param  baseAddr  Base address         
 * \param  trgtAddr  Target address
 * \param  len       Length
 * \param  type      Type
 * \param  ibRgn     Inbound region
 * \param  barNum    BAR number
 * \return 0 on success and -1 on failure
 */
status_t pcieRcConfigAtuIbRegion(pcieRootcompDataHandle_t *pdata,
		unsigned long baseAddr, 
		unsigned long trgtAddr,
		unsigned long len,
		unsigned int type,
		unsigned int ibRgn,
		unsigned char barNum);

/**
 * \brief Configure RC ATU outbound region 
 *
 * \param  *pdata    Pointer to RC private data structure of PCIe stack
 * \param  baseAddr  Base address         
 * \param  trgtAddr  Target address
 * \param  len       Length
 * \param  type      Type
 * \param  obRgn     Outbound region
 * \param  barNum    BAR number
 * \return 0 on success and -1 on failure
 */
status_t pcieRcConfigAtuObRegion(pcieRootcompDataHandle_t *pdata,
		unsigned long baseAddr, 
		unsigned long trgtAddr,
		unsigned long len,
		unsigned int type,
		unsigned int obRgn);

/**
 * \brief read or write RC configuration area 
 *
 * \param  *pdata             Pointer to RC private data structure of PCIe stack
 * \param  regOffset          Offset to the register
 * \param  *buffer            output buffer into which data is read or whose content is used for write
 * \param  capId              Capability ID
 * \param  configSpaceSelect  Selection of configuration space
 * \param  accessSize         Access size
 * \param  rw                 read or write action specifier
 * \return 0 success and -1 on failure
 */
status_t pcieRcConfigReadWrite(pcieRootcompDataHandle_t *pdata, 
		unsigned int regOffset,
		unsigned int *buffer ,
		unsigned int capId,
		unsigned char configSpaceSelect,
		unsigned char accessSize,
		readWriteFlag rw);

/**
 * \brief Deinitialize RC and release relevant memory
 *
 * \param  *pdata   Pointer to RC private data structure of PCIe stack
 * \return 
 */
void pcieRcDeinit(pcieRootcompDataHandle_t *pdata);
/**
 * \brief  Deinitialize print API link
 *
 * \param  *pdata             Pointer to RC private data structure of PCIe stack
 * \return 
 */
void pcieRcDeinitprint(pcieRootcompDataHandle_t *pdata);

/**
 * \brief PCIe enumeration of entire fabric under RC 
 *
 * \param  *pdata             Pointer to RC private data structure of PCIe stack
 * \return 0 on success and -1 on failure
 */
status_t pcieRcEnumerate(pcieRootcompDataHandle_t *pdata);

/**
 * \brief Get EP memory details 
 *
 * \param  *pdata        Pointer to RC private data structure of PCIe stack
 * \param  vendorId      EP Vendor ID  
 * \param  pcieDeviceId  EP Device ID
 * \param  barNum        BAR number
 * \param  bus           Bus number
 * \param  dev           Device number
 * \param  fun           Function number
 * \param  *addr         Address pointer to EP memory
 * \param  *size         Pointer to size value
 * \return 0 on success and -1 on failure
 */
status_t pcieRcGetEpMemDetails(pcieRootcompDataHandle_t *pdata,
		unsigned short vendorId, 
		unsigned short pcieDeviceId,
		unsigned char barNum, 
		unsigned char bus, 
		unsigned char dev, 
		unsigned char fun,
		unsigned int *addr,
		unsigned int *size);

/**
 * \brief Allocate private data structures and get driver API pointers
 *
 * \param  **tmp_pdata        Pointer to RC private data structure of PCIe stack pointer
 * \return 0 on success and -1 on failure
 */
status_t pcieRcPreInit(pcieRootcompDataHandle_t **tmp_pdata);

/**
 * \brief Program RC configuration region
 *
 * \param  *pdata             Pointer to RC private data structure of PCIe stack
 * \return 0 on success and -1 on failure
 */
status_t pcieRcProgConfigRegion(pcieRootcompDataHandle_t *pdata);

/**
 * \brief Program prefetch IO limit 
 *
 * \param  *pdata             Pointer to RC private data structure of PCIe stack
 * \param  rcBarConfigReg     RC BAR configuration register
 * \param  type1BaseLmtCntrl  type 1 base lmt cntrol
 * \param  enableOrDisable    Enable or Disable
 * \return Value on success and ERRORINVAL on failure
 */
status_t pcieRcProgPrefetchIoLimit(pcieRootcompDataHandle_t *pdata,
		unsigned int rcBarConfigReg ,
		unsigned int type1BaseLmtCntrl,
		unsigned int enableOrDisable);

/**
 * \brief Read EP configuration area
 *
 * \param  *pdata      Pointer to RC private data structure of PCIe stack
 * \param  *buffer     Pointer to output buffer into which data will be read
 * \param  offset      Register offset
 * \param  size        size
 * \param  bus         Bus number
 * \param  pcieDevice  Device number
 * \param  function)   Function number
 * \return 0 on succesfull read and EINVAL on failure
 */
status_t pcieRcReadEpConfig(pcieRootcompDataHandle_t *pdata,
		unsigned int *buffer,
		unsigned short offset,
		unsigned char size,
		unsigned char bus,
		unsigned char pcieDevice,
		unsigned char function);

/**
 * \brief Read EP memory
 *
 * \param  *pdata      Pointer to RC private data structure of PCIe stack
 * \param  offset      Offset
 * \param  *dataPtr    Pointer to output data buffer
 * \param  *addrPtr    Pointer to memory address
 * \param  numOfBytes  Number of bytes to be read
 * \return 0 on success and -1 on failure
 */
status_t pcieRcReadEpMem(pcieRootcompDataHandle_t *pdata,
		unsigned int offset,
		unsigned char *dataPtr,
		unsigned int *addrPtr,
		unsigned int numOfBytes);

/**
 * \brief Write EP configuration area
 *
 * \param  *pdata      Pointer to RC private data structure of PCIe stack
 * \param  *buffer     Pointer to input buffer from which data will written
 * \param  offset      Register offset
 * \param  size        size
 * \param  bus         Bus number
 * \param  pcieDevice  Device number
 * \param  function)   Function number
 * \return 0 on succesfull read and EINVAL on failure
 */
status_t pcieRcWriteEpConfig(pcieRootcompDataHandle_t *pdata,		
		unsigned short offset,
		unsigned char size,
		unsigned char bus,
		unsigned char pcieDevice,
		unsigned char function,
		unsigned int value);

/**
 * \brief Write EP memory
 *
 * \param  *pdata      Pointer to RC private data structure of PCIe stack
 * \param  offset      Offset
 * \param  *dataPtr    Pointer to input data buffer
 * \param  *addrPtr    Pointer to memory address
 * \param  numOfBytes  Number of bytes to be written
 * \return 0 on success and -1 on failure
 */
status_t pcieRcWriteEpMem(pcieRootcompDataHandle_t *pdata,
		unsigned int offset,
		unsigned char *dataPtr,
		unsigned int *addrPtr,
		unsigned int numOfBytes);
//ADDED NEWLY
/**
 * \brief Configure ATU outbound TLP information 
 *
 * \param  *pdata           Pointer to RC private data structure of PCIe stack
 * \param  ob_rgn           Outbound region
 * \param  format           Format
 * \param  tlp_type         TLP type
 * \param  traffic_class    Traffic class
 * \param  is_id_order      Indicates if the ID is ordered ID
 * \param  tlp_hints        TLP hints
 * \param  tlp_digest       TLP digest
 * \param  poisoned_data    Indicates if data is poisoned data
 * \param  attr             Attribute
 * \param  address_type     Address type
 * \param  length           Length
 * \param  tag              Tag
 * \param  processing_hints Processing hints
 * \return 0 on success and -1 on failure
 */
status_t pcieRcAtuObCfgTlpInfo(pcieRootcompDataHandle_t *pdata, unsigned int ob_rgn,unsigned int format,
					unsigned int tlp_type, unsigned int traffic_class,
					unsigned int is_id_order, unsigned int tlp_hints, unsigned int tlp_digest,
					unsigned int poisoned_data, unsigned int attr, unsigned int address_type,
					unsigned int length, unsigned int tag, unsigned int processing_hints);
/**
 * \brief Register write
 *
 * \param  *pdata   Pointer to RC private data structure of PCIe stack
 * \param  size     Size
 * \param  offset   Offset address
 * \param  data     Data to be written
 * \return
 */
status_t pcieRcRegWrite (pcieRootcompDataHandle_t *pdata, uint32_t size, uint32_t offset, uint32_t data);
/**
 * \brief Register write
 *
 * \param  *pdata   Pointer to RC private data structure of PCIe stack
 * \param  size     Size
 * \param  offset   Offset address
 * \param  data     Data to be written
 * \return
 */
status_t pcieRcRegRead (pcieRootcompDataHandle_t *pdata, uint32_t size, uint32_t offset, void *data);

/**
 * \brief Alter the PCIe link size
 *
 * \param  *pdata         Pointer to RC private data structure of PCIe stack
 * \param  link_width     Target link width
 * \return 0 on success and -1 on failure
 */
status_t pcieRcResizeLink(pcieRootcompDataHandle_t *pdata, int link_width);

/**
 * \brief Tie off unused lanes 
 *
 * \param  *pdata        Pointer to RC private data structure of PCIe stack
 * \param  lanes         Lanes
 * \return 0 on success and -1 on failure
 */
status_t pcieRcUnusedTieOffLanes(pcieRootcompDataHandle_t *pdata, int lanes);

/**
 * \brief Change PCIe link speed 
 *
 * \param  *pdata             Pointer to RC private data structure of PCIe stack
 * \param  target_speed_req   Target link speed to be requested
 * \return 0 on success and -1 on failure
 */
status_t pcieRcSpeedChangeRequest(pcieRootcompDataHandle_t *pdata, unsigned int target_speed_req);


#endif //PCIE_LDD_H


