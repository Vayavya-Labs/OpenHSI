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

#include "lib_ep.h"

/**
 * \brief Allocate memory for device private data structure
 *
 * \param  **tmp_drv_data    Pointer to RC private data structure pointer of device 
 * \return 
 */

status_t pcieEpPreInitDev(struct pcie_dev_ep_prv_data **tmp_drv_data);

/**
 * \brief configure EP device
 *
 * \param  *drv_data            Pointer to RC private data structure of device 
 * \param  ep_base              EP base address
 * \param  pcie_axi_addr        AXI address
 * \param  num_of_funcs         Number of functions
 * \param  num_ob_regions       Number of outbound regions
 * \param  num_ib_regions       Number of inbound regions
 * \param  num_rd_dma_channels  Number of read dma channels
 * \param  num_wr_dma_channels  Number of write dma channels
 * \return 
 */
status_t pcieEpCfgDev(struct pcie_dev_ep_prv_data *drv_data,unsigned long ep_base,unsigned long pcie_axi_addr,
		unsigned int num_of_funcs,unsigned char num_ob_regions,unsigned char num_ib_regions,
		unsigned char num_rd_dma_channels,unsigned char num_wr_dma_channels );

/**
 * \brief configure device outbound region
 *
 * \param  *drv_data     Pointer to RC private data structure of device 
 * \param  base_addr  EP base address
 * \param  trgt_addr  Target address
 * \param  len        Length
 * \param  type       Type
 * \param  ob_rgn     Outbound region
 * \return 
 */
status_t pcieEpConfigObDev(struct pcie_dev_ep_prv_data *drv_data, unsigned long base_addr, 
		unsigned long trgt_addr,unsigned long len,unsigned int type,unsigned int ob_rgn);

/**
 * \brief EP device initialize
 * \param  *drv_data     Pointer to RC private data structure of device 
 * \return 
 */
status_t pcieEpInitDev(struct pcie_dev_ep_prv_data *drv_data);

/**
 * \brief configure PCIe stack ldd layer values
 *
 * \param  **pdata_pro          Pointer to EP private data structure of device
 * \param  **drv_data           Pointer to RC private data structure of device 
 * \param  ep_base              EP base address
 * \param  pcie_axi_addr        AXI address
 * \param  num_of_funcs         Number of functions
 * \param  num_rd_dma_channels  Number of read dma channels
 * \param  num_wr_dma_channels  Number of write dma channels
 * \param  base_addr            Base address
 * \param  trgt_addr            Target address
 * \param  len                  Length
 * \param  type                 Type
 * \param  atu_ob_base_addr     ATU outbound base address
 * \param  atu_ob_trgt_addr     ATU outbound target address
 * \param  atu_ob_len           ATU outbound length
 * \param  atu_ob_type          ATU outbound type
 * \return 
 */

int pcieEpInitSetUpLdd(struct ep_prv_data **pdata_pro, 
				struct pcie_dev_ep_prv_data **drv_data, 
				unsigned long ep_base,
				unsigned long pcie_axi_addr,
				unsigned int num_of_funcs,
				unsigned char num_rd_dma_channels,
				unsigned char num_wr_dma_channels,
				unsigned long base_addr,
				unsigned long trgt_addr,
				unsigned long len,
				unsigned int type,
				unsigned long atu_ob_base_addr,
				unsigned long atu_ob_trgt_addr,
				unsigned long atu_ob_len,
				unsigned int atu_ob_type);

/**
 * \brief Deinitialize device
 *
 * \param  *drv_data     Pointer to RC private data structure of device 
 * \return 
 */
status_t pcieEpDeinitDev(struct pcie_dev_ep_prv_data *drv_data);

/**
 * \brief Deinitialize memory allocated for device private data structures
 *
 * \param  *drv_data  Pointer to RC private data structure of device 
 * \param  *pdata     Pointer to the PCIe EP stack
 * \return 
 */
status_t pcieEpDeinitialize(struct pcie_dev_ep_prv_data *drv_data,struct ep_prv_data *pdata );

