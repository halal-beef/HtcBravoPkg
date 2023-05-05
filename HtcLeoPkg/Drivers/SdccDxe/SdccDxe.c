/** @file
  MMC/SD Card driver for OMAP 35xx (SDIO not supported)

  This driver always produces a BlockIo protocol but it starts off with no Media
  present. A TimerCallBack detects when media is inserted or removed and after
  a media change event a call to BlockIo ReadBlocks/WriteBlocks will cause the
  media to be detected (or removed) and the BlockIo Media structure will get
  updated. No MMC/SD Card harward registers are updated until the first BlockIo
  ReadBlocks/WriteBlocks after media has been insterted (booting with a card
  plugged in counts as an insertion event).

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  **/

#include "SdccDxe.h"
#include <Library/gpio.h>

EFI_BLOCK_IO_MEDIA gMMCHSMedia = 
{
	SIGNATURE_32('e', 'm', 'm', 'c'),         // MediaId
	FALSE,                                    // RemovableMedia
	TRUE,                                     // MediaPresent
	FALSE,                                    // LogicalPartition
	FALSE,                                    // ReadOnly
	FALSE,                                    // WriteCaching
	512,                                      // BlockSize
	4,                                        // IoAlign
	0,                                        // Pad
	0                                         // LastBlock
};

typedef struct
{
	VENDOR_DEVICE_PATH  Mmc;
	EFI_DEVICE_PATH     End;
} MMCHS_DEVICE_PATH;

MMCHS_DEVICE_PATH gMmcHsDevicePath =
{
	{
		HARDWARE_DEVICE_PATH,
		HW_VENDOR_DP,
		(UINT8)(sizeof(VENDOR_DEVICE_PATH)),
		(UINT8)((sizeof(VENDOR_DEVICE_PATH)) >> 8),
		0xb615f1f5, 0x5088, 0x43cd, 0x80, 0x9c, 0xa1, 0x6e, 0x52, 0x48, 0x7d, 0x00
	},
	{
		END_DEVICE_PATH_TYPE,
		END_ENTIRE_DEVICE_PATH_SUBTYPE,
		{ 
			sizeof(EFI_DEVICE_PATH_PROTOCOL), 
			0 
		}
	}
};


/**

  Reset the Block Device.

  @param  This                 Indicates a pointer to the calling context.
  @param  ExtendedVerification Driver may perform diagnostics on reset.
  @retval EFI_SUCCESS          The device was reset.
  @retval EFI_DEVICE_ERROR     The device is not functioning properly and could
  not be reset.
  **/
EFI_STATUS
EFIAPI
MMCHSReset(
	IN EFI_BLOCK_IO_PROTOCOL          *This,
	IN BOOLEAN                        ExtendedVerification
)
{
	
	return EFI_SUCCESS;
}


/**

  Read BufferSize bytes from Lba into Buffer.
  @param  This       Indicates a pointer to the calling context.
  @param  MediaId    Id of the media, changes every time the media is replaced.
  @param  Lba        The starting Logical Block Address to read from
  @param  BufferSize Size of Buffer, must be a multiple of device block size.
  @param  Buffer     A pointer to the destination buffer for the data. The caller is
  responsible for either having implicit or explicit ownership of the buffer.


  @retval EFI_SUCCESS           The data was read correctly from the device.
  @retval EFI_DEVICE_ERROR      The device reported an error while performing the read.
  @retval EFI_NO_MEDIA          There is no media in the device.
  @retval EFI_MEDIA_CHANGED     The MediaId does not matched the current device.
  @retval EFI_BAD_BUFFER_SIZE   The Buffer was not a multiple of the block size of the device.
  @retval EFI_INVALID_PARAMETER The read request contains LBAs that are not valid,

  or the buffer is not on proper alignment.

  EFI_STATUS

  **/
EFI_STATUS
EFIAPI
MMCHSReadBlocks(
	IN EFI_BLOCK_IO_PROTOCOL          *This,
	IN UINT32                         MediaId,
	IN EFI_LBA                        Lba,
	IN UINTN                          BufferSize,
	OUT VOID                          *Buffer
)
{
	EFI_STATUS Status = EFI_SUCCESS;
	EFI_TPL    OldTpl;
	INT32      ret;

	OldTpl = gBS->RaiseTPL(TPL_NOTIFY);


	
	/*if(ret != MMC_BOOT_E_SUCCESS)
	{
		Status = EFI_DEVICE_ERROR;
	}*/

	gBS->RestoreTPL(OldTpl);
	
	return Status;
}


/**

  Write BufferSize bytes from Lba into Buffer.
  @param  This       Indicates a pointer to the calling context.
  @param  MediaId    The media ID that the write request is for.
  @param  Lba        The starting logical block address to be written. The caller is
  responsible for writing to only legitimate locations.
  @param  BufferSize Size of Buffer, must be a multiple of device block size.
  @param  Buffer     A pointer to the source buffer for the data.

  @retval EFI_SUCCESS           The data was written correctly to the device.
  @retval EFI_WRITE_PROTECTED   The device can not be written to.
  @retval EFI_DEVICE_ERROR      The device reported an error while performing the write.
  @retval EFI_NO_MEDIA          There is no media in the device.
  @retval EFI_MEDIA_CHNAGED     The MediaId does not matched the current device.
  @retval EFI_BAD_BUFFER_SIZE   The Buffer was not a multiple of the block size of the device.
  @retval EFI_INVALID_PARAMETER The write request contains LBAs that are not valid,
  or the buffer is not on proper alignment.

  **/
EFI_STATUS
EFIAPI
MMCHSWriteBlocks(
	IN EFI_BLOCK_IO_PROTOCOL          *This,
	IN UINT32                         MediaId,
	IN EFI_LBA                        Lba,
	IN UINTN                          BufferSize,
	IN VOID                           *Buffer
)
{
	EFI_STATUS Status = EFI_SUCCESS;
	EFI_TPL    OldTpl;
	INT32      ret;

	OldTpl = gBS->RaiseTPL(TPL_NOTIFY);

	/*ret = mmc_write(gMMCHSMedia.BlockSize * Lba, BufferSize, (UINT32 *)Buffer);
	
	if(ret != MMC_BOOT_E_SUCCESS)
	{
		Status = EFI_DEVICE_ERROR;
	}*/

	gBS->RestoreTPL(OldTpl);
	
	return Status;

}


/**

  Flush the Block Device.
  @param  This              Indicates a pointer to the calling context.
  @retval EFI_SUCCESS       All outstanding data was written to the device
  @retval EFI_DEVICE_ERROR  The device reported an error while writting back the data
  @retval EFI_NO_MEDIA      There is no media in the device.

  **/
EFI_STATUS
EFIAPI
MMCHSFlushBlocks(
	IN EFI_BLOCK_IO_PROTOCOL  *This
)
{
	return EFI_SUCCESS;
}


EFI_BLOCK_IO_PROTOCOL gBlockIo = {
	EFI_BLOCK_IO_INTERFACE_REVISION,   // Revision
	&gMMCHSMedia,                      // *Media
	MMCHSReset,                        // Reset
	MMCHSReadBlocks,                   // ReadBlocks
	MMCHSWriteBlocks,                  // WriteBlocks
	MMCHSFlushBlocks                   // FlushBlocks
};

static unsigned mmc_sdc_base[] =
{ 
	MSM_SDC1_BASE, 
	MSM_SDC2_BASE, 
	MSM_SDC3_BASE, 
	MSM_SDC4_BASE 
};

extern struct mmc_host mmc_host;
extern struct mmc_card mmc_card;

EFI_STATUS
EFIAPI
MMCHSInitialize(
	IN EFI_HANDLE         ImageHandle,
	IN EFI_SYSTEM_TABLE   *SystemTable
)
{
	EFI_STATUS  Status;
//!gpio_get(153)
	if (1) {
        //HTCLEO_GPIO_SD_STATUS = 153
        DEBUG((EFI_D_ERROR, "SD card inserted\n"));

		/* Trying Slot 1 first */
		DEBUG((EFI_D_ERROR, "MMCHSInitialize eMMC slot 1 init START!\n"));

		if (mmc_legacy_init())
		{

			DEBUG((EFI_D_ERROR, "MMCHSInitialize eMMC slot 1 init failed!\n"));

		}
		else
		{
			DEBUG((EFI_D_ERROR, "MMCHSInitialize eMMC slot 1 init ok!\n"));
		}
		
		//gMMCHSMedia.LastBlock = (UINT64)((mmc_card.capacity / 512) - 1);
		//DEBUG((EFI_D_ERROR, "Sdcc init DONE!\n"));

		{
			UINT32 blocksize;
			blocksize = 512;//mmc_get_device_blocksize();

			DEBUG((EFI_D_ERROR, "eMMC Block Size:%d\n", blocksize));

			VOID * Data;
			//mmc_data_t *data;

			Status = gBS->AllocatePool(EfiBootServicesData, (blocksize), &Data);

			if (EFI_ERROR(Status)) {
				DEBUG((EFI_D_ERROR, "test memory alloc failed!\n"));
				return Status;
			}

			int ret = 0;
			//ret = mmc_read(blocksize, (UINT32 *)data, blocksize);
			//ret = sdcc_read_data(mmc, cmd, data);//returns -8

			

			DEBUG((EFI_D_ERROR, "SdccDxe: Test block read\n"));

			/*ret = mmc_read_blocks(mmc, &Data, 512, 1);//struct mmc *mmc, void *dst, int start, int blkcnt
			
			if (ret != MMC_BOOT_E_SUCCESS)
			{
				DEBUG((EFI_D_ERROR, "mmc_read failed! ret = %d\n", ret));
				MicroSecondDelay(2000000);
				return EFI_DEVICE_ERROR;
			}

			DEBUG((EFI_D_ERROR, "mmc_read succeeded! ret = %d\n", ret));
			MicroSecondDelay(2000000);

			UINT8 * STR = (UINT8 *)Data;

			DEBUG((EFI_D_ERROR, "First 8 Bytes = %c%c%c%c%c%c%c%c\n", STR[0], STR[1], STR[2], STR[3], STR[4], STR[5], STR[6], STR[7]));
			MicroSecondDelay(2000000);*/

			Status = gBS->FreePool(Data);

			DEBUG((EFI_D_ERROR, "We should be good to go\n"));
		}


		//Publish BlockIO.
		DEBUG((EFI_D_ERROR, "Publish the BlockIO protocol\n"));
		MicroSecondDelay(2000000);
		Status = gBS->InstallMultipleProtocolInterfaces(
			&ImageHandle,
			&gEfiBlockIoProtocolGuid, &gBlockIo,
			&gEfiDevicePathProtocolGuid, &gMmcHsDevicePath,
			NULL
			);
		return Status;
	}
	else {

		DEBUG((EFI_D_ERROR, "SD card not inserted\n"));
		DEBUG((EFI_D_ERROR, "SD status : %x\n", gpio_get(153)));

		Status = 0;
		return Status;
	}
}
