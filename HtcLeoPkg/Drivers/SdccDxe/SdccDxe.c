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

/*
 * Function: MmcReadInternal
 * Arg     : Data address on card, o/p buffer & data length
 * Return  : 0 on Success, non zero on failure
 * Flow    : Read data from the card to out
 */
STATIC UINT32 MmcReadInternal
(
    //BIO_INSTANCE *Instance, 
    UINT32 DataAddr, 
    UINT32 *Buf, 
    UINT32 DataLen
)
{
    UINT32 Ret = 0;
    UINT32 BlockSize = gMMCHSMedia.BlockSize;//Instance->BlockMedia.BlockSize;
    /* Operation block size. Will decrement until reaches 1 */
    UINT32 OpBlkSize = 64;
    UINT32 ReadSize;
    UINT8 *Sptr = (UINT8 *) Buf;

    ASSERT(!(DataAddr % BlockSize));
    ASSERT(!(DataLen % BlockSize));

    /*
    * DMA onto write back memory is unsafe/nonportable,
    * but callers to this routine normally provide
    * write back buffers. Invalidate cache
    * before read data from MMC.
    */
    WriteBackInvalidateDataCacheRange(Buf, DataLen);

    /* TODO: This function is aware of max data that can be
    * tranferred using sdhci adma mode, need to have a cleaner
    * implementation to keep this function independent of sdhci
    * limitations
    */

    /* Set size */
    ReadSize = BlockSize * OpBlkSize;//512*64

	DEBUG((EFI_D_ERROR, "ReadSize / BlockSize=%d\n", (ReadSize/BlockSize)));

read:
    while (DataLen > ReadSize) 
    {
        Ret = mmc_bread((DataAddr / BlockSize), (ReadSize / BlockSize), (VOID *) Sptr);
        if (Ret == 0)
        {
            // Attempt to reduce the block size
            if (OpBlkSize > 1)
            {
                // Decrement op size
                OpBlkSize = OpBlkSize / 2;
                ReadSize = BlockSize * OpBlkSize;
                goto read;
            }
            else
            {
                DEBUG((EFI_D_ERROR, "Failed Reading block @ %x\n",(UINTN) (DataAddr / BlockSize)));
                goto fail;
            }
        }
        Sptr += ReadSize;
        DataAddr += ReadSize;
        DataLen -= ReadSize;
    }

    if (DataLen)
    {
        Ret = mmc_bread((DataAddr / BlockSize), (DataLen / BlockSize), (VOID *) Sptr);
        if (Ret == 0)
        {
            // Attempt to reduce the block size
			DEBUG((EFI_D_ERROR, "Attempt to reduce the block size\n"));
            if (OpBlkSize > 1)
            {
                // Decrement op size
                OpBlkSize = OpBlkSize / 2;
                ReadSize = BlockSize * OpBlkSize;
                goto read;
            }
            else
            {
                DEBUG((EFI_D_ERROR, "Failed Reading block @ %x\n",(UINTN) (DataAddr / BlockSize)));
                goto fail;
            }
        }
    }

    return 1;
fail:
	DEBUG((EFI_D_ERROR, "MmcReadInternal: Fail!!\n"));
	mdelay(5000);
    return 0;
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
	UINTN      BlockSize;
	UINTN      ret;

    BlockSize = gMMCHSMedia.BlockSize;

    /*DEBUG((EFI_D_ERROR, "MMCHSReadBlocks: BufferSize =  0x%08lx\n", BufferSize));//%d
	DEBUG((EFI_D_ERROR, "MMCHSReadBlocks: BlockSize =  %d\n", BlockSize));//%d
    DEBUG((EFI_D_ERROR, "MMCHSReadBlocks: Lba =  %d\n", (UINT32) Lba));
	DEBUG((EFI_D_ERROR, "MMCHSReadBlocks: BlockCount =  0x%08lx\n", BufferSize / BlockSize));
    mdelay(3000);*/

	if (BufferSize % BlockSize != 0) 
    {
		DEBUG((EFI_D_ERROR, "MMCHSReadBlocks: BAD buffer!!!\n"));
    	mdelay(5000);
        return EFI_BAD_BUFFER_SIZE;
    }

	if (Buffer == NULL) 
    {
		DEBUG((EFI_D_ERROR, "MMCHSReadBlocks: Invalid parameter!!!\n"));
    	mdelay(5000);
        return EFI_INVALID_PARAMETER;
    }

	if (BufferSize == 0) 
    {
		DEBUG((EFI_D_ERROR, "MMCHSReadBlocks: BufferSize = 0\n"));
    	mdelay(5000);
        return EFI_SUCCESS;
    }

	OldTpl = gBS->RaiseTPL(TPL_NOTIFY);

	ret = MmcReadInternal((UINT64) Lba * 512, Buffer, BufferSize); //Instance deleted for now
	
	if (ret == 1)
        return EFI_SUCCESS;
    else
        DEBUG((EFI_D_ERROR, "MMCHSReadBlocks: Read error!\n"));
        mdelay(5000);
        return EFI_DEVICE_ERROR;

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

	/* TBD */
	
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

//extern struct mmc_host mmc_host;
//extern struct mmc_card mmc_card;

EFI_STATUS
EFIAPI
MMCHSInitialize(
	IN EFI_HANDLE         ImageHandle,
	IN EFI_SYSTEM_TABLE   *SystemTable
)
{
	EFI_STATUS  Status;

	if (!gpio_get(HTCLEO_GPIO_SD_STATUS)) {
        DEBUG((EFI_D_ERROR, "SD card inserted\n"));

		/* Trying Slot 1 first */
		if (mmc_legacy_init())
		{
			DEBUG((EFI_D_ERROR, "MMCHSInitialize eMMC slot 1 init failed!\n"));
		}
		else
		{
			DEBUG((EFI_D_ERROR, "MMCHSInitialize eMMC slot 1 init ok!\n"));
		}
		
		//gMMCHSMedia.LastBlock = (UINT64)((mmc_card.capacity / 512) - 1);

		UINT32 blocksize = 512; //mmc_get_device_blocksize();

		DEBUG((EFI_D_ERROR, "SD Block Size:%d\n", blocksize));

        UINT8 BlkDump[512];
		ZeroMem(BlkDump, 512);
		BOOLEAN FoundMbr = FALSE;
		for (UINTN i = 0; i <= MIN(512, 50); i++) //MIN(mBlkDesc.lba, 50)
		{
            int blk = mmc_bread(i, 1, &BlkDump);
            if (blk)
            {
                if (BlkDump[510] == 0x55 && BlkDump[511] == 0xAA)
                {
                    DEBUG((EFI_D_INFO, "MBR found at %d \n", i));
                    FoundMbr = TRUE;
                    break;
                }
                DEBUG((EFI_D_INFO, "MBR not found at %d \n", i));
                return EFI_DEVICE_ERROR; //Hack?
            }
		}    
        if (!FoundMbr)
        {
            DEBUG((EFI_D_ERROR, "(Protective) MBR not found \n"));
            CpuDeadLoop();
        }

		//Publish BlockIO.
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
