/** @file

  Copyright (c) 2011 - 2020, Arm Limited. All rights reserved.<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#ifndef _PREPI_H_
#define _PREPI_H_

#include <PiPei.h>

#include <Library/PcdLib.h>
#include <Library/ArmLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/HobLib.h>
#include <Library/SerialPortLib.h>
#include <Library/ArmPlatformLib.h>

#include "iomap.h"
#include "display.h"

extern UINT64  mSystemMemoryEnd;

#define MSM_MDP_BASE1 0xAA200000
#define FB_ADDR 0x11A08000

#define MSM_MDP_BASE1 0xAA200000
#define LCDC_BASE     0xE0000

#define LCDC_PIXCLK_IN_PS 26
#define LCDC_FB_PHYS      0x16600000
#define LCDC_FB_BPP       16

#define BIT(x)  (1<<(x))
#define DMA_DSTC0G_8BITS (BIT(1)|BIT(0))
#define DMA_DSTC1B_8BITS (BIT(3)|BIT(2))
#define DMA_DSTC2R_8BITS (BIT(5)|BIT(4))
#define CLR_G 0x0
#define CLR_B 0x1
#define CLR_R 0x2
#define MDP_GET_PACK_PATTERN(a,x,y,z,bit) (((a)<<(bit*3))|((x)<<(bit*2))|((y)<<bit)|(z))
#define DMA_PACK_ALIGN_LSB 0
#define DMA_PACK_PATTERN_RGB					\
        (MDP_GET_PACK_PATTERN(0,CLR_R,CLR_G,CLR_B,2)<<8)
#define DMA_DITHER_EN                       BIT(24)
#define DMA_OUT_SEL_LCDC                    BIT(20)
#define DMA_IBUF_FORMAT_RGB565              BIT(25)

#define MDP_RGB_SIZE	((LCDC_FB_HEIGHT<<16) + LCDC_FB_WIDTH)
#define MDP_RGB_565_FORMAT (BIT(14) | (1<<9) | (0<<8) | (0<<6) | (1<<4) | (1<<2) | (2<<0))

#define LCDC_BASE     0xE0000

#define LCDC_PIXCLK_IN_PS 26
#define LCDC_FB_PHYS      0x16600000
#define LCDC_FB_BPP       16

#define BIT(x)  (1<<(x))
#define DMA_DSTC0G_8BITS (BIT(1)|BIT(0))
#define DMA_DSTC1B_8BITS (BIT(3)|BIT(2))
#define DMA_DSTC2R_8BITS (BIT(5)|BIT(4))
#define CLR_G 0x0
#define CLR_B 0x1
#define CLR_R 0x2
#define MDP_GET_PACK_PATTERN(a,x,y,z,bit) (((a)<<(bit*3))|((x)<<(bit*2))|((y)<<bit)|(z))
#define DMA_PACK_ALIGN_LSB 0
#define DMA_PACK_PATTERN_RGB					\
        (MDP_GET_PACK_PATTERN(0,CLR_R,CLR_G,CLR_B,2)<<8)
#define DMA_DITHER_EN                       BIT(24)
#define DMA_OUT_SEL_LCDC                    BIT(20)
#define DMA_IBUF_FORMAT_RGB565              BIT(25)

#define MDP_RGB_SIZE	((LCDC_FB_HEIGHT<<16) + LCDC_FB_WIDTH)
#define MDP_RGB_565_FORMAT (BIT(14) | (1<<9) | (0<<8) | (0<<6) | (1<<4) | (1<<2) | (2<<0))

RETURN_STATUS
EFIAPI
TimerConstructor (
  VOID
  );

VOID
PrePiMain (
  IN  UINTN   UefiMemoryBase,
  IN  UINTN   StacksBase,
  IN  UINT64  StartTimeStamp
  );

EFI_STATUS
EFIAPI
MemoryPeim (
  IN EFI_PHYSICAL_ADDRESS  UefiMemoryBase,
  IN UINT64                UefiMemorySize
  );

EFI_STATUS
EFIAPI
PlatformPeim (
  VOID
  );

VOID
PrimaryMain (
  IN  UINTN   UefiMemoryBase,
  IN  UINTN   StacksBase,
  IN  UINT64  StartTimeStamp
  );

VOID
SecondaryMain (
  IN  UINTN  MpId
  );

// Either implemented by PrePiLib or by MemoryInitPei
VOID
BuildMemoryTypeInformationHob (
  VOID
  );

EFI_STATUS
GetPlatformPpi (
  IN  EFI_GUID  *PpiGuid,
  OUT VOID      **Ppi
  );

// Initialize the Architecture specific controllers
VOID
ArchInitialize (
  VOID
  );

VOID
EFIAPI
ProcessLibraryConstructorList (
  VOID
  );

#endif /* _PREPI_H_ */
