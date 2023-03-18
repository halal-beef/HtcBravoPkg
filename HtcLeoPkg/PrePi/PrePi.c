/** @file

  Copyright (c) 2011-2017, ARM Limited. All rights reserved.

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <PiPei.h>

#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugAgentLib.h>
#include <Library/IoLib.h>
#include <Library/PrePiLib.h>
#include <Library/PrintLib.h>
#include <Library/PrePiHobListPointerLib.h>
#include <Library/TimerLib.h>
#include <Library/PerformanceLib.h>

#include <Ppi/GuidedSectionExtraction.h>
#include <Ppi/ArmMpCoreInfo.h>
#include <Ppi/SecPerformance.h>

#include "PrePi.h"

UINT64  mSystemMemoryEnd = FixedPcdGet64 (PcdSystemMemoryBase) +
                           FixedPcdGet64 (PcdSystemMemorySize) - 1;

VOID
FbconSetup(VOID)
{
  UINTN Width = 480;//FixedPcdGet32();
  UINTN Height = 800;
  UINTN Bpp = 16;

	MmioWrite32(MSM_MDP_BASE1 + 0x90004, (Height << 16) | Width);
	MmioWrite32(MSM_MDP_BASE1 + 0x9000c, Height * Bpp / 8);
	MmioWrite32(MSM_MDP_BASE1 + 0x90010, 0);

	MmioWrite32(MSM_MDP_BASE1 + 0x90000, DMA_PACK_ALIGN_LSB|DMA_PACK_PATTERN_RGB|DMA_DITHER_EN|DMA_OUT_SEL_LCDC|
	       DMA_IBUF_FORMAT_RGB565|DMA_DSTC0G_8BITS|DMA_DSTC1B_8BITS|DMA_DSTC2R_8BITS);

	int hsync_period  = LCDC_HSYNC_PULSE_WIDTH_DCLK + LCDC_HSYNC_BACK_PORCH_DCLK + Width + LCDC_HSYNC_FRONT_PORCH_DCLK;
	int vsync_period  = (LCDC_VSYNC_PULSE_WIDTH_LINES + LCDC_VSYNC_BACK_PORCH_LINES + Height + LCDC_VSYNC_FRONT_PORCH_LINES) * hsync_period;
	int hsync_start_x = LCDC_HSYNC_PULSE_WIDTH_DCLK + LCDC_HSYNC_BACK_PORCH_DCLK;
	int hsync_end_x   = hsync_period - LCDC_HSYNC_FRONT_PORCH_DCLK - 1;
	int display_hctl  = (hsync_end_x << 16) | hsync_start_x;
	int display_vstart= (LCDC_VSYNC_PULSE_WIDTH_LINES + LCDC_VSYNC_BACK_PORCH_LINES) * hsync_period + LCDC_HSYNC_SKEW_DCLK;
	int display_vend  = vsync_period - (LCDC_VSYNC_FRONT_PORCH_LINES * hsync_period) + LCDC_HSYNC_SKEW_DCLK - 1;

	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x4, (hsync_period << 16) | LCDC_HSYNC_PULSE_WIDTH_DCLK);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x8, vsync_period);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0xc, LCDC_VSYNC_PULSE_WIDTH_LINES * hsync_period);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x10, display_hctl);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x14, display_vstart);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x18, display_vend);

	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x28, 0);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x2c, 0xff);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x30, LCDC_HSYNC_SKEW_DCLK);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x38, 0);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x1c, 0);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x20, 0);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x24, 0);
	MmioWrite32(MSM_MDP_BASE1 + LCDC_BASE + 0x0, 1);
}

VOID
UartInit(VOID)
{
  CHAR8                       Buffer[100];
  UINTN                       CharCount;

  /* Add flashlight for debugging */

  // Initialize the framebuffer
  MmioWrite32(MSM_MDP_BASE1 + 0x90008, FB_ADDR);
  //FbconSetup();

  // Initialize the Serial Port
  SerialPortInitialize ();
  CharCount = AsciiSPrint (
                Buffer,
                sizeof (Buffer),
                "UEFI firmware (version %s built at %a on %a)\n\r",
                (CHAR16 *)PcdGetPtr (PcdFirmwareVersionString),
                __TIME__,
                __DATE__
                );
  SerialPortWrite ((UINT8 *)Buffer, CharCount);
}

VOID
PrePiMain (
  IN  UINTN   UefiMemoryBase,
  IN  UINTN   StacksBase,
  IN  UINT64  StartTimeStamp
  )
{
  EFI_HOB_HANDOFF_INFO_TABLE  *HobList;
  EFI_STATUS                  Status;
  //CHAR8                       Buffer[100];
  //UINTN                       CharCount;
  UINTN                       StacksSize;
  FIRMWARE_SEC_PERFORMANCE    Performance;

  // Initialize the architecture specific bits
  ArchInitialize ();

  
/*
  DEBUG((
        EFI_D_INFO | EFI_D_LOAD,
        "UEFI Memory Base = 0x%p, Stack Base = 0x%p\n",
        UefiMemoryBase,
        StacksBase
    ));*/


  // Declare the PI/UEFI memory region
  HobList = HobConstructor (
              (VOID *)UefiMemoryBase,
              FixedPcdGet32 (PcdSystemMemoryUefiRegionSize),
              (VOID *)UefiMemoryBase,
              (VOID *)StacksBase // The top of the UEFI Memory is reserved for the stacks
              );
  PrePeiSetHobList (HobList);

  // Initialize MMU and Memory HOBs (Resource Descriptor HOBs)
  Status = MemoryPeim (UefiMemoryBase, FixedPcdGet32 (PcdSystemMemoryUefiRegionSize));
  ASSERT_EFI_ERROR (Status);

  // Create the Stacks HOB (reserve the memory for all stacks)
  if (ArmIsMpCore ()) {
    StacksSize = PcdGet32 (PcdCPUCorePrimaryStackSize) +
                 ((FixedPcdGet32 (PcdCoreCount) - 1) * FixedPcdGet32 (PcdCPUCoreSecondaryStackSize));
  } else {
    StacksSize = PcdGet32 (PcdCPUCorePrimaryStackSize);
  }

  BuildStackHob (StacksBase, StacksSize);

  // TODO: Call CpuPei as a library
  BuildCpuHob (ArmGetPhysicalAddressBits (), PcdGet8 (PcdPrePiCpuIoSize));

  // Store timer value logged at the beginning of firmware image execution
  //Performance.ResetEnd = GetTimeInNanoSecond (StartTimeStamp);

  // Build SEC Performance Data Hob
  BuildGuidDataHob (&gEfiFirmwarePerformanceGuid, &Performance, sizeof (Performance));

  // Set the Boot Mode
  SetBootMode (ArmPlatformGetBootMode ());

  // Initialize Platform HOBs (CpuHob and FvHob)
  Status = PlatformPeim ();
  ASSERT_EFI_ERROR (Status);

  UartInit();

  // Now, the HOB List has been initialized, we can register performance information
  //PERF_START (NULL, "PEI", NULL, StartTimeStamp);

  // SEC phase needs to run library constructors by hand.
  ProcessLibraryConstructorList ();

  // Assume the FV that contains the SEC (our code) also contains a compressed FV.
  Status = DecompressFirstFv ();
  ASSERT_EFI_ERROR (Status);

  // Load the DXE Core and transfer control to it
  Status = LoadDxeCoreFromFv (NULL, 0);
  ASSERT_EFI_ERROR (Status);
}

VOID
CEntryPoint (
  IN  UINTN  MpId,
  IN  UINTN  UefiMemoryBase,
  IN  UINTN  StacksBase
  )
{
  UINT64  StartTimeStamp;

  // Initialize the platform specific controllers
  ArmPlatformInitialize (MpId);

  StartTimeStamp = 0;

  // Data Cache enabled on Primary core when MMU is enabled.
  ArmDisableDataCache ();
  // Invalidate instruction cache
  ArmInvalidateInstructionCache ();
  // Enable Instruction Caches on all cores.
  ArmEnableInstructionCache ();

  // Wait the Primary core has defined the address of the Global Variable region (event: ARM_CPU_EVENT_DEFAULT)
  ArmCallWFE ();

  // Goto primary Main.
  //PrimaryMain (UefiMemoryBase, StacksBase, StartTimeStamp);
  PrePiMain (UefiMemoryBase, StacksBase, StartTimeStamp);

  // DXE Core should always load and never return
  ASSERT (FALSE);
}
