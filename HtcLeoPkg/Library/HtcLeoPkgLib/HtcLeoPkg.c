/** @file
*
*  Copyright (c) 2018, Linaro Limited. All rights reserved.
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/ops.h>
#include <Library/arm.h>

#include <Ppi/ArmMpCoreInfo.h>

ARM_CORE_INFO mHiKey960InfoTable[] = {
    {
        // Cluster 0, Core 0
        0x000,

        // MP Core MailBox Set/Get/Clear Addresses and Clear Value
        (UINT64)0xFFFFFFFF
    },
};

/**
  Return the current Boot Mode
  This function returns the boot reason on the platform
  @return   Return the current Boot Mode of the platform
**/
EFI_BOOT_MODE
ArmPlatformGetBootMode (
    VOID
) {
    return BOOT_WITH_FULL_CONFIGURATION;
}

/*
Needed to boot in lk:

  // early arch stuff
	arch_early_init();

	// do any super early platform initialization
	platform_early_init();

	// do any super early target initialization
	target_early_init();
*/

static void set_vector_base(UINTN addr)
{
	__asm__ volatile("mcr	p15, 0, %0, c12, c0, 0" :: "r" (addr));
}

#define MEMBASE 0x28000000

#define __ALIGNED(x) __attribute__((aligned(x)))
UINT32 tt[4096] __ALIGNED(16384);

#define MMU_FLAG_CACHED 0x1
#define MMU_FLAG_BUFFERED 0x2
#define MMU_FLAG_READWRITE 0x4

#define MB (1024*1024)

void arm_mmu_map_section(UINTN paddr, UINTN vaddr, UINTN flags)
{
	INTN index;
	UINTN AP;
	UINTN CB;
	UINTN TEX = 0;

	AP = (flags & MMU_FLAG_READWRITE) ? 0x3 : 0x2;
	CB = ((flags & MMU_FLAG_CACHED) ? 0x2 : 0) | ((flags & MMU_FLAG_BUFFERED) ? 0x1 : 0);

	index = vaddr / MB;
	// section mapping
	tt[index] = (paddr & ~(MB-1)) | (TEX << 12) | (AP << 10) | (0<<5) | (CB << 2) | (2<<0);

	arm_invalidate_tlb();
}

void arm_mmu_init(void)
{
	//int i;

	/* set some mmu specific control bits */
	arm_write_cr1(arm_read_cr1() & ~((1<<29)|(1<<28)|(1<<0))); // access flag disabled, TEX remap disabled, mmu disabled

	/* set up an identity-mapped translation table with cache disabled */
	//for (i=0; i < 4096; i++) {
	//	arm_mmu_map_section(i * MB, i * MB,  MMU_FLAG_READWRITE); // map everything uncached
	//}

	/* set up the translation table base */
	arm_write_ttbr((UINT32)tt);

	/* set up the domain access register */
	arm_write_dacr(0x00000001);

	/* turn on the mmu */
	arm_write_cr1(arm_read_cr1() | 0x1);
}

void arch_early_init(void)
{
	/* turn off the cache */
	arch_disable_cache(UCACHE);

	/* set the vector base to our exception vectors so we dont need to double map at 0 */
	set_vector_base(MEMBASE);

	//arm_mmu_init();

	/* turn the cache back on */
	arch_enable_cache(UCACHE);

	/* enable cp10 and cp11 */
	UINT32 val;
	__asm__ volatile("mrc	p15, 0, %0, c1, c0, 2" : "=r" (val));
	val |= (3<<22)|(3<<20);
	__asm__ volatile("mcr	p15, 0, %0, c1, c0, 2" :: "r" (val));

	/* set enable bit in fpexc */
	val = (1<<30);
	__asm__ volatile("mcr  p10, 7, %0, c8, c0, 0" :: "r" (val));
}

VOID
platform_early_init()
{}

VOID
target_early_init()
{
  //cedesmith: write reset vector while we can as MPU kicks in after flash_init();
	MmioWrite32(0, 0xe3a00546); //mov r0, #0x11800000
	MmioWrite32(4, 0xe590f004); //ldr	r15, [r0, #4]
}

/**
  Initialize controllers that must setup in the normal world
  This function is called by the ArmPlatformPkg/Pei or ArmPlatformPkg/Pei/PlatformPeim
  in the PEI phase.
**/
RETURN_STATUS
ArmPlatformInitialize (
    IN  UINTN                     MpId
) {
    arch_early_init();
    platform_early_init();
    target_early_init();

    return RETURN_SUCCESS;
}

EFI_STATUS
PrePeiCoreGetMpCoreInfo (
    OUT UINTN                   *CoreCount,
    OUT ARM_CORE_INFO           **ArmCoreTable
) {
    // Only support one cluster
    *CoreCount    = sizeof(mHiKey960InfoTable) / sizeof(ARM_CORE_INFO);
    *ArmCoreTable = mHiKey960InfoTable;
    return EFI_SUCCESS;
}

// Needs to be declared in the file. Otherwise gArmMpCoreInfoPpiGuid is undefined in the contect of PrePeiCore
EFI_GUID mArmMpCoreInfoPpiGuid = ARM_MP_CORE_INFO_PPI_GUID;
ARM_MP_CORE_INFO_PPI mMpCoreInfoPpi = { PrePeiCoreGetMpCoreInfo };

EFI_PEI_PPI_DESCRIPTOR      gPlatformPpiTable[] = {
    {
        EFI_PEI_PPI_DESCRIPTOR_PPI,
        &mArmMpCoreInfoPpiGuid,
        &mMpCoreInfoPpi
    }
};

VOID
ArmPlatformGetPlatformPpiList (
    OUT UINTN                   *PpiListSize,
    OUT EFI_PEI_PPI_DESCRIPTOR  **PpiList
) {
    *PpiListSize = sizeof(gPlatformPpiTable);
    *PpiList = gPlatformPpiTable;
}