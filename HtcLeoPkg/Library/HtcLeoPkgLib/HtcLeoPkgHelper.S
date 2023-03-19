#
#  Copyright (c) 2018, Linaro Limited. All rights reserved.
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution.  The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
#

#include <AsmMacroIoLib.h>
#include <Library/ArmLib.h>

#include <Library/ops.h>

/* void arch_disable_cache(uint flags) */
ASM_FUNC(arch_disable_cache)
	stmfd	sp!, {r4-r11, lr}

	mov		r7, r0						// save flags

	mrs		r12, cpsr					// save the old interrupt state
	.word	0xf10c01c0	/* cpsid iaf */	// interrupts disabled

.Ldcache_disable:
	tst		r7, #DCACHE
	beq		.Licache_disable
	mrc     p15, 0, r0, c1, c0, 0		// cr1
	tst		r0, #(1<<2)					// is the dcache already disabled?
	beq		.Ldcache_already_disabled

	bic		r0, #(1<<2)
	mcr		p15, 0, r0, c1, c0, 0		// disable dcache

	// flush and invalidate the dcache
	// NOTE: trashes a bunch of registers, can't be spilling stuff to the stack
	bl		flush_invalidate_cache_v7

	b		.Ldcache_disable_L2

.Ldcache_already_disabled:
	// make sure all of the caches are invalidated
	// NOTE: trashes a bunch of registers, can't be spilling stuff to the stack
	bl		invalidate_cache_v7

.Ldcache_disable_L2:

	// disable the L2, if present
	mrc     p15, 0, r0, c1, c0, 1		// aux cr1
	bic		r0, #(1<<1)
	mcr		p15, 0, r0, c1, c0, 1		// disable L2 dcache

.Licache_disable:
	tst		r7, #ICACHE
	beq		.Ldone_disable

	mrc     p15, 0, r0, c1, c0, 0		// cr1
	bic		r0, #(1<<12)
	mcr		p15, 0, r0, c1, c0, 0		// disable icache

.Ldone_disable:
	// make sure the icache is always invalidated
	mov		r0, #0
	mcr		p15, 0, r0, c7, c5, 0		// invalidate icache to PoU

	msr		cpsr, r12
	ldmfd	sp!, {r4-r11, pc}


ASM_FUNC(arch_enable_cache)
	stmfd	sp!, {r4-r11, lr}

	mov		r7, r0						// save flags

	mrs		r12, cpsr					// save the old interrupt state
	.word	0xf10c01c0	/* cpsid iaf */	// interrupts disabled
	
.Ldcache_enable:
	tst		r7, #DCACHE
	beq		.Licache_enable
	mrc     p15, 0, r0, c1, c0, 0		// cr1
	tst		r0, #(1<<2)					// is the dcache already enabled?
	bne		.Licache_enable

	// invalidate L1 and L2
	// NOTE: trashes a bunch of registers, can't be spilling stuff to the stack
	bl		invalidate_cache_v7

	// enable the L2, if present
	mrc     p15, 0, r0, c1, c0, 1		// aux cr1
	orr		r0, #(1<<1)
	mcr		p15, 0, r0, c1, c0, 1		// enable L2 dcache

	mrc     p15, 0, r0, c1, c0, 0		// cr1
	orr		r0, #(1<<2)
	mcr		p15, 0, r0, c1, c0, 0		// enable dcache

.Licache_enable:
	tst		r7, #ICACHE
	beq		.Ldone_enable

	mov		r0, #0
	mcr		p15, 0, r0, c7, c5, 0		// invalidate icache to PoU

	mrc     p15, 0, r0, c1, c0, 0		// cr1
	orr		r0, #(1<<12)
	mcr		p15, 0, r0, c1, c0, 0		// enable icache

.Ldone_enable:
	msr		cpsr, r12
	ldmfd	sp!, {r4-r11, pc}

// flush & invalidate cache routine, trashes r0-r6, r9-r11
flush_invalidate_cache_v7:
	/* from ARMv7 manual, B2-17 */
	MRC 	p15, 1, R0, c0, c0, 1 		// Read CLIDR 
	ANDS 	R3, R0, #0x7000000 
	MOV 	R3, R3, LSR #23 			// Cache level value (naturally aligned) 
	BEQ 	.Lfinished 
	MOV 	R10, #0 
.Loop1:
	ADD 	R2, R10, R10, LSR #1 		// Work out 3xcachelevel 
	MOV 	R1, R0, LSR R2 				// bottom 3 bits are the Cache type for this level 
	AND 	R1, R1, #7 					// get those 3 bits alone 
	CMP 	R1, #2 
	BLT 	.Lskip 						// no cache or only instruction cache at this level 
	MCR 	p15, 2, R10, c0, c0, 0 		// write the Cache Size selection register 
	.word	0xf57ff06f	// ISB 			// ISB to sync the change to the CacheSizeID reg 
	MRC 	p15, 1, R1, c0, c0, 0 		// reads current Cache Size ID register 
	AND 	R2, R1, #0x7 				// extract the line length field 
	ADD 	R2, R2, #4 					// add 4 for the line length offset (log2 16 bytes) 
	LDR 	R4, =0x3FF 
	ANDS 	R4, R4, R1, LSR #3 			// R4 is the max number on the way size (right aligned) 
	CLZ 	R5, R4 						// R5 is the bit position of the way size increment 
	LDR 	R6, =0x00007FFF 
	ANDS 	R6, R6, R1, LSR #13 		// R6 is the max number of the index size (right aligned) 
.Loop2:
	MOV 	R9, R4 						// R9 working copy of the max way size (right aligned) 
.Loop3:
	ORR 	R11, R10, R9, LSL R5 		// factor in the way number and cache number into R11 
	ORR 	R11, R11, R6, LSL R2 		// factor in the index number 
	MCR 	p15, 0, R11, c7, c14, 2 	// clean & invalidate by set/way 
	SUBS 	R9, R9, #1 					// decrement the way number 
	BGE 	.Loop3 
	SUBS 	R6, R6, #1 					// decrement the index 
	BGE 	.Loop2 
.Lskip:
 	ADD 	R10, R10, #2 					// increment the cache number 
	CMP 	R3, R10 
	BGT 	.Loop1 

.Lfinished:
	mov		r10, #0
	mcr		p15, 2, r10, c0, c0, 0		// select cache level 0
	.word	0xf57ff06f	// isb

	bx		lr

// invalidate cache routine, trashes r0-r6, r9-r11
invalidate_cache_v7:
	/* from ARMv7 manual, B2-17 */
	MRC 	p15, 1, R0, c0, c0, 1 		// Read CLIDR 
	ANDS 	R3, R0, #0x7000000 
	MOV 	R3, R3, LSR #23 			// Cache level value (naturally aligned) 
	BEQ 	.Lfinished_invalidate
	MOV 	R10, #0 
.Loop1_invalidate:
	ADD 	R2, R10, R10, LSR #1 		// Work out 3xcachelevel 
	MOV 	R1, R0, LSR R2 				// bottom 3 bits are the Cache type for this level 
	AND 	R1, R1, #7 					// get those 3 bits alone 
	CMP 	R1, #2 
	BLT 	.Lskip_invalidate 			// no cache or only instruction cache at this level 
	MCR 	p15, 2, R10, c0, c0, 0 		// write the Cache Size selection register 
	.word	0xf57ff06f	// ISB 			// ISB to sync the change to the CacheSizeID reg 
	MRC 	p15, 1, R1, c0, c0, 0 		// reads current Cache Size ID register 
	AND 	R2, R1, #0x7 				// extract the line length field 
	ADD 	R2, R2, #4 					// add 4 for the line length offset (log2 16 bytes) 
	LDR 	R4, =0x3FF 
	ANDS 	R4, R4, R1, LSR #3 			// R4 is the max number on the way size (right aligned) 
	CLZ 	R5, R4 						// R5 is the bit position of the way size increment 
	LDR 	R6, =0x00007FFF 
	ANDS 	R6, R6, R1, LSR #13 		// R6 is the max number of the index size (right aligned) 
.Loop2_invalidate:
	MOV 	R9, R4 						// R9 working copy of the max way size (right aligned) 
.Loop3_invalidate:
	ORR 	R11, R10, R9, LSL R5 		// factor in the way number and cache number into R11 
	ORR 	R11, R11, R6, LSL R2 		// factor in the index number 
	MCR 	p15, 0, R11, c7, c6, 2 		// invalidate by set/way 
	SUBS 	R9, R9, #1 					// decrement the way number 
	BGE 	.Loop3_invalidate 
	SUBS 	R6, R6, #1 					// decrement the index 
	BGE 	.Loop2_invalidate 
.Lskip_invalidate:
 	ADD 	R10, R10, #2 				// increment the cache number 
	CMP 	R3, R10 
	BGT 	.Loop1_invalidate 

.Lfinished_invalidate:
	mov		r10, #0
	mcr		p15, 2, r10, c0, c0, 0		// select cache level 0
	.word	0xf57ff06f	// isb

	bx		lr

/* shared cache flush routines */

	/* void arch_flush_cache_range(addr_t start, size_t len); 
ASM_FUNC(arch_clean_cache_range)
0:
	mcr		p15, 0, r0, c7, c10, 1		// clean cache to PoC by MVA
	add		r0, r0, #CACHE_LINE
	subs	r1, r1, #CACHE_LINE
	bhs		0b
	
	mov		r0, #0
	mcr		p15, 0, r0, c7, c10, 4		// data sync barrier (formerly drain write buffer)

	bx		lr*/

	/* void arch_flush_invalidate_cache_range(addr_t start, size_t len); 
ASM_FUNC(arch_clean_invalidate_cache_range)
0:
	mcr		p15, 0, r0, c7, c14, 1		// clean & invalidate cache to PoC by MVA
	add		r0, r0, #CACHE_LINE
	subs	r1, r1, #CACHE_LINE
	bhs		0b

	mov		r0, #0
	mcr		p15, 0, r0, c7, c10, 4		// data sync barrier (formerly drain write buffer)

	bx		lr*/






ASM_FUNC(ArmPlatformPeiBootAction)
  // Disable L2 cache
  mrc   p15, 0, r0, c1, c0, 1
  bic   r0, r0, #0x00000002
  mcr   p15, 0, r0, c1, c0, 1
  // Disable Strict alignment checking & Enable Instruction cache
  mrc     p15, 0, r0, c1, c0, 0
  bic     r0, r0, #0x00002300     /* clear bits 13, 9:8 (--V- --RS) */
  bic     r0, r0, #0x00000005     /* clear bits 0, 2 (---- -C-M) */
  bic     r0, r0, #0x00000002     /* Clear bit 1 (Alignment faults) */
  orr     r0, r0, #0x00001000     /* set bit 12 (I) enable I-Cache */
  mcr     p15, 0, r0, c1, c0, 0
  bx    lr

//UINTN
//ArmPlatformGetCorePosition (
//  IN UINTN MpId
//  );
ASM_FUNC(ArmPlatformGetCorePosition)
  and   r1, r0, #ARM_CORE_MASK
  and   r0, r0, #ARM_CLUSTER_MASK
  add   r0, r1, r0, LSR #7
  bx    lr

//UINTN
//ArmPlatformGetPrimaryCoreMpId (
//  VOID
//  );
ASM_FUNC(ArmPlatformGetPrimaryCoreMpId)
  MOV32  (r0, FixedPcdGet32 (PcdArmPrimaryCore))
  bx    lr

//UINTN
//ArmPlatformIsPrimaryCore (
//  IN UINTN MpId
//  );
ASM_FUNC(ArmPlatformIsPrimaryCore)
  MOV32  (r1, FixedPcdGet32 (PcdArmPrimaryCoreMask))
  and   r0, r0, r1
  MOV32  (r1, FixedPcdGet32 (PcdArmPrimaryCore))
  cmp   r0, r1
  moveq r0, #1
  movne r0, #0
  bx    lr