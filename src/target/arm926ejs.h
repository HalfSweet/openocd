/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef ARM926EJS_H
#define ARM926EJS_H

#include "arm9tdmi.h"
#include "armv4_5_mmu.h"

#define	ARM926EJS_COMMON_MAGIC 0xa926a926

typedef struct arm926ejs_common_s
{
	arm9tdmi_common_t arm9tdmi_common;
	uint32_t common_magic;
	armv4_5_mmu_common_t armv4_5_mmu;
	int (*read_cp15)(target_t *target, uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t *value);
	int (*write_cp15)(target_t *target, uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t value);
	uint32_t cp15_control_reg;
	uint32_t d_fsr;
	uint32_t i_fsr;
	uint32_t d_far;
} arm926ejs_common_t;

static inline struct arm926ejs_common_s *
target_to_arm926(struct target_s *target)
{
	return container_of(target->arch_info, struct arm926ejs_common_s,
		arm9tdmi_common.arm7_9_common.armv4_5_common);
}


int arm926ejs_init_arch_info(target_t *target,
		arm926ejs_common_t *arm926ejs, struct jtag_tap *tap);
int arm926ejs_register_commands(struct command_context_s *cmd_ctx);
int arm926ejs_arch_state(struct target_s *target);
int arm926ejs_write_memory(struct target_s *target,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
int arm926ejs_soft_reset_halt(struct target_s *target);

#endif /* ARM926EJS_H */
