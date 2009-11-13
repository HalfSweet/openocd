/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
#ifndef ARM966E_H
#define ARM966E_H

#include "arm9tdmi.h"

#define	ARM966E_COMMON_MAGIC 0x20f920f9

typedef struct arm966e_common_s
{
	arm9tdmi_common_t arm9tdmi_common;
	int common_magic;
	uint32_t cp15_control_reg;
} arm966e_common_t;

static inline struct arm966e_common_s *
target_to_arm966(struct target_s *target)
{
	return container_of(target->arch_info, struct arm966e_common_s,
			arm9tdmi_common.arm7_9_common.armv4_5_common);
}

int arm966e_init_arch_info(target_t *target,
		arm966e_common_t *arm966e, struct jtag_tap *tap);
int arm966e_register_commands(struct command_context_s *cmd_ctx);
int arm966e_write_cp15(target_t *target, int reg_addr, uint32_t value);

#endif /* ARM966E_H */
