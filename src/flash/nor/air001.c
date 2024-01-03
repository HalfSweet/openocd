// SPDX-License-Identifier: GPL-2.0-or-laterFailed to write m

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2024 by HalfSweet                                       *
 *   halfsweet@halfsweet.cn                                                *
 ***************************************************************************/

#include <stdint.h>
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>

#define AIR001_PAGE_SIZE 128

#define FLASH_REG_BASE_B0 0x40022000
#define FLASH_REG_BASE_B1 0x40022040

#define AIR001_FLASH_ACR     0x00
#define AIR001_FLASH_KEYR    0x08 
#define AIR001_FLASH_OPTKEYR 0x0C
#define AIR001_FLASH_SR      0x10
#define AIR001_FLASH_CR      0x14
#define AIR001_FLASH_OPTR    0x20
#define AIR001_FLASH_WRPR    0x2C

/* TODO: Check if code using these really should be hard coded to bank 0.
 * There are valid cases, on dual flash devices the protection of the
 * second bank is done on the bank0 reg's. */
#define AIR001_FLASH_KEYR_B0    0x40022004
#define AIR001_FLASH_OPTKEYR_B0 0x40022008
#define AIR001_FLASH_SR_B0      0x4002200C
#define AIR001_FLASH_CR_B0      0x40022010
#define AIR001_FLASH_OPTR_B0    0x40022020
#define AIR001_FLASH_WRPR_B0    0x40022020

/* option byte location */



/* FLASH_CR register bits */

#define FLASH_PG			(1 << 0) 
#define FLASH_PER			(1 << 1)
#define FLASH_MER			(1 << 2) 
#define FLASH_SER           (1 << 11)
#define FLASH_OPTSTRT		(1 << 17)
#define FLASH_PGSTRT        (1 << 19)
#define FLASH_EOPIE         (1 << 24)
#define FLASH_ERRIE         (1 << 25)
#define FLASH_OBL_LAUNCH	(1 << 27)
#define FLASH_OPTLOCK       (1 << 30)
#define FLASH_LOCK			(1 << 31)

#define FLASH_STRT	        (0) 
#define FLASH_OPTWRE	    (0) 
#define FLASH_OPTPG			(0)
#define FLASH_OPTER			(0) 

/* FLASH_SR register bits */

#define FLASH_BSY		(1 << 16) 
#define FLASH_WRPRTERR	(1 << 4) 
#define FLASH_EOP		(1 << 0) 
#define FLASH_OPTVERR   (1 << 15)
#define FLASH_PGERR		(1 << 2)

/* STM32_FLASH_OBR bit definitions (reading) */

// no obr reg...
#define OPT_ERROR		0
#define OPT_READOUT		1
#define OPT_RDWDGSW		2
#define OPT_RDRSTSTOP	3
#define OPT_RDRSTSTDBY	4
#define OPT_BFB2		5	/* dual flash bank only */

/* register unlock keys */

#define KEY1			0x45670123
#define KEY2			0xCDEF89AB

/* timeout values */

#define FLASH_PRE_TIMEOUT 100
#define FLASH_WRITE_TIMEOUT 10
#define FLASH_ERASE_TIMEOUT 100

struct air001_options {
	uint8_t rdp;
	uint8_t user;
	uint16_t data;
	uint32_t protection;
};

struct air001_flash_bank {
	struct air001_options option_bytes;
	int ppage_size;
	bool probed;

	bool has_dual_banks;
	/* used to access dual flash bank air001 */
	bool can_load_options;
	uint32_t register_base;
	uint8_t default_rdp;
	int user_data_offset;
	int option_offset;
	uint32_t user_bank_size;
};

static int air001_mass_erase(struct flash_bank *bank);
static int air001_write_sector(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t words_count);
static int air001_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count);

/* flash bank air001 <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(air001_flash_bank_command)
{
	struct air001_flash_bank *air001_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	air001_info = malloc(sizeof(struct air001_flash_bank));

	bank->driver_priv = air001_info;
	air001_info->probed = false;
	air001_info->has_dual_banks = false;
	air001_info->can_load_options = false;
	air001_info->register_base = FLASH_REG_BASE_B0;
	air001_info->user_bank_size = bank->size;

	/* The flash write must be aligned to a page boundary */
	bank->write_start_alignment = bank->write_end_alignment = AIR001_PAGE_SIZE;

	return ERROR_OK;
}

static inline int air001_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	struct air001_flash_bank *air001_info = bank->driver_priv;
	return reg + air001_info->register_base;
}

static inline int air001_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_SR), status);
}

static int air001_wait_status_busy(struct flash_bank *bank, int timeout, bool expect_eop)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = air001_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_BSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FLASH_BUSY;
		}
		alive_sleep(1);
	}

	if (status & FLASH_WRPRTERR) {
		LOG_ERROR("Air001 device protected");
		retval = ERROR_FLASH_PROTECTED;
	}

	if (status & FLASH_OPTVERR) {
		LOG_ERROR("Air001 device programming option fail");
		retval = ERROR_FLASH_OPERATION_FAILED;
	}


	if (!(status & FLASH_EOP) && expect_eop)
	{
		LOG_ERROR("Air001 device programming failed / flash not erased");
		retval = ERROR_FLASH_OPERATION_FAILED;
	}

	/* Clear all possible flags */
	if (status & (FLASH_WRPRTERR | FLASH_PGERR | FLASH_EOP)) {
		target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_SR),
				FLASH_WRPRTERR | FLASH_OPTVERR | FLASH_EOP);
	}
	return retval;
}

static int air001_check_operation_supported(struct flash_bank *bank)
{
	struct air001_flash_bank *air001_info = bank->driver_priv;

	/* if we have a dual flash bank device then
	 * we need to perform option byte stuff on bank0 only */
	if (air001_info->register_base != FLASH_REG_BASE_B0) {
		LOG_ERROR("Option byte operations must use bank 0");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int air001_read_options(struct flash_bank *bank)
{
	struct air001_flash_bank *air001_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t option_bytes;
	int retval;

	/* read user and read protection option bytes, user data option bytes */
	retval = target_read_u32(target, AIR001_FLASH_OPTR_B0, &option_bytes);
	if (retval != ERROR_OK)
		return retval;

	air001_info->option_bytes.rdp = (option_bytes & (1 << OPT_READOUT)) ? 0 : air001_info->default_rdp;
	air001_info->option_bytes.user = (option_bytes >> air001_info->option_offset >> 2) & 0xff;
	air001_info->option_bytes.data = (option_bytes >> air001_info->user_data_offset) & 0xffff;

	/* read write protection option bytes */
	retval = target_read_u32(target, AIR001_FLASH_WRPR_B0, &air001_info->option_bytes.protection);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int air001_erase_options(struct flash_bank *bank)
{
	struct air001_flash_bank *air001_info = bank->driver_priv;
	struct target *target = bank->target;

	/* read current options */
	air001_read_options(bank);

	/* unlock flash registers */
	int retval = target_write_u32(target, AIR001_FLASH_KEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, AIR001_FLASH_KEYR_B0, KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* unlock option flash registers */
	retval = target_write_u32(target, AIR001_FLASH_OPTKEYR_B0, KEY1);
	if (retval != ERROR_OK)
		goto flash_lock;
	retval = target_write_u32(target, AIR001_FLASH_OPTKEYR_B0, KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* erase option bytes */
	retval = target_write_u32(target, AIR001_FLASH_CR_B0, FLASH_OPTER | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		goto flash_lock;
	retval = target_write_u32(target, AIR001_FLASH_CR_B0, FLASH_OPTER | FLASH_STRT | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		goto flash_lock;

	retval = air001_wait_status_busy(bank, FLASH_ERASE_TIMEOUT, true);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* clear read protection option byte
	 * this will also force a device unlock if set */
	air001_info->option_bytes.rdp = air001_info->default_rdp;

	return ERROR_OK;

flash_lock:
	target_write_u32(target, AIR001_FLASH_CR_B0, FLASH_LOCK);
	return retval;
}

static int air001_write_options(struct flash_bank *bank)
{
	struct air001_flash_bank *air001_info = NULL;
	struct target *target = bank->target;

	air001_info = bank->driver_priv;

	/* unlock flash registers */
	int retval = target_write_u32(target, AIR001_FLASH_KEYR_B0, KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, AIR001_FLASH_KEYR_B0, KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* unlock option flash registers */
	retval = target_write_u32(target, AIR001_FLASH_OPTKEYR_B0, KEY1);
	if (retval != ERROR_OK)
		goto flash_lock;
	retval = target_write_u32(target, AIR001_FLASH_OPTKEYR_B0, KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* program option bytes */
	retval = target_write_u32(target, AIR001_FLASH_CR_B0, FLASH_OPTPG | FLASH_OPTWRE);
	if (retval != ERROR_OK)
		goto flash_lock;

	uint8_t opt_bytes[16];

	target_buffer_set_u16(target, opt_bytes, air001_info->option_bytes.rdp);
	target_buffer_set_u16(target, opt_bytes + 2, air001_info->option_bytes.user);
	target_buffer_set_u16(target, opt_bytes + 4, air001_info->option_bytes.data & 0xff);
	target_buffer_set_u16(target, opt_bytes + 6, (air001_info->option_bytes.data >> 8) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 8, air001_info->option_bytes.protection & 0xff);
	target_buffer_set_u16(target, opt_bytes + 10, (air001_info->option_bytes.protection >> 8) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 12, (air001_info->option_bytes.protection >> 16) & 0xff);
	target_buffer_set_u16(target, opt_bytes + 14, (air001_info->option_bytes.protection >> 24) & 0xff);

	/* Block write is preferred in favour of operation with ancient ST-Link
	 * firmwares without 16-bit memory access. See
	 * 480: flash: stm32f1x: write option bytes using the loader
	 * https://review.openocd.org/c/openocd/+/480
	 */
	// TODO: solve this
	//retval = air001_write_block(bank, opt_bytes, STM32_OB_RDP, sizeof(opt_bytes) / 2);

flash_lock:
	{
		int retval2 = target_write_u32(target, AIR001_FLASH_CR_B0, FLASH_LOCK);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int air001_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t protection;

	int retval = air001_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	/* medium density - each bit refers to a 4 sector protection block
	 * high density - each bit refers to a 2 sector protection block
	 * bit 31 refers to all remaining sectors in a bank */
	retval = target_read_u32(target, AIR001_FLASH_WRPR_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = (protection & (1 << i)) ? 0 : 1;

	return ERROR_OK;
}

static int air001_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
    struct target *target = bank->target;
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return air001_mass_erase(bank);

	// check busy, maybe another operation is pending
	int retval = air001_wait_status_busy(bank, FLASH_PRE_TIMEOUT, false);
	if (retval != ERROR_OK)
		return retval;

	/* unlock flash registers */
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	for (unsigned int i = first; i <= last; i += AIR001_PAGE_SIZE / 4) {
		retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_CR), FLASH_PER | FLASH_EOPIE | FLASH_ERRIE);
		if (retval != ERROR_OK)
			goto flash_lock;
		
		// todo: better find out base address
		retval = target_write_u32(target, 0x08000000 + bank->sectors[i].offset, 0xFFFFFFFF);
		if (retval != ERROR_OK) {
			LOG_DEBUG("Ignoring address write retval %d ", retval);
		}

		retval = air001_wait_status_busy(bank, FLASH_ERASE_TIMEOUT, true);
		if (retval != ERROR_OK)
			goto flash_lock;
	}

flash_lock:
	{
		int retval2 = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_CR), FLASH_LOCK);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

static int air001_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct air001_flash_bank *air001_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = air001_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = air001_erase_options(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("air001 failed to erase options");
		return retval;
	}

	for (unsigned int i = first; i <= last; i++) {
		if (set)
			air001_info->option_bytes.protection &= ~(1 << i);
		else
			air001_info->option_bytes.protection |= (1 << i);
	}

	return air001_write_options(bank);
}

/** Writes a block to flash either using target algorithm
 *  or use fallback, host controlled halfword-by-halfword access.
 *  Flash controller must be unlocked before this call.
 */
static int air001_write_sector(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t address, uint32_t words_count)
{
	struct target *target = bank->target;
	int retval;

	/* The flash write must be aligned to a AIR001_PAGE_SIZE boundary.
	 * The flash infrastructure ensures it, do just a security check
	 */
	assert(address % AIR001_PAGE_SIZE == 0);
	assert(words_count > 0);

	//LOG_DEBUG("words count %d address 0x%x", words_count - 1, address);

	if (words_count > 1) {
		retval = target_write_memory(target, address, 4, words_count - 1, buffer);
		if (retval != ERROR_OK)
			return retval;
		address += 4 * (words_count - 1);
		buffer += 4 * (words_count - 1);
	}

	// set FLASH_PGSTRT before writing last word

	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_CR), FLASH_PG | FLASH_EOPIE | FLASH_ERRIE | FLASH_PGSTRT);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_memory(target, address, 4, 1, buffer);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Ignoring final address write retval %d ", retval);
		//return retval;
	}

	retval = air001_wait_status_busy(bank, 5, true);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int air001_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* The flash write must be aligned to a word boundary (32bit).
	 * The flash infrastructure ensures it, do just a security check
	 */
	assert(offset % AIR001_PAGE_SIZE == 0);
	assert(count % AIR001_PAGE_SIZE == 0);

	int retval, retval2;


	// check busy, maybe another operation is pending
	retval = air001_wait_status_busy(bank, FLASH_PRE_TIMEOUT, false);
	if (retval != ERROR_OK)
		return retval;

	/* unlock flash registers */
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	/* enable flash programming */
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_CR), FLASH_PG | FLASH_EOPIE | FLASH_ERRIE);
	if (retval != ERROR_OK)
		goto reset_pg_and_lock;

	for (unsigned int sector=0; sector < bank->num_sectors; sector++)
	{
		if (bank->sectors[sector].offset == offset)
		{
			// need to program this sector
			/* write to flash */
			retval = air001_write_sector(bank, buffer, bank->base + offset, bank->sectors[sector].size / 4);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			offset += bank->sectors[sector].size;
			buffer += bank->sectors[sector].size;
		}
	}


reset_pg_and_lock:
	retval2 = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_CR), FLASH_LOCK);
	if (retval == ERROR_OK)
		retval = retval2;

	return retval;
}

struct air001_property_addr {
	uint32_t device_id;
	uint32_t flash_size;
};

static int air001_get_property_addr(struct target *target, struct air001_property_addr *addr)
{
	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	switch (cortex_m_get_impl_part(target)) {
	case CORTEX_M0P_PARTNO:
		addr->device_id = 0x40015800;
		addr->flash_size = 0x1FFF0D00; // not sure
		return ERROR_OK;
	default:
		LOG_ERROR("Cannot identify target as a air001");
		return ERROR_FAIL;
	}
}

static int air001_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	struct target *target = bank->target;
	struct air001_property_addr addr;

	int retval = air001_get_property_addr(target, &addr);
	if (retval != ERROR_OK)
		return retval;

	return target_read_u32(target, addr.device_id, device_id);
}

static int air001_get_flash_size(struct flash_bank *bank, uint16_t *flash_size_in_kb)
{
	struct target *target = bank->target;
	struct air001_property_addr addr;

	int retval = air001_get_property_addr(target, &addr);
	if (retval != ERROR_OK)
		return retval;

    // The manufacturer doesn't specify
    *flash_size_in_kb = 32;
    return ERROR_OK;
	// return target_read_u16(target, addr.flash_size, flash_size_in_kb);
}

static int air001_probe(struct flash_bank *bank)
{
	struct air001_flash_bank *air001_info = bank->driver_priv;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t dbgmcu_idcode;
	int page_size;
	uint32_t base_address = 0x08000000;

	air001_info->probed = false;
	air001_info->register_base = FLASH_REG_BASE_B0;
	air001_info->user_data_offset = 10;
	air001_info->option_offset = 0;

	/* default factory read protection level 0 */
	air001_info->default_rdp = 0xA5;

	/* read air001 device id register */
	int retval = air001_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("device id = 0x%08" PRIx32 "", dbgmcu_idcode);

	uint16_t device_id = dbgmcu_idcode & 0xfff;
	uint16_t rev_id = dbgmcu_idcode >> 16;

	/* set page size, protection granularity and max flash size depending on family */
	switch (device_id) {
	case 0x00:
		if (rev_id == 0x6000)
		{
			LOG_WARNING("AIR001 MCU detected!");
			page_size = AIR001_PAGE_SIZE;
			air001_info->ppage_size = 4;
            air001_info->user_bank_size = 32 * 1024;
			max_flash_size_in_kb = 32;
			air001_info->user_data_offset = 16;
			air001_info->option_offset = 6;
			air001_info->default_rdp = 0xAA;
			air001_info->can_load_options = true;
			break;
		}
		__attribute__ ((fallthrough));
	default:
		LOG_WARNING("Cannot identify target as a AIR001 family.");
		return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = air001_get_flash_size(bank, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("Air001 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	if (air001_info->has_dual_banks) {
		/* split reported size into matching bank */
		if (bank->base != 0x08080000) {
			/* bank 0 will be fixed 512k */
			flash_size_in_kb = 512;
		} else {
			flash_size_in_kb -= 512;
			/* bank1 also uses a register offset */
			air001_info->register_base = FLASH_REG_BASE_B1;
			base_address = 0x08080000;
		}
	}

	/* if the user sets the size manually then ignore the probed value
	 * this allows us to work around devices that have a invalid flash size register value */
	if (air001_info->user_bank_size) {
		LOG_INFO("ignoring flash probed value, using configured bank size");
		flash_size_in_kb = air001_info->user_bank_size / 1024;
	}

	LOG_INFO("flash size = %d KiB", flash_size_in_kb);

	/* did we assign flash size? */
	assert(flash_size_in_kb != 0xffff);

	/* calculate numbers of pages */
	int num_pages = flash_size_in_kb * 1024 / page_size;

	/* check that calculation result makes sense */
	assert(num_pages > 0);

	free(bank->sectors);
	bank->sectors = NULL;

	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	bank->base = base_address;
	bank->size = (num_pages * page_size);

	bank->num_sectors = num_pages;

	LOG_INFO("page size = %d", page_size);
	LOG_INFO("number of sectors/pages = %d", bank->num_sectors);

	bank->sectors = alloc_block_array(0, page_size, num_pages);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* calculate number of write protection blocks */
	int num_prot_blocks = num_pages / air001_info->ppage_size;
	if (num_prot_blocks > 32)
		num_prot_blocks = 32;

	bank->num_prot_blocks = num_prot_blocks;
	bank->prot_blocks = alloc_block_array(0, air001_info->ppage_size * page_size, num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	if (num_prot_blocks == 32)
		bank->prot_blocks[31].size = (num_pages - (31 * air001_info->ppage_size)) * page_size;

	air001_info->probed = true;

	return ERROR_OK;
}

static int air001_auto_probe(struct flash_bank *bank)
{
	struct air001_flash_bank *air001_info = bank->driver_priv;
	if (air001_info->probed)
		return ERROR_OK;
	return air001_probe(bank);
}

#if 0
COMMAND_HANDLER(air001_handle_part_id_command)
{
	return ERROR_OK;
}
#endif

static int get_air001_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	uint32_t dbgmcu_idcode;

	/* read stm32 device id register */
	int retval = air001_get_device_id(bank, &dbgmcu_idcode);
	if (retval != ERROR_OK)
		return retval;

	uint16_t device_id = dbgmcu_idcode & 0xfff;
	uint16_t rev_id = dbgmcu_idcode >> 16;
	const char *device_str;
	const char *rev_str = NULL;

	switch (device_id) {
    case 0x00:
        device_str = "AIR001";
        rev_str = "Unknown";
        break;

	default:
		command_print_sameline(cmd, "Cannot identify target as a Air001\n");
		return ERROR_FAIL;
	}

	if (rev_str)
		command_print_sameline(cmd, "%s - Rev: %s", device_str, rev_str);
	else
		command_print_sameline(cmd, "%s - Rev: unknown (0x%04x)", device_str, rev_id);

	return ERROR_OK;
}

COMMAND_HANDLER(air001_handle_lock_command)
{
	struct target *target = NULL;
	struct air001_flash_bank *air001_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	air001_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = air001_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	if (air001_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "air001 failed to erase options");
		return ERROR_OK;
	}

	/* set readout protection */
	air001_info->option_bytes.rdp = 0;

	if (air001_write_options(bank) != ERROR_OK) {
		command_print(CMD, "air001 failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD, "air001 locked");

	return ERROR_OK;
}

COMMAND_HANDLER(air001_handle_unlock_command)
{
	struct target *target = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = air001_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	if (air001_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "air001 failed to erase options");
		return ERROR_OK;
	}

	if (air001_write_options(bank) != ERROR_OK) {
		command_print(CMD, "air001 failed to unlock device");
		return ERROR_OK;
	}

	command_print(CMD, "air001 unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(air001_handle_options_read_command)
{
	uint32_t optionbyte, protection;
	struct target *target = NULL;
	struct air001_flash_bank *air001_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	air001_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = air001_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, AIR001_FLASH_OPTR_B0, &optionbyte);
	if (retval != ERROR_OK)
		return retval;

	uint16_t user_data = optionbyte >> air001_info->user_data_offset;

	retval = target_read_u32(target, AIR001_FLASH_WRPR_B0, &protection);
	if (retval != ERROR_OK)
		return retval;

	if (optionbyte & (1 << OPT_ERROR))
		command_print(CMD, "option byte complement error");

	command_print(CMD, "option byte register = 0x%" PRIx32 "", optionbyte);
	command_print(CMD, "write protection register = 0x%" PRIx32 "", protection);

	command_print(CMD, "read protection: %s",
				(optionbyte & (1 << OPT_READOUT)) ? "on" : "off");

	/* user option bytes are offset depending on variant */
	optionbyte >>= air001_info->option_offset;

	command_print(CMD, "watchdog: %sware",
				(optionbyte & (1 << OPT_RDWDGSW)) ? "soft" : "hard");

	command_print(CMD, "stop mode: %sreset generated upon entry",
				(optionbyte & (1 << OPT_RDRSTSTOP)) ? "no " : "");

	command_print(CMD, "standby mode: %sreset generated upon entry",
				(optionbyte & (1 << OPT_RDRSTSTDBY)) ? "no " : "");

	if (air001_info->has_dual_banks)
		command_print(CMD, "boot: bank %d", (optionbyte & (1 << OPT_BFB2)) ? 0 : 1);

	command_print(CMD, "user data = 0x%02" PRIx16 "", user_data);

	return ERROR_OK;
}

COMMAND_HANDLER(air001_handle_options_write_command)
{
	struct target *target = NULL;
	struct air001_flash_bank *air001_info = NULL;
	uint8_t optionbyte;
	uint16_t useropt;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	air001_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = air001_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = air001_read_options(bank);
	if (retval != ERROR_OK)
		return retval;

	/* start with current options */
	optionbyte = air001_info->option_bytes.user;
	useropt = air001_info->option_bytes.data;

	/* skip over flash bank */
	CMD_ARGC--;
	CMD_ARGV++;

	while (CMD_ARGC) {
		if (strcmp("SWWDG", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 0);
		else if (strcmp("HWWDG", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 0);
		else if (strcmp("NORSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 1);
		else if (strcmp("RSTSTOP", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 1);
		else if (strcmp("NORSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte |= (1 << 2);
		else if (strcmp("RSTSTNDBY", CMD_ARGV[0]) == 0)
			optionbyte &= ~(1 << 2);
		else if (strcmp("USEROPT", CMD_ARGV[0]) == 0) {
			if (CMD_ARGC < 2)
				return ERROR_COMMAND_SYNTAX_ERROR;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], useropt);
			CMD_ARGC--;
			CMD_ARGV++;
		} else if (air001_info->has_dual_banks) {
			if (strcmp("BOOT0", CMD_ARGV[0]) == 0)
				optionbyte |= (1 << 3);
			else if (strcmp("BOOT1", CMD_ARGV[0]) == 0)
				optionbyte &= ~(1 << 3);
			else
				return ERROR_COMMAND_SYNTAX_ERROR;
		} else
			return ERROR_COMMAND_SYNTAX_ERROR;
		CMD_ARGC--;
		CMD_ARGV++;
	}

	if (air001_erase_options(bank) != ERROR_OK) {
		command_print(CMD, "air001 failed to erase options");
		return ERROR_OK;
	}

	air001_info->option_bytes.user = optionbyte;
	air001_info->option_bytes.data = useropt;

	if (air001_write_options(bank) != ERROR_OK) {
		command_print(CMD, "air001 failed to write options");
		return ERROR_OK;
	}

	command_print(CMD, "air001 write options complete.\n"
				"INFO: %spower cycle is required "
				"for the new settings to take effect.",
				air001_info->can_load_options
					? "'air001 options_load' command or " : "");

	return ERROR_OK;
}

COMMAND_HANDLER(air001_handle_options_load_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	struct air001_flash_bank *air001_info = bank->driver_priv;

	if (!air001_info->can_load_options) {
		LOG_ERROR("Command not applicable to air001 devices - power cycle is "
			"required instead.");
		return ERROR_FAIL;
	}

	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = air001_check_operation_supported(bank);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK) {
		(void)target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_CR), FLASH_LOCK);
		return retval;
	}

	/* force re-load of option bytes - generates software reset */
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_CR), FLASH_OBL_LAUNCH);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int air001_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	// check busy, maybe another operation is pending
	int retval = air001_wait_status_busy(bank, FLASH_PRE_TIMEOUT, false);
	if (retval != ERROR_OK)
		return retval;

	/* unlock option flash registers */
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_KEYR), KEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_KEYR), KEY2);
	if (retval != ERROR_OK)
		goto flash_lock;

	/* mass erase flash memory */
	retval = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_CR), FLASH_MER | FLASH_EOPIE);
	if (retval != ERROR_OK)
		goto flash_lock;

	// according to datasheet we should write something to flash
	// todo: get location
	retval = target_write_u32(target, 0x20000000, 0xFF);

	if (retval != ERROR_OK) {
		LOG_DEBUG("Ignoring address write retval %d ", retval);
	}

	retval = air001_wait_status_busy(bank, FLASH_ERASE_TIMEOUT, true);
	if (retval != ERROR_OK)
		goto flash_lock;

flash_lock:
	{
		int retval2 = target_write_u32(target, air001_get_flash_reg(bank, AIR001_FLASH_CR), FLASH_LOCK);
		if (retval == ERROR_OK)
			retval = retval2;
	}
	return retval;
}

COMMAND_HANDLER(air001_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = air001_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "Air001 mass erase complete");
	else
		command_print(CMD, "Air001 mass erase failed");

	return retval;
}

static const struct command_registration air001_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = air001_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = air001_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = air001_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "options_read",
		.handler = air001_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option bytes.",
	},
	{
		.name = "options_write",
		.handler = air001_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('SWWDG'|'HWWDG') "
			"('RSTSTNDBY'|'NORSTSTNDBY') "
			"('RSTSTOP'|'NORSTSTOP') ('USEROPT' user_data)",
		.help = "Replace bits in device option bytes.",
	},
	{
		.name = "options_load",
		.handler = air001_handle_options_load_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Force re-load of device option bytes.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration air001_command_handlers[] = {
	{
		.name = "air001",
		.mode = COMMAND_ANY,
		.help = "air001 flash command group",
		.usage = "",
		.chain = air001_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver air001_flash = {
	.name = "air001",
	.commands = air001_command_handlers,
	.flash_bank_command = air001_flash_bank_command,
	.erase = air001_erase,
	.protect = air001_protect,
	.write = air001_write,
	.read = default_flash_read,
	.probe = air001_probe,
	.auto_probe = air001_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = air001_protect_check,
	.info = get_air001_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
