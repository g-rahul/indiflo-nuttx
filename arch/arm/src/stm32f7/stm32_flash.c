/****************************************************************************
 * arch/arm/src/stm32f7/stm32_flash.c
 *
 *   Copyright (C) 2018 Wolpike LLC. All rights reserved.
 *   Author: Evgeniy Bobkov <evgen@wolpike.com>
 *
 * Ported from stm32f20xxf40xx_flash.c, this is the original license:
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Provides standard flash access functions, to be used by the  flash mtd
 * driver.  The interface is defined in the include/nuttx/progmem.h
 *
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "barriers.h"

#include "hardware/stm32_flash.h"
#include "stm32_waste.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_KEY1         0x45670123
#define FLASH_KEY2         0xcdef89ab
#define FLASH_OPTKEY1      0x08192a3b
#define FLASH_OPTKEY2      0x4c5d6e7f
#define FLASH_ERASEDVALUE  0xff

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void flash_unlock(void)
{
  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32_waste();
    }

  if (getreg32(STM32_FLASH_CR) & FLASH_CR_LOCK)
    {
      /* Unlock sequence */

      putreg32(FLASH_KEY1, STM32_FLASH_KEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_KEYR);
    }
}

static void flash_lock(void)
{
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_LOCK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_flash_unlock(void)
{
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  flash_unlock();
  nxmutex_unlock(&g_lock);

  return ret;
}

int stm32_flash_lock(void)
{
  int ret;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return ret;
    }

  flash_lock();
  nxmutex_unlock(&g_lock);

  return ret;
}

/****************************************************************************
 * Name: stm32_flash_writeprotect
 *
 * Description:
 *   Enable or disable the write protection of a flash sector.
 *
 ****************************************************************************/

int stm32_flash_writeprotect(size_t page, bool enabled)
{
  uint32_t reg;
  uint32_t val;

  if (page >= STM32_FLASH_NBLOCK_ACT)
    {
      return -EFAULT;
    }

  /* Select the register that contains the bit to be changed */

  if (page < 12)
    {
      reg = STM32_FLASH_OPTCR;
    }
#if defined(CONFIG_STM32_FLASH_CONFIG_I)
  else
    {
      reg = STM32_FLASH_OPTCR1;
      page -= 12;
    }
#else
  else
    {
      return -EFAULT;
    }
#endif

  /* Read the option status */

  val = getreg32(reg);

  /* Set or clear the protection */

  if (enabled)
    {
      val &= ~(1 << (16 + page));
    }
  else
    {
      val |=  (1 << (16 + page));
    }

  /* Unlock options */

  putreg32(FLASH_OPTKEY1, STM32_FLASH_OPTKEYR);
  putreg32(FLASH_OPTKEY2, STM32_FLASH_OPTKEYR);

  /* Write options */

  putreg32(val, reg);

  /* Trigger programming */

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  /* Wait for completion */

  while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
    {
      stm32_waste();
    }

  /* Re-lock options */

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTLOCK);
  return 0;
}

size_t up_progmem_pagesize(size_t page)
{
  return UP_PROGMEM_PAGE_SIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  size_t page_end = 0;
  size_t i;

  if (addr >= STM32_FLASH_BASE)
    {
      addr -= STM32_FLASH_BASE;
    }

  if (addr >= STM32_FLASH_SIZE)
    {
      return -EFAULT;
    }

  for (i = 0; i < STM32_FLASH_NPAGES; ++i)
    {
      page_end += up_progmem_pagesize(i);
      if (page_end > addr)
        {
          return i;
        }
    }

  return -EFAULT;
}

size_t up_progmem_getaddress(size_t page)
{
  size_t base_address = STM32_FLASH_BASE;
  size_t i;

  if (page >= STM32_FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  for (i = 0; i < page; ++i)
    {
      base_address += up_progmem_pagesize(i);
    }

  return base_address;
}

size_t up_progmem_neraseblocks(void)
{
  return UP_PROGMEM_ERASE_NBLOCKS;
}

bool up_progmem_isuniform(void)
{
#ifdef STM32_FLASH_PAGESIZE
  return true;
#else
  return false;
#endif
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= STM32_FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != FLASH_ERASEDVALUE)
        {
          bwritten++;
        }
    }

  return bwritten;
}


size_t up_progmem_erasesize(size_t block)
{
  return FLASH_SECTOR_SIZE;
}

static size_t stm32_flash_blocksize(size_t block)
{
  static const size_t block_sizes[STM32_FLASH_NBLOCK_ACT] = STM32_FLASH_SIZES_ACT;

  if (block >= sizeof(block_sizes) / sizeof(*block_sizes))
  {
    return 0;
  }
  else
  {
    return block_sizes[block];
  }
}

size_t stm32_flash_blockgetaddress(size_t block)
{
  size_t base_address = STM32_FLASH_BASE;
  size_t i;

  if (block >= STM32_FLASH_NBLOCK_ACT)
  {
    return SIZE_MAX;
  }

  for (i = 0; i < block; ++i)
  {
    base_address += stm32_flash_blocksize(i);
  }

  return base_address;
}

static size_t stm32_flash_isblockerased(size_t block)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (block >= STM32_FLASH_NBLOCK_ACT)
  {
    return -EFAULT;
  }

  /* Verify */

  for (addr = stm32_flash_blockgetaddress(block), count = stm32_flash_blocksize(block);
       count; count--, addr++)
  {
    if (getreg8(addr) != FLASH_ERASEDVALUE)
    {
      bwritten++;
    }
  }

  return bwritten;

}


static ssize_t progmem_eraseblock_discrete(size_t block)
{
  int ret = 0;
  int erase_attempt = 0;

  if (block >= STM32_FLASH_NBLOCK_ACT)
  {
    return -EFAULT;
  }

  ret = nxmutex_lock(&g_lock);

  if (ret < 0)
  {
    return (ssize_t)ret;
  }

/* Get flash ready and begin erasing single block */
flash_erase:

  erase_attempt++;

  flash_unlock();

  modifyreg32(STM32_FLASH_SR, 0, FLASH_SR_EOP);
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_SER);
  modifyreg32(STM32_FLASH_CR, FLASH_CR_SNB_MASK, FLASH_CR_SNB(block));
  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_STRT);
 
  while ((getreg32(STM32_FLASH_SR) & FLASH_SR_BSY) 
               && !(getreg32(STM32_FLASH_SR) & FLASH_SR_EOP))
  {
    stm32_waste();
  }

  if( getreg32(STM32_FLASH_SR) & (FLASH_SR_OPERR
                               |  FLASH_SR_WRPERR
                               |  FLASH_SR_PGAERR
                               |  FLASH_SR_PGPERR
                               |  FLASH_SR_PGSERR))
  {
    return -EIO; /* failure */
  }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_SER, 0);
  nxmutex_unlock(&g_lock);

  /* Verify */
  if (stm32_flash_isblockerased(block) == 0)
    {
      return stm32_flash_blocksize(block); /* success */
    }
  else
    {
      if(erase_attempt == 5)
      {
        return -EIO; /* failure */
      }
      else
      {
        goto flash_erase;  /* Take More Attempt */
      }
    }
}

ssize_t up_progmem_eraseblock(size_t block)
{
  int ret = 0;
  size_t i = 0;
  size_t merge_blk_cnt = FLASH_MERGE_BLK_COUNT;

  /* Erase combined HW sectors for block index < 5 */
  if (block == 0 )
  {
    for (i = 0; i < merge_blk_cnt; i++)
    {
      ret = progmem_eraseblock_discrete(i);
      if (ret < 0)
      {
        return ret;
      }
    }
  }
  else
  {
    block += (FLASH_MERGE_BLK_COUNT - 1);
    return progmem_eraseblock_discrete(block);
  }

  return ret;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  uint8_t *byte = (uint8_t *)buf;
  size_t written = count;
  uintptr_t flash_base;
  int ret;

  /* Check for valid address range */

  if (addr >= STM32_FLASH_BASE &&
      addr + count <= STM32_FLASH_BASE + STM32_FLASH_SIZE)
    {
      flash_base = STM32_FLASH_BASE;
    }
  else if (addr >= STM32_OPT_BASE &&
           addr + count <= STM32_OPT_BASE + STM32_OPT_SIZE)
    {
      flash_base = STM32_OPT_BASE;
    }
  else
    {
      return -EFAULT;
    }

  addr -= flash_base;

  ret = nxmutex_lock(&g_lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Get flash ready and begin flashing */

  flash_unlock();

  modifyreg32(STM32_FLASH_CR, 0, FLASH_CR_PG);

  /* TODO: implement up_progmem_write() to support other sizes than 8-bits */

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PSIZE_MASK, FLASH_CR_PSIZE_X8);

  for (addr += flash_base; count; count -= 1, byte++, addr += 1)
    {
      /* Write half-word and wait to complete */

      putreg8(*byte, addr);

      /* Data synchronous Barrier (DSB) just after the write operation. This
       * will force the CPU to respect the sequence of instruction (no
       * optimization).
       */

      ARM_DSB();

      while (getreg32(STM32_FLASH_SR) & FLASH_SR_BSY)
        {
          stm32_waste();
        }

      /* Verify */

      if (getreg32(STM32_FLASH_SR) & FLASH_CR_SER)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          nxmutex_unlock(&g_lock);
          return -EROFS;
        }

      if (getreg8(addr) != *byte)
        {
          modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);
          nxmutex_unlock(&g_lock);
          return -EIO;
        }
    }

  modifyreg32(STM32_FLASH_CR, FLASH_CR_PG, 0);

  nxmutex_unlock(&g_lock);
  return written;
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}
