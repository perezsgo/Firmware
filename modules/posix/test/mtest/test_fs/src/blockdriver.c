/* Copyright 2014, ACSE & CADIEEL
 *    ACSE   : http://www.sase.com.ar/asociacion-civil-sistemas-embebidos/ciaa/
 *    CADIEEL: http://www.cadieel.org.ar
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Raw block device management driver
 **
 ** Definicion de funciones y variables relacionadas con el driver de manejo de blockdevice sin un filesystem asociado.
 ** Se trata al Device como un unico archivo, no se tiene en cuenta el formateo de datos.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Template Template to start a new module
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/

#include "ciaaPOSIX_stdlib.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_string.h"
#include "ciaaPOSIX_stdbool.h"
#include "ciaaBlockDevices.h"
#include "vfs.h"
#include "blockdriver.h"

/*==================[macros and definitions]=================================*/
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static int blockdriver_open(struct file_desc *file)
{
   file->node->f_info.file_pointer = 0;
   return 0;
}
static int blockdriver_close(struct file_desc *file)
{
   return 0;
}

static size_t blockdriver_read(struct file_desc *file, void *buf, size_t size)
{
   uint32_t pointer;
   ciaaDevices_deviceType const *device;
   size_t ret;

   pointer = file->node->f_info.file_pointer;
   device = file->node->fs_info.device;
   ret = ciaaBlockDevices_lseek(device, pointer, SEEK_SET);
   if(ret!=pointer)
   {
      ciaaPOSIX_printf("ext2_get_superblock: Fallo seek\n");
      return 0;
   }

   ret = ciaaBlockDevices_read(device, (uint8_t *)buf, size);
   file->node->f_info.file_pointer += ret;
   return ret;
}

static size_t blockdriver_write(struct file_desc *file, void *buf, size_t size)
{
   uint32_t pointer;
   ciaaDevices_deviceType const *device;
   size_t ret;

   pointer = file->node->f_info.file_pointer;
   device = file->node->fs_info.device;
   ret = ciaaBlockDevices_lseek(device, pointer, SEEK_SET);
   if(ret!=pointer)
   {
      ciaaPOSIX_printf("ext2_get_superblock: Fallo seek\n");
      return 0;
   }
   ret = ciaaBlockDevices_write(device, (uint8_t *)buf, size);
   file->node->f_info.file_pointer += ret;
   return ret;
}

static int blockdriver_ioctl(struct file_desc *file, int request)
{
   ciaaDevices_deviceType const *device;
   size_t ret;

   device = file->node->fs_info.device;
   ret = ciaaBlockDevices_ioctl(device, request, NULL);
   return ret;
}

static int blockdriver_init(void *par){return 0;}
static int blockdriver_format(void *par){return 0;}
static int blockdriver_mount(vnode_t *dev_node, vnode_t *dest_node){return 0;}
static int blockdriver_create_node(vnode_t *parent_node, vnode_t *child_node){return 0;}
static int blockdriver_delete_node(vnode_t *node){return 0;}
static int blockdriver_truncate(vnode_t *node, uint32_t length){return 0;}
static int blockdriver_umount(vnode_t *node){return 0;}

/*==================[internal data definition]===============================*/

static struct fsdriver_operations blockdriver_operations =
{
   blockdriver_open,
   blockdriver_close,
   blockdriver_read,
   blockdriver_write,
   blockdriver_ioctl,

   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL,
   NULL
};

/*==================[external data definition]===============================*/

//Variable global. Se declarara extern en vfs.c
struct filesystem_driver blockdev_driver =
{
   "BLOCKDEV",
   &blockdriver_operations
};

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
