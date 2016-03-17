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

/** \brief Short description of this file
 **
 ** Long description of this file
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Template Template to start a new module
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MZ      Marcos Ziegler
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160101 v0.0.1 MZ initial version
 */

/*
TODO:
-Implementar todo con memoria estatica, salvo excepciones donde se justifique
-Optimizar algoritmos tanto en memoria como en velocidad
-Implementar create_node, write. En general todos los algoritmos de reserva de bloques y nodos
*/

/*==================[inclusions]=============================================*/

#include "ciaaPOSIX_stdlib.h"
#include "ciaaBlockDevices.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_string.h"
#include "ciaaPOSIX_stdbool.h"
#include "ext2.h"
#include "vfs.h"

/*==================[macros and definitions]=================================*/

#define ASSERT_MSG(cond, msg) assert_msg((cond), (msg), __FILE__, __LINE__)

/*==================[internal data declaration]==============================*/
/*==================[internal functions declaration]=========================*/
static int ext2_block_map(const vnode_t * dest_node, uint32_t file_block, uint32_t *device_block_p);
static int ext2_buf_read_file(vnode_t * dest_node, uint8_t *buf, uint32_t size, uint32_t *total_read_size_p);
static int ext2_get_superblock(ciaaDevices_deviceType const * device, struct ext2sb * sb_p);
static int ext2_mount_load(vnode_t *dir_node);
static int ext2_mount_load_rec(vnode_t *dir_node);
static void print_superblock(struct ext2sb * superblock);
static void print_groupdescriptor(struct ext2_gd *gd);
static void print_inode(struct ext2fs_dinode * inode);
static int ext2_read_inode(vnode_t *dest_node, uint32_t inumber);
static int ext2_search_directory(vnode_t *dir_node, const char *dir_name, int length, uint32_t *inumber_p);
static node_type_t ext2type_to_vfstype(uint8_t ext2_type);
static int ext2_close(vnode_t * dest_node);
static int ext2_file_open(struct file_desc *file);
static int ext2_file_close(struct file_desc *file);
static size_t ext2_file_read(struct file_desc *file, void *buf, size_t size);
static size_t ext2_file_write(struct file_desc *file, void *buf, size_t size);
static int ext2_file_ioctl(struct file_desc *file, int request);
static int ext2_init(void *par);
static int ext2_format(vnode_t *dev_node);
static int ext2_mount(vnode_t *dev_node, vnode_t *dest_node);
static int ext2_create_node(vnode_t *parent_node, vnode_t *child_node);
static int ext2_delete_node(vnode_t *node);
static int ext2_truncate(vnode_t *node, uint32_t length);
static int ext2_umount(vnode_t *node);

/*==================[internal data definition]===============================*/

struct fsdriver_operations ext2_driver_operations =
{
   ext2_file_open,
   ext2_file_close,
   ext2_file_read,
   ext2_file_write,
   ext2_file_ioctl,

   ext2_init,
   ext2_format,
   ext2_mount,
   ext2_create_node,
   ext2_delete_node,
   ext2_truncate,
   ext2_umount
};

/*==================[external data definition]===============================*/

//Variable global. Se declarara extern en vfs.c
struct filesystem_driver ext2_driver =
{
   "EXT2",
   &ext2_driver_operations
};

/*==================[internal functions definition]==========================*/

static void assert_msg(int cond, char* msg, char * file, int line)
{
   if (cond)
   {
      /* assertion is ok */
      ciaaPOSIX_printf("OK: Assert in %s:%d\n", file, line);
   }
   else
   {
      ciaaPOSIX_printf("ERROR: Assert Failed in %s:%d %s\n", file, line, msg);
   }
}

/*Usar el test de block devices de la ciaa. Copiar los contenidos en filesystem y agregar test de ext2.*/

/*
 * Read a new inode into a file structure.
 */


/*
Donde almaceno la informacion del filesystem?
La informacion que tenga que almacenar va a depender del filesystem (ext2, fat, etc.). Entonces voy a tener que tener un struct distinto por cada filesystem. Ej: struct ext2_fs_info, struct fat32_fs_info, etc.
Estas estructuras me sirven para saber de donde sacar la informacion del disco. Por ejemplo en ext2 voy a necesitar guardar en esta estructura informacion del superblock y los group descriptor.

Voy a intentar implementarlo como esta implementada la capa de block devices. Va a ser similar pero una capa mas arriba.
Esta estructura va en void * layer de la ciaaDevices_deviceType del flash.
typedef struct {
   char const * filename;                 // <= Pointer to file name
   uint32_t position;                     // <= Courrent position
   FILE * storage;                        // <= Pointer to file storage
} ciaaDriverFlash_flashType;

Tengo que tener una estructura similar para poner en el layer de la ciaaFilesystems_ext2Type (TODO).

typedef struct ext2_fs_info {
   struct ext2sb e2sb;
   struct   ext2_gd *e2fs_gd; // group descripors 
   // The following items are only used when the super_block is in memory.
   int32_t s_bshift;   // ``lblkno'' calc of logical blkno
   int32_t s_bmask;   // ``blkoff'' calc of blk offsets
   int64_t s_qbmask;   // ~fs_bmask - for use with quad size
   int32_t   s_fsbtodb;   // fsbtodb and dbtofsb shift constant
   int32_t   s_ncg;   // number of cylinder groups
   uint32_t   s_block_size;           // block size in bytes.
   uint32_t   s_inodes_per_block;     // Number of inodes per block
   uint32_t   s_itb_per_group;        // Number of inode table blocks per group
   uint32_t   s_gdb_count;            // Number of group descriptor blocks
   uint32_t   s_desc_per_block;       // Number of group descriptors per block
   uint32_t   s_groups_count;         // Number of groups in the fs
   size_t     s_page_count;         // Number of pages of embox for file r/w buffer
   uint16_t      s_sectors_in_block;     // s_block_size / 512
   uint32_t   s_bsearch;              // all data blocks  below this block are in use
   uint8_t       s_blocksize_bits;       // Used to calculate offsets (e.g. inode block), always s_log_block_size + 10.
   char mntto[EXT2_PATH_MAX];
} ext2_fs_info_t;


Para leer los group descriptor tengo que pedir memoria dinamica. Lo pido por fuera de la funcion o dentro de la funcion?
Opcion 1:


*/

/*
Obtener descriptores de grupo.
La cantidad de descriptores de grupo depende del tamanio del dispositivo, por lo que hay que inicializar la memoria anteriormente.
Idea: Primero se lee el superblock, ya que tiene tamanio fijo. Luego se lee el campo donde indica cuantos descriptores de grupo hay.
Sabiendo cuantos descriptores de grupo hay, se hace malloc y se leen alli los gd.
ext2_buff_alloc(dir_nas, sizeof(struct ext2_gd) * fsi->s_ncg).
Donde se consigue fsi->s_ncg? En ext2_readsblock(). Lo calcula como
((s_blocks_count-s_first_data_block)+(s_blocks_per_group-1))/s_blocks_per_group
Otra forma de calcularlo: s_inodes_count/s_inodes_per_group.
*/

/*
Tengo que crear la funcion ext2_mount.  Montar un filesystem es poner en memoria los datos del mount que aparecen en el superblock y metadata. Un mount lleva asociados un device, un filesystem (implicito) y un directorio destino donde se monta. Ya tengo definido un device y un filesystem, pero no tengo definido un directorio destino, ya que no tengo un vfs.
Opcion 1: Tratar de copiar embox
Opcion 2: Tratar de ver como estan implementados los directorios en ciaa y hacer algo relacionado
Opcion 3: Tirar estructuras al aire donde recoger la informacion

int kmount(const char *dev, const char *dir, const char *fs_type) //Funcion del vfs kernel
int ext2fs_mount(void *dev, void *dir)   //Funcion del ext2fs
drv = fs_driver_find_drv(fs_type);   //De donde saca el driver? Ya estaba instalado y lo busca en el kernel
drv->fsop->mount(dev_node.node, root_path.node)
Ej: fsdrv->fsop->mount("/dev", NULL); (Ver rootfs.c en src/fs/)
dev es el nodo del device o el path del device segun una bandera del driver. Dir es el directorio destino donde se va a montar el fs.
mount(FS_DEV, FS_DIR, FS_NAME)
#define FS_DEV  "/dev/ramdisk"; #define FS_DIR    "/tmp"; #define FS_NAME  "vfat"

struct filesystem
{
   struct fs_driver *driver;             // Metodos del filesystem
   struct block_dev *bdevice;            // Dispositivo en el que reside el filesystem
   void             *fs_info;             // Informacion dependiente del driver utilizado
}

Que necesito de dev y dir? (argumentos de ext2fs_mount()). Se justifica pasar struct node como argumento? VER CODIGO

Decision 1: Disenho el sistema sin considerar que despues voy a poner un vfs.
int ext2fs_mount(ciaaDevices_deviceType const * const device, void *dir)
Tendria que tener una estructura que represente un mount.
Tendria que tener un registro de los mounts hechos.

ext2fs_mount() me tiene que devolver alguna estructura o dato para despues utilizarlo con open, seek, etc.

int ext2fs_format(void *dev). dev es un nodo.
*/


int ext2_get_superblock(ciaaDevices_deviceType const * device, struct ext2sb * sb_p)
{
   int32_t ret;
   ret = ciaaBlockDevices_lseek(device, EXT2_SBOFF, SEEK_SET);
   if(ret!=EXT2_SBOFF)
   {
      ciaaPOSIX_printf("ext2_get_superblock: Fallo seek\n");
      return -1;
   }
   ret = ciaaBlockDevices_read(device, (uint8_t *)sb_p, EXT2_SBSIZE);
   if(ret!=EXT2_SBSIZE)
   {
      ciaaPOSIX_printf("ext2_get_superblock: Fallo read\n");
      return -1;
   }
   return 0;
}

static void print_superblock(struct ext2sb * superblock)
{
   ciaaPOSIX_printf("s_inodes_count: %d\n", superblock->s_inodes_count);
   ciaaPOSIX_printf("s_blocks_count: %d\n", superblock->s_blocks_count);
   ciaaPOSIX_printf("s_r_blocks_count: %d\n", superblock->s_r_blocks_count);
   ciaaPOSIX_printf("s_free_blocks_count: %d\n", superblock->s_free_blocks_count);
   ciaaPOSIX_printf("s_free_inodes_count: %d\n", superblock->s_free_inodes_count);
   ciaaPOSIX_printf("s_first_data_block: %d\n", superblock->s_first_data_block);
   ciaaPOSIX_printf("s_log_block_size: %d\n", superblock->s_log_block_size);
   ciaaPOSIX_printf("s_log_frag_size: %d\n", superblock->s_log_frag_size);
   ciaaPOSIX_printf("s_blocks_per_group: %d\n", superblock->s_blocks_per_group);
   ciaaPOSIX_printf("s_frags_per_group: %d\n", superblock->s_frags_per_group);
   ciaaPOSIX_printf("s_inodes_per_group: %d\n", superblock->s_inodes_per_group);
   ciaaPOSIX_printf("s_mtime: %d\n", superblock->s_mtime);
   ciaaPOSIX_printf("s_wtime: %d\n", superblock->s_wtime);
   ciaaPOSIX_printf("s_mnt_count: %d\n", superblock->s_mnt_count);
   ciaaPOSIX_printf("s_max_mnt_count: %d\n", superblock->s_max_mnt_count);
   ciaaPOSIX_printf("s_magic: %d\n", superblock->s_magic);
   ciaaPOSIX_printf("s_state: %d\n", superblock->s_state);
   ciaaPOSIX_printf("s_errors: %d\n", superblock->s_errors);
   ciaaPOSIX_printf("s_minor_rev_level: %d\n", superblock->s_minor_rev_level);
   ciaaPOSIX_printf("s_lastcheck: %d\n", superblock->s_lastcheck);
   ciaaPOSIX_printf("s_checkinterval: %d\n", superblock->s_checkinterval);
   ciaaPOSIX_printf("s_creator_os: %d\n", superblock->s_creator_os);
   ciaaPOSIX_printf("s_rev_level: %d\n", superblock->s_rev_level);
   ciaaPOSIX_printf("s_def_resuid: %d\n", superblock->s_def_resuid);
   ciaaPOSIX_printf("s_def_resgid: %d\n", superblock->s_def_resgid);
   ciaaPOSIX_printf("s_first_ino: %d\n", superblock->s_first_ino);
   ciaaPOSIX_printf("s_inode_size: %d\n", superblock-> s_inode_size);
   ciaaPOSIX_printf("s_block_group_nr: %d\n", superblock->s_block_group_nr);
   ciaaPOSIX_printf("s_feature_compat: %d\n", superblock->s_feature_compat);
   ciaaPOSIX_printf("s_feature_incompat: %d\n", superblock->s_feature_incompat);
   ciaaPOSIX_printf("s_feature_ro_compat: %d\n", superblock->s_feature_ro_compat);
}

static void print_groupdescriptor(struct ext2_gd *gd)
{

   ciaaPOSIX_printf("\n**Group Descriptor**\n");
   ciaaPOSIX_printf("block_bitmap: %d\n",gd->block_bitmap);     /* Blocks bitmap block */
   ciaaPOSIX_printf("inode_bitmap: %d\n",gd->inode_bitmap);     /* Inodes bitmap block */
   ciaaPOSIX_printf("inode_table: %d\n",gd->inode_table);      /* Inodes table block */
   ciaaPOSIX_printf("free_blocks_count: %d\n",gd->free_blocks_count);   /* Free blocks count */
   ciaaPOSIX_printf("free_inodes_count: %d\n",gd->free_inodes_count);   /* Free inodes count */
   ciaaPOSIX_printf("used_dirs_count: %d\n",gd->used_dirs_count);     /* Directories count */

}

static void print_inode(struct ext2fs_dinode * inode)
{
   int i;
   ciaaPOSIX_printf("i_mode: %d\n", inode->i_mode);
   ciaaPOSIX_printf("i_uid: %d\n", inode->i_uid);
   ciaaPOSIX_printf("i_size: %d\n", inode->i_size);
   ciaaPOSIX_printf("i_atime: %d\n", inode->i_atime);
   ciaaPOSIX_printf("i_ctime: %d\n", inode->i_ctime);
   ciaaPOSIX_printf("i_mtime: %d\n", inode->i_mtime);
   ciaaPOSIX_printf("i_dtime: %d\n", inode->i_dtime);
   ciaaPOSIX_printf("i_gid: %d\n", inode->i_gid);
   ciaaPOSIX_printf("i_links_count: %d\n", inode->i_links_count);
   ciaaPOSIX_printf("i_blocks: %d\n", inode->i_blocks);
   ciaaPOSIX_printf("i_flags: %d\n", inode->i_flags);
   for(i=0; i<N_DIRECT_BLOCKS + N_INDIRECT_BLOCKS; i++)
      ciaaPOSIX_printf("Block %d: %04x\n",i, (inode->i_block)[i]);     /* blocks */
   ciaaPOSIX_printf("i_gen: %d\n",inode->i_gen);/* 100: generation number */
}
// Read a new inode into a file structure.
static int ext2_read_inode(vnode_t *dest_node, uint32_t inumber)
{
   size_t ret;
   int32_t inode_group, inode_offset;
   struct ext2fs_dinode * disk_inode_p;
   struct ext2_file_info * f_info;
   struct ext2_fs_info * fs_info;
   ciaaDevices_deviceType const *bdev;

   fs_info = (struct ext2_fs_info *) dest_node->fs_info.down_layer_info;
   bdev = dest_node->fs_info.device;

   if(fs_info==NULL)
   {
      ciaaPOSIX_printf("ext2_read_inode: El filesystem no esta mounted\n");
      return -1;
   }
   if(bdev==NULL)
   {
      ciaaPOSIX_printf("ext2_read_inode: No hay dispositivo para el archivo\n");
      return -1;
   }
   //ciaaPOSIX_printf("print_superblock: inside ext2_read_inode\n");
   //print_superblock(&fs_info->e2sb);
   //Division por 0. Conflictivo. Verificar antes
   if(fs_info->e2sb.s_inodes_per_group==0)
   {
      ciaaPOSIX_printf("ext2_read_inode: Invalid superblock\n");
      return -1;
   }
   //Calculo la posicion del inode dentro del device a partir de la informacion del fs
   inode_group = (inumber-1)/fs_info->e2sb.s_inodes_per_group;   //(inumber-1) porque se empiezan a contar desde 1.
                           //Calculo en que grupo esta el inode destino

   inode_offset = EXT2_SBOFF + ((((fs_info->e2fs_gd)[inode_group]).inode_table)-1)*(fs_info->s_block_size) +
         ((inumber-1)%fs_info->e2sb.s_inodes_per_group)*(fs_info->e2sb.s_inode_size);

   if(dest_node->f_info.down_layer_info ==  NULL)
      dest_node->f_info.down_layer_info = ciaaPOSIX_malloc(sizeof(struct ext2_file_info));
   f_info = (struct ext2_file_info *)dest_node->f_info.down_layer_info;
   if(f_info==NULL)
   {
      ciaaPOSIX_printf("ext2_read_inode: Fallo ciaa_malloc para finfo\n");
      return -1;
   }
   //Cargo el inode en el downlayer del node_info, representa la metadata fisica del nodo
   disk_inode_p = (struct ext2fs_dinode *) ciaaPOSIX_malloc(sizeof(struct ext2fs_dinode));
   if(dest_node->n_info.down_layer_info == NULL)
      dest_node->n_info.down_layer_info = ciaaPOSIX_malloc(sizeof(struct ext2fs_dinode));
   //disk_inode_p = (struct ext2fs_dinode *) dest_node->n_info.down_layer_info;
   if(disk_inode_p==NULL)
   {
      ciaaPOSIX_free(dest_node->f_info.down_layer_info);
      dest_node->f_info.down_layer_info = NULL;
      ciaaPOSIX_printf("ext2_read_inode: Fallo ciaa_malloc para dinode\n");
      return -1;
   }

   ret = ciaaBlockDevices_lseek(bdev, inode_offset, SEEK_SET);
   if(ret!=inode_offset)
   {
      ciaaPOSIX_printf("ext2_read_inode: Fallo lseek\n");
      ciaaPOSIX_free(disk_inode_p);
      ciaaPOSIX_free(f_info);
      return -1;
   }
   ret = ciaaBlockDevices_read(bdev, (uint8_t *)disk_inode_p, sizeof(struct ext2fs_dinode));
   if(ret!=sizeof(struct ext2fs_dinode))
   {
      ciaaPOSIX_printf("ext2_read_inode: Fallo read\n");
      ciaaPOSIX_free(dest_node->n_info.down_layer_info);
      dest_node->n_info.down_layer_info = NULL;
      ciaaPOSIX_free(dest_node->f_info.down_layer_info);
      dest_node->f_info.down_layer_info = NULL;
      return -1;
   }
   f_info->f_inumber = inumber;
   dest_node->n_info.down_layer_info = (void *) disk_inode_p;
   ciaaPOSIX_memcpy(&(f_info->f_di), disk_inode_p, sizeof(struct ext2fs_dinode));
   dest_node->f_info.down_layer_info = (void *) f_info;
   ciaaPOSIX_printf("\n\next2_read_inode: inode obtenido:\n");
   print_inode(dest_node->n_info.down_layer_info);
   return 0;
}



 // Search a directory for a name and return its
 // inode number.

//7 de noviembre: Mande fruta, no tiene sentido leer en file_offset. Tengo que utilizar ext2_buf_read_file().
static int ext2_search_directory(vnode_t *dir_node, const char *dir_name, int length,
      uint32_t *inumber_p)
{
   uint8_t *data_pointer;
   struct   ext2fs_direct * start_pointer, * end_pointer;
   //int ret;
   int i;
   uint32_t lret; //Para numeros grandes

   //ciaaDevices_deviceType const *bdev;
   struct ext2fs_dinode * ninfo;
   struct ext2_file_info *finfo;
   struct ext2_fs_info *fsinfo;

   //bdev = dir_node->fs_info.device;
   ninfo = (struct ext2fs_dinode *)dir_node->n_info.down_layer_info;
   finfo = (struct ext2_file_info *)dir_node->f_info.down_layer_info;
   fsinfo = (struct ext2_fs_info *)dir_node->fs_info.down_layer_info;

   if(ninfo==NULL||finfo==NULL||fsinfo==NULL)
   {
      ciaaPOSIX_printf("ext2_search_directory: El archivo no fue correctamente abierto. Abort\n");
      return -1;
   }
   finfo->f_pointer = 0;
   //Mientras el puntero sea menor al tamanio del archivo
   data_pointer=(uint8_t*)ciaaPOSIX_malloc(ninfo->i_size);//TODO: Hago malloc del tamanio del archivo entero
                              //Mucho malloc
   if(data_pointer==NULL)
   {
      ciaaPOSIX_printf("ext2_search_directory: Fallo malloc\n");
   }
   ext2_buf_read_file(dir_node, (uint8_t*)data_pointer, ninfo->i_size, &lret);
   if(lret!=ninfo->i_size)
   {
      ciaaPOSIX_printf("ext2_search_directory: Fallo ext2_buf_read_file\n");
   }
   for(i=0; i<(ninfo->i_size);i++)
      ciaaPOSIX_printf("%c",data_pointer[i]);
   for(   start_pointer=(struct ext2fs_direct *)data_pointer,
      end_pointer=(struct ext2fs_direct *)(data_pointer + ninfo->i_size);
      start_pointer<end_pointer;
      start_pointer=(struct ext2fs_direct *)((char*)start_pointer + start_pointer->e2d_reclen))//Ver aritmetica de punteros, puede estar mal.
   {
      if(start_pointer->e2d_reclen <= 0)
      {
         break;
      }
      if(start_pointer->e2d_ino == 0)
      {
         continue;
      }
      if(   start_pointer->e2d_namlen == length &&
         !ciaaPOSIX_memcmp(dir_name, start_pointer->e2d_name, length))
      {
         //Se encontro el directorio
         *inumber_p=start_pointer->e2d_ino;
         ciaaPOSIX_free(data_pointer);
         return 0;
      }
   }

   ciaaPOSIX_free(data_pointer);
   return -1; //Error, no se encontro directorio
}

static node_type_t ext2type_to_vfstype(uint8_t ext2_type)
{
   switch (ext2_type) {
   case EXT2_FT_REG_FILE: return VFS_FTREG;
   case EXT2_FT_DIR: return VFS_FTDIR;
   default: return VFS_FTUNKNOWN;
   }
}


static int ext2_close(vnode_t * dest_node) {
   struct ext2_file_info *finfo;

   finfo = dest_node->f_info.down_layer_info;

   if (finfo != NULL)
   {
      ciaaPOSIX_free(finfo);      
   }

   return 0;
}


static int ext2_file_open(struct file_desc *file)
{
   int ret;
   struct ext2_file_info *finfo;
   //struct ext2_fs_info *fsinfo;

   //fsinfo=file->node->fs_info.down_layer_info;
   finfo=file->node->f_info.down_layer_info;

   /* reset seek pointer */

   ret = ext2_read_inode(file->node, finfo->f_inumber);
   if(ret)
   {
      return -1;
   }

   return 0;
}

static int ext2_file_close(struct file_desc *file)
{
   //No hago nada. En embox solo libera el buffer
   return 0;
}

static size_t ext2_file_read(struct file_desc *file, void *buf, size_t size)
{
   uint32_t read_size;
   //struct ext2_file_info *finfo;
   int ret;

   //finfo = file->node->f_info.down_layer_info;
   /*TODO: Verificar que file->cursor sea valido*/
   file->node->f_info.file_pointer = file->cursor;
   ciaaPOSIX_printf("ext2_file_read(): Cursor antes de read: %d\n", file->cursor);
   ret = ext2_buf_read_file(file->node, (uint8_t *)buf, size, &read_size);
   if(ret)
   {
      ciaaPOSIX_printf("ext2_file_read(): Hubo problema con ext2_buf_read_file\n");
      return 0;
   }
   file->cursor += read_size;
   ciaaPOSIX_printf("ext2_file_read(): Cursor despues de read: %d\n", file->cursor);
   return read_size;
}

static size_t ext2_file_write(struct file_desc *file, void *buf, size_t size)
{
   //TODO
   return 0;
}

static int ext2_file_ioctl(struct file_desc *file, int request)
{
   return 0;
}

static int ext2_init(void *par)
{
   return 0;
}

/*
Here are the rules used to allocate new inodes:

the inode for a new file is allocated in the same group of the inode of its parent directory.
inodes are allocated equally between groups.
Here are the rules used to allocate new blocks:

a new block is allocated in the same group as its inode.
allocate consecutive sequences of blocks.
*/
/*
Determining the Number of Block Groups
From the Superblock, extract the size of each block, the total number of inodes, the total number of blocks, the number of blocks per block group, and the number of inodes in each block group. From this information we can infer the number of block groups there are by:
Rounding up the total number of blocks divided by the number of blocks per block group
Rounding up the total number of inodes divided by the number of inodes per block group
Both (and check them against each other)
*/
/*
Determining the number of inodes
El numero de nodos se debe indicar manualmente por un ratio de blocks/inodes
En linux, mke2fs da un ratio predeterminado de 4
*/
/*
El bitmap de bloque de un grupo esta limitado a solo 1 bloque, entonces:
La cantidad de bloques por grupo es la cantidad de bits en un bloque.
Para el caso de un bloque de 1024 bytes, la cantidad de bloques es 8*1024.
   blocks_per_cg = sblock.e2fs_bsize * NBBY;
   ncg = howmany(bcount - sblock.e2fs.e2fs_first_dblock, blocks_per_cg);
   blocks_gd = howmany(sizeof(struct ext2_gd) * ncg, bsize);
*/
/*
// Bit map related macros. 
#define	setbit(a,i)	((a)[(i)/NBBY] |= 1<<((i)%NBBY))
#define	clrbit(a,i)	((a)[(i)/NBBY] &= ~(1<<((i)%NBBY)))
#define	isset(a,i)	((a)[(i)/NBBY] & (1<<((i)%NBBY)))
#define	isclr(a,i)	(((a)[(i)/NBBY] & (1<<((i)%NBBY))) == 0)

// Macros for counting and rounding. 
#ifndef howmany
#define	howmany(x, y)	(((x)+((y)-1))/(y))
#endif
#define	roundup(x, y)	((((x)+((y)-1))/(y))*(y))
#define	rounddown(x,y)	(((x)/(y))*(y))
#define	roundup2(x, m)	(((x) + (m) - 1) & ~((m) - 1))
#define	powerof2(x)	((((x)-1)&(x))==0)

// Macros for min/max. 
#define	MIN(a,b)	((/\*CONSTCOND*\/(a)<(b))?(a):(b))
#define	MAX(a,b)	((/\*CONSTCOND*\/(a)>(b))?(a):(b))

#define group_base(c) (superblock->s_first_data_block + superblock->s_blocks_per_group * (c))
*/

static int ext2_format(vnode_t *dev_node)
{
   int ret;
   ciaaDevices_deviceType const *bdev;
   //struct ext2fs_dinode * ninfo;
   //struct ext2_file_info *finfo;
   //struct ext2_fs_info *fsinfo;
   ciaaDevices_blockType blockInfo;

   struct ext2sb superblock;
   struct ext2_gd *gd_buffer;
   uint8_t *bitmap_buffer;
   struct ext2fs_dinode node;
   struct ext2fs_direct dir;
   
   uint16_t i,j,aux,group_index;
   uint16_t ngroups, nblocks_gd, inodes_per_group, inodeblocks_per_group,
            minmetablocks_per_group,  nblocks_last_group, dev_bsize, nblocks_group_overhead;
   uint32_t dev_size, block_offset, free_blocks_count, free_inodes_count;
   uint32_t group_offset, blocks_per_group;
   //ninfo=dev_node->n_info.down_layer_info;
   //finfo=dev_node->f_info.down_layer_info;
   //fsinfo=dev_node->fs_info.down_layer_info;
   bdev = dev_node->fs_info.device;

   if(dev_node->n_info.type != VFS_FTBLK)
   {
      return -1;   //Solo puedo formatear un dispositivo
   }

   ret = ciaaBlockDevices_ioctl(bdev, ciaaPOSIX_IOCTL_BLOCK_GETINFO, &blockInfo);
   if(ret<0)
   {
      return -1;
   }

   /* Uso el tamano del bloque fisico como el tamano del bloque del fs */
   dev_bsize=blockInfo.blockSize;
   if(dev_bsize < 1024)   /* Tamano minimo del bloque de ext2 */
      dev_bsize = 1024;
   dev_size=blockInfo.lastPosition;

   /* Inicializo los contenidos del superblock default */
   ciaaPOSIX_memset((uint8_t *)&superblock, 0, sizeof(struct ext2sb));

   for(aux=dev_bsize>>10, i=0;aux;i++, aux=aux>>1);   /*Calculo el logaritmo del blocksize */
   superblock.s_log_block_size = i;
   superblock.s_first_data_block = (dev_bsize > EXT2_SBOFF) ? 0 : 1;  /* First Data Block */
   superblock.s_blocks_per_group = dev_bsize*8;  /* # Blocks per group */
   superblock.s_frags_per_group = superblock.s_blocks_per_group;   /* # Fragments per group */
   superblock.s_wtime = 0;             /* Write time */
   superblock.s_mnt_count = 0;            /* Mount count */
   superblock.s_max_mnt_count = 0;        /* Maximal mount count */
   superblock.s_magic = 0xEF53;                /* Magic signature */
   superblock.s_state = 1;                /* File system state. EXT2_VALID_FS */
   superblock.s_errors = 1;               /* Behaviour when detecting errors. EXT2_ERRORS_CONTINUE */
   superblock.s_lastcheck = 0;         /* time of last check */
   superblock.s_rev_level = 1;         /* Revision level */
   superblock.s_first_ino = 11;         /* First non-reserved inode. EXT2_GOOD_OLD_FIRST_INO */
   superblock.s_inode_size = 128;           /* size of inode structure. EXT2_GOOD_OLD_INODE_SIZE */
   superblock.s_block_group_nr = 0;       /* block group # of this superblock */
   superblock.s_feature_compat = 0x00;    /* compatible feature set */
   superblock.s_feature_incompat = 0x1F;  /* incompatible feature set */
   superblock.s_feature_ro_compat = 1;  /* readonly-compatible feature set */
   superblock.s_uuid[0] = 0;   /* 128-bit uuid for volume */
   superblock.s_uuid[1] = 0;
   superblock.s_uuid[2] = 0;
   superblock.s_uuid[3] = 0;
   superblock.s_uuid[4] = 0;
   superblock.s_uuid[5] = 0;
   superblock.s_uuid[6] = 0;
   superblock.s_uuid[7] = 0;
   superblock.s_uuid[8] = 0;
   superblock.s_uuid[9] = 0;
   superblock.s_uuid[10] = 0;
   superblock.s_uuid[11] = 0;
   superblock.s_uuid[12] = 0;
   superblock.s_uuid[13] = 0;
   superblock.s_uuid[14] = 0;
   superblock.s_uuid[15] = 0;
   ciaaPOSIX_strcpy((char *) superblock.s_volume_name, "ext2");      /* volume name */

   /* Total blocks */
   superblock.s_blocks_count = dev_size / dev_bsize;
   /* Factor 4 */
   superblock.s_inodes_count = superblock.s_blocks_count / 4;
   /* Reserved blocks count */
   superblock.s_r_blocks_count = 0;
   /* Cantidad de grupos de bloques */
   ngroups = howmany(superblock.s_blocks_count - superblock.s_first_data_block,
                     superblock.s_blocks_per_group);
   /* Cantidad de bloques por grupo */
   blocks_per_group = dev_bsize*8;
   /* Cantidad de bloques que ocupan los group descriptors en total por cada grupo */
   nblocks_gd = howmany(sizeof(struct ext2_gd)*ngroups, dev_bsize);
   /* Cantidad de nodos que entran en un bloque */
   inodes_per_group = superblock.s_inodes_count / ngroups;
   /* Cantidad de bloques que ocupan los nodos en total por cada grupo */
   inodeblocks_per_group = howmany(superblock.s_inode_size * inodes_per_group, dev_bsize);
   /* Cantidad de bloques de prefijo de cada grupo */
   minmetablocks_per_group = 1 /*Superblock*/ + nblocks_gd + 1 /*Block bitmap*/ +
                     1 /*Inode bimap*/ + inodeblocks_per_group + 1 /*At least 1 data block*/;
   /* El ultimo grupo tiene en general menos bloques que el resto, son los bloques que sobran */
   nblocks_last_group = superblock.s_blocks_count - superblock.s_first_data_block -
                        superblock.s_blocks_per_group * (ngroups - 1);

   /* Si el tamanio del ultimo grupo es demasiado pequenio, lo elimino */
   if(nblocks_last_group < minmetablocks_per_group)
   {
      superblock.s_blocks_count -= nblocks_last_group;
      ngroups--;
      nblocks_last_group = superblock.s_blocks_per_group;
      nblocks_gd = howmany(sizeof(struct ext2_gd)*ngroups, dev_bsize);
      inodes_per_group = superblock.s_inodes_count / ngroups;
   }

   /* La cantidad de nodos dentro de un grupo debe ser multiplo de 8 para entrar en bloques enteros */
   superblock.s_inodes_per_group = rounddown(superblock.s_inodes_per_group,8);
   /* Cantidad total de nodos */
   superblock.s_inodes_count = inodes_per_group * ngroups;
   /* Cantidad de bloques que ocupan los nodos en cada bloque */
   inodeblocks_per_group = howmany(superblock.s_inode_size * inodes_per_group, dev_bsize);
   /* Cantidad de bloques de overhead por grupo*/
   nblocks_group_overhead = 1/*Superblock*/ + nblocks_gd + 1 /*Blockbitmap*/+ 1 /*Inodebitmap*/+ inodeblocks_per_group;

   superblock.s_free_inodes_count = superblock.s_inodes_count - 11; /* Incluyo el root como reservado */

   //Inicializo group descriptors. Los preparo en un buffer
   gd_buffer = (struct ext2_gd *) ciaaPOSIX_malloc(sizeof(struct ext2_gd)*ngroups);
   if(gd_buffer==NULL)
   {
      return -1;
   }
   for(group_index=0; group_index<ngroups; group_index++)
   {
      block_offset = superblock.s_first_data_block + superblock.s_blocks_per_group*group_index;
      block_offset += 1 /*Superblock*/+ nblocks_gd;
      gd_buffer[group_index].block_bitmap = block_offset;
      block_offset += 1; /*Block bitmap*/
      gd_buffer[group_index].inode_bitmap = block_offset;
      if(group_index == ngroups-1)
         gd_buffer[group_index].free_blocks_count = nblocks_last_group -
                                                      minmetablocks_per_group /*Overhead*/+ 1;
      else
         gd_buffer[group_index].free_blocks_count = nblocks_gd - minmetablocks_per_group /*Overhead*/+ 1;
      free_blocks_count += gd_buffer[group_index].free_blocks_count;
      if(group_index == 0)
         gd_buffer[group_index].free_inodes_count = inodes_per_group - EXT2_RESERVED_INODES;
      else
         gd_buffer[group_index].free_inodes_count = inodes_per_group;

      free_inodes_count += gd_buffer[group_index].free_inodes_count;
      gd_buffer[group_index].used_dirs_count = 0;
   }
   /* Si hay nodos reservados, estos numeros cambian */
   superblock.s_free_inodes_count = free_inodes_count;
   superblock.s_free_blocks_count = free_blocks_count;

   //Ya tengo preparados el superblock y los gd, solo me queda escribir los buffer en orden en el disco
   for(group_index=0; group_index<ngroups; group_index++)
   {
      group_offset = EXT2_SBOFF + superblock.s_blocks_per_group * dev_bsize * group_index;	//Es confuso, pero parece correcto.
      //Si el block size es 1024, block 0 es boot, block 1 es superblock. Blocks 1 a 8192 son blocks del group 1
      //Por lo tanto el boot block no cuenta como block del group 1.

      ret = ciaaBlockDevices_lseek(bdev, group_offset, SEEK_SET);
      if(ret)
      {
         ciaaPOSIX_free(gd_buffer);
         return -1;
      }
      ret = ciaaBlockDevices_write(bdev, (uint8_t *)&superblock, sizeof(struct ext2sb));
      if(ret)
      {
         ciaaPOSIX_free(gd_buffer);
         return -1;
      }
      ret = ciaaBlockDevices_lseek(bdev, group_offset + EXT2_SBSIZE, SEEK_SET);
      if(ret)
      {
         ciaaPOSIX_free(gd_buffer);
         return -1;
      }
      ret = ciaaBlockDevices_write(bdev, (uint8_t *)gd_buffer, sizeof(struct ext2_gd) * ngroups);
      if(ret)
      {
         ciaaPOSIX_free(gd_buffer);
         return -1;
      }
   }
   //Falta completar block bitmap e inode bitmap. Va a ser medio dificil porque tengo que setear bits
   //y solo puedo escribir bytes. La solucion es escribir sobre un buffer y despues escribir en el disco.
   //Tener un buffer del tamanio de un bloque podria ser conveniente, ya que los bitmaps ocupan exact. eso.

   //Pido memoria del tamanio de un bloque
   bitmap_buffer = (uint8_t *) ciaaPOSIX_malloc(dev_bsize);
   if(bitmap_buffer==NULL)
   {
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }

   /* Block bitmap */
   for(group_index=0; group_index<ngroups; group_index++)
   {
      ciaaPOSIX_memset(bitmap_buffer, 0, dev_bsize);
      //El ultimo grupo en general tiene menos bloques que el resto
      //Tengo que setear los bits de los bloques que no existen.nblocks_last_group bits
      //Seteo primero los primeros nblocks_last_group/8 bytes
      //Si hay 9 bloques, tengo que setear los primeros 9/8=1 bytes, despues los bits desde 8*1 hasta 9-1
      //Si hay 7 bloques, tengo que setear los primeros 7/8=0 bytes, despues los bits desde 8*0 hasta 7-1
      if(group_index == ngroups-1)
      {
         for(i=nblocks_last_group; i%8!=0; i++)
         {
            setbit(bitmap_buffer, i);
         }
         for(i=i/8; i<blocks_per_group/8; i++)
         {
            bitmap_buffer[i] = 0xFF;
         }
      }
      for(i=0; i<nblocks_group_overhead/8; i++)
      {
         bitmap_buffer[i] = 0xFF;
      }
      i*=8;
      for(i=0; i<nblocks_group_overhead;i++)
      {
         setbit(bitmap_buffer,i);
      }

      ret = ciaaBlockDevices_lseek(bdev, gd_buffer[group_index].block_bitmap * dev_bsize, SEEK_SET);
      if(ret)
      {
         ciaaPOSIX_free(bitmap_buffer);
         ciaaPOSIX_free(gd_buffer);
         return -1;
      }
      ret = ciaaBlockDevices_write(bdev, (uint8_t *)bitmap_buffer, dev_bsize);
      if(ret)
      {
         ciaaPOSIX_free(bitmap_buffer);
         ciaaPOSIX_free(gd_buffer);
         return -1;
      }
   }

   /* Inode bitmap */
   for(group_index=0; group_index<ngroups; group_index++)
   {
      i = inodes_per_group/8;
      ciaaPOSIX_memset(bitmap_buffer, 0, i);   /*Todos los inodes libres*/
      ciaaPOSIX_memset(bitmap_buffer+i, 0xFF, dev_bsize-i);   /*Estos inodes no existen, asi que los pongo ocupados*/
      /*Los nodos de 0 a 9 (primeros 10 nodos) son reservados. s_first_ino es 11, pero contando desde 1 a 11.*/
      if (group_index == 0)
      {
         /* mark reserved inodes */
         for (i = 0; i < EXT2_FIRSTINO-1; i++)
            setbit(bitmap_buffer, i);
      }

      ret = ciaaBlockDevices_lseek(bdev, gd_buffer[group_index].inode_bitmap * dev_bsize, SEEK_SET);
      if(ret)
      {
         ciaaPOSIX_free(bitmap_buffer);
         ciaaPOSIX_free(gd_buffer);
         return -1;
      }
      ret = ciaaBlockDevices_write(bdev, (uint8_t *)bitmap_buffer, dev_bsize);
      if(ret)
      {
         ciaaPOSIX_free(bitmap_buffer);
         ciaaPOSIX_free(gd_buffer);
         return -1;
      }
   }

   /* root directory */
   ciaaPOSIX_memset(&node, 0, sizeof(struct ext2fs_dinode));

   node.i_mode = 040755;
   node.i_uid = node.i_gid = 1000;
   node.i_size = 1024;
   node.i_atime = node.i_ctime = node.i_mtime = 1683851637;
   node.i_dtime = 0;
   node.i_links_count = 2;
   node.i_blocks = 2;
   /* Rescato el block bitmap del primer bloque para buscar un bloque libre donde poner el root */
   /* Este bloque libre va a ser el node.i_block[0] */
   ret = ciaaBlockDevices_lseek(bdev, gd_buffer[0].block_bitmap * dev_bsize, SEEK_SET);   /*Block bitmap*/
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }
   ret = ciaaBlockDevices_read(bdev, (uint8_t *)bitmap_buffer, dev_bsize);
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }
   for(i=0; i<superblock.s_blocks_per_group/8 && bitmap_buffer[i]!=0xFF; i++);   /*Busco el primer byte con un bit en 0*/
   if(i==superblock.s_blocks_per_group/8)
   {
      return -1;
   }
   for(j=0; j<8; j++)
   {
      if((bitmap_buffer[i] & (1<<j)) == 0)   /* El bit j del byte bitmap_buffer[i] es 0 */
      {                                    /* El bloque i*8+j estaba libre y ahora lo marco*/
         bitmap_buffer[i] |= (1<<j);
         break;
      }
   }
   /* Vuelvo a escribir el bitmap con el bloque ya reservado */
   ret = ciaaBlockDevices_lseek(bdev, gd_buffer[0].block_bitmap * dev_bsize, SEEK_SET);   /*Block bitmap*/
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }
   ret = ciaaBlockDevices_write(bdev, (uint8_t *)bitmap_buffer, dev_bsize);
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }

   node.i_block[0] = superblock.s_first_data_block + 8*i + j;
   gd_buffer[0].free_blocks_count--;
   gd_buffer[0].free_inodes_count--;
   gd_buffer[0].used_dirs_count++;

   /*Escribo los cambios hechos al group descriptor*/
   ret = ciaaBlockDevices_lseek(bdev, EXT2_GDOFF, SEEK_SET);   /*First group descriptor*/
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }
   ret = ciaaBlockDevices_write(bdev, (uint8_t *)&gd_buffer[0], sizeof(struct ext2fs_dinode));
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }

   ret = ciaaBlockDevices_lseek(bdev, gd_buffer[0].inode_table * dev_bsize +
         superblock.s_inode_size, SEEK_SET);   /*Second inode in table*/
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }
   ret = ciaaBlockDevices_write(bdev, (uint8_t *)&node, sizeof(struct ext2_gd));
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }

   /*Create root entry*/

   /*Self entry*/
   dir.e2d_ino = 2;
   dir.e2d_reclen = 12;
   dir.e2d_namlen = 1;
   dir.e2d_type = 2;
   ciaaPOSIX_strcpy(dir.e2d_name, ".");

   ret = ciaaBlockDevices_lseek(bdev, node.i_block[0]*dev_bsize, SEEK_SET);   /*First entry, "."*/
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }
   ret = ciaaBlockDevices_write(bdev, (uint8_t *)&dir, sizeof(struct ext2fs_direct));
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }

   ret = ciaaBlockDevices_lseek(bdev, node.i_block[0]*dev_bsize + dir.e2d_reclen, SEEK_SET);   /*Second entry, ".."*/
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }

   /*Parent entry*/
   dir.e2d_ino = 2;
   dir.e2d_reclen = 1012;
   dir.e2d_namlen = 2;
   dir.e2d_type = 2;
   ciaaPOSIX_strcpy(dir.e2d_name, "..");

   ret = ciaaBlockDevices_write(bdev, (uint8_t *)&dir, sizeof(struct ext2fs_direct));
   if(ret)
   {
      ciaaPOSIX_free(bitmap_buffer);
      ciaaPOSIX_free(gd_buffer);
      return -1;
   }
   ciaaPOSIX_free(bitmap_buffer);

   return 0;
}

//dest_node ya esta creado. Hay que transferir la informacion de fs y file
static int ext2_mount(vnode_t *dev_node, vnode_t *dest_node)
{
   //FIXME
   int ret;
   ciaaDevices_deviceType const *bdev;
   struct ext2fs_dinode * ninfo;
   struct ext2_file_info *finfo;
   struct ext2_fs_info *fsinfo;

   //Tengo que llenar informacion para el root inode?
   finfo=dest_node->f_info.down_layer_info=(struct ext2_file_info *) ciaaPOSIX_malloc(sizeof(struct ext2_file_info));
   ASSERT_MSG(finfo!=NULL, "ext2_mount(): Mem error, could not alloc finfo");
   if(finfo == NULL)
   {
      return -1;
   }
   ninfo=dest_node->n_info.down_layer_info=(struct ext2fs_dinode *) ciaaPOSIX_malloc(sizeof(struct ext2fs_dinode));
   ASSERT_MSG(ninfo!=NULL, "ext2_mount(): Mem error, could not alloc ninfo");
   if(ninfo == NULL)
   {
      return -1;
   }
   fsinfo=dest_node->fs_info.down_layer_info=(struct ext2_fs_info *)ciaaPOSIX_malloc(sizeof(struct ext2_fs_info));
   ASSERT_MSG(fsinfo!=NULL, "ext2_mount(): Mem error, could not alloc fsinfo");
   if(fsinfo == NULL)
   {
      return -1;
   }

   //Cargo la informacion del filesystem a partir del superblock
   bdev = dest_node->fs_info.device;
   ret = ext2_get_superblock(bdev, &(fsinfo->e2sb));
   ASSERT_MSG(ret>=0, "ext2_mount(): Could not get superblock");
   if(ret)
   {
      return -1;
   }
   //ciaaPOSIX_printf("print_superblock: inside ext2_mount\n");
   //print_superblock(&fsinfo->e2sb);
   //La cantidad de descriptores de grupo lo calculo a partir de otros datos del superblock.
   fsinfo->s_groups_count = (fsinfo->e2sb.s_inodes_count)/(fsinfo->e2sb.s_inodes_per_group); //Calculo la cantidad de grupos
   fsinfo->s_block_size = 1024 << fsinfo->e2sb.s_log_block_size;
   fsinfo->s_inodes_per_block = (fsinfo->s_block_size)/(fsinfo->e2sb.s_inode_size);

   fsinfo->e2fs_gd = (struct ext2_gd *)ciaaPOSIX_malloc(fsinfo->s_groups_count*sizeof(struct ext2_gd));
   ASSERT_MSG(fsinfo->e2fs_gd!=NULL, "ext2_mount(): Mem error, could not alloc gd");
   if(fsinfo->e2fs_gd==NULL)
   {
   return -1;
   }
   ret = ciaaBlockDevices_lseek(bdev, EXT2_GDOFF, SEEK_SET);
   ASSERT_MSG(ret==EXT2_GDOFF, "ext2_mount(): lseek gd failed");
   if(ret!=EXT2_GDOFF)
   {
   ciaaPOSIX_free(fsinfo->e2fs_gd);
   return -1;
   }
   ret = ciaaBlockDevices_read(bdev, (uint8_t *)fsinfo->e2fs_gd, fsinfo->s_groups_count*(sizeof(struct ext2_gd)));
   ASSERT_MSG(ret==fsinfo->s_groups_count*(sizeof(struct ext2_gd)), "ext2_mount(): read gd failed");
   if(ret!=fsinfo->s_groups_count*(sizeof(struct ext2_gd)))
   {
   ciaaPOSIX_free(fsinfo->e2fs_gd);
   return -1;
   }


   //Recibo un nodo ya creado por el vfs. Pero el vfs no puede crear la estructura del down_layer_info, tengo que llenarla yo
   ciaaPOSIX_printf("ext2_mount(): Entro a  ext2_mount_load()\n");
   //ciaaPOSIX_printf("print_superblock: inside ext2_mount\n");
   //print_superblock(&fsinfo->e2sb);
   ret = ext2_mount_load(dest_node);
   ASSERT_MSG(ret>=0, "ext2_mount(): mount load failed");
   return ret;
}
static int ext2_create_node(vnode_t *parent_node, vnode_t *child_node)
{
   return 0;
}
static int ext2_delete_node(vnode_t *node)
{
   return 0;
}
static int ext2_truncate(vnode_t *node, uint32_t length)
{
   return 0;
}
static int ext2_umount(vnode_t *node)
{
   return 0;
}

static int ext2_buf_read_file(vnode_t * dest_node, uint8_t *buf, uint32_t size, uint32_t *total_read_size_p) {
   //Si size>finfo->f_di.i_size tirar error o truncar? me voy a fijar en ext2fs_read()
   int ret;
   size_t i;
   uint32_t file_block, block_offset;
   uint32_t device_block, device_offset;
   uint16_t block_size, block_shift, block_mask;
   uint32_t total_remainder_size, block_remainder, read_size, buf_pos;
   struct ext2fs_dinode * ninfo;
   struct ext2_file_info *finfo;
   struct ext2_fs_info *fsinfo;
   ciaaDevices_deviceType const *device;

   ninfo = dest_node->n_info.down_layer_info;
   fsinfo = dest_node->fs_info.down_layer_info;
   finfo = dest_node->f_info.down_layer_info;
   device = dest_node->fs_info.device;

   block_size = fsinfo->s_block_size;
   //block_shift = (uint16_t)fsinfo->e2sb.s_log_block_size;   //Da 0, por que?. s_log_block_size es respecto a 1024
   block_shift = (uint16_t)fsinfo->e2sb.s_log_block_size + 10;
   block_mask = (uint32_t)block_size-1;
   file_block = finfo->f_pointer >> block_shift;
   block_offset = finfo->f_pointer & block_mask;

   ciaaPOSIX_printf(   "ext2_buf_read_file: Valores:\n"\
            "block_size: %d\nblock_shift: %d\nblock_mask: 0x%04x\n"\
            "file_block: %d\nblock_offset: %d\n",\
            block_size,block_shift,block_mask,file_block,block_offset);

   if(total_read_size_p!=NULL)
      *total_read_size_p = 0;

   /*Truncate read size*/
   if(finfo->f_pointer + size > ninfo->i_size)
      size = ninfo->i_size - finfo->f_pointer;

   /*Tengo que leer total_remainder_size bytes.
   No puedo leer todo junto, tengo que leerlo de a bloques.
   Primero tengo que buscar el bloque a partir de la posicion actual del archivo.
   Leer en la posicion de inicio de bloque mas el offset que corresponde offset%block_size.
   Solo puedo leer hasta el fin del bloque actual. Calculo cuantos bytes puedo leer y se los resto a total_remainder_size.
   Una vez leidos los bytes hasta el fin del bloque y copiados los datos al buffer, sumo la cantidad de bytes leidos al
   fpointer. Paso a la siguiente iteracion.*/

   /*Inicio: total_remainder_size = size.
   Condicion: total_remainder_size>0*/
   ciaaPOSIX_printf("ext2_buf_read_file: Comienza el loop\n");
   for(   total_remainder_size = size, buf_pos = 0, i = 0;
      total_remainder_size>0;)
   {
      ciaaPOSIX_printf("ext2_buf_read_file: Loop %d\n", i);
      file_block = finfo->f_pointer >> block_shift;
      block_offset = finfo->f_pointer & block_mask;
      ret = ext2_block_map(dest_node, file_block, &device_block);
      if(ret)
      {
         if(total_read_size_p!=NULL)
            *total_read_size_p = buf_pos;
         return ret;
      }
      device_offset = (device_block << block_shift) + block_offset;

      block_remainder = block_size - block_offset;
      if(total_remainder_size > block_remainder)
         read_size = block_remainder;
      else
         read_size = total_remainder_size;

      ret = ciaaBlockDevices_lseek(device, device_offset, SEEK_SET);
      if(ret!=device_offset)
      {
         if(total_read_size_p!=NULL)
            *total_read_size_p = buf_pos;
         return -1;
      }
      ret = ciaaBlockDevices_read(device, (uint8_t *)(buf+buf_pos), read_size);
      if(ret!=read_size)
      {
         if(total_read_size_p!=NULL)
            *total_read_size_p = buf_pos;
         return -1;
      }
      ciaaPOSIX_printf(   "ext2_buf_read_file: Valores:\n"\
               "device_offset: %d\nread_size: %d\nbuf_pos: %d\n"\
               "total_remainder_size: %d\nblock_remainder: %d\n",\
               device_offset, read_size, buf_pos, total_remainder_size, block_remainder);
      buf_pos+=read_size;
      finfo->f_pointer+=read_size;
      total_remainder_size-=read_size;
   }

   if(total_read_size_p != NULL)
      *total_read_size_p = buf_pos;
   if(buf_pos != size)
      return -1;
   return 0;
}

/*
   Mapeo bloques de archivo a bloques de dispositivo
 */
static int ext2_block_map(const vnode_t * dest_node, uint32_t file_block, uint32_t *device_block_p)
{
   uint32_t shift_block_level, shift_single_block_level;
   uint32_t temp_block_num, n_entries_per_block, index_offset, aux, block_level, relative_file_block;
   size_t ret;
   struct ext2fs_dinode * ninfo;
   //struct ext2_file_info *finfo;
   struct ext2_fs_info *fsinfo;
   ciaaDevices_deviceType const *device;

   device = dest_node->fs_info.device;
   ninfo = dest_node->n_info.down_layer_info;
   fsinfo = dest_node->fs_info.down_layer_info;
   //finfo = dest_node->f_info.down_layer_info;
   n_entries_per_block = fsinfo->s_block_size / sizeof(uint32_t);

   //shift_single_block_level va a ser la cantidad de bits de la mascara del nivel de bloque
   for(shift_single_block_level=0, aux=n_entries_per_block;aux;shift_single_block_level++)
   {
      //ciaaPOSIX_printf("ext2_block_map: aux: %d\n", aux);
      aux = aux>>1;   //Hace loop infinito
   }

      ciaaPOSIX_printf(   "ext2_block_map: Valores:\n"\
               "n_entries_per_block: %d\nshift_single_block_level: %d\n"\
               "file_block: %d\n",\
               n_entries_per_block, shift_single_block_level, file_block);

   if (file_block < N_DIRECT_BLOCKS)
   {
      //Es un bloque directo
      *device_block_p = ninfo->i_block[file_block];
      ciaaPOSIX_printf("ext2_block_map: Devuelvo bloque directo #%d\n", *device_block_p);
      return 0;
   }

   ciaaPOSIX_printf("ext2_block_map: No es bloque directo, empiezo el calculo...\n");
   file_block -= N_DIRECT_BLOCKS;

   for (   shift_block_level=0, block_level=1;
      file_block >= (int32_t)1<<(shift_single_block_level+shift_block_level);
      relative_file_block -= (int32_t)1<<(shift_single_block_level+shift_block_level),
      shift_block_level+=shift_single_block_level, block_level++)
   {
   //Cada iteracion corresponde a un nivel de indireccion de bloque.
   //Cuando el file_block esta dentro del nivel actual, se sale del loop.
   //A file_block se le va restando la cantidad de bloques del nivel actual.
   //Al finalizar el loop, file_block es el offset dentro del nivel correspondiente
   //block_level queda con el nivel de indireccion, 

               //Me fijo si me paso de la cantidad de bloques del nivel
//1024B/4B=256bloques
//1 2 4 8 16 32 64 128 256 512 1024
//0 1 2 3 4  5  6  7   8   9   10
//N_INDIR_BLOCKS_PER_BLOCK: BLOCK_SIZE / sizeof(uint32_t)
//N_INDIR_BLOCKS_PER_BLOCK**2
//N_INDIR_BLOCKS_PER_BLOCK**3
      if(file_block <= (int32_t) 1 << shift_block_level)
      {
         //El numero de bloque esta dentro de este nivel, no supera su limite
         break;
      }
      if (block_level > N_INDIRECT_BLOCKS)
      {   // Que es NIADDR * fi->f_nishift?
         //NIADRR es la cantidad de posiciones de bloques indirectos
         //NIADDR es 3: bloques indir, bloques dindir, bloques tindir.
         //El numero de bloque se pasa del limite maximo
         *device_block_p=0;
         return -1;
      }
   }

   //Cual de todos los bloques indirectos es?
   //Si block_level=1, el bloque indirecto es i_blocks[12], indireccion nivel 1
   //Si block_level=2, el bloque indirecto es i_blocks[13], indireccion nivel 2
   //Si block_level=2, el bloque indirecto es i_blocks[14], indireccion nivel 3
   temp_block_num = ninfo->i_block[N_DIRECT_BLOCKS + block_level - 1];
   file_block = relative_file_block;

   for (;block_level;block_level--)
   {
      if (temp_block_num == 0)
      {
         *device_block_p = 0; //Bloque no existe
         return -1;
      }
      //
      index_offset =    (temp_block_num << fsinfo->e2sb.s_log_block_size) +		//FIXME. Revisar el ejemplo
                        ((file_block & (((1<<shift_single_block_level)-1))<<(shift_block_level))
                        >> shift_block_level) * sizeof(uint32_t);
//Con file_block = 200,  1<<shift_block_level
      ret = ciaaBlockDevices_lseek(device, index_offset, SEEK_SET);	//Tendria que ser 256
      if(ret!=index_offset)
      {
         return -1;
      }
      ret = ciaaBlockDevices_read(device, (uint8_t *)temp_block_num, sizeof(uint32_t));
      if(ret!=sizeof(uint32_t))
      {
         return -1;
      }
      shift_block_level -= shift_single_block_level;
   }


   *device_block_p = temp_block_num;
   return 0;
}



static int ext2_mount_load(vnode_t *dir_node)
{
   int ret;
   //struct ext2_fs_info * fsinfo;
   //fsinfo=dir_node->fs_info.down_layer_info;
   //Como manejo el problema de hacer el mount sobre un directorio que ya existia?
   /*Leo el root inode*/
   //ciaaPOSIX_printf("print_superblock: inside ext2_mount_load\n");
   //print_superblock(&fsinfo->e2sb);
   ret = ext2_read_inode(dir_node, EXT2_ROOTINO);   //Estoy sobreescribiendo algo de lo que tenia antes?
   ASSERT_MSG(ret>=0, "ext2_mount_load(): ext2_read_inode failed");
   if(ret)
   {
      return -1;
   }
   ret = ext2_mount_load_rec(dir_node);
   ASSERT_MSG(ret>=0, "ext2_mount_load(): ext2_mount_load_rec failed");
   if(ret)
   {
      return -1;
   }

   return 0;
}
//Recibo el root vnode preparado
//Error: No estoy ignorando ./, entonces creo recursivamente el mismo dir hasta el infinito
static int ext2_mount_load_rec(vnode_t *dir_node)
{

   uint8_t *data_pointer;
   struct   ext2fs_direct * start_pointer, * end_pointer;
   int ret;
   uint32_t lret;
   int mode;
   vnode_t *child_node;

   //ciaaDevices_deviceType const *bdev;
   struct ext2fs_dinode * ninfo;
   struct ext2_file_info *finfo;
   //struct ext2_fs_info *fsinfo;

   //bdev = dir_node->fs_info.device;
   ninfo = (struct ext2fs_dinode *)dir_node->n_info.down_layer_info;
   finfo = (struct ext2_file_info *)dir_node->f_info.down_layer_info;
   //fsinfo = (struct ext2_fs_info *)dir_node->fs_info.down_layer_info;

   //Leo el directorio desde el principio
   finfo->f_pointer = 0;
   //Mientras el puntero sea menor al tamanio del archivo
   ciaaPOSIX_printf("ext2_mount_load_rec: Pido %d bytes para el archivo\n", ninfo->i_size);
   data_pointer=(uint8_t*)ciaaPOSIX_malloc(ninfo->i_size);//TODO: Hago malloc del tamanio del archivo entero
                              //Mucho malloc
   ASSERT_MSG(data_pointer!=NULL, "ext2_mount_load_rec(): Mem error");
   if(data_pointer==NULL)
   {
      ciaaPOSIX_free(data_pointer);
      return -1;
   }
   ext2_buf_read_file(dir_node, (uint8_t*)data_pointer, ninfo->i_size, &lret);   //Leo todo el directorio
   ASSERT_MSG(lret==ninfo->i_size, "ext2_mount_load_rec(): ext2_buf_read_file() failed");
   if(lret!=ninfo->i_size)
   {
      ciaaPOSIX_free(data_pointer);
      return -1;
   }


   for(   start_pointer=(struct ext2fs_direct *)data_pointer,
      end_pointer=(struct ext2fs_direct *)(data_pointer + ninfo->i_size);
      start_pointer<end_pointer;
      start_pointer=(struct ext2fs_direct *)((char*)start_pointer + start_pointer->e2d_reclen))
   {
      ciaaPOSIX_printf("\n\n\next2_mount_load_rec(): start_pointer=%p\n", start_pointer);
      ciaaPOSIX_printf("ext2_mount_load_rec(): end_pointer=%p\n", end_pointer);
      ciaaPOSIX_printf("ext2_mount_load_rec(): e2d_ino=%d e2d_reclen=%d e2d_namlen=%d e2d_type=%d e2d_name=%.*s\n\n",
            start_pointer->e2d_ino, start_pointer->e2d_reclen, start_pointer->e2d_namlen,
            start_pointer->e2d_type, start_pointer->e2d_namlen, start_pointer->e2d_name);
      if(start_pointer->e2d_reclen <= 0)
      {
         ciaaPOSIX_printf("ext2_mount_load_rec(): Fallo ext2_buf_read_file\n");
         break;
      }
      if(start_pointer->e2d_ino == 0)
      {
         continue;
      }
      if(!ciaaPOSIX_strcmp(start_pointer->e2d_name, ".") || !ciaaPOSIX_strcmp(start_pointer->e2d_name, ".."))
      {
         continue;
      }
      mode=0;
      child_node = vfs_create_child(dir_node, start_pointer->e2d_name, start_pointer->e2d_namlen, mode);
      ASSERT_MSG(child_node!=NULL, "ext2_mount_load_rec(): Mem error");
      if(child_node==NULL)
      {
         return -1;
      }
      //Lleno los campos del nodo. Primero tengo que pedir espacio para los datos de la capa inferior
      child_node->f_info.down_layer_info = (struct ext2_file_info *) ciaaPOSIX_malloc(sizeof(struct ext2_file_info));
      //child_node->fs_info.down_layer_info = (struct ext2_fs_info *) ciaaPOSIX_malloc(sizeof(struct ext2_fs_info));
      child_node->n_info.down_layer_info = (struct ext2fs_dinode *) ciaaPOSIX_malloc(sizeof(struct ext2fs_dinode));
      if(child_node->f_info.down_layer_info == NULL || child_node->fs_info.down_layer_info == NULL
         || child_node->n_info.down_layer_info == NULL)   //FIXME
      {
         ciaaPOSIX_printf("ext2_mount_load_rec: Mem error downlayer\n");
         return -1;
      }
      ciaaPOSIX_printf("\n\next2_mount_load_rec: Padre: %s, Hijo: %s\n",
            dir_node->f_info.file_name, child_node->f_info.file_name);
      ciaaPOSIX_printf("ext2_mount_load_rec: Entrando a ext2_read_inode()\n");
      ASSERT_MSG(ret>=0, "ext2_mount_load_rec(): ext2_read_inode() failed");
      ret = ext2_read_inode(child_node, start_pointer->e2d_ino);
      ASSERT_MSG(ret>=0, "ext2_mount_load_rec(): ext2_read_inode() failed");
      if(ret)
      {
         ciaaPOSIX_free(data_pointer);
         return -1;
      }
      if(start_pointer->e2d_type == EXT2_FT_DIR)
      {
         ret = ext2_mount_load_rec(child_node);
         if(ret)
         {
            ciaaPOSIX_free(data_pointer);
            return -1;
         }
      }

   }
   //Ya pase por todos los subtrees de dir_node. Exito
   ciaaPOSIX_free(data_pointer);
   return 0;
}

static node_type_t ext2_nodetype_e2tovfs(int imode)
{
   int file_format;
   node_type_t type;
   file_format = imode & 0xF000;
   switch(file_format)
   {
      case EXT2_S_IFREG:
         type=VFS_FTREG;
      break;
      case EXT2_S_IFBLK:
         type=VFS_FTBLK;
      break;
      case EXT2_S_IFDIR:
         type=VFS_FTDIR;
      break;
      default:
         type=VFS_FTUNKNOWN;
      break;
   }
   return type;
}

/*
static uint32_t ext2_alloc_inode_bit() {


   ciaaDevices_deviceType const *bdev;
   struct ext2fs_dinode * ninfo;
   struct ext2_file_info *finfo;
   struct ext2_fs_info *fsinfo;

   ciaaDevices_deviceType const *bdev;

   // Find the first group with free inodes
   for(i=0; i<fsinfo->ngroups; i++)
   {
      if(fsinfo->e2fs_gd[i].free_inodes_count > 0)
         break;
   }
   if(i==fsinfo->ngroups)
   {
      // No group with free inodes found
      return -1
   }
   group = i;
   gd = &(fsinfo->e2fs_gd[i]);

   ret = ciaaBlockDevices_lseek(bdev, fsinfo->e2fs_gd[i].inode_bitmap * fsinfo->block_size, SEEK_SET);
   if(ret)
   {
      return -1;
   }
   ret = ciaaBlockDevices_read(bdev, (uint8_t *)block_buffer, fsinfo->block_size);
   if(ret)
   {
      return -1;
   }

   for(i=0; i < fsinfo->block_size && block_buffer[i]==0xFF; i++);   //Busco el primer byte con un bit en 0
   if(i==fsinfo->fsinfo->block_size)
   {
      // All inodes occupied
      return -1;
   }
   // Now i is the index of the byte with the free bit
   for(j=0; j<8; j++)
   {
      if((block_buffer[i] & (1<<j)) == 0)   // El bit j del byte block_buffer[i] es 0
      {                                    // El bloque i*8+j estaba libre y ahora lo marco
         block_buffer[i] |= (1<<j);
         break;
      }
   }
   // Numero de nodo contando desde 0. XXX: Tendria que contar desde 1?
   finfo->f_inumber = group * fsinfo->e2sb.s_inodes_per_group + 8*i + j;
   // Devuelvo el bitmap midificado al disco
   ret = ciaaBlockDevices_lseek(bdev, fsinfo->e2fs_gd[i].inode_bitmap * fsinfo->block_size, SEEK_SET);
   if(ret)
   {
      return -1;
   }
   ret = ciaaBlockDevices_read(bdev, (uint8_t *)block_buffer, fsinfo->block_size);
   if(ret)
   {
      return -1;
   }

   gd->free_inodes_count--;
   fsinfo->e2sb.s_free_inodes_count--;
	if (is_dir) {
		gd->used_dirs_count++;
	}
   // Devuelvo los valores modificados del superblock y los group descriptor al disco
   for(group_index=0; group_index<ngroups; group_index++)
   {
      group_offset = EXT2_SBOFF + fsinfo->e2sb.s_blocks_per_group*fsinfo->block_size*group_index;	//Es confuso, pero parece correcto.
      //Si el block size es 1024, block 0 es boot, block 1 es superblock. Blocks 1 a 8192 son blocks del group 1
      //Por lo tanto el boot block no cuenta como block del group 1.

      ret = ciaaBlockDevices_lseek(bdev, group_offset, SEEK_SET);
      if(ret)
      {
         return -1;
      }
      ret = ciaaBlockDevices_write(bdev, (uint8_t *)&(fsinfo->e2sb), sizeof(struct ext2sb));
      if(ret)
      {
         return -1;
      }
      ret = ciaaBlockDevices_lseek(bdev, group_offset + EXT2_SBSIZE, SEEK_SET);
      if(ret)
      {
         return -1;
      }
      ret = ciaaBlockDevices_write(bdev, (uint8_t *)gd, sizeof(struct ext2_gd) * fsinfo->ngroups);
      if(ret)
      {
         return -1;
      }
   }

	return 0;
}
*/

/*

   if (block_pos < EXT2_NDIR_BLOCKS)
   {
      finfo->f_di.i_block[block_pos] = new_block;
      finfo->f_di.i_blocks += fsinfo->sectors_in_block;   //FIXME: Agregar el campo sectors_in_block
							  //Cada sector son 512 bytes
         return 0;
   }

   temp_block_num = finfo->f_di.i_block[EXT2_DIR_BLOCK - 1 + block_level];
   if (temp_block_num == 0)
   {
      //Reservo un bloque en el gd. temp_block_num toma el valor del numero de este bloque
      //Limpio el bloque
   }
      ////////////////////////////////////////////////////////////////////////////////////////
   for (;block_level;block_level--)
   {
      index_offset =   (temp_block_num << fsinfo->e2sb.s_log_block_size) +
                       block_num & (( ((1<<shift_single_block_level)-1) << shift_block_level)
                       >> shift_block_level) * sizeof(uint32_t);
      ret = ciaaBlockDevices_lseek(device, index_offset, SEEK_SET);	//Tendria que ser 256
      if(ret!=index_offset)
      {
         return -1;
      }
      ret = ciaaBlockDevices_read(device, (uint8_t *)temp_block_num, sizeof(uint32_t));
      if(ret!=sizeof(uint32_t))
      {
         return -1;
      }

      if (temp_block_num == 0)
      {
         ret = ext2_block_reserve(node, &temp_block_num);
         ret = ciaaBlockDevices_lseek(device, index_offset, SEEK_SET);	//Tendria que ser 256
         if(ret!=index_offset)
         {
            return -1;
         }
         ret = ciaaBlockDevices_write(device, (uint8_t *)temp_block_num, sizeof(uint32_t));
         if(ret!=sizeof(uint32_t))
         {
            return -1;
         }
         //Reservo un bloque en el gd. temp_block_num toma el valor del numero de este bloque
         //Limpio el bloque
      }
   }
      /////////////////////////////////////////////////////////////////////////////////////////////
*/

//Max 2.5kb de ram. Es poco.
/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
