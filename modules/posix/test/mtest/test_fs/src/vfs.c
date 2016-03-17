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
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*
TODO:
-Utilizar solo memoria estatica, salvo excepciones que lo justifiquen
-Agregar mutex o semaforos o colas segun corresponda para soportar multitasking
*/

/*==================[inclusions]=============================================*/
#include "ciaaPOSIX_stdlib.h"
#include "ciaaBlockDevices.h"
#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_string.h"
#include "ciaaPOSIX_stdbool.h"
#include "vfs.h"

/*==================[macros and definitions]=================================*/

#define ASSERT_MSG(cond, msg) assert_msg((cond), (msg), __FILE__, __LINE__)

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/


extern vnode_t *vfs_create_child(vnode_t *parent, const char *name, size_t namelen, mode_t mode);

static int vfs_get_mntpt_path_rec(vnode_t *node, char *path, uint16_t *len_p);
static int vfs_get_mntpt_path(vnode_t *node, char *path, uint16_t *len_p);
static vnode_t *vfs_node_alloc(const char *name, size_t name_len);
static void vfs_node_free(vnode_t *node);
static int kmount(const char *dev, const char *dir, const char *fs_type);
static int vfs_inode_reserve(const char *path, vnode_t **inode_p);
static int vfs_inode_search(char **path_p, vnode_t **ret_node_p);
static struct filesystem_driver *vfs_get_driver(const char *driver_name);
static int file_descriptor_table_init(void);
static struct file_desc *file_desc_create(vnode_t *node);
static int file_desc_destroy(struct file_desc *file_desc);
static struct file_desc *file_desc_get(uint16_t index);

/*==================[internal data definition]===============================*/

/** \brief Nodo root
 *
 * Es el nodo padre de todo el vfs.
 * 
 */
static vnode_t *vfs_root_inode = NULL;

/** \brief Filesystem drivers declaration
 *
 * Declaro los drivers que va a utilizar el vfs. Estan definidos en sus respectivos archivos
 * 
 */
extern struct filesystem_driver ext2_driver;
extern struct filesystem_driver pseudofs_driver;
extern struct filesystem_driver blockdev_driver;

/** \brief Filesystem drivers table
 *
 * Tabla utilizada por vfs_get_driver() para buscar el driver solicitado
 * 
 */
static struct filesystem_driver *vfs_fsdriver_table[] =
{
   &ext2_driver,
   &blockdev_driver,
   NULL,
   NULL,
   NULL
};

/** \brief Filesystem drivers table
 *
 * Tabla de descriptores de archivo. 
 * 
 */
static struct file_descriptor_table _f_tab;
static struct file_descriptor_table *file_desc_tab_p = &_f_tab;


/*==================[external data definition]===============================*/

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

static int vfs_get_mntpt_path_rec(vnode_t *node, char *path, uint16_t *len_p)
{
   uint16_t name_len;
   int ret;
   if(node->n_info.is_mount_dir==false)
   ret = vfs_get_mntpt_path_rec(node->parent_node, path, len_p);
   if(ret)
   {
      return -1;
   }
   name_len = ciaaPOSIX_strlen(node->f_info.file_name);
   if(*len_p + name_len > FS_PATH_MAX-1)
      return -1;
   ciaaPOSIX_strcpy((char *)path+(*len_p), node->f_info.file_name);
   *len_p += ciaaPOSIX_strlen(node->f_info.file_name);
   return 0;
}

static int vfs_get_mntpt_path(vnode_t *node, char *path, uint16_t *len_p)
{
   int ret;
   *len_p = 0;
   ret = vfs_get_mntpt_path_rec(node, path, len_p);
   return ret;
}

static vnode_t *vfs_node_alloc(const char *name, size_t name_len)
{
   vnode_t * new_node;

   //ASSERT_MSG(name_len, "\tvfs_node_alloc(): !name_len failed");
   if (!name_len)
   {
      return NULL;
   }
   //ASSERT_MSG(*name && name_len <= FS_NAME_MAX, "\tvfs_node_alloc(): !(*name) || name_len > FS_NAME_MAX failed");
   if (!(*name) || name_len > FS_NAME_MAX)
   {
      return NULL;
   }
   new_node = (vnode_t *) ciaaPOSIX_malloc(sizeof(vnode_t));
   //ASSERT_MSG(new_node!=NULL, "\tvfs_node_alloc(): !(*name) || name_len > FS_NAME_MAX failed");
   if (new_node == NULL)
   {
      return NULL;
   }
   ciaaPOSIX_memset(new_node, 0, sizeof(vnode_t));
   ciaaPOSIX_strncpy(new_node->f_info.file_name, name, name_len);
   new_node->f_info.file_name[name_len] = '\0';

   return new_node;
}

/*Supongo que ya estan liberados de memoria los elementos internos de vnode_t*/
static void vfs_node_free(vnode_t *node)
{
   if(node!=NULL)
      ciaaPOSIX_free(node);
}

//Crear una funcion a la cual se le de un path y devuelva el nodo que corresponde. Ver fs_perm_lookup.
//Para dev, ver  block_dev_create en embox. drivers/block_dev/block_dev.c
//Hay que diferenciar un nodo de archivo de un nodo de block device. Habria que tener un indicador de tipo de nodo
//Ver tambien mount() de Nuttx en fs_mount.c.

//int mount(   const char *source, const char *target,
//      const char *filesystemtype, unsigned long mountflags,
//      const void *data);
//En Nuttx, verifico que sea un block device con el macro INODE_IS_BLOCK(inode). Se fija en node->i_flags
//Ok, ya diferencie un nodo de archivo de un nodo de block device. Ahora, donde almaceno la informacion del block device?
//Estoy viendo donde inizializa Nuttx al filesystem. En os_start() parece que no. Sin embargo utiliza write(0,buffer,size),
//O sea, utiliza el stdout, que tendria que ser parte del filesystem. Estoy estudiando fs_write.c.
//Cada thread tiene asociado una cantidad de archivos. Hay algunos preabiertos, como stdout. list = sched_getfiles();
//Hace inode->u.i_ops->write(this_file, buf, nbytes); asi que tiene asociado algun driver stdout.
//Ver en la incializacion cuando le asigna eso.

//"Create stdout, stderr, stdin on the IDLE task.  These will be
//inherited by all of the threads created by the IDLE task"
//stdin lo creo con fd = open("/dev/console", O_RDWR);
//Como hizo open si todavia no esta el filesystem iniciado? Ver la implementacion de open.
//Ver register_driver(), involucra el path del serial device. "Register a character driver inode the pseudo file system."
//int register_driver(FAR const char *path, FAR const struct file_operations *fops,
//                    mode_t mode, FAR void *priv)

//Reserva un nodo en el path que corresponde y le pone los datos
//   inode_reserve(path, &node);
//      node->u.i_ops   = fops;
//      node->i_mode    = mode;
//      node->i_private = priv;
//Que almacena en priv?: Almacena una estructura del tipo uart_dev_t *dev.

//Tengo serial_operations:
//static const struct file_operations g_serialops =
//{
//  uart_open,
//  uart_close,
//  uart_read,
//  uart_write,
//  0,
//  uart_ioctl,
//};
//Estas operations saben que en node->i_private esta el driver del dispositivo, entonces acceden a esto. Saben a que estructura
//castear.
//Inicializar los file descriptors como cualquier otro archivo, cuyo fs sea especial para file descriptors.
//Ver inode_reserve() para ver como reservar los nodos intermedios de un path.
//Ej: /dir1/dir2


//Ver implementacion en Nuttx. inode_search() en fs/inode/fs_inode.c
//Capaz me conviene que vfs_inode_search() devuelva el ultimo elemento valido del path.
//De este modo, si tengo que crear un subtree, puedo tener el ultimo elemento valido, y de ahi crear el subtree con los elementos
//restantes.
//Ver la implementacion en Nuttx
//struct inode *inode_search(const char **path,
//                               FAR struct inode **peer,
//                               FAR struct inode **parent,
//                               const char **relpath)

/*
Ejemplos

Tengo solo el root:
/
Quiero poner el nodo: /n1/n2

search:
node=vfs_root_inode; El ultimo nodo encontrado va a ser el root. Hay que agregar n1 y n2. Empiezo desde node->child
*/

/*

ret=vfs_inode_search(&path, &inode);
if(!ret)
{
   //El nodo ya existe
   return -1;
}
//Ahora inode apunta al ultimo nodo valido, por lo que hay que empezar creando inode->child
//Ahora path apunta al primer caracter del primer elemento del path a crear, por lo que
//inode->child debe ser nombrado a partir de este elemento.
p_start=p_end=path;

for(p_end=p_start; *p_end!='\0' && *p_end!='/'; p_end++);
//Ahora p_end apunta al primer '/'
child = vfs_create_child(inode, p_start, p_end-p_start, 0);   //mode=0
if(child == NULL)
{
   return -1;
}
inode=child;
p_start = p_end;
for(;*p_start!='\0'&&*p_start!='/';p_start++);   //Cuando termina estoy en la primera letra despues del '/'
if(*p_start=='\0')            //Llegue al fin del path que busco
{
   //El ultimo node es el que corresponde
   return 0;
}
for(p_end=p_start; *p_end!='\0' && *p_end!='/'; p_end++);   //Ahora p_start y p_end apuntan al siguiente elemento del path
                        //Node tendria que apuntar al nodo recien creado
child = vfs_create_child(inode, p_start, p_end-p_start, 0);
if(child == NULL)
{
   return -1;
}
node=child;

p_start = p_end;
for(;*p_start!='\0'&&*p_start!='/';p_start++);
*path_p = p_start;
if(*p_start=='\0')
{
   return 0;
}
for(p_end=p_start; *p_end!='\0' && *p_end!='/'; p_end++);


int vfs_inode_reserve(const char *path, FAR struct inode **inode)
{
ret=vfs_inode_search(&path, &inode);
if(!ret)
{
   //El nodo ya existe
   return -1;
}
//Ahora inode apunta al ultimo nodo valido, por lo que hay que empezar creando inode->child
//Ahora path apunta al primer caracter del primer elemento del path a crear, por lo que
//inode->child debe ser nombrado a partir de este elemento.
p_start=p_end=path;
while(1)
{
for(p_end=p_start; *p_end!='\0' && *p_end!='/'; p_end++);   //Ahora p_start y p_end apuntan al siguiente elemento del path
                        //Node tendria que apuntar al nodo recien creado
child = vfs_create_child(inode, p_start, p_end-p_start, 0);
if(child == NULL)
{
   return -1;
}
node=child;
p_start = p_end;
for(;*p_start!='\0'&&*p_start!='/';p_start++);   //Cuando termina estoy en la primera letra despues del '/'
if(*p_start=='\0')            //Llegue al fin del path que busco
{
   //El ultimo node es el que corresponde
   return 0;
}
}
}

*/

//Devuelve -1 si el directorio ya existia
//Crea directorios intermedios
//TODO: Si recibe un path sin el '/' inicial, es un relative path. Crea el arbol a partir del nodo entregado
//Si esta el '/' inicial, crea el arbol a partir del root,  como ahora
//La hoja creada es de tipe VFS_FTREG
static int vfs_inode_reserve(const char *path, vnode_t **inode_p)
{
   int ret;
   vnode_t *inode, *child;
   char *p_start, *p_end, *aux_path;

   aux_path = (char *)path;
   ret=vfs_inode_search(&aux_path, &inode);
   ASSERT_MSG(ret, "\tvfs_inode_reserve(): Node already exists");
   if(!ret)
   {
      //El nodo ya existe
      return -1;
   }
// *path_p va a apuntar al primer caracter del primer elemento del path que no se encuentra en el arbol
// *ret_node_p va a apuntar al ultimo nodo valido del arbol incluido en el path.
//ciaaPOSIX_printf("vfs_inode_reserve():Primer elemento que no se encuentra: %s\n", aux_path);
//ciaaPOSIX_printf("vfs_inode_reserve():Ultimo nodo valido: %s\n", inode->f_info.file_name);
   //Ahora inode apunta al ultimo nodo valido, por lo que hay que empezar creando inode->child
   //Ahora path apunta al primer caracter del primer elemento del path a crear, por lo que
   //inode->child debe ser nombrado a partir de este elemento.
   p_start=p_end=aux_path;
   for(p_start=aux_path; *p_start=='/'; p_start++);   //Salteo los '/' iniciales
   while(1)
   {
      for(p_end=p_start; *p_end!='\0' && *p_end!='/'; p_end++);
      //Ahora p_start y p_end apuntan al siguiente elemento del path
      //Node tendria que apuntar al nodo recien creado
      //ciaaPOSIX_printf("vfs_inode_reserve(): %s, %d\n", p_start, (int)(p_end-p_start));
      if(p_end==p_start)
      {
         
         return -1;
      }
      child = vfs_create_child(inode, p_start, (uint32_t)(p_end-p_start), 0);
      //ciaaPOSIX_printf("vfs_inode_reserve():Padre %s\n", inode->f_info.file_name);
      //ciaaPOSIX_printf("vfs_inode_reserve():Hijo %s\n", inode->child_node->f_info.file_name);
      *inode_p = child;
      ASSERT_MSG(child!=NULL, "\tvfs_inode_reserve(): vfs_create_child() failed");
      if(child == NULL)
      {
         return -1;
      }
      inode=child;
      for(p_start=p_end;*p_start=='/';p_start++);
      //Cuando termina estoy en la primera letra despues del '/'
      if(*p_start=='\0')//Llegue al fin del path que busco
      {
         //El ultimo node es el que corresponde
         return 0;
      }
      //Queda camino por seguir. El nodo creado es un directorio
      child->n_info.type = VFS_FTDIR;
   }
}

//*path_p va a apuntar al primer caracter del primer elemento del path que no se encuentra en el arbol
//*ret_node_p va a apuntar al ultimo nodo valido del arbol incluido en el path.
//FIXME: Devuelve el hijo de root, y no root, si el path dado no coincide para nada
//Devuelve -1 si el path no empieza con / o no existe
//
static int vfs_inode_search(char **path_p, vnode_t **ret_node_p)
{
   uint16_t pnamelen; //Path element namelength
   char *p_start, *p_end;
   uint32_t ret;
   vnode_t *node;

    if (*path_p==NULL || (*path_p)[0]=='\0' || *(path_p)[0]!='/')
   {
      *ret_node_p = NULL;
      return -1;
   }

   *ret_node_p = node = vfs_root_inode;

   p_start = p_end = *path_p;
   for(;*p_start=='/';p_start++);   //Cuando termina estoy en la primera letra despues del '/'
   *path_p = p_start;
   if(*p_start=='\0')   //En el path solo habian '/'. Devuelvo el root
   {
      *ret_node_p=vfs_root_inode;
      return 0;
   }
   for(p_end=p_start; *p_end!='\0' && *p_end!='/'; p_end++);
   node = node->child_node;   //Comienzo en el primer nivel debajo del root
   while(node!=NULL)   //Si el nodo es NULL significa que ninguno de los hermanos del nivel coincide con el
            //elemento del path, entonces el path no es valido
   {
      //Los caracteres que conforman el elemento del path van a estar comprendidos entre p_start y p_end.
      //p_start apunta a la primera letra del elemento. p_end apunta al siguiente caracter de la ultima letra del elemento.
      pnamelen = p_end-p_start;   //Longitud del nombre entre '/' sin contar el '\0'.
      while(node!=NULL)
      {
         ciaaPOSIX_printf("Comparando %s con %s\n",p_start,node->f_info.file_name);
         ret = ciaaPOSIX_strncmp(p_start, node->f_info.file_name, pnamelen);
         if(!ret)
         {
            ciaaPOSIX_printf("\tCoincidieron %s con %s\n",p_start,node->f_info.file_name);
            //Coinciden los nombres, tengo que analizar el hijo
            break;
         }
      //El nombre del nodo no coincide con el elemento del path. Tengo que comparar el mismo elemento del path
      //con el siguiente nodo del mismo nivel.
      //Tener que comparar el mismo elemento del path significa que p_start y p_end no deben cambiar
         node = node->sibling_node;
      }
      if(node==NULL)
      {
         ciaaPOSIX_printf("No se encontraron coincidencias en este nivel. Chau\n");
      //Al iterar sobre los hermanos no se encontro ningun elemento del path que coincida con el
      //nombre de alguno de los nodos de ese nivel del arbol, asi que el path es invalido.
      //*ret_node_p va a apuntar al ultimo nodo valido del path
         return -1;
      }
      //El elemento del path coincide con el nombre del nodo. Hay que seguir con el siguiente elemento
      //del path en el siguiente nivel del arbol de nodos
      *ret_node_p = node;
      node = node->child_node;
      p_start = p_end;
      for(;*p_start=='/';p_start++);   //Cuando termina estoy en la primera letra despues del '/'
      *path_p = p_start;
      if(*p_start=='\0')            //Llegue al fin del path que busco
      {
         ciaaPOSIX_printf("Se encontro el nodo buscado. Exito\n");
         //El ultimo node es el que corresponde
         return 0;
      }
      for(p_end=p_start; *p_end!='\0' && *p_end!='/'; p_end++);
      //Cuando termina p_end esta en el '/' que sigue O p_end esta en el '\0' si era ultimo elemento del path
   }
   //El path contiene mas elementos que la prifundidad del arbol, no existe el nodo buscado. Devuelvo fallido
   return -1;
}

static struct filesystem_driver *vfs_get_driver(const char *driver_name)
{
   int i;

   for(i=0; vfs_fsdriver_table[i]!=NULL; i++)
   {
      if(!ciaaPOSIX_strcmp(driver_name, vfs_fsdriver_table[i]->driver_name))
         return vfs_fsdriver_table[i];
   }
   return NULL;
}

/*#######################FILE_DESC######################*/

static int file_descriptor_table_init(void)
{
/*
   file_desc_tab_p = (file_descriptor_table *) ciaaPOSIX_malloc(sizeof(file_descriptor_table));
   if(file_desc_tab_p == NULL)
      return -1;
*/
   ciaaPOSIX_memset(file_desc_tab_p, 0, sizeof(struct file_descriptor_table));
   return 0;
}

static struct file_desc *file_desc_create(vnode_t *node)
{
   struct file_desc *file;
   //int ret;
   uint16_t i;

   if(file_desc_tab_p->n_busy_desc >= FILE_DESC_MAX)
      return NULL;
   
   for(i=0; i<FILE_DESC_MAX; i++)
   {
      if(file_desc_tab_p->table[i] == NULL)
         break;
   }
   //Ahora i es el indice del primer file_desc desocupado en la tabla
   if(i==FILE_DESC_MAX)   //No hay file_desc libres
      return NULL;
   // allocate new descriptor
   file = (struct file_desc *) ciaaPOSIX_malloc(sizeof(struct file_desc));
   if(file == NULL)
      return NULL;
   ciaaPOSIX_memset(file, 0, sizeof(struct file_desc));
   file->node = node;

   file_desc_tab_p->table[i] = file;
   file_desc_tab_p->n_busy_desc++;

   
   file->node = node;
   file->index = i;

   return file;
}

static int file_desc_destroy(struct file_desc *file_desc)
{
   uint16_t index;

   index = file_desc->index;
   ciaaPOSIX_free(file_desc_tab_p->table[index]);
   file_desc_tab_p->table[index] = NULL;
   if(file_desc_tab_p->n_busy_desc)
      file_desc_tab_p->n_busy_desc--;
   return 0;
}

struct file_desc *file_desc_get(uint16_t index)
{

   if(index >= FILE_DESC_MAX)
      return NULL;

   return file_desc_tab_p->table[index];
}



/*########################/FILE_DESC#####################*/

/*########################AUXILIAR#####################*/
static int vfs_print_tree_rec(vnode_t *node, int level)
{
   int ret, i;

   if(node==NULL)
   {
      return 0;
   }
   for(i=0; i<level; i++)
      ciaaPOSIX_printf("---");
   ciaaPOSIX_printf("%s\n",node->f_info.file_name);

   ret=vfs_print_tree_rec(node->child_node, level+1);
   if(ret)
   {
      return -1;
   }
   ret=vfs_print_tree_rec(node->sibling_node, level);
   if(ret)
   {
      return -1;
   }

   return 0;
}

int vfs_print_tree(void)
{
   return vfs_print_tree_rec(vfs_root_inode, 0);
}
/*########################/AUXILIAR#####################*/

/*==================[external functions definition]==========================*/

/** \brief VFS create child
 *
 * Required by driver implementation
 * 
 */
extern vnode_t *vfs_create_child(vnode_t *parent, const char *name, size_t namelen, mode_t mode)
{
   vnode_t *child;

   child = vfs_node_alloc(name, namelen);
   //ciaaPOSIX_printf("%s\n", child->f_info.file_name);
   ASSERT_MSG(child!=NULL, "\tvfs_create_child(): vfs_node_alloc() failed");
   if(child == NULL)
   return NULL;
   child->sibling_node = parent->child_node;
   parent->child_node = child;
   child->parent_node = parent;
   //Copio la informacion de filesystem. No hago copia nueva, solo apunto a lo del padre
   ciaaPOSIX_memcpy(&(child->fs_info), &(parent->fs_info), sizeof(struct filesystem_info));
   child->f_info.file_name[namelen] = '\0';
   child->f_info.file_pointer=0;
   child->f_info.down_layer_info=NULL;
   return child;
   
}

/** \brief VFS initialization
 *
 * Inicializa el root inode, los directorios basicos y asigna un archivo para cada dispositivo
 * 
 */
extern int vfs_init(void)
{
   vnode_t *aux_inode;
   int ret;
   ciaaDevices_deviceType * device;

   /*up_initialize() en Nuttx*/
   vfs_root_inode = (vnode_t *) ciaaPOSIX_malloc(sizeof(vnode_t));
   if(vfs_root_inode==NULL)
   {
      return -1;
   }
   vfs_root_inode->n_info.is_mount_dir = true;
   vfs_root_inode->n_info.type = VFS_FTDIR;
   vfs_root_inode->f_info.file_name[0] = '\0';
   ret = file_descriptor_table_init();
   ASSERT_MSG(-1 != ret, "vfs_init(): file_descriptor_table_init() failed");
   if(ret)
   {
      return -1;
   }
   /*Crear dispositivos*/
   ret = vfs_inode_reserve("/dev/block/fd/0", &aux_inode);
   ASSERT_MSG(-1 != ret, "vfs_init(): vfs_inode_reserve() failed");
   if(aux_inode == NULL)
   {
      //No se pudo crear el nodo
      return -1;
   }

   aux_inode->n_info.type = VFS_FTBLK;
   aux_inode->fs_info.device = ciaaDevices_getDevice("/dev/block/fd/0");
   ASSERT_MSG(aux_inode->fs_info.device!=NULL, "vfs_init(): vfs_get_driver() failed");
   if(aux_inode->fs_info.device == NULL)
   {
      return -1;
   }
   device = ciaaBlockDevices_open("/dev/block/fd/0", (ciaaDevices_deviceType *)aux_inode->fs_info.device, O_RDWR);
   ASSERT_MSG(device!=NULL, "vfs_init(): failed to open device");

   aux_inode->fs_info.drv = vfs_get_driver("BLOCKDEV");
   ASSERT_MSG(aux_inode->fs_info.drv != NULL, "vfs_init(): vfs_get_driver() failed");
   if(aux_inode->fs_info.drv == NULL)
   {
      return -1;
   }



   ret = vfs_inode_reserve("/dev/char/fd/0", &aux_inode);
   ASSERT_MSG(-1 != ret, "vfs_init(): vfs_inode_reserve() failed");
   if(aux_inode == NULL)
   {
      //No se pudo crear el nodo
      return -1;
   }

   return 0;
}

/** \brief Format a device
 *
 * 
 * 
 */
int format(const char *device_path, const char *fs_type)
{
   vnode_t *devnode;
   char *devpath;
   struct filesystem_driver *fs_driver;
   int ret;

   devpath = (char *)device_path;
   ret = vfs_inode_search(&devpath, &devnode);
   ASSERT_MSG(ret==0, "mount(): format() failed. Device doesnt exist");
   if(ret)
   {
      //El nodo del device no existe
      return -1;
   }

   fs_driver = vfs_get_driver(fs_type);
   ASSERT_MSG(fs_driver!=NULL, "format(): Device node not valid");
   if(fs_driver == NULL)
   {
      //No existe un driver con el nombre dado
      return -1;
   }

   ASSERT_MSG(devnode->n_info.type == VFS_FTBLK, "format(): Target file not a device");
   if(devnode->n_info.type != VFS_FTBLK)
   {
      //El supuesto nodo device no es device
      return -1;
   }

   //Llamo a la funcion de bajo nivel
   ret = fs_driver->driver_op->fs_format(devnode);
   ASSERT_MSG(ret==0, "format(): Lower layer format failed");
   if(ret)
   {
      //Fallo el format de bajo nivel
      return -1;
   }
   return 0;
}

/** \brief VFS mount
 *
 * Ir a la definicion de POSIX
 * 
 */
extern int mount(char *device_path,  char *target_path, char *fs_type)
{
   char *devpath, *tpath;
   struct filesystem_driver *fs_driver;
   int ret;
   vnode_t *targetnode, *devnode;

   /*TODO: Agregar el tema de crear un nuevo nodo donde agregar el mount, validar que no existe*/
   /*En Nuttx hace ret = inode_reserve(target, &mountpt_inode); target es el path del dir destino*/
   /*Cancela si ya existe un node en el path*/
   tpath = target_path;
   ret = vfs_inode_reserve(tpath, &targetnode);
   ASSERT_MSG(ret==0, "mount(): vfs_inode_reserve() failed creating target node");
   if(ret)
   {
      //El directorio ya existe
      return -1;
   }
   devpath = device_path;
   ret = vfs_inode_search(&devpath, &devnode);
   ASSERT_MSG(ret==0, "mount(): vfs_inode_search() failed. Device doesnt exist");
   if(ret)
   {
      //El nodo del device no existe
      return -1;
   }
   ASSERT_MSG(devnode->n_info.type == VFS_FTBLK, "mount(): Device node not valid");
   if(devnode->n_info.type != VFS_FTBLK)
   {
      //El supuesto nodo device no es device
      return -1;
   }
   fs_driver = vfs_get_driver(fs_type);
   ASSERT_MSG(fs_driver!=NULL, "mount(): Device node not valid");
   if(fs_driver == NULL)
   {
      //No existe un driver con el nombre dado
      return -1;
   }
   //Lleno los campos que corresponden del target inode
   targetnode->fs_info.drv = fs_driver;
   targetnode->fs_info.device = devnode->fs_info.device;
   targetnode->n_info.is_mount_dir = true;
   //Llamo a la funcion de bajo nivel
   ret = fs_driver->driver_op->fs_mount(devnode, targetnode);
   ASSERT_MSG(ret==0, "mount(): Lower layer mount failed");
   if(ret)
   {
      //Fallo el mount de bajo nivel
      return -1;
   }
   return 0;
}

extern int mkdir(const char *dir_path, mode_t mode)
{
  vnode_t *dir_inode_p, *parent_inode_p;
  int               ret;

   char *tpath = (char *) dir_path;
   /*Creo un nodo pelado en el dir_path*/
   /*vfs_inode_reserve hereda los atributos de fs del padre*/
   ret = vfs_inode_reserve(tpath, &dir_inode_p);
   if(ret)
   {
      //El directorio ya existe
      return -1;
   }
   /*Lleno los campos del nodo*/
   dir_inode_p->n_info.type = VFS_FTDIR;
   /*Los atributos de fs ya fueron heredados mediante vfs_inode_reserve*/
   parent_inode_p = dir_inode_p->parent_node;
   ret = (dir_inode_p->fs_info).drv->driver_op->fs_create_node(parent_inode_p, dir_inode_p);
   if(ret)
   {
      //No se pudo crear el nodo de la capa inferior. Manejar la situacion
      return -1;
   }
   //En Nuttx, mkdir de la capa inferior toma como argumentos (inode, relpath, mode). Supongo que no voy a necesitar
   //relpath, ya que tengo el inumber del nodo padre.
   return 0;

}

extern int open(const char *path, int flags) {

   vnode_t *target_inode, *parent_inode;
   struct file_desc *file;
   char *auxpath;
   int ret, fd;

   auxpath = (char *) path;
   ret = vfs_inode_search(&auxpath, &target_inode);   //Devuelve 0 si encuentra el nodo
   if (ret)   //El nodo no existia, hay que crear uno nuevo si la opcion esta dada, sino error
   {
      ciaaPOSIX_printf("open(): El nodo no existia, hay que crear uno nuevo\n");
      if (flags & VFS_O_CREAT)
      {
         ciaaPOSIX_printf("open(): Se dio orden de crearlo, lo creo\n");
         /*Creo un nodo pelado en el dir_path*/
         /*vfs_inode_reserve hereda los atributos de fs del padre*/
         auxpath= (char *)path;
         ret = vfs_inode_reserve(auxpath, &target_inode);
         ASSERT_MSG(target_inode!=NULL, "open(): vfs_inode_reserve() failed");
         if(target_inode==NULL)
         {
            //Hubo un error al crear el nodo
            return -1;
         }
         /*Llenar todos los campos como kcreat*/
         /*Creo el archivo en la capa inferior*/
         parent_inode = target_inode->parent_node;
         ret = target_inode->fs_info.drv->driver_op->fs_create_node(parent_inode, target_inode);
         ASSERT_MSG(target_inode!=NULL, "open(): fs_create_node() failed");
         if(ret)
         {
            //No se pudo crear el nodo a bajo nivel. Manejar la situacion
            return -1;
         }
      }
      else
      {
      ciaaPOSIX_printf("open(): El nodo no existia, pero no se dio la orden de crearlo. Devuelvo error\n");
         //El nodo no existia pero no se dio la orden de crearlo. Devuelvo error
         return -1;
      }
   }
   else      //El nodo ya existe, no hay que crearlo, solo abrir el archivo
   {
      ciaaPOSIX_printf("open(): El nodo ya existia\n");
      /* When used with O_CREAT, if the file already exists it is an error
       * and the open() will fail. */
      if ((flags & VFS_O_EXCL) && (flags & VFS_O_CREAT))
      {
         ciaaPOSIX_printf("open(): El nodo ya existia y se ordeno crearlo. Error\n");
         return -1;
      }

      if(target_inode->n_info.type == VFS_FTDIR || target_inode->n_info.type == VFS_FTDIR)
      {
         ciaaPOSIX_printf("open(): El nodo es un directorio. Error\n");
         //No puedo abrir un directorio o un nodo desconocido
         return -1;
      }
   }

   file = file_desc_create(target_inode);
   ASSERT_MSG(file!=NULL, "open(): file_desc_create() failed");
   if(file == NULL)
   {
      return -1;
   }
   fd = file->index;
   //Como maneja Nuttx el tema del file_desc? En embox necesito manipular el archivo
   //No me alcanza con tener el numero fd.
   //Nuttx obtiene la lista de archivos del proceso y usa la estructura file correspondiente como argumento
   //fd = files_allocate(inode, oflags, 0, 0);
   //list = sched_getfiles();
   //open((struct file*)&list->fl_files[fd]);
   //Seria mas correcto, ya que desc contiene tambien al nodo, para que pasar argumentos redundantes?

   /*Parece tonto el open de bajo nivel, ya que ahora esta casi todo el nodo armado y se puede acceder facilmente
   al read sin preparacion. El tema es si quiero allocar un buffer que facilite el read o algo asi, mas adelante.
   Voy a necesitar open y close para no tener un monton de memoria ocupada sin necesidad por otros nodos*/
   ret = file->node->fs_info.drv->driver_op->file_open(file);
   ASSERT_MSG(ret>=0, "open(): file_open() failed");
   if(ret)
   {
      return -1;
   }

   //it = task_resource_idesc_table(task_self());
   //rc = idesc_table_add(it, (struct idesc *)kfile, 0);
   return fd;

}

extern ssize_t read(int fd, void *buf, size_t nbytes)
{
   struct file_desc *file;
   ssize_t ret;

  /* Get the thread-specific file list */
   file = file_desc_get(fd);
  /* Were we given a valid file descriptor? */
   ASSERT_MSG(file!=NULL, "read(): file_desc_get() failed");
   if(file==NULL)
   {
      //Invalid file descriptor
      return -1;
   }
  /* Yes.. Was this file opened for read access? */
   /* No.. File is not read-able. Error */

   /* Is a driver or mountpoint registered? If so, does it support
   * the read method?
   */
   /*No. Error*/
   if(file->node->fs_info.drv->driver_op->file_read == NULL)
   {
      //El archivo no soporta lectura
      return 1;
   }
  /* Yes.. then let it perform the read.  NOTE that for the case
   * of the mountpoint, we depend on the read methods bing
   * identical in signature and position in the operations vtable.
   */
   ret = file->node->fs_info.drv->driver_op->file_read(file, buf, nbytes);
   ASSERT_MSG(ret==nbytes, "read(): file_read() failed");
   if(ret!=nbytes)
   {
      //Setear ERROR para indicar que no se pudo leer todo
      //return -1;
   }

  /* If an error occurred, set errno and return -1 (ERROR) */

  /* Otherwise, return the number of bytes read */
   return ret;
}

/*

Diciembre: Tengo que ver como quiero que funcione el ejemplo.

vfs_init();
ramdisk_create(FS_DEV, FS_BLOCKS * PAGE_SIZE());
format(FS_DEV, FS_NAME);
mount(FS_DEV, FS_DIR, FS_NAME);
mkdir(FS_DIR1, 0777);
mkdir(FS_DIR2, 0777);
fd1 = creat(FS_FILE1, 0);
fd2 = creat(FS_FILE2, 0);

Noviembre. Tengo que empezar a construir minimamente el vfs.
ext2_mount_entry(): Importantisimo construir vfs_subtree_create().
ext2_open(): Tengo que construir vfs_get_relative_path().

prex: Como inicializo el fs en el boot?

static void
mount_fs(void)
{
   char line[128];
   FILE *fp;
   char *spec, *file, *type, *p;
   char nodev[] = "";
   int i;

   DPRINTF(("boot: mounting file systems\n"));


    // Mount root.

   if (mount("", "/", "ramfs", 0, NULL) < 0)
      sys_panic("boot: mount failed");


   // Create some default directories.

   i = 0;
   while (base_dir[i] != NULL) {
      if (mkdir(base_dir[i], 0) == -1)
         sys_panic("boot: mkdir failed");
      i++;
   }


    // Mount file system for /boot.

   if (mount("/dev/ram0", "/boot", "arfs", 0, NULL) < 0)
      sys_panic("boot: mount failed");


   //Mount file systems described in fstab.

   if ((fp = fopen("/boot/fstab", "r")) == NULL)
      sys_panic("boot: no fstab");

   for (;;) {
      if ((p = fgets(line, sizeof(line), fp)) == NULL)
         break;
      spec = strtok(p, " \t\n");
      if (spec == NULL || *spec == '#')
         continue;
      file = strtok(NULL, " \t\n");
      type = strtok(NULL, " \t\n");
      if (!strcmp(file, "/") || !strcmp(file, "/boot"))
         continue;
      if (!strcmp(spec, "none"))
         spec = nodev;

      // We create the mount point automatically
      mkdir(file, 0);
      mount(spec, file, type, 0, 0);
   }
   fclose(fp);
}

int mount(FAR const char *source, FAR const char *target,
          FAR const char *filesystemtype, unsigned long mountflags,
          FAR const void *data)
{

  FAR struct inode *blkdrvr_inode = NULL;
  FAR struct inode *mountpt_inode;
  FAR const struct mountpt_operations *mops;
  void *fshandle;
  int errcode;
  int ret;mops

  mops = mount_findfs(g_bdfsmap, filesystemtype);
  find_blockdriver(source, mountflags, &blkdrvr_inode);
  mount_findfs(g_nonbdfsmap, filesystemtype)) != NULL)
  inode_semtake();

  mountpt_inode = inode_find(target, NULL);
  inode_reserve(target, &mountpt_inode);
  ret = mops->bind(blkdrvr_inode, data, &fshandle);

  INODE_SET_MOUNTPT(mountpt_inode);

  mountpt_inode->u.i_mops  = mops;
  mountpt_inode->i_private = fshandle;
  inode_semgive();

  return OK;


errout_with_mountpt:
  mountpt_inode->i_crefs = 0;
  inode_remove(target);
  inode_semgive();
#ifdef BDFS_SUPPORT
#ifdef NONBDFS_SUPPORT
  if (blkdrvr_inode)
#endif
    {
       inode_release(blkdrvr_inode);
    }
#endif

  inode_release(mountpt_inode);
  goto errout;

errout_with_semaphore:
  inode_semgive();
#ifdef BDFS_SUPPORT
#ifdef NONBDFS_SUPPORT
  if (blkdrvr_inode)
#endif
    {
      inode_release(blkdrvr_inode);
    }
#endif

errout:
  set_errno(errcode);
  return ERROR;

#else
  fdbg("ERROR: No filesystems enabled\n");
  set_errno(ENOSYS);
  return ERROR;
#endif BDFS_SUPPORT || NONBDFS_SUPPORT
}
*/

/*
En la inicializacion voy a tener que montar un ramfs en el root (/) asociado a un
ramdevice. Luego voy a poder montar los otros fs, por ejemplo, un ext2 asociado al blockdevice SD.
Podre manejarme al principio sin el ramfs asociado al ramdevice?.
ext2_mount_entry(): Importantisimo construir vfs_subtree_create().
ext2_open(): Tengo que construir vfs_get_relative_path().

Necesito hacer algo como en embox, tener una estructura path donde esta el nodo y el mount?
struct path {
   struct node *node;
   struct mount_descriptor *mnt_desc;
};

Puede de alguna manera ser suficiente el node para tener toda la informacion?
Que tiene mnt_desc?
struct mount_descriptor {
   struct node *mnt_point;
   struct node *mnt_root;
   struct mount_descriptor *mnt_parent;
   struct dlist_head mnt_mounts;
   struct dlist_head mnt_child;
   char mnt_dev[MOUNT_DESC_STRINFO_LEN];
};

Podria tener una lista de mounts y que cada nodo tenga un elemento que apunte a un mount en particular.
Fijarse como esta implementado el mount en otros SO, por ejemplo prex.
Que pasa si no tengo una lista de mounts? en Prex parece que no hay nada asi, en mount solo agregan info del device.
En embox hago mount_table_add(&dir_node, root_path.node, dev) dentro de mount.


struct inode
{
  FAR struct inode *i_peer;     // Link to same level inode 
  FAR struct inode *i_child;    // Link to lower level inode 
  int16_t           i_crefs;    // References to inode 
  uint16_t          i_flags;    // Flags for inode 
  union inode_ops_u u;          // Inode operations 
#ifdef CONFIG_FILE_MODE
  mode_t            i_mode;     // Access mode flags 
#endif
  FAR void         *i_private;  // Per inode driver private data 
  char              i_name[1];  // Name of inode (variable) 
};
*/

/*
Tengo que planificar las estructuras de datos para el arbol.
Tomar ejemplo de Nuttx que es mas simple, o de embox que es mas complicado?
Como hace en Nuttx para ir al padre? (si el path es /..)
Intento 1: Pruebo implementarlo como Nuttx, agregando un puntero al nodo padre.

  vnode_t *parent_node;      //Link to parent inode
  vnode_t *sibling_node;     // Link to same level inode 
  vnode_t *child_node;    // Link to lower level inode

Si en Nuttx un nodo no tiene hermanos, como lo represento? Con NULL como peer.
"./" Tiene NULL de parent.
Copio los metodos de embox adaptandolos a las estructuras de Nuttx.
Implemento los metodos que aparecen en mount:
vfs_subtree_create(); vfs_get_relative_path();
struct node *vfs_subtree_create(struct node *parent, const char *path, mode_t mode)
static struct node *__vfs_subtree_create_child(struct node *parent, const char *name, size_t len, mode_t mode)

vfs_get_relative_path(): Sirve para tener el path desde root dentro del device. Como hago para saber cual es el root del mounted fs?
Recorro el arbol hacia arriba hasta que el padre sea un mount_point. El padre que es el mount_point es el root. Ahi comienza el path.
Como lo implementa en Nuttx? Todavia no lo encontre.
Idea: No manejarse por relative path. Siempre conservar el full path de una forma u otra. Recorrer el path desde el root hasta el leaf y fijarse en los nodos intermediarios si son mount_point. El ultimo con mount_point es el root_path del device del leaf. Fijarse como adaptar esto a embox. Seria bueno utilizar open(filep, RELPATH, oflags, mode), destacandose RELPATH. Ver inode_find() en Nuttx.
Donde necesito ext2_open() en embox? Puedo prescindir de este metodo? Lo importante de ext2_open() es que calcula el relative path.
Podria tener guardado el...
Estoy utilizando ext2_open() para buscar el path porque todavia no tengo en el vfs los nodos que representan los archivos.
*/


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
