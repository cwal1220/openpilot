#include <assert.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#define MMZ_ALLOC_MEM _IOWR('g', 1, unsigned long)
#define MMZ_FREE_MEM _IOWR('g', 2, unsigned long)
#define MMZ_DEV "/dev/mmz"

typedef struct {
  void *user_virt_addr;
  void *kernel_virt_addr;
  unsigned long mmz_phys;
  unsigned long length;
} mmz_data_type;

typedef struct mmz_node {
  mmz_data_type data;
  struct mmz_node *next;
} mmz_node;

static pthread_mutex_t mmz_mutex = PTHREAD_MUTEX_INITIALIZER;
static int fd_mmz = -1;
static mmz_node *plist = NULL;

static inline void thead_csi_dcache_clean_invalid_range(void *addr, uint64_t size) {
  uint64_t op_addr = (uint64_t)addr;
  int64_t op_size = size + op_addr % 64;
  const int64_t linesize = 64;

  __asm volatile("fence iorw, iorw");
  while (op_size > 0) {
    __asm volatile(".insn i 0x0b, 0, x0, %0, 0x027" : : "r"(op_addr));
    op_addr += linesize;
    op_size -= linesize;
  }
  __asm volatile("fence iorw, iorw");
  __asm volatile("fence.i");
  __asm volatile("fence r, r");
}

static mmz_node *mmz_create_node(void) {
  mmz_node *node = (mmz_node *)calloc(1, sizeof(mmz_node));
  if (node == NULL) {
    perror("calloc mmz_node");
    exit(1);
  }
  return node;
}

static void mmz_push_back(mmz_node **head, mmz_node *node) {
  assert(head);
  if (*head == NULL) {
    *head = node;
    return;
  }

  mmz_node *tail = *head;
  while (tail->next != NULL) tail = tail->next;
  tail->next = node;
}

static mmz_node *mmz_find(mmz_node *head, void *ptr) {
  for (mmz_node *cur = head; cur != NULL; cur = cur->next) {
    if (cur->data.user_virt_addr == ptr) return cur;
  }
  return NULL;
}

static void mmz_erase(mmz_node **head, mmz_node *node) {
  assert(head);
  assert(*head);
  assert(node);

  if (node == *head) {
    *head = node->next;
    free(node);
    return;
  }

  mmz_node *prev = *head;
  while (prev->next != node) prev = prev->next;
  prev->next = node->next;
  free(node);
}

static int mmz_init_locked(void) {
  fd_mmz = open(MMZ_DEV, O_RDWR | O_SYNC);
  if (fd_mmz < 0) {
    perror("open /dev/mmz");
    return -1;
  }
  return 0;
}

static int mmz_free_locked(unsigned long phy_addr, void *virt_addr) {
  (void)phy_addr;
  mmz_node *node = mmz_find(plist, virt_addr);
  if (node == NULL) return -1;

  if (munmap(virt_addr, node->data.length) == -1) {
    perror("munmap /dev/mmz");
  }

  ioctl(fd_mmz, MMZ_FREE_MEM, &node->data);
  mmz_erase(&plist, node);
  return 0;
}

int kd_mpi_sys_mmz_alloc(unsigned long *phy_addr, void **virt_addr, const char *mmb, const char *zone, unsigned int len) {
  (void)mmb;
  (void)zone;

  pthread_mutex_lock(&mmz_mutex);
  if (fd_mmz < 0 && mmz_init_locked() != 0) {
    pthread_mutex_unlock(&mmz_mutex);
    return -1;
  }

  mmz_node *node = mmz_create_node();
  node->data.length = len;
  if (ioctl(fd_mmz, MMZ_ALLOC_MEM, &node->data) != 0) {
    perror("MMZ_ALLOC_MEM");
    free(node);
    pthread_mutex_unlock(&mmz_mutex);
    return -1;
  }

  node->data.user_virt_addr = mmap(NULL, node->data.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mmz, node->data.mmz_phys);
  if (node->data.user_virt_addr == MAP_FAILED) {
    perror("mmap /dev/mmz");
    ioctl(fd_mmz, MMZ_FREE_MEM, &node->data);
    free(node);
    pthread_mutex_unlock(&mmz_mutex);
    return -1;
  }

  *phy_addr = node->data.mmz_phys;
  *virt_addr = node->data.user_virt_addr;
  mmz_push_back(&plist, node);
  pthread_mutex_unlock(&mmz_mutex);
  return 0;
}

int kd_mpi_sys_mmz_alloc_cached(unsigned long *phy_addr, void **virt_addr, const char *mmb, const char *zone, unsigned int len) {
  return kd_mpi_sys_mmz_alloc(phy_addr, virt_addr, mmb, zone, len);
}

int kd_mpi_sys_mmz_free(unsigned long phy_addr, void *virt_addr) {
  pthread_mutex_lock(&mmz_mutex);
  int ret = mmz_free_locked(phy_addr, virt_addr);
  pthread_mutex_unlock(&mmz_mutex);
  return ret;
}

int kd_mpi_sys_mmz_flush_cache(unsigned long phy_addr, void *virt_addr, unsigned int size) {
  (void)phy_addr;
  thead_csi_dcache_clean_invalid_range(virt_addr, size);
  return 0;
}

int kd_mpi_mmz_init(void) {
  pthread_mutex_lock(&mmz_mutex);
  int ret = fd_mmz >= 0 ? 0 : mmz_init_locked();
  pthread_mutex_unlock(&mmz_mutex);
  return ret;
}

int kd_mpi_mmz_deinit(void) {
  pthread_mutex_lock(&mmz_mutex);
  while (plist != NULL) {
    if (mmz_free_locked(plist->data.mmz_phys, plist->data.user_virt_addr) != 0) break;
  }
  if (fd_mmz >= 0) close(fd_mmz);
  fd_mmz = -1;
  pthread_mutex_unlock(&mmz_mutex);
  return 0;
}
