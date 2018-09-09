#ifndef __HEAP_H__
#define __HEAP_H__

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void heapInit(unsigned char *heap, size_t size);
void *heapMalloc(size_t size);
void heapFree(void *ptr);
void heapPrint(void);
void *heapRealloc(void *ptr, size_t size);
void *heapCalloc(size_t nitems, size_t size);
size_t getBytesToCopy(void);

size_t heapCopyTo(void *buffer);
void* heapRestoreFrom(void *buffer, size_t);

#ifdef __cplusplus
}
#endif

#endif // __HEAH_H__
