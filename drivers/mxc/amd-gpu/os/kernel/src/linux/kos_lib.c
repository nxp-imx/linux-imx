/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
 
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/limits.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <asm/current.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include "kos_libapi.h"

//////////////////////////////////////////////////////////////////////////////
//  defines
//////////////////////////////////////////////////////////////////////////////
#define KOS_STATS_ENABLE
#define _CRT_SECURE_NO_DEPRECATE


//////////////////////////////////////////////////////////////////////////////
//  includes
//////////////////////////////////////////////////////////////////////////////
#define __KOSLIB_EXPORTS


#define KOS_MALLOC(s)               kmalloc(s, GFP_KERNEL)
#define KOS_CALLOC(num, size)       kcalloc(num, size, GFP_KERNEL)
#define KOS_REALLOC(p, s)           krealloc(p, s, GFP_KERNEL)
#define KOS_FREE(p)                 kfree(p); p = 0
#define KOS_DBGFLAGS_SET(flag)


//////////////////////////////////////////////////////////////////////////////
// stats
//////////////////////////////////////////////////////////////////////////////

#ifdef KOS_STATS_ENABLE

os_stats_t kos_stats = {0, 0, 0, 0, 0, 0, 0, 0};

#define KOS_STATS(x) x

#else
#define KOS_STATS(x)
#endif // KOS_STATS_ENABLE


extern unsigned int get_current_fd(void); // QUICK HACK


//////////////////////////////////////////////////////////////////////////////
//  assert API
//////////////////////////////////////////////////////////////////////////////
KOS_API void
kos_assert_hook(const char* file, int line, int expression)
{
    if (expression)
    {
        return;
    }
    else
    {
        printk(KERN_ERR "Assertion failed at %s:%d!\n", file, line);
        //BUG();
    }

    // put breakpoint here
}


//////////////////////////////////////////////////////////////////////////////
//  heap API (per process)
//////////////////////////////////////////////////////////////////////////////
KOS_API void*
kos_malloc(int size)
{
    void* ptr = KOS_MALLOC(size);

    KOS_ASSERT(ptr);
    KOS_STATS(kos_stats.heap_allocs++);
    KOS_STATS(kos_stats.heap_alloc_bytes += size);

    return (ptr);
}


//----------------------------------------------------------------------------

KOS_API void*
kos_calloc(int num, int size)
{
    void* ptr = KOS_CALLOC(num, size);

    KOS_ASSERT(ptr);
    KOS_STATS(kos_stats.heap_allocs++);
    KOS_STATS(kos_stats.heap_alloc_bytes += (size * num));

    return (ptr);
}

//----------------------------------------------------------------------------

KOS_API void*
kos_realloc(void* ptr, int size)
{
    void* newptr;

    KOS_ASSERT(ptr);
    newptr = KOS_REALLOC(ptr, size);

    KOS_ASSERT(newptr);

    return (newptr);
}

//----------------------------------------------------------------------------

KOS_API void
kos_free(void* ptr)
{
    KOS_STATS(kos_stats.heap_frees++);

    KOS_FREE(ptr);
}


//////////////////////////////////////////////////////////////////////////////
//  shared heap API (cross process)
//////////////////////////////////////////////////////////////////////////////
KOS_API void*
kos_shared_malloc(int size)
{
    void* ptr;

    ptr = NULL; // shared alloc

    KOS_ASSERT(ptr);
    KOS_STATS(kos_stats.shared_heap_allocs++);
    KOS_STATS(kos_stats.shared_heap_alloc_bytes += size);

    return (ptr);
}

//----------------------------------------------------------------------------

KOS_API void*
kos_shared_calloc(int num, int size)
{
    void* ptr;

    ptr = NULL; // shared calloc

    KOS_ASSERT(ptr);
    KOS_STATS(kos_stats.shared_heap_allocs++);
    KOS_STATS(kos_stats.shared_heap_alloc_bytes += (size * num));
    return (ptr);
}

//----------------------------------------------------------------------------

KOS_API void*
kos_shared_realloc(void* ptr, int size)
{
    void* newptr;
    (void) ptr;      // unreferenced formal parameter
    (void) size;     // unreferenced formal parameter

    newptr = NULL; // shared realloc

    KOS_ASSERT(newptr);

    return (newptr);
}

//----------------------------------------------------------------------------

KOS_API void
kos_shared_free(void* ptr)
{
    (void) ptr;      // unreferenced formal parameter
    KOS_ASSERT(0);   // not implemented

    KOS_STATS(kos_stats.shared_heap_frees++);

    // shared free
}

//////////////////////////////////////////////////////////////////////////////
//  memory access API
//////////////////////////////////////////////////////////////////////////////
KOS_API void*
kos_memcpy(void* dst, const void* src, int count)
{
    KOS_ASSERT(src);
    KOS_ASSERT(dst);
    return memcpy(dst, src, count);
}

//----------------------------------------------------------------------------

KOS_API void*
kos_memset(void* dst, int value, int count)
{
    KOS_ASSERT(dst);
    return memset(dst, value, count);
}

//----------------------------------------------------------------------------

KOS_API int
kos_memcmp(void* dst, void* src, int count)
{
    KOS_ASSERT(src);
    KOS_ASSERT(dst);
    return memcmp(dst, src, count);
}

//////////////////////////////////////////////////////////////////////////////
//  physical memory API
//////////////////////////////////////////////////////////////////////////////
KOS_API int
kos_alloc_physical(void** virt_addr, void** phys_addr, int pages)
{
    *virt_addr = dma_alloc_coherent(NULL, pages*PAGE_SIZE, (dma_addr_t*)*phys_addr, GFP_DMA | GFP_KERNEL);
    if(*virt_addr == NULL)
        printk(KERN_ERR "%s:dma_alloc_coherent error:pages=%d\n", __func__, pages);

    return OS_SUCCESS;
}

//----------------------------------------------------------------------------

KOS_API int
kos_free_physical(void* virt_addr, int pages)
{
    (void) virt_addr;      // unreferenced formal parameter
    (void) pages;         // unreferenced formal parameter

    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API int
kos_map_physical(void** virt_addr, void** phys_addr, int pages)
{
    (void) virt_addr;      // unreferenced formal parameter
    (void) phys_addr;     // unreferenced formal parameter
    (void) pages;         // unreferenced formal parameter

    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API int
kos_unmap_physical(void* virt_addr, int pages)
{
    (void) virt_addr;      // unreferenced formal parameter
    (void) pages;         // unreferenced formal parameter

    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API void
kos_memoryfence(void)
{
}

//----------------------------------------------------------------------------

KOS_API void
kos_enable_memoryleakcheck(void)
{
    // perform automatic leak checking at program exit
    KOS_DBGFLAGS_SET(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
}

//////////////////////////////////////////////////////////////////////////////
//  string API
//////////////////////////////////////////////////////////////////////////////

KOS_API char*
kos_strcpy(char* strdestination, const char* strsource)
{
    KOS_ASSERT(strdestination);
    KOS_ASSERT(strsource);
    return strcpy(strdestination, strsource);
}

//----------------------------------------------------------------------------

KOS_API char*
kos_strncpy(char* destination, const char* source, int length)
{
    KOS_ASSERT(destination);
    KOS_ASSERT(source);
    return strncpy(destination, source, length);
}

//----------------------------------------------------------------------------

KOS_API char*
kos_strcat(char* strdestination, const char* strsource)
{
    KOS_ASSERT(strdestination);
    KOS_ASSERT(strsource);
    return strcat(strdestination, strsource);
}

//----------------------------------------------------------------------------

KOS_API int
kos_strcmp(const char* string1, const char* string2)
{
    KOS_ASSERT(string1);
    KOS_ASSERT(string2);
    return strcmp(string1, string2);
}

//----------------------------------------------------------------------------

KOS_API int
kos_strncmp(const char* string1, const char* string2, int length)
{
    KOS_ASSERT(string1);
    KOS_ASSERT(string2);
    return strncmp(string1, string2, length);
}

//----------------------------------------------------------------------------

KOS_API int
kos_strlen(const char* string)
{
    KOS_ASSERT(string);
    return strlen(string);
}

//----------------------------------------------------------------------------
#if 0
KOS_API int
kos_atoi(const char* string)
{
    return atoi(string);
}

//----------------------------------------------------------------------------

KOS_API unsigned int
kos_strtoul(const char* nptr, char** endptr, int base)
{
    return (unsigned int)strtoul(nptr, endptr, base);
}
#endif


//////////////////////////////////////////////////////////////////////////////
//  sync API
//////////////////////////////////////////////////////////////////////////////

#define kos_mutex_get(mutexhandle, mutex)       \
    if (!mutexhandle)   return (OS_FAILURE);    \
    mutex = (struct mutex *)mutexhandle;

//----------------------------------------------------------------------------

oshandle_t
kos_mutex_create(char* name)
{
    struct mutex *mutex;

    mutex = kos_malloc(sizeof(struct mutex));
    KOS_ASSERT(mutex);
    if (!mutex) return (OS_HANDLE_NULL);

    mutex_init(mutex);

    KOS_STATS(kos_stats.objects_alloc++);

    return (oshandle_t)mutex;
}

//----------------------------------------------------------------------------
/*
KOS_API oshandle_t
kos_mutex_open(char* name)
{
    // only for inter-process mutexes
    if (name == NULL)
        return (OS_HANDLE_NULL);

    return (kos_mutex_create(name));
}
*/

//----------------------------------------------------------------------------

KOS_API int
kos_mutex_free(oshandle_t mutexhandle)
{
    struct mutex *mutex;
    kos_mutex_get(mutexhandle, mutex);

    KOS_ASSERT(mutex);
    kos_free(mutex);

    KOS_STATS(kos_stats.objects_free++);

    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API int
kos_mutex_lock(oshandle_t mutexhandle)
{
    struct mutex *mutex;
    kos_mutex_get(mutexhandle, mutex);

    KOS_ASSERT(mutex);
    mutex_lock(mutex);

    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API int
kos_mutex_locktry(oshandle_t mutexhandle)
{
    struct mutex    *mutex;
    kos_mutex_get(mutexhandle, mutex);

    KOS_ASSERT(mutex);
    return mutex_trylock(mutex);
}

//----------------------------------------------------------------------------
#if 0
KOS_API int
kos_mutex_lockwait(oshandle_t mutexhandle, int millisecondstowait)
{
    struct mutex    *mutex;
    kos_mutex_get(mutexhandle, mutex);

    if (mutex->named)
    {
        DWORD   result;

        if (millisecondstowait == OS_INFINITE) millisecondstowait = INFINITE;

        result = WaitForSingleObject(mutex->handle, millisecondstowait);

        if (result != WAIT_OBJECT_0) return (OS_FAILURE);
    }
    else
    {
        // WTF???? windows does not have this but wince50 does??
        //BOOL  result;
        //result = TryEnterCriticalSection(mutex->critical_section);
        //if (!result) return (OS_FAILURE);
        EnterCriticalSection(&mutex->critical_section);
    }

    return (OS_SUCCESS);
}
#endif
//----------------------------------------------------------------------------

KOS_API int
kos_mutex_unlock(oshandle_t mutexhandle)
{
    struct mutex    *mutex;
    kos_mutex_get(mutexhandle, mutex);

    KOS_ASSERT(mutex);
    mutex_unlock(mutex);

    return (OS_SUCCESS);
}

KOS_API unsigned int
kos_process_getid(void)
{
return get_current_fd();
//  return current->pid;
}

//----------------------------------------------------------------------------
#if 0
KOS_API int
kos_interlock_incr(int* ptr)
{
    int retval;

    retval = InterlockedIncrement((LONG*)ptr);

    return (retval);
}

//----------------------------------------------------------------------------

KOS_API int
kos_interlock_decr(int* ptr)
{
    int retval;

    retval = InterlockedDecrement((LONG*)ptr);

    return (retval);
}

//----------------------------------------------------------------------------

KOS_API int
kos_interlock_xchg(int* ptr, int value)
{
    int retval;

    retval = InterlockedExchange((LONG*)ptr, value);

    return (retval);
}

//----------------------------------------------------------------------------

KOS_API int
kos_interlock_compxchg(int* ptr, int value, int compvalue)
{
    int retval;

    retval = InterlockedCompareExchange((LONG*)ptr, value, compvalue);

    return ((int)retval);
}

//----------------------------------------------------------------------------

KOS_API int
kos_interlock_xchgadd(int* ptr, int value)
{
    int retval;

    retval = InterlockedExchangeAdd((LONG*)ptr, value);

    return (retval);
}
#endif

/* ------------------------------------------------------------------- *//*
 * \brief           Creates new event semaphore
 * \param           uint32 a_manualReset
 *                  When this param is zero, system automatically resets the
 *                  event state to nonsignaled after waiting thread has been
 *                  released
 * \return          oshandle_t
*//* ------------------------------------------------------------------- */
KOS_API oshandle_t
kos_event_create(int a_manualReset)
{
    struct completion *comp = KOS_MALLOC(sizeof(struct completion));

    KOS_ASSERT(comp);
    if(!comp)
    {
        return (oshandle_t)NULL;
    }

    init_completion(comp);

    return (oshandle_t)comp;
}

/* ------------------------------------------------------------------- *//*
 * \brief           Frees event semaphore
 * \param           oshandle_t a_event, event semaphore
 * \return          int
*//* ------------------------------------------------------------------- */
KOS_API int
kos_event_destroy(oshandle_t a_event)
{
    struct completion *comp = (struct completion *)a_event;

    KOS_ASSERT(comp);
//  KOS_ASSERT(completion_done(comp));

    KOS_FREE(comp);
    return (OS_SUCCESS);
}

/* ------------------------------------------------------------------- *//*
 * \brief           Signals event semaphore
 * \param           oshandle_t a_event, event semaphore
 * \return          int
*//* ------------------------------------------------------------------- */
KOS_API int
kos_event_signal(oshandle_t a_event)
{
    struct completion *comp = (struct completion *)a_event;

    KOS_ASSERT(comp);
    complete_all(comp);     // perhaps complete_all?
    return (OS_SUCCESS);
}

/* ------------------------------------------------------------------- *//*
 * \brief           Resets event semaphore state to nonsignaled
 * \param           oshandle_t a_event, event semaphore
 * \return          int
*//* ------------------------------------------------------------------- */
KOS_API int
kos_event_reset(oshandle_t a_event)
{
    struct completion *comp = (struct completion *)a_event;

    KOS_ASSERT(comp);
    INIT_COMPLETION(*comp);
    return (OS_SUCCESS);
}

/* ------------------------------------------------------------------- *//*
 * \brief           Waits event semaphore to be signaled
 * \param           oshandle_t a_event, event semaphore
 * \return          int
*//* ------------------------------------------------------------------- */
KOS_API int
kos_event_wait(oshandle_t a_event, int a_milliSeconds)
{
    struct completion *comp = (struct completion *)a_event;

    KOS_ASSERT(comp);
    if(a_milliSeconds == OS_INFINITE)
    {
        wait_for_completion_killable(comp);
    }
    else
    {
        // should interpret milliseconds really to jiffies?
        if(!wait_for_completion_timeout(comp, msecs_to_jiffies(a_milliSeconds)))
        {
            return (OS_FAILURE);
        }
    }
    return (OS_SUCCESS);
}

#if 0
//////////////////////////////////////////////////////////////////////////////
//  interrupt handler API
//////////////////////////////////////////////////////////////////////////////
KOS_API int
kos_interrupt_enable(int interrupt)
{
    (void) interrupt;  // unreferenced formal parameter
    KOS_ASSERT(0);     // not implemented

    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API int
kos_interrupt_disable(int interrupt)
{
    (void) interrupt;  // unreferenced formal parameter
    KOS_ASSERT(0);     // not implemented

    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API int
kos_interrupt_setcallback(int interrupt, void* handler)
{

    KOS_ASSERT(handler);

    (void) interrupt;  // unreferenced formal parameter
    (void) handler;    // unreferenced formal parameter
    KOS_ASSERT(0);     // not implemented

    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API int
kos_interrupt_clearcallback(int interrupt, void* handler)
{
    KOS_ASSERT(handler);

    (void) interrupt;  // unreferenced formal parameter
    (void) handler;    // unreferenced formal parameter
    KOS_ASSERT(0);     // not implemented

    return (OS_SUCCESS);
}


//////////////////////////////////////////////////////////////////////////////
//  thread and process API
//////////////////////////////////////////////////////////////////////////////
KOS_API unsigned int
kos_tls_alloc(void)
{
    DWORD index;
    index = TlsAlloc();

    if (index == TLS_OUT_OF_INDEXES) return (OS_TLS_OUTOFINDEXES);

    return (index);
}

//----------------------------------------------------------------------------

KOS_API int
kos_tls_free(unsigned int tlsindex)
{
    BOOL result;

    result = TlsFree(tlsindex);

    if (!result) return (OS_FAILURE);
    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API void*
kos_tls_read(unsigned int tlsindex)
{
    void* value;

    value = TlsGetValue(tlsindex);

    return (value);
}

//----------------------------------------------------------------------------

KOS_API int
kos_tls_write(unsigned int tlsindex, void* tlsvalue)
{
    BOOL result;

    result = TlsSetValue(tlsindex, tlsvalue);

    if (!result) return (OS_FAILURE);
    return (OS_SUCCESS);
}

#endif

//----------------------------------------------------------------------------

KOS_API void
kos_sleep(unsigned int milliseconds)
{
    msleep(milliseconds);
}

#if 0
//----------------------------------------------------------------------------

KOS_API unsigned int
kos_process_getid(void)
{
    DWORD id;

    id = GetCurrentProcessId();

    return(id);
}

//----------------------------------------------------------------------------

KOS_API unsigned int
kos_thread_getid(void)
{
    DWORD id;

    id = GetCurrentThreadId();

    return (id);
}


//////////////////////////////////////////////////////////////////////////////
//  timing API
//////////////////////////////////////////////////////////////////////////////
KOS_API unsigned int
kos_timestamp(void)
{
    unsigned int milliseconds;

    milliseconds = GetTickCount();

    return (milliseconds);
}


//////////////////////////////////////////////////////////////////////////////
//  libary API
//////////////////////////////////////////////////////////////////////////////
KOS_API oshandle_t
kos_lib_map(char* libraryname)
{
    kos_library_t* lib;
    HMODULE module;

    KOS_ASSERT(libraryname);

    module = LoadLibrary(libraryname);

    if (!module) return (OS_HANDLE_NULL);

    lib = (kos_library_t*)kos_malloc(sizeof(kos_library_t));

    if (!lib)
    {
        FreeLibrary(module);
        return (OS_HANDLE_NULL);
    }

    lib->module = module;

    KOS_STATS(kos_stats.objects_alloc++);

    return (oshandle_t) lib;
}

//----------------------------------------------------------------------------

KOS_API int
kos_lib_unmap(oshandle_t libhandle)
{
    BOOL result;
    kos_library_t* lib = (kos_library_t*)libhandle;

    KOS_ASSERT(lib);

    result = FreeLibrary(lib->module);

    kos_free(lib);

    KOS_STATS(kos_stats.objects_free++);

    if (!result) return (OS_FAILURE);
    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

KOS_API void*
kos_lib_getaddr(oshandle_t libhandle, char* procname)
{
    FARPROC procaddr;
    kos_library_t* lib = (kos_library_t*)libhandle;

    KOS_ASSERT(lib);

    procaddr = GetProcAddress(lib->module, procname);

    return (void*)procaddr;
}
#endif

//////////////////////////////////////////////////////////////////////////////
//  query API
//////////////////////////////////////////////////////////////////////////////

static int
kos_get_endianness(void)
{
    int  value;
    char* ptr;

    value = 0x01FFFF00;

    ptr = (char*)&value;

    KOS_ASSERT((*ptr == 0x00) || (*ptr == 0x01));

    return (int)*ptr;
}

//----------------------------------------------------------------------------

KOS_API int
kos_get_sysinfo(os_sysinfo_t* sysinfo)
{
    KOS_ASSERT(sysinfo);
    if (!sysinfo) return (OS_FAILURE);

    sysinfo->cpu_mhz            = 0;
    sysinfo->cpu_type           = 0;
    sysinfo->cpu_version        = 0;
    sysinfo->os_type            = 0;
    sysinfo->os_version         = 0;
    sysinfo->sysmem_size        = 0;
    sysinfo->page_size          = 0x1000;
    sysinfo->max_path           = PATH_MAX;
//  sysinfo->tls_slots          = TLS_MINIMUM_AVAILABLE - 1;
    sysinfo->endianness         = kos_get_endianness();

    return (OS_SUCCESS);
}

//----------------------------------------------------------------------------

#ifdef KOS_STATS
KOS_API int
kos_get_stats(os_stats_t* stats)
{
    kos_memcpy(stats, &kos_stats, sizeof(os_stats_t));
    return (OS_SUCCESS);
}
#else
KOS_API int
kos_get_stats(os_stats_t* stats)
{
    return (KOS_NOTSUPPORTED);
}
#endif // KOS_STATS

/*-------------------------------------------------------------------*//*!
 * \brief Sync block API
 *        Same mutex needed from different blocks of driver
 *//*-------------------------------------------------------------------*/

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Sync block start
 *
 * \param   void
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/

static struct mutex* syncblock_mutex = 0;

KOS_API int kos_syncblock_start(void)
{
    int return_value;

    if(!syncblock_mutex)
    {
        syncblock_mutex = kos_mutex_create("syncblock");
    }

    if(syncblock_mutex)
    {
        return_value = kos_mutex_lock(syncblock_mutex);
    }
    else
    {
        return_value = -1;
    }

    return return_value;
}
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Sync block end
 *
 * \param   void
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int kos_syncblock_end(void)
{
    int return_value;

    if(syncblock_mutex)
    {
        return_value = kos_mutex_unlock(syncblock_mutex);
    }
    else
    {
        return_value = -1;
    }

    return return_value;
}

KOS_API oshandle_t kos_thread_create(oshandle_t a_function, unsigned int* a_threadId)
{
	struct task_struct *task = kthread_run(a_function, 0, "kos_thread_%p", a_threadId);
	*a_threadId = (unsigned int)task;
	return (oshandle_t)task;
}

KOS_API void kos_thread_destroy( oshandle_t a_task )
{
	kthread_stop((struct task_struct *)a_task);
}
