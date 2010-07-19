/* Copyright (c) 2008-2010, Advanced Micro Devices. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __KOSAPI_H
#define __KOSAPI_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "os_types.h"


//////////////////////////////////////////////////////////////////////////////
//   entrypoint abstraction
//////////////////////////////////////////////////////////////////////////////


#if defined(_WIN32) && !defined (_WIN32_WCE) && !defined(__SYMBIAN32__)
#define KOS_DLLEXPORT   __declspec(dllexport)
#define KOS_DLLIMPORT   __declspec(dllimport)
#elif defined(_WIN32) && defined (_WIN32_WCE)
#define KOS_DLLEXPORT   __declspec(dllexport)   
#define KOS_DLLIMPORT
#else
#define KOS_DLLEXPORT   extern
#define KOS_DLLIMPORT   
#endif // _WIN32


//////////////////////////////////////////////////////////////////////////////
//   KOS lib entrypoints
//////////////////////////////////////////////////////////////////////////////
#ifdef __KOSLIB_EXPORTS
#define KOS_API         KOS_DLLEXPORT
#else
#define KOS_API         KOS_DLLIMPORT
#endif // __KOSLIB_EXPORTS

//////////////////////////////////////////////////////////////////////////////
//                             assert API
//////////////////////////////////////////////////////////////////////////////
KOS_API void                    kos_assert_hook(const char* file, int line, int expression);

#if defined(DEBUG) || defined(DBG) || defined (_DBG) || defined (_DEBUG)

#if defined(_WIN32) && !defined(__SYMBIAN32__) || defined(_WIN32_WCE)
#include <assert.h>
#define KOS_ASSERT(expression)  assert(expression)
#elif defined(_BREW)
#include <assert.h>
#define KOS_ASSERT(expression)  kos_assert_hook(__FILE__, __LINE__, expression)
#elif defined(__SYMBIAN32__)
//#include <assert.h>
//#define   KOS_ASSERT(expression)  assert(expression)
#define KOS_ASSERT(expression)  /**/
#elif defined(__ARM__)
#define KOS_ASSERT(expression)
#elif defined(_LINUX)
#define KOS_ASSERT(expression) //kos_assert_hook(__FILE__, __LINE__, (int)(expression))
#endif

#else

#define KOS_ASSERT(expression)

#endif // DEBUG || DBG || _DBG

#if defined(_WIN32) && defined(_DEBUG) && !defined(_WIN32_WCE) && !defined(__SYMBIAN32__)
#pragma warning ( push, 3 )
#include <crtdbg.h>
#pragma warning  (pop)
#define KOS_MALLOC_DBG(size)    _malloc_dbg(size, _NORMAL_BLOCK, __FILE__, __LINE__)
#else
#define KOS_MALLOC_DBG(size)    kos_malloc(int size)
#endif // _WIN32 _DEBUG

#define kos_assert(expression)  KOS_ASSERT(expression)
#define kos_malloc_dbg(size)    KOS_MALLOC_DBG(size)

#ifdef UNDER_CE
#define KOS_PAGE_SIZE 0x1000
#endif

typedef enum mutexIndex mutexIndex_t;
//////////////////////////////////////////////////////////////////////////////
//  Interprocess shared memory initialization 
//////////////////////////////////////////////////////////////////////////////
// TODO: still valid?
KOS_API int             kos_sharedmem_create(unsigned int map_addr, unsigned int size);
KOS_API int             kos_sharedmem_destroy(void);

//////////////////////////////////////////////////////////////////////////////
//  heap API (per process)
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Allocate memory for a kernel side process.
 *
 *
 * \param   int size    Amount of bytes to be allocated.
 * \return  Pointer to the reserved memory, NULL if any error.
 *//*-------------------------------------------------------------------*/ 
KOS_API void*           kos_malloc(int size);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Allocate memory for a kernel side process. Clears the reserved memory.
 *
 *
 * \param   int num     Number of elements to allocate.
 * \param   int size    Element size in bytes.
 * \return  Pointer to the reserved memory, NULL if any error.
 *//*-------------------------------------------------------------------*/ 
KOS_API void*           kos_calloc(int num, int size);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Re-allocate an existing memory for a kernel side process.
 *          Contents of the old block will be copied to the new block
 *          taking the sizes of both blocks into account.
 *
 *
 * \param   void* memblock  Pointer to the old memory block.
 * \param   int size        Size of the new block in bytes.
 * \return  Pointer to the new memory block, NULL if any error.
 *//*-------------------------------------------------------------------*/ 
KOS_API void*           kos_realloc(void* memblock, int size);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Free a reserved memory block from the kernel side process.
 *
 *
 * \param   void* memblock  Pointer to the memory block.
 *//*-------------------------------------------------------------------*/ 
KOS_API void            kos_free(void* memblock);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Enable automatic memory leak checking performed at program exit.
 *
 *
 *//*-------------------------------------------------------------------*/ 
KOS_API void            kos_enable_memoryleakcheck(void);


//////////////////////////////////////////////////////////////////////////////
//  shared heap API (cross process)
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Allocate memory that can be shared between user and kernel
 *          side processes.
 *
 *
 * \param   int size    Amount of bytes to be allocated.
 * \return  Pointer to the new memory block, NULL if any error.
 *//*-------------------------------------------------------------------*/ 
KOS_API void*           kos_shared_malloc(int size);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Allocate memory that can be shared between user and kernel
 *          side processes. Clears the reserved memory.
 *
 *
 * \param   int num Number of elements to allocate.
 * \param   int size    Element size in bytes.
 * \return  Pointer to the reserved memory, NULL if any error.
 *//*-------------------------------------------------------------------*/ 
KOS_API void*           kos_shared_calloc(int num, int size);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Re-allocate an existing user/kernel shared memory block.
 *          Contents of the old block will be copied to the new block
 *          taking the sizes of both blocks into account.
 *
 *
 * \param   void* ptr   Pointer to the old memory block.
 * \param   int size    Size of the new block in bytes.
 * \return  Pointer to the new memory block, NULL if any error.
 *//*-------------------------------------------------------------------*/
KOS_API void*           kos_shared_realloc(void* ptr, int size);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Free a reserved shared memory block.
 *
 *
 * \param   void* ptr   Pointer to the memory block.
 *//*-------------------------------------------------------------------*/
 KOS_API void           kos_shared_free(void* ptr);


//////////////////////////////////////////////////////////////////////////////
//  memory API
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Copies the values of num bytes from the location pointed by src
 *          directly to the memory block pointed by dst.
 *
 *
 * \param   void* dst   Pointer to the destination memory block.
 * \param   void* src   Pointer to the source memory block.
 * \param   void* count Amount of bytes to copy.
 * \return  Returns the dst pointer, NULL if any error.
 *//*-------------------------------------------------------------------*/
KOS_API void*           kos_memcpy(void* dst, const void* src, int count);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Fills the destination memory block with the given value.
 *
 *
 * \param   void* dst   Pointer to the destination memory block.
 * \param   int value   Value to be written to each destination address.
 * \param   void* count Number of bytes to be set to the value.
 * \return  Returns the dst pointer, NULL if any error.
 *//*-------------------------------------------------------------------*/
KOS_API void*           kos_memset(void* dst, int value, int count);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Compares two memory blocks.
 *
 *
 * \param   void* dst   Pointer to the destination memory block.
 * \param   void* src   Pointer to the source memory block.
 * \param   void* count Number of bytes to compare.
 * \return  Zero if identical, >0 if first nonmatching byte is greater in dst.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_memcmp(void* dst, void* src, int count);


//////////////////////////////////////////////////////////////////////////////
//  physical memory API
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Allocates a physically contiguous memory block.
 *
 *
 * \param   void** virt_addr    Pointer where to store the virtual address of the reserved block.
 * \param   void** phys_addr    Pointer where to store the physical address of the reserved block.
 * \param   int pages           Number of pages to reserve (default page size = 4096 bytes).
 * \return  Zero if ok, othervise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_alloc_physical(void** virt_addr, void** phys_addr, int pages);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Free a physically contiguous allocated memory block.
 *
 *
 * \param   void* virt_addr     Virtual address of the memory block.
 * \param   int pages           Number of pages.
 * \return  Zero if ok, othervise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_free_physical(void* virt_addr, int pages);

KOS_API void            kos_memoryfence(void);


//////////////////////////////////////////////////////////////////////////////
//  string API
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Perform a string copy.
 *
 *
 * \param   void* strdestination    Pointer to destination memory.
 * \param   void* strsource     Pointer to the source string.
 * \return  Zero if ok, othervise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API char*           kos_strcpy(char* strdestination, const char* strsource);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Perform a string copy with given length.
 *
 *
 * \param   void* destination   Pointer to destination memory.
 * \param   void* source        Pointer to the source string.
 * \param   int length          Amount of bytes to copy.
 * \return  Returns the destination pointer.
 *//*-------------------------------------------------------------------*/
KOS_API char*           kos_strncpy(char* destination, const char* source, int length);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Append source string to destination string.
 *
 *
 * \param   void* strdestination    Pointer to destination string.
 * \param   void* strsource         Pointer to the source string.
 * \return  Returns the destination pointer.
 *//*-------------------------------------------------------------------*/
KOS_API char*           kos_strcat(char* strdestination, const char* strsource);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Compare two strings.
 *
 *
 * \param   void* string1   Pointer to first string.
 * \param   void* string2   Pointer to second string.
 * \param   void* length    Number of bytes to compare.
 * \return  Zero if identical, >0 if first string is lexically greater <0 if not.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_strcmp(const char* string1, const char* string2);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Compares two strings of given length.
 *
 *
 * \param   void* string1   Pointer to first string.
 * \param   void* string2   Pointer to second string.
 * \param   void* length    Number of bytes to compare.
 * \return  Zero if identical, >0 if first string is lexically greater <0 if not.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_strncmp(const char* string1, const char* string2, int length);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Calculates the length of a string..
 *
 *
 * \param   void* string    Pointer to the string.
 * \return  Lenght of the string in bytes.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_strlen(const char* string);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Convert an numeric ascii string to integer value.
 *
 *
 * \param   void* string    Pointer to the string.
 * \return  Integer value extracted from the string.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_atoi(const char* string);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Convert string to unsigned long integer.
 *
 *
 * \param   void* nptr      Pointer to the string.
 * \param   char** endptr   If not null, will be set to point to the next character after the number.
 * \param   int base        Base defining the type of the numeric string.
 * \return  Unsigned integer value extracted from the string.
 *//*-------------------------------------------------------------------*/
KOS_API unsigned int    kos_strtoul(const char* nptr, char** endptr, int base);


//////////////////////////////////////////////////////////////////////////////
//  sync API
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Create a mutex instance.
 *
 *
 * \param   void* name      Name string for the new mutex.
 * \return  Returns a handle to the mutex.
 *//*-------------------------------------------------------------------*/
KOS_API oshandle_t      kos_mutex_create(const char* name);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Get a handle to an already existing mutex.
 *
 *
 * \param   void* name      Name string for the new mutex.
 * \return  Returns a handle to the mutex.
 *//*-------------------------------------------------------------------*/
KOS_API oshandle_t      kos_mutex_open(const char* name);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Free the given mutex.
 *
 *
 * \param   oshandle_t mutexhandle  Handle to the mutex.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_mutex_free(oshandle_t mutexhandle);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Lock the given mutex.
 *
 *
 * \param   oshandle_t mutexhandle  Handle to the mutex.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_mutex_lock(oshandle_t mutexhandle);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Try to lock the given mutex, if already locked returns immediately.
 *
 *
 * \param   oshandle_t mutexhandle  Handle to the mutex.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_mutex_locktry(oshandle_t mutexhandle);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Try to lock the given mutex by waiting for its release. Returns without locking if the
 * mutex is already locked and cannot be acquired within the given period.
 *
 *
 * \param   oshandle_t mutexhandle  Handle to the mutex.
 * \param   int millisecondstowait  Time to wait for the mutex to be available.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_mutex_lockwait(oshandle_t mutexhandle, int millisecondstowait);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Unlock the given mutex.
 *
 *
 * \param   oshandle_t mutexhandle  Handle to the mutex.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_mutex_unlock(oshandle_t mutexhandle);

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Increments (increases by one) the value of the specified 32-bit variable as an atomic operation.
 *
 *
 * \param   int* ptr Pointer to the value to be incremented.
 * \return  Returns the new incremented value.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_interlock_incr(int* ptr);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Decrements (decreases by one) the value of the specified 32-bit variable as an atomic operation.
 *
 *
 * \param   int* ptr Pointer to the value to be decremented.
 * \return  Returns the new decremented value.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_interlock_decr(int* ptr);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Atomic replacement of a value.
 *
 *
 * \param   int* ptr Pointer to the value to be replaced.
 * \param   int value The new value.
 * \return  Returns the old value.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_interlock_xchg(int* ptr, int value);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Perform an atomic compare-and-exchange operation on the specified values. Compares the two specified 32-bit values and exchanges
* with another 32-bit value based on the outcome of the comparison.
 *
 *
 * \param   int* ptr Pointer to the value to be replaced.
 * \param   int value The new value.
 * \param   int compvalue Value to be compared with.
 * \return  Returns the initial value of the first given parameter.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_interlock_compxchg(int* ptr, int value, int compvalue);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Atomic addition of two 32-bit values.
 *
 *
 * \param   int* ptr Pointer to the target value.
 * \param   int value Value to be added to the target.
 * \return  Returns the initial value of the target.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_interlock_xchgadd(int* ptr, int value);

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Create an event semaphore.
 *
 *
 * \param   int a_manualReset Selection for performing reset manually (or by the system).
 * \return  Returns an handle to the created semaphore.
 *//*-------------------------------------------------------------------*/
KOS_API oshandle_t      kos_event_create(int a_manualReset);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Destroy an event semaphore.
 *
 *
 * \param   oshandle_t a_event Handle to the semaphore.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_event_destroy(oshandle_t a_event);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Signal an event semaphore.
 *
 *
 * \param   oshandle_t a_event Handle to the semaphore.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_event_signal(oshandle_t a_event);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Reset an event semaphore.
 *
 *
 * \param   oshandle_t a_event Handle to the semaphore.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_event_reset(oshandle_t a_event);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Wait for an event semaphore to be freed and acquire it.
 *
 *
 * \param   oshandle_t a_event Handle to the semaphore.
 * \param   int a_milliSeconds Time to wait.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_event_wait(oshandle_t a_event, int a_milliSeconds);


//////////////////////////////////////////////////////////////////////////////
//  interrupt handler API
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Enable an interrupt with specified id.
 *
 *
 * \param   int interrupt   Identification number for the interrupt.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_interrupt_enable(int interrupt);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Disable an interrupt with specified id.
 *
 *
 * \param   int interrupt   Identification number for the interrupt.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_interrupt_disable(int interrupt);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Set the callback function for an interrupt.
 *
 *
 * \param   int interrupt   Identification number for the interrupt.
 * \param   void* handler   Pointer to the callback function.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_interrupt_setcallback(int interrupt, void* handler);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Remove a callback function from an interrupt.
 *
 *
 * \param   int interrupt   Identification number for the interrupt.
 * \param   void* handler   Pointer to the callback function.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_interrupt_clearcallback(int interrupt, void* handler);


//////////////////////////////////////////////////////////////////////////////
//  thread and process API
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Allocate an entry from the thread local storage table.
 *
 *
 * \return  Index of the reserved entry.
 *//*-------------------------------------------------------------------*/
KOS_API unsigned int    kos_tls_alloc(void);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Free an entry from the thread local storage table.
 *
 *
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_tls_free(unsigned int tlsindex);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Read the value of an entry in the thread local storage table.
 *
 *
 * \param   unsigned int tlsindex   Index of the entry.
 * \return  Returns the value of the entry.
 *//*-------------------------------------------------------------------*/
KOS_API void*           kos_tls_read(unsigned int tlsindex);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Write a value to an entry in the thread local storage table.
 *
 *
 * \param   unsigned int tlsindex   Index of the entry.
 * \param   void* tlsvalue          Value to be written.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_tls_write(unsigned int tlsindex, void* tlsvalue);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Put the thread to sleep for the given time period.
 *
 *
 * \param   unsigned int milliseconds   Time in milliseconds.
 *//*-------------------------------------------------------------------*/
KOS_API void            kos_sleep(unsigned int milliseconds);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Get the id of the current process.
 *
 *
 * \return  Returns the process id.
 *//*-------------------------------------------------------------------*/
KOS_API unsigned int    kos_process_getid(void);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Get the id of the current caller process.
 *
 *
 * \return  Returns the caller process id.
 *//*-------------------------------------------------------------------*/
KOS_API unsigned int    kos_callerprocess_getid(void);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Get the id of the current thread.
 *
 *
 * \return  Returns the thread id.
 *//*-------------------------------------------------------------------*/
KOS_API unsigned int    kos_thread_getid(void);

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Create a new thread.
 *
 *
 * \param   oshandle_t a_function Handle to the function to be executed in the thread.
 * \param   unsigned int* a_threadId Pointer to a value where to store the ID of the new thread.
 * \return  Returns an handle to the created thread.
 *//*-------------------------------------------------------------------*/
KOS_API oshandle_t      kos_thread_create(oshandle_t a_function, unsigned int* a_threadId);

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Destroy the given thread.
 *
 *
 * \param   oshandle_t a_task Handle to the thread to be destroyed.
 *//*-------------------------------------------------------------------*/
KOS_API void            kos_thread_destroy( oshandle_t a_task );

//////////////////////////////////////////////////////////////////////////////
//  timing API
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Get the current time as a timestamp.
 *
 *
 * \return  Returns the timestamp.
 *//*-------------------------------------------------------------------*/
KOS_API unsigned int    kos_timestamp(void);


//////////////////////////////////////////////////////////////////////////////
//  libary API
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Map the given library (not required an all OS'es).
 *
 *
 * \param   char* libraryname   The name string of the lib.
 * \return  Returns a handle for the lib.
 *//*-------------------------------------------------------------------*/
KOS_API oshandle_t      kos_lib_map(char* libraryname);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Unmap the given library.
 *
 * \param   oshandle_t libhandle Handle to the lib.
 * \return  Returns an error code incase of an error.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_lib_unmap(oshandle_t libhandle);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Get the address of a lib.
 *
 * \param   oshandle_t libhandle Handle to the lib.
 * \return  Returns a pointer to the lib.
 *//*-------------------------------------------------------------------*/
KOS_API void*           kos_lib_getaddr(oshandle_t libhandle, char* procname);


//////////////////////////////////////////////////////////////////////////////
//  query API
//////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Get device system info.
 *
 * \param   os_sysinfo_t* sysinfo   Pointer to the destination sysinfo structure.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_get_sysinfo(os_sysinfo_t* sysinfo);

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Get system status info.
 *
 * \param   os_stats_t* stats   Pointer to the destination stats structure.
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_get_stats(os_stats_t* stats);

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Sync block start
 *
 * \param   void
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_syncblock_start(void);
/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Sync block end
 *
 * \param   void
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_syncblock_end(void);

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Sync block start with argument
 *
 * \param   void
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int kos_syncblock_start_ex( mutexIndex_t a_index );

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Sync block start with argument
 *
 * \param   void
 * \return  Returns NULL if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int kos_syncblock_end_ex( mutexIndex_t a_index );

//////////////////////////////////////////////////////////////////////////////
//  file API
//////////////////////////////////////////////////////////////////////////////

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Opens a file
 *
 * \param   const char* filename    Name of the file to open.
 * \param   const char* mode        Mode used for file opening. See fopen.
 * \return  Returns file handle or NULL if error.
 *//*-------------------------------------------------------------------*/
KOS_API oshandle_t      kos_fopen(const char* filename, const char* mode);

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Writes to a file
 *
 * \param   oshandle_t file     Handle of the file to write to.
 * \param   const char* format  Format string. See fprintf.
 * \return  Returns the number of bytes written
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_fprintf(oshandle_t file, const char* format, ...);

/*-------------------------------------------------------------------*//*!
 * \external
 * \brief   Closes a file
 *
 * \param   oshandle_t file     Handle of the file to close.
 * \return  Returns zero if no error, otherwise an error code.
 *//*-------------------------------------------------------------------*/
KOS_API int             kos_fclose(oshandle_t file);

#ifdef  __SYMBIAN32__
KOS_API void kos_create_dfc(void);
KOS_API void kos_signal_dfc(void);
KOS_API void kos_enter_critical_section();
KOS_API void kos_leave_critical_section();
#endif  // __SYMBIAN32__

#ifdef __cplusplus
}
#endif // __cplusplus
#endif  // __KOSAPI_H
