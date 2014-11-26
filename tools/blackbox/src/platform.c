#include "platform.h"

void thread_create(thread_t *thread, void*(*threadFunc)(void *data), void *data)
{
#if defined(WIN32)
	*thread = CreateThread(NULL, 0, threadFunc, data, 0);
#else
    pthread_create(thread, NULL, threadFunc, data);
#endif
}

void semaphore_signal(semaphore_t *sem)
{
#if defined(__APPLE__)
	dispatch_semaphore_signal(*sem);
#elif defined(WIN32)
	ReleaseSemaphore(*sem, 1, NULL);
#else
	sem_post(sem);
#endif
}

void semaphore_wait(semaphore_t *sem)
{
#if defined(__APPLE__)
	dispatch_semaphore_wait(*sem, DISPATCH_TIME_FOREVER);
#elif defined(WIN32)
	WaitForSingleObject(*sem, INFINITE);
#else
	sem_wait(sem);
#endif
}

void semaphore_create(semaphore_t *sem, int initialCount)
{
#if defined(__APPLE__)
	*sem = dispatch_semaphore_create(initialCount);
#elif defined(WIN32)
	*sem = CreateSemaphore(NULL, initialCount, initialCount, NULL);
#else
	sem_init(sem, 0, initialCount);
#endif
}

void semaphore_destroy(semaphore_t *sem)
{
#if defined(__APPLE__)
	dispatch_release(*sem);
#elif defined(WIN32)
	CloseHandle(*sem);
#else
sem_destroy(sem);
#endif
}
