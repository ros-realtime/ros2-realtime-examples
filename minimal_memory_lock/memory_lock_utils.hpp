// Copyright 2022 Carlos San Vicente
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MINIMAL_MEMORY_LOCK__MEMORY_LOCK_UTILS_HPP_
#define MINIMAL_MEMORY_LOCK__MEMORY_LOCK_UTILS_HPP_

#include <sys/types.h>
#include <sys/mman.h>

#include <malloc.h>

#include <string>

void lock_memory()
{
  // Lock all current and future pages
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    throw std::runtime_error("mlockall failed. Error code " + std::string(strerror(errno)));
  }

  // Turn off malloc trimming.
  if (mallopt(M_TRIM_THRESHOLD, -1) == 0) {
    throw std::runtime_error(
            "mallopt for trim threshold failed. Error code " +
            std::string(strerror(errno)));
  }

  // Turn off mmap usage.
  if (mallopt(M_MMAP_MAX, 0) == 0) {
    mallopt(M_TRIM_THRESHOLD, 128 * 1024);
    munlockall();
    throw std::runtime_error(
            "mallopt for mmap failed. Error code " + std::string(
              strerror(errno)));
  }
}

void preallocate_memory(size_t memory_size)
{
  void * buf = nullptr;
  const size_t pg_sz = sysconf(_SC_PAGESIZE);
  int res;
  res = posix_memalign(&buf, static_cast<size_t>(pg_sz), memory_size);
  if (res != 0) {
    throw std::runtime_error(
            "Failed to reserve memory. Error code " +
            std::string(strerror(errno)));
  }
  free(buf);
}

void set_default_thread_stacksize(size_t stacksize)
{
  pthread_attr_t attr;
  int ret;
  ret = pthread_getattr_default_np(&attr);
  if (ret != 0) {
    std::cerr << "Failed to call pthread_getattr_default_np. Error code: " <<
      strerror(errno) << std::endl;
    return;
  }

  ret = pthread_attr_setstacksize(&attr, stacksize);
  if (ret != 0) {
    std::cerr << "Failed to call pthread_attr_setstacksize. Error code: " <<
      strerror(errno) << std::endl;
    return;
  }
  ret = pthread_setattr_default_np(&attr);
  if (ret != 0) {
    std::cerr << "Failed to call pthread_setattr_default_np. Error code: " <<
      strerror(errno) << std::endl;
    return;
  }
}

#endif  // MINIMAL_MEMORY_LOCK__MEMORY_LOCK_UTILS_HPP_
