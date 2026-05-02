// Copyright 2011 The Chromium Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/system/sys_info.h"

#include <stddef.h>
#include <stdint.h>
#include <sys/param.h>
#include <sys/shm.h>
#include <sys/sysctl.h>
#include <uvm/uvm_extern.h>

#include "base/notreached.h"
#include "base/posix/sysctl.h"
#include "base/numerics/safe_conversions.h"
#include "base/strings/string_util.h"

namespace base {

namespace {

ByteSize AmountOfMemory(int pages_name) {
  long pages = sysconf(pages_name);
  long page_size = sysconf(_SC_PAGESIZE);
  if (pages <= 0 || page_size <= 0) {
    return ByteSize(0);
  }
  return ByteSize(checked_cast<uint64_t>(pages)) *
         checked_cast<uint64_t>(page_size);
}

}  // namespace

// static
int SysInfo::NumberOfProcessors() {
  int mib[] = {CTL_HW, HW_NCPU};
  int ncpu;
  size_t size = sizeof(ncpu);
  if (sysctl(mib, std::size(mib), &ncpu, &size, NULL, 0) < 0) {
    NOTREACHED();
    return 1; 
  }
  return ncpu > 0 ? ncpu : 1;
}

// static
std::string SysInfo::CPUModelName() {
  int mib[] = { CTL_HW, HW_MODEL };
  char name[256];
  size_t size = std::size(name);

  if (sysctl(mib, std::size(mib), &name, &size, NULL, 0) == 0) {
    return name;
  }

  return std::string();
}

// static
ByteSize SysInfo::AmountOfTotalPhysicalMemoryImpl() {
  return AmountOfMemory(_SC_PHYS_PAGES);
}

// static
ByteSize SysInfo::AmountOfAvailablePhysicalMemoryImpl() {
#if defined(_SC_AVPHYS_PAGES)
  return AmountOfMemory(_SC_AVPHYS_PAGES);
#else
  struct uvmexp_sysctl uvmexp;
  size_t len = sizeof(uvmexp);
  int mib[] = { CTL_VM, VM_UVMEXP2 };
  if (sysctl(mib, std::size(mib), &uvmexp, &len, NULL, 0) <0) {
    NOTREACHED();
    return ByteSize(0);
  }
  return ByteSize(checked_cast<uint64_t>(uvmexp.free)) *
       checked_cast<uint64_t>(uvmexp.pagesize);
#endif
}

// static
uint64_t SysInfo::MaxSharedMemorySize() {
  int mib[] = {CTL_KERN, KERN_SYSVIPC, KERN_SYSVIPC_SHMMAX};
  uint64_t limit;
  size_t size = sizeof(limit);
  if (sysctl(mib, std::size(mib), &limit, &size, NULL, 0) < 0) {
    NOTREACHED();
    return 0; 
  }
  return limit;
}

// static
SysInfo::HardwareInfo SysInfo::GetHardwareInfoSync() {
  HardwareInfo info;
  // Set the manufacturer to "NetBSD" and the model to
  // an empty string.
  info.manufacturer = "NetBSD";
  info.model = HardwareModelName();
  DCHECK(IsStringUTF8(info.manufacturer));
  DCHECK(IsStringUTF8(info.model));
  return info;
}

}  // namespace base
