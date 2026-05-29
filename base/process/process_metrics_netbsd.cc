// Copyright 2013 The Chromium Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/process/process_metrics.h"

#include <stddef.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/resource.h>
#include <sys/param.h>
#include <sys/sysctl.h>
#include <sys/vmmeter.h>
#include <uvm/uvm_extern.h> // struct uvmexp_sysctl

#include "base/memory/ptr_util.h"
#include "base/types/expected.h"
#include "base/values.h"
#include "base/notimplemented.h"

namespace base {

ProcessMetrics::ProcessMetrics(ProcessHandle process) : process_(process) {}

base::expected<ProcessMemoryInfo, ProcessUsageError>
ProcessMetrics::GetMemoryInfo() const {
  ProcessMemoryInfo memory_info;
  struct kinfo_proc2 info;
  size_t length = sizeof(struct kinfo_proc2);

  int mib[] = { CTL_KERN, KERN_PROC2, KERN_PROC_PID, process_,
                sizeof(struct kinfo_proc2), 1 };

  if (process_ == 0) {
    return base::unexpected(ProcessUsageError::kSystemError);
  }

  if (sysctl(mib, std::size(mib), &info, &length, NULL, 0) < 0) {
    return base::unexpected(ProcessUsageError::kSystemError);
  }

  if (length == 0) {
    return base::unexpected(ProcessUsageError::kProcessNotFound);
  }

  memory_info.resident_set_bytes =
    checked_cast<uint64_t>(info.p_vm_rssize * getpagesize());

  return memory_info;
}

base::expected<TimeDelta, ProcessCPUUsageError>
ProcessMetrics::GetCumulativeCPUUsage() {
  struct kinfo_proc2 info;
  size_t length = sizeof(struct kinfo_proc2);
  struct timeval tv;

  int mib[] = { CTL_KERN, KERN_PROC2, KERN_PROC_PID, process_,
                sizeof(struct kinfo_proc2), 1 };

  if (process_ == 0) {
    return base::unexpected(ProcessCPUUsageError::kSystemError);
  }

  if (sysctl(mib, std::size(mib), &info, &length, NULL, 0) < 0) {
    return base::unexpected(ProcessCPUUsageError::kSystemError);
  }

  if (length == 0) {
    return base::unexpected(ProcessCPUUsageError::kProcessNotFound);
  }

  tv.tv_sec = info.p_rtime_sec;
  tv.tv_usec = static_cast<suseconds_t>(info.p_rtime_usec);

  return base::ok(Microseconds(TimeValToMicroseconds(tv)));
}

// static
std::unique_ptr<ProcessMetrics> ProcessMetrics::CreateProcessMetrics(
    ProcessHandle process) {
  return WrapUnique(new ProcessMetrics(process));
}

size_t GetSystemCommitCharge() {
  int mib[] = { CTL_VM, VM_UVMEXP2 };
  struct uvmexp_sysctl uvm;
  size_t len = sizeof(uvm);

  if (sysctl(mib, std::size(mib), &uvm, &len, NULL, 0) < 0) {
    return 0;
  }

  const int64_t used_pages =
      std::max<int64_t>(0, uvm.npages - uvm.free - uvm.inactive);

  const int64_t used_kbytes =
      used_pages * uvm.pagesize / 1024;

  return static_cast<size_t>(used_kbytes);
}

int ProcessMetrics::GetOpenFdSoftLimit() const {
  struct rlimit rl;

  if (getrlimit(RLIMIT_NOFILE, &rl) != 0) {
    return -1;
  }

  return (rl.rlim_cur > INT_MAX) ? INT_MAX : (int)rl.rlim_cur;
}

int ProcessMetrics::GetOpenFdCount() const {
  int count = 0;
  int max_fd = GetOpenFdSoftLimit();
  if (max_fd == -1) {
    return -1;
  } else if (max_fd > 10000) {
    max_fd = 10000;
  }

  for (int i = 0; i < max_fd; ++i) {
    if (fcntl(i, F_GETFD) >= 0 || errno != EBADF) {
      count++;
    }
  }

  return count;
}

bool ProcessMetrics::GetPageFaultCounts(PageFaultCounts* counts) const {
  NOTIMPLEMENTED();
  return false;
}

bool GetSystemMemoryInfo(SystemMemoryInfo* meminfo) {
  NOTIMPLEMENTED();
  return false;
}

bool GetSystemDiskInfo(SystemDiskInfo* diskinfo) {
  NOTIMPLEMENTED();
  return false;
}

bool GetVmStatInfo(VmStatInfo* vmstat) {
  NOTIMPLEMENTED();
  return false;
}

int ProcessMetrics::GetIdleWakeupsPerSecond() {
  NOTIMPLEMENTED();
  return 0;
}

SystemDiskInfo::SystemDiskInfo() {
  reads = 0;
  reads_merged = 0;
  sectors_read = 0;
  read_time = 0;
  writes = 0;
  writes_merged = 0;
  sectors_written = 0;
  write_time = 0;
  io = 0;
  io_time = 0;
  weighted_io_time = 0;
}

SystemDiskInfo::SystemDiskInfo(const SystemDiskInfo&) = default;

SystemDiskInfo& SystemDiskInfo::operator=(const SystemDiskInfo&) = default;

}  // namespace base
