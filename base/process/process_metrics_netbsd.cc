// Copyright 2013 The Chromium Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/process/process_metrics.h"

#include <stddef.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/param.h>
#include <sys/sysctl.h>
#include <sys/vmmeter.h>

#include "base/files/dir_reader_posix.h" // DirReaderPosix
#include "base/process/internal_linux.h" // GetProcPidDir()
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

  const int64_t used_bytes =
      used_pages * uvm.pagesize;

  return static_cast<size_t>(used_bytes);
}

int ProcessMetrics::GetOpenFdCount() const {
  // Use /proc/<pid>/fd to count the number of entries there.
  FilePath fd_path = internal::GetProcPidDir(process_).Append("fd");

  DirReaderPosix dir_reader(fd_path.value().c_str());
  if (!dir_reader.IsValid()) {
    return -1;
  }

  int total_count = 0;
  for (; dir_reader.Next();) {
    const char* name = dir_reader.name();
    if (strcmp(name, ".") != 0 && strcmp(name, "..") != 0) {
      ++total_count;
    }
  }

  return total_count;
}

int ProcessMetrics::GetOpenFdSoftLimit() const {
  return getdtablesize();
//  return GetMaxFds();
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
