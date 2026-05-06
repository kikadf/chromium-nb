// Copyright 2011 The Chromium Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/process/process_handle.h"
#include "base/files/file_util.h"

#include <stddef.h>
#include <sys/param.h>
#include <sys/proc.h>
#include <sys/sysctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <optional>

#include "base/files/file_path.h"
#include "base/posix/sysctl.h"

namespace base {

ProcessId GetParentProcessId(ProcessHandle process) {
  struct kinfo_proc2 info;
  size_t length = sizeof(struct kinfo_proc2);
  int mib[] = { CTL_KERN, KERN_PROC2, KERN_PROC_PID, process,
                sizeof(struct kinfo_proc2), 1 };

  if (sysctl(mib, std::size(mib), &info, &length, NULL, 0) < 0) {
    return -1;
  }

  if (length != sizeof(info)) {
    return -1;
  }

  return info.p_ppid;
}

FilePath GetProcessExecutablePath(ProcessHandle process) {
  std::optional<std::string> pathname =
      base::StringSysctl({CTL_KERN, KERN_PROC_ARGS, process, KERN_PROC_PATHNAME});

  return FilePath(pathname.value_or(std::string{}));
}

}  // namespace base
