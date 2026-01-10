// Copyright 2014 The Chromium Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "services/device/hid/hid_service_netbsd.h"

#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <usbhid.h>
#include <dev/usb/usb.h>
#include <dev/usb/usbhid.h>
#include <dlfcn.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "base/files/file.h"
#include "base/files/file_path.h"
#include "base/files/file_util.h"
#include "base/files/scoped_file.h"
#include "base/location.h"
#include "base/sequence_checker.h"
#include "base/strings/string_number_conversions.h"
#include "base/strings/string_split.h"
#include "base/strings/string_util.h"
#include "base/task/sequenced_task_runner.h"
#include "base/task/thread_pool.h"
#include "base/threading/scoped_blocking_call.h"
#include "build/build_config.h"
#include "build/chromeos_buildflags.h"
#include "components/device_event_log/device_event_log.h"
#include "device/udev_linux/scoped_udev.h"
#include "device/udev_linux/udev_watcher.h"
#include "services/device/hid/hid_connection_netbsd.h"

namespace device {

struct HidServiceNetBSD::ConnectParams {
  ConnectParams(scoped_refptr<HidDeviceInfo> device_info,
                bool allow_protected_reports,
                bool allow_fido_reports,
                ConnectCallback callback)
      : device_info(std::move(device_info)),
        allow_protected_reports(allow_protected_reports),
        allow_fido_reports(allow_fido_reports),
        callback(std::move(callback)),
	      task_runner(base::SequencedTaskRunner::GetCurrentDefault()),
      blocking_task_runner(
        base::ThreadPool::CreateSequencedTaskRunner(kBlockingTaskTraits)) {}
      ~ConnectParams() {}

  scoped_refptr<HidDeviceInfo> device_info;
  bool allow_protected_reports;
  bool allow_fido_reports;
  ConnectCallback callback;
  scoped_refptr<base::SequencedTaskRunner> task_runner;
  scoped_refptr<base::SequencedTaskRunner> blocking_task_runner;
  base::ScopedFD fd;
};

class HidServiceNetBSD::BlockingTaskRunnerHelper : public UdevWatcher::Observer {
 public:
  BlockingTaskRunnerHelper(base::WeakPtr<HidServiceNetBSD> service)
      : service_(std::move(service)),
        task_runner_(base::SequencedTaskRunner::GetCurrentDefault()) {
    DETACH_FROM_SEQUENCE(sequence_checker_);
  }

  BlockingTaskRunnerHelper(const BlockingTaskRunnerHelper&) = delete;
  BlockingTaskRunnerHelper& operator=(const BlockingTaskRunnerHelper&) = delete;

  ~BlockingTaskRunnerHelper() override {
    DCHECK_CALLED_ON_VALID_SEQUENCE(sequence_checker_);
  }

  void Start() {
    DCHECK_CALLED_ON_VALID_SEQUENCE(sequence_checker_);

    void *library = dlopen("libudev.so", RTLD_NOW | RTLD_LOCAL);
    if (library) {
      dlclose(library);
      watcher_ = UdevWatcher::StartWatching(this);
      watcher_->EnumerateExistingDevices();
    } else {
      HID_LOG(ERROR) << "No udev available. HID device detecting isn't work.";
    }

    task_runner_->PostTask(FROM_HERE,
                           base::BindOnce(&HidServiceNetBSD::FirstEnumerationComplete, service_));
  }

  // UdevWatcher::Observer
  void OnDeviceAdded(ScopedUdevDevicePtr device) override {
    DCHECK_CALLED_ON_VALID_SEQUENCE(sequence_checker_);
    base::ScopedBlockingCall scoped_blocking_call(FROM_HERE, base::BlockingType::MAY_BLOCK);
    auto null_as_empty = [](const char *r) -> std::string {
      return (r != nullptr) ? r : "";
    };
    std::vector<uint8_t> report_descriptor;
    base::File device_file;
    int flags, result;
    base::ScopedFD fd;
    struct usb_ctl_report_desc rdesc;
    struct usb_device_info dinfo;

    const char* subsystem = udev_device_get_subsystem(device.get());
    if (!subsystem || strcmp(subsystem, "fido") != 0)
      return;

    std::string device_path_ = udev_device_get_syspath(device.get());
    if (device_path_.empty())
      return;
    base::FilePath device_path(device_path_);

    flags = base::File::FLAG_OPEN | base::File::FLAG_READ | base::File::FLAG_WRITE;
    device_file.Initialize(device_path, flags);
    if (!device_file.IsValid()) {
      HID_LOG(ERROR) << "Failed to open '" << device_path << "': "
                     << base::File::ErrorToString(device_file.error_details());
      return;
    }

    fd.reset(device_file.TakePlatformFile());

    result = HANDLE_EINTR(ioctl(fd.get(), USB_GET_REPORT_DESC, &rdesc));
    if (result < 0) {
      HID_LOG(ERROR) << "Failed to get reportdesc: " << device_path;
      return;
    }
    report_descriptor.assign(rdesc.ucrd_data, rdesc.ucrd_data + rdesc.ucrd_size);

    result = HANDLE_EINTR(ioctl(fd.get(), USB_GET_DEVICEINFO, &dinfo));
    if (result < 0) {
      HID_LOG(ERROR) << "Failed to get deviceinfo: " << device_path;
      return;
    }

    scoped_refptr<HidDeviceInfo> device_info(new HidDeviceInfo(
      device_path.value(),
      /*physical_device_id*/"",
	    dinfo.udi_vendorNo,
	    dinfo.udi_productNo,
	    null_as_empty(dinfo.udi_product),
	    null_as_empty(dinfo.udi_serial),
      device::mojom::HidBusType::kHIDBusTypeUSB,
      base::span<const uint8_t>(report_descriptor),
	    device_path.value()));

    task_runner_->PostTask(FROM_HERE,
                           base::BindOnce(&HidServiceNetBSD::AddDevice, service_, device_info));
  }

  void OnDeviceRemoved(ScopedUdevDevicePtr device) override {
    DCHECK_CALLED_ON_VALID_SEQUENCE(sequence_checker_);
    base::ScopedBlockingCall scoped_blocking_call(FROM_HERE, base::BlockingType::MAY_BLOCK);

    const char* device_path = udev_device_get_syspath(device.get());
    if (device_path) {
      task_runner_->PostTask(FROM_HERE,
                             base::BindOnce(&HidServiceNetBSD::RemoveDevice,
                              service_,
                              std::string(device_path)));
    }
  }

  void OnDeviceChanged(ScopedUdevDevicePtr) override {}

  SEQUENCE_CHECKER(sequence_checker_);
  std::unique_ptr<UdevWatcher> watcher_;

  // This weak pointer is only valid when checked on this task runner.
  base::WeakPtr<HidServiceNetBSD> service_;
  scoped_refptr<base::SequencedTaskRunner> task_runner_;
};

HidServiceNetBSD::HidServiceNetBSD()
    : blocking_task_runner_(base::ThreadPool::CreateSequencedTaskRunner(kBlockingTaskTraits)),
      helper_(nullptr, base::OnTaskRunnerDeleter(blocking_task_runner_)) {
  // We need to properly initialize |blocking_task_helper_| here because we need
  // |weak_factory_| to be created first.
  helper_.reset(new BlockingTaskRunnerHelper(weak_factory_.GetWeakPtr()));
  blocking_task_runner_->PostTask(FROM_HERE,
                                  base::BindOnce(&BlockingTaskRunnerHelper::Start,
                                    base::Unretained(helper_.get())));
}

HidServiceNetBSD::~HidServiceNetBSD() = default;

base::WeakPtr<HidService> HidServiceNetBSD::GetWeakPtr() {
  return weak_factory_.GetWeakPtr();
}

void HidServiceNetBSD::Connect(const std::string& device_guid,
                              bool allow_protected_reports,
                              bool allow_fido_reports,
                              ConnectCallback callback) {
  DCHECK_CALLED_ON_VALID_SEQUENCE(sequence_checker_);

  const auto& map_entry = devices().find(device_guid);
  if (map_entry == devices().end()) {
    base::SequencedTaskRunner::GetCurrentDefault()->PostTask(FROM_HERE,
                            base::BindOnce(std::move(callback), nullptr));
    return;
  }
  scoped_refptr<HidDeviceInfo> device_info = map_entry->second;

  auto params = std::make_unique<ConnectParams>(device_info,
                                                allow_protected_reports,
                                                allow_fido_reports,
                                                std::move(callback));
  scoped_refptr<base::SequencedTaskRunner> blocking_task_runner = params->blocking_task_runner;
  blocking_task_runner->PostTask(FROM_HERE,
                                 base::BindOnce(&HidServiceNetBSD::OpenOnBlockingThread,
                                    std::move(params)));
}

// static
void HidServiceNetBSD::OpenOnBlockingThread(
    std::unique_ptr<ConnectParams> params) {
  base::ScopedBlockingCall scoped_blocking_call(FROM_HERE, base::BlockingType::MAY_BLOCK);
  scoped_refptr<base::SequencedTaskRunner> task_runner = params->task_runner;

  base::FilePath device_path(params->device_info->device_node());
  base::File device_file;
  int flags = base::File::FLAG_OPEN | base::File::FLAG_READ | base::File::FLAG_WRITE;
  device_file.Initialize(device_path, flags);
  if (!device_file.IsValid()) {
    base::File::Error file_error = device_file.error_details();

    if (file_error == base::File::FILE_ERROR_ACCESS_DENIED) {
      HID_LOG(EVENT) << "Access denied opening device read-write, trying read-only.";
      flags = base::File::FLAG_OPEN | base::File::FLAG_READ;
      device_file.Initialize(device_path, flags);
    }
  }
  if (!device_file.IsValid()) {
    HID_LOG(EVENT) << "Failed to open '" << params->device_info->device_node() << "': "
                   << base::File::ErrorToString(device_file.error_details());
    task_runner->PostTask(FROM_HERE,
                          base::BindOnce(std::move(params->callback), nullptr));
    return;
  }
  params->fd.reset(device_file.TakePlatformFile());

  task_runner->PostTask(FROM_HERE,
                        base::BindOnce(&HidServiceNetBSD::FinishOpen,
                                       std::move(params)));
}

// static
void HidServiceNetBSD::FinishOpen(std::unique_ptr<ConnectParams> params) {
  DCHECK(params->fd.is_valid());

  if (!base::SetNonBlocking(params->fd.get())) {
    HID_PLOG(DEBUG) << "Failed to set the non-blocking flag on the device fd";
    std::move(params->callback).Run(nullptr);
    return;
  }

  std::move(params->callback)
      .Run(base::MakeRefCounted<HidConnectionNetBSD>(
          std::move(params->device_info),
          std::move(params->fd),
          std::move(params->blocking_task_runner),
          params->allow_protected_reports,
          params->allow_fido_reports));
}

}  // namespace device
