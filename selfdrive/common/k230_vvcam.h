#pragma once

#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <string>

#include "selfdrive/common/util.h"

namespace k230_vvcam {

constexpr int kDeviceScanLimit = 20;
constexpr int kReadyTimeoutMs = 10000;
constexpr int kReadyPollMs = 100;
constexpr char kIspDaemonName[] = "isp_media_server_debian";
constexpr char kSetupLockPath[] = "/tmp/k230_vvcam_setup.lock";

inline bool is_pid_dir(const char *name) {
  if (name == nullptr || *name == '\0') return false;
  for (const char *p = name; *p != '\0'; ++p) {
    if (!std::isdigit(static_cast<unsigned char>(*p))) return false;
  }
  return true;
}

inline std::string read_proc_cmdline(const char *pid) {
  const std::string path = std::string("/proc/") + pid + "/cmdline";

  int fd = HANDLE_EINTR(open(path.c_str(), O_RDONLY | O_CLOEXEC));
  if (fd < 0) return {};

  char buf[1024];
  const ssize_t n = HANDLE_EINTR(read(fd, buf, sizeof(buf) - 1));
  close(fd);
  if (n <= 0) return {};

  buf[n] = '\0';
  for (ssize_t i = 0; i < n; ++i) {
    if (buf[i] == '\0') buf[i] = ' ';
  }
  return std::string(buf, static_cast<size_t>(n));
}

inline bool isp_daemon_ready() {
  DIR *proc = opendir("/proc");
  if (proc == nullptr) return false;

  bool ready = false;
  while (dirent *ent = readdir(proc)) {
    if (!is_pid_dir(ent->d_name)) continue;

    const std::string cmdline = read_proc_cmdline(ent->d_name);
    if (cmdline.find(kIspDaemonName) == std::string::npos) continue;

    ready = true;
    break;
  }

  closedir(proc);
  return ready;
}

inline int detect_vvcam_video00() {
  for (int i = 0; i < kDeviceScanLimit; ++i) {
    const std::string dev_path = util::string_format("/dev/video%d", i);
    int fd = HANDLE_EINTR(open(dev_path.c_str(), O_RDONLY | O_CLOEXEC));
    if (fd < 0) continue;

    v4l2_capability cap = {};
    const int ret = HANDLE_EINTR(ioctl(fd, VIDIOC_QUERYCAP, &cap));
    close(fd);
    if (ret != 0) continue;

    if (std::strcmp(reinterpret_cast<const char *>(cap.card), "vvcam-video.0.0") == 0) {
      return i;
    }
  }
  return -1;
}

inline bool video_capture_ready(int device) {
  const std::string dev_path = util::string_format("/dev/video%d", device);
  int fd = HANDLE_EINTR(open(dev_path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC));
  if (fd < 0) return false;

  v4l2_format format = {};
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  const bool ready = HANDLE_EINTR(ioctl(fd, VIDIOC_G_FMT, &format)) == 0;
  close(fd);
  return ready;
}

inline bool vvcam_ready(int required_nodes = 1) {
  if (!isp_daemon_ready()) return false;

  const int video00 = detect_vvcam_video00();
  if (video00 < 0) return false;

  for (int i = 0; i < required_nodes; ++i) {
    if (!video_capture_ready(video00 + i)) return false;
  }
  return true;
}

inline bool wait_for_ready(int timeout_ms = kReadyTimeoutMs, int required_nodes = 1) {
  const int attempts = std::max(1, timeout_ms / kReadyPollMs);
  for (int i = 0; i < attempts; ++i) {
    if (vvcam_ready(required_nodes)) {
      return true;
    }
    util::sleep_for(kReadyPollMs);
  }
  return vvcam_ready(required_nodes);
}

class SetupLock {
public:
  SetupLock() {
    fd = HANDLE_EINTR(open(kSetupLockPath, O_CREAT | O_RDWR | O_CLOEXEC, 0666));
    if (fd >= 0 && HANDLE_EINTR(flock(fd, LOCK_EX)) != 0) {
      close(fd);
      fd = -1;
    }
  }

  ~SetupLock() {
    if (fd >= 0) {
      flock(fd, LOCK_UN);
      close(fd);
    }
  }

  bool locked() const { return fd >= 0; }

private:
  int fd = -1;
};

}  // namespace k230_vvcam
