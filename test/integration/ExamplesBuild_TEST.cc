/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gtest/gtest.h>
#include <cstring>
#include <fstream>
#include <string>

#include "test_config.hh"  // NOLINT(build/include)

#ifdef _WIN32
# define GZ_PATH_MAX _MAX_PATH
#elif defined(PATH_MAX)
# define GZ_PATH_MAX PATH_MAX
#elif defined(_XOPEN_PATH_MAX)
# define GZ_PATH_MAX _XOPEN_PATH_MAX
#else
# define GZ_PATH_MAX _POSIX_PATH_MAX
#endif


// Helper functions copied from
// https://github.com/gazebosim/gz-common/raw/ign-common3/src/Filesystem_TEST.cc

#ifndef _WIN32
#include <dirent.h>  // NOLINT(build/include_order)
#include <fcntl.h>  // NOLINT(build/include_order)
#include <limits.h>  // NOLINT(build/include_order)
#include <stdlib.h>  // NOLINT(build/include_order)
#include <sys/stat.h>  // NOLINT(build/include_order)
#include <sys/types.h>  // NOLINT(build/include_order)
#include <unistd.h>  // NOLINT(build/include_order)

/////////////////////////////////////////////////
bool createAndSwitchToTempDir(std::string &_newTempPath)
{
  std::string tmppath;
  const char *tmp = std::getenv("TMPDIR");
  if (tmp)
  {
    tmppath = std::string(tmp);
  }
  else
  {
    tmppath = std::string("/tmp");
  }

  tmppath += "/XXXXXX";

  char *dtemp = mkdtemp(const_cast<char *>(tmppath.c_str()));
  if (dtemp == nullptr)
  {
    return false;
  }
  if (chdir(dtemp) < 0)
  {
    return false;
  }

  // cppcheck-suppress *
  char resolved[PATH_MAX];
  if (realpath(dtemp, resolved) == nullptr)
  {
    return false;
  }

  _newTempPath = std::string(resolved);

  return true;
}

#else
#include <io.h>  // NOLINT(build/include_order)
#include <windows.h>  // NOLINT(build/include_order)
#include <winioctl.h>  // NOLINT(build/include_order)
#include <winnt.h>  // NOLINT(build/include_order)
#include <cstdint>
#include "./win_dirent.h"

/////////////////////////////////////////////////
bool createAndSwitchToTempDir(std::string &_newTempPath)
{
  char tempPath[MAX_PATH + 1];
  DWORD pathLen = ::GetTempPathA(MAX_PATH, tempPath);
  if (pathLen >= MAX_PATH || pathLen <= 0)
  {
    return false;
  }
  std::string pathToCreate(tempPath);
  srand(static_cast<uint32_t>(time(nullptr)));

  for (int count = 0; count < 50; ++count)
  {
    // Try creating a new temporary directory with a randomly generated name.
    // If the one we chose exists, keep trying another path name until we reach
    // some limit.
    std::string newDirName;
    newDirName.append(std::to_string(::GetCurrentProcessId()));
    newDirName.push_back('_');
    // On Windows, rand_r() doesn't exist as an alternative to rand(), so the
    // cpplint warning is spurious.  This program is not multi-threaded, so
    // it is safe to suppress the threadsafe_fn warning here.
    newDirName.append(
       std::to_string(rand()    // NOLINT(runtime/threadsafe_fn)
                      % ((int16_t)0x7fff)));

    pathToCreate += newDirName;
    if (::CreateDirectoryA(pathToCreate.c_str(), nullptr))
    {
      _newTempPath = pathToCreate;
      return ::SetCurrentDirectoryA(_newTempPath.c_str()) != 0;
    }
  }

  return false;
}

struct handle_wrapper
{
  HANDLE handle;
  explicit handle_wrapper(HANDLE h)
    : handle(h) {}
  ~handle_wrapper()
  {
    if (handle != INVALID_HANDLE_VALUE)
    {
      ::CloseHandle(handle);
    }
  }
};

//////////////////////////////////////////////////
HANDLE create_file_handle(const std::string &_path, DWORD _dwDesiredAccess,
    DWORD _dwShareMode,
    LPSECURITY_ATTRIBUTES _lpSecurityAttributes,
    DWORD _dwCreationDisposition,
    DWORD _dwFlagsAndAttributes,
    HANDLE _hTemplateFile)
{
  return ::CreateFileA(_path.c_str(), _dwDesiredAccess,
      _dwShareMode, _lpSecurityAttributes,
      _dwCreationDisposition, _dwFlagsAndAttributes,
      _hTemplateFile);
}

//////////////////////////////////////////////////
bool not_found_error(int _errval)
{
  return _errval == ERROR_FILE_NOT_FOUND
    || _errval == ERROR_PATH_NOT_FOUND
    || _errval == ERROR_INVALID_NAME  // "tools/src/:sys:stat.h", "//foo"
    || _errval == ERROR_INVALID_DRIVE  // USB card reader with no card
    || _errval == ERROR_NOT_READY  // CD/DVD drive with no disc inserted
    || _errval == ERROR_INVALID_PARAMETER  // ":sys:stat.h"
    || _errval == ERROR_BAD_PATHNAME  // "//nosuch" on Win64
    || _errval == ERROR_BAD_NETPATH;  // "//nosuch" on Win32
}

#ifndef MAXIMUM_REPARSE_DATA_BUFFER_SIZE
#define MAXIMUM_REPARSE_DATA_BUFFER_SIZE  (16 * 1024)
#endif

typedef struct _REPARSE_DATA_BUFFER {
  ULONG  ReparseTag;
  USHORT  ReparseDataLength;
  USHORT  Reserved;
  union {
    struct {
      USHORT  SubstituteNameOffset;
      USHORT  SubstituteNameLength;
      USHORT  PrintNameOffset;
      USHORT  PrintNameLength;
      ULONG  Flags;
      WCHAR  PathBuffer[1];
      /*  Example of distinction between substitute and print names:
          mklink /d ldrive c:\
SubstituteName: c:\\??\
PrintName: c:\
*/
    } SymbolicLinkReparseBuffer;
    struct {
      USHORT  SubstituteNameOffset;
      USHORT  SubstituteNameLength;
      USHORT  PrintNameOffset;
      USHORT  PrintNameLength;
      WCHAR  PathBuffer[1];
    } MountPointReparseBuffer;
    struct {
      UCHAR  DataBuffer[1];
    } GenericReparseBuffer;
  };
} REPARSE_DATA_BUFFER, *PREPARSE_DATA_BUFFER;

//////////////////////////////////////////////////
bool is_reparse_point_a_symlink(const std::string &_path)
{
  handle_wrapper h(create_file_handle(_path, FILE_READ_EA,
        FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
        nullptr, OPEN_EXISTING,
        FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OPEN_REPARSE_POINT,
        nullptr));
  if (h.handle == INVALID_HANDLE_VALUE)
  {
    return false;
  }

  std::vector<wchar_t> buf(MAXIMUM_REPARSE_DATA_BUFFER_SIZE);

  // Query the reparse data
  DWORD dwRetLen;
  BOOL result = ::DeviceIoControl(h.handle, FSCTL_GET_REPARSE_POINT,
      nullptr, 0, buf.data(), (DWORD)buf.size(), &dwRetLen, nullptr);
  if (!result)
  {
    return false;
  }

  return reinterpret_cast<const REPARSE_DATA_BUFFER*>(&buf[0])->ReparseTag
    == IO_REPARSE_TAG_SYMLINK
    // Issue 9016 asked that NTFS directory junctions be recognized as
    // directories.  That is equivalent to recognizing them as symlinks, and
    // then the normal symlink mechanism will recognize them as directories.
    //
    // Directory junctions are very similar to symlinks, but have some
    // performance and other advantages over symlinks. They can be created
    // from the command line with "mklink /j junction-name target-path".
    || reinterpret_cast<const REPARSE_DATA_BUFFER*>(&buf[0])->ReparseTag
    == IO_REPARSE_TAG_MOUNT_POINT;  // "directory junction" or "junction"
}

//////////////////////////////////////////////////
bool process_status_failure()
{
  int errval(::GetLastError());

  if (not_found_error(errval))
  {
    return false;
  }
  else if ((errval == ERROR_SHARING_VIOLATION))
  {
    return true;  // odd, but this is what boost does
  }
  return false;
}

//////////////////////////////////////////////////
bool internal_check_path(const std::string &_path, DWORD &attr)
{
  attr = ::GetFileAttributesA(_path.c_str());
  if (attr == 0xFFFFFFFF)
  {
    return process_status_failure();
  }

  // reparse point handling;
  //   since GetFileAttributesW does not resolve symlinks, try to open file
  //   handle to discover if the file exists
  if (attr & FILE_ATTRIBUTE_REPARSE_POINT)
  {
    handle_wrapper h(
        create_file_handle(
          _path,
          0,  // dwDesiredAccess; attributes only
          FILE_SHARE_DELETE | FILE_SHARE_READ | FILE_SHARE_WRITE,
          0,  // lpSecurityAttributes
          OPEN_EXISTING,
          FILE_FLAG_BACKUP_SEMANTICS,
          0));  // hTemplateFile
    if (h.handle == INVALID_HANDLE_VALUE)
    {
      return process_status_failure();
    }

    if (!is_reparse_point_a_symlink(_path))
    {
      return true;
    }
  }

  return true;
}
#endif

//////////////////////////////////////////////////
bool isDirectory(const std::string &_path)
{
#ifndef _WIN32
  struct stat path_stat;

  if (::stat(_path.c_str(), &path_stat) != 0)
  {
    return false;
  }

  // cppcheck-suppress ConfigurationNotChecked
  return S_ISDIR(path_stat.st_mode);
#else
  DWORD attr;

  if (internal_check_path(_path, attr))
  {
    return (attr & FILE_ATTRIBUTE_DIRECTORY) != 0;
  }

  return false;
#endif
}

//////////////////////////////////////////////////
bool isFile(const std::string &_path)
{
  std::ifstream f(_path);
  return (!isDirectory(_path)) && f.good();
}

//////////////////////////////////////////////////
void removeAll(const std::string &_path)
{
  if (isDirectory(_path))
  {
    DIR *dir = opendir(_path.c_str());
    if (dir)
    {
      struct dirent *p;
      while ((p=readdir(dir)))
      {
        // Skip special files.
        if (!std::strcmp(p->d_name, ".") || !std::strcmp(p->d_name, ".."))
          continue;

        removeAll(_path + "/" + p->d_name);
      }
    }
    closedir(dir);

    // Remove the directory
    bool removed;
#ifdef _WIN32
    removed = RemoveDirectory(_path.c_str());
#else
    removed = (rmdir(_path.c_str()) == 0);
    if (!removed)
    {
      // A sym link would end up here
      removed = (std::remove(_path.c_str()) == 0);
    }
#endif
    ASSERT_TRUE(removed);
  }
  else if (isFile(_path))
  {
    const bool removed = (std::remove(_path.c_str()) == 0);
    ASSERT_TRUE(removed);
  }
}

//////////////////////////////////////////////////
TEST(ExamplesBuild, Build)
{
  // \todo(nkoenig) Fix windows.
#ifndef _WIN32
  // Path to examples of the given type
  std::string examplesDir = std::string(PROJECT_SOURCE_PATH) + "/examples/";

  // Create a temp build directory
  std::string tmpBuildDir;
  ASSERT_TRUE(createAndSwitchToTempDir(tmpBuildDir));
  std::cout << "Build directory: " << tmpBuildDir<< std::endl;

  char cmd[1024];

  // cd build && cmake source
  snprintf(cmd, sizeof(cmd), "cd %s && cmake %s && make",
    tmpBuildDir.c_str(), examplesDir.c_str());
  ASSERT_EQ(system(cmd), 0);

  // Remove temp dir
  removeAll(tmpBuildDir);
#endif
}
