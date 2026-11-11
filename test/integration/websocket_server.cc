/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include <libwebsockets.h>

#include <gz/common/Filesystem.hh>
#include <gz/common/TempDirectory.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/bytes.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <gz/transport/Node.hh>

#include "WebsocketServer.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;

// The fixture is in the global namespace because that is where
// WebsocketServer.hh forward-declares and befriends it.
class WebsocketServerTest : public InternalFixture<::testing::Test>
{
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    this->sandbox = std::make_unique<common::TempDirectory>(
        "gz-sim-ws-", "test", true);
    ASSERT_TRUE(this->sandbox->Valid());

    this->allowedDir =
        common::joinPaths(this->sandbox->Path(), "resources");
    this->fuelCacheDir =
        common::joinPaths(this->sandbox->Path(), "fuel");
    this->outsideDir =
        common::joinPaths(this->sandbox->Path(), "outside");

    ASSERT_TRUE(common::createDirectories(this->allowedDir));
    ASSERT_TRUE(common::createDirectories(this->fuelCacheDir));
    ASSERT_TRUE(common::createDirectories(this->outsideDir));

    ASSERT_TRUE(this->WriteFile(
        common::joinPaths(this->allowedDir, "ok.txt"), "hello"));
    ASSERT_TRUE(this->WriteFile(
        common::joinPaths(this->fuelCacheDir, "cached.txt"), "fuel-payload"));
    this->secretPath = common::joinPaths(this->outsideDir, "secret.txt");
    ASSERT_TRUE(this->WriteFile(this->secretPath, "must-not-leak"));

    // Symlink inside the sandbox-allowed dir that escapes to the outside
    // file. weakly_canonical resolves the link, so the allowlist check
    // must catch this as denied.
    this->escapeSymlink =
        common::joinPaths(this->allowedDir, "escape");
    std::error_code ec;
    std::filesystem::create_symlink(this->secretPath, this->escapeSymlink, ec);
    if (ec)
    {
      if (ec == std::errc::function_not_supported ||
          ec == std::errc::operation_not_supported)
      {
        this->symlinkSupported = false;
      }
      else
      {
        // Any other error is a real setup failure — fail loudly so it is
        // not silently masked as an unsupported-filesystem skip.
        GTEST_FAIL() << "create_symlink failed unexpectedly: "
                     << ec.message()
                     << " (source=" << this->secretPath
                     << ", dest=" << this->escapeSymlink << ")";
      }
    }
    else
    {
      this->symlinkSupported = true;
    }

    // Point the production code at our sandboxed allowlists.
    ASSERT_TRUE(common::setenv("GZ_SIM_RESOURCE_PATH",
        this->allowedDir.c_str()));
    ASSERT_TRUE(common::setenv("GZ_FUEL_CACHE_PATH",
        this->fuelCacheDir.c_str()));
  }

  protected: void TearDown() override
  {
    EXPECT_TRUE(common::unsetenv("GZ_SIM_RESOURCE_PATH"))
        << "Failed to unset GZ_SIM_RESOURCE_PATH; subsequent tests may "
           "inherit a stale allowlist";
    EXPECT_TRUE(common::unsetenv("GZ_FUEL_CACHE_PATH"))
        << "Failed to unset GZ_FUEL_CACHE_PATH; subsequent tests may "
           "inherit a stale allowlist";
    this->sandbox.reset();
    InternalFixture::TearDown();
  }

  // Inject a pre-authorized fake Connection into the server's connection
  // map so that OnAsset's QueueMessage call has a valid target. Befriended
  // access lets us see the private nested Connection type.
  protected: void InjectAuthorizedConnection(
      sim::systems::WebsocketServer &_server, int _socketId)
  {
    auto c = std::make_unique<sim::systems::WebsocketServer::Connection>();
    c->creationTime = GZ_SYSTEM_TIME();
    c->authorized = true;
    _server.connections[_socketId] = std::move(c);
  }

  // Mark a connection unauthorized and set the server's user key. Befriended
  // access lets us reach the private members.
  protected: void RequireAuthorization(
      sim::systems::WebsocketServer &_server, int _socketId,
      const std::string &_key)
  {
    _server.connections[_socketId]->authorized = false;
    _server.authorizationKey = _key;
  }

  protected: bool IsAuthorized(
      const sim::systems::WebsocketServer &_server, int _socketId)
  {
    return _server.connections.at(_socketId)->authorized;
  }

  // Pop the first queued outbound frame from the given connection. The
  // production code prefixes each buffer with LWS_PRE padding bytes (lws
  // requires this for in-place serialization); we strip those off before
  // returning.
  protected: std::string PopFrame(
      sim::systems::WebsocketServer &_server, int _socketId)
  {
    auto it = _server.connections.find(_socketId);
    if (it == _server.connections.end() || !it->second)
      return "";

    auto &conn = *it->second;
    std::lock_guard<std::mutex> lock(conn.mutex);
    if (conn.buffer.empty() || conn.len.empty())
      return "";

    auto buf = std::move(conn.buffer.front());
    int len = conn.len.front();
    conn.buffer.pop_front();
    conn.len.pop_front();
    return std::string(buf.get() + LWS_PRE, len);
  }

  // Split a BUILD_MSG frame into "op,topic,type,payload". Because the
  // payload is serialized protobuf and may contain commas/NULs, we only
  // split on the first three commas.
  protected: struct Frame
  {
    std::string op;
    std::string topic;
    std::string type;
    std::string payload;
  };

  protected: static Frame ParseFrame(const std::string &_raw)
  {
    Frame f;
    std::size_t p1 = _raw.find(',');
    std::size_t p2 = (p1 == std::string::npos) ?
        std::string::npos : _raw.find(',', p1 + 1);
    std::size_t p3 = (p2 == std::string::npos) ?
        std::string::npos : _raw.find(',', p2 + 1);
    if (p1 == std::string::npos || p2 == std::string::npos ||
        p3 == std::string::npos)
    {
      return f;
    }
    f.op = _raw.substr(0, p1);
    f.topic = _raw.substr(p1 + 1, p2 - p1 - 1);
    f.type = _raw.substr(p2 + 1, p3 - p2 - 1);
    f.payload = _raw.substr(p3 + 1);
    return f;
  }

  // Invoke OnAsset with a constructed frame. No Configure() call is
  // needed: the operations vector is populated by the default member
  // initializer and we inject the connection by hand.
  protected: Frame CallOnAsset(
      sim::systems::WebsocketServer &_server, int _socketId,
      const std::vector<std::string> &_frameParts)
  {
    _server.OnAsset(_socketId, _frameParts);
    return ParseFrame(this->PopFrame(_server, _socketId));
  }

  // Returns AssertionFailure so callers can use ASSERT_TRUE(WriteFile(...))
  // and get a fatal abort rather than a silent continue.
  protected: testing::AssertionResult WriteFile(
      const std::string &_path, const std::string &_data)
  {
    std::ofstream out(_path, std::ios::binary);
    if (!out.is_open())
      return testing::AssertionFailure() << "open failed: " << _path;
    out << _data;
    return testing::AssertionSuccess();
  }

  protected: std::unique_ptr<common::TempDirectory> sandbox;
  protected: std::string allowedDir;
  protected: std::string fuelCacheDir;
  protected: std::string outsideDir;
  protected: std::string secretPath;
  protected: std::string escapeSymlink;
  protected: bool symlinkSupported{false};
};

// -------------------------------------------------------------------------
// OnAsset: malicious-input regression tests for issue #3589.
// Before the fix, any of these would return a `gz.msgs.Bytes` payload
// containing the raw file contents. With the allowlist check in place,
// each must produce a `gz.msgs.StringMsg` error frame instead.
// -------------------------------------------------------------------------

TEST_F(WebsocketServerTest, OnAsset_EmptyUri)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  // OnAsset checks frameParts.size() <= 1 first; pass a single "asset"
  // element so we exercise the dedicated `asset_uri_missing` branch.
  auto f = this->CallOnAsset(server, 1, {"asset"});

  EXPECT_EQ("asset", f.op);
  EXPECT_EQ("gz.msgs.StringMsg", f.type);

  msgs::StringMsg msg;
  ASSERT_TRUE(msg.ParseFromString(f.payload));
  EXPECT_EQ("asset_uri_missing", msg.data());
}

TEST_F(WebsocketServerTest, OnAsset_AbsolutePathOutsideAllowlist)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  // `secret.txt` exists on disk (so common::exists short-circuits the
  // resolver service call), but lives outside both GZ_SIM_RESOURCE_PATH
  // and the fuel cache. weakly_canonical resolves it as-is and the
  // allowlist check must reject it.
  auto f = this->CallOnAsset(server, 1, {"asset", this->secretPath, "", ""});

  EXPECT_EQ("asset", f.op);
  EXPECT_EQ("gz.msgs.StringMsg", f.type);
  msgs::StringMsg msg;
  ASSERT_TRUE(msg.ParseFromString(f.payload));
  EXPECT_EQ("asset_access_denied", msg.data())
      << "off-allowlist access was permitted; #3589 regressed";
}

TEST_F(WebsocketServerTest, OnAsset_SymlinkEscape)
{
  if (!this->symlinkSupported)
    GTEST_SKIP() << "symlinks unsupported on this filesystem";

  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  // `escape` lives inside GZ_SIM_RESOURCE_PATH textually, but the link
  // target is the outside-tree `secret.txt`. weakly_canonical resolves
  // the link before the allowlist check; the check must catch the escape.
  auto f = this->CallOnAsset(server, 1,
      {"asset", this->escapeSymlink, "", ""});

  EXPECT_EQ("asset", f.op);
  EXPECT_EQ("gz.msgs.StringMsg", f.type);
  msgs::StringMsg msg;
  ASSERT_TRUE(msg.ParseFromString(f.payload));
  EXPECT_EQ("asset_access_denied", msg.data())
      << "symlink-escape access was permitted; allowlist canonicalization "
         "may be skipping link resolution";
}

TEST_F(WebsocketServerTest, OnAsset_ParentDirectoryTraversal)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  // Path inside the sandbox-allowed dir, but with `..` segments that
  // weakly_canonical will normalize back out of the allowlist.
  std::string traversal = common::joinPaths(
      this->allowedDir, "..", "outside", "secret.txt");
  auto f = this->CallOnAsset(server, 1, {"asset", traversal, "", ""});

  EXPECT_EQ("asset", f.op);
  EXPECT_EQ("gz.msgs.StringMsg", f.type);
  msgs::StringMsg msg;
  ASSERT_TRUE(msg.ParseFromString(f.payload));
  EXPECT_EQ("asset_access_denied", msg.data())
      << "parent-directory traversal was permitted; #3589 regressed";
}

TEST_F(WebsocketServerTest, OnAsset_DirectoryPrefixCollision)
{
  // A path whose name shares the same string prefix as the allowed directory
  // but is a sibling, not a child. The production allowlist check appends a
  // trailing separator before the prefix match; without that guard, a naive
  // string-prefix test would wrongly admit this sibling.
  std::string evilDir = this->allowedDir + "-evil";
  ASSERT_TRUE(common::createDirectories(evilDir));
  std::string evilFile = common::joinPaths(evilDir, "evil.txt");
  ASSERT_TRUE(this->WriteFile(evilFile, "must-not-leak"));

  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);
  auto f = this->CallOnAsset(server, 1, {"asset", evilFile, "", ""});

  EXPECT_EQ("asset", f.op);
  EXPECT_EQ("gz.msgs.StringMsg", f.type);
  msgs::StringMsg msg;
  ASSERT_TRUE(msg.ParseFromString(f.payload));
  EXPECT_EQ("asset_access_denied", msg.data())
      << "sibling directory whose name starts with the allowed-dir name was "
         "admitted; allowlist prefix check may be missing trailing-separator "
         "guard";
}

// -------------------------------------------------------------------------
// OnAsset: legitimate access tests. The allowlist check must not break
// the happy-path delivery of files that live under an allowed root.
// -------------------------------------------------------------------------

TEST_F(WebsocketServerTest, OnAsset_AllowedResourcePath)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  std::string okPath = common::joinPaths(this->allowedDir, "ok.txt");
  auto f = this->CallOnAsset(server, 1, {"asset", okPath, "", ""});

  EXPECT_EQ("asset", f.op);
  EXPECT_EQ("gz.msgs.Bytes", f.type)
      << "asset under GZ_SIM_RESOURCE_PATH was rejected; allowlist is "
         "too strict";

  msgs::Bytes bytes;
  ASSERT_TRUE(bytes.ParseFromString(f.payload));
  EXPECT_EQ("hello", bytes.data());
}

TEST_F(WebsocketServerTest, OnAsset_AllowedFuelCachePath)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  std::string cachedPath =
      common::joinPaths(this->fuelCacheDir, "cached.txt");
  auto f = this->CallOnAsset(server, 1, {"asset", cachedPath, "", ""});

  EXPECT_EQ("asset", f.op);
  EXPECT_EQ("gz.msgs.Bytes", f.type)
      << "asset under GZ_FUEL_CACHE_PATH was rejected; fuel-cache allow "
         "is missing or broken";

  msgs::Bytes bytes;
  ASSERT_TRUE(bytes.ParseFromString(f.payload));
  EXPECT_EQ("fuel-payload", bytes.data());
}

TEST_F(WebsocketServerTest, OnAsset_AllowlistedFileNotReadable)
{
  // File is under GZ_SIM_RESOURCE_PATH (passes allowlist) but has no read
  // permission, so infile.open() fails. Expects asset_not_found.
  if (geteuid() == 0)
    GTEST_SKIP() << "permission-based test is meaningless when running as root";

  std::string noreadPath =
      common::joinPaths(this->allowedDir, "noread.txt");
  ASSERT_TRUE(this->WriteFile(noreadPath, "should-not-be-served"));
  ASSERT_EQ(0, chmod(noreadPath.c_str(), 0000))
      << "Failed to set file permissions; test cannot proceed";

  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);
  auto f = this->CallOnAsset(server, 1, {"asset", noreadPath, "", ""});

  chmod(noreadPath.c_str(), 0644);  // restore before TearDown deletes it

  EXPECT_EQ("asset", f.op);
  EXPECT_EQ("gz.msgs.StringMsg", f.type);
  msgs::StringMsg msg;
  ASSERT_TRUE(msg.ParseFromString(f.payload));
  EXPECT_EQ("asset_not_found", msg.data())
      << "unreadable allowlisted file was served or wrongly denied; "
         "expected asset_not_found after open() failed";
}

// -------------------------------------------------------------------------
// OnMessage: binary request payloads.
// A `req` payload is a serialized protobuf. It can hold zero bytes and
// commas, and must reach the service unchanged.
// -------------------------------------------------------------------------

namespace
{
// Records the last request a test service received.
struct EchoRecorder
{
  std::mutex mutex;
  std::string received;
  int calls{0};

  bool OnRequest(const msgs::StringMsg &_req, msgs::StringMsg &_rep)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->received = _req.SerializeAsString();
    ++this->calls;
    _rep.set_data("ok");
    return true;
  }
};
}  // namespace

TEST_F(WebsocketServerTest, OnMessage_RequestBinaryPayload)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  EchoRecorder recorder;
  transport::Node node;
  ASSERT_TRUE(node.Advertise("/test/ws_echo", &EchoRecorder::OnRequest,
      &recorder));

  // The payload holds a comma (0x2c) and zero bytes. Zero bytes also make
  // the frame differ from the C string that ends at the first one.
  msgs::StringMsg req;
  req.set_data(std::string("a,b\0c,d", 7));
  const std::string payload = req.SerializeAsString();
  ASSERT_NE(std::string::npos, payload.find('\0'));
  ASSERT_NE(std::string::npos, payload.find(','));

  const std::string frame =
      "req,/test/ws_echo,gz.msgs.StringMsg," + payload;
  server.OnMessage(1, frame);

  {
    std::lock_guard<std::mutex> lock(recorder.mutex);
    EXPECT_EQ(1, recorder.calls);
    EXPECT_EQ(payload, recorder.received);
  }

  auto f = ParseFrame(this->PopFrame(server, 1));
  EXPECT_EQ("req", f.op);
  EXPECT_EQ("gz.msgs.StringMsg", f.type);
  msgs::StringMsg rep;
  ASSERT_TRUE(rep.ParseFromString(f.payload));
  EXPECT_EQ("ok", rep.data());
}

TEST_F(WebsocketServerTest, OnMessage_RequestPoseIdentityOrientation)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  struct PoseRecorder
  {
    std::mutex mutex;
    std::string received;
    bool OnRequest(const msgs::Pose &_req, msgs::StringMsg &_rep)
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->received = _req.SerializeAsString();
      _rep.set_data("ok");
      return true;
    }
  } recorder;

  transport::Node node;
  ASSERT_TRUE(node.Advertise("/test/ws_pose", &PoseRecorder::OnRequest,
      &recorder));

  msgs::Pose pose;
  pose.set_name("box");
  pose.mutable_position()->set_x(1.0);
  pose.mutable_position()->set_y(2.0);
  pose.mutable_orientation()->set_w(1.0);
  const std::string payload = pose.SerializeAsString();

  server.OnMessage(1, "req,/test/ws_pose,gz.msgs.Pose," + payload);

  std::lock_guard<std::mutex> lock(recorder.mutex);
  EXPECT_EQ(payload, recorder.received);
}

TEST_F(WebsocketServerTest, OnMessage_RequestEmptyPayload)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  EchoRecorder recorder;
  transport::Node node;
  ASSERT_TRUE(node.Advertise("/test/ws_empty", &EchoRecorder::OnRequest,
      &recorder));

  // A message with only default values serializes to zero bytes, so the
  // frame ends right after the third comma.
  server.OnMessage(1, "req,/test/ws_empty,gz.msgs.StringMsg,");

  {
    std::lock_guard<std::mutex> lock(recorder.mutex);
    EXPECT_EQ(1, recorder.calls);
    EXPECT_EQ("", recorder.received);
  }

  auto f = ParseFrame(this->PopFrame(server, 1));
  EXPECT_EQ("req", f.op);
  msgs::StringMsg rep;
  ASSERT_TRUE(rep.ParseFromString(f.payload));
  EXPECT_EQ("ok", rep.data());
}

TEST_F(WebsocketServerTest, OnMessage_RequestMalformedFrames)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);

  // Too few commas: rejected as an invalid frame, nothing is queued.
  server.OnMessage(1, "req,/test/none");
  EXPECT_EQ("", this->PopFrame(server, 1));

  // Empty service: reported as not found instead of crashing.
  server.OnMessage(1, "req,,gz.msgs.StringMsg,payload");
  auto f = ParseFrame(this->PopFrame(server, 1));
  EXPECT_EQ("req", f.op);
  msgs::StringMsg rep;
  ASSERT_TRUE(rep.ParseFromString(f.payload));
  EXPECT_EQ("service_not_found", rep.data());
}

TEST_F(WebsocketServerTest, OnMessage_AuthWithEmptyFields)
{
  sim::systems::WebsocketServer server;
  this->InjectAuthorizedConnection(server, 1);
  this->RequireAuthorization(server, 1, "secret");

  // Other operations still split with empty fields merged, so the key is
  // the last part of "auth,,,secret".
  server.OnMessage(1, "auth,,,secret");
  EXPECT_TRUE(this->IsAuthorized(server, 1));
  EXPECT_EQ("authorized", this->PopFrame(server, 1));
}
