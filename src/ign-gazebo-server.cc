#include <ignition/gazebo/Server.hh>
#include <ignition/common/Console.hh>

int main(int argc, char** argv)
{
  ignition::common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "ign_gazebo/src/systems");
  ignition::common::Console::SetVerbosity(4);

  ignition::gazebo::ServerConfig serverConfig;
  ignition::gazebo::Server server(serverConfig);

  server.Run(true, 0, true);
  return 0;
}

