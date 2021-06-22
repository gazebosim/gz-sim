#include <ignition/common/Console.hh>

#include <ignition/gazebo/gui/Gui.hh>

#include <QDirIterator>

int main(int argc, char** argv)
{
  ignition::common::Console::SetVerbosity(4);

  QDirIterator it(":", QDirIterator::Subdirectories);
  while (it.hasNext()) {
      qDebug() << it.next();
  }

  return ignition::gazebo::gui::runGui(argc, argv, 
      "/home/mjcarroll/workspaces/bazignel_edifice2/ign_gazebo/src/gui/gui.config"
      );
}

