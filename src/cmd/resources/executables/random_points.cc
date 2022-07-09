//This is a sample code showing how an ignition executable works
#include <iostream>
#include <ignition/msgs.hh>
int main()
{
  ignition::msgs::Vector3d point1;
  point1.set_x(std::rand());
  point1.set_y(std::rand());
  point1.set_z(std::rand());
  ignition::msgs::Vector3d point2;
  point2.set_x(std::rand());
  point2.set_y(std::rand());
  point2.set_z(std::rand());
  std::cout << "Random_point1:\n" << point1.DebugString() << std::endl;
  std::cout << "Random_point2:\n" << point2.DebugString() << std::endl;
  return 0;
}