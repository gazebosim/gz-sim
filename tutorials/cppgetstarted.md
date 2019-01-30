\page cppgetstarted C++ Get Started

Previous Tutorial: \ref install 

## Overview

This tutorial describes how to get started using Ignition Math with C++.

We will run through an example that determines the distance between two
points in 3D space. Start by creating a bare-bones main file using the
editor of your choice.

```{.cpp}
int main()
{
  return 0;
}
```

The easiest way to include Ignition Math is through the `ignition/math.hh`
header file. Alternatively, you can include only the header files you need.
For this example, we'll take the short and easy approach. 

At this point your main file should look like

```{.cpp}
#include <ignition/math.hh>

int main()
{
  return 0;
}
```

Now let's create to 3D points with arbitrary values. We will use the
ignition::math::Vector3 class to represent these points. Ignition Math provides a handy
ignition::math::Vector3d type which is a typedef of `Vector3<double>`. The result of this
addition will be a main file similar to the following.

```{.cpp}
#include <ignition/math.hh>

int main()
{
  ignition::math::Vector3d point1(1, 3, 5);
  ignition::math::Vector3d point2(2, 4, 6);

  return 0;
}
```

Finally, we can compute the distance between `point1` and `point2` using the
ignition::math::Vector3::Distance() function and output the distance value.

```{.cpp}
#include <ignition/math.hh>

int main()
{
  ignition::math::Vector3d point1(1, 3, 5);
  ignition::math::Vector3d point2(2, 4, 6);

  double distance = point1.Distance(point2);
  std::cout << "Distance from " << point1 << " to " << point2 << " is " <<
    distance << std::endl;
  return 0;
}
```

## Bonus: Vector2 Example

The following is an example program that uses Vector2 to perform some simple
computation. 

\snippet examples/vector2_example.cc complete
