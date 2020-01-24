\page rotation_example Rotation example

This example explains how to use quaternions and euler angles, and how to convert between them.

## Compiling and running the code

Go to `ign-math/examples` and use `cmake` to compile the code:

```{.sh}
hg clone https://bitbucket.org/ignitionrobotics/ign-math
cd ign-math/examples
mkdir build
cd build
cmake ..
make
```

When the code is compiled, you can run two different examples, one which converts from quaternion to euler angles:

```{.sh}
Usage:
  quaternion_to_euler <float_w> <float_x> <float_y> <float_z>

Example
  quaternion_to_euler 0.5 0.5 0.5 0.5
```

And the other which converts from euler to quaternion:

```{.sh}
Usage (angles specified in radians):
  quaternion_from_euler <float_roll> <float_pitch> <float_yaw>

Example
  quaternion_from_euler 0 0 1.57
```

The ouput of each program, respectively:

```{.sh}
./quaternion_to_euler 0.5 0.5 0.5 0.5
Normalizing Quaternion components:
  W 0.5
  X 0.5
  Y 0.5
  Z 0.5
to
  W 0.5
  X 0.5
  Y 0.5
  Z 0.5

Converting to Euler angles
 roll   1.570796 radians
 pitch -0.000000 radians
 yaw    1.570796 radians

 roll   90.000000 degrees
 pitch -0.000000 degrees
 yaw    90.000000 degrees

to Rotation matrix
    0.000000   0.000000   1.000000
    1.000000   0.000000   0.000000
    0.000000   1.000000   0.000000
```

```{.sh}
./quaternion_from_euler 0 0 1.57
Converting Euler angles:
 roll   0.000000 radians
 pitch  0.000000 radians
 yaw    1.570000 radians

 roll      0.000000 degrees
 pitch     0.000000 degrees
 yaw      89.954374 degrees

to Quaternion
 W  0.707388
 X  0.000000
 Y  0.000000
 Z  0.706825

to Rotation matrix
    0.000796  -1.000000   0.000000
    1.000000   0.000796   0.000000
    0.000000   0.000000   1.000000
```

## Code

You can create some objects to express rotations:

```{.cpp}
ignition::math::Quaterniond q(roll, pitch, yaw);
ignition::math::Matrix3d m(q);
ignition::math::Vector3d euler;
```

To access the quaterions attributes:

```{.cpp}
std::cout << "\nto Quaternion\n";
printf(" W % .6f\n X % .6f\n Y % .6f\n Z % .6f\n",
      q.W(), q.X(), q.Y(), q.Z());
```

Or to acccess the rotation matrix elements:

```{.cpp}
std::cout << "\nto Rotation matrix\n";
printf("   % .6f  % .6f  % .6f\n"
       "   % .6f  % .6f  % .6f\n"
       "   % .6f  % .6f  % .6f\n",
        m(0, 0), m(0, 1), m(0, 2),
        m(1, 0), m(1, 1), m(1, 2),
        m(2, 0), m(2, 1), m(2, 2));
```
