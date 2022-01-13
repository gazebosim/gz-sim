\page angle Angle example

This tutorial explains how to use the `Angle` class from Ignition Math library.

## C++ example

### Compile the code

Go to `ign-math/examples` and use `cmake` to compile the code:

```{.sh}
git clone https://github.com/ignitionrobotics/ign-math/ -b ign-math6
cd ign-math/examples
mkdir build
cd build
cmake ..
make
```

When the code is compiled, run:

```{.sh}
./angle_example
```

The ouput of the program:

```{.sh}
The angle 'a' should be zero: 0
Pi in radians: 3.14159
Pi in degrees: 180
Pi + PI/2 in radians: 4.71239
Normalized to the range -Pi and Pi: -1.5708
```

### Code

The code instantiates an angle class. The default constructed angle should be zero.

\snippet examples/angle_example.cc Create an angle

There are some predefined angles, such as:

\snippet examples/angle_example.cc constant pi

By default, all values are in radians, but you can use the method `Degree` to convert to degrees.

\snippet examples/angle_example.cc Output the angle in radians and degrees.

The `Angle` class overloads the `+=`, and many other, math operators.

\snippet examples/angle_example.cc The Angle class overloads the +=, and many other, math operators.

Use the method `Normalized` to bound the value between `-PI` and `PI`.

\snippet examples/angle_example.cc normalized

## Ruby example

This example will only work if the Ruby interface library was compiled and installed. Modify the `RUBYLIB` environment variable to include the Ignition Math library install path. For example, if you install to `/usr`:

```{.sh}
export RUBYLIB=/usr/lib/ruby:$RUBYLIB
```

Execute the code:

```{.sh}
ruby angle_example.rb
```

### Code

There are some predefined values:

```{.rb}
printf("PI in degrees = %f\n", Ignition::Math::Angle.Pi.Degree)
```

Create new objects:

```{.rb}
a1 = Ignition::Math::Angle.new(1.5707)
a2 = Ignition::Math::Angle.new(0.7854)
```

Use the values in radians or degrees:

```{.rb}
printf("a1 = %f radians, %f degrees\n", a1.Radian, a1.Degree)
printf("a2 = %f radians, %f degrees\n", a2.Radian, a2.Degree)
```

The `Angle` class overloads math operators.

```{.rb}
printf("a1 * a2 = %f radians, %f degrees\n", (a1 * a2).Radian, (a1 * a2).Degree)
printf("a1 + a2 = %f radians, %f degrees\n", (a1 + a2).Radian, (a1 + a2).Degree)
printf("a1 - a2 = %f radians, %f degrees\n", (a1 - a2).Radian, (a1 - a2).Degree)
```

Normalize the value between `-PI` and `PI`.

```{.rb}
a3 = Ignition::Math::Angle.new(15.707)
printf("a3 = %f radians, %f degrees\n", a3.Radian, a3.Degree)
a3.Normalize
printf("a3.Normalize = %f radians, %f degrees\n", a3.Radian, a3.Degree)
```
