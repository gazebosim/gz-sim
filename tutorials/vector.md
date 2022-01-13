\page vector Vector example

This tutorial explains how to use the `Vector` classes from Ignition Math library.

## C++ example

### Compile the code

To compile the code, go to `ign-math/examples` and use `cmake`:

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
./vector2_example
```

The ouput of the program:

```{.sh}
Vec2: 2 4
Vec2a: 1 2
Vec2b: 1.2 3.4
Vec2: x=2 y=4
Vec2a: x=1 y=2
Vec2b: x=1.2 y=3.4
4
2 8
3 6
1 2
2 2
2.23607
```

### Code

Create a `Vector2` called `vec2` of doubles using the typedef `Vector2d`. **The initial x and y values are zero**. The x and y component of `vec2` can be set at anytime.

\snippet examples/vector2_example.cc constructor


The `Vector2` class is a template, so you can also create a `Vector2` using `ignition::math::Vector2<double>`:

\snippet examples/vector2_example.cc constructor2

It's also possible to set initial values. Here we are using a `Vector2` of floats:

\snippet examples/vector2_example.cc constructor3

We can output the contents of each vector using `std::cout`.

\snippet examples/vector2_example.cc stdout

You can also get access to each component in the vector using the `X()`, `Y()` accessors or the `[]` operator, The operator is clamped to the range `[0, 1]`.

\snippet examples/vector2_example.cc access

The `Vector2` class overloads many common operators, such as:

\snippet examples/vector2_example.cc operators

There are also many useful function such as finding the distance between two vectors.

\snippet examples/vector2_example.cc distance

**There are more functions in Vector2. Take a look at the [API](https://ignitionrobotics.org/libs/math)**

## Ruby examples

This example will only work if the Ruby interface library was compiled and installed. Modify the `RUBYLIB` environment variable to include the Ignition Math library install path. For example, if you install to `/usr`:

```{.sh}
export RUBYLIB=/usr/lib/ruby:$RUBYLIB
```

Execute the examples:

```{.sh}
ruby vector2_example.rb
ruby vector3_example.rb
```

### Code

Create a `Vector2` of doubles using the typedef `Vector2d`. It's possible to set initial values or use another object to create a identical copy.

```{.rb}
va = Ignition::Math::Vector2d.new(1, 2)
```

You can get access to each component in the vector using the `X()`, `Y()` accessors.

```{.rb}
printf("va = %f %f\n", va.X(), va.Y())
printf("vb = %f %f\n", vb.X(), vb.Y())
printf("vc = %f %f\n", vc.X(), vc.Y())
```

The `Vector2` class overloads many common operators, such as:

```{.rb}
vb += va
printf("vb += va: %f %f\n", vb.X(), vb.Y())
```

There are also many useful functions, such as finding the distance between two vectors or normalizing a vector.

```{.rb}
vb.Normalize
printf("vb.Normalize = %f %f\n", vb.X(), vb.Y())
printf("vb.Distance(va) = %f\n", vb.Distance(va))
```

You can create vectors with 3 dimensions using the typedef `Vector3d`:

```{.rb}
v1 = Ignition::Math::Vector3d.new(0, 0, 0)
```

You can also get access to each component in the vector using the `X()`, `Y()` and `Z()` accessors:

```{.rb}
printf("v =: %f %f %f\n", v1.X(), v1.Y(), v1.Z())
```
