\page example_triangle Triangle example

This tutorial explains how to use the `Triangle` class from Ignition Math library.

## Compile the code

Go to `ign-math/examples` and use `cmake` to compile the code:

```{.sh}
cd ign-math/examples
mkdir build
cd build
cmake ..
make
```

When the code is compiled, run:

```{.sh}
./triangle_example
```

The ouput of the program:

```{.sh}
Vertex 1: -1 0
Vertex 2: 0 1
Vertex 3: 1 0
Side 1: -1 0 0 1
Side 2: 0 1 1 0
Side 3: 1 0 -1 0
Perimeter=4.82843 Area=1
Triangle contains the point 0, 0.5
A line from (-2, 0.5) to (2, 0.5) intersects the triangle at the
following points:
	 Pt1=-0.5 0.5
	 Pt2=0.5 0.5
```

## Code

Create a triangle with the following vertices:

```{.cpp}
ignition::math::Triangle tri(
    ignition::math::Vector2d(-1, 0),
    ignition::math::Vector2d(0, 1),
    ignition::math::Vector2d(1, 0));
```

The individual vertices are accessible through the `[]` operator.


```{.cpp}
std::cout << "Vertex 1: " << tri[0] << "\n"
          << "Vertex 2: " << tri[1] << "\n"
          << "Vertex 3: " << tri[2] << "\n";
```

Each side of the triangle is also accessible via the `Side` function.

```{.cpp}
std::cout << "Side 1: " << tri.Side(0) << "\n"
          << "Side 2: " << tri.Side(1) << "\n"
          << "Side 3: " << tri.Side(2) << "\n";
```

It's also possible to set each vertex individually or set all the vertices at once.

```{.cpp}
tri.Set(0, ignition::math::Vector2d(-10, 0));
tri.Set(1, ignition::math::Vector2d(0, 20));
tri.Set(2, ignition::math::Vector2d(10, 2));

tri.Set(ignition::math::Vector2d(-1, 0),
        ignition::math::Vector2d(0, 1),
        ignition::math::Vector2d(1, 0));
```

You can get the perimeter length and area of the triangle

```{.cpp}
std::cout << "Perimeter=" << tri.Perimeter()
          << " Area=" << tri.Area() << "\n";
```

The `Contains` function checks if a line or point is inside the triangle.

```{.cpp}
if (tri.Contains(ignition::math::Vector2d(0, 0.5)))
  std::cout << "Triangle contains the point 0, 0.5\n";
else
  std::cout << "Triangle does not contain the point 0, 0.5\n";
```

The `Intersects` function checks if a line segment intersects the triangle. It also returns the points of intersection.

```{.cpp}
ignition::math::Vector2d pt1, pt2;
if (tri.Intersects(ignition::math::Line2d(-2, 0.5, 2, 0.5), pt1, pt2))
{
  std::cout << "A line from (-2, 0.5) to (2, 0.5) intersects "
            << "the triangle at the\nfollowing points:\n"
            << "\t Pt1=" << pt1 << "\n"
            << "\t Pt2=" << pt2 << "\n";
}
else
{
  std::cout << "A line from (-2, 0.5) to (2, 0.5) does not intersect "
            << "the triangle\n";
}
```

There are more functions in `Triangle`. Take a look at the [API](https://ignitionrobotics.org/api/math/6.4/index.html)
