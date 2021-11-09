\page triangle Triangle example

This tutorial explains how to use the `Triangle` class from Ignition Math library.

## Compile the code

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

\snippet examples/triangle_example.cc constructor

The individual vertices are accessible through the `[]` operator.

\snippet examples/triangle_example.cc access1

Each side of the triangle is also accessible via the `Side` method. Each side consists of 2 vertices, the following code will print out the X and Y values of each vertex.

\snippet examples/triangle_example.cc access2

It's also possible to set each vertex individually or set all the vertices at once.

\snippet examples/triangle_example.cc vertex1
\snippet examples/triangle_example.cc vertex2

You can get the perimeter length and area of the triangle

\snippet examples/triangle_example.cc perimeter and area

The `Contains` function checks if a line or point is inside the triangle.

\snippet examples/triangle_example.cc contains

The `Intersects` function checks if a line segment intersects the triangle. It also returns the points of intersection.

\snippet examples/triangle_example.cc intersect

There are more functions in `Triangle`. Take a look at the [API](https://ignitionrobotics.org/api/math/6.4/index.html)
