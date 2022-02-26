\page pythongetstarted Python Get Started

Previous Tutorial: \ref cppgetstarted

## Overview

This tutorial describes how to get started using Ignition Math with Python.

**NOTE**: If you have compiled Ignition Math from source, you should export
your `PYTHONPATH`.

```bash
export PYTHONPATH=$PYTHONPATH:<path to your workspace>/install/lib/python
```

We will run through an example that determines the distance between two
points in 3D space. Start by creating a bare-bones main file using the
editor of your choice.

```python
def main():
  pass

if __name__ == "__main__":
  main()
```

The easiest way to include Ignition Math is through `import ignition.math`.

At this point your main file should look like

```python
import ignition.math

def main():
  pass

if __name__ == "__main__":
  main()
```

Now let's create two 3D points with arbitrary values. We will use the
`ignition.math.Vector3` class to represent these points. Ignition Math provides
some `Vector3` types which are: `Vector3d` (Vector3 using doubles), `Vector3f` (Vector3 using floats)
and `Vector3i` (Vector3 using integers). The result of this addition will be a
main file similar to the following.

```python
from ignition.math import Vector3d

def main():
  point1 = Vector3d(1, 3, 5)
  point2 = Vector3d(2, 4, 6)

if __name__ == "__main__":
  main()
```

Finally, we can compute the distance between `point1` and `point2` using the
`ignition.math.Vector3.distance()` function and output the distance value.

```python
from ignition.math import Vector3d

def main():
  point1 = Vector3d(1, 3, 5)
  point2 = Vector3d(2, 4, 6)

  distance = point1.distance(point2);

  print("Distance from {} to {} is {}".format(point1, point2, distance))

if __name__ == "__main__":
  main()
```
