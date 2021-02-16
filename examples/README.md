## Examples

Example programs using Ignition Math.

## Build

Create directory:

```
cd examples
mkdir build
cd build
```

Configure:

```
cmake ..
```

Build on Unix:


```
make
```

To build on Windows, make sure the configuration matches `ign-math`'s 
configuration:

```
cmake -build . --config Release
```

## Run

Several executables were created in the build folder. 

For example, run the angle exaample on Unix:

```
cd examples/build
./angle_example
```

Run on Windows:

```
cd examples\build\Release
angle_example.exe
```
