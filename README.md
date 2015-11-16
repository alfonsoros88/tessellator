# Tessellator

Simple executable to help me to quickly tessellate clouds.

### Compilation

```bash
mkdir build
cd build
cmake ..
make
```

### Usage

Options:

- *-i*: Input file (.pcd or .png from a dataset)
- *-o*: Output file (.ply or .vtk)

Here is an example:

```bash
./tessellate -i /path/to/input/file.pcd -o model.vtk
```
