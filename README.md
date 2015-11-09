Voxelyze
========

Voxelyze is a general purpose multi-material voxel simulation library for static and dynamic analysis. To quickly get a feel for its capabilities you can create and play with Voxelyze objects using [VoxCAD](http://www.voxcad.com) (Windows and Linux executables available). An paper describing the theory and capabilities of Voxelyze has been published in Soft Robotics journal: "[Dynamic Simulation of Soft Multimaterial 3D-Printed Objects](http://online.liebertpub.com/doi/pdfplus/10.1089/soro.2013.0010)" (2014). [Numerous](https://sites.google.com/site/jonhiller/hardware/soft-robots) 
[academic](http://creativemachines.cornell.edu/soft-robots), [corporate](http://www.fastcompany.com/3006259/stratasyss-programmable-materials-just-add-water), and [educational](http://www.sciencebuddies.org/science-fair-projects/project_ideas/Robotics_p016.shtml) projects make use of Voxelyze.


User guide: [https://github.com/jonhiller/Voxelyze/wiki](https://github.com/jonhiller/Voxelyze/wiki)

Documentation: [http://jonhiller.github.io/Voxelyze/annotated.html](http://jonhiller.github.io/Voxelyze/annotated.html)


Basic Usage
--------

Basic use of Voxelyze consists of five simple steps:

1. Create a Voxelyze instance
2. Create a material
3. Add voxels using this material
4. Specify voxels that should be fixed in place or have force applied
5. Execute timesteps

```c++
#include "Voxelyze.h"
CVoxelyze Vx(0.005); //5mm voxels
CVX_Material* pMaterial = Vx.addMaterial(1000000, 1000); //A material with stiffness E=1MPa and density 1000Kg/m^3
CVX_Voxel* Voxel1 = Vx.setVoxel(pMaterial, 0, 0, 0); //Voxel at index x=0, y=0. z=0
Voxel1->external()->setFixedAll(); //Fixes all 6 degrees of freedom with an external condition
for (int i=0; i<100; i++) Vx.doTimeStep(); //simulates 100 timesteps
```

That particular example won't do anything interesting - the single voxel will just stay put. Here is another slightly more interesting example to create and test a 3-voxel cantilever beam

```c++
#include "Voxelyze.h"
CVoxelyze Vx(0.005); //5mm voxels
CVX_Material* pMaterial = Vx.addMaterial(1000000, 1000); //A material with stiffness E=1MPa and density 1000Kg/m^3
CVX_Voxel* Voxel1 = Vx.setVoxel(pMaterial, 0, 0, 0); //Voxel at index x=0, y=0. z=0
CVX_Voxel* Voxel2 = Vx.setVoxel(pMaterial, 1, 0, 0);
CVX_Voxel* Voxel3 = Vx.setVoxel(pMaterial, 2, 0, 0); //Beam extends in the +X direction

Voxel1->external()->setFixedAll(); //Fixes all 6 degrees of freedom with an external condition on Voxel 1
Voxel3->external()->setForce(0, 0, -1); //pulls Voxel 3 downward with 1 Newton of force.

for (int i=0; i<100; i++) Vx.doTimeStep(); //simulate  100 timesteps.

```

For further usage, please refer to the [user guide](https://github.com/jonhiller/Voxelyze/wiki). Complete documentation is avaialable [here](http://jonhiller.github.io/Voxelyze/annotated.html).

Compiling Voxelyze
--------

The Voxelyze code is structured as a library and compiles on windows and linux. An included Visual Studio project compiles Voxelyze to a static *.lib, and the usual "make" can be executed on linux to build a static *.a library.

Define "USE_OMP" in the preprocessor to enable multithreaded solving.

