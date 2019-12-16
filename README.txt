Preview:
![PreviewImageOfTheSimulation](https://github.com/Pintrue/Simulation/blob/master/rendersteps.png)

To compile this code, please first go to the root directory, then follow the instructions below:

	$ mkdir build
	$ cd build
	$ cmake ..
	$ make


Notice the default cmake without any option variable will compile the codebase to non-GUI version.

To compile with GUI option, follow the instructions above until "$ cd build", and use:

	$ cmake .. -DGUI=ON
	$ make

This will produce an executable that has Graphical User Interface (GUI).


To clean the already built files, make sure your working directory is in "build/", then follow the follow instructions:

	$ cd ..
	$ rm -rf build/


