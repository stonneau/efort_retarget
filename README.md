Compilation instructions:

Required external libraries:
	Eigen 3
	PQP (see below)
	Drawstuff (optional, for debugging application, let me know if you want it)

1 build PQP 1.3
	+ unzip the file "./external/PQP_v1.3.zip"
	+ follow the instructions in the readme file to run make and
	build library PQP
	
2 Generate the project with CMake
	+ If you are not familiar with CMAKE, you can use cmake-gui as a helper tool.
	+ the generation of the project file will happen after loading CMakeLists.txt
	located at the root of the project.
	+ Generate the project. If Eigen is installed it should be detected automatically,
	otherwise you'll have to indicate its location manually.
	+ Similarly you will be required to indicate the path to PQP.
	For instance on my computer the following variables are defined:
	
	PQP_INCLUDE_DIR /home/stonneau/dev/efort_retarget/external/include
	PQP_LIBRARY  /home/stonneau/dev/efort_retarget/external/PQP_v1.3/lib/libPQP.a
	PQP_LIBRARY_DIR  /home/stonneau/dev/efort_retarget/external/PQP_v1.3/lib/
	
	Once this is done the project file should generate automatically for the setting of your choice,
	then you can build the libraries.
	
3 Integrate retarget library in your project
	+ You are interested in the library retarget, which wraps 
	all other code within a standard API.
	Only one class is defined, Motion.h , which contains
	a method Retarget with th API we defined (all is documented).
	
	+ To load a motion for the test scenario considered so far 
	(rideCar.bvh), you need to call the method LoadMotion(const std::string& scenario)
	where scenario is the path to the file located in ./rami/scenarios/rami.scen
	
4. Current state of the library
	+ For copyright reasons, the octree data structure is not present in this version of the code, which might
	be a little slow at the moment.
	
	+ The range of Motion of all the effectors of the character, as well as joint limits, have not been yet defined.
	This will be updated rapidly.
	
	+ At the moment, the API only allows to perform Retargetting for one frame.
	At this stage I m mostly concerned with the ability to communicate successfully.
	
	
	
