Charles Pastuszenski
Computer Graphics
CS77
Assignment 2 README

Note: I did the de Casteljau Tessellation EC.

ABOUT:
	
	In Assignment 2, students implemented a simple OpenGL viewer with support for the 
	following features:

	Tesselation
		Cylinder uniform tesselation
		Quad/Triangle Mesh recursive tesselation
		Catmull-Clark Subdivision Surfaces recursive tesselation
		Bezier splines uniform and recursive (de Casteljau EC) tesselation
	Transformation
		Implement simple function transformation

BUILD NOTES:

	To build the project, simply type "make" in a terminal from the directory in which this
	README is located. To run the test program, type "sh render_all.sh" to run the shell
	script included in the framework that performs 8 separate modeling tests. test07.png and test08.png
	will be produced by this test. These images demonstrate that de Casteljau tessellation
	works in this implementation, with test08.png depicting the tessellation performed in
	test07.png with the de Casteljau algorithm using a uniform tessellation algorithm.