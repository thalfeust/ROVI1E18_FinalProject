# ROVI1E18_FinalProject

*** File structure ***
root : 
	- Plugin
		- *SamplePluginPA10 // to configure
			- [...]
	- Robotic/
		- *build // to create
		- src/*
	- Vision/
		- *build // to create
		- CMakeList.txt
		- data/
			- marker_4.png
			- marker_color/*
			- marker_corny/*
			- marker_color_hard/*
			- marker_corny_hard/*
		- src		
	- Workcells
		- *PA10Workcell // to configure
			- [...]

* : files not in the Git repository

// VISION PART

to execute the programs :

1) Create Vision/build
2) Inside build : cmake ..
3) Inside build : make

Commands Marker Color :
-> ./mainT_seq1 [PATH = ../data/marker_corny/]
examples :
./mainT_seq1 ../data/marker_color/
./mainT_seq1 ../data/marker_color_hard/

Commands Marker Corny :
-> ./mainT_seq3 ../data/marker_4.png [PATH=../data/marker_corny/]
examples :
./mainT_seq3 ../data/marker_4.png ../data/marker_corny/
./mainT_seq3 ../data/marker_4.png ../data/marker_corny_hard/

// ROBOTIC PART

same steps 1), 2) and 3)

Commands :
-> ./main [MOTIONS=Slow : (Slow,Medium,Fast)] [PTS_NBR=1 : (1,3)] [OUTPUT=None : (Q,tool,error)]
examples : 
./main Fast 3 error  -> 3 pts tracking on the motion Fast and print the error
./main Slow 1 Q      -> 1 pts tracking on the motion Slow and print each Q
./main Medium 1 tool -> 1 pts tracking on the motion Medium and print each poses of tool Frame


// FUSION PART

/!\ ****** /!\

in the file Plugin/SamplePluginPA10/src/SamplePlugin.cpp :
- Verify the paths configuration (begin line 27)
- do the steps 1), 2) and 3)
