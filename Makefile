build: main.cpp geometric_trajectory_optimizer.cpp
	g++ main.cpp geometric_trajectory_optimizer.cpp -o main -I/usr/include/python3.8 -lpython3.8 -lnlopt -lm
debug: main.cpp geometric_trajectory_optimizer.cpp
	g++ main.cpp geometric_trajectory_optimizer.cpp -o main -g -I/usr/include/python3.8 -lpython3.8 -lnlopt -lm
run: main
	./main
fast:
	g++ main.cpp geometric_trajectory_optimizer.cpp -o main -O3 -g -I/usr/include/python3.8 -lpython3.8 -lnlopt -lm
clean:
	rm main
push:
	cp geometric_trajectory_optimizer.cpp /home/michal/sgtdv_ws/sgtdv-ros_implementation/src/path_planning/src
	cp geometric_trajectory_optimizer.hpp /home/michal/sgtdv_ws/sgtdv-ros_implementation/src/path_planning/include
