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
