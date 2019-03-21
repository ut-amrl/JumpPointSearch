CXX = g++
FLAGS = -std=c++11 -march=native -g -I/opt/X11/include -I/usr/include/eigen3 -I/usr/local/include/eigen3 -L/opt/X11/lib -I..
OPTIMIZE = -O3

.DEFAULT = all

all: bin/jps

test: bin/test_jps
	./bin/test_jps

bin/jps: jps_main.cpp jps.cpp obstaclemap.cpp imagemap.cpp jps.h \
		simple_queue.h redundant_queue.h Makefile
	$(CXX) $(FLAGS) $(OPTIMIZE) jps_main.cpp obstaclemap.cpp \
		imagemap.cpp jps.cpp \
		../util/timer.cc ../util/random.cc \
		-lX11 -lpthread -o bin/jps -DNDEBUG

bin/test_jps: jps_tests.cpp jps.cpp jps.h simple_queue.h Makefile
	$(CXX) $(FLAGS) -O0 jps_tests.cpp jps.cpp -lX11 -lpthread -lgtest -lglog -o bin/test_jps

clean:
	rm -rf bin/*
