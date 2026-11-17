CXX ?= g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Werror -I../../SenseGrip
slip_replay: slip_replay.cpp ../../SenseGrip/slip_core.cpp ../../SenseGrip/slip_core.h
	$(CXX) $(CXXFLAGS) -o $@ slip_replay.cpp ../../SenseGrip/slip_core.cpp
clean:
	rm -f slip_replay
