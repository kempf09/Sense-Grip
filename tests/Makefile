CXX ?= g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../SenseGrip -I.
PURE = $(wildcard ../SenseGrip/*_core.cpp ../SenseGrip/*_math.cpp ../SenseGrip/*_proto.cpp ../SenseGrip/*_detect.cpp ../SenseGrip/*_util.cpp)
TESTS = $(wildcard test_*.cpp)

.PHONY: all test clean
all: test

run_tests: runner.cpp $(TESTS) $(PURE) check.h $(wildcard ../SenseGrip/*.h)
	$(CXX) $(CXXFLAGS) -o $@ runner.cpp $(TESTS) $(PURE)

test: run_tests
	./run_tests

clean:
	rm -f run_tests
