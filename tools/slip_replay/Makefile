CXX ?= g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -Werror -I../../SenseGrip
FW = ../../SenseGrip
SRC = slip_replay.cpp $(FW)/slip_core.cpp $(FW)/log_core.cpp $(FW)/fsm_core.cpp $(FW)/object_core.cpp \
      $(FW)/tof_geometry.cpp $(FW)/config_core.cpp $(FW)/fsr_filter.cpp $(FW)/servo_proto.cpp $(FW)/contact_core.cpp
all: slip_replay equiv_test
slip_replay: $(SRC) $(wildcard $(FW)/*.h)
	$(CXX) $(CXXFLAGS) -o $@ $(SRC)
EQ = equiv_test.cpp $(filter-out slip_replay.cpp,$(SRC))
equiv_test: $(EQ) $(wildcard $(FW)/*.h)
	$(CXX) $(CXXFLAGS) -o $@ $(EQ)
check: slip_replay equiv_test
	cd ../.. && tools/slip_replay/selftest.sh && tools/slip_replay/equiv_test.sh
clean:
	rm -f slip_replay equiv_test
