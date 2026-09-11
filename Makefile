CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Wpedantic -O2
LDFLAGS ?=

TARGETS := spl06_altimeter mma845x_inclinometer

.PHONY: all clean

all: $(TARGETS)

spl06_altimeter: src/spl06_altimeter.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

mma845x_inclinometer: src/mma845x_inclinometer.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGETS)
