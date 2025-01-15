# Compiler
CXX = g++
CXXFLAGS = -std=c++17 -I/usr/include/eigen3 -I/usr/include/python3.10 -lpython3.10

# Linker flags to include Python and matplotlib dependencies
LDFLAGS = -lpython3.10

# Targets
TARGET = driverLinearKF
SOURCES = driverLinearKF.cpp LinearKF.cpp quickPlot.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# Default target
all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

# Clean up build files
clean:
	rm -f $(OBJECTS) $(TARGET)
