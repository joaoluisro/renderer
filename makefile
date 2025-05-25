CXX := g++
CXXFLAGS := -I$(INCDIR) -Wall -Wextra -std=c++17	-O3

SRCDIR := src
INCDIR := headers
BUILDDIR := build_make

# Source files
SRCS := main.cpp $(wildcard $(SRCDIR)/*.cpp)

# Object files in build directory
OBJS := $(BUILDDIR)/main.o \
        $(patsubst $(SRCDIR)/%.cpp,$(BUILDDIR)/%.o,$(wildcard $(SRCDIR)/*.cpp))

TARGET := main

.PHONY: all clean

all: $(BUILDDIR) $(TARGET)

# Link executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Ensure build directory exists
$(BUILDDIR):
	mkdir -p $(BUILDDIR)

# Compile main.cpp to build/main.o
$(BUILDDIR)/main.o: main.cpp | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile src/*.cpp to build/*.o
$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp | $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILDDIR) $(TARGET)