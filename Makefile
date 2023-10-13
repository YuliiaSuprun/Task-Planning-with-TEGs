CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -IGridWorld -ILTLStuff -I$(HOME)/miniconda3/envs/spotenv/include
LDFLAGS = -L$(HOME)/miniconda3/envs/spotenv/lib -lspot -lgvc -lcgraph

SRC = main.cpp GridWorld/GridWorldDomain.cpp LTLStuff/TEGTask.cpp
OBJ = $(SRC:.cpp=.o)
DEP = $(OBJ:.o=.d)
HEADERS = LTLFormula.h GridState.h GridAction.h GridWorldDomain.h TEGTask.h ProductState.h
TARGET = main

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^

-include $(DEP)

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -MMD -c $< -o $@

clean:
	rm -f $(OBJ) $(DEP) $(TARGET)

.PHONY: all clean
