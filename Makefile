CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -IGridWorld -ILTLStuff -I$(HOME)/miniconda3/envs/spotenv/include -I/usr/local/include
LDFLAGS = -L$(HOME)/miniconda3/envs/spotenv/lib -lspot -lbddx -lgvc -lcgraph -L/usr/local/lib -lsfml-graphics -lsfml-window -lsfml-system

SRC = main.cpp LTLStuff/TEGProblem.cpp GridWorld/GridWorldDomain.cpp GridWorld/GridWorldPlotter.cpp
OBJ = $(SRC:.cpp=.o)
DEP = $(OBJ:.o=.d)
HEADERS = LTLFormula.h GridState.h GridAction.h GridWorldDomain.h TEGProblem.h ProductState.h GridWorldPlotter.h Action.h SkillAction.h DFANode.h
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
