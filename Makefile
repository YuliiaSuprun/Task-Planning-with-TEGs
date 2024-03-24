CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -IDomainStuff -ILTLStuff -I$(HOME)/miniconda3/envs/spotenv/include -I/usr/local/include -I../pddlboat/include
LDFLAGS = -L$(HOME)/miniconda3/envs/spotenv/lib -lspot -lbddx -lgvc -lcgraph -L/usr/local/lib -lsfml-graphics -lsfml-window -lsfml-system -lboost_system -L../pddlboat/build/release -lpddlboat-cpp -lpddlboat-cpp.0.1.0

SRC = main.cpp LTLStuff/TEGProblem.cpp LTLStuff/DFANode.cpp LTLStuff/DFAManager.cpp LTLStuff/ProductManager.cpp LTLStuff/DomainManager.cpp DomainStuff/GridWorldDomain.cpp DomainStuff/GridWorldPlotter.cpp DomainStuff/PDDLDomain.cpp DomainStuff/PDDLProblem.cpp DomainStuff/PDDLAction.cpp DomainStuff/PDDLState.cpp
OBJ = $(SRC:.cpp=.o)
DEP = $(OBJ:.o=.d)
HEADERS = LTLFormula.h GridState.h GridAction.h GridWorldDomain.h TEGProblem.h ProductState.h GridWorldPlotter.h Action.h CompoundAction.h PrimitiveAction.h DFANode.h DFAManager.h DomainManager.h ProductManager.h Domain.h PDDLDomain.h PDDLProblem.h PDDLAction.h PDDLState.h
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
