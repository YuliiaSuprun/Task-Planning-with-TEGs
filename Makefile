CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -IDomainStuff -ILTLStuff -I$(HOME)/miniconda3/envs/spotenv/include -I/usr/local/include -I../pddlboat/include
LDFLAGS = -L$(HOME)/miniconda3/envs/spotenv/lib -lspot -lbddx -lgvc -lcgraph -L/usr/local/lib -lsfml-graphics -lsfml-window -lsfml-system -lboost_system -L../pddlboat/build/release -lpddlboat-cpp -lpddlboat-cpp.0.1.0

# Source files for each target
SRC_COMMON = LTLStuff/TEGProblem.cpp LTLStuff/DFANode.cpp LTLStuff/DFAManager.cpp LTLStuff/ProductManager.cpp LTLStuff/DomainManager.cpp DomainStuff/GridWorldDomain.cpp DomainStuff/GridWorldPlotter.cpp DomainStuff/PDDLDomain.cpp DomainStuff/PDDLProblem.cpp DomainStuff/PDDLAction.cpp DomainStuff/PDDLState.cpp
SRC_MAIN_SINGLE = main_single.cpp $(SRC_COMMON)
SRC_MAIN_SINGLE_P4P = main_single_p4p.cpp $(SRC_COMMON)
SRC_MAIN_ALL_IN_DIR = main_all.cpp $(SRC_COMMON)
SRC_MAIN_ALL_IN_DIR_P4P = main_all_in_dir_p4p.cpp $(SRC_COMMON)

# Object files for each target
OBJ_MAIN_SINGLE = $(SRC_MAIN_SINGLE:.cpp=.o)
OBJ_MAIN_SINGLE_P4P = $(SRC_MAIN_SINGLE_P4P:.cpp=.o)
OBJ_MAIN_ALL_IN_DIR = $(SRC_MAIN_ALL_IN_DIR:.cpp=.o)
OBJ_MAIN_ALL_IN_DIR_P4P = $(SRC_MAIN_ALL_IN_DIR_P4P:.cpp=.o)

HEADERS = LTLFormula.h GridState.h GridAction.h GridWorldDomain.h TEGProblem.h ProductState.h GridWorldPlotter.h Action.h CompoundAction.h PrimitiveAction.h DFANode.h DFAManager.h DomainManager.h ProductManager.h Domain.h PDDLDomain.h PDDLProblem.h PDDLAction.h PDDLState.h
TARGET = main_single main_single_p4p main_all main_all_in_dir_p4p

# Default rule
all: $(TARGET)

# Build each target
main_single: $(OBJ_MAIN_SINGLE)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^

main_single_p4p: $(OBJ_MAIN_SINGLE_P4P)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^

main_all: $(OBJ_MAIN_ALL_IN_DIR)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^

main_all_in_dir_p4p: $(OBJ_MAIN_ALL_IN_DIR_P4P)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^

# Pattern rule to compile .cpp files into .o files
%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -MMD -c $< -o $@

# Include dependencies
-include $(OBJ_MAIN_SINGLE:.o=.d)
-include $(OBJ_MAIN_SINGLE_P4P:.o=.d)
-include $(OBJ_MAIN_ALL_IN_DIR:.o=.d)
-include $(OBJ_MAIN_ALL_IN_DIR_P4P:.o=.d)

# Clean up
clean:
	rm -f $(OBJ_MAIN_SINGLE) $(OBJ_MAIN_SINGLE_P4P) $(OBJ_MAIN_ALL_IN_DIR) $(OBJ_MAIN_ALL_IN_DIR_P4P) $(TARGET) *.d

.PHONY: all clean

