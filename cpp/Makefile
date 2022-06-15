CXX :=$(if $(shell which g++-11),g++-11,g++)
CC = $(CXX)
CFLAGS = -g -O2 -std=c++11 -Wunused-result -fopenmp

INTGRAPH=CIntGraph
INTGRAPH_HPP=../sparsedomsets/cpp/graph/CIntGraph.hpp
INTGRAPH_CPP=../sparsedomsets/cpp/graph/CIntGraph.cpp
INTDIGRAPH=CIntDiGraph
INTDIGRAPH_HPP=../sparsedomsets/cpp/graph/CIntDiGraph.hpp
INTDIGRAPH_CPP=../sparsedomsets/cpp/graph/CIntDiGraph.cpp
# INTGRAPHSLOW=CIntGraphSlow
# INTGRAPHSLOW_HPP=../sparsedomsets/cpp/graph/CIntGraphSlow.hpp
# INTGRAPHSLOW_CPP=../sparsedomsets/cpp/graph/CIntGraphSlow.cpp
GRAPH_OPS=c_graph_ops
GRAPH_OPS_HPP=../sparsedomsets/cpp/graph/c_graph_ops.hpp
GRAPH_OPS_CPP=../sparsedomsets/cpp/graph/c_graph_ops.cpp
EDGELIST=c_edgelist
EDGELIST_HPP=../sparsedomsets/cpp/readwrite/c_edgelist.hpp
EDGELIST_CPP=../sparsedomsets/cpp/readwrite/c_edgelist.cpp
DOMINATING=c_dominating
DOMINATING_HPP=../sparsedomsets/cpp/algorithms/c_dominating.hpp
DOMINATING_CPP=../sparsedomsets/cpp/algorithms/c_dominating.cpp
PARTITION=c_partition
PARTITION_HPP=../sparsedomsets/cpp/algorithms/c_partition.hpp
PARTITION_CPP=../sparsedomsets/cpp/algorithms/c_partition.cpp
FLOW=c_flow
FLOW_HPP=../sparsedomsets/cpp/algorithms/c_flow.hpp
FLOW_CPP=../sparsedomsets/cpp/algorithms/c_flow.cpp

TIMER_HPP=timer.hpp

DISTANCE_CLOSURE=distance_closure
DISTANCE_CLOSURE_BIN=../bin/$(DISTANCE_CLOSURE)
DISTANCE_CLOSURE_SIZE=distance_closure_size
DISTANCE_CLOSURE_SIZE_BIN=../bin/$(DISTANCE_CLOSURE_SIZE)

DOMSET=domset
DOMSET_BIN=../bin/$(DOMSET)

NBRPRT=nbrprt
NBRPRT_BIN=../bin/$(NBRPRT)

DOMSET_STATS=domset_stats
DOMSET_STATS_BIN=../bin/domset-stats

MAKE_ILP=make_ilp
MAKE_ILP_BIN=../bin/${MAKE_ILP}

MAKE_QP=make_qp
MAKE_QP_BIN=../bin/${MAKE_QP}

PARSE_QP=parse_qp
PARSE_QP_BIN=../bin/${PARSE_QP}

#all: $(DISTANCE_CLOSURE) $(DISTANCE_CLOSURE_SIZE) $(DOMSET) $(DOMSET_STATS) $(MAKE_ILP) $(MAKE_QP) $(PARSE_QP) $(NBRPRT)
all: $(DOMSET) $(MAKE_ILP) $(MAKE_QP) $(PARSE_QP) $(NBRPRT)

$(DISTANCE_CLOSURE): build/$(DISTANCE_CLOSURE).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(GRAPH_OPS).o build/$(EDGELIST).o
	$(CXX) $(CFLAGS) build/$(DISTANCE_CLOSURE).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(GRAPH_OPS).o build/$(EDGELIST).o -o $(DISTANCE_CLOSURE_BIN)

$(DISTANCE_CLOSURE_SIZE): build/$(DISTANCE_CLOSURE_SIZE).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(GRAPH_OPS).o build/$(EDGELIST).o
	$(CXX) $(CFLAGS) build/$(DISTANCE_CLOSURE_SIZE).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(GRAPH_OPS).o build/$(EDGELIST).o -o $(DISTANCE_CLOSURE_SIZE_BIN)

$(DOMSET): build/$(DOMSET).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(DOMINATING).o build/$(GRAPH_OPS).o
	$(CXX) $(CFLAGS) build/$(DOMSET).o build/$(INTGRAPH).o  build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(DOMINATING).o build/$(GRAPH_OPS).o -o $(DOMSET_BIN)

$(NBRPRT): build/$(NBRPRT).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(PARTITION).o build/$(FLOW).o build/$(GRAPH_OPS).o
	$(CXX) $(CFLAGS) build/$(NBRPRT).o build/$(INTGRAPH).o  build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(PARTITION).o build/$(FLOW).o build/$(GRAPH_OPS).o -o $(NBRPRT_BIN)

$(DOMSET_STATS): build/$(DOMSET_STATS).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o  build/$(EDGELIST).o build/$(GRAPH_OPS).o
	$(CXX) $(CFLAGS) build/$(DOMSET_STATS).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o  build/$(EDGELIST).o build/$(GRAPH_OPS).o -o $(DOMSET_STATS_BIN)

$(MAKE_ILP): build/$(MAKE_ILP).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(GRAPH_OPS).o
	$(CXX) $(CFLAGS) build/$(MAKE_ILP).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(GRAPH_OPS).o -o $(MAKE_ILP_BIN)

$(MAKE_QP): build/$(MAKE_QP).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(GRAPH_OPS).o
	$(CXX) $(CFLAGS) build/$(MAKE_QP).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(GRAPH_OPS).o -o $(MAKE_QP_BIN)

$(PARSE_QP): build/$(PARSE_QP).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(GRAPH_OPS).o
	$(CXX) $(CFLAGS) build/$(PARSE_QP).o build/$(INTGRAPH).o build/$(INTDIGRAPH).o build/$(EDGELIST).o build/$(GRAPH_OPS).o -o $(PARSE_QP_BIN)

build/$(DISTANCE_CLOSURE).o: $(DISTANCE_CLOSURE).cpp $(INTGRAPH_HPP) $(INTDIGRAPH_HPP) $(GRAPH_OPS_HPP) $(EDGELIST_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(DISTANCE_CLOSURE_SIZE).o: $(DISTANCE_CLOSURE_SIZE).cpp $(TIMER_HPP) $(INTGRAPH_HPP) $(INTDIGRAPH_HPP) $(GRAPH_OPS_HPP) $(EDGELIST_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(DOMSET).o: $(DOMSET).cpp $(TIMER_HPP) $(EDGELIST_HPP) $(DOMINATING_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(NBRPRT).o: $(NBRPRT).cpp $(TIMER_HPP) $(EDGELIST_HPP) $(PARTITION_HPP) $(FLOW_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(DOMSET_STATS).o: $(DOMSET_STATS).cpp $(EDGELIST_HPP) $(GRAPH_OPS_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(MAKE_ILP).o: $(MAKE_ILP).cpp $(TIMER_HPP) $(EDGELIST_HPP) $(GRAPH_OPS_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(MAKE_QP).o: $(MAKE_QP).cpp $(TIMER_HPP) $(EDGELIST_HPP) $(GRAPH_OPS_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(PARSE_QP).o: $(PARSE_QP).cpp $(TIMER_HPP) $(EDGELIST_HPP) $(GRAPH_OPS_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(INTGRAPH).o: $(INTGRAPH_CPP) $(INTGRAPH_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(INTDIGRAPH).o: $(INTDIGRAPH_CPP) $(INTDIGRAPH_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(DOMINATING).o: $(DOMINATING_CPP) $(DOMINATING_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(PARTITION).o: $(PARTITION_CPP) $(PARTITION_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(FLOW).o: $(FLOW_CPP) $(FLOW_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(GRAPH_OPS).o: $(GRAPH_OPS_CPP) $(GRAPH_OPS_HPP) $(INTGRAPH_HPP) $(INTDIGRAPH_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@

build/$(EDGELIST).o: $(EDGELIST_CPP) $(EDGELIST_HPP) $(INTGRAPH_HPP) $(INTDIGRAPH_HPP)
	$(CXX) -c $(CFLAGS) $< -o $@
