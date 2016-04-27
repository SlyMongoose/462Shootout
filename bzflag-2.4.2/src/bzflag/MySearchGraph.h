#ifndef	BZF_MYSEARCHGRAPH_H
#define	BZF_MYSEARCHGRAPH_H

// YAGSBPL libraries
#include "yagsbpl_base.h"
#include "planners/A_star.h"

// A node of the graph
class myNode
{
public:
	int x, y; // Profiling observation: integer coordinates, hence operator==,
			  //  makes the search significantly faster (almost 10 folds than double)
	bool operator==(const myNode& n) { return (x == n.x && y == n.y); }; // This must be defined for the node
};

class MySearchGraph
{
public:
	MySearchGraph(void);

	int x, y; // Profiling observation: integer coordinates, hence operator==,
			  //  makes the search significantly faster (almost 10 folds than double)
};


// ============================================================
// Functions that describe the graph, placed inside a class
// Here the class "GraphFunctionContainer" is redefining the virtual functions declared in "SearchGraphDescriptorFunctionContainer"
// Thus the names of the functions are important here.

class GraphFunctionContainer : public SearchGraphDescriptorFunctionContainer<myNode, double>
{
public:

	int getHashBin(MySearchGraph& n); // Use the absolute value of x coordinate as hash bin counter. Not a good choice though!
	bool isAccessible(MySearchGraph& n);
	void getSuccessors(MySearchGraph& n, std::vector<myNode>* s, std::vector<double>* c); // Define a 8-connected graph
	double getHeuristics(MySearchGraph& n1, MySearchGraph& n2);

	// -------------------------------
	// constructors
	GraphFunctionContainer(int cr, std::vector<int> ranges)
	{
		circleRadius = cr; Xmin = ranges[0]; Xmax = ranges[1]; Ymin = ranges[2]; Ymax = ranges[3];
	}
	GraphFunctionContainer()
	{
		circleRadius = 10; Xmin = -20; Xmax = 20; Ymin = -20; Ymax = 20;
	}

private:

	int circleRadius;
	int Xmin, Xmax, Ymin, Ymax;

};

#endif