/*
 * Copyright (c) 2017 Ultimaker B.V.
 *
 * CuraEngine is released under the terms of the AGPLv3 or higher.
 */
#ifndef CURA_INFILL_FREMATSPIRAL_H
#define CURA_INFILL_FREMATSPIRAL_H

#include <cstdint>

#include "../../libs/clipper/clipper.hpp"
#include "../utils/polygon.h"
#include "../sliceDataStorage.h"

/*
 * This Fermat Spiral Infill code is implemented according to the following articles:
 *  - "Connected Fermat Spirals for Layered Fabrication" by H. Zhao et al., SIGGRAPH '16
 */
namespace cura {

/*
 * This class helps to generate a fermat spiral infill for the given polygon objects.
 */
class FermatSpiralInfillGenerator {
public:
    void generateInfill(const Polygons& in_outline, Polygons& result_lines, const SliceMeshStorage* mesh);
};


/*
 * Data structure for the sprial-contour tree.
 * This class represents a node in that tree.
 */
struct SpiralContourNode
{
    uint32_t level;  // i - distance from the outmost boundary
    uint32_t index;  // j - the index number of the contour with the same distance
    ClipperLib::Path path;  // the polygon path of this contour
    struct SpiralContourNode * parent;  // parent node of this node
    int32_t type;   // connection type: I or II
    std::vector<struct SpiralContourNodeConnection *> to_child_connection_list;
    std::vector<struct SpiralContourNodeConnection *> to_parent_connection_list;
};


/*
 * Represents an edge from p1 (out) -> p2 (in).
 * Note that the edge is undirected. The outer and inner points are merely for better understanding.
 */
struct Edge
{
    ClipperLib::IntPoint p1;
    ClipperLib::IntPoint p2;
    double distance;
};


/*
 * Represents all edges between the parent node and the child node.
 * Note that the edge is undirected. 'parent' and 'child' are merely for better understanding.
 */
struct SpiralContourNodeConnection
{
    struct SpiralContourNode *parent_node;
    struct SpiralContourNode *child_node;
    int32_t weight;
    std::vector<struct Edge> edge_list;
};


/*
 * This class represents a sprial-contour tree. It takes a polygon object and generates a
 * minimum spanning tree (MST) based on 
 */
class SpiralContourTree
{
public:
    SpiralContourTree();
    ~SpiralContourTree();

    /*
     * Safely clears everything and releases all the allocated memories.
     */
    void clear();

    /*
     * Initialises the spiral-contour tree to process the given polygon paths.
     * Note that the given polygon paths must belong to a single object.
     */
    void setPolygons(const ClipperLib::Paths& paths);

    /*
     * Starts constructing a spiral-contour tree using all the added nodes and edges.
     * This function creates a minimum spanning tree (MST).
     */
    void constructTree();

    /*
     * Once the spiral-contour tree has been constructed, we can generate a continous path.
     */
    void generatePath();

private:
    /*
     * Adds the given SpiralContourNode into the tree node list and assigns
     * the given level number and an auto-generated index number for it.
     * 
     * param node  - The SpiralContourNode to be added.
     * param level - The level number this node belongs to.
     */
    void addNode(struct SpiralContourNode *node, uint32_t level);

    /*
     * Creates child nodes of the given parent node and all child nodes for the child nodes this function has created.
     */
    void createNodes(uint32_t current_level, const ClipperLib::Paths& paths);

    void updateNodeType(struct SpiralContourNode *node);

    void addConnectionEdge(struct SpiralContourNode *parent_node, struct SpiralContourNode *child_node, const struct Edge& edge);
    void computeConnectionEdges(struct SpiralContourNode *cij, struct SpiralContourNode *cip1j, struct SpiralContourNode *cip1k);

#ifndef NDEBUG
    /*
     * Prints the minimum spanning tree (MST).
     */
    void printMST(const struct SpiralContourNode *node, uint32_t level);
#endif // NDEBUG

private:
    std::vector<std::vector<struct SpiralContourNode *>> m_contour_node_list;  // indexed by [level, index]
    std::vector<struct SpiralContourNode *>           m_all_contour_node_list; // a flat list of all spiral contour nodes
    std::vector<struct SpiralContourNodeConnection *> m_all_node_connection_list;  // a flat list of all connections between all spiral contour nodes
    struct SpiralContourNode *m_tree_root;
};


} // namespace cura

#endif // CURA_FREMATSPIRAL_INFILL_H
