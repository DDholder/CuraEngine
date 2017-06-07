/*
 * Copyright (c) 2017 Ultimaker B.V.
 *
 * CuraEngine is released under the terms of the AGPLv3 or higher.
 */
#include <cassert>
#include <cmath>
#include <cstdlib>

#include <algorithm>
#include <vector>

#ifndef NDEBUG
# include <iostream>
#endif // NDEBUG

#include "FermatSpiral.h"

using namespace cura;


/*
 * Calculates the distance between the two given points (with integer coordinates)
 */
static inline double p2p_dist(const ClipperLib::IntPoint& p1, const ClipperLib::IntPoint& p2)
{
    return std::sqrt((p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y));
}


void FermatSpiralInfillGenerator::generateInfill(const Polygons& in_outline, Polygons& result_lines, const SliceMeshStorage* mesh)
{
#ifndef NDEBUG
    std::cout << "original in outline = " << in_outline.size() << std::endl;
#endif // NDEBUG

#ifndef NDEBUG
    for (uint32_t i = 0; i < in_outline.size(); ++i)
    {
        for (uint32_t j = 0; j < in_outline[i].size(); ++j)
        {
            std::cout << "---- " << in_outline[i][j] << std::endl;
        }
    }
#endif // NDEBUG

    // devide the whole thing into individual (non-intersecting) polygon parts
    std::vector<PolygonsPart> individual_polygon_part_list = in_outline.splitIntoParts();

#ifndef NDEBUG
    std::cout << "individual polygon parts = " << individual_polygon_part_list.size() << std::endl;
    for (uint32_t i = 0; i < individual_polygon_part_list.size(); ++i) // each part
    {
        for (uint32_t j = 0; j < individual_polygon_part_list[i].size(); ++j) // each path
        {
            for (uint32_t k = 0; k < individual_polygon_part_list[i][j].size(); ++k) // each point
            {
                std::cout << "---- " << individual_polygon_part_list[i][j][k] << std::endl;
            }
        }
    }
#endif // NDEBUG

    std::vector<ClipperLib::Paths> individual_paths_list;
    std::vector<SpiralContourTree *> tree_list;
    for (auto itr_part = individual_polygon_part_list.begin(); itr_part != individual_polygon_part_list.end(); ++itr_part)
    {
        ClipperLib::Paths this_part_paths = (*itr_part).getPaths();
        
        individual_paths_list.push_back(this_part_paths);
        SpiralContourTree *tree = new SpiralContourTree();
        tree_list.push_back(tree);

#ifndef NDEBUG
        std::cout << "handling individual part ..." << std::endl;
#endif // NDEBUG
        tree->setPolygons(this_part_paths);
        tree->constructTree();
    }
}


SpiralContourTree::SpiralContourTree()
    : m_tree_root(nullptr)
{
    this->m_contour_node_list.clear();
    this->m_all_node_connection_list.clear();
}


SpiralContourTree::~SpiralContourTree()
{
    // safely clear everything
    clear();
}

/*
 * Safely clears everything and releases all the allocated memories.
 */
void SpiralContourTree::clear()
{
    // TODO: safely clear everything
    m_tree_root = nullptr;

    // clear all connection nodes
    for (uint32_t i = 0; i < m_all_node_connection_list.size(); ++i)
    {
        delete m_all_node_connection_list[i];
    }
    m_all_node_connection_list.clear();

    // clear all tree nodes
    for (uint32_t i = 0; i < m_all_contour_node_list.size(); ++i)
    {
        delete m_all_contour_node_list[i];
    }
    m_all_contour_node_list.clear();

    m_all_node_connection_list.clear();
}


void SpiralContourTree::addConnectionEdge(struct SpiralContourNode *parent_node, struct SpiralContourNode *child_node, const struct Edge& edge)
{
    struct SpiralContourNodeConnection *connection = nullptr;

    // find an existing connection object
    for (auto itr_conn = m_all_node_connection_list.begin(); itr_conn != m_all_node_connection_list.end(); ++itr_conn)
    {
        // TODO: optimise
        if ((*itr_conn)->parent_node->level != parent_node->level or (*itr_conn)->parent_node->index != parent_node->index)
        {
            continue;
        }
        if ((*itr_conn)->child_node->level != child_node->level or (*itr_conn)->child_node->index != child_node->index)
        {
            continue;
        }

        connection = *itr_conn;
        break;
    }

    // create a new connection object if no existing can be found
    if (connection == nullptr)
    {
#ifndef NDEBUG
        std::cout << "cannot find connection between "
            << "[" << parent_node->level << "," << parent_node->index << "]" << " --- "
            << "[" << child_node->level << "," << child_node->index << "]"
            << std::endl;
#endif // NDEBUG
        connection = new struct SpiralContourNodeConnection;
        connection->parent_node = parent_node;
        connection->child_node = child_node;
        connection->edge_list = std::vector<Edge>();
        this->m_all_node_connection_list.push_back(connection);
    }

    connection->edge_list.push_back(edge);
    connection->weight = connection->edge_list.size();  // weight is the number of edges
}


void SpiralContourTree::addNode(struct SpiralContourNode *node, uint32_t level)
{
    // make sure that the node list for this level exists
    while (level >= m_contour_node_list.size())
    {
        m_contour_node_list.push_back(std::vector<struct SpiralContourNode *>());
    }

    // assign level number and node id and add to list
    node->level = level;
    node->index = m_contour_node_list[level].size();

    this->m_contour_node_list[level].push_back(node);
    this->m_all_contour_node_list.push_back(node);
}


struct ConnectionSorter
{
    inline bool operator() (const struct SpiralContourNodeConnection * conn1, const struct SpiralContourNodeConnection * conn2)
    {
        return (conn1->weight < conn2->weight);
    }
};


void SpiralContourTree::setPolygons(const ClipperLib::Paths& paths)
{
    if (paths.size() == 0)
    {
        return;
    }

    // the first path is the outline, so we can directly feed the whole paths to the
    // handling function and it will generate spiral contour c[0,0] for the outline
    // path automatically.
    struct SpiralContourNode *root_node = new struct SpiralContourNode;
    memset(root_node, 0, sizeof(struct SpiralContourNode));

    createNodes(0, paths);

    // set root node
    this->m_tree_root = this->m_contour_node_list[0][0];
}

/*
 * Creates child nodes of the given parent node and all child nodes for the child nodes this function has created.
 */
void SpiralContourTree::createNodes(uint32_t current_level, const ClipperLib::Paths& paths)
{
    for (auto itr_path = paths.begin(); itr_path != paths.end(); ++itr_path)
    {
        // create a child node for each Polygon Path and continue
        struct SpiralContourNode *child_node = new struct SpiralContourNode;
        memset(child_node, 0, sizeof(struct SpiralContourNode));

        // create the child node
        child_node->path = *itr_path;
        child_node->parent = nullptr;
        child_node->to_child_connection_list = std::vector<struct SpiralContourNodeConnection *>();
        child_node->to_parent_connection_list = std::vector<struct SpiralContourNodeConnection *>();

        this->addNode(child_node, current_level);
#ifndef NDEBUG
        std::cout << "added child node [" << current_level << "," << child_node->index << "]" << std::endl;
#endif // NDEBUG

        // create child nodes for this node (if any)
        ClipperLib::Paths child_node_paths;

        ClipperLib::ClipperOffset clipper(1.2, 10.0);  // TODO: make this configurable
        clipper.AddPath(*itr_path, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        clipper.MiterLimit = 1.2;
        clipper.Execute(child_node_paths, -7.0);

        // create spiral contour tree nodes for this child node
        createNodes(current_level + 1, child_node_paths);
    }
}


void SpiralContourTree::constructTree()
{
    if (m_contour_node_list.size() == 0)
    {
#ifndef NDEBUG
        std::cout << "no contour node to process." << std::endl;
#endif // NDEBUG
        return;
    }

    // make sure we only have 1 root node (c_i,j)
    assert(m_contour_node_list[0].size() == 1);

#ifndef NDEBUG
    std::cout << "nodes on each level:" << std::endl;
    for (uint32_t i = 0; i < m_contour_node_list.size(); ++i)
    {
        std::cout << " - " << i << " : " << m_contour_node_list[i].size() << std::endl;
    }
#endif // NDEBUG

    // generate connections between nodes
    uint32_t i;
    uint32_t j;
    uint32_t jp;
    uint32_t k;
    for (i = 0; i < m_contour_node_list.size(); ++i)
    {
#ifndef NDEBUG
        std::cout << "level: " << i << std::endl;
#endif // NDEBUG
        // if there is no next level, do nothing
        if (i + 1 >= m_contour_node_list.size())
        {
#ifndef NDEBUG
            std::cout << "> skipping level " << i << " because level " << i+1 << " is empty." << std::endl;
#endif // NDEBUG
            break;
        }

        for (j = 0; j < m_contour_node_list[i].size(); ++j)
        {
#ifndef NDEBUG
            std::cout << "> handling c[i,j] = " << i << ", " << j << std::endl;
#endif // NDEBUG
            struct SpiralContourNode *c_ij = m_contour_node_list[i][j];

            // if there is only one contour, then there will only be one connection
            if (m_contour_node_list[i + 1].size() == 1)
            {
#ifndef NDEBUG
                std::cout << "> level " << i+1 << " only have 1 node, directly calculating "
                    << "[" << i << "," << j << "]" << " ---- "
                    << "[" << i+1 << "," << 0 << "]"
                    << std::endl;
#endif // NDEBUG
                struct SpiralContourNode *c_ip1_jp = m_contour_node_list[i + 1][0];
                computeConnectionEdges(c_ij, c_ip1_jp, nullptr);
                continue;
            }

            // get c[i+1,j'] and c[i+1,k]
            for (jp = 0; jp < m_contour_node_list[i + 1].size(); ++jp)
            {
                struct SpiralContourNode *c_ip1_jp = m_contour_node_list[i + 1][jp];
                for (k = 0; k < m_contour_node_list[i + 1].size(); ++k)
                {
                    if (k == jp)
                    {
                        continue;
                    }
                    struct SpiralContourNode *c_ip1_k = m_contour_node_list[i + 1][k];
                    computeConnectionEdges(c_ij, c_ip1_jp, c_ip1_k);
                }
            }
        }
    }

#ifndef NDEBUG
    std::cout << ">>>>>>>>> calculated connections:" << std::endl;
    for (uint32_t i = 0; i < m_all_node_connection_list.size(); ++i)
    {
        std::cout << " - "
            << "[" << m_all_node_connection_list[i]->parent_node->level << ", " << m_all_node_connection_list[i]->parent_node->index << "]"
            << " ---- "
            << "[" << m_all_node_connection_list[i]->child_node->level << ", " << m_all_node_connection_list[i]->child_node->index << "]"
            << " --> " << m_all_node_connection_list[i]->weight
            << std::endl;
    }
#endif // NDEBUG

    // sort the connections based on weight
    std::vector<struct SpiralContourNodeConnection *> sorted_connection_list = m_all_node_connection_list;
    std::sort(sorted_connection_list.begin(), sorted_connection_list.end(), ConnectionSorter());

#ifndef NDEBUG
    std::cout << ">>>>>>>>> sorted connections:" << std::endl;
    for (uint32_t i = 0; i < sorted_connection_list.size(); ++i)
    {
        std::cout << " - "
            << "[" << sorted_connection_list[i]->parent_node->level << ", " << sorted_connection_list[i]->parent_node->index << "]"
            << " ---- "
            << "[" << sorted_connection_list[i]->child_node->level << ", " << sorted_connection_list[i]->child_node->index << "]"
            << " --> " << sorted_connection_list[i]->weight
            << std::endl;
    }
#endif // NDEBUG

    // create a minimum spanning tree (MST)
    std::vector<struct SpiralContourNode *> already_connected_node_list;
    already_connected_node_list.reserve(this->m_all_contour_node_list.size());
    uint32_t created_connection_count = 0;

#ifndef NDEBUG
    std::cout << ">>>>>>>>> creating MST ..." << std::endl;
    i = 0;
#endif // NDEBUG
    for (auto itr_conn = sorted_connection_list.begin(); itr_conn != sorted_connection_list.end(); ++itr_conn)
    {
#ifndef NDEBUG
            std::cout << "> handling connection " << i++ << std::endl;
#endif // NDEBUG
        // if all nodes have been connected, no need to continue
        if (created_connection_count == this->m_all_contour_node_list.size() - 1)
        {
#ifndef NDEBUG
            std::cout << "established connections have reached " << created_connection_count << ", stopping ..." << std::endl;
#endif // NDEBUG
            break;
        }

        struct SpiralContourNode *parent_node = (*itr_conn)->parent_node;
        struct SpiralContourNode *child_node = (*itr_conn)->child_node;

        // make sure we don't create a cyclic link
        bool found_parent = false;
        bool found_child = false;
        for (auto itr_connected_node = already_connected_node_list.begin(); itr_connected_node != already_connected_node_list.end(); ++itr_connected_node)
        {
            if ((*itr_connected_node)->level == parent_node->level and (*itr_connected_node)->index == parent_node->index)
            {
                found_parent = true;
            }
            else if ((*itr_connected_node)->level == child_node->level and (*itr_connected_node)->index == child_node->index)
            {
                found_child = true;
            }
            if (found_parent and found_child)
            {
                break;
            }
        }
        // make sure we don't create a cyclic link
        if (found_parent and found_child and child_node->parent != nullptr)
        {
#ifndef NDEBUG
        std::cout << " skip connection: "
            << "[" << (*itr_conn)->parent_node->level << "," << (*itr_conn)->parent_node->index << "]" << " --> "
            << "[" << (*itr_conn)->child_node->level << "," << (*itr_conn)->child_node->index << "]"
            << std::endl;
#endif // NDEBUG
            continue;
        }

        // set this connection
        parent_node->to_child_connection_list.push_back(*itr_conn);
        this->updateNodeType(parent_node);
        child_node->to_parent_connection_list.push_back(*itr_conn);
        this->updateNodeType(child_node);
        child_node->parent = parent_node;
        ++created_connection_count;

#ifndef NDEBUG
        std::cout << " connecting: "
            << "[" << (*itr_conn)->parent_node->level << "," << (*itr_conn)->parent_node->index << "]" << " --> "
            << "[" << (*itr_conn)->child_node->level << "," << (*itr_conn)->child_node->index << "]"
            << std::endl;
#endif // NDEBUG

        if (!found_parent)
        {
            already_connected_node_list.push_back(parent_node);
        }
        if (!found_child)
        {
            already_connected_node_list.push_back(child_node);
        }
    }
    
#ifndef NDEBUG
    std::cout << ">>>>>>>>> MST connections:" << std::endl;
    printMST(this->m_tree_root, 0);
#endif // NDEBUG
}

void SpiralContourTree::generatePath()
{
    // TODO: implement
}

void SpiralContourTree::updateNodeType(struct SpiralContourNode *node)
{
    uint32_t connection_count = node->to_parent_connection_list.size() + node->to_child_connection_list.size();
    uint32_t type = connection_count <= 2 ? 1 : 2;
    node->type = type;
}

#ifndef NDEBUG
/*
 * Prints the minimum spanning tree (MST).
 */
void SpiralContourTree::printMST(const struct SpiralContourNode *node, uint32_t level)
{
    if (node == nullptr)
    {
        return;
    }

    for (uint32_t i = 0; i <= level; ++i)
    {
        std::cout << "--";
    }

    std::cout << "[" << node->level << "," << node->index << "] t-" << node->type << std::endl;

    for (auto itr_conn = node->to_child_connection_list.begin(); itr_conn != node->to_child_connection_list.end(); ++itr_conn)
    {
        struct SpiralContourNodeConnection* connection = *itr_conn;
        printMST(connection->child_node, level + 1);
    }
}
#endif // NDEBUG


void SpiralContourTree::computeConnectionEdges(
    struct SpiralContourNode *cij,
    struct SpiralContourNode *cip1j,
    struct SpiralContourNode *cip1k)
{
    // take points on this node and compute distance towards the other
    for (auto itr_pt_cij = cij->path.begin(); itr_pt_cij != cij->path.end(); ++itr_pt_cij)
    {
        for (auto itr_pt_cip1j = cip1j->path.begin(); itr_pt_cip1j != cip1j->path.end(); ++itr_pt_cip1j)
        {
            const double dj_prime = p2p_dist(*itr_pt_cij, *itr_pt_cip1j);
            bool is_dj_prime_smallest = true;

            // some levels may only have 1 contour, so there won't be any left to compare with
            if (cip1k != nullptr)
            {
                for (auto itr_pt_cip1k = cip1k->path.begin(); itr_pt_cip1k != cip1k->path.end(); ++itr_pt_cip1k)
                {
                    const double dk_prime = p2p_dist(*itr_pt_cij, *itr_pt_cip1k);
                    if (dj_prime >= dk_prime) {
                        is_dj_prime_smallest = false;
                        break;
                    }
                }
            }

            if (is_dj_prime_smallest) {
                // create this edge
                struct Edge edge;
                edge.p1 = *itr_pt_cij;
                edge.p2 = *itr_pt_cip1j;
                edge.distance = dj_prime;

                // add edge to the tree
                this->addConnectionEdge(cij, cip1j, edge);
            }
        }
    }
}
