/* ================================================================================
 * Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, 
 *     University of Waterloo, All rights reserved.
 * 
 * Authors: 
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 * 
 * CopyPolicy: Released under the terms of the BSD 3-Clause License. 
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/Core/graph.h>

typedef std::vector< hriPhysio::Core::Edge >::iterator EdgeIter;
typedef std::pair<int, int> pii;

using namespace hriPhysio::Core;


Graph::Graph(int n) : num_nodes(n), num_edges(0) {

    nbr.reset(new std::vector< Edge >[num_nodes]);
    for (size_t idx = 0; idx < num_nodes; ++idx) {
        nbr[idx].clear();
    }
}


Graph::~Graph() {
    nbr.reset();
}


bool Graph::addEdge(int u, int v, double weight/*=1.0*/) {
    
    //-- Check to see if our u and v are inbounds?
    if (u >= num_nodes || u < 0 || v >= num_nodes || v < 0) { return false; }

    //-- Add an ``undirected`` edge between the vertices.
    nbr[u].push_back(Edge(v, weight));
    nbr[v].push_back(Edge(u, weight));
    ++num_edges;

    return true;
}


int Graph::getNumEdges() {
    return num_edges;
}


void Graph::shortestPath(int source, int target, std::string& message) {

    //-- Check to see if source and target are the same.
    if (source == target) {
        message = std::to_string(source) + '-' + std::to_string(target);
        return;
    }

    //-- Run dijkstras algorithm.
    dijkstra(source);

    //-- Recover the path from the previous 
    //-- nodes used to get to the target.
    std::vector<int> path; path.clear();

    int from = target;
    path.push_back(from);
    while(prev[from] != -1) {
        from = prev[from];
        path.push_back(from);
    }

    //-- Reverse this so constructing the string is easier.
    std::reverse(path.begin(), path.end());
    message = "";

    //-- If there was only one element in the path, 
    //-- then no such route exists.
    if (path.size() == 1) {
        return;
    }

    //-- Construct the string adding ``-`` between the nodes.
    for (size_t idx = 0; idx < path.size(); ++idx) {
        if (idx) { message += "-"; }           //-- i.e. not zero.
        message += std::to_string(path[idx]);  //-- append the node to the message.
    }
}


void Graph::dijkstra(int source) {

    //-- Keep track of which nodes have been used.
    std::vector< bool > used(num_nodes, false);

    //-- Use a p-queue to maintaine least-cost path.
    std::priority_queue< pii, std::vector<pii>, std::greater<pii> > fringe;

    //-- Flush some buffers we will use.
    dist.resize(num_nodes); 
    prev.resize(num_nodes); 

    std::fill(dist.begin(), dist.end(), -1);
    std::fill(prev.begin(), prev.end(), -1);


    //-- Init the first ``step``.
    dist[source] = 0;
    fringe.push( std::make_pair(dist[source], source) );


    //-- Take steps in the graph until a path is found!
    //-- If there is no path, we will exhaust all options.
    while (!fringe.empty()) {
        
        //-- Take the next best option.
        pii next = fringe.top(); fringe.pop();

        //-- What node are we using?
        int from = next.second;

        //-- If we have already tried to use this vertex, don't bother.
        if (used[from]) { continue; }

        //-- Don't try to revisit here in the future.
        used[from] = true;

        //-- Check each neighbor from u to v.
        for (EdgeIter iter = nbr[from].begin(); iter != nbr[from].end(); ++iter) {
            
            int to      = iter->to;
            int weight  = iter->weight + next.first;

            //-- If this has already been used, don't bother processing.
            if (used[to]) { continue; }

            //-- If this node has not been visited yet, or if 
            //-- we have found a better path to get here, update.
            if (dist[to] == -1 || weight < dist[to]) {

                dist[to] = weight;
                prev[to] = from;

                fringe.push( std::make_pair(dist[to], to) );
            }
        }
    }
}
