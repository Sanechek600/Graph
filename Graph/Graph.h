#pragma once

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <stack>
#include <set>

template<typename Vertex, typename Distance = double>
class Graph {
public:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance distance;
        Edge() : from(Vertex()), to(Vertex()), distance(Distance()) {}
        Edge(const Vertex& f, const Vertex& t, const Distance& d) : from(f), to(t), distance(d) {}
    };
private:
    std::unordered_map<Vertex, std::vector<Edge>> _graph;
public:
    bool has_vertex(const Vertex& v) const {
        return _graph.find(v) != _graph.end();
    }
    void add_vertex(const Vertex& v) {
        if (!has_vertex(v)) {
            _graph[v] = std::vector<Edge>();
        }
    }
    bool remove_vertex(const Vertex& v) {
        auto it = _graph.find(v);
        if (it != _graph.end()) {
            _graph.erase(it);
            for (auto& pair : _graph) {
                auto& edges = pair.second;
                edges.erase(std::remove_if(edges.begin(), edges.end(), [&v](const Edge& e) {
                    return e.to == v;
                    }), edges.end());
            }
            return true;
        }
        return false;
    }
    std::vector<Vertex> vertices() const {
        std::vector<Vertex> result;
        for (const auto& pair : _graph) {
            result.push_back(pair.first);
        }
        return result;
    }
    void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
        if (has_vertex(from) && has_vertex(to)) {
            _graph[from].push_back(Edge(from, to, d));
        }
    }
    bool remove_edge(const Vertex& from, const Vertex& to) {
        auto it = _graph.find(from);
        if (it != _graph.end()) {
            auto& edges = it->second;
            auto edge_it = std::remove_if(edges.begin(), edges.end(), [&to](const Edge& e) {return e.to == to; });
            if (edge_it != edges.end()) {
                edges.erase(edge_it, edges.end());
                return true;
            }
        }
        return false;
    }
    bool remove_edge(const Edge& e) {
        auto it = _graph.find(e.from);
        if (it != _graph.end()) {
            auto& edges = it->second;
            auto edge_it = std::remove_if(edges.begin(), edges.end(), [&e](const Edge& edge) {
                return edge.from == e.from && edge.to == e.to && edge.distance == e.distance;
                });
            if (edge_it != edges.end()) {
                edges.erase(edge_it, edges.end());
                return true;
            }
        }
        return false;
    }
    bool has_edge(const Vertex& from, const Vertex& to) const {
        auto it = _graph.find(from);
        if (it != _graph.end()) {
            const auto& edges = it->second;
            return std::any_of(edges.begin(), edges.end(), [&to](const Edge& e) {return e.to == to; });
        }
        return false;
    }
    bool has_edge(const Edge& e) const {
        auto it = _graph.find(e.from);
        if (it != _graph.end()) {
            const auto& edges = it->second;
            return std::any_of(edges.begin(), edges.end(), [&e](const Edge& edge) {
                return edge.from == e.from && edge.to == e.to && edge.distance == e.distance;
                });
        }
        return false;
    }
    std::vector<Edge> edges(const Vertex& vertex) {
        auto it = _graph.find(vertex);
        if (it != _graph.end()) {
            return it->second;
        }
        return std::vector<Edge>();
    }
    size_t order() const {
        return _graph.size();
    }
    size_t degree(const Vertex& v) const {
        auto it = _graph.find(v);
        if (it != _graph.end()) {
            return it->second.size();
        }
        return 0;
    }
    std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        if (!has_vertex(from) || !has_vertex(to))
            throw std::invalid_argument("Vertex does not exist in the graph");

        std::unordered_map<Vertex, Distance> distance;
        std::unordered_map<Vertex, Edge> predecessor;
        std::set<std::pair<Distance, Vertex>> pq;

        for (const Vertex& v : vertices()) {
            distance[v] = std::numeric_limits<Distance>::infinity();
            predecessor[v] = Edge();
        }

        distance[from] = Distance();
        pq.insert({ Distance(), from });

        while (!pq.empty()) {
            Vertex u = pq.begin()->second;
            pq.erase(pq.begin());

            for (const Edge& e : _graph.find(u)->second) {
                Vertex v = e.to;
                Distance weight = e.distance;
                if (distance[u] + weight < distance[v]) {
                    pq.erase({ distance[v], v });
                    distance[v] = distance[u] + weight;
                    predecessor[v] = e;
                    pq.insert({ distance[v], v });
                }
            }
        }

        std::vector<Edge> path;
        for (Vertex v = to; predecessor[v].from != Vertex(); v = predecessor[v].from) {
            path.push_back(predecessor[v]);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }



    std::vector<Vertex> walk(const Vertex& start_vertex) const {
        std::vector<Vertex> result;
        if (!has_vertex(start_vertex)) {
            return result;
        }
        std::unordered_map<Vertex, bool> visited;
        std::stack<Vertex> stack;

        visited[start_vertex] = true;
        stack.push(start_vertex);
        while (!stack.empty()) {
            Vertex current = stack.top();
            stack.pop();
            result.push_back(current);
            for (const auto& pair : _graph) {
                const auto& edges = pair.second;
                if (pair.first == current) {
                    for (const auto& edge : edges) {
                        if (!visited[edge.to]) {
                            visited[edge.to] = true;
                            stack.push(edge.to);
                        }
                    }
                    break;
                }
            }
        }
        return result;
    }
};
template<typename Vertex, typename Distance = double>
Vertex find_optimal_warehouse(const Graph<Vertex, Distance>& graph) {
    std::vector<Vertex> trade_points = graph.vertices();
    Vertex optimal_warehouse = Vertex();
    Distance min_longest_distance = std::numeric_limits<Distance>::max();
    for (const auto& trade_point : trade_points) {
        Distance longest_distance = std::numeric_limits<Distance>::min();
        for (const auto& destination : trade_points) {
            Distance comb_distance = Distance();
            if (trade_point != destination) {
                std::vector<typename Graph<Vertex, Distance>::Edge> shortest_paths = graph.shortest_path(trade_point, destination);
                for (const auto& edge : shortest_paths) {
                    comb_distance += edge.distance;
                }
            }
            if (longest_distance < comb_distance) {
                longest_distance = comb_distance;
            }
        }
        if (longest_distance < min_longest_distance) {
            min_longest_distance = longest_distance;
            optimal_warehouse = trade_point;
        }
    }
    return optimal_warehouse;
}