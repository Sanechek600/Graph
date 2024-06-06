#include "Graph.h"﻿

int main() {
	Graph<int, double> graph1;
	graph1.add_vertex(1);
	graph1.add_vertex(2);
	graph1.add_vertex(3);
	graph1.add_vertex(4);
	graph1.add_edge(1, 2, 10.0);
	graph1.add_edge(1, 3, 10.0);
	graph1.add_edge(1, 4, 10.0);
	graph1.add_edge(4, 3, 5.0);
	std::cout << "______________________________" << std::endl;
	std::cout << graph1.degree(1) << std::endl;
	for (auto i : graph1.edges(1)) {
		graph1.remove_edge(i);
	}
	std::cout << "______________________________" << std::endl;
	std::cout << graph1.degree(1) << std::endl;
	std::cout << "______________________________" << std::endl;
	auto vertices = graph1.vertices();
	for (auto i : vertices) {
		std::cout << i << std::endl;
	}
	std::cout << "______________________________" << std::endl;
	graph1.remove_vertex(1);
	auto vertices1 = graph1.vertices();
	for (auto i : vertices1) {
		std::cout << i << std::endl;
	}
	std::cout << "______________________________" << std::endl;
	std::cout << graph1.order() << std::endl;
	std::cout << "______________________________" << std::endl;
	std::cout << graph1.degree(2) << std::endl;
	std::cout << graph1.degree(3) << std::endl;
	std::cout << graph1.degree(4) << std::endl;
	std::cout << "______________________________" << std::endl;
	graph1.add_edge(2, 4, 5);
	graph1.add_edge(4, 3, 6);
	auto walk = graph1.walk(2);
	for (auto i : walk) {
		std::cout << i << std::endl;
	}
	std::cout << "______________________________" << std::endl;
	graph1.add_vertex(5);
	graph1.add_vertex(6);
	graph1.add_edge(3, 2, 11);
	graph1.add_edge(2, 3, 10);
	graph1.add_edge(3, 5, 100);
	graph1.add_edge(3, 6, 6);
	graph1.add_edge(5, 6, 152);
	graph1.add_edge(5, 2, 7);
	graph1.add_edge(5, 3, 10);
	graph1.add_edge(6, 2, 44);
	graph1.add_edge(6, 3, 33);
	graph1.add_edge(6, 4, 18);
	graph1.add_edge(6, 5, 1);
	std::cout << "  2\t3\t4\t5\t6" << std::endl;
	for (auto i : graph1.vertices()) {
		std::cout << i << ": ";
		for (auto j : graph1.vertices()) {
			auto path = graph1.shortest_max_path(i, j);
			auto distance = 0;
			for (auto& d : path) {
				distance += d.distance;
			}
			if (distance <= -100000) std::cout << "inf" << "\t";
			else std::cout << distance << "\t";
		}
		std::cout << std::endl;
	}
	std::cout << "Best warehouse: " << find_optimal_warehouse(graph1) << std::endl;
}