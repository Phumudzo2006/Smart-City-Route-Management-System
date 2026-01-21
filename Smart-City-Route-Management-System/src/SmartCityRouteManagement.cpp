
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <string>
#include <limits>
#include <iomanip>
#include <ctime>

using namespace std;

//Structure to represent an edge between two intersections in the city network
struct Edge
{
    int destination;
    double distance;
    double travelTime;
    double cost;
    string routeName;
    Edge(int dest, double dist, double time, double c, string name)
        : destination(dest), distance(dist), travelTime(time), cost(c), routeName(name) {}
};

//Stores results of shortest path calculation
struct PathResult
{
    vector<int> path;
    double totalDistance;
    double totalTime;
    double totalCost;
    vector<string> explanations;
    bool pathFound;
    PathResult() : totalDistance(0), totalTime(0), totalCost(0), pathFound(false) {}
};

//NodeDistance used in priority queue for Dijkstra's algorithm
struct NodeDistance
{
    int node;
    double distance;

    NodeDistance(int n, double d) : node(n), distance(d) {}


    bool operator>(const NodeDistance& other) const
    {
        return distance > other.distance;
    }
};

//The main class for the Route Management System

class RouteManagementSystem {
private:
    map<int, vector<Edge>> graph;
    map<int, string> nodeNames;
    double predictCongestionFactor(int hour)
    {
        if ((hour >= 7 && hour <= 9) || (hour >= 17 && hour <= 19))
            {
                return 1.5;
            }
        return 1.0;
    }

public:
    //Add a node(intersection) to the network
    void addNode(int nodeId, string name = "")
    {
        if (graph.find(nodeId) == graph.end())
            {
                graph[nodeId] = vector<Edge>();
                nodeNames[nodeId] = name.empty() ? "Node_" + to_string(nodeId) : name;
            }
    }
    //Add a route (edge) between two nodes
    void addRoute(int source, int destination, double distance,
                  double travelTime, double cost, string routeName) {
        addNode(source);
        addNode(destination);
        graph[source].push_back(Edge(destination, distance, travelTime, cost, routeName));
        graph[destination].push_back(Edge(source, distance, travelTime, cost, routeName));

        cout << "* Route added: " << nodeNames[source] << " - " << nodeNames[destination]
             << " (" << distance << " km, " << travelTime << " min, $" << cost << ")" << endl;
    }

    //Remove route between two nodes
    bool removeRoute(int source, int destination)
    {
        bool removed = false;
        if (graph.find(source) != graph.end()) //Remove from source list
            {
                auto& edges = graph[source];
                auto it = remove_if(edges.begin(), edges.end(),
                    [destination](const Edge& e) { return e.destination == destination; });
                if (it != edges.end())
                {
                    edges.erase(it, edges.end());
                    removed = true;
                }
        }
        if (graph.find(destination) != graph.end()) //Remove from destination list
        {
            auto& edges = graph[destination];
            auto it = remove_if(edges.begin(), edges.end(),
                [source](const Edge& e) { return e.destination == source; });
            if (it != edges.end())
            {
                edges.erase(it, edges.end());
                removed = true;
            }
        }

        if (removed) //the final Displayed message
            {
                cout << "~ Route removed: " << nodeNames[source] << " ↔ " << nodeNames[destination] << endl;
            } else {
                cout << "!!! Route not found between nodes " << source << " and " << destination << endl;
                }

        return removed;
    }

    //update an existing route's properties
    bool updateRoute(int source, int destination, double newDistance,
                     double newTravelTime, double newCost) {
        bool updated = false;

        //update for both directions
        if (graph.find(source) != graph.end())
            {
                for (auto& edge : graph[source])
                {
                    if (edge.destination == destination)
                    {
                        edge.distance = newDistance;
                        edge.travelTime = newTravelTime;
                        edge.cost = newCost;
                        updated = true;
                        break;
                    }
                }
            }
        if (graph.find(destination) != graph.end())
        {
            for (auto& edge : graph[destination])
            {
                if (edge.destination == source)
                {
                    edge.distance = newDistance;
                    edge.travelTime = newTravelTime;
                    edge.cost = newCost;
                    updated = true;
                    break;
                }
            }
        }

        if (updated)
            {
                cout << "+ Route updated: " << nodeNames[source] << " - " << nodeNames[destination] << endl;
            } else {
                cout << "!!! Route not found for update" << endl;
            }

        return updated;
    }

    //display all routes in the system
    void displayAllRoutes()
    {
        cout << "\n" << string(80, '=') << endl;
        cout << "ALL ROUTES IN THE NETWORK" << endl;
        cout << string(80, '=') << endl;

        if (graph.empty())
            {
                cout << "No routes in the system." << endl;
                return;
            }
        set<pair<int, int>> displayed;

        for (const auto& [source, edges] : graph)
            {
                for (const auto& edge : edges)
                {
                    if (displayed.find({edge.destination, source}) == displayed.end())
                    {
                        cout << left << setw(20) << (nodeNames[source] + " - " + nodeNames[edge.destination])

                        << " | Distance: " << setw(6) << edge.distance << " km"
                        << " | Time: " << setw(6) << edge.travelTime << " min"
                        << " | Cost: $" << setw(6) << edge.cost
                        << " | Route: " << edge.routeName << endl;
                        displayed.insert({source, edge.destination});
                    }
                }
            }
        cout << string(80, '=') << endl;
    }

    //Get all edges from the graph for sorting/searching
    vector<Edge> getAllEdges()
    {
        vector<Edge> allEdges;
        set<pair<int, int>> added;

        for (const auto& [source, edges] : graph)
            {
                for (const auto& edge : edges)
                {
                    if (added.find({edge.destination, source}) == added.end())
                    {
                        allEdges.push_back(edge);
                        added.insert({source, edge.destination});
                    }
                }
            }
            return allEdges;
    }

    //Sort routes based on selected criterion(distance, time, or cost)
    void sortRoutes(string criterion)
    {
        vector<Edge> edges = getAllEdges();
        if (criterion == "distance")
            {
                sort(edges.begin(), edges.end(),
                    [](const Edge& a, const Edge& b) { return a.distance < b.distance; });
                cout << "\n* Routes sorted by DISTANCE:" << endl;
            } else if (criterion == "time") {
                sort(edges.begin(), edges.end(),
                    [](const Edge& a, const Edge& b) { return a.travelTime < b.travelTime; });
                cout << "\n* Routes sorted by TRAVEL TIME:" << endl;
            } else if (criterion == "cost") {
                sort(edges.begin(), edges.end(),
                    [](const Edge& a, const Edge& b) { return a.cost < b.cost; });
                cout << "\n* Routes sorted by COST:" << endl;
            } else {
                cout << "Invalid criterion. Use 'distance', 'time', or 'cost'." << endl;
                return;
            }

        //Display sorted results
        cout << string(80, '-') << endl;
        for (const auto& edge : edges)
            {
                cout << "Distance: " << setw(6) << edge.distance << " km | "
                << "Time: " << setw(6) << edge.travelTime << " min | "
                << "Cost: $" << setw(6) << edge.cost << " | "
                << "Route: " << edge.routeName << endl;
            }
        cout << string(80, '-') << endl;
    }

    //search for routes by keyword
    void searchRoute(string keyword)
    {
        vector<Edge> edges = getAllEdges();
        vector<Edge> results;
        for (const auto& edge : edges)
            {
                string lowerRouteName = edge.routeName;
                string lowerKeyword = keyword;
                transform(lowerRouteName.begin(), lowerRouteName.end(), lowerRouteName.begin(), ::tolower);
                transform(lowerKeyword.begin(), lowerKeyword.end(), lowerKeyword.begin(), ::tolower);

            if (lowerRouteName.find(lowerKeyword) != string::npos)
                {
                    results.push_back(edge);
                }
            }

        //Display search results
        if (results.empty())
            {
                cout << "No routes found matching '" << keyword << "'" << endl;
            } else {
                cout << "\n* Found " << results.size() << " route(s) matching '" << keyword << "':" << endl;
                cout << string(80, '-') << endl;
                for (const auto& edge : results)
                {
                    cout << "Route: " << edge.routeName
                     << " | Distance: " << edge.distance << " km"
                     << " | Time: " << edge.travelTime << " min"
                     << " | Cost: $" << edge.cost << endl;
                }
            cout << string(80, '-') << endl;
        }
    }

    //Dijkstra's Algorithm for finding shortest path
    // XAI:this implementation uses a priority queue (min-heap) to efficiently
    PathResult dijkstraShortestPath(int source, int destination, string criterion = "distance", bool applyAI = false)
    {
        PathResult result;

        // XAI: Check if source and destination nodes exist in the graph
        // This validation prevents errors and provides clear feedback
        if (graph.find(source) == graph.end())
            {
                result.explanations.push_back("ERROR: Source node " + to_string(source) + " does not exist in the network.");
                return result;
            }
        if (graph.find(destination) == graph.end())
        {
                result.explanations.push_back("ERROR: Destination node " + to_string(destination) + " does not exist in the network.");
                return result;
            }

        // XAI: Initialize data structures for Dijkstra's algorithm
        // - distances: stores the shortest known distance to each node
        // - previous: stores the previous node in the optimal path
        // - pq: priority queue for selecting the next node to explore
        map<int, double> distances;
        map<int, int> previous;
        priority_queue<NodeDistance, vector<NodeDistance>, greater<NodeDistance>> pq;
        for (const auto& [nodeId, _] : graph)
            {
                distances[nodeId] = numeric_limits<double>::infinity();
            }

        // XAI: Set source distance to 0 (starting point)
        // All other nodes start at infinity (unreachable)
        distances[source] = 0;
        pq.push(NodeDistance(source, 0));

        result.explanations.push_back("=== DIJKSTRA'S ALGORITHM EXECUTION ===");
        result.explanations.push_back("Algorithm: Dijkstra's shortest path algorithm");
        result.explanations.push_back("Optimization criterion: " + criterion);
        result.explanations.push_back("Source: " + nodeNames[source] + " (Node " + to_string(source) + ")");
        result.explanations.push_back("Destination: " + nodeNames[destination] + " (Node " + to_string(destination) + ")");

        // AI Integration
        double congestionFactor = 1.0;
        if (applyAI) {
            time_t now = time(0);
            tm* ltm = localtime(&now);
            int currentHour = ltm->tm_hour;
            congestionFactor = predictCongestionFactor(currentHour);

            result.explanations.push_back("\n--- AI CONGESTION PREDICTION ---");
            result.explanations.push_back("Current time: " + to_string(currentHour) + ":00");
            result.explanations.push_back("Congestion factor: " + to_string(congestionFactor) + "x");
            if (congestionFactor > 1.0) {
                result.explanations.push_back("! Peak hour detected! Travel times increased by " + to_string((int)((congestionFactor - 1.0) * 100)) + "%");
            } else {
                result.explanations.push_back("* Normal traffic conditions");
            }
        }

        result.explanations.push_back("\n--- ALGORITHM STEPS ---");

        int stepCount = 0;

        // Dijkstra main loop// XAI: Main Dijkstra loop - process nodes in order of increasing distance
        // This greedy approach guarantees finding the optimal path
        while (!pq.empty())
            {
                NodeDistance current = pq.top();
                pq.pop();

                int currentNode = current.node;
                double currentDistance = current.distance;

                // XAI: Skip if we've already found a better path to this node
                // This optimization prevents redundant processing
                if (currentDistance > distances[currentNode])
                {
                    continue;
                }

            stepCount++;
            result.explanations.push_back("\nStep " + to_string(stepCount) + ": Exploring node " +
                                        nodeNames[currentNode] + " (current " + criterion + ": " +
                                        to_string(currentDistance) + ")");

            // XAI: Early termination - if we reached the destination, we're done
            // Dijkstra guarantees this is the optimal path
            if (currentNode == destination)
                {
                result.explanations.push_back("* Destination reached! Optimal path found.");
                break;
                }

            // XAI: Explore all neighbors of the current node
            // This is the "relaxation" step in Dijkstra's algorithm
            for (const Edge& edge : graph[currentNode])
            {
                int neighbor = edge.destination;
                double edgeWeight;
                if (criterion == "distance")
                {
                    edgeWeight = edge.distance;
                } else if (criterion == "time") {
                    edgeWeight = edge.travelTime * congestionFactor;  // Apply AI congestion
                } else if (criterion == "cost") {
                    edgeWeight = edge.cost;
                } else {
                    edgeWeight = edge.distance;  // Default to distance
                }

                double newDistance = currentDistance + edgeWeight;

                // XAI: If we found a shorter path to the neighbor, update it
                // This is the core of the greedy optimization
                if (newDistance < distances[neighbor])
                    {
                        double oldDistance = distances[neighbor];
                        distances[neighbor] = newDistance;
                        previous[neighbor] = currentNode;
                        pq.push(NodeDistance(neighbor, newDistance));
                        if (oldDistance == numeric_limits<double>::infinity())
                        {
                            result.explanations.push_back("  -->Found path to " + nodeNames[neighbor] +
                                                    " via " + edge.routeName +
                                                    " (" + criterion + ": " + to_string(edgeWeight) +
                                                    ", total: " + to_string(newDistance) + ")");
                        } else {
                            result.explanations.push_back("  --> Better path to " + nodeNames[neighbor] +
                                                    " via " + edge.routeName +
                                                    " (new: " + to_string(newDistance) +
                                                    " < old: " + to_string(oldDistance) + ")");
                                }
                    }
            }
        }
        if (distances[destination] == numeric_limits<double>::infinity())
            {
                result.explanations.push_back("\n!!! NO PATH FOUND from " + nodeNames[source] +
                                        " to " + nodeNames[destination]);
                result.pathFound = false;
                return result;
            }

            result.explanations.push_back("\n--- PATH RECONSTRUCTION ---");

            vector<int> path;
            int current = destination;
            while (current != source)
                {
                path.push_back(current);
                current = previous[current];
            }
            path.push_back(source);
            reverse(path.begin(), path.end());

            result.path = path;
            result.pathFound = true;

        // XAI: Calculate total metrics for the path
        result.totalDistance = 0;
        result.totalTime = 0;
        result.totalCost = 0;

        result.explanations.push_back("Optimal path: ");
        for (size_t i = 0; i < path.size() - 1; i++)
            {
            int from = path[i];
            int to = path[i + 1];

            // Find the edge between these nodes
            for (const Edge& edge : graph[from])
                {
                    if (edge.destination == to)
                    {
                        result.totalDistance += edge.distance;
                        result.totalTime += edge.travelTime * (applyAI ? congestionFactor : 1.0);
                        result.totalCost += edge.cost;

                        result.explanations.push_back("  " + to_string(i + 1) + ". " +
                                                nodeNames[from] + " → " + nodeNames[to] +
                                                " via " + edge.routeName +
                                                " (distance: " + to_string(edge.distance) + " km, " +
                                                "time: " + to_string(edge.travelTime) + " min, " +
                                                "cost: $" + to_string(edge.cost) + ")");
                        break;
                    }
                }
            }

        result.explanations.push_back("\n--- SUMMARY ---");
        result.explanations.push_back("Total distance: " + to_string(result.totalDistance) + " km");
        result.explanations.push_back("Total time: " + to_string(result.totalTime) + " minutes");
        result.explanations.push_back("Total cost: $" + to_string(result.totalCost));
        result.explanations.push_back("Number of segments: " + to_string(path.size() - 1));
        result.explanations.push_back("\nWhy this path is optimal:");
        result.explanations.push_back("Dijkstra's algorithm guarantees the shortest path by always");
        result.explanations.push_back("exploring the most promising node next (greedy choice property)");
        result.explanations.push_back("and building optimal solutions from optimal sub-solutions");
        result.explanations.push_back("(optimal substructure property).");

        return result;
    }

    //Display detailed path result
    void displayPathResult(const PathResult& result)
    {
        cout << "\n" << string(80, '=') << endl;
        cout << "SHORTEST PATH RESULT WITH XAI EXPLANATIONS" << endl;
        cout << string(80, '=') << endl;

        for (const string& explanation : result.explanations)
            {
                cout << explanation << endl;
            }

        cout << string(80, '=') << endl;
    }

   // Utility functions
    string getNodeName(int nodeId)
    {
        return nodeNames.count(nodeId) ? nodeNames[nodeId] : "Node_" + to_string(nodeId);
    }


    bool hasNodes()
    {
        return !graph.empty();
    }


    vector<int> getAllNodeIds()
    {
        vector<int> nodeIds;
        for (const auto& [nodeId, _] : graph)
            {
                nodeIds.push_back(nodeId);
            }
        return nodeIds;
    }
};

// Main function with menu interface

int main() {
    RouteManagementSystem system;

  cout << "\nSMART CITY ROUTE MANAGEMENT SYSTEM\n";
    cout << "---------------------------------\n";
    cout << "  "<<endl;
    cout << "Initializing sample city network..." << endl;


    system.addNode(1, "Downtown");
    system.addNode(2, "Airport");
    system.addNode(3, "University");
    system.addNode(4, "Harbor");
    system.addNode(5, "Stadium");

    system.addRoute(1, 2, 15.5, 25, 12.50, "Highway_A1");
    system.addRoute(1, 3, 8.2, 15, 6.00, "Main_Street");
    system.addRoute(2, 3, 22.0, 35, 18.00, "Airport_Road");
    system.addRoute(3, 4, 12.5, 20, 9.50, "Coastal_Drive");
    system.addRoute(4, 5, 6.8, 12, 5.00, "Harbor_Way");
    system.addRoute(1, 5, 18.0, 28, 14.00, "Stadium_Express");

    cout << "* Sample network initialized with 6 routes.\n" << endl;

    // interactive menu loop
    while (true) {
        cout << "\n========= MAIN MENU =========\n";
        cout << "1. Add a route\n";
        cout << "2. Remove a route\n";
        cout << "3. Update a route\n";
        cout << "4. View all routes\n";
        cout << "5. Find the shortest path\n";
        cout << "6. Sort routes\n";
        cout << "7. Search for a route\n";
        cout << "8. Exit\n";
        cout << "Enter your choice: ";

        int choice;
        cin >> choice;

        if (cin.fail())
            {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << "Invalid input. Please enter a number." << endl;
            continue;
            }

        switch (choice)
        {
            case 1: {
                // Add a route
                int source, destination;
                double distance, travelTime, cost;
                string routeName;

                cout << "\n--- Add New Route ---" << endl;
                cout << "Enter source node ID: ";
                cin >> source;
                cout << "Enter destination node ID: ";
                cin >> destination;
                cout << "Enter distance (km): ";
                cin >> distance;
                cout << "Enter travel time (minutes): ";
                cin >> travelTime;
                cout << "Enter cost ($): ";
                cin >> cost;
                cout << "Enter route name: ";
                cin.ignore();
                getline(cin, routeName);

                system.addRoute(source, destination, distance, travelTime, cost, routeName);
                break;
            }

            case 2: {
                // Remove a route
                int source, destination;
                cout << "\n--- Remove Route ---" << endl;
                cout << "Enter source node ID: ";
                cin >> source;
                cout << "Enter destination node ID: ";
                cin >> destination;

                system.removeRoute(source, destination);
                break;
            }

            case 3: {
                // Update a route
                int source, destination;
                double distance, travelTime, cost;

                cout << "\n--- Update Route ---" << endl;
                cout << "Enter source node ID: ";
                cin >> source;
                cout << "Enter destination node ID: ";
                cin >> destination;
                cout << "Enter new distance (km): ";
                cin >> distance;
                cout << "Enter new travel time (minutes): ";
                cin >> travelTime;
                cout << "Enter new cost ($): ";
                cin >> cost;

                system.updateRoute(source, destination, distance, travelTime, cost);
                break;
            }

            case 4: {
                // View all routes
                system.displayAllRoutes();
                break;
            }

            case 5: {
                // Find shortest path using Dijkstra's algorithm
                if (!system.hasNodes()) {
                    cout << "No nodes in the system. Please add routes first." << endl;
                    break;
                }

                int source, destination;
                string criterion;
                char useAI;

                cout << "\n--- Find Shortest Path (Dijkstra's Algorithm) ---" << endl;
                cout << "Available nodes:" << endl;
                vector<int> nodeIds = system.getAllNodeIds();
                for (int id : nodeIds) {
                    cout << "  Node " << id << ": " << system.getNodeName(id) << endl;
                }

                cout << "\nEnter source node ID: ";
                cin >> source;
                cout << "Enter destination node ID: ";
                cin >> destination;
                cout << "Enter optimization criterion (distance/time/cost): ";
                cin >> criterion;
                cout << "Apply AI congestion prediction? (y/n): ";
                cin >> useAI;

                bool applyAI = (useAI == 'y' || useAI == 'Y');

                // XAI: Execute Dijkstra's algorithm with full explanations
                PathResult result = system.dijkstraShortestPath(source, destination, criterion, applyAI);
                system.displayPathResult(result);

                break;
            }

            case 6: {
                // Sort routes
                string criterion;
                cout << "\n--- Sort Routes ---" << endl;
                cout << "Enter criterion (distance/time/cost): ";
                cin >> criterion;

                system.sortRoutes(criterion);
                break;
            }

            case 7: {
                // Searching for a route
                string keyword;
                cout << "\n--- Search Routes ---" << endl;
                cout << "Enter search keyword: ";
                cin.ignore();
                getline(cin, keyword);

                system.searchRoute(keyword);
                break;
            }

            case 8: {
                cout<<""<<endl;
                cout << "Thank you for using Smart City Route Management System! Goodbye!" << endl;
                return 0;

            }

            default:
                cout << "Invalid choice. Please select 1-8." << endl;
        }
    }

    return 0;
}




/*

                           PROJECT DOCUMENTATION


 Problem Analysis and Solution Approach

[1.] Problem Analysis and Solution Approach

Modern cities face significant challenges in optimizing transport routes due to congestion, varying traffic patterns, and the need for transparency in automated decisions.
This system models the city’s road network as a (wighted graph), where intersections (or zones) are nodes, and roads are edges with distance or cost weights.

Chosen Approach:
A graph-based model using {Dijkstra’s Algorithm} for shortest path calculation.
This approach provides:
* Deterministic and explainable route results
* Efficient performance for sparse networks
* Clear reasoning logs for transparency (XAI component)


[2.] Data Structures and Algorithms Used

~Data Structures

1. std::map<int, vector<Edge>> — Adjacency List
   * Represents the city road network efficiently.
   * Lookup and iteration are O(1) average for sparse graphs.
   * Justification: Cities usually have limited connections between nodes (roads), so adjacency lists save memory.

2. struct Edge
   * Stores destination node and travel cost.
   * Lightweight representation for graph edges.

3. std::priority_queue (Min-Heap)
   * Used in Dijkstra’s algorithm to always expand the current shortest path.
   * Justification: Reduces selection time for the smallest distance node to O(log V).

4. std::map<int, double> dist
   * Stores current best-known distances from the source node.
   * Initialized to infinity, updated as shorter paths are found.

5. std::map<int, int> parent
   * Tracks the previous node for each vertex to reconstruct the route path.

6. struct PathResult
   * Encapsulates the shortest distance and an explanation of each decision step.
   * Justification: Supports explainable AI (XAI) by showing *why* each route was chosen.



~ Algorithms

1. Dijkstra’s Algorithm
   * Finds the shortest path from source to destination.
   * Time complexity: O((V + E) log V)
   * Guarantees optimal routes on graphs with non-negative edge weights.
   * Chosen for clarity, explainability, and reliability.

2. AI-Based Congestion Adjustment
   * Edge weights are dynamically adjusted based on time of day.
   * Reflects realistic traffic conditions, e.g.:
   * Peak hours (7–9 AM, 5–7 PM) → +20% road cost
   *Off-peak → normal weight
   * Justification: Adds adaptive intelligence while maintaining transparency.



[3.] XAI Principles Applied (Explainable Artificial Intelligence)

* Transparent Computation:** Every routing decision is recorded and displayed.
* Runtime Explanation:** The `PathResult` includes step-by-step logs showing why a node was selected next.
* Human-Readable Output:** Routes are described in a way non-technical stakeholders can understand.
* Predictive Context:** The algorithm explains when congestion or time adjustments affect route choice.

**Example:**



[4.] AI Integration

Purpose: Enhance realism and decision intelligence.

~Congestion Prediction:
  * Based on static rules (time-based multipliers).
  * Example:
    - Morning (7–9 AM): +20% distance weight
    - Afternoon (5–7 PM): +25%
    - Normal hours: 0% adjustment
  * Justification: Reflects observed urban traffic peaks.

~ Future Extension Ideas:
  * Integration with live IoT traffic data.
  * Reinforcement learning for adaptive weight adjustment.
  * Predictive maintenance (detecting bottlenecks from usage frequency).



[5.] Menu Interface Example

SMART CITY ROUTE MANAGEMENT SYSTEM

1.Add Road Connection

2.View Road Network

3.Find Optimal Route

4.Explain Route Decision (XAI)

5.Exit
Enter your choice: 3
Enter source node: 1
Enter destination node: 5

Shortest route found: 1 → 3 → 5
Total Distance: 12.4 km
Explanation:

Node 3 chosen (lowest current distance)

Edge 3→5 had lowest cost (adjusted for off-peak)


*/
