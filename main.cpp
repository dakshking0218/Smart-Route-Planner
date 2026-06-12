#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <string>
#include <limits>
#include <algorithm>
#include <iomanip>

using namespace std;

// Structure to represent a road (Edge) in the transportation network
struct Road {
    string destination;
    double distance;       // Base distance in km
    double trafficFactor;  // 1.0 = Clear, 2.0 = Moderate, 3.5 = Heavy Congestion
    
    // Calculates effective travel time (weight for Dijkstra)
    double getTravelTime() const {
        return distance * trafficFactor;
    }
};

// Class handling the graph-based transit network
class TransportGraph {
private:
    unordered_map<string, vector<Road>> adjList;

public:
    // Add a bi-directional road to the network
    void addRoad(const string& u, const string& v, double distance) {
        adjList[u].push_back({v, distance, 1.0});
        adjList[v].push_back({u, distance, 1.0});
    }

    // Dynamically update traffic conditions on a specific road segment
    bool updateTraffic(const string& u, const string& v, double newTrafficFactor) {
        bool updated = false;
        if (adjList.find(u) != adjList.end()) {
            for (auto& road : adjList[u]) {
                if (road.destination == v) { road.trafficFactor = newTrafficFactor; updated = true; }
            }
        }
        if (adjList.find(v) != adjList.end()) {
            for (auto& road : adjList[v]) {
                if (road.destination == u) { road.trafficFactor = newTrafficFactor; updated = true; }
            }
        }
        return updated;
    }

    // Display all active paths, current traffic status, and raw distances
    void displayNetwork() const {
        cout << "\n================= CURRENT TRANSIT NETWORK =================\n";
        cout << left << setw(15) << "Source" << setw(15) << "Destination" << setw(15) << "Distance(km)" << "Traffic Condition\n";
        cout << "-----------------------------------------------------------\n";
        for (const auto& pair : adjList) {
            for (const auto& road : pair.second) {
                // To avoid printing duplicate bi-directional roads twice visually, print conditionally
                if (pair.first < road.destination) {
                    string trafficStr = "Clear (1.0x)";
                    if (road.trafficFactor > 1.0 && road.trafficFactor <= 2.0) trafficStr = "Moderate (2.0x)";
                    else if (road.trafficFactor > 2.0) trafficStr = "Heavy Gridlock (" + to_string(road.trafficFactor).substr(0,3) + "x)";
                    
                    cout << left << setw(15) << pair.first 
                         << setw(15) << road.destination 
                         << setw(15) << road.distance 
                         << trafficStr << "\n";
                }
            }
        }
        cout << "===========================================================\n";
    }

    // Core Dijkstra's Algorithm implementation optimizing for shortest travel time
    pair<vector<string>, double> findShortestRoute(const string& start, const string& end) {
        if (adjList.find(start) == adjList.end() || adjList.find(end) == adjList.end()) {
            return {{}, -1.0}; // Node not found
        }

        // Min-heap structure storing pairs of: {accumulated_time, current_node}
        priority_queue<pair<double, string>, vector<pair<double, string>>, greater<pair<double, string>>> pq;
        
        unordered_map<string, double> minTravelTime;
        unordered_map<string, string> parentMap;

        // Initialize node structures
        for (const auto& pair : adjList) {
            minTravelTime[pair.first] = numeric_limits<double>::infinity();
        }

        // Base case initialization
        minTravelTime[start] = 0.0;
        pq.push({0.0, start});

        while (!pq.empty()) {
            double currentTime = pq.top().first;
            string currentNode = pq.top().second;
            pq.pop();

            // Early exit check if target reached
            if (currentNode == end) break;

            // Skip processing if a better path was already recorded
            if (currentTime > minTravelTime[currentNode]) continue;

            for (const auto& road : adjList[currentNode]) {
                double nextTime = currentTime + road.getTravelTime();
                
                if (nextTime < minTravelTime[road.destination]) {
                    minTravelTime[road.destination] = nextTime;
                    parentMap[road.destination] = currentNode;
                    pq.push({nextTime, road.destination});
                }
            }
        }

        // Path reconstruction sequence
        if (minTravelTime[end] == numeric_limits<double>::infinity()) {
            return {{}, -1.0}; // Destination unreachable
        }

        vector<string> path;
        string temp = end;
        while (temp != start) {
            path.push_back(temp);
            temp = parentMap[temp];
        }
        path.push_back(start);
        reverse(path.begin(), path.end());

        return {path, minTravelTime[end]};
    }
};

// Manager class handling system routing flow, history, and CLI interface
class RoutePlannerSystem {
private:
    TransportGraph network;
    vector<string> routeHistory; // Tracks recent routing logs

    void seedInitialMapData() {
        // Seed default routing grid nodes (Intersections/Hubs)
        network.addRoad("TechPark", "Hub_A", 4.5);
        network.addRoad("TechPark", "Suburbs", 8.0);
        network.addRoad("Hub_A", "Downtown", 3.2);
        network.addRoad("Hub_A", "Airport", 12.5);
        network.addRoad("Suburbs", "Downtown", 6.1);
        network.addRoad("Suburbs", "Station", 5.0);
        network.addRoad("Downtown", "Station", 2.5);
        network.addRoad("Downtown", "Airport", 9.0);
        network.addRoad("Station", "Airport", 7.2);
    }

public:
    RoutePlannerSystem() {
        seedInitialMapData();
    }

    void calculateRoute() {
        string src, dest;
        cout << "\nEnter Starting Node Name: ";
        cin >> src;
        cout << "Enter Destination Node Name: ";
        cin >> dest;

        auto result = network.findShortestRoute(src, dest);
        
        if (result.second == -1.0) {
            cout << "\n[ERROR] Route couldn't be evaluated. Ensure valid spelling or check if node connectivity exists.\n";
            return;
        }

        cout << "\n=================== OPTIMAL ROUTE FOUND ===================\n";
        cout << "Path: ";
        string historyEntry = src + " -> " + dest + " | Path: ";
        for (size_t i = 0; i < result.first.size(); ++i) {
            cout << result.first[i];
            historyEntry += result.first[i];
            if (i < result.first.size() - 1) {
                cout << " -> ";
                historyEntry += " -> ";
            }
        }
        cout << "\nEstimated Travel Time index unit: " << fixed << setprecision(2) << result.second << " units\n";
        cout << "===========================================================\n";

        // Push search statement to local vector tracker
        routeHistory.push_back(historyEntry + " (Time: " + to_string(result.second).substr(0,4) + ")");
    }

    void simulateLiveTraffic() {
        string src, dest;
        double dynamicFactor;
        cout << "\nEnter Road Source Segment: ";
        cin >> src;
        cout << "Enter Road Destination Segment: ";
        cin >> dest;
        cout << "Enter Live Congestion Modifier Multiplier (e.g., 1.0 for Clear, 3.0 for Traffic Jam): ";
        cin >> dynamicFactor;

        if (network.updateTraffic(src, dest, dynamicFactor)) {
            cout << "\n[SUCCESS] Traffic model synchronized dynamically for segment: " << src << " <-> " << dest << ".\n";
        } else {
            cout << "\n[ERROR] Direct segment link mapping does not exist between these nodes.\n";
        }
    }

    void printHistory() const {
        cout << "\n====================== ROUTING HISTORY ======================\n";
        if (routeHistory.empty()) {
            cout << "No route queries logged during this active runtime session.\n";
        } else {
            for (size_t i = 0; i < routeHistory.size(); ++i) {
                cout << (i + 1) << ". " << routeHistory[i] << "\n";
            }
        }
        cout << "=============================================================\n";
    }

    void showNetwork() const {
        network.displayNetwork();
    }
};

int main() {
    RoutePlannerSystem systemApp;
    int userChoice = 0;

    while (true) {
        cout << "\n*** SMART TRANSIT GRAPH ROUTE PLANNER ENGINE ***\n";
        cout << "1. View Complete Road Grid System Info\n";
        cout << "2. Calculate Fastest Dynamic Route (Dijkstra)\n";
        cout << "3. Inject Real-Time Traffic Simulation Data\n";
        cout << "4. Display Query Session Route History\n";
        cout << "5. Exit System App Terminal\n";
        cout << "Choose Action (1-5): ";
        
        if (!(cin >> userChoice)) {
            cout << "Invalid runtime type. Stopping app session safely.\n";
            break;
        }

        if (userChoice == 5) {
            cout << "\nShutting down engine terminal safely. Smooth traveling!\n";
            break;
        }

        switch (userChoice) {
            case 1: systemApp.showNetwork(); break;
            case 2: systemApp.calculateRoute(); break;
            case 3: systemApp.simulateLiveTraffic(); break;
            case 4: systemApp.printHistory(); break;
            default: cout << "Invalid Selection Option. Try alternative input action item range.\n";
        }
    }
    return 0;
}
