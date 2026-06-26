#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <string>
#include <limits>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <set>

using namespace std;

// ─────────────────────────────────────────────────────────────────────────────
// Structure to represent a road (Edge) in the transportation network
// ─────────────────────────────────────────────────────────────────────────────
struct Road {
    string destination;
    double distance;       // Base distance in km
    double trafficFactor;  // 1.0 = Clear, 2.0 = Moderate, 3.5 = Heavy Congestion
    bool   blocked;        // NEW: whether this road segment is temporarily closed

    Road(const string& dest, double dist, double tf = 1.0)
        : destination(dest), distance(dist), trafficFactor(tf), blocked(false) {}

    // Calculates effective travel time (weight for Dijkstra)
    // Blocked roads return infinity so Dijkstra naturally avoids them
    double getTravelTime() const {
        if (blocked) return numeric_limits<double>::infinity();
        return distance * trafficFactor;
    }

    // Returns raw distance (used to display actual km in results)
    double getDistance() const {
        return blocked ? numeric_limits<double>::infinity() : distance;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Helper: format a double to a fixed-precision string cleanly
// (replaces the fragile substr(0,4) hack)
// ─────────────────────────────────────────────────────────────────────────────
static string fmtDouble(double val, int precision = 2) {
    ostringstream oss;
    oss << fixed << setprecision(precision) << val;
    return oss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
// Class handling the graph-based transit network
// Uses hash-table adjacency list for O(1) average node lookup
// ─────────────────────────────────────────────────────────────────────────────
class TransportGraph {
private:
    unordered_map<string, vector<Road>> adjList;

public:
    // Add a bi-directional road to the network
    void addRoad(const string& u, const string& v, double distance) {
        adjList[u].emplace_back(v, distance, 1.0);
        adjList[v].emplace_back(u, distance, 1.0);
    }

    // ── NEW: Temporarily block / unblock a road segment ──────────────────────
    // Returns true if the segment was found, false otherwise
    bool setRoadBlocked(const string& u, const string& v, bool blockState) {
        bool found = false;
        auto applyBlock = [&](const string& src, const string& dst) {
            auto it = adjList.find(src);
            if (it == adjList.end()) return;
            for (auto& road : it->second) {
                if (road.destination == dst) {
                    road.blocked = blockState;
                    found = true;
                }
            }
        };
        applyBlock(u, v);
        applyBlock(v, u);
        return found;
    }

    // Dynamically update traffic conditions on a specific road segment
    // Integrates live traffic multipliers into path weights so Dijkstra
    // automatically recalculates optimal routes around real-time congestion
    bool updateTraffic(const string& u, const string& v, double newTrafficFactor) {
        bool updated = false;
        auto applyTraffic = [&](const string& src, const string& dst) {
            auto it = adjList.find(src);
            if (it == adjList.end()) return;
            for (auto& road : it->second) {
                if (road.destination == dst) {
                    road.trafficFactor = newTrafficFactor;
                    updated = true;
                }
            }
        };
        applyTraffic(u, v);
        applyTraffic(v, u);
        return updated;
    }

    // Display all active paths, current traffic status, and raw distances
    void displayNetwork() const {
        cout << "\n================= CURRENT TRANSIT NETWORK =================\n";
        cout << left
             << setw(15) << "Source"
             << setw(15) << "Destination"
             << setw(15) << "Distance(km)"
             << setw(22) << "Traffic Condition"
             << "Status\n";
        cout << "-------------------------------------------------------------------\n";

        // Collect printed pairs to avoid printing bi-directional edges twice
        set<pair<string,string>> printed;

        for (const auto& entry : adjList) {
            for (const auto& road : entry.second) {
                string a = entry.first, b = road.destination;
                if (a > b) swap(a, b);
                if (!printed.insert({a, b}).second) continue; // already printed

                // ── Traffic label uses actual factor value (bug fix) ──────────
                string trafficStr;
                if (road.trafficFactor <= 1.0) {
                    trafficStr = "Clear (1.0x)";
                } else if (road.trafficFactor <= 2.0) {
                    trafficStr = "Moderate (" + fmtDouble(road.trafficFactor) + "x)";
                } else {
                    trafficStr = "Heavy Gridlock (" + fmtDouble(road.trafficFactor) + "x)";
                }

                string statusStr = road.blocked ? "*** BLOCKED ***" : "Open";

                cout << left
                     << setw(15) << entry.first
                     << setw(15) << road.destination
                     << setw(15) << road.distance
                     << setw(22) << trafficStr
                     << statusStr << "\n";
            }
        }
        cout << "===================================================================\n";
    }

    // ── Core Dijkstra's Algorithm ─────────────────────────────────────────────
    // Optimizes for shortest travel time using a min-priority queue → O(E log V)
    // Returns: { path vector, total_travel_time, total_distance }
    // Blocked roads are automatically avoided (their weight is infinity)
    struct RouteResult {
        vector<string> path;
        double totalTime;      // weighted travel time (distance × traffic factor)
        double totalDistance;  // raw distance in km
    };

    RouteResult findShortestRoute(const string& start, const string& end) {
        RouteResult bad = {{}, -1.0, -1.0};

        if (adjList.find(start) == adjList.end() ||
            adjList.find(end)   == adjList.end()) return bad;

        // Min-heap: {accumulated_time, current_node}
        priority_queue<
            pair<double, string>,
            vector<pair<double, string>>,
            greater<pair<double, string>>
        > pq;

        unordered_map<string, double> minTime;
        unordered_map<string, double> distAtMin; // tracks actual km on best path
        unordered_map<string, string> parentMap;

        for (const auto& entry : adjList)
            minTime[entry.first] = numeric_limits<double>::infinity();

        minTime[start]    = 0.0;
        distAtMin[start]  = 0.0;
        pq.push({0.0, start});

        while (!pq.empty()) {
            double currentTime = pq.top().first;
            string currentNode = pq.top().second;
            pq.pop();

            if (currentNode == end) break;
            if (currentTime > minTime[currentNode]) continue; // stale entry

            for (const auto& road : adjList[currentNode]) {
                double edgeTime = road.getTravelTime();
                if (edgeTime == numeric_limits<double>::infinity()) continue; // skip blocked

                double nextTime = currentTime + edgeTime;
                if (nextTime < minTime[road.destination]) {
                    minTime[road.destination]   = nextTime;
                    distAtMin[road.destination] = distAtMin[currentNode] + road.distance;
                    parentMap[road.destination] = currentNode;
                    pq.push({nextTime, road.destination});
                }
            }
        }

        if (minTime[end] == numeric_limits<double>::infinity()) return bad;

        // Path reconstruction
        vector<string> path;
        string temp = end;
        while (temp != start) {
            path.push_back(temp);
            temp = parentMap[temp];
        }
        path.push_back(start);
        reverse(path.begin(), path.end());

        return {path, minTime[end], distAtMin[end]};
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Manager class: routing flow, history, CLI interface
// ─────────────────────────────────────────────────────────────────────────────
class RoutePlannerSystem {
private:
    TransportGraph network;
    vector<string>  routeHistory;

    void seedInitialMapData() {
        // Urban transit grid — nodes represent intersections / transit hubs
        network.addRoad("TechPark",  "Hub_A",    4.5);
        network.addRoad("TechPark",  "Suburbs",  8.0);
        network.addRoad("Hub_A",     "Downtown", 3.2);
        network.addRoad("Hub_A",     "Airport",  12.5);
        network.addRoad("Suburbs",   "Downtown", 6.1);
        network.addRoad("Suburbs",   "Station",  5.0);
        network.addRoad("Downtown",  "Station",  2.5);
        network.addRoad("Downtown",  "Airport",  9.0);
        network.addRoad("Station",   "Airport",  7.2);
        // NEW nodes — expanded urban network
        network.addRoad("TechPark",  "Mall",     3.0);
        network.addRoad("Mall",      "Hub_A",    2.8);
        network.addRoad("Mall",      "Hospital", 4.1);
        network.addRoad("Hospital",  "Downtown", 3.5);
        network.addRoad("Hospital",  "Station",  6.0);
        network.addRoad("University","Suburbs",  2.2);
        network.addRoad("University","Hospital", 5.5);
        network.addRoad("University","TechPark", 6.8);
    }

public:
    RoutePlannerSystem() { seedInitialMapData(); }

    // ── Option 2: Calculate route — now shows BOTH distance AND time ──────────
    void calculateRoute() {
        string src, dest;
        cout << "\nEnter Starting Node Name  : ";
        cin >> src;
        cout << "Enter Destination Node Name: ";
        cin >> dest;

        auto result = network.findShortestRoute(src, dest);

        if (result.totalTime < 0) {
            cout << "\n[ERROR] Route could not be evaluated. "
                    "Check spelling or node connectivity.\n";
            return;
        }

        cout << "\n=================== OPTIMAL ROUTE FOUND ===================\n";
        cout << "Path: ";
        string historyEntry = src + " -> " + dest + " | Path: ";

        for (size_t i = 0; i < result.path.size(); ++i) {
            cout << result.path[i];
            historyEntry += result.path[i];
            if (i < result.path.size() - 1) {
                cout << " -> ";
                historyEntry += " -> ";
            }
        }

        // ── Show distance AND time separately (new feature) ──────────────────
        cout << "\n\nTotal Distance      : " << fmtDouble(result.totalDistance) << " km";
        cout << "\nEstimated Travel Time: " << fmtDouble(result.totalTime)
             << " units  (distance x traffic factor)\n";
        cout << "===========================================================\n";

        historyEntry += " | Dist: " + fmtDouble(result.totalDistance)
                      + " km | Time: " + fmtDouble(result.totalTime);
        routeHistory.push_back(historyEntry);
    }

    // ── Option 3: Live traffic injection ─────────────────────────────────────
    void simulateLiveTraffic() {
        string src, dest;
        double dynamicFactor;
        cout << "\nEnter Road Source Segment     : ";
        cin >> src;
        cout << "Enter Road Destination Segment: ";
        cin >> dest;
        cout << "Enter Congestion Multiplier (1.0=Clear, 2.0=Moderate, 3.5=Heavy): ";
        cin >> dynamicFactor;

        if (dynamicFactor < 1.0) {
            cout << "\n[WARNING] Traffic factor below 1.0 is unrealistic. Setting to 1.0.\n";
            dynamicFactor = 1.0;
        }

        if (network.updateTraffic(src, dest, dynamicFactor)) {
            cout << "\n[SUCCESS] Traffic model updated for segment: "
                 << src << " <-> " << dest << ".\n";
        } else {
            cout << "\n[ERROR] No direct road segment found between these nodes.\n";
        }
    }

    // ── NEW Option 5: Block / Unblock a road ─────────────────────────────────
    void manageRoadBlock() {
        string src, dest, action;
        cout << "\nEnter Road Source Node      : ";
        cin >> src;
        cout << "Enter Road Destination Node : ";
        cin >> dest;
        cout << "Enter action (block / unblock): ";
        cin >> action;

        bool blockState = (action == "block");

        if (network.setRoadBlocked(src, dest, blockState)) {
            string stateStr = blockState ? "BLOCKED" : "UNBLOCKED";
            cout << "\n[SUCCESS] Road segment " << src << " <-> " << dest
                 << " is now " << stateStr << ".\n";
            if (blockState) {
                cout << "[INFO] Dijkstra will automatically route around this segment.\n";
            }
        } else {
            cout << "\n[ERROR] No direct road segment found between these nodes.\n";
        }
    }

    void printHistory() const {
        cout << "\n====================== ROUTING HISTORY ======================\n";
        if (routeHistory.empty()) {
            cout << "No route queries logged in this session.\n";
        } else {
            for (size_t i = 0; i < routeHistory.size(); ++i)
                cout << (i + 1) << ". " << routeHistory[i] << "\n";
        }
        cout << "=============================================================\n";
    }

    void showNetwork() const { network.displayNetwork(); }
};

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    RoutePlannerSystem systemApp;
    int userChoice = 0;

    cout << "\n***  SMART TRANSIT GRAPH ROUTE PLANNER ENGINE  ***\n";
    cout << "Nodes available: TechPark, Hub_A, Suburbs, Downtown,\n"
            "                 Airport, Station, Mall, Hospital, University\n";

    while (true) {
        cout << "\n─────────────────────────────────────────\n";
        cout << "1. View Complete Road Network\n";
        cout << "2. Calculate Fastest Route  (Dijkstra)\n";
        cout << "3. Update Live Traffic Data\n";
        cout << "4. Display Route History\n";
        cout << "5. Block / Unblock a Road Segment\n";
        cout << "6. Exit\n";
        cout << "Choose Action (1-6): ";

        if (!(cin >> userChoice)) {
            cout << "Invalid input. Exiting safely.\n";
            break;
        }

        switch (userChoice) {
            case 1: systemApp.showNetwork();      break;
            case 2: systemApp.calculateRoute();   break;
            case 3: systemApp.simulateLiveTraffic(); break;
            case 4: systemApp.printHistory();     break;
            case 5: systemApp.manageRoadBlock();  break;
            case 6:
                cout << "\nShutting down engine safely. Safe travels!\n";
                return 0;
            default:
                cout << "Invalid selection. Please choose between 1 and 6.\n";
        }
    }
    return 0;
}
