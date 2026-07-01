# 🚦 Smart Transit Graph Route Planner

A **graph-based urban routing engine** built in C++ that computes optimal travel paths across a city transit network using **Dijkstra's Algorithm**. Supports real-time traffic simulation, dynamic road blocking, and live route recalculation — all from the command line.

---

## 📌 Overview

This project models a city's road network as a **weighted directed graph**, where intersections are nodes and roads are edges weighted by distance and live traffic conditions. The engine finds the fastest route between any two points, automatically rerouting around congestion or blocked roads.

Built as a demonstration of core **Data Structures & Algorithms** concepts applied to a real-world problem domain.

---

## ⚙️ How It Works

```
   TechPark ──── Mall ──── Hub_A ──── Airport
      │            │          │           │
   Suburbs     Hospital   Downtown    Station
      │            │          │           │
   University ─────────── Suburbs ────────┘
```

Each road has two properties that combine into a **travel weight**:

```
Travel Weight = Distance (km) × Traffic Factor
```

| Traffic Factor | Condition      |
|----------------|----------------|
| 1.0×           | Clear          |
| 2.0×           | Moderate       |
| 3.5×           | Heavy Gridlock |
| ∞              | Blocked        |

Dijkstra's algorithm runs on these weights, so it naturally avoids heavily congested or blocked roads and finds the globally optimal path.

---

## 🧠 Algorithm & Data Structure Details

### Dijkstra's Algorithm — `O(E log V)`
The core shortest-path engine uses a **min-heap priority queue** that always processes the lowest-cost node next. This gives `O(E log V)` time complexity, making it efficient even as the network scales.

```cpp
priority_queue<
    pair<double, string>,
    vector<pair<double, string>>,
    greater<pair<double, string>>
> pq;
```

### Hash Table Adjacency List — `O(1)` average lookup
The road network is stored as an `unordered_map<string, vector<Road>>`. Node lookups by name run in **O(1) average time**, enabling instant coordinate retrieval and neighbor traversal without scanning the full graph.

```cpp
unordered_map<string, vector<Road>> adjList;
```

### Dynamic Weight Recalculation
Traffic factors are stored per-edge and read at query time — not baked in at construction. This means a single `updateTraffic()` call is enough; the next route query automatically reflects the new conditions with no graph rebuild needed.

---

## 🗺️ Available Nodes (City Map)

| Node         | Type              |
|--------------|-------------------|
| `TechPark`   | Business Hub      |
| `Hub_A`      | Transit Junction  |
| `Downtown`   | City Centre       |
| `Airport`    | Airport Terminal  |
| `Suburbs`    | Residential Zone  |
| `Station`    | Railway Station   |
| `Mall`       | Commercial Zone   |
| `Hospital`   | Medical Centre    |
| `University` | Academic Campus   |

---



## 🖥️ Features & Usage

### Menu Options

```
1. View Complete Road Network
2. Calculate Fastest Route  (Dijkstra)
3. Update Live Traffic Data
4. Display Route History
5. Block / Unblock a Road Segment
6. Exit
```

---

### 1. View Road Network
Displays every road segment with distance, live traffic condition, and open/blocked status.

```
Source          Destination     Distance(km)   Traffic Condition      Status
---------------------------------------------------------------------------
TechPark        Hub_A           4.5            Clear (1.0x)           Open
Downtown        Airport         9.0            Heavy Gridlock (3.5x)  Open
Suburbs         Station         5.0            Clear (1.0x)           Open
```

---

### 2. Calculate Fastest Route
Enter any two nodes. The engine returns the optimal path, total distance in km, and estimated travel time index.

```
Enter Starting Node Name  : TechPark
Enter Destination Node Name: Airport

=================== OPTIMAL ROUTE FOUND ===================
Path: TechPark -> Hub_A -> Downtown -> Airport

Total Distance      : 16.70 km
Estimated Travel Time: 16.70 units  (distance x traffic factor)
===========================================================
```

---

### 3. Update Live Traffic Data
Inject a congestion multiplier onto any road segment. The next route calculation automatically factors it in.

```
Enter Road Source Segment     : Downtown
Enter Road Destination Segment: Airport
Enter Congestion Multiplier   : 3.5

[SUCCESS] Traffic model updated for segment: Downtown <-> Airport.
```

Re-running the route after this will reroute via Station instead.

---

### 4. Block / Unblock a Road
Simulate road closures (accidents, construction). Blocked roads are assigned infinite weight so Dijkstra routes around them entirely.

```
Enter Road Source Node      : Hub_A
Enter Road Destination Node : Airport
Enter action (block / unblock): block

[SUCCESS] Road segment Hub_A <-> Airport is now BLOCKED.
[INFO] Dijkstra will automatically route around this segment.
```

---

### 5. Route History
View all route queries from the current session with their paths, distances, and times.

```
1. TechPark -> Airport | Path: TechPark -> Hub_A -> Airport | Dist: 17.00 km | Time: 17.00
2. TechPark -> Airport | Path: TechPark -> Hub_A -> Downtown -> Airport | Dist: 16.70 km | Time: 16.70
```

---


---

## 💡 Key Technical Decisions

**Why Dijkstra over BFS/DFS?**
BFS finds the fewest hops, not the fastest route. Since edges have variable weights (distance × traffic), Dijkstra is the correct algorithm — it guarantees the globally optimal weighted path.

**Why `unordered_map` over `map`?**
`std::map` gives `O(log N)` lookup via a red-black tree. `unordered_map` gives `O(1)` average via hashing — important for fast node resolution in a city-scale graph.

**Why store `blocked` as a flag vs. removing the edge?**
Removing an edge requires rebuilding the adjacency list. A boolean flag means blocking and unblocking is `O(degree)` — fast and reversible, which is realistic for a live traffic system.

---

