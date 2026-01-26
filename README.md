# 🚦 Smart City Route Management System

An intelligent **C++ console-based transportation network simulator** that models a city's road system using graph theory and finds optimal routes using **Dijkstra’s Algorithm** with **AI-based congestion prediction** and **Explainable AI (XAI)**.

---

## 🌍 Project Overview

Modern cities struggle with:
- Traffic congestion
- Route inefficiencies
- Lack of transparency in automated navigation systems

This project simulates a **smart city routing system** that:
- Represents intersections as **nodes**
- Represents roads as **weighted edges**
- Calculates the **optimal route** based on:
  - Distance
  - Travel Time
  - Cost

It also includes:
✔ AI-inspired congestion adjustment  
✔ Step-by-step decision explanations (XAI)  
✔ Route searching and sorting features  

---

## 🧠 Core Technologies & Concepts

| Concept | Implementation |
|--------|----------------|
| Programming Language | C++ |
| Graph Representation | Adjacency List (`map<int, vector<Edge>>`) |
| Pathfinding Algorithm | Dijkstra’s Algorithm |
| Data Structures | Map, Vector, Priority Queue (Min-Heap), Set |
| AI Feature | Time-based congestion prediction |
| XAI | Detailed step-by-step path reasoning |

---

## 🛣 Features

### 🔹 Route Management
- Add new routes
- Remove routes
- Update route details
- View full road network

### 🔹 Smart Pathfinding
Find the shortest path based on:
- 📏 Distance
- ⏱ Travel Time
- 💲 Cost

### 🔹 AI Congestion Prediction
Simulates real-world traffic:
| Time | Traffic Condition | Effect |
|------|------------------|--------|
| 7–9 AM | Morning Peak | +50% travel time |
| 5–7 PM | Evening Peak | +50% travel time |
| Other hours | Normal | No change |

### 🔹 Explainable AI (XAI)
The system explains:
- Each step Dijkstra takes
- Why a node was selected
- How costs were calculated
- Why the final path is optimal

---

## 🏗 Data Structures Used

- `struct Edge` — Represents roads between intersections  
- `map<int, vector<Edge>>` — Adjacency list for the city network  
- `priority_queue` — Efficient shortest path node selection  
- `map<int, double>` — Stores shortest distances  
- `map<int, int>` — Stores parent nodes for path reconstruction  

---

## ⚙ Algorithm

### Dijkstra’s Algorithm
- Time Complexity: **O((V + E) log V)**
- Guarantees shortest path for non-negative weights
- Uses greedy strategy + optimal substructure

---

## 🖥 Sample Network

The system initializes with:

| Node ID | Location |
|--------|----------|
| 1 | Downtown |
| 2 | Airport |
| 3 | University |
| 4 | Harbor |
| 5 | Stadium |

---

## 📋 Menu System


---

## ▶ How to Run

### Compile:
```bash
g++ main.cpp -o smart_city

Run:
./smart_city

Example Output
Step 1: Exploring node Downtown
  --> Found path to University via Main_Street
Step 2: Exploring node University
  --> Better path to Harbor via Coastal_Drive

Optimal Path:
Downtown → University → Harbor → Stadium

Total Distance: 27.5 km
Total Time: 42 minutes
Total Cost: $20.50
