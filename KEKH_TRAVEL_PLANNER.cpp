#include <bits/stdc++.h>
using namespace std;
#ifdef LOCAL
#include "algo/debug.h"
#else
#define debug(...) 42
#endif

map<int, string> locations; //Store locations

map<int, vector<tuple<int, int, int, int>>> adj; //Graphical representation adjacency list (start, end, cost, distance, time)

void loaddata(const string& data){ //Load location and route informations (data.txt)
  ifstream file(data);
  if(!file.is_open()){
    cout << "Can not open file" << "\n";
    exit(1);
  }
  string line;
  bool readlocation = false, readadj = false;
  while(getline(file, line)){
    if(line.empty() || line[0] == '#'){
      if(line.find("# Location Data") != string::npos){
        readlocation = true;
        readadj = false;
      }else if(line.find("# Adjacency Data") != string::npos){
        readlocation = false;
        readadj = true;
      }
      continue;
    }
    if(readlocation){
      stringstream ss(line);
      int id;
      string name;
      ss >> id; //Read location names and #location
      getline(ss, name);
      locations[id] = name.substr(1);
    }else if(readadj){
      stringstream ss(line);
      int start, end, cost, distance, time;
      ss >> start >> end >> cost >> distance >> time; //Read start, destination, cost, distance, time from txt file
      adj[start].emplace_back(end, cost, distance, time);
      adj[end].emplace_back(start, cost, distance, time);
    }
  }
  file.close();
}

void printlocations(){
  cout << "Visitable locations: " << "\n";
  for(auto as: locations){
    cout << as.first + 1 << ". " << as.second << "\n";
  }
}

int choosestart(){ //Starting point choosing option
  cout << "From the locations below where do you want to start from?" << "\n" << flush;
  printlocations();
  int start;
  cout << "\n";
  cout << "Enter your choice: " << flush;
  cin >> start; //Take start location input
  cout << "\n";
  if(start < 1 || start > locations.size()){ //If choice is invalid
    cout << "Please Enter between 1 - " << locations.size() << "\n";
    return choosestart();
  }
  return start;
}

int chooseend(){ //Destination point choosing option
  cout << "From the locations below where do you want to go?" << "\n";
  printlocations();
  cout << "\n";
  cout << "Enter your choice: " << flush;
  int end;
  cin >> end; //Take destination location input
  cout << "\n";
  if(end < 1 || end > locations.size()){
    cout << "Please Enter between 1 - " << locations.size() << "\n";
    return chooseend();
  }
  return end;
}

vector<int> menu(){ //Main menu
  vector<int> current(5);
  cout << "\n" << "------ KEKH Travel Planner ------" << "\n";
  cout << "What do you want?" << "\n";
  cout << "| 1. Cheapest route." << "\n";
  cout << "| 2. Shortest distance." << "\n";
  cout << "| 3. Minimum time." <<"\n";
  cout << "| 4. Show visitable locations." << "\n";
  cout << "| 5. Exit program." << "\n\n";
  cout << "Enter your choice: " << flush;
  int choice;
  cin >> choice; //Take choice input
  cout << "\n";
  if(choice > 5 || choice < 1){ //Handle invalid choice
    cout << "---- Invalid choice ----" << "\n";
    return menu();
  }
  current[0] = choice;
  if(choice == 4){
    return current;
  }
  if(choice == 5){
    return current; //If choice = 5, exit the program
  }
  int start = choosestart();
  int end = chooseend();
  start--;
  end--;
  current[1] = start;
  current[2] = end;
  return current;
}

void dij(int start, int end){ //Dijkstra O(E * log(V)) for cost; 
  //Stores cost and path as a pair for multiple route feature
  priority_queue<pair<int, vector<int>>, vector<pair<int, vector<int>>>, greater<pair<int, vector<int>>>> pq; 
  unordered_set<string> visited; //Store routes to avoid vising same routes
  vector<pair<int, vector<int>>> tp; //Stores top routes with cost and routh path
  pq.push({0, {start}});
  while(!pq.empty() && tp.size() < 3){
    auto [currc, currp] = pq.top();
    pq.pop();
    int u = currp.back();
    if(u == end){
      tp.emplace_back(currc, currp);
      continue;
    }
    string pathk = "";
    for(int n: currp) pathk += to_string(n) + ",";
    if(visited.count(pathk)) continue; //Avoid going to same route
    visited.insert(pathk);
    for(auto& neighbour: adj[u]){
      int v, cost, distance, time;
      tie(v, cost, distance, time) = neighbour;
      int wt = cost;
      if(find(currp.begin(), currp.end(), v) == currp.end()){ //New route dicovered
        vector<int> np = currp;
        np.push_back(v);
        pq.push({currc + wt, np});
      }
    }
  }
  cout << "Top 3 minimum cost routes from " << locations[start] << " to " << locations[end] << "\n";
  for(auto& [cost, path]: tp){ //Prints top 3 routes from the tp vector
    cout<< "Cost: " << cost << " Taka" <<", Route: ";
    for(int i = 0; i < path.size(); i++){
      if(i < path.size() - 1){
        cout << locations[path[i]] << " -> ";
      }else{
        cout << locations[path[i]];
      }
    }
    cout << "\n";
  }
  cout << "\n"; 
}

void bell(int start, int end){ //Bellman ford for shortest distance O(V * E)
  vector<int> magnitude(locations.size(), INT_MAX);
  magnitude[start] = 0;
  for(int i = 0; i < locations.size() - 1; i++){
    for(int u = 0; u < locations.size(); u++){
      for(auto& neighbour: adj[u]){
        int v, cost, distance, time;
        tie(v, cost, distance, time) = neighbour;
        int wt = distance;
        if(magnitude[u] != INT_MAX && magnitude[v] > magnitude[u] + wt){
          magnitude[v] = magnitude[u] + wt; //Shorter distance found
        }
      }
    }
  }
  cout << "Shortest distance from " << locations[start] << " to " << locations[end] << " = ";
  cout << magnitude[end];
  cout << " Kilometers";
  cout << "\n";
  cout << flush;
}

void flo(int start, int end){ // Floyd Warshall for minimum time O(V ^ 3)
  vector<vector<int>> graph(locations.size(), vector<int>(locations.size(), INT_MAX)); //Turn adjacency list into adjacency matrix
  for(int i = 0; i < locations.size(); i++){
    graph[i][i] = 0;
  }
  for(auto& [u, edges]: adj){
    for(auto& [v, x, y, z]: edges){
      graph[u][v] = z; //update weight as time
    }
  }
  for(int k = 0; k < locations.size(); k++){
    for(int i = 0; i < locations.size(); i++){
      for(int j = 0; j < locations.size(); j++){
        if(graph[i][k] != INT_MAX && graph[k][j] != INT_MAX && (graph[i][j] > graph[i][k] + graph[k][j])){
          graph[i][j] = graph[i][k] + graph[k][j]; //less time found
        }
      }
    }
  }
  cout << "Minimum time from " << locations[start] << " to " << locations[end] << " = ";
  cout << graph[start][end] << " hours\n";
  cout << flush;
}

int main() { //Main function
  ios::sync_with_stdio(false);
  cin.tie(0);
  loaddata("data.txt"); //Data.txt to read the location and routes info
  while(true){
    cout << flush;
    vector<int> curr = menu();
    if(curr[0] == 1){
      dij(curr[1], curr[2]);
    }else if(curr[0] == 2){
      bell(curr[1], curr[2]);
    }else if(curr[0] == 3){
      flo(curr[1], curr[2]);
    }else if(curr[0] == 4){
      printlocations();
    }else if(curr[0] == 5){
      cout << "Thanks for using KEKH. Goodbye!" << "\n";
      break;
    }else{
      cout << "---- Invalid choice ----";
      curr = menu();
    }
  }
}