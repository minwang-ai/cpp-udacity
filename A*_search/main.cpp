#include <algorithm> // for sort
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <cmath>

using std::cout;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;
using std::array;
using std::abs;
using std::sort;

enum class State {kEmpty, kObstacle, kClosed, kPath, kStart, kFinish};

// directional deltas for expanding neighbors (down, left, up, right)
constexpr int delta[4][2]{{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

struct Node {
  /*
  Node struct to represent a cell in the grid
  x,y : coordinates of the cell
  g: cost from start
  h: heuristic value to the goal
  */
    int x;
    int y;
    int g;
    int h;

    bool operator==(const Node& other) const {
    return x == other.x && y == other.y && g == other.g && h == other.h;
}

};


auto ParseLine(const string& line) -> vector<State> {
    /*
    Parse each line of the board file to a vector of State enum
    
    Param: line: string, a line of the board file, 0 for empty cell, 1 for obstacle
    Return: vector of State enum, KEmpty for empty cell, KObstacle for obstacle
    */

    istringstream sline(line);
    int n;
    char c;

    vector<State> row;
    while (sline >> n >> c && c == ',') {
      if (n == 0) {
      	row.push_back(State::kEmpty);
      }
      else {
      	row.push_back(State::kObstacle);
      }
    }
    return row;
}

auto ReadBoardFile(string path) -> vector<vector<State>> {
  /*
  Read board file line by line and parse each line to a vector of State enum 
  
  param: path: string, path to the file
  return: 2D vector of State enum, representing the board
  */
  ifstream myfile (path);
  vector<vector<State>> board{};
  if (myfile) {
    string line;
    while (getline(myfile, line)) {
      vector<State> row = ParseLine(line);
      board.push_back(row);
    }
    return board;
  }
  else {
    throw std::runtime_error("Error: Unable to open the file.");
    return {};
  }
}

bool Compare(const Node& node_1, const Node& node_2){
  /*
  Compare two nodes based on their f values, f = g + h

  param: node_1: Node struct, representing the first node
         node_2: Node struct, representing the second node
  */

  int f1 = node_1.g + node_1.h;
  int f2 = node_2.g + node_2.h;
  return f1 > f2; 
}


void CellSort(vector<Node>& v) {
  /**
  Sort the open list of nodes based on the f value
 
  param: v: vector of Node structs
  */
  sort(v.begin(), v.end(), Compare);
}

auto Heuristic (const int x1, const int y1, const int x2, const int y2) -> int {
  /*
  Calculate the heuristic value h between two points (use manhattan disance)

  param: x1: int, x coordinate of the first point
         y1: int, y coordinate of the first point
         x2: int, x coordinate of the second point
         y2: int, y coordinate of the second point
  */
  int distance = abs(x2 - x1) + abs(y2 - y1);
  return distance;
}

bool CheckValidCell(const int x, const int y, vector<vector<State>>& grid){
  /*
  Check that a cell is valid: on the grid, not an obstacle, and clear(closed). 

  param: x: int, x coordinate of the cell
         y: int, y coordinate of the cell
         grid: 2D vector of State enum, representing the board
  */
  bool on_grid_x = (x >= 0 && x < grid.size());
  bool on_grid_y = (y >= 0 && y < grid[0].size());
  return on_grid_x && on_grid_y && grid[x][y] == State::kEmpty;
}

void AddToOpen(int x, int y, int g, int h, vector<Node>& open_nodes, vector<vector<State>>& grid){
  /** 
  Add a node to the open list and mark it as open. 

  param: x, y: coordinates of the node
         g: g value (cost from start)
         h: h value (heuristic to goal)
         open_nodes: vector of Node structs representing open nodes
         grid: the current state of the grid
  */
  Node node = {x, y, g, h};
  open_nodes.push_back(node);
  grid[x][y] = State::kClosed;
}


void ExpandNeighbors(const Node& current, const array<int, 2>& goal, vector<Node>& open_nodes, vector<vector<State>>& grid){
  /** 
  Add a node to the open list and mark it as open. 

  param  current: Node struct, representing the current node
         goal: int array, representing the goal node
         open_nodes: vector of Node structs representing open nodes
         grid: the current state of the grid
  */

  // Loop through current node's potential neighbors.
  for (const auto& direction : delta){
    int x2 = current.x + direction[0];
    int y2 = current.y + direction[1];
  
    // Check that the potential neighbor's x2 and y2 values are on the grid and not closed.
    if (CheckValidCell(x2, y2, grid)) {
      // Increment g value, compute h value, and add neighbor to open list.
      int g2 = current.g + 1;
      int h2 = Heuristic(x2, y2, goal[0], goal[1]);
      AddToOpen(x2, y2, g2, h2, open_nodes, grid);
	  }
  }
}

auto Search(vector<vector<State>>& grid, const array<int, 2>& init, const array<int, 2>& goal) -> vector<vector<State>> {
  /** 
  Implementation of A* search algorithm

  param: grid: 2D vector of State enum, representing the board
         init: int array, representing the start node
         goal: int array, representing the goal node
  */

  // Create the vector of open nodes.
  vector<Node> open_nodes;
  
  // Initialize the starting node.
  int x = init[0];
  int y = init[1];
  int g = 0;
  int h = Heuristic(x, y, goal[0],goal[1]);
  AddToOpen(x, y, g, h, open_nodes, grid);

  while (open_nodes.size() > 0) {
    // Get the next node
    CellSort(open_nodes);
    auto current = open_nodes.back();
    open_nodes.pop_back();
    x = current.x;
    y = current.y;
    grid[x][y] = State::kPath;

    // Check if we're done.
    if (x == goal[0] && y == goal[1]) {
      // Set the init grid cell to kStart, and 
      // set the goal grid cell to kFinish before returning the grid. 
      grid[init[0]][init[1]] = State::kStart;
      grid[goal[0]][goal[1]] = State::kFinish;
      return grid;
    }
    
    // If we're not done, expand search to current node's neighbors.
    ExpandNeighbors(current, goal, open_nodes, grid);
  }
  
  // We've run out of new nodes to explore and haven't found a path.
  cout << "No path found!" << "\n";
  return std::vector<vector<State>>{};
}

string CellString(const State& cell) {
  /*
  Trnaslate the State enum to a string, representing the cell state
  
  param: cell: State enum, kEmpty or kObstacle
  return: string representation of the cell state, "0" for empty cell, "⛰️" for obstacle
  */
  switch(cell) {
    case State::kObstacle: return "⛰️    ";
    case State::kPath: return "🚗   ";
    case State::kStart: return "🚦   ";
    case State::kFinish: return "🏁   ";
    default: return "0    "; 
  }
}


void PrintBoard (const vector<vector<State>>& board){
  /*
  Print the board to the console

  Param: board: 2D vector of State enum, representing the board
  */
  for (const auto& row: board){
  	for (const auto& element: row){
    cout << CellString(element) << " ";
    }
    cout << "\n";
  }
}

#include "test.cpp"
int main() {
  array<int, 2> init{0, 0};
  array<int, 2> goal{4, 5};
  auto board = ReadBoardFile("1.board");
  auto solution = Search(board, init, goal);

  PrintBoard(solution);
  // Tests
  TestHeuristic();
  TestAddToOpen();
  TestCompare();
  TestSearch();
  TestCheckValidCell();
  TestExpandNeighbors();
}