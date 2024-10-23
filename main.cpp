#include <algorithm> // for sort
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

using std::cout;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;
using std::abs;
using std::sort;

enum class State {kEmpty, kObstacle, kClosed, kPath, kStart, kFinish};

// directional deltas for expanding neighbors (down, left, up, right)
const int delta[4][2]{{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

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
    cout << "Error: Unable to open the file." << "\n";
    return {};
  }
}

bool Compare(vector<int>& node_1, vector<int>& node_2){
  /*
  Compare two nodes based on their f values, f = g + h

  param: node_1: vector of ints {x, y, g, h}, node 1
         node_2: vector of ints {x, y, g, h}, node 2
  */

  int f1 = node_1[2]+node_1[3];
  int f2 = node_2[2]+node_2[3];
  return f1 > f2; 
}


void CellSort(vector<vector<int>> *v) {
  /**
  Sort the two-dimensional vector of ints in descending order.
 
  param: v: pointer to a two-dimensional vector of ints. When calling the function, pass the address of the open vector.
 */
  sort(v->begin(), v->end(), Compare);
}

auto Heuristic (int x1, int y1, int x2, int y2) -> int {
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

bool CheckValidCell(int x, int y, vector<vector<State>>& grid){
  /*
  Check that a cell is valid: on the grid, not an obstacle, and clear(closed). 

  param: x: int, x coordinate of the cell
         y: int, y coordinate of the cell
         grid: 2D vector of State enum, representing the board
  */
  bool on_grid_x = (x >= 0 && x < grid.size());
  bool on_grid_y = (y >= 0 && y < grid[0].size());
  if (on_grid_x && on_grid_y && grid[x][y]== State::kEmpty){
  	return true;
  }
  else {
  return false;
  }
}

void AddToOpen(int x, int y, int g, int h, vector<vector<int>>& open_nodes, vector<vector<State>>& grid){
  /** 
  Add a node to the open list and mark it as open. 

  param: x: int, x coordinate of the cell
         y: int, y coordinate of the cell
         g: int, g value of the cell
         h: int, h value of the cell
         open_nodes: 2D vector of ints, representing the open list
         grid: 2D vector of State enum, representing the board
  */
  vector<int> node = {x, y, g, h};
  open_nodes.push_back(node);
  grid[x][y] = State::kClosed;
}


void ExpandNeighbors(const vector<int>& current, int goal[2], vector<vector<int>>& open_nodes, vector<vector<State>>& grid){
  /** 
  Add a node to the open list and mark it as open. 

  param: current: vector of ints {x, y, g, h}, representing the current node
         goal: int array, representing the goal node
         open_nodes: 2D vector of ints, representing the open list
         grid: 2D vector of State enum, representing the board
  */

  // Get current node's data.
	int x = current[0];
	int y = current[1];
	int g = current[2];
  // Loop through current node's potential neighbors.
  for (const auto& direction : delta){
    int x2 = x + direction[0];
    int y2 = y + direction[1];
  
    // Check that the potential neighbor's x2 and y2 values are on the grid and not closed.
    if (CheckValidCell(x2, y2, grid)) {
      // Increment g value, compute h value, and add neighbor to open list.
      int g2 = g + 1;
      int h2 = Heuristic(x2, y2, goal[0], goal[1]);
      AddToOpen(x2, y2, g2, h2, open_nodes, grid);
	  }
  }
}

vector<vector<State>> Search(vector<vector<State>> grid, int init[2], int goal[2]) {
  /** 
  Implementation of A* search algorithm

  param: grid: 2D vector of State enum, representing the board
         init: int array, representing the start node
         goal: int array, representing the goal node
  */

  // Create the vector of open nodes.
  vector<vector<int>> open {};
  
  // Initialize the starting node.
  int x = init[0];
  int y = init[1];
  int g = 0;
  int h = Heuristic(x, y, goal[0],goal[1]);
  AddToOpen(x, y, g, h, open, grid);

  while (open.size() > 0) {
    // Get the next node
    CellSort(&open);
    auto current = open.back();
    open.pop_back();
    x = current[0];
    y = current[1];
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
    ExpandNeighbors(current, goal, open, grid);
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
  int init[2] = {0,0};
  int goal[2] = {4,5};
  // int goal[2] = {0,5};
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