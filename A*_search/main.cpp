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


/**
 * @brief Node struct to represent a cell in the grid.
 * 
 * Each node has x, y coordinates along with g (cost from start) and h (heuristic cost to goal).
 * 
 * @param x int: X-coordinate of the node.
 * @param y int: Y-coordinate of the node.
 * @param g int: Cost from the start node.
 * @param h int: Heuristic cost from the node to the goal node.
 */
struct Node {
    int x;
    int y;
    int g;
    int h;

    bool operator==(const Node& other) const {
    return x == other.x && y == other.y && g == other.g && h == other.h;
  }

};


/**
 * @brief Parse each line of the board file to a vector of State enums.
 * 
 * The function reads a line where 0 represents an empty space and 1 represents an obstacle.
 * It converts these values into State enum values.
 * 
 * @param line string: A string representing a line of the board file.
 * @return vector<State> A vector where each element is either State::kEmpty or State::kObstacle.
 */
auto ParseLine(const string& line) -> vector<State> {
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

/**
 * @brief Reads the board file line by line and converts it to a 2D vector of State enums.
 * 
 * @param path string: Path to the board file.
 * @return vector<vector<State>> A 2D vector representing the grid.
 * @throws std::runtime_error if the file cannot be opened.
 */
auto ReadBoardFile(string path) -> vector<vector<State>> {
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

/**
 * @brief Compare two nodes based on their f values (f = g + h).
 * 
 * @param node_1 const Node&: The first node.
 * @param node_2 const Node&: The second node.
 * @return bool True if f1 > f2, otherwise false.
 */
bool Compare(const Node& node_1, const Node& node_2){
  int f1 = node_1.g + node_1.h;
  int f2 = node_2.g + node_2.h;
  return f1 > f2; 
}

/**
 * @brief Sort the list of nodes in descending order based on their f value.
 * 
 * @param v vector<Node>&: A vector of Node structs.
 */
void CellSort(vector<Node>& v) {
  sort(v.begin(), v.end(), Compare);
}

/**
 * @brief Calculate the Manhattan distance heuristic between two points.
 * 
 * @param x1 int: The x-coordinate of the first point.
 * @param y1 int: The y-coordinate of the first point.
 * @param x2 int: The x-coordinate of the second point.
 * @param y2 int: The y-coordinate of the second point.
 * @return int The Manhattan distance between the two points.
 */
auto Heuristic (const int x1, const int y1, const int x2, const int y2) -> int {
  int distance = abs(x2 - x1) + abs(y2 - y1);
  return distance;
}

/**
 * @brief Check if a cell is valid (on the grid, not an obstacle, and not closed).
 * 
 * @param x int: The x-coordinate of the cell.
 * @param y int: The y-coordinate of the cell.
 * @param grid vector<vector<State>>&: The 2D vector representing the board.
 * @return bool True if the cell is valid, false otherwise.
 */
bool CheckValidCell(const int x, const int y, vector<vector<State>>& grid){
  bool on_grid_x = (x >= 0 && x < grid.size());
  bool on_grid_y = (y >= 0 && y < grid[0].size());
  return on_grid_x && on_grid_y && grid[x][y] == State::kEmpty;
}

/**
 * @brief Add a node to the open list and mark it as closed on the grid.
 * 
 * @param x int: The x-coordinate of the node.
 * @param y int: The y-coordinate of the node.
 * @param g int: The cost from the start node to this node.
 * @param h int: The heuristic value from this node to the goal.
 * @param open_nodes vector<Node>&: The vector of open nodes.
 * @param grid vector<vector<State>>&: The 2D vector representing the grid.
 */
void AddToOpen(int x, int y, int g, int h, vector<Node>& open_nodes, vector<vector<State>>& grid){
  Node node = {x, y, g, h};
  open_nodes.push_back(node);
  grid[x][y] = State::kClosed;
}

/**
 * @brief Expand the current node's neighbors and add valid ones to the open list.
 * 
 * @param current const Node&: The current node being expanded.
 * @param goal const array<int, 2>&: The goal node's coordinates.
 * @param open_nodes vector<Node>&: The vector of open nodes.
 * @param grid vector<vector<State>>&: The 2D vector representing the grid.
 */
void ExpandNeighbors(const Node& current, const array<int, 2>& goal, vector<Node>& open_nodes, vector<vector<State>>& grid){
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

/**
 * @brief Implementation of the A* search algorithm.
 * 
 * @param grid vector<vector<State>>&: The 2D vector representing the grid.
 * @param init const array<int, 2>&: The starting node's coordinates.
 * @param goal const array<int, 2>&: The goal node's coordinates.
 * @return vector<vector<State>> The modified grid with the path from start to goal.
 */
auto Search(vector<vector<State>>& grid, const array<int, 2>& init, const array<int, 2>& goal) -> vector<vector<State>> {
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

/**
 * @brief Translate a State enum value to a string representation.
 * 
 * @param cell const State&: The cell's state as a State enum.
 * @return string A string representation of the state (e.g., "0" for empty cell, "⛰️" for obstacle).
 */
string CellString(const State& cell) {
  switch(cell) {
    case State::kObstacle: return "⛰️    ";
    case State::kPath: return "🚗   ";
    case State::kStart: return "🚦   ";
    case State::kFinish: return "🏁   ";
    default: return "0    "; 
  }
}

/**
 * @brief Translate a State enum value to a string representation.
 * 
 * @param cell const State&: The cell's state as a State enum.
 * @return string A string representation of the state (e.g., "0" for empty cell, "⛰️" for obstacle).
 */
void PrintBoard (const vector<vector<State>>& board){
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