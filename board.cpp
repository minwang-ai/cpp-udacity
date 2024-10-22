#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using std::cout;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;

enum class State {kEmpty, kObstacle};

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

string CellString(const State& cell) {
  /*
  Trnaslate the State enum to a string, representing the cell state
  
  param: cell: State enum, kEmpty or kObstacle
  return: string representation of the cell state, "0" for empty cell, "⛰️" for obstacle
  */
  switch(cell) {
    case State::kObstacle: return "⛰️   ";
    default: return "0   "; 
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

int main() {
  auto board = ReadBoardFile("1.board");
  PrintBoard(board);
}