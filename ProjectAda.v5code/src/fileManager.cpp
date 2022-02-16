#include "robot-config.h"

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>

std::ofstream ofs;

int main() {
    if( Brain.SDcard.isInserted() ) {
      // create a file with long filename
      ofs.open("a_long_filename_debug.txt", std::ofstream::out);
      ofs << "lorem ipsum\r\n";
      ofs << "this was a test of a file with long file name\r\n";
      ofs.close();

      Brain.Screen.printAt(10, 40, "done");
    }
    else {
      Brain.Screen.printAt(10, 40, "No SD Card");        
    }
}