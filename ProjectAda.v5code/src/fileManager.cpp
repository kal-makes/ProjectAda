#include "vex.h"
using namespace vex;
extern brain Brain;
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
std::ofstream ofs;

using std::string;
int x = 0;
int file() {
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
    return 0;
}
int PIDinit(){
   if(Brain.SDcard.isInserted()){
    ofs.open("PIDVALUES.txt", std::ofstream::out | std::ios_base::app);
    ofs<<"|Lateral Error|"<<","<<"|Rotational Error|"<<","<<"|AVG Power (V)|"<<","<<"|Distance Val (deg)|"<<","<<"|Desired Distance (deg)|"<<","<<"|Turn Val (deg)|"<<","<<"|Desired Turn (deg)|"<<","<<"|Time (msec)|"<<std::endl;
    ofs.close();
  }
  return 1;
}

void PIDwrite(double lat_error, double rot_error, double avg_power, int counter, float distance_val, float desired_distance, float turn_val, float desired_turn){
  if( Brain.SDcard.isInserted()){
      ofs.open("PIDVALUES.csv", std::ofstream::out | std::ios_base::app);
      ofs<<lat_error<<","<<rot_error<<","<<","<<avg_power<<","<<distance_val<<","<<desired_distance<<","<<turn_val<<","<<desired_turn<<","<<counter<<std::endl;
      ofs.close();
  }
}
