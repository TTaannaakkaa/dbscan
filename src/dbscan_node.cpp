//  Copyright 2025 Takuma Tanaka

#include "dbscan/dbscan.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dbscan");
  DBSCAN dbscan;
  dbscan.process();
  return 0;
}