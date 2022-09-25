This is the candidate README file with explanation of my solution to the Tesla Coding Challenge.

In addition to "main.cpp", "network.h" and "network.cpp", there are also other classes and structs 
that I have written to break down this problem into smaller subsections.

"a_star.h" and "astar.cpp" contain the class Astar which is a framework for the A* search algorithm.

"structs.h" contains three structs that are helpful in solving this Challenge

"path.h" and "path.cpp" contain the class Path which stores the waypoints from initial to goal chargers.

To compile use:
    g++ -std=c++11 -O1 main.cpp network.cpp util.cpp path.cpp a_star.cpp -o candidate_solution

To run use:
    ./candidate_solution <initial charger> <final charger>

The result will be displayed in the same Terminal where "candidate_solution" is run.