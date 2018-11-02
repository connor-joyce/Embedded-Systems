/*
 * bfs.h
 *
 *  Created on: Oct 31, 2018
 *      Author: root
 */


#ifndef BFS_H_
#define BFS_H_

#define MAZE_WIDTH (10)
#define ELEMENT_COUNT (MAZE_WIDTH*MAZE_WIDTH)

#define RANK(row, col) ((row)*MAZE_WIDTH+(col))
#define ROW(rank) ((rank)/MAZE_WIDTH)
#define COL(rank) ((rank)%MAZE_WIDTH)

void find_shortest_path(int start_rank, int goal_rank,
        const int obstacles[]);
void find_shortest_path_a(int start_rank, int goal_rank,
        const int obstacles[]);

int test_main();

#endif /* BFS_H_ */
