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

int test_main();

#endif /* BFS_H_ */
