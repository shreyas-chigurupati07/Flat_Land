# Flatland

## Project Overview

This project explores discrete motion planning for a point robot in a 2D grid environment, comparing the effectiveness of Depth First Search (DFS), Breadth First Search (BFS), Dijkstra's Algorithm, and a Random Planner in navigating through various obstacle densities.

## Problem Statement

The goal is to navigate a point robot from the top left to the bottom right corner of a 128x128 grid filled with randomly placed obstacles, using different planning algorithms to evaluate their performance in terms of path discovery and efficiency.

## Implementation Details

- **Environment:** A 128x128 grid with obstacles at densities ranging from 0% to 75%.
- **Planning Algorithms:** DFS, BFS, Dijkstra's Algorithm, and a Random Planner implemented in Python, utilizing NumPy for matrix operations and Matplotlib for visualization.
- **Performance Metrics:** Number of iterations to reach the goal and algorithm efficiency at various obstacle densities.

## Results

The project includes performance analysis across different obstacle densities, illustrating how each algorithm copes with increasing complexity in the environment.

<p float="left">
  <img src="![Screenshot 2023-09-17 at 9 42 47 PM](https://github.com/shreyas-chigurupati07/Flat_Land/assets/84034817/72ec2dc1-4fca-406b-ae85-f2338341193b)" width="100" />
  <img src="/img2.png" width="100" /> 
  <img src="/img3.png" width="100" />
</p>
### Path found by DFS
![Screenshot 2023-09-17 at 9 42 47 PM](https://github.com/shreyas-chigurupati07/Flat_Land/assets/84034817/72ec2dc1-4fca-406b-ae85-f2338341193b)
### Path found by DFS
![Screenshot 2023-09-17 at 9 54 14 PM](https://github.com/shreyas-chigurupati07/Flat_Land/assets/84034817/23839634-525d-489b-be8a-98aa3ef013c3)
### Path found by DFS
![Screenshot 2023-09-17 at 9 54 25 PM](https://github.com/shreyas-chigurupati07/Flat_Land/assets/84034817/9ffc7351-8aeb-4f23-a2be-a8bbf470324b)
### Path found by DFS
![Screenshot 2023-09-17 at 10 04 04 PM](https://github.com/shreyas-chigurupati07/Flat_Land/assets/84034817/72723ada-cc3c-425b-b6e6-7a650eaed454)

## Dependencies

- Python
- NumPy
- Matplotlib



## References

- Steven M. LaValle. Planning Algorithms. Cambridge University Press, May 2006.
