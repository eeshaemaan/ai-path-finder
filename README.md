# Pathfinding Algorithms Comparison

## Overview

This project is a comprehensive demonstration of **pathfinding algorithms** applied to a randomly generated 2D grid. It is designed to showcase **search strategy efficiency, path cost, and execution time**. The project is ideal for portfolio or CV presentation, highlighting practical implementation skills in Python, algorithm design, and data visualization.

## Features

* **Random Grid Generation:** Generates a grid of customizable size with:

  * Start (`S`) and Goal (`G`) positions
  * Obstacles (`#`), Water (`~`), Ink areas (`&`), and normal traversable cells (`.`)
* **Search Algorithms Implemented:**

  1. Breadth-First Search (BFS)
  2. Depth-First Search (DFS)
  3. Iterative Deepening Search (IDS)
  4. Uniform Cost Search (UCS)
  5. A* Search (A*)
  6. Greedy Best-First Search (GBFS)
  7. Beam Search
* **Constraints Handled:**

  * **Claustrophobia Rule:** Limits movement in areas surrounded by more than 2 obstacles.
  * **Ink Rule:** Adds penalty cost when crossing continuous water (`~`) cells.
* **Visualization:**

  * Prints the grid with the computed path.
  * Displays a bar chart comparing **path cost/length** and **execution time** for all algorithms.

## Installation & Requirements

* **Python 3.x**
* Required Libraries:

  ```bash
  pip install matplotlib
  ```

## Usage

1. Clone or download the repository.
2. Run the Python script:

   ```bash
   python pathfinding_comparison.py
   ```
3. The program will output:

   * Path for each algorithm on the console.
   * Path cost/length and execution time.
   * A bar graph comparing all algorithms.

## Customization

* **Grid Size:** Adjust `ROWS` and `COLS` at the top of the script.
* **Terrain Probabilities:** Modify `WEIGHTS` for different distribution of obstacles, water, and ink areas.
* **Beam Width:** Adjust in the Beam Search function for exploration tuning.

## Project Highlights

* Demonstrates **algorithmic problem-solving** with multiple search strategies.
* Incorporates **realistic constraints** (claustrophobia and ink rules) to reflect practical scenarios.
* Visual and quantitative comparison of **efficiency and path optimality**.
* Fully **self-contained, modular Python script** suitable for portfolio showcase.

## Potential Applications

* Robotics navigation simulations.
* AI path planning in games or virtual environments.
* Educational tool for teaching search algorithms.

## Author

**[Your Name]**
Email: [[your.email@example.com](mailto:your.email@example.com)]
LinkedIn: [Your LinkedIn URL]
GitHub: [Your GitHub URL]

---

This project demonstrates strong proficiency in **Python programming, algorithm design, and data visualization**, making it an ideal portfolio piece for roles in AI, software development, or data analysis.
