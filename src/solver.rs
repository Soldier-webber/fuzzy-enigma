// Maze solving module
// Implements the A* pathfinding algorithm to find the shortest solution

use crate::maze::{Maze, Cell};
use pathfinding::prelude::astar;

/// Represents the state of a position in the maze during solving
/// Includes the current position and the path taken to reach it
#[derive(Clone, Eq, PartialEq)]
struct State {
    position: (usize, usize),
}

/// Solves mazes using the A* pathfinding algorithm
pub struct MazeSolver {
    maze: Maze,
}

impl MazeSolver {
    /// Creates a new MazeSolver for the given maze
    /// 
    /// # Arguments
    /// * `maze` - The maze to solve
    pub fn new(maze: Maze) -> Self {
        MazeSolver { maze }
    }

    /// Solves the maze using the A* pathfinding algorithm
    /// 
    /// A* is a best-first search algorithm that combines:
    /// - g(n): the actual cost from the start node
    /// - h(n): a heuristic estimate to the goal (Manhattan distance)
    /// - f(n) = g(n) + h(n): total estimated cost
    /// 
    /// Returns Some(path) with the solution as a vector of coordinates,
    /// or None if no solution exists
    pub fn solve_astar(&self) -> Option<Vec<(usize, usize)>> {
        let start = (0, 0);
        let goal = (self.maze.width() - 1, self.maze.height() - 1);

        // Use A* to find the shortest path
        // The first element of the tuple is the path, the second is the cost
        let result = astar(
            &State {
                position: start,
            },
            |state| self.successors(state),      // Function to get neighbors
            |state| self.heuristic(state.position, goal), // Heuristic function
            |state| state.position == goal,      // Is goal check
        );

        // Extract the path from the result
        result.map(|(path, _cost)| {
            path.into_iter()
                .map(|state| state.position)
                .collect()
        })
    }

    /// Generates successor states (neighboring valid cells)
    /// 
    /// # Arguments
    /// * `state` - Current state
    /// 
    /// Returns a vector of (next_state, cost) tuples for valid moves
    fn successors(&self, state: &State) -> Vec<(State, usize)> {
        let (x, y) = state.position;
        let mut neighbors = Vec::new();

        // Define the four cardinal directions (up, down, left, right)
        let directions = [
            (x, y.saturating_sub(1)), // Up
            (x, y + 1),               // Down
            (x.saturating_sub(1), y), // Left
            (x + 1, y),               // Right
        ];

        // Check each neighbor
        for &(nx, ny) in &directions {
            // Verify the neighbor is within bounds
            if nx < self.maze.width() && ny < self.maze.height() {
                // Check if the neighbor is a path (not a wall)
                if let Some(Cell::Path) = self.maze.get_cell(nx, ny) {
                    let next_state = State {
                        position: (nx, ny),
                    };
                    // Each move costs 1
                    neighbors.push((next_state, 1));
                }
            }
        }

        neighbors
    }

    /// Heuristic function: Manhattan distance to goal
    /// 
    /// This estimates the minimum distance to the goal from a given position.
    /// The Manhattan distance is the sum of absolute differences in coordinates.
    /// 
    /// # Arguments
    /// * `pos` - Current position (x, y)
    /// * `goal` - Goal position (x, y)
    /// 
    /// Returns the estimated distance
    fn heuristic(&self, pos: (usize, usize), goal: (usize, usize)) -> usize {
        let (x1, y1) = pos;
        let (x2, y2) = goal;

        // Calculate Manhattan distance
        let dx = if x1 > x2 { x1 - x2 } else { x2 - x1 };
        let dy = if y1 > y2 { y1 - y2 } else { y2 - y1 };

        dx + dy
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heuristic() {
        let maze = Maze::new(10, 10);
        let solver = MazeSolver::new(maze);

        // Test Manhattan distance calculation
        assert_eq!(solver.heuristic((0, 0), (3, 4)), 7);
        assert_eq!(solver.heuristic((5, 5), (5, 5)), 0);
        assert_eq!(solver.heuristic((0, 0), (1, 1)), 2);
    }
}
