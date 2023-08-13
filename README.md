D* Path Planning Algorithm

This code implements the D* path planning algorithm using Python. The D* algorithm aims to find a path between a start and a goal point in a grid-based environment, accounting for obstacles and dynamic changes.

Code Overview:
* State: A class representing a state in the grid map. Each state has coordinates, a parent state, a state tag, and related attributes.
* Map: A class representing the grid map. It initializes the map with states, provides methods to get neighbors, and sets obstacles on the map.
* Dstar: The main D* algorithm class. It performs state processing, inserts and removes states from the open list, modifies state costs, and runs the algorithm.
* main(): The main function where the path planning process is executed. It loads a grayscale image as a map, converts it to an obstacle map, sets obstacles, defines the start and end points, and runs the D* algorithm.


Changes made:

The code starts by utilizing the existing D* example from the provided source. It extends its functionality by introducing a new function, create_obstacle_gridmap, which serves the purpose of transforming an image into an obstacle grid map. This function accepts parameters such as the image path, obstacle value, and free space value. In return, it provides essential information, including the dimensions of the map or image, and a list of obstacle positions.

While it was possible to incorporate the create_obstacle_gridmap function directly within the Map class by initializing the class instance with the image path, this approach was intentionally avoided. This decision was made to preserve the integrity of the original code, minimizing changes to the existing structure. Adopting this approach ensures that alterations to the Map class initialization and the removal of certain parts of the class could have been necessary. By maintaining the obstacle value and free values as variables, as opposed to hardcoded values, the code's adaptability for potential future enhancements remains intact.

Prerequisites:

The code requires the numpy, matplotlib, and cv2 (OpenCV) libraries to be installed.

Usage
1. Clone the repository containing the code to your local machine:
2. Navigate to the code directory:
3. Modify the image_path variable in the main() function to point to the path of the image you want to use as the map.
4. Set the start and goal coordinates in the main() function to define the start and end points for path planning.
5. Optionally, set show_animation to True to visualize the intermediate steps and final path on the grid map.
6. Run the script:

The code will execute the D* path planning algorithm, display the map with obstacles, the start and goal points, and the final path.

Notes
The D* algorithm works with grid-based maps where each cell can be either an obstacle or free space.
The code also provides visualization options to display the map, obstacles, start and goal points, intermediate steps, and the final path.
