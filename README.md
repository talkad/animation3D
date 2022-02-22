# EngineForAnimationCourse
Graphic Engine based on Libigl

For compiling:
1. Clone or download the project
2. Download Cmake from the link in the site
3. Run Cmake gui. choose the project folder and destination folder for the cpp project files. choose after pressing configure choose compiler (VS2019 for example). After finish configuration successfully, press configure again and after it finishes press generate. 
4. If everything pass successfully got to the destination folder and launch the project. 
5. Copy configuration.txt from tutorial/sandBox to build/tutorial/sandBox sandBox as a startup project and compile the project (it could take few mineutes);   


# About our project:
We did the snake skinning and motion in three dimensional space (by using the keys: up, down, left, right).
We implemented two points of view - The first one is a static top view and the other is from the snake point of view.
We created an interactive menu showing the current level, score, timer and an option for the player to pause and resume the game.
In the game there are three types of moving objects (the target): one moving in a straight line, the other is bouncing and the last one moving according to bezier curve.
The object moving in a straight line grants the lowest amount of points and the bezier curve movement object grants the highest score.
The player can score object by colliding with them (The implementation uses a bounding box per snake joint), or by clicking them with the mouse.
For each level the player must obtain certain a amount of points before the timer is up, otherwise he will lose the game.
When a level is passed the menu will show the next level button and when the game is over it will show a play again button.

# Bonuses:
1. Fog - We used a particle generator and object blending when the object is far enough from the camera position.
2. Object bezier curve movement.
3. Sound when you level up, lose the game, when collecting a target and when a bouncy hits the ground.
4. Interactive user interface using ImGui.
5. Gravity and bouncing objects.
6. Cubemap
7. Heightmap
8. Texture

# Difficulties:
1. The main struggle was dealing with the snake collisions and camera position for the snake point of view. 
It was complicated to define the snake points position after the skinning execution, 
we overcame this issue by defining a movable for each joint and we used MyTranslate and MyRotate to define its place. 
2. Another difficulty was to learn about shaders and how to use them, the most difficult shader was the fog, 
we overcame it by reading a lot of tutorials and deeply investigating this topic.
