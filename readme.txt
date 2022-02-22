Link to repo: https://github.com/talkad/animation3D

Tal Kadosh 319126546
Yonatan Pisman 316262997

About our project:
we did the snake skinning and motion in three dimentional space (by using the keys: up, down, left, right, w and s).
we implemented two points of view - The first ome is a static top view and the other is from the snake point of view.
we created an interactive menu showing the current level, score, timer and an option for the player to pause and resume the game.
in the game there are three types of moving objects (the target): one moving on straightline, the other is bouncing and the last one moving on bezier curve.
the object moving on straight line giving the smallest amount of points and the bezier moving object the highest score.
The player can score object by colliding with them (the implementation used bounding object for each snake joint), or by clicking them with the mouse.
for each level the player must obtain certain amount of points before the timer is up, otherwise he lost the game.
when level passed the menu showing a next level button and when the game over it is showing a play again button.

Bonuses:
1. Fog - we used particle generator and object blending we it get away from the camera position.
2. moving object according to bezier curve
3. sound when level up, game over, when collecting a target and when a bouncy object bounced.
4. Interactive user interface using ImGui
5. gravity and bouncing objects
6. cubemap
7. heightmap
8. texture

Difficulties:
1. The main struggle was dealing with the snake collisions and camera position for snake point of view. It was complicated to define the snake points position after the skinning exection, 
we overcame this issue by defining for each joint a movable and we used MyTranslate and MyRotate to define its place. 
2. Another difficulty was to learn about shaders and to use them, the most difficult shader was the fog. we overcame it by reading alot of tutorials and deeply investigating this topic.


