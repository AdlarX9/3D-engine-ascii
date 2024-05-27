#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <vector>

using namespace std;

#include "three.hpp"

int main() {
    
    // Exemple de scène 3D :
    
    Scene scene;
    Camera camera(150, 50, 110);
    
    camera.position(-2, 2, 1.5);
    camera.lookAt(0, 0, 0);
    
    PointLight light(1, 0, 0, 1);
    scene.add(light);


    Cube cube;
    scene.add(cube);


    int l = 2;
    Plane plane(l, l);
    scene.add(plane);


    Renderer renderer(scene, camera);
    
    
    // Exemple de vidéo possible :

    vector<Action> actions = {
        
        Action([ &cube, &plane ](double t) {
            cube.rotateSelf(90 / (9 * 4), -1, 1, 0);
            plane.rotateSelf(120 / (9 * 4), 0, -1, 2);
        }, 3),
        
        Action([ &cube, &plane ](double t) {
            cube.rotateSelf(90 / (9 * 4), 0, 1, 1);
            plane.rotateScene((-100 - 20 * t) / (9 * 4), -2, 1, -1);
        }, 3, -3)

    };

    TimeLine tl(renderer, actions);
    const string p = "/Users/Alexis/Documents/Code/C++/three-in-the-terminal/three-in-the-terminal/video.txt";
    tl.play(); // note : "tl.play(p)" joue la vidéo précisé dans le chemin
    

    return 0;
}
