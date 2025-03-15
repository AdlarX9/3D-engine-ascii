#ifndef three_hpp
#define three_hpp

#include <stdio.h>

#include <vector>
#include <list>
#include <map>
#include <iostream>
#include <functional>



// Les quaternions sont des nombres à la base de mon moteur 3D
// Ils me servent à représenter des points, des rotations et des translations
// plus d'informations sur Internet.
class Quaternion {
public:
    Quaternion(double w = 0, double i = 0, double j = 0, double k = 0);
    // accesseurs et modifieurs
    void w(double w);
    double w() const;
    void x(double x);
    double x() const;
    void y(double y);
    double y() const;
    void z(double z);
    double z() const;
    // retourne la distance a l'origine (la norme dans le cas d'un vecteur)
    double norm() const;
    // règle cette distance
    void norm(double norm);
    // règle la norme à 1
    void normalize();
    virtual void set(double w = 0, double x = 0, double y = 0, double z = 0);
    virtual void afficher(std::ostream& flow) const;
    // si on a w + ai + bj + ck, cette méthode retourne w - ai - bj - ck
    // pratique pour calculer des rotations
    Quaternion inverse() const;
    ~Quaternion();
    virtual Quaternion& operator*=(Quaternion const& quaternion);
    virtual Quaternion& operator+=(Quaternion const& quaternion);
    virtual Quaternion& operator-=(Quaternion const& quaternion);
    virtual Quaternion& operator*=(double factor);
    virtual Quaternion& operator/=(double factor);
protected:
    double m_w;
    double m_x;
    double m_y;
    double m_z;
};
Quaternion operator*(Quaternion const& quaternion1, Quaternion const& quaternion2);
Quaternion operator+(Quaternion const& quaternion1, Quaternion const& quaternion2);
Quaternion operator-(Quaternion const& quaternion1, Quaternion const& quaternion2);
Quaternion operator*(Quaternion quaternion, double factor);
Quaternion operator/(Quaternion quaternion, double divider);
Quaternion operator*(double factor, Quaternion quaternion);
Quaternion operator/(double divider, Quaternion quaternion);
bool operator==(Quaternion quaternion1, Quaternion quaternion2);
std::ostream& operator<<(std::ostream& flow, Quaternion const& quaternion);


// Les quaternions unitaires représentent les rotations de maniere compacte.
// Pour mieux les comprendre, allez sur eater.net/quaternions
// Ce n'est pas un cours de maths.
class UnitQuaternion : public Quaternion {
public:
    UnitQuaternion(double angle = 0, double x = 0, double y = 0, double z = 0);
    UnitQuaternion(Quaternion quaternion);
    void set(double angle, double x = 0, double y = 0, double z = 0);
    ~UnitQuaternion();
};


// représente un point dans l'espace
class Point : public Quaternion {
public:
    Point(double x = 0, double y = 0, double z = 0);
    Point(Quaternion quaternion);
    void set(double x = 0, double y = 0, double z = 0);
    ~Point();
};


// représente un vecteur
class Vector : public Quaternion {
public:
    Vector(double x = 0, double y = 0, double z = 0);
    Vector(Point point1, Point point2);
    Vector(Point point);
    Vector(Vector& vector);
    Vector(Quaternion quaternion);
    // retourne vrai si 2 vecteurs ont des sens opposés
    bool isOpposedTo(Vector vector) const;
    // retourne le produit scalaire
    double scalarProduct(Vector const& vector) const;
    // retourne l'angle entre deux vecteurs
    double angleTo(Vector const& vector) const;
    ~Vector();
};
// Ici, pleins d'opérateurs pour éviter les ambigüités au niveau du compilateur
Vector operator*(Vector const& vector, double const& factor);
Vector operator*(double const& factor, Vector const& vector);
Vector operator*(Vector const& vector, Quaternion const& quaternion);
Vector operator*(Quaternion const& quaternion, Vector const& vector);
Vector operator+(Vector const& vector, Quaternion const& quaternion);
Vector operator-(Vector const& vector, Quaternion const& quaternion);
Vector operator+(Quaternion const& quaternion, Vector const& vector);
Vector operator-(Quaternion const& quaternion, Vector const& vector);
Vector operator+(Vector const& vector1, Vector const& vector2);
Vector operator-(Vector const& vector1, Vector const& vector2);



// représente une face
class Face {
public:
    Face(std::vector<Point> face);
    // calcule l'équation de plan
    void setEquation();
    // retourne l'équation de plan
    Quaternion equation() const;
    // retourne le nombre de points
    int size() const;
    void afficher(std::ostream& flow) const;
    Point operator[](unsigned int index);
    Face& operator+=(Vector const& vector);
    Face& operator+=(Quaternion const& quaternion);
    ~Face();
private:
    std::vector<Point> m_points; // les points de la face
    Quaternion m_equation; // c'est l'équation de plan de la face
};
Face operator+(Face const& face, Vector const& vector);
Face operator+(Face const& face, Quaternion const& quaternion);
std::ostream& operator<<(std::ostream& flow, Face const& face);


// représente un objet en 3D
class Mesh {
public:
    Mesh(std::vector<Point> points, std::vector<std::vector<int> > faces);
    // apliquent une translation à l'objet
    void apply(Point point);
    void apply(Vector Vector);
    // fait tourner l'objet autour de ses propres axes
    void rotateSelf(double angle, double x = 0, double y = 0, double z = 0);
    // fait tourner l'objet autour de la scène
    void rotateScene(double angle, double x = 0, double y = 0, double z = 0);
    // retourne le nombre de faces
    double size() const;
    // retourne le centre de l'objet
    Point center() const;
    // retourne le vecteur de translation de l'objet
    Vector Vector() const;
    // retourne le quaternion unitaire de l'objet
    UnitQuaternion quaternion() const;
    Face face(int index) const;
    Face operator[](unsigned int index);
    ~Mesh();
protected:
    UnitQuaternion m_quaternion;
    class Vector m_vector;
    std::vector<Point> m_points; // les points de l'objet
    std::vector<std::vector<int> > m_faces; // les faces de l'objet, chaque point est désigné par son index dans m_points
};


// représente une lumière. Pas de lumière, tu vois rien.
class Light {
public:
    Light(double intensity);
    virtual void set(double x = 0, double y = 0, double z = 0) = 0;
    // Retourne le rayon de lumière que reçoit un point donné
    virtual Vector ray(Point point) const = 0;
    // fixe et retourne l'intensité de la lumière
    void intensity(double intensity);
    double intensity() const;
    virtual ~Light();
protected:
    double m_intensity;
    // plusieurs attributs sont nécessaires, mais cela laisse une grande liberté aux classes filles
};





// représente la scène 3D, l'endroit.
class Scene {
public:
    Scene(double ambientLight = 0.1);
    // ajoute un objet
    void add(Mesh& mesh);
    // la lumière
    void add (Light& light);
    // retire un objet de la scène
    void remove(Mesh& mesh);
    // retourne le nombre d'objet que contient la scène
    int size() const;
    // retourne le nombre de faces qu'il y a dans la scène
    int nbrFaces() const;
    void ambientLight(double ambientLight);
    double ambientLight() const;
    std::list<Mesh*>::iterator end();
    std::list<Mesh*>::iterator begin();
    std::list<Mesh*> meshes() const;
    Light& light() const;
    Mesh& operator[](int index) const;
    ~Scene();
private:
    std::list<Mesh*> m_meshes; // Ses objets
    Light* m_light; // Sa lumière
    double m_ambientLight; // Sa lumière ambiente
};


// représente la caméra
class Camera {
public:
    Camera(unsigned int const width, unsigned int const height, double fov, double max = 1000);
    // fixe sa position
    void position(double x, double y, double z);
    // retourne sa position
    Point& position();
    // permet de tourner la caméra
    void rotateSelf(double angle, double x = 0, double y = 0, double z = 0);
    void rotateScene(double angle, double x = 0, double y = 0, double z = 0);
    // déplace la caméra
    void translateX(double length);
    void translateY(double length);
    void translateZ(double length);
    // détermine vers où regarde la caméra
    void lookAt(double x = 0, double y = 0, double z = 0);
    void lookAt(Point point);
    void setVectors();
    Vector direction() const;
    Vector toRight() const;
    Vector toDown() const;
    Vector v00() const;
    // retourne le vecteur projeté de la caméra passant par le pixel (x ; y)
    Vector ray(unsigned int x, unsigned int y) const;
    double width() const;
    void width(double width);
    double height() const;
    void height(double height);
    double max() const;
    void fov(double fov);
    double fov() const;
    ~Camera();
private:
    unsigned int m_height; // le nombre de lignes
    unsigned int m_width; // le nombre de caractères dans une ligne
    double m_fov; // l'angle de vue horizontal, entre la limite gauche et la limite droite
    double m_max; // distance maximale à laquelle la caméra peut voir
    Point m_position; // sa position
    Vector m_direction; // son vecteur directeur, résultant de .lookAt()
    
    // quelques vecteurs pour s'éviter des calculs :
    Vector m_toRight;
    Vector m_toDown;
    Vector m_v00;
};


// représente le rendu 3D, l'image.
class Renderer {
public:
    // Il est constitué d'une scène et d'une caméra
    Renderer(Scene& scene, Camera& camera);
    Renderer();
    void scene(Scene scene);
    Scene& scene();
    void camera(Camera camera);
    Camera& camera();
    void render();
    // génère l'image constitué des intensités lumineuses
    std::vector< std::vector<double> > create();
    // affiche l'image dans le terminal
    void print(std::vector<std::vector<double> > intensities) const;
    ~Renderer();
private:
    Scene m_scene; // la scène
    Camera m_camera; // la caméra
    // retourne l'intensité lumineuses reçu par la caméra pour un pixel et une face précise
    std::pair<const double, double> intensity(Point p0, Vector v, Face face, Light& light, double ambientLight, double max) const;
    // tableau de caractères, se matérialisent par des nuances de gris
    static unsigned int const matSize;
    static char const materials[];
};





// quelques lumières :

class SunLight : public Light {
public:
    SunLight(double intensity, double x = 0, double y = 0, double z = 0);
    void set(double x = 0, double y = 0, double z = 1);
    Vector ray(Point point) const;
    ~SunLight();
private:
    Vector m_ray;
};


class PointLight : public Light {
public:
    PointLight(double intensity, double x = 0, double y = 0, double z = 0);
    void set(double x = 0, double y = 0, double z = 1);
    Vector ray(Point point) const;
    ~PointLight();
private:
    Point m_position;
};


// Quelques Mesh :

class Cube : public Mesh {
public:
    Cube(double length = 1);
    ~Cube();
};


class Plane : public Mesh {
public:
    Plane(double x = 1, double y = 1);
    ~Plane();
};





// une action est un mouvement que va faire la caméra
// exemple : une translation d'un cube vers la droite de 1 qui dure 2 secondes
// et est en retard de -0.5 secondes (donc en avance)
class Action {
public:
    Action(std::function<void(double t)> func, double time, double lateness = 0);
    std::function<void(double t)> func() const;
    double time() const;
    double lateness() const;
    ~Action();
private:
    std::function<void(double t)> m_func; // la fonction anonyme qui décrit le mouvement
    // le paramètre t représente la linéarité, il évolue de 0 à 1 tout au long du mouvement
    double m_time;
    double m_lateness;
};


// représente une vidéo
class TimeLine {
public:
    TimeLine(Renderer& renderer, std::vector<Action> const& actions, double fps = 9);
    TimeLine(Renderer& renderer);
    // génère la vidéo
    std::vector<std::vector<std::vector<double> > > create() const;
    // retourne le temps de la vidéo
    double time() const;
    // calcule et joue directement la vidéo
    void play() const;
    // calcule et sauvegrde la vidéo dans le chemin indiqué
    void save(std::string path) const;
    // lit et joue la vidéo situé dans le chemin indiqué
    void play(std::string path) const;
    ~TimeLine();
private:
    Renderer& m_renderer;
    std::vector<Action> m_actions;
    int m_fps;
};



#endif
