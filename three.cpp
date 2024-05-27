#include <cmath>
#include <thread>
#include <chrono>
#include <fstream>

#include "three.hpp"

using namespace std;





Quaternion::Quaternion(double w, double x, double y, double z) : m_w(w), m_x(x), m_y(y), m_z(z) {}

void Quaternion::w(double w) {
    this->set(w, this->x(), this->y(), this->z());
}

void Quaternion::x(double x) {
    this->set(this->w(), x, this->y(), this->z());
}

void Quaternion::y(double y) {
    this->set(this->w(), this->x(), y, this->z());
}

void Quaternion::z(double z) {
    this->set(this->w(), this->x(), this->y(), z);
}

double Quaternion::w() const {
    return m_w;
}

double Quaternion::x() const {
    return m_x;
}

double Quaternion::y() const {
    return m_y;
}

double Quaternion::z() const {
    return m_z;
}

double Quaternion::norm() const {
    return sqrt(m_w*m_w + m_x*m_x + m_y*m_y + m_z*m_z);
}

void Quaternion::norm(double norm) {
    this->normalize();
    m_w *= norm;
    m_x *= norm;
    m_y *= norm;
    m_z *= norm;
}

void Quaternion::normalize() {
    double n = this->norm();
    if (n != 0) {
        m_x /= n;
        m_y /= n;
        m_z /= n;
        m_w /= n;
    }
}

void Quaternion::set(double w, double x, double y, double z) {
    m_w = w;
    m_x = x;
    m_y = y;
    m_z = z;
}

void Quaternion::afficher(std::ostream& flux) const {
    flux << '(' << m_w << " ; " << m_x << " ; " << m_y << " ; " << m_z << ')';
}

Quaternion Quaternion::inverse() const {
    return Quaternion(m_w, -m_x, -m_y, -m_z);
}


Quaternion& Quaternion::operator*=(Quaternion const& q) {
    double w(m_w), x(m_x), y(m_y), z(m_z), w1(q.w()), x1(q.x()), y1(q.y()), z1(q.z());
    m_w = w * w1 - x * x1 - y * y1 - z * z1;
    m_x = w * x1 + x * w1 + y * z1 - z * y1;
    m_y = w * y1 + y * w1 - x * z1 + z * x1;
    m_z = w * z1 + z * w1 + x * y1 - y * x1;
    return *this;
}

Quaternion& Quaternion::operator+=(Quaternion const& q) {
    m_w += q.w();
    m_x += q.x();
    m_y += q.y();
    m_z += q.z();
    return *this;
}

Quaternion& Quaternion::operator-=(Quaternion const& q) {
    m_w -= q.w();
    m_x -= q.x();
    m_y -= q.y();
    m_z -= q.z();
    return *this;
}

Quaternion& Quaternion::operator*=(double factor) {
    m_w *= factor;
    m_x *= factor;
    m_y *= factor;
    m_z *= factor;
    return *this;
}

Quaternion& Quaternion::operator/=(double divider) {
    m_w /= divider;
    m_x /= divider;
    m_y /= divider;
    m_z /= divider;
    return *this;
}


Quaternion operator*(Quaternion const& q1, Quaternion const& q2) {
    Quaternion result(q1);
    result *= q2;
    return result;
}

Quaternion operator+(Quaternion const& q1, Quaternion const& q2) {
    Quaternion result(q1);
    result += q2;
    return result;
}

Quaternion operator-(Quaternion const& q1, Quaternion const& q2) {
    Quaternion result(q1);
    result -= q2;
    return result;
}

bool operator==(Quaternion q1, Quaternion q2) {
    return q1.w() == q2.w() and q1.x() == q2.x() and q1.y() == q2.y() and q1.z() == q2.z();
}

Quaternion operator+(Quaternion q1, Quaternion const& q2) {
    Quaternion result(q1);
    return result += q2;
}

Quaternion operator-(Quaternion q1, Quaternion const& q2) {
    Quaternion result(q1);
    return result -= q2;
}

Quaternion operator*(Quaternion q, double factor) {
    Quaternion result(q);
    return result *= factor;
}

Quaternion operator*(double factor, Quaternion q)  {
    Quaternion result(q);
    return result *= factor;
}

Quaternion operator/(Quaternion q, double divider) {
    Quaternion result(q);
    return result /= divider;
}

Quaternion operator/(double divider, Quaternion q) {
    Quaternion result(q);
    return result /= divider;
}

ostream& operator<<(ostream& flow, Quaternion const& quaternion) {
    quaternion.afficher(flow);
    return flow;
}

Quaternion::~Quaternion() {}




UnitQuaternion::UnitQuaternion(double angle, double x, double y, double z) : Quaternion::Quaternion(std::cos(angle / 360 * M_PI),
    x * std::sin(angle / 360 * M_PI), y * std::sin(angle / 360 * M_PI), z * std::sin(angle / 360 * M_PI))
{
    this->normalize();
}

UnitQuaternion::UnitQuaternion(Quaternion q) : Quaternion::Quaternion(q.w(), q.x(), q.y(), q.z()) {
    this->normalize();
}

void UnitQuaternion::set(double angle, double x, double y, double z) {
    double cos = std::cos(angle / 360 * M_PI);
    double sin = std::sin(angle / 360 * M_PI);
    m_w = cos;
    m_x = x * sin;
    m_y = y * sin;
    m_z = z * sin;
    this->normalize();
}

UnitQuaternion::~UnitQuaternion() {}




Point::Point(double x, double y, double z) : Quaternion::Quaternion(0, x, y, z) {}
Point::Point(Quaternion q) : Quaternion::Quaternion(0, q.x(), q.y(), q.z()) {}

void Point::set(double x, double y, double z) {
    m_w = 0;
    m_x = x;
    m_y = y;
    m_z = z;
}

Point::~Point() {}




Vector::Vector(double x, double y, double z) : Quaternion::Quaternion(0, x, y, z) {}
Vector::Vector(Point pt1, Point pt2) : Quaternion::Quaternion(0, pt2.x() - pt1.x(), pt2.y() - pt1.y(), pt2.z() - pt1.z()) {}
Vector::Vector(Point pt) : Quaternion::Quaternion(0, pt.x(), pt.y(), pt.z()) {}
Vector::Vector(Vector& v) : Quaternion::Quaternion(0, v.x(), v.y(), v.z()) {}
Vector::Vector(Quaternion q) : Quaternion::Quaternion(0, q.x(), q.y(), q.z()) {}

Vector operator*(Vector const& u, Vector const& v) {
    Vector w(
            u.y() * v.z() - u.z() * v.y(),
            u.z() * v.x() - u.x() * v.z(),
            u.x() * v.y() - u.y() * v.x()
    );
    return w;
}

bool Vector::isOpposedTo(Vector v) const {
    Vector u(*this);
    u.normalize();
    v.normalize();
    return abs(u.x() + v.x()) < 1e-13 and abs(u.y() + v.y()) < 1e-13 and abs(u.z() + v.z()) < 1e-13;
}

double Vector::scalarProduct(Vector const& v) const {
    return m_x * v.x() + m_y * v.y() + m_z * v.z();
}

double Vector::angleTo(Vector const& v) const {
    
    // u . v / (||u|| * ||v||)
    double argument = this->scalarProduct(v) / (this->norm() * v.norm());

    // Assurez-vous que l'argument est dans la plage valide [-1, 1]
    if (argument <= -1.0) {
        return M_PI;  // Angle de 180 degrés (pi radians) si le cosinus est -1
    } else if (argument >= 1.0) {
        return 0.0;   // Angle de 0 degré si le cosinus est 1
    } else {
        return acos(argument);
    }
}

Vector operator*(Vector const& u, double const& factor) {
    return Vector(u.x() * factor, u.y() * factor, u.z() * factor);
}

Vector operator*(double const& factor, Vector const& u) {
    return Vector(u.x() * factor, u.y() * factor, u.z() * factor);
}

Vector operator*(Vector const& vector, Quaternion const& quaternion) {
    Vector result(vector);
    return result *= quaternion;
}

Vector operator*(Quaternion const& quaternion, Vector const& vector) {
    Vector result(vector);
    return result *= quaternion;
}

Vector operator+(Vector const& vector, Quaternion const& quaternion) {
    Vector result(vector);
    return result += quaternion;
}

Vector operator-(Vector const& vector, Quaternion const& quaternion) {
    Vector result(vector);
    return result -= quaternion;
}

Vector operator+(Quaternion const& quaternion, Vector const& vector) {
    Vector result(vector);
    return result += quaternion;
}

Vector operator-(Quaternion const& quaternion, Vector const& vector) {
    Vector result(vector);
    return result -= quaternion;
}

Vector operator+(Vector const& vector1, Vector const& vector2) {
    Vector result(vector1);
    return result += vector2;
}

Vector operator-(Vector const& vector1, Vector const& vector2) {
    Vector result(vector1);
    return result -= vector2;
}

Vector::~Vector() {}




Face::Face(vector<Point> face) : m_points(face), m_equation(Quaternion()) {
    this->setEquation();
}

void Face::setEquation() {
    Face& face = *this;
    Vector n = Vector(face[0], face[1]) * Vector(face[1], face[2]);
    double  a(n.x()), b(n.y()), c(n.z());

    // d = -ax - by - cz en prenant (x ; y ; z) un point de la face
    double d = -a * face[0].x() - b * face[0].y() - c * face[0].z();
    m_equation = Quaternion(d, a, b, c);
}

Quaternion Face::equation() const {
    return m_equation;
}

Point Face::operator[](unsigned int index) {
    return m_points[index];
}

int Face::size() const {
    int counter(0);
    for (int i = 0; i < m_points.size(); i++) { counter++; }
    return counter;
}

void Face::afficher(std::ostream& flow) const {
    flow << '{' << endl;
    for (int i = 0; i < this->size(); i++) {
        flow << "    " << m_points[i] << endl;
    }
    flow << '}' << endl;
}

Face& Face::operator+=(Vector const& v) {
    for (int i = 0; i < m_points.size(); i++) {
        double x = m_points[i].x();
        x += v.x();
        m_points[i].x(x);
        double y = m_points[i].y();
        y += v.y();
        m_points[i].y(y);
        double z = m_points[i].z();
        z += v.z();
        m_points[i].z(z);
    }
    return *this;
}

Face& Face::operator+=(Quaternion const& q) {
    vector<Point> tab(m_points);
    for (int i = 0; i < tab.size(); i++) {
        tab[i] += q;
    }
    m_points = tab;
    return *this;
}

Face operator+(Face const& face, Vector const& v) {
    Face result(face);
    result += v;
    return result;
}

Face operator+(Face const& face, Quaternion const& q) {
    Face r(face);
    return r += q;
}

ostream& operator<<(ostream& flow, Face const& face) {
    face.afficher(flow);
    return flow;
}

Face::~Face() {}




Mesh::Mesh(vector<Point> points, vector<vector<int> > faces) :
    m_points(points), m_faces(faces), m_vector(Vector()), m_quaternion(UnitQuaternion())
{}

void Mesh::apply(Point point) {
    m_vector = point;
}

void Mesh::apply(class Vector Vector) {
    m_vector += Vector;
}

void Mesh::rotateSelf(double angle, double x, double y, double z) {
    double cos = std::cos(angle / 360 * M_PI);
    double sin = std::sin(angle / 360 * M_PI);
    Quaternion q(cos, x * sin, y * sin, z * sin);
    m_quaternion = m_quaternion * q;
}

void Mesh::rotateScene(double angle, double x, double y, double z) {
    double cos = std::cos(angle / 360 * M_PI);
    double sin = std::sin(angle / 360 * M_PI);
    Quaternion q(cos, x * sin, y * sin, z * sin);
    m_quaternion = q * m_quaternion;
    m_vector = q * m_vector * q.inverse();
}

double Mesh::size() const {
    return m_faces.size();
}

Point Mesh::center() const {
    Point p;
    return p += m_vector;
}

Vector Mesh::Vector() const {
    return m_vector;
}

UnitQuaternion Mesh::quaternion() const {
    return m_quaternion;
}

Face Mesh::face(int i) const {
    vector<Point> tab;
    for (int j = 0; j < m_faces[i].size(); j++) {
        tab.push_back(m_quaternion * m_points[m_faces[i][j]] * m_quaternion.inverse() + m_vector);
    }
    Face face(tab);
    return face;
}

Face Mesh::operator[](unsigned int i) {
    return this->face(i);
}

Mesh::~Mesh() {}




Light::Light(double intensity) : m_intensity(intensity) {}

void Light::intensity(double intensity) {
    m_intensity = intensity;
}

double Light::intensity() const {
    return m_intensity;
}

Light::~Light() {}






Scene::Scene(double ambientLight) : m_ambientLight(ambientLight) {}

void Scene::add(Mesh& mesh) {
    m_meshes.push_back(&mesh);
}

void Scene::add(Light& light) {
    m_light = &light;
}

void Scene::remove(Mesh& mesh) {
    for (auto it = m_meshes.begin(); it != m_meshes.end();) {
        if (*it == &mesh) {
            it = m_meshes.erase(it);
        } else {
            ++it;
        }
    }
}

int Scene::size() const {
    int counter(0);
    for (int i = 0; i < m_meshes.size(); i++) { counter++; }
    return counter;
}

int Scene::nbrFaces() const {
    int nbr(0);
    
    for (const auto& mesh : m_meshes) {
        for (int i = 0; i < mesh->size(); i++) {
            nbr++;
        }
    }
    
    return nbr;
}

void Scene::ambientLight(double ambientLight) {
    m_ambientLight = ambientLight;
}

double Scene::ambientLight() const {
    return m_ambientLight;
}

list<Mesh*>::iterator Scene::end() {
    return m_meshes.end();
}

list<Mesh*>::iterator Scene::begin() {
    return m_meshes.begin();
}

list<Mesh*> Scene::meshes() const {
    return m_meshes;
}

Light& Scene::light() const {
    return *m_light;
}

Mesh& Scene::operator[](int index) const {
    auto it = m_meshes.begin();
    std::advance(it, index);
    return **it;
}

Scene::~Scene() {}




Camera::Camera(unsigned int const width, unsigned int const height, double fov, double max) :
    m_height(height), m_width(width), m_fov(fov * M_PI / 180), m_max(max), m_position(Point()),
    m_direction(Vector(0, 0, -m_width / 2 / tan(m_fov / 2)))
{
    this->setVectors();
}

void Camera::position(double x, double y, double z) {
    m_position.set(x, y, z);
}

Point& Camera::position() {
    return m_position;
}

void Camera::rotateSelf(double angle, double x, double y, double z) {
    UnitQuaternion q(angle, x, y, z);
    this->lookAt((q * m_direction * q.inverse()) + m_position);
}

void Camera::rotateScene(double angle, double x, double y, double z) {
    UnitQuaternion q(angle, x, y, z);
    Point lookingAt = m_position + m_direction;
    this->lookAt((q * lookingAt * q.inverse()));
    m_position = q * m_position * q.inverse();
}

void Camera::translateX(double l) {
    m_position.set(m_position.x() + l, m_position.y(), m_position.z());
}

void Camera::translateY(double l) {
    m_position.set(m_position.x(), m_position.y() + l, m_position.z());
}

void Camera::translateZ(double l) {
    m_position.set(m_position.x(), m_position.y(), m_position.z() + l);
}

void Camera::lookAt(double x, double y, double z) {
    m_direction = Vector(m_position, Point(x, y, z));
    m_direction.norm(m_width / 2 / tan(m_fov / 2));
    this->setVectors();
}

void Camera::lookAt(Point p) {
    m_direction = Vector(m_position, p);
    m_direction.norm(m_width / 2 / tan(m_fov / 2));
    this->setVectors();
}

void Camera::setVectors() {
    
    Vector toRight(m_direction.y(), -m_direction.x());
    toRight.norm(m_width / 2 * 12 / 25);

    Vector toDown = (toRight * -1) * m_direction;
    toDown.norm(m_height / 2);
    
    Vector v00 = m_direction - toRight - toDown;
    
    toRight.normalize();
    toDown.normalize();
    
    m_toRight = toRight;
    m_toDown = toDown;
    m_v00 = v00;
}

Vector Camera::direction() const {
    return m_direction;
}

Vector Camera::toRight() const {
    return m_toRight;
}

Vector Camera::toDown() const {
    return m_toDown;
}

Vector Camera::v00() const {
    return m_v00;
}

Vector Camera::ray(unsigned int x, unsigned int y) const {
    Vector vxy = m_v00 + m_toRight * x * 12 / 25 + m_toDown * y;
    return vxy;
}

double Camera::width() const {
    return m_width;
}

void Camera::width(double width) {
    m_width = width;
    this->setVectors();
}

double Camera::height() const {
    return m_height;
}

void Camera::height(double height) {
    m_height = height;
    this->setVectors();
}

double Camera::max() const {
    return m_max;
}

void Camera::fov(double fov) {
    m_fov = fov;
}

double Camera::fov() const {
    return m_fov;
}

Camera::~Camera() {}




Renderer::Renderer(Scene& scene, Camera& camera) : m_scene(scene), m_camera(camera) {}
Renderer::Renderer() : m_scene(Scene()), m_camera(Camera(1, 1, 1)) {}

const unsigned int Renderer::matSize = 12;

const char Renderer::materials[matSize] = { ' ', '.', ',', '-', ':', ';', '/', '{', 'f', '3', '@', '$' };

void Renderer::scene(Scene scene) {
    m_scene = scene;
}

Scene& Renderer::scene() {
    return m_scene;
}

void Renderer::camera(Camera camera) {
    m_camera = camera;
}

Camera& Renderer::camera() {
    return m_camera;
}

void Renderer::render() {
    this->print(this->create());
}

vector<vector<double> > Renderer::create() {
    vector<vector<double> > intensities(m_camera.height(), vector<double>());
    
    Point position = m_camera.position();
    Light& light = m_scene.light();
    double ambientLight = m_scene.ambientLight();
    double max = m_camera.max();
    
    for (int y = 0; y < m_camera.height(); y++) {
        for (int x = 0; x < m_camera.width(); x++) {
            map<double, double> intersections;
            Vector ray = m_camera.ray(x, y);
            for (const auto& mesh : m_scene.meshes()) {
                for (int i = 0; i < mesh->size(); i++) {
                    pair<const double, double> pair(this->intensity(
                        position,
                        ray,
                        mesh->face(i),
                        light,
                        ambientLight,
                        max
                    ));
                    intersections[pair.first] = pair.second;
                }
            }
            intensities[y].push_back(intersections.begin()->second);
        }
    }
    return intensities;
}

// p0 : Point de départ du rayon projeté par la caméra
// v : vecteur directionnel définissant la droite et ayant pour origine p0
// face : face que l'on veut tester
// light : le vecteur lumière afin de déterminer quel quantité de lumière reçoit la caméra
// max : distance d'affichage maximale
pair<const double, double> Renderer::intensity(Point p0, Vector v, Face face, Light& light, double ambientLight, double max) const {
    
    // => étape 1 : déterminer le point d'intersection entre la droite et le plan formé par la face
    Quaternion e = face.equation();
    double  d(e.w()), a(e.x()), b(e.y()), c(e.z()), // a ; b ; c et d sont 4 définissent l'équation cartésienne du plan
            ap(v.x()), bp(v.y()), cp(v.z()), // (a' ; b' ; c') définiseent le vecteur directionnel de la droite;
            x0(p0.x()), y0(p0.y()), z0(p0.z()); // x0 ; y0 et z0 sont les coordonnées du point de départ de la droite
    
    // a(x0 + a't) + b(y0 + b't) + c(z0 + c't) + d = 0 avec (a' ; b' ; c') étant le vecteur directionnel v
    double t = (-a * x0 - b * y0 - c * z0 - d) / (a * ap + b * bp + c * cp);
    
    // Point d'intersection M défini selon l'équation de la droite
    Point M(
        x0 + ap * t,
        y0 + bp * t,
        z0 + cp * t
    );

    // => étape 2 : savoir si le point M est dans la face en posant M le barycentre de tous les triangles de la face
    
    // On itère pour balayer tous les triangles qui constituent la face
    Point A(face[0]);
    for (int i = 2; i < face.size(); i++) {
        Point B(face[i - 1]), C(face[i]);
        
        Vector MA(M, A), MB(M, B), MC(M, C);
        double xa(MA.x()), xb(MB.x()), xc(MC.x()),
               ya(MA.y()), yb(MB.y()), yc(MC.y());
        
        Vector normal = Vector(A, B) * Vector(B, C);
        
        if (abs(normal.z()) < 1e-5) {
            ya = MA.z(); yb = MB.z(); yc = MC.z();
        }

        if (abs(normal.y()) < 1e-5 and abs(normal.z()) < 1e-5) {
            xa = MA.y(); xb = MB.y(); xc = MC.y();
        }

        // résolution du système à 3 équations 3 inconnues posé par :
        // weightA * MA + weightB * MB + weightC * MC = 0
        if (yb - yc == 0) { yc += 1e-5; } // on augmente de 1e-5 pour éviter les divisions par 0
        if ((xa - xc) * (yb - yc) + (ya - yc) * (xc - xb) == 0) { xa += 1e-5; }
        double weightA = (yc * (xb - xc) - xc * (yb - yc)) / ((xa - xc) * (yb - yc) + (ya - yc) * (xc - xb));
        double weightB = (-weightA * (ya - yc) - yc) / (yb - yc);
        double weightC = 1 - weightA - weightB;
        
        // si les poids de A, B et C sont positifs, M est dans le triangle
        if (weightA >= 0 and weightB >= 0 and weightC >= 0) {

            Vector AB(A, B), BC(B, C), normal(AB * BC);
            if (v.angleTo(normal) > M_PI / 2) { normal *= -1; }
            
            // Projection orthogonale de u sur la normale = u . v / ||v|| * v
            Vector proj = v.scalarProduct(normal) / normal.norm() * normal;
            
            // On calcule le rayon de la caméra refléchi sur la face
            Vector reflectedRay = v + 2 * proj;
            
            double reflection = reflectedRay.angleTo(light.ray(M) * -1) + ambientLight / 10;
            double faceLight = normal.angleTo(light.ray(M) * -1) + ambientLight / 10;
            if (normal.angleTo(light.ray(M) * -1) <= M_PI / 2) {
                faceLight *= ambientLight * 5;
                reflection = 0;
            }
            
            double intensity = (reflection + faceLight) * light.intensity() / (2 * M_PI);
            
            // On retourne l'intensité de la lumière reçu par la caméra sous la forme d'un quotient
            return pair<const double, double>(Vector(p0, M).norm(), intensity);
            
        } else { continue; }
    }
    // Le point ne passe pas par la face : pas de lumière reçue
    return pair<const double, double>(max, 0);
}

void Renderer::print(vector<vector<double> > intensities) const {
    for(int y = 0; y < intensities.size(); y++) {
        for (int x = 0; x < intensities[y].size(); x++) {
            
            int index = ceil(intensities[y][x] * matSize);
            
            if (index > matSize - 1) { index = matSize - 1; }
            
            cout << materials[index];
        }
        cout << endl;
    }
}

Renderer::~Renderer() {}





SunLight::SunLight(double i, double x, double y, double z) : Light::Light(i), m_ray(Vector(-x, -y, -z)) {}

void SunLight::set(double x, double y, double z) {
    m_ray.x(-x);
    m_ray.y(-y);
    m_ray.z(-z);
}

Vector SunLight::ray(Point p) const {
    return m_ray;
}

SunLight::~SunLight() {}




PointLight::PointLight(double i, double x, double y, double z) : Light::Light(i), m_position(Point(x, y, z)) {}

void PointLight::set(double x, double y, double z) {
    m_position = Point(x, y, z);
}

Vector PointLight::ray(Point p) const {
    return Vector(m_position, p);
}

PointLight::~PointLight() {}






Cube::Cube(double l) : Mesh::Mesh(vector<Point>({
    Point(-0.5 * l, -0.5 * l, -0.5 * l), Point(0.5 * l, -0.5 * l, -0.5 * l), Point(0.5 * l, 0.5 * l, -0.5 * l), Point(-0.5 * l, 0.5 * l, -0.5 * l),
    Point(-0.5 * l, 0.5 * l, 0.5 * l), Point(-0.5 * l, -0.5 * l, 0.5 * l), Point(0.5 * l, -0.5 * l, 0.5 * l), Point(0.5 * l, 0.5 * l, 0.5 * l)
}), vector<vector<int> >({
    { 0, 1, 2, 3 }, { 0, 5, 6, 1 }, { 1, 6, 7, 2 }, { 2, 7, 4, 3 }, { 0, 5, 4, 3 }, { 4, 5, 6, 7 }
})) {}

Cube::~Cube() {}




Plane::Plane(double x, double y) : Mesh::Mesh(vector<Point>({
    Point(-0.5 * x, -0.5 * y), Point(0.5 * x, -0.5 * y), Point(0.5 * x, 0.5 * y), Point(-0.5 * x, 0.5 * y)
}), vector<vector<int> >({
    { 0, 1, 2, 3 }
})) {}

Plane::~Plane() {}





Action::Action(function<void(double t)> func, double time, double lateness) : m_func(func), m_time(time), m_lateness(lateness) {}

function<void(double t)> Action::func() const {
    return m_func;
}

double Action::time() const {
    return m_time;
}

double Action::lateness() const {
    return m_lateness;
}

Action::~Action() {}




TimeLine::TimeLine(Renderer& renderer, vector<Action> const& actions, double fps) :
    m_renderer(renderer), m_actions(actions), m_fps(fps)
{}

TimeLine::TimeLine(Renderer& renderer) : m_renderer(renderer) {}

double TimeLine::time() const {
    double time(0);
    for (int i = 0; i < m_actions.size(); i++) {
        time += m_actions[i].time() + m_actions[i].lateness();
    }
    return time;
}

vector<vector<vector<double> > > TimeLine::create() const {
    cout << "calculations in progress." << endl;
    double t = m_renderer.camera().width() * m_renderer.camera().height() * m_renderer.scene().nbrFaces() * m_fps * this->time()
    / 36000000 * 95718;
    cout << "estimated time : " << t / 1000 << 's' << endl;
    
    vector<vector<vector<double> > > video;
    
    double time(this->time());
    int images = time * m_fps;
    
    if (t >= 4700) { cout << string(images - 1, '=') << endl; }
    for (int i = 0; i < images; i++) {
        if (i != 0 and t >= 4700) { cout << '*'; }
        for (int a = 0; a < m_actions.size(); a++) {
            
            double begin(0), end;
            for (int b = 0; b < a; b++) {
                begin += m_actions[b].time() + m_actions[b].lateness();
            }
            begin += m_actions[a].lateness();
            end = begin + m_actions[a].time();
            
            double tSpent = (i * time) / images;
            if (tSpent >= begin and tSpent <= end) {
                m_actions[a].func()((i - begin * m_fps) / (m_fps * (end - begin)));
            } else if (tSpent >= begin - m_actions[a].lateness() and tSpent < begin) {
                m_actions[a].func()(0);
            }
        }
        video.push_back(m_renderer.create());
    }
    cout << endl;
    
    return video;
}

void TimeLine::play() const {
    auto start = std::chrono::high_resolution_clock::now();
    vector<vector<vector<double> > > video = this->create();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    cout << string(m_renderer.camera().height() / 5, '\n');
    for (int i = 0; i < video.size(); i++) {
        m_renderer.print(video[i]);
        this_thread::sleep_for(chrono::milliseconds(1000 / m_fps));
    }
    
    cout << "calculated in " << duration << "ms" << endl;
}

void TimeLine::save(string path) const {
    try {
        ofstream flow(path.c_str());
        
        if (flow) {
            
            auto start = std::chrono::high_resolution_clock::now();
            vector<vector<vector<double> > > video = this->create();
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            
            flow << video.size() << ' ' << video[0].size() << ' ' << video[0][0].size() << ' ' << endl;
            
            for (int t = 0; t < video.size(); t++) {
                for (int y = 0; y < video[t].size(); y++) {
                    for (int x = 0; x < video[t][y].size(); x++) {
                        flow << video[t][y][x] << ' ';
                    }
                }
            }
            cout << "Writing completed!" << endl;
            cout << "calculated in " << duration << "ms" << endl;
            
        }
    } catch (exception const& e) {
        cerr << "ERREUR : " << e.what() << endl;
    }
}

void TimeLine::play(string path) const {
    try {
        ifstream flow(path.c_str());
        
        if (flow) {
            int images, height, width;
            flow >> images;
            flow >> height;
            flow >> width;
            
            vector<vector<vector<double> > > video(images, vector<vector<double> >(height, vector<double>()));
            
            double value;
            for (int t = 0; t < images; t++) {
                for (int y = 0; y < height; y++) {
                    for (int x = 0; x < width; x++) {
                        flow >> value;
                        video[t][y].push_back(value);
                    }
                }
            }
            
            for (int i = 0; i < images; i++) {
                m_renderer.print(video[i]);
                this_thread::sleep_for(chrono::milliseconds(1000 / m_fps));
            }
            
        }
    } catch (exception const& e) {
        cerr << "ERREUR : " << e.what() << endl;
    }
}

TimeLine::~TimeLine() {}

