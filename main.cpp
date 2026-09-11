/*
 * TRABALHO PRÁTICO - BOIDS (Computação Gráfica)
 *
 * Compilação no macOS:
 * clang++ -o boids main.cpp -framework OpenGL -framework GLUT -Wno-deprecated
 *
 * --- CONTROLES ---
 * Movimento:  Setas ou W/A/S/D (Frente/Esq/Trás/Dir) ou Z (Trás)
 * Q, E:       Subir / Descer o Alvo
 * 1, 2, 3, 4: Alternar Câmeras (1:Aérea 2:Atrás 3:Lateral 4:Torre)
 * +, -:       Adicionar / Remover Boids
 * F:          Ligar/Desligar Fog (Neblina)
 * P:          Pausar/Despausar Simulação
 * S:          Também funciona como Step (passo-a-passo) quando pausado
 * O:          Adicionar Obstáculo Aleatóriot
 * ESC:        Sair
 */

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iostream>

// --- CONSTANTES E CONFIGURAÇÕES ---
const float PI = 3.14159265359;
const int WIDTH = 800;
const int HEIGHT = 600;

// Parâmetros do comportamento dos Boids
float RAIO_VISAO = 60.0f;
float DIST_SEPARACAO = 50.0f;
float VELOCIDADE_MAX = 0.3f;
float FORCA_MAX = 0.02f;
float RAIO_OBSTACULO = 15.0f; // Raio visual do obstáculo para desvio

// --- ESTRUTURAS DE DADOS ---

struct Vec3 {
    float x, y, z;

    Vec3(float _x = 0, float _y = 0, float _z = 0) : x(_x), y(_y), z(_z) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }

    void operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; }

    float magnitude() const { return sqrt(x*x + y*y + z*z); }

    void normalize() {
        float m = magnitude();
        if (m > 0) { x /= m; y /= m; z /= m; }
    }

    static float dist(const Vec3& a, const Vec3& b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
    }

    // Produto Vetorial
    static Vec3 cross(const Vec3& a, const Vec3& b) {
        return Vec3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    // Produto Escalar
    static float dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
};

struct Obstacle {
    Vec3 position;
    float radius;
};

// --- CLASSE BOID ---
class Boid {
public:
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    Vec3 lastLateralForce; // Usado para o Banking (inclinação)
    float wingAngle;
    float wingSpeed;

    Boid(float x, float y, float z) {
        position = Vec3(x, y, z);
        velocity = Vec3((rand()%100 - 50)/100.0f, (rand()%100 - 50)/100.0f, (rand()%100 - 50)/100.0f);
        velocity.normalize();
        acceleration = Vec3(0, 0, 0);
        lastLateralForce = Vec3(0, 0, 0);
        wingAngle = 0;
        wingSpeed = 0.5f;
    }

    void update(const std::vector<Boid>& boids, Vec3 target, const std::vector<Obstacle>& obstacles) {
        Vec3 sep = separation(boids);
        Vec3 ali = alignment(boids);
        Vec3 coh = cohesion(boids);
        Vec3 seek = seekTarget(target);
        Vec3 avoid = avoidObstacles(obstacles);

        // Pesos
        sep = sep * 1.5f;
        ali = ali * 1.0f;
        coh = coh * 1.0f;
        seek = seek * 0.8f;
        avoid = avoid * 3.0f; // Prioridade alta para não bater

        // Evitar chão
        Vec3 avoidGround(0,0,0);
        if (position.y < 5.0f) avoidGround = Vec3(0, 1.0f, 0) * 2.0f;

        acceleration = sep + ali + coh + seek + avoid + avoidGround;

        // Calcular força lateral para banking antes de atualizar velocidade
        // A força lateral é a componente da aceleração perpendicular à velocidade
        Vec3 velNorm = velocity;
        velNorm.normalize();
        Vec3 forwardComp = velNorm * Vec3::dot(acceleration, velNorm);
        lastLateralForce = acceleration - forwardComp;

        velocity += acceleration;

        if (velocity.magnitude() > VELOCIDADE_MAX) {
            velocity.normalize();
            velocity = velocity * VELOCIDADE_MAX;
        }

        position += velocity;

        // Impedir que atravesse o chão
        if (position.y < 1.0f) {
            position.y = 1.0f;
            velocity.y = abs(velocity.y) * 0.5f; // Rebate para cima
        }

        // Asa
        wingAngle += wingSpeed;
    }

    Vec3 separation(const std::vector<Boid>& boids) {
        Vec3 steer(0, 0, 0);
        int count = 0;
        for (const auto& other : boids) {
            float d = Vec3::dist(position, other.position);
            if (d > 0 && d < DIST_SEPARACAO) {
                Vec3 diff = position - other.position;
                diff.normalize();
                diff = diff / d;
                steer += diff;
                count++;
            }
        }
        if (count > 0) steer = steer / (float)count;
        if (steer.magnitude() > 0) {
            steer.normalize();
            steer = steer * VELOCIDADE_MAX;
            steer = steer - velocity;
            if (steer.magnitude() > FORCA_MAX) {
                steer.normalize();
                steer = steer * FORCA_MAX;
            }
        }
        return steer;
    }

    Vec3 alignment(const std::vector<Boid>& boids) {
        Vec3 sum(0, 0, 0);
        int count = 0;
        for (const auto& other : boids) {
            float d = Vec3::dist(position, other.position);
            if (d > 0 && d < RAIO_VISAO) {
                sum += other.velocity;
                count++;
            }
        }
        if (count > 0) {
            sum = sum / (float)count;
            sum.normalize();
            sum = sum * VELOCIDADE_MAX;
            Vec3 steer = sum - velocity;
            if (steer.magnitude() > FORCA_MAX) {
                steer.normalize();
                steer = steer * FORCA_MAX;
            }
            return steer;
        }
        return Vec3(0,0,0);
    }

    Vec3 cohesion(const std::vector<Boid>& boids) {
        Vec3 sum(0, 0, 0);
        int count = 0;
        for (const auto& other : boids) {
            float d = Vec3::dist(position, other.position);
            if (d > 0 && d < RAIO_VISAO) {
                sum += other.position;
                count++;
            }
        }
        if (count > 0) {
            sum = sum / (float)count;
            return seekTarget(sum);
        }
        return Vec3(0,0,0);
    }

    Vec3 seekTarget(Vec3 target) {
        Vec3 desired = target - position;
        desired.normalize();
        desired = desired * VELOCIDADE_MAX;
        Vec3 steer = desired - velocity;
        if (steer.magnitude() > FORCA_MAX) {
            steer.normalize();
            steer = steer * FORCA_MAX;
        }
        return steer;
    }

    // Desviar de obstáculos (Esferas)
    Vec3 avoidObstacles(const std::vector<Obstacle>& obstacles) {
        Vec3 steer(0,0,0);
        for(const auto& obs : obstacles) {
            float d = Vec3::dist(position, obs.position);
            // Se estiver perto de colidir (raio do obstáculo + margem)
            if (d < obs.radius + RAIO_OBSTACULO) {
                Vec3 diff = position - obs.position;
                diff.normalize();
                // Força inversamente proporcional à distância
                steer += diff * (VELOCIDADE_MAX / (d * 0.1f));
            }
        }
        return steer;
    }
};

// --- VARIÁVEIS GLOBAIS ---
std::vector<Boid> flock;
std::vector<Obstacle> obstacles;
Vec3 targetPos(120, 40, 120); // Alvo na mesma região dos pássaros
int cameraMode = 1;
float groundSize = 2000.0f;

// Luz
GLfloat light_pos[] = { 50.0f, 250.0f, 50.0f, 1.0f }; // Posicional para sombras

// Estados
bool fogEnabled = false;
bool paused = false;
bool singleStep = false;

// Controle de teclas pressionadas para movimento suave
bool keyW = false, keyA = false, keyS = false, keyD = false, keyZ = false;
bool keyQ = false, keyE = false;
bool keyUp = false, keyDown = false, keyLeft = false, keyRight = false;
bool keyPageUp = false, keyPageDown = false;

// --- FUNÇÕES DE DESENHO ---

// Desenha o modelo geométrico do Boid
void drawBoidGeometry() {
    // Corpo
    glPushMatrix();
    glScalef(1.0f, 1.0f, 2.0f);
    glutSolidCone(1.0, 2.0, 6, 2);
    glPopMatrix();

    // Asas
    glBegin(GL_TRIANGLES);
    glVertex3f(0, 0, 0); glVertex3f(3, 0, -1); glVertex3f(0, 0, 1.5); // Esq
    glVertex3f(0, 0, 0); glVertex3f(-3, 0, -1); glVertex3f(0, 0, 1.5); // Dir
    glEnd();
}

void drawBoid(Boid& b) {
    glPushMatrix();
    glTranslatef(b.position.x, b.position.y, b.position.z);

    // --- BANKING (10% da nota) ---
    // Construir um sistema de coordenadas local
    Vec3 forward = b.velocity;
    forward.normalize();

    Vec3 globalUp(0, 1, 0);
    // O vetor "Up" ideal inclina-se na direção da força lateral (curva)
    // Se estiver virando para a esquerda, a força é para esquerda, então o Up inclina para esquerda
    Vec3 bankingUp = globalUp + (b.lastLateralForce * 15.0f); // 15.0 é fator de exagero visual
    bankingUp.normalize();

    // Recalcular vetores base
    Vec3 right = Vec3::cross(forward, bankingUp); // Vetor lateral (asa a asa)
    right.normalize();
    Vec3 up = Vec3::cross(right, forward); // Vetor cima real corrigido
    up.normalize();

    // Criar matriz de rotação 4x4 manualmente a partir dos vetores
    // Coluna 1: Right, Coluna 2: Up, Coluna 3: Forward (positivo para ponta seguir movimento)
    // OpenGL usa column-major
    float rotMatrix[16] = {
        right.x, right.y, right.z, 0,
        up.x,    up.y,    up.z,    0,
        forward.x, forward.y, forward.z, 0, // Forward positivo - cone aponta para +Z
        0,       0,       0,       1
    };

    // Aplicar rotação baseada no frame construído
    glMultMatrixf(rotMatrix);

    // Animação de bater asas (apenas rotação local das asas se quisesse sofisticar, aqui rotaciona levemente o corpo)
    float flap = sin(b.wingAngle) * 5.0f;
    glRotatef(flap, 0, 0, 1); // Pequena oscilação no Roll

    // Cor
    float mat_diffuse[] = { 0.7f, 0.85f, 0.98f, 1.0f };
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

    drawBoidGeometry();

    glPopMatrix();
}

// Gera matriz de sombra projetada no plano Y=0
void shadowMatrix(GLfloat shadowMat[4][4], GLfloat groundplane[4], GLfloat lightpos[4]) {
    GLfloat dot;
    // Produto escalar entre plano e luz
    dot = groundplane[0] * lightpos[0] + groundplane[1] * lightpos[1] +
          groundplane[2] * lightpos[2] + groundplane[3] * lightpos[3];

    shadowMat[0][0] = dot - lightpos[0] * groundplane[0];
    shadowMat[1][0] = 0.f - lightpos[0] * groundplane[1];
    shadowMat[2][0] = 0.f - lightpos[0] * groundplane[2];
    shadowMat[3][0] = 0.f - lightpos[0] * groundplane[3];

    shadowMat[0][1] = 0.f - lightpos[1] * groundplane[0];
    shadowMat[1][1] = dot - lightpos[1] * groundplane[1];
    shadowMat[2][1] = 0.f - lightpos[1] * groundplane[2];
    shadowMat[3][1] = 0.f - lightpos[1] * groundplane[3];

    shadowMat[0][2] = 0.f - lightpos[2] * groundplane[0];
    shadowMat[1][2] = 0.f - lightpos[2] * groundplane[1];
    shadowMat[2][2] = dot - lightpos[2] * groundplane[2];
    shadowMat[3][2] = 0.f - lightpos[2] * groundplane[3];

    shadowMat[0][3] = 0.f - lightpos[3] * groundplane[0];
    shadowMat[1][3] = 0.f - lightpos[3] * groundplane[1];
    shadowMat[2][3] = 0.f - lightpos[3] * groundplane[2];
    shadowMat[3][3] = dot - lightpos[3] * groundplane[3];
}

void drawObstacles() {
    float mat_obs[] = { 1.0f, 0.2f, 0.2f, 1.0f }; // Vermelho
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_obs);

    for (const auto& obs : obstacles) {
        glPushMatrix();
        glTranslatef(obs.position.x, obs.position.y, obs.position.z);
        glutSolidSphere(obs.radius, 16, 16);
        glPopMatrix();
    }
}

void drawTarget() {
    glPushMatrix();
    glTranslatef(targetPos.x, targetPos.y, targetPos.z);
    float mat_yellow[] = { 1.0f, 1.0f, 0.0f, 1.0f }; // Amarelo
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_yellow);
    glutWireSphere(1.5, 10, 10); // Wireframe para diferenciar
    glPopMatrix();
}

void drawEnvironment() {
    // Chão
    float mat_green[] = { 0.1f, 0.3f, 0.1f, 1.0f };
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_green);

    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    // Grade simples para noção de velocidade
    for(float x = -groundSize; x < groundSize; x += 20) {
        for(float z = -groundSize; z < groundSize; z += 20) {
            glVertex3f(x, 0, z);
            glVertex3f(x, 0, z+18);
            glVertex3f(x+18, 0, z+18);
            glVertex3f(x+18, 0, z);
        }
    }
    glEnd();

    // Torre central
    float mat_gray[] = { 0.6f, 0.6f, 0.6f, 1.0f };
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_gray);
    glPushMatrix();
    glRotatef(-90, 1, 0, 0);
    glutSolidCone(15.0, 100.0, 10, 10);
    glPopMatrix();
    
    // Lua
    float mat_moon[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_moon);
    glMaterialfv(GL_FRONT, GL_EMISSION, mat_moon); // Emite luz própria
    glPushMatrix();
    glTranslatef(150.0f, 200.0f, -150.0f);
    glutSolidSphere(30.0, 30, 30);
    glPopMatrix();
    
    // Resetar emissão
    float no_emission[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glMaterialfv(GL_FRONT, GL_EMISSION, no_emission);
}

// --- LÓGICA E CALLBACKS ---

void init() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE); // Importante para scaling

    GLfloat light_ambient[] = { 0.4f, 0.4f, 0.45f, 1.0f };
    GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };

    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);

    glClearColor(0.05f, 0.1f, 0.2f, 1.0f);

    // --- CONFIGURAÇÃO DE FOG (5%) ---
    GLfloat fogColor[] = {0.05f, 0.1f, 0.2f, 1.0f};
    glFogfv(GL_FOG_COLOR, fogColor);
    glFogi(GL_FOG_MODE, GL_EXP2);
    glFogf(GL_FOG_DENSITY, 0.002f);
    glHint(GL_FOG_HINT, GL_NICEST);

    // Inicializa Boids - começam em aglomerado distante do centro
    for(int i=0; i<30; i++) {
        flock.push_back(Boid(100 + (rand()%40), 30 + (rand()%20), 100 + (rand()%40)));
    }

    // Inicializa Obstáculos
    obstacles.push_back({Vec3(30, 20, 30), 10.0f});
    obstacles.push_back({Vec3(-40, 15, -20), 12.0f});
}

Vec3 getFlockCenter() {
    if (flock.empty()) return Vec3(0,0,0);
    Vec3 sum(0,0,0);
    for (const auto& b : flock) sum += b.position;
    return sum / (float)flock.size();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Aplica Fog se habilitado
    if (fogEnabled) glEnable(GL_FOG);
    else glDisable(GL_FOG);

    Vec3 center = getFlockCenter();

    // Câmeras
    if (cameraMode == 1) { // Torre - atrás e acima para controles intuitivos
        gluLookAt(center.x, center.y + 150, center.z + 200, center.x, center.y, center.z, 0, 1, 0);
    }
    else if (cameraMode == 2) { // Atrás
        Vec3 eye = center - (Vec3(0,0,1) * 120.0f); // Mais distante
        eye.y += 40.0f;
        gluLookAt(eye.x, eye.y, eye.z, center.x, center.y, center.z, 0, 1, 0);
    }
    else if (cameraMode == 3) { // Lateral
        gluLookAt(center.x + 120, center.y + 20, center.z, center.x, center.y, center.z, 0, 1, 0);
    }
    else if (cameraMode == 4) { // Topo da Torre Central
        gluLookAt(0, 100, 0, center.x, center.y, center.z, 0, 1, 0);
    }

    // Atualizar luz para que ela fique fixa no mundo
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

    drawEnvironment();
    drawObstacles();
    drawTarget();

    // --- DESENHO DOS BOIDS ---
    for (auto& b : flock) {
        drawBoid(b);
    }

    // --- SOMBRAS (5%) ---
    glDisable(GL_LIGHTING); // Sombra não tem iluminação própria
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.0f, 0.0f, 0.0f, 0.3f); // Sombra escura sutil

    GLfloat shadowMat[4][4];
    GLfloat groundPlane[4] = {0, 1, 0, -0.01f}; // Plano Y=0.01 ligeiramente acima do chão

    shadowMatrix(shadowMat, groundPlane, light_pos);

    glPushMatrix();
        glMultMatrixf((GLfloat*)shadowMat);
        // Desenha apenas a geometria dos boids achatada
        for (auto& b : flock) {
            glPushMatrix();
            glTranslatef(b.position.x, b.position.y, b.position.z);
            // Rotacionar sombra igual ao boid
            Vec3 forward = b.velocity; forward.normalize();
            float angleY = atan2(forward.x, forward.z) * 180.0 / PI;
            glRotatef(angleY, 0, 1, 0);
            drawBoidGeometry();
            glPopMatrix();
        }
    glPopMatrix();

    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);

    glutSwapBuffers();
}

void updateTargetPosition() {
    float speed = 2.0f; // Velocidade mais rápida
    
    // Movimento relativo à câmera
    if (cameraMode == 1 || cameraMode == 2 || cameraMode == 4) {
        // Câmeras 1, 2 e 4 - controles padrão
        if (keyW || keyUp) targetPos.z -= speed;
        if (keyS || keyZ || keyDown) targetPos.z += speed;
        if (keyA || keyLeft) targetPos.x -= speed;
        if (keyD || keyRight) targetPos.x += speed;
    }
    else if (cameraMode == 3) {
        // Câmera 3 (lateral) - A/D controlam Z
        if (keyW || keyUp) targetPos.z -= speed;
        if (keyS || keyZ || keyDown) targetPos.z += speed;
        if (keyA || keyLeft) targetPos.z -= speed;
        if (keyD || keyRight) targetPos.z += speed;
    }
    
    // Q/E e Page Up/Down - sempre vertical
    if (keyQ || keyPageUp) targetPos.y += speed;
    if (keyE || keyPageDown) targetPos.y -= speed;
}

void timer(int value) {
    // Atualizar posição do alvo baseado nas teclas pressionadas
    updateTargetPosition();
    
    // --- PAUSA E DEBUG (5%) ---
    if (!paused || singleStep) {
        for (auto& b : flock) {
            b.update(flock, targetPos, obstacles);
        }
        singleStep = false; // Executa um frame e para se estiver em debug
    }
    glutPostRedisplay(); // Sempre atualiza a tela
    glutTimerFunc(16, timer, 0);
}

void reshape(int w, int h) {
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)w / h, 1.0f, 5000.0f);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int x, int y) {
    float speed = 3.0f;
    switch (key) {
        case 27: exit(0); break;
        case '1': cameraMode = 1; break;
        case '2': cameraMode = 2; break;
        case '3': cameraMode = 3; break;
        case '4': cameraMode = 4; break;

        // Controle de Boids
        case '+': flock.push_back(Boid(targetPos.x, targetPos.y + 5, targetPos.z)); break;
        case '-': if (!flock.empty()) flock.pop_back(); break;

        // WASD para controlar alvo - apenas seta as flags
        case 'w': case 'W': keyW = true; break;
        case 'a': case 'A': keyA = true; break;
        case 'd': case 'D': keyD = true; break;
        case 'z': case 'Z': keyZ = true; break;
        case 'q': case 'Q': keyQ = true; break;
        case 'e': case 'E': keyE = true; break;

        // Extras
        case 'f': case 'F': fogEnabled = !fogEnabled; break;
        case 'p': case 'P': paused = !paused; break;
        case 's': case 'S': 
            if(paused) singleStep = true;
            else keyS = true;
            break;

        case 'o': case 'O': { // Add obstáculo visível
            Vec3 center = getFlockCenter();
            // Gera obstáculo na direção da câmera para o centro do bando
            Vec3 direction = center - Vec3(0, 100, 0); // Direção simplificada da torre
            direction.normalize();
            float distance = 50.0f + rand()%100; // 50 a 150 unidades da torre
            Vec3 obsPos = Vec3(0, 100, 0) + direction * distance;
            obsPos.y = 20.0f + rand()%30; // Ajustar altura
            obstacles.push_back({obsPos, 15.0f + rand()%20});
            break;
        }
    }
    glutPostRedisplay();
}

void keyboardUp(unsigned char key, int x, int y) {
    switch (key) {
        case 'w': case 'W': keyW = false; break;
        case 'a': case 'A': keyA = false; break;
        case 's': case 'S': keyS = false; break;
        case 'd': case 'D': keyD = false; break;
        case 'z': case 'Z': keyZ = false; break;
        case 'q': case 'Q': keyQ = false; break;
        case 'e': case 'E': keyE = false; break;
    }
}

void specialKeys(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_UP:    keyUp = true; break;
        case GLUT_KEY_DOWN:  keyDown = true; break;
        case GLUT_KEY_LEFT:  keyLeft = true; break;
        case GLUT_KEY_RIGHT: keyRight = true; break;
        case GLUT_KEY_PAGE_UP: keyPageUp = true; break;
        case GLUT_KEY_PAGE_DOWN: keyPageDown = true; break;
    }
}

void specialKeysUp(int key, int x, int y) {
    switch (key) {
        case GLUT_KEY_UP:    keyUp = false; break;
        case GLUT_KEY_DOWN:  keyDown = false; break;
        case GLUT_KEY_LEFT:  keyLeft = false; break;
        case GLUT_KEY_RIGHT: keyRight = false; break;
        case GLUT_KEY_PAGE_UP: keyPageUp = false; break;
        case GLUT_KEY_PAGE_DOWN: keyPageDown = false; break;
    }
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Boids");

    init();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(specialKeys);
    glutSpecialUpFunc(specialKeysUp);
    glutTimerFunc(0, timer, 0);

    std::cout << "--- CONTROLES ---\n";
    std::cout << "[Setas ou WASD] Mover Alvo (W/S: Frente/Tras, A/D: Esq/Dir)\n";
    std::cout << "[Q/E] Subir/Descer Alvo\n";
    std::cout << "[1, 2, 3, 4] Cameras (1:Aerea 2:Atras 3:Lateral 4:Torre)\n";
    std::cout << "[+/-] Adicionar/Remover Boids\n";
    std::cout << "[F] Fog On/Off\n";
    std::cout << "[P] Pausar\n";
    std::cout << "[S quando pausado] Step - Passo-a-passo\n";
    std::cout << "[O] Criar Obstaculo\n";

    glutMainLoop();
    return 0;
}
