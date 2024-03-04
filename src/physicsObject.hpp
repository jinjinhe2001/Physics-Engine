#pragma once

#include "object.hpp"
#include "mesh_helper.hpp"
#include "octree.hpp"

class RigidObject : public V1Object
{
public:
    Vec3f v; // velocity;
    Vec3f w; // angular velocity
    Vec3f dv = 0;
    Vec3f dw = 0;

    float miu_t = 0.7;
    float mass = 300;
    Mat4f I_ref; // inertia matrix
    Mat4f I_refInverse;

    float linear_decay = 0.999f;
    float angular_decay = 0.98f;
    float restitution = 0.5f; // collision
    float g = 9.8;

    Vec3f AABBmin;
    Vec3f AABBmax;
    Mesh AABB; // Actually, it's octree's mesh
    Vec3f AABBAverageLength = 0;
    ShaderProgram AABBshader;
    BufferObject AABBbuffer;
    VAO AABBVao;

    OctreeNode *root;
    int octreeDepth = 4;
    Mesh octreeMesh;

public:
    RigidObject(const std::string meshPath = "", const std::string shaderPath = "./shaders/default",
                const std::string texPath = "")
        : V1Object(meshPath, shaderPath, texPath) {}

    ~RigidObject()
    {
        if (root != nullptr)
            deleteTree(root);
    }

    void Octree2Mesh(OctreeNode *node)
    {
        if (node != nullptr)
        {
            if (node->depth == octreeDepth)
            {
                addAABB(AABB, Vec3f(node->xmin, node->ymin, node->zmin),
                        Vec3f(node->xmax, node->ymax, node->zmax));
            }

            for (int i = 0; i < 8; i++)
            {
                Octree2Mesh(node->children[i]);
            }
        }
    }

    void createAABBAndOctree()
    {
        AABBmin = Vec3f(9999, 9999, 9999);
        AABBmax = Vec3f(-9999, -9999, -9999);

        for (auto vert : mesh.vertices())
        {
            for (int i = 0; i < 3; i++)
            {
                if (vert[i] < AABBmin[i])
                    AABBmin[i] = vert[i];
                if (vert[i] > AABBmax[i])
                    AABBmax[i] = vert[i];
            }
        }
        for (int i = 0; i < 3; i++) {
            AABBAverageLength[i] = (fabs(AABBmax[i]) + fabs(AABBmin[i])) / 2.0f;
        }
        createOctree();
        // addAABB(AABB, AABBmin, AABBmax);
        Octree2Mesh(root);

        AABBVao.create();
        AABBVao.bind();
        AABBbuffer.bufferType(GL_ARRAY_BUFFER);
        AABBbuffer.usage(GL_STATIC_DRAW);
        AABBbuffer.create();
        AABBbuffer.bind();
        AABBbuffer.data(AABB.vertices().size() * sizeof(float) * 3,
                        AABB.vertices().data());

        AABBVao.enableAttrib(0);
        AABBVao.attribPointer(0, AABBbuffer, 3, GL_FLOAT, 0, 0);

        std::string shaderPath("./shaders/line");
        std::fstream vert(std::string(shaderPath + ".vert").c_str(), std::ios::in);
        std::fstream frag(std::string(shaderPath + ".frag").c_str(), std::ios::in);
        std::stringstream vertStr;
        std::stringstream fragStr;
        if (!vert.good() || !frag.good())
        {
            std::cout << "ERROR: loading obj:(" << shaderPath << ") file is not good.\n";
        }
        vertStr << vert.rdbuf();
        fragStr << frag.rdbuf();
        if (!AABBshader.compile(vertStr.str().c_str(), fragStr.str().c_str()))
        {
            std::cout << "ERROR: loading obj:(" << shaderPath << ") file is not good.\n";
        }
        vert.close();
        frag.close();
    }

    void createOctree()
    {
        // create AABB first
        root = new OctreeNode();
        root->depth = 1;
        createMeshOctree(root, mesh, AABBmin.x, AABBmax.x,
                         AABBmin.y, AABBmax.y,
                         AABBmin.z, AABBmax.z, octreeDepth);

        // preOrder(root);
        // std::cout << "octree node num:" << nodeNum(root) << std::endl;
        octreeToMesh(root, octreeMesh, octreeDepth);
        // std::cout << "octree mesh num:" << octreeMesh.vertices().size() << std::endl;
    }

    void drawAABB(Graphics &g, Nav &camera)
    {
        AABBshader.use();

        g.translate(nav.pos());
        g.rotate(nav.quat());
        g.scale(scale);

        AABBshader.uniform("model", g.modelMatrix());
        AABBshader.uniform("view", g.viewMatrix());
        AABBshader.uniform("projection", g.projMatrix());
        AABBshader.uniform("color", Vec3f(1.0f, 0.5f, 0.7f));

        AABBVao.bind();
        glDrawArrays(GL_LINES, 0, AABB.vertices().size());
    }

    void initIRef()
    {
        auto vertices = octreeMesh.vertices();
        Mat4f S = ScaleMatrix(scale);
        float m = mass / vertices.size();
        for (int i = 0; i < vertices.size(); i++)
        {
            vertices[i] = S * Vec4f(vertices[i], 1.0f);
            float diag = m * vertices[i].magSqr();
            I_ref += Mat4f(diag - m * vertices[i][0] * vertices[i][0], -m * vertices[i][0] * vertices[i][1], -m * vertices[i][0] * vertices[i][2], 0,
                           -m * vertices[i][1] * vertices[i][0], diag - m * vertices[i][1] * vertices[i][1], -m * vertices[i][1] * vertices[i][2], 0,
                           -m * vertices[i][2] * vertices[i][0], -m * vertices[i][2] * vertices[i][1], diag - m * vertices[i][2] * vertices[i][2], 0,
                           0, 0, 0, 1);
        }
        I_refInverse = I_ref.inversed();
    }

    void addVelocity(Vec3f _v = Vec3f(0, 7.0f, 0))
    {
        restitution = 0.5;
        v += _v;
    }

    void collisonImpulse_plane(Vec3f P, Vec3f N)
    {
        auto &vertices = octreeMesh.vertices();
        Mat4f R;
        Mat4f S = ScaleMatrix(scale);
        nav.quat().toMatrix(R.elems());
        R = S * R;
        Vec3f x = nav.pos();
        Vec3f collideL(0);
        Vec3f collideV(0);
        float count = 0;

        for (int i = 0; i < vertices.size(); i++)
        {
            Vec3f Rri = R * Vec4f(vertices[i], 1.0f);
            if ((x + Rri - P).dot(N) < 0)
            {
                Vec3f vi = v + w.cross(Rri);
                if (dot(vi, N) < 0)
                {
                    collideL += Rri;
                    collideV += vi;
                    count += 1;
                }
            }
        }
        if (count > 0)
        {
            collideL *= (1 / count);
            collideV *= (1 / count);
            Mat4f Rri_cross = CrossMatrix(collideL);
            Vec3f j(0.0f); // Impulse

            Vec3f v_ni = dot(collideV, N) * N;
            Vec3f v_ti = collideV - v_ni;

            float friction;
            if (v_ti.mag() < 1e-5)
                friction = 0;
            else
                friction = max(1 - miu_t * (1 + restitution) * v_ni.mag() / v_ti.mag(), 0.0f);
            // 1 - μt(1 + μn)||Vni||/||Vti||

            v_ni *= -restitution;
            v_ti *= friction;
            //std::cout<< restitution<<","<<friction<<std::endl;

            Mat4f K = Mat4f::identity() * (1 / mass) - Rri_cross * I_refInverse * Rri_cross;
            j = K.inversed() * Vec4f(v_ni + v_ti - collideV, 1.0f);

            /*std::cout<<"I_ref"<<std::endl;
            for(int i = 0; i < 16; i++){
                std::cout<<I_ref[i]<<std::endl;
            }
            std::cout<<"I_refInverse"<<std::endl;
            for(int i = 0; i < 16; i++){
                std::cout<<I_refInverse[i]<<std::endl;
            }
            std::cout<<"K"<<std::endl;
            for(int i = 0; i < 16; i++){
                std::cout<<K[i]<<std::endl;
            }
            std::cout<<"J"<<std::endl;
            std::cout<<j<<"\n";*/

            dv += (1 / mass) * j;

            dw += I_refInverse * (Rri_cross * Vec4f(j, 1.0f));
        }
    }

    void rigidBodyCollision(RigidObject &object)
    {
        auto &vertices = octreeMesh.vertices();
        Mat4f R;
        Mat4f S = ScaleMatrix(scale);
        nav.quat().toMatrix(R.elems());
        R = S * R;
        Vec3f x = nav.pos();
        Vec3f collideL(0);
        Vec3f collideV(0);
        Vec3f objectCollideL(0);
        Vec3f collideSurface;
        float count = 0;

        Vec3f objectX = object.nav.pos();
        Mat4f objectR;
        Mat4f objectS = ScaleMatrix(object.scale);
        object.nav.quat().toMatrix(objectR.elems());
        objectR = objectS * objectR;
        auto inverseObjectR = objectR.inversed();

        for (int i = 0; i < vertices.size(); i++)
        {
            Vec3f Rri = R * Vec4f(vertices[i], 1.0f);
            Vec3f transformedX = inverseObjectR * Vec4f(x + Rri - objectX, 1.0f);
            if (inBox(transformedX, object.AABBmin, object.AABBmax))
            {
                Vec3f vi = v + w.cross(Rri);
                if (dot(vi, x + Rri - objectX) < 0)
                {
                    collideSurface += (x + Rri - objectX).normalize();
                    objectCollideL += x + Rri - objectX;
                    collideL += Rri;
                    collideV += vi;
                    count += 1;
                }
            }
        }
        // std::cout << count << std::endl;
        if (count > 0)
        {
            collideSurface *= (1 / count);
            Vec3f N = collideSurface.normalize();
            objectCollideL *= (1 / count);
            collideL *= (1 / count);
            collideV *= (1 / count);
            Mat4f Rri_cross = CrossMatrix(collideL);
            Mat4f objectRri_cross = CrossMatrix(objectCollideL);
            Vec3f j(0.0f); // Impulse

            Vec3f v_ni = dot(collideV, N) * N;
            Vec3f v_ti = collideV - v_ni;

            float friction;
            if (v_ti.mag() < 1e-5)
                friction = 0;
            else
                friction = max(1 - miu_t * (1 + restitution) * v_ni.mag() / v_ti.mag(), 0.0f);
            // 1 - μt(1 + μn)||Vni||/||Vti||

            v_ni *= -restitution;
            v_ti *= friction;
            // std::cout<<friction<<std::endl;

            Mat4f K = Mat4f::identity() * (1 / mass) - Rri_cross * I_refInverse * Rri_cross;
            j = K.inversed() * Vec4f(v_ni + v_ti - collideV, 1.0f);
            dv += (1 / mass) * j / 2;

            dw += I_refInverse * (Rri_cross * Vec4f(j, 1.0f)) / 2;

            object.dv += (1 / object.mass) * (-j) / 2;
            object.dw += object.I_refInverse * (objectRri_cross * Vec4f(-j, 1.0f)) / 2;
        }
    }

    void onAnimate(double dt) override
    {
        Vec3f Fg(0, -mass * g, 0);
        v *= linear_decay;
        v += (float)dt * Fg * (1 / mass);
        w *= angular_decay;
        if (v.mag() < 0.5f)
        {
            if (restitution < 1e-6) {
                restitution = 0;
            } else {
                restitution *= 0.9;
            }
        }
        else
        {
            restitution = 0.5;
        }

        collisonImpulse_plane(Vec3f(0, -1.5f, 0), Vec3f(0, 1, 0));
        collisonImpulse_plane(Vec3f(15.0f, 0, 0), Vec3f(-1, 0, 0));
        collisonImpulse_plane(Vec3f(-15.0f, 0, 0), Vec3f(1, 0, 0));
        collisonImpulse_plane(Vec3f(0, 0, 15.0f), Vec3f(0, 0, -1));
        collisonImpulse_plane(Vec3f(0, 0, -15.0f), Vec3f(0, 0, 1));

        v += dv;
        w += dw;
        dv = 0;
        dw = 0;

        auto &x = nav.pos();
        x += (float)dt * v;
        Quatf dq(0, w.x * dt * 0.5, w.y * dt * 0.5, w.z * dt * 0.5);

        auto &q = nav.quat();
        q += dq * q;
        q.normalize();
    }
};

class MassSpring : public V1Object
{
public:
    float mass = 1.0f;
    float damping = 0.99f;
    float rho = 0.995f;
    float spring_k = 80000;
    std::vector<int> E;
    std::vector<float> L;
    std::vector<Vec3f> V;
    Vec3f g = Vec3f(0, -9.8f, 0);
    int n = 81;

    MassSpring(const std::string meshPath = "", const std::string shaderPath = "./shaders/default",
               const std::string texPath = "")
        : V1Object(meshPath, shaderPath, texPath) {}

    void onCreate() override
    {
        V1Object::onCreate();
        mesh.reset();
        std::vector<Vec3f> X(n * n);
        std::vector<Vec2f> UV(n * n);
        std::vector<Mesh::Index> triangles((n - 1) * (n - 1) * 6);
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < n; i++)
            {
                X[j * n + i] = Vec3f(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
                UV[j * n + i] = Vec2f(i / (n - 1.0f), j / (n - 1.0f));
            }
        }
        int t = 0;
        for (int j = 0; j < n - 1; j++)
        {
            for (int i = 0; i < n - 1; i++)
            {
                triangles[t * 6 + 0] = j * n + i;
                triangles[t * 6 + 1] = j * n + i + 1;
                triangles[t * 6 + 2] = (j + 1) * n + i + 1;
                triangles[t * 6 + 3] = j * n + i;
                triangles[t * 6 + 4] = (j + 1) * n + i + 1;
                triangles[t * 6 + 5] = (j + 1) * n + i;
                t++;
            }
        }
        std::vector<int> _E(triangles.size() * 2);
        for (int i = 0; i < triangles.size(); i += 3)
        {
            _E[i * 2 + 0] = triangles[i + 0];
            _E[i * 2 + 1] = triangles[i + 1];
            _E[i * 2 + 2] = triangles[i + 1];
            _E[i * 2 + 3] = triangles[i + 2];
            _E[i * 2 + 4] = triangles[i + 2];
            _E[i * 2 + 5] = triangles[i + 0];
        }
        for (int i = 0; i < _E.size(); i += 2)
        {
            if (_E[i] > _E[i + 1])
                std::swap(_E[i], _E[i + 1]);
        }
        QuickSort(_E, 0, _E.size() / 2 - 1);

        int eNumber = 0;
        for (int i = 0; i < _E.size(); i += 2)
        {
            if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
                eNumber++;
        }
        E.resize(eNumber * 2);
        for (int i = 0, e = 0; i < _E.size(); i += 2)
        {
            if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
            {
                E[e * 2 + 0] = _E[i + 0];
                E[e * 2 + 1] = _E[i + 1];
                e++;
            }
        }
        L.resize(E.size() / 2);
        for (int e = 0; e < E.size() / 2; e++)
        {
            int v0 = E[e * 2 + 0];
            int v1 = E[e * 2 + 1];
            L[e] = (X[v0] - X[v1]).mag();
        }
        V.resize(X.size());
        for (int i = 0; i < V.size(); i++)
        {
            V[i] = Vec3f(0, 0, 0);
        }

        mesh.vertices() = X;
        mesh.indices() = triangles;
        mesh.texCoord2s() = UV;
        generateNormals();
        reBindAll();
    }
    void computeGradient(std::vector<Vec3f> &X, std::vector<Vec3f> &XHat, float t, std::vector<Vec3f>& G)
    {
        for (int i = 0; i < G.size(); i++)
        {
            G[i] = (1 / t) * mass * (X[i] - XHat[i]) * (1 / t);
            G[i] -= mass * g;
        }
        for (int e = 0; e < E.size() / 2; e++)
        {
            int i = E[e * 2 + 0];
            int j = E[e * 2 + 1];
            Vec3f dir = X[i] - X[j];
            
            G[i] += spring_k * (1 - L[e] / (dir).mag()) * (dir);
            G[j] -= spring_k * (1 - L[e] / (dir).mag()) * (dir);
        }
    }

    void onAnimate(double dt) override {
        Mat4f R;
        Mat4f S = ScaleMatrix(scale);
        nav.quat().toMatrix(R.elems());
        R = S * R;
        Mat4f InversedR = R.inversed();
        Vec3f x = nav.pos();

        auto X = mesh.vertices();
        for (int i = 0; i < X.size(); i++) {
            X[i] = Vec3f(R * Vec4f(X[i], 1.0f)) + x;
        }
        std::vector<Vec3f> last_X(X.size());
        std::vector<Vec3f> XHat(X.size());
        std::vector<Vec3f> G(X.size());

        for (int i = 0; i < X.size(); i++) {
            V[i] *= damping;
            XHat[i] = X[i] + V[i] * dt;
            X[i] = XHat[i];
            last_X[i] = Vec3f(0);
            G[i] = Vec3f(0);
        }

        for (int k = 0; k < 32; k++) {
            float w = 0;
            computeGradient(X, XHat, dt, G);
            if (k == 0) w = 1;
            else if (k == 1) w = 2 / (2 - rho * rho);
            else w = 4 / (4 - rho * rho * w);
            auto oldX = X;
            for (int i = 0; i < X.size(); i++) {
                if (i == 0 || i == n - 1) continue;
                X[i] -= G[i] / (float)((1 / dt) * mass * (1 / dt) + 4 * spring_k);
                X[i] = w * X[i] + (1 - w) * last_X[i];
            }
            last_X = oldX;
        }
        for (int i = 0; i < X.size(); i++) {
            if (i == 0 || i == n - 1) continue;
            V[i] += (X[i] - XHat[i]) * (1 / dt);
        }

        for (int i = 0; i < X.size(); i++) {
            X[i] = Vec3f(InversedR * Vec4f(X[i] - x, 1.0f));
        }

        mesh.vertices() = X;

        collisonImpulse_plane(Vec3f(0, -1.5f, 0), Vec3f(0, 1, 0), dt);
        collisonImpulse_plane(Vec3f(15.0f, 0, 0), Vec3f(-1, 0, 0), dt);
        collisonImpulse_plane(Vec3f(-15.0f, 0, 0), Vec3f(1, 0, 0), dt);
        collisonImpulse_plane(Vec3f(0, 0, 15.0f), Vec3f(0, 0, -1), dt);
        collisonImpulse_plane(Vec3f(0, 0, -15.0f), Vec3f(0, 0, 1), dt);

        reBindVertices();
    }
    // after scale
    void reCalculateL() {
        auto& X = mesh.vertices();
        for (int e = 0; e < E.size() / 2; e++)
        {
            int v0 = E[e * 2 + 0];
            int v1 = E[e * 2 + 1];
            L[e] = (X[v0] * scale - X[v1] * scale).mag();
        }
    }

    void collisonImpulse_plane(Vec3f P, Vec3f N, float dt) {
        auto &vertices = mesh.vertices();
        N = N.normalize();
        Mat4f R;
        Mat4f S = ScaleMatrix(scale);
        nav.quat().toMatrix(R.elems());
        R = S * R;
        Mat4f InversedR = R.inversed();
        Vec3f x = nav.pos();


        for (int i = 0; i < vertices.size(); i++)
        {
            Vec3f Rri = R * Vec4f(vertices[i], 1.0f);
            if ((x + Rri - P).dot(N) < 0)
            {
                if (dot(V[i], N) < 0)
                {
                    float dis = (x + Rri - P).dot(N);
                    Vec3f newX = x + Rri + (-dis + Vec3f(0.01f)) * N;
                    V[i] += -dis * (1 / dt) * N;
                    vertices[i] = Vec3f(InversedR * Vec4f(newX - x, 1.0f));
                }
            }
        }
    }

    void rigidBodyCollision(RigidObject &object, float dt) {
        auto &vertices = mesh.vertices();
        Mat4f R;
        Mat4f S = ScaleMatrix(scale);
        nav.quat().toMatrix(R.elems());
        R = S * R;
        Mat4f InversedR = R.inversed();
        Vec3f x = nav.pos();
        Vec3f collideL(0);
        Vec3f collideV(0);
        Vec3f objectCollideL(0);
        Vec3f collideSurface;
        float count = 0;

        Vec3f objectX = object.nav.pos();
        Mat4f objectR;
        Mat4f objectS = ScaleMatrix(object.scale);
        object.nav.quat().toMatrix(objectR.elems());
        objectR = objectS * objectR;
        auto inverseObjectR = objectR.inversed();

        for (int i = 0; i < vertices.size(); i++)
        {
            if (i == 0 || i == n - 1) continue;
            Vec3f Rri = R * Vec4f(vertices[i], 1.0f);
            Vec3f transformedX = inverseObjectR * Vec4f(x + Rri - objectX, 1.0f);
            if (inBox(transformedX, object.AABBmin, object.AABBmax))
            {
                {//if ((- x - Rri + objectX).dot(V[i]) > 0) {
                    if (transformedX.mag() < object.AABBAverageLength.mag() * 1.0f) {
                        transformedX = transformedX.normalize() * object.AABBAverageLength.mag() * 1.0f;
                        Vec3f newX = objectR * Vec4f(transformedX, 1.0f) + objectX;
                        V[i] += (newX - x - Rri) / dt;
                        vertices[i] = Vec3f(InversedR * Vec4f(newX - x, 1.0f));
                    }
                }
            }
        }
    }
};