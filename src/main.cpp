#include <iostream>
#include <memory>

// for master branch
// #include "al/core.hpp"

// for devel branch
#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/graphics/al_Image.hpp"
#include "object.hpp"
#include "physicsObject.hpp"
#include "skybox.hpp"
#include "al/app/al_DistributedApp.hpp"
#include "al/io/al_Imgui.hpp"
#include "al/math/al_Ray.hpp"

using namespace al;

struct MyApp : DistributedApp
{
  std::vector<std::shared_ptr<RigidObject>> bunnys;
  std::unique_ptr<V1Object> plane;
  std::unique_ptr<Skybox> skybox;
  std::shared_ptr<MassSpring> cloth1;
  std::shared_ptr<MassSpring> cloth2;
  int nearOne = -1;
  float nearT = 9999;
  int axis = -1;

  // bool showOctree = true;
  float viewDistance = 30.0f;
  float theta1 = 0.75 * M_2PI;
  float theta2 = 0.125 * M_2PI;
  Vec3f dragDir;
  float dragFactor = 0.05f;

  // for networking
  std::vector<ParameterPose> poses;
  std::vector<ParameterVec3> cloth1Pos;
  std::vector<ParameterVec3> cloth2Pos;
  ParameterBool showOctree{"showOctree", "", true};
  /*ParameterDouble viewDistance{"viewDistance", "", 30.0};
  ParameterDouble theta1{"theta1", "", 0.75 * M_2PI};
  ParameterDouble theta2{"theta2", "", 0.125 * M_2PI};
  ParameterDouble dragFactor{"dragFactor", "", 0.05f};*/
  ParameterVec4 para4{"para4", "", Vec4f(30.0, 0.75 * M_2PI, 0.125 * M_2PI, 0.05f)};
  ParameterInt bunnyNum{"bunnyNum", "", 0};

  void createCloth()
  {
    cloth1 = std::make_shared<MassSpring>("",
                                          "./shaders/cloth",
                                          "./assets/cloth/cloth.jpeg");
    cloth1->onCreate();
    cloth1->scale = Vec3f(0.9f);
    cloth1->reCalculateL();
    cloth1->nav.pos(0, 8, 0);
    cloth1->material.shininess(128);
    cloth1->singleLight.pos(5, 10, -5);
    /*for (int i = 0; i < cloth1->mesh.vertices().size(); i++)
    {
      cloth1Pos.push_back(ParameterVec3("cloth1_" + std::to_string(i)));
    }
    for (int i = 0; i < cloth1->mesh.vertices().size(); i++)
    {
      parameterServer() << cloth1Pos[i];
    }*/
    cloth2 = std::make_shared<MassSpring>("",
                                          "./shaders/cloth",
                                          "./assets/cloth/cloth.jpeg");
    cloth2->onCreate();
    cloth2->scale = Vec3f(0.9f);
    cloth2->reCalculateL();
    cloth2->nav.pos(0, 8, -8);
    cloth2->material.shininess(128);
    cloth2->singleLight.pos(5, 10, -5);

    /*for (int i = 0; i < cloth2->mesh.vertices().size(); i++)
    {
      cloth2Pos.push_back(ParameterVec3("cloth2_" + std::to_string(i)));
    }
    for (int i = 0; i < cloth2->mesh.vertices().size(); i++)
    {
      parameterServer() << cloth2Pos[i];
    }*/
  }

  void createBunny()
  {
  if (bunnys.size() >= 20) return;
    std::shared_ptr<RigidObject> bunny = std::make_shared<RigidObject>(
        "./assets/bunny/bunny.obj",
        "./shaders/default",
        "./assets/bunny/bunny-atlas.jpg");

    bunny->onCreate();
    bunny->generateNormals();
    bunny->scale = Vec3f(0.005);
    bunny->nav.pos(1, 4, 1);
    bunny->nav.quat().fromAxisAngle(-0.5 * M_2PI, 1, 0, 0);
    bunny->material.shininess(8.0f);
    bunny->singleLight.pos(5, 10, -5);
    bunny->createAABBAndOctree();
    bunny->initIRef();
    bunnys.push_back(bunny);
    if (isPrimary())
    {
      bunnyNum = bunnys.size();
    }
    //poses.push_back(ParameterPose("bunnys_" + std::to_string(bunnys.size() - 1)));
    //parameterServer() << poses[bunnys.size() - 1];
  }

  void createPlane()
  {
    plane = std::make_unique<V1Object>("./assets/plane/plane.obj",
                                       "./shaders/default",
                                       "./assets/plane/uvmap.jpeg");
    plane->onCreate();
    plane->scale = Vec3f(1);
    plane->nav.pos(0, -1.5, 0);
    plane->nav.quat().fromAxisAngle(-0.25 * M_2PI, 1, 0, 0);
    plane->material.shininess(32);
    plane->singleLight.pos(5, 10, -5);
  }

  void createSkybox()
  {
    std::vector<std::string> faces{
        "./assets/skybox/right.jpg",
        "./assets/skybox/left.jpg",
        "./assets/skybox/top.jpg",
        "./assets/skybox/bottom.jpg",
        "./assets/skybox/front.jpg",
        "./assets/skybox/back.jpg",
    };
    skybox = std::make_unique<Skybox>(faces);
  }

  void onCreate() override
  {
    createBunny();
    createCloth();
    createPlane();
    createSkybox();
    nav().pos(viewDistance * sinf(theta1),
              viewDistance * sinf(theta2),
              viewDistance * cosf(theta1));
    nav().faceToward(Vec3f(0));
    Vec3d euler;
    nav().quat().toEuler(euler);
    euler.z = 0;
    nav().quat().fromEuler(euler);
    navControl().disable();
    parameterServer() << showOctree << para4 << bunnyNum;
    for (int i = 0; i < 20; i++) {
      poses.push_back(ParameterPose("bunnys_" + std::to_string(i)));
    }
    for (int i = 0; i < 20; i++) {
      parameterServer() << poses[i];
    }
  }

  bool onKeyDown(Keyboard const &k) override
  {
    if (!isPrimary()) return true;
    if (nearOne >= 0)
    {
      switch (k.key())
      {
      case ' ':
      {
        bunnys[nearOne]->addVelocity(Vec3f(0, 100.0f * dragFactor, 0));
      }
      break;
      }
    }
    return true;
  }
  void onAnimate(double dt) override
  {
    dt = 0.016f;
    if (dt < 1e-6)
      return;
    if (!isPrimary())
    {
      auto _para4 = para4.get();
      viewDistance = _para4.x;
      theta1 = _para4.y;
      theta2 = _para4.z;
      dragFactor = _para4.w;
      for (int i = 0; i < bunnys.size(); i++)
      {
        bunnys[i]->nav.set(poses[i].get());
      }
      cloth1->onAnimate(dt);
      for (int i = 0; i < bunnys.size(); i++)
      {
        cloth1->rigidBodyCollision(*bunnys[i], dt);
      }
      cloth2->onAnimate(dt);
      for (int i = 0; i < bunnys.size(); i++)
      {
        cloth2->rigidBodyCollision(*bunnys[i], dt);
      }

      while (bunnyNum > bunnys.size())
      {
        createBunny();
      }
      nav().pos(viewDistance * sinf(theta1),
                viewDistance * sinf(theta2),
                viewDistance * cosf(theta1));
      nav().faceToward(Vec3f(0));

      Vec3d euler;
      nav().quat().toEuler(euler);
      euler.z = 0;
      nav().quat().fromEuler(euler);
      /*auto X = cloth1->mesh.vertices();
      for (int i = 0; i < cloth1->mesh.vertices().size(); i++)
      {
        X[i] = Vec3f(0);
      }
      cloth1->mesh.vertices() = X;
      cloth1->reBindVertices();
      auto Y = cloth2->mesh.vertices();
      for (int i = 0; i < cloth2->mesh.vertices().size(); i++)
      {
        Y[i] = cloth2Pos[i].get();
      }
      cloth2->mesh.vertices() = Y;
      cloth2->reBindVertices();*/
      return;
    }
    for (int i = 0; i < bunnys.size(); i++)
    {
      for (int j = 0; j < bunnys.size(); j++)
      {
        if (i == j)
          continue;
        bunnys[i]->rigidBodyCollision(*bunnys[j]);
      }
    }

    for (int i = 0; i < bunnys.size(); i++)
    {
      bunnys[i]->onAnimate(dt);
    }
    cloth1->onAnimate(dt);
    for (int i = 0; i < bunnys.size(); i++)
    {
      cloth1->rigidBodyCollision(*bunnys[i], dt);
    }
    cloth2->onAnimate(dt);
    for (int i = 0; i < bunnys.size(); i++)
    {
      cloth2->rigidBodyCollision(*bunnys[i], dt);
    }

    for (int i = 0; i < bunnys.size(); i++)
    {
      poses[i] = bunnys[i]->nav.pos();
    }
    
    para4 = Vec4f(viewDistance, theta1, theta2, dragFactor);
    /*for (int i = 0; i < cloth1->mesh.vertices().size(); i++)
    {
      cloth1Pos[i] = cloth1->mesh.vertices()[i];
    }
    for (int i = 0; i < cloth2->mesh.vertices().size(); i++)
    {
      cloth2Pos[i] = cloth2->mesh.vertices()[i];
    }*/
  }

  void onDraw(Graphics &g) override
  {
    g.depthTesting(true);
    g.clear(0.2);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    g.pushMatrix();
    skybox->onDraw(g, nav());
    g.popMatrix();

    for (int i = 0; i < bunnys.size(); i++)
    {
      g.pushMatrix();
      bunnys[i]->onDraw(g, nav());
      g.popMatrix();
      if (showOctree.get() || i == nearOne)
      {
        g.pushMatrix();
        bunnys[i]->drawAABB(g, nav());
        g.popMatrix();
      }
    }

    g.pushMatrix();
    cloth1->onDraw(g, nav());
    g.popMatrix();

    g.pushMatrix();
    cloth2->onDraw(g, nav());
    g.popMatrix();

    g.pushMatrix();
    plane->onDraw(g, nav());
    g.popMatrix();

    if (isPrimary()) {
      drawImGUI(g);
    }
  }

  void drawImGUI(Graphics &g)
  {
    if (!isPrimary()) return;
    imguiBeginFrame();

    ImGui::Begin("GUI");
    static bool _showOctree = true;
    ImGui::Checkbox("Show Octree", &_showOctree);
    showOctree = _showOctree;

    static float _dis = 30.0f;
    ImGui::SliderFloat("View Distance", &_dis, 1.0f, 60.0f, "ratio = %.3f");
    {
      nav().pos(viewDistance * sinf(theta1),
                viewDistance * sinf(theta2),
                viewDistance * cosf(theta1));
      nav().faceToward(Vec3f(0));

      Vec3d euler;
      nav().quat().toEuler(euler);
      euler.z = 0;
      nav().quat().fromEuler(euler);
    }
    viewDistance = _dis;

    static float _drag = 0.05f;
    ImGui::SliderFloat("Drag Factor", &_drag, 0.01f, 0.2f, "ratio = %.3f");
    dragFactor = _drag;

    if (ImGui::Button("Add Bunny"))
    {
      createBunny();
      if (isPrimary())
      {
      }
    }
    ImGui::End();
    imguiEndFrame();
    imguiDraw();
  }

  void onInit() override
  {
    if (isPrimary()) {
      imguiInit();
    }
  }
  void onExit() override { 
    if (isPrimary())
      imguiShutdown(); 
  }

  Vec3d unproject(Vec3d screenPos)
  {
    auto &g = graphics();
    auto mvp = g.projMatrix() * g.viewMatrix() * g.modelMatrix();
    Matrix4d invprojview = Matrix4d::inverse(mvp);
    Vec4d worldPos4 = invprojview.transform(screenPos);
    return worldPos4.sub<3>(0) / worldPos4.w;
  }

  Rayd getPickRay(int screenX, int screenY)
  {
    Rayd r;
    Vec3d screenPos;
    screenPos.x = (screenX * 1. / width()) * 2. - 1.;
    screenPos.y = ((height() - screenY) * 1. / height()) * 2. - 1.;
    screenPos.z = -1.;
    Vec3d worldPos = unproject(screenPos);
    r.origin().set(worldPos);

    screenPos.z = 1.;
    worldPos = unproject(screenPos);
    r.direction().set(worldPos);
    r.direction() -= r.origin();
    r.direction().normalize();
    return r;
  }

  bool onMouseDown(const Mouse &m) override
  {
    if (!isPrimary()) return true;
    Rayd r = getPickRay(m.x(), m.y());
    nearOne = -1;
    nearT = 9999;
    for (int i = 0; i < bunnys.size(); i++)
    {
      Mat4f R;
      Mat4f S = ScaleMatrix(bunnys[i]->scale);
      bunnys[i]->nav.quat().toMatrix(R.elems());
      R = S * R;
      auto inversedR = R.inversed();
      Vec3f x = bunnys[i]->nav.pos();

      float radius = Vec3f(R * Vec4f(bunnys[i]->AABBAverageLength, 1.0f)).mag();
      float t = r.intersectSphere(x, radius * 0.8f);
      // std::cout << "center" << x << "scl" << radius * 0.8f << "t" << t << std::endl;
      if (t > 0 && t < nearT)
      {
        nearT = t;
        nearOne = i;
        dragDir = r.direction().cross(Vec3f(0, 1, 0)).normalize();
      }
    }
    return true;
  }

  bool onMouseUp(const Mouse &m) override
  {
    if (!isPrimary()) return true;
    nearOne = -1;
    nearT = 9999;
    return true;
  }

  bool onMouseDrag(const Mouse &m) override
  {
    if (!isPrimary()) return true;
    if (nearOne == -1)
    {
      theta1 = theta1 - m.dx() * 0.001f;
      theta2 = theta2 + m.dy() * 0.001f;
      nav().pos(viewDistance * sinf(theta1),
                viewDistance * sinf(theta2),
                viewDistance * cosf(theta1));
      nav().faceToward(Vec3f(0));

      Vec3d euler;
      nav().quat().toEuler(euler);
      euler.z = 0;
      nav().quat().fromEuler(euler);
    }
    else
    {
      float dx = m.dx() * dragFactor;
      float dy = -m.dy() * dragFactor;
      Vec3f axisDir = dragDir.cross(Vec3f(0, 1, 0)).normalize();
      auto result = dragDir * dx - axisDir * dy;
      bunnys[nearOne]->addVelocity(result);
    }

    return true;
  }
};

int main()
{
  MyApp app;
  app.dimensions(1080, 720);
  app.start();
}
