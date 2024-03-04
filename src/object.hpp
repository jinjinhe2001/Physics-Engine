#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include "al/io/al_ControlNav.hpp"
#include "al/graphics/al_Mesh.hpp"
#include "al/graphics/al_Light.hpp"
#include "al/graphics/al_Shader.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/graphics/al_DefaultShaders.hpp"
#include "al/graphics/al_DefaultShaderString.hpp"
#include "al/graphics/al_BufferObject.hpp"
#include "al/graphics/al_Graphics.hpp"
#include "loader.hpp"
#include "math_helper.hpp"

using namespace al;
class Object
{
public:
    Nav nav;
    Vec3f scale;
    Mesh mesh;
    Texture texture;
    Material material;
    ShaderProgram shader;

    // multiObject may use the same source, reduce memory costing
    
public:
    Object(const std::string meshPath = "", const std::string shaderPath = "", 
        const std::string texPath = "") {
        if (!strcmp(meshPath.c_str(), "")) {
            addSphere(mesh);
        } else {
            std::vector<Vec3f> vertices;
            std::vector<Vec2f> uvs;
            std::vector<Vec3f> normals;
            loadOBJ(meshPath.c_str(), vertices, uvs, normals);
            indexVBO(vertices, uvs, normals, mesh.indices(), mesh.vertices(), mesh.texCoord2s(), mesh.normals());
        }

        if (!strcmp(shaderPath.c_str(), "")) {
            al::ShaderSources un = al::defaultShaderUniformColor(false, false, false);
            shader.compile(un.vert.c_str(), un.frag.c_str());
        } else {
            std::fstream vert(std::string(shaderPath + ".vert").c_str(), std::ios::in);
            std::fstream frag(std::string(shaderPath + ".frag").c_str(), std::ios::in);
            std::stringstream vertStr;
            std::stringstream fragStr;
            if (!vert.good() || !frag.good()) {
                std::cout<<"ERROR: loading obj:(" << shaderPath << ") file is not good.\n";
            }
            vertStr << vert.rdbuf();
            fragStr << frag.rdbuf();
            if (!shader.compile(vertStr.str().c_str(), fragStr.str().c_str())) {
                std::cout<<"ERROR: loading obj:(" << shaderPath << ") file is not good.\n";
            }
            vert.close();
            frag.close();
        }
        if (!strcmp(texPath.c_str(), "")) {
            // pass
        } else {
            loadTexture(texture, texPath);
        }
        // default material
        material.ambient(Color(1.0f, 1.0f, 1.0f, 1.0f));
        material.diffuse(Color(1.0f, 1.0f, 1.0f, 1.0f));
        material.specular(Color(1.0f, 1.0f, 1.0f, 1.0f));
        material.shininess(32.0f);
    }

    ~Object() {
        //shader.destroy();
    }
    virtual void onCreate() = 0;
    virtual void onAnimate(double dt) = 0;
    virtual void onDraw(Graphics& g, Nav& camera) = 0;
};

class V1Object : public Object 
{
public:
    Light singleLight;

    BufferObject bufferArray[3];
    BufferObject elementBuffer;
    VAO vao;
    V1Object(const std::string meshPath = "", const std::string shaderPath = "./shaders/default", 
        const std::string texPath = "") 
        : Object(meshPath, shaderPath, texPath) {}

    void onCreate() override {
        Color lightColor(1.0f, 1.0f, 1.0f, 1.0f);
        singleLight.ambient(lightColor * 0.5f);
        singleLight.diffuse(lightColor * 0.7f);
        singleLight.specular(Color(1.0f, 1.0f, 1.0f, 1.0f));

        
        std::vector<int>bufferSize({3, 2, 3});
        vao.create();
        for (int i = 0; i < bufferSize.size(); i++) {
            bufferArray[i].bufferType(GL_ARRAY_BUFFER);
            bufferArray[i].usage(GL_STATIC_DRAW);
            bufferArray[i].create();
            bufferArray[i].bind();
            if (i == 0)
                bufferArray[i].data(mesh.vertices().size() * sizeof(float) * bufferSize[i], mesh.vertices().data());
            else if (i == 1) {
                bufferArray[i].data(mesh.texCoord2s().size() * sizeof(float) * bufferSize[i], mesh.texCoord2s().data());
            } else {
                bufferArray[i].data(mesh.normals().size() * sizeof(float) * bufferSize[i], mesh.normals().data());
            }
            vao.bind();
            vao.enableAttrib(i);
            vao.attribPointer(i, bufferArray[i], bufferSize[i], GL_FLOAT, 0, 0);
        }
        elementBuffer.bufferType(GL_ELEMENT_ARRAY_BUFFER);
        elementBuffer.usage(GL_STATIC_DRAW);
        elementBuffer.create();
        elementBuffer.bind();
        elementBuffer.data(mesh.indices().size() * sizeof(unsigned int), mesh.indices().data());
    }
    
    void onAnimate(double dt) override {
        // pass
    }

    void onDraw(Graphics& g, Nav& camera) override {
        shader.use();

        g.translate(nav.pos());
        g.rotate(nav.quat());
        g.scale(scale);
        

        shader.uniform("model", g.modelMatrix());
        shader.uniform("view", g.viewMatrix());
        shader.uniform("projection", g.projMatrix());

        shader.uniform("viewPos", camera.pos());
        shader.uniform("material.ambient", Vec3f(material.ambient().r, 
        material.ambient().g, material.ambient().b));
        shader.uniform("material.diffuse", Vec3f(material.diffuse().r, 
        material.diffuse().g, material.diffuse().b));
        shader.uniform("material.specular", Vec3f(material.specular().r, 
        material.specular().g, material.specular().b));
        shader.uniform("material.shininess", material.shininess());

        shader.uniform("light.position", Vec3f(singleLight.pos()));
        shader.uniform("light.ambient", Vec3f(singleLight.ambient().r, 
        singleLight.ambient().g, singleLight.ambient().b));
        shader.uniform("light.diffuse", Vec3f(singleLight.diffuse().r, 
        singleLight.diffuse().g, singleLight.diffuse().b));
        shader.uniform("light.specular", Vec3f(singleLight.specular().r, 
        singleLight.specular().g, singleLight.specular().b));

        glUniform1i(glGetUniformLocation(shader.id(), "texture1"), 0);
        texture.bind(0);

        vao.bind();

        elementBuffer.bind();
        glDrawElements(
			GL_TRIANGLES,
			mesh.indices().size(),
			GL_UNSIGNED_INT,
			(void*)0
		);
    }

    void generateNormals() {
        mesh.generateNormals();
        bufferArray[2].bind();
        bufferArray[2].data(mesh.normals().size() * sizeof(float) * 3, mesh.normals().data());

        vao.bind();
        vao.enableAttrib(2);
        vao.attribPointer(2, bufferArray[2], 3, GL_FLOAT, 0, 0);
    }
    void reBindVertices() {
        bufferArray[0].bind();
        bufferArray[0].data(mesh.vertices().size() * sizeof(float) * 3, mesh.vertices().data());

        vao.bind();
        vao.enableAttrib(0);
        vao.attribPointer(0, bufferArray[0], 3, GL_FLOAT, 0, 0);
    }
    void reBindAll() {
        std::vector<int>bufferSize({3, 2, 3});
        vao.bind();
        for (int i = 0; i < bufferSize.size(); i++) {
            bufferArray[i].bind();
            if (i == 0)
                bufferArray[i].data(mesh.vertices().size() * sizeof(float) * bufferSize[i], mesh.vertices().data());
            else if (i == 1) {
                bufferArray[i].data(mesh.texCoord2s().size() * sizeof(float) * bufferSize[i], mesh.texCoord2s().data());
            } else {
                bufferArray[i].data(mesh.normals().size() * sizeof(float) * bufferSize[i], mesh.normals().data());
            }
            vao.enableAttrib(i);
            vao.attribPointer(i, bufferArray[i], bufferSize[i], GL_FLOAT, 0, 0);
        }
        elementBuffer.bind();
        elementBuffer.data(mesh.indices().size() * sizeof(unsigned int), mesh.indices().data());
    }
};
