#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "al/graphics/al_BufferObject.hpp"
#include "al/graphics/al_Graphics.hpp"
#include "al/graphics/al_Shader.hpp"
#include "al/graphics/al_Image.hpp"
#include "al/io/al_ControlNav.hpp"

using namespace al;

class Skybox
{
public:
    ShaderProgram shader;
    VAO vao;
    BufferObject buffer;
    GLuint skyTexture;
public:
    Skybox(std::vector<std::string> faces,
           const std::string shaderPath = "./shaders/skybox") {
        if (faces.size() != 6) {
            std::cout << "skybox file error, size != 6" << std::endl;
        }
        float skyboxVertices[] = {
            // positions          
            -1.0f,  1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f, -1.0f,

            -1.0f, -1.0f,  1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f,  1.0f,
            -1.0f, -1.0f,  1.0f,

            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f,  1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,

            -1.0f, -1.0f,  1.0f,
            -1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f, -1.0f,  1.0f,
            -1.0f, -1.0f,  1.0f,

            -1.0f,  1.0f, -1.0f,
            1.0f,  1.0f, -1.0f,
            1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,
            -1.0f,  1.0f,  1.0f,
            -1.0f,  1.0f, -1.0f,

            -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f,  1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f,  1.0f,
            1.0f, -1.0f,  1.0f
        };
        vao.create();
        vao.bind();
        buffer.bufferType(GL_ARRAY_BUFFER);
        buffer.usage(GL_STATIC_DRAW);
        buffer.create();
        buffer.bind();
        buffer.data(sizeof(skyboxVertices), skyboxVertices);
        vao.enableAttrib(0);
        vao.attribPointer(0, buffer, 3, GL_FLOAT, 0, 0);
        loadCubeMap(faces);

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

    };

    void loadCubeMap(std::vector<std::string> faces) {
        glGenTextures(1, &skyTexture);
        glBindTexture(GL_TEXTURE_CUBE_MAP, skyTexture);
        int width, height, nrComponents;
        
        for (int i = 0; i < faces.size(); i++) {
            Image image;
            image.load(faces[i]);
            if (image.array().size() == 0)
            {
                std::cout << "failed to load image " << faces[i] << std::endl;
            } else {
                std::cout << "loaded image size: " << image.width() << ", "
                    << image.height() << std::endl;
                glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 
                             0, 
                             GL_RGBA, 
                             image.width(), 
                             image.height(), 
                             0, 
                             GL_RGBA, 
                             GL_UNSIGNED_BYTE, 
                             image.array().data());
            }
        }

        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    }

    void onDraw(Graphics& g, Nav& camera) {
        g.depthMask(GL_FALSE);
        shader.use();
        vao.bind();
        glBindTexture(GL_TEXTURE_CUBE_MAP, skyTexture);
        shader.uniform("projection", g.projMatrix());
        shader.uniform("view", g.viewMatrix());
        glDrawArrays(GL_TRIANGLES, 0, 36);
        g.depthMask(GL_TRUE);
    }
};
