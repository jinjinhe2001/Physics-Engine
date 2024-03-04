// deprecated 
#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include "al/graphics/al_Mesh.hpp"

using namespace al;

namespace helper {
    struct FixedMesh
    {
        Mesh mesh;
        std::vector<unsigned int> uvIndices;
        std::vector<unsigned int> normalIndices;
        Mesh& operator()() {
            return mesh;
        }
    };

    class Object
    {   
    public:
        Object() {}
        FixedMesh fMesh;
        Image image;
        Texture texture;
        Material material;
        Shader shader;
        void loadTexture(const std::string filename);
    };

    Color uv2Color(Image& image, float u, float v) {
        Image::RGBAPix rgb = image.at(round(u * image.width()), round(v * image.height()));
        return Color(rgb.r, rgb.g, rgb.b, rgb.a);
    }

    void Object::loadTexture(const std::string filename) {
        image.load(filename);

        if (image.array().size() == 0) {
            std::cout << "failed to load image " << filename << std::endl;
        }
        std::cout << "loaded image size: " << image.width() << ", "
            << image.height() << std::endl;

        texture.create2D(image.width(), image.height());
        texture.submit(image.array().data(), GL_RGBA, GL_UNSIGNED_BYTE);

        texture.filter(Texture::LINEAR);
    }

    void stringSplit(std::string &str, const char& split, std::vector<std::string>& rst)
    {
        std::istringstream iss(str);
        std::string token;
        while (getline(iss, token, split))
        {
            rst.push_back(token);
        }
    }

    void stoiVUvN(std::vector<std::string>& rst, int& vIndex, int& uvIndex, int& nIndex)
    {
        vIndex = stoi(rst[0]);
        uvIndex = stoi(rst[1]);
        nIndex = stoi(rst[2]);
    }

    void loadObjFile(FixedMesh& mesh, const std::string filename) {
        std::ifstream in(filename.c_str(), std::ios::in); 
        if (!in.good())
        {
            std::cout << "ERROR: loading obj:(" << filename << ") file is not good" << "\n";
            exit(0);
        }
        std::string line;
        std::vector<Vec2f> texCoords;

        while (!in.eof())
        {
            std::getline(in, line);
            std::istringstream iss(line.c_str());
            char trash;
            if (!line.compare(0, 2, "v "))
            {
                iss >> trash;
                float x, y, z;
                iss >> x >> y >> z;
                mesh().vertex(x, y, z);
            }
            else if (!line.compare(0, 3, "vn "))
            {
                iss >> trash >> trash;
                float x, y, z;
                iss >> x >> y >> z;
                mesh().normal(x, y, z);
            }
            else if (!line.compare(0, 3, "vt "))
            {
                iss >> trash >> trash;
                float x, y;
                iss >> x >> y;
                texCoords.push_back(Vec2f(x, 1 - y));
            }
            else if (!line.compare(0, 2, "f "))
            {
                iss >> trash;
                std::string str;
                int vIndex, uvIndex, nIndex;
                iss >> str;
                if (str.find('/') != std::string::npos)
                {
                    std::vector<std::string> rst;
                    stringSplit(str,'/', rst);
                    stoiVUvN(rst, vIndex, uvIndex, nIndex);
                    mesh().index(vIndex - 1);
                    mesh.uvIndices.push_back(uvIndex - 1);
                    mesh.normalIndices.push_back(nIndex - 1);
                    while (iss >> str)
                    {
                        rst.clear();
                        stringSplit(str, '/', rst);
                        stoiVUvN(rst, vIndex, uvIndex, nIndex);
                        mesh().index(vIndex - 1);
                        mesh.uvIndices.push_back(uvIndex - 1);
                        mesh.normalIndices.push_back(nIndex - 1);
                    }
                }
                else
                {
                    int vIndex1, vIndex2, vIndex3;
                    vIndex1 = stoi(str);
                    iss >> vIndex2 >> vIndex3;
                    mesh().index(vIndex1 - 1);
                    mesh().index(vIndex2 - 1);
                    mesh().index(vIndex3 - 1);
                }
            }
        }
        if (mesh.uvIndices.size() != 0) {
            
            mesh().texCoord2s().resize(mesh().vertices().size());
            for (int i = 0; i < mesh.uvIndices.size(); i++) {
                mesh().texCoord2s()[mesh().indices().at(i)] = texCoords.at(mesh.uvIndices.at(i));
            }
        }
    }
}