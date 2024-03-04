#include <vector>
#include <iostream>
#include <string>
#include <cstring>
#include <map>

#include "al/math/al_Vec.hpp"
#include "al/graphics/al_Mesh.hpp"
#include "al/graphics/al_Texture.hpp"
#include "al/graphics/al_Image.hpp"

using namespace al;
// Considering writing loader function to a class

/// @brief use loadOBJ to load raw .obj data
/// @param path
/// @param out_vertices
/// @param out_uvs
/// @param out_normals
/// @return
bool loadOBJ(
    const char *path,
    std::vector<Vec3f> &out_vertices,
    std::vector<Vec2f> &out_uvs,
    std::vector<Vec3f> &out_normals)
{
    printf("Loading OBJ file %s...\n", path);

    std::vector<unsigned int> vertexIndices, uvIndices, normalIndices;
    std::vector<Vec3f> temp_vertices;
    std::vector<Vec2f> temp_uvs;
    std::vector<Vec3f> temp_normals;

    FILE *file = fopen(path, "r");
    if (file == NULL)
    {
        printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
        getchar();
        return false;
    }

    while (1)
    {

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.

        // else : parse lineHeader

        if (strcmp(lineHeader, "v") == 0)
        {
            Vec3f vertex;
            fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
            temp_vertices.push_back(vertex);
        }
        else if (strcmp(lineHeader, "vt") == 0)
        {
            Vec2f uv;
            fscanf(file, "%f %f\n", &uv.x, &uv.y);
            uv.y = 1 - uv.y; // Invert V coordinate since we will only use DDS texture, which are inverted. Remove if you want to use TGA or BMP loaders.
            temp_uvs.push_back(uv);
        }
        else if (strcmp(lineHeader, "vn") == 0)
        {
            Vec3f normal;
            fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
            temp_normals.push_back(normal);
        }
        else if (strcmp(lineHeader, "f") == 0)
        {
            std::string vertex1, vertex2, vertex3;
            unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
            int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2]);
            if (matches != 9)
            {
                printf("File can't be read by our simple parser :-( Try exporting with other options\n");
                fclose(file);
                return false;
            }
            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);
            uvIndices.push_back(uvIndex[0]);
            uvIndices.push_back(uvIndex[1]);
            uvIndices.push_back(uvIndex[2]);
            normalIndices.push_back(normalIndex[0]);
            normalIndices.push_back(normalIndex[1]);
            normalIndices.push_back(normalIndex[2]);
        }
        else
        {
            // Probably a comment, eat up the rest of the line
            char stupidBuffer[1000];
            fgets(stupidBuffer, 1000, file);
        }
    }

    // For each vertex of each triangle
    for (unsigned int i = 0; i < vertexIndices.size(); i++)
    {

        // Get the indices of its attributes
        unsigned int vertexIndex = vertexIndices[i];
        unsigned int uvIndex = uvIndices[i];
        unsigned int normalIndex = normalIndices[i];
        if (normalIndex > temp_normals.size())
            normalIndex = temp_normals.size();
        if (uvIndex > temp_uvs.size())
            uvIndex = temp_uvs.size();

        // Get the attributes thanks to the index
        Vec3f vertex = temp_vertices.at(vertexIndex - 1);
        Vec2f uv = uvIndex != 0 ? temp_uvs.at(uvIndex - 1) : Vec2f(0, 0);
        Vec3f normal = normalIndex != 0 ? temp_normals.at(normalIndex - 1) : Vec3f(0);

        // Put the attributes in buffers
        out_vertices.push_back(vertex);
        out_uvs.push_back(uv);
        out_normals.push_back(normal);
    }

    fclose(file);
    return true;
}

struct PackedVertex
{
    Vec3f position;
    Vec2f uv;
    Vec3f normal;
    bool operator<(const PackedVertex that) const
    {
        return memcmp((void *)this, (void *)&that, sizeof(PackedVertex)) > 0;
    };
};

bool getSimilarVertexIndex_fast(
    PackedVertex &packed,
    std::map<PackedVertex, unsigned short> &VertexToOutIndex,
    unsigned short &result)
{
    std::map<PackedVertex, unsigned short>::iterator it = VertexToOutIndex.find(packed);
    if (it == VertexToOutIndex.end())
    {
        return false;
    }
    else
    {
        result = it->second;
        return true;
    }
}

/// @brief use indexVBO to get the data into the right order
/// @param in_vertices
/// @param in_uvs
/// @param in_normals
/// @param out_indices
/// @param out_vertices
/// @param out_uvs
/// @param out_normals
void indexVBO(
    std::vector<Vec3f> &in_vertices,
    std::vector<Vec2f> &in_uvs,
    std::vector<Vec3f> &in_normals,

    std::vector<Mesh::Index> &out_indices,
    std::vector<Vec3f> &out_vertices,
    std::vector<Vec2f> &out_uvs,
    std::vector<Vec3f> &out_normals)
{
    std::map<PackedVertex, unsigned short> VertexToOutIndex;

    // For each input vertex
    for (unsigned int i = 0; i < in_vertices.size(); i++)
    {

        PackedVertex packed = {in_vertices[i], in_uvs[i], in_normals[i]};

        // Try to find a similar vertex in out_XXXX
        unsigned short index;
        bool found = getSimilarVertexIndex_fast(packed, VertexToOutIndex, index);

        if (found)
        { // A similar vertex is already in the VBO, use it instead !
            out_indices.push_back(index);
        }
        else
        { // If not, it needs to be added in the output data.
            out_vertices.push_back(in_vertices[i]);
            out_uvs.push_back(in_uvs[i]);
            out_normals.push_back(in_normals[i]);
            unsigned short newindex = (unsigned short)out_vertices.size() - 1;
            out_indices.push_back(newindex);
            VertexToOutIndex[packed] = newindex;
        }
    }
}

void loadTexture(Texture &texture, const std::string filename)
{
    Image image;
    image.load(filename);

    if (image.array().size() == 0)
    {
        std::cout << "failed to load image " << filename << std::endl;
    }
    std::cout << "loaded image size: " << image.width() << ", "
              << image.height() << std::endl;

    texture.create2D(image.width(), image.height());
    texture.submit(image.array().data(), GL_RGBA, GL_UNSIGNED_BYTE);

    texture.filter(Texture::LINEAR);
}