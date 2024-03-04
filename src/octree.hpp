#pragma once

#include <iostream>
#include "al/graphics/al_Mesh.hpp"

using namespace al;
struct OctreeNode
{
    int data;
    int depth;
    float xmin, xmax;
    float ymin, ymax;
    float zmin, zmax;
    OctreeNode *children[8];

    OctreeNode // 节点类
        (float nodeValue = 0.0f,
         float xminValue = 0.0f, float xmaxValue = 0.0f,
         float yminValue = 0.0f, float ymaxValue = 0.0f,
         float zminValue = 0.0f, float zmaxValue = 0.0f,
         OctreeNode *top_left_front_Node = NULL,
         OctreeNode *top_left_back_Node = NULL,
         OctreeNode *top_right_front_Node = NULL,
         OctreeNode *top_right_back_Node = NULL,
         OctreeNode *bottom_left_front_Node = NULL,
         OctreeNode *bottom_left_back_Node = NULL,
         OctreeNode *bottom_right_front_Node = NULL,
         OctreeNode *bottom_right_back_Node = NULL)
        : data(nodeValue),
          xmin(xminValue), xmax(xmaxValue),
          ymin(yminValue), ymax(ymaxValue),
          zmin(zminValue), zmax(zmaxValue)
    {
        children[0] = top_left_front_Node,
        children[1] = top_left_back_Node;
        children[2] = top_right_front_Node;
        children[3] = top_right_back_Node;
        children[4] = bottom_left_front_Node;
        children[5] = bottom_left_back_Node;
        children[6] = bottom_right_front_Node;
        children[7] = bottom_right_back_Node;
    }
};

void deleteTree(OctreeNode * & root)
{
    if (root != nullptr)
    {
        for (int i = 0; i < 8; i++)
        {
            deleteTree(root->children[i]);
        }
        delete root;
        root = nullptr;
    }
}

void createMeshOctree(OctreeNode *root, Mesh &mesh, float xmin, float xmax,
                      float ymin, float ymax,
                      float zmin, float zmax, int maxDepth)
{
    if (maxDepth-- > 0)
    {
        int num = 0;
        root->xmin = xmin;
        root->xmax = xmax;
        root->ymin = ymin;
        root->ymax = ymax;
        root->zmin = zmin;
        root->zmax = zmax;
        for (auto &vert : mesh.vertices())
        {
            if (vert.x > xmin && vert.x < xmax &&
                vert.y > ymin && vert.y < ymax &&
                vert.z > zmin && vert.z < zmax)
            {
                num++;
            }
        }
        root->data = num;
        //std::cout << num << std::endl;
        if (num == 0)
            return;
        float xm = (xmax - xmin) / 2;
        float ym = (ymax - ymin) / 2;
        float zm = (ymax - ymin) / 2;
        for (int i = 0; i < 8; i++)
        {
            root->children[i] = new OctreeNode();
            root->children[i]->depth = root->depth + 1;
        }
        createMeshOctree(root->children[0], mesh, xmin, xmax - xm, ymax - ym, ymax, zmax - zm, zmax, maxDepth);
        createMeshOctree(root->children[1], mesh, xmin, xmax - xm, ymin, ymax - ym, zmax - zm, zmax, maxDepth);
        createMeshOctree(root->children[2], mesh, xmax - xm, xmax, ymax - ym, ymax, zmax - zm, zmax, maxDepth);
        createMeshOctree(root->children[3], mesh, xmax - xm, xmax, ymin, ymax - ym, zmax - zm, zmax, maxDepth);
        createMeshOctree(root->children[4], mesh, xmin, xmax - xm, ymax - ym, ymax, zmin, zmax - zm, maxDepth);
        createMeshOctree(root->children[5], mesh, xmin, xmax - xm, ymin, ymax - ym, zmin, zmax - zm, maxDepth);
        createMeshOctree(root->children[6], mesh, xmax - xm, xmax, ymax - ym, ymax, zmin, zmax - zm, maxDepth);
        createMeshOctree(root->children[7], mesh, xmax - xm, xmax, ymin, ymax - ym, zmin, zmax - zm, maxDepth);
        for (int i = 0; i < 8; i++)
        {
            if (root->children[i]->data == 0)
                deleteTree(root->children[i]);
        }
    }
}

void octreeToMesh(OctreeNode *p, Mesh& mesh, int& maxDepth) {
    if (p != nullptr)
    {
        if (maxDepth == p->depth) {
            mesh.vertex(Vec3f((p->xmax + p->xmin) / 2, 
                              (p->ymax + p->ymin) / 2,
                              (p->zmax + p->zmin) / 2));
        } else {
            for (int i = 0; i < 8; i++) {
                octreeToMesh(p->children[i], mesh, maxDepth);
            }
        }
    }
}

void preOrder(OctreeNode *p)
{
    if (p != nullptr)
    {
        std::cout << "current data: " << p->data << " depth: " << p->depth << "\n pos: ";
        std::cout << "xmin: " << p->xmin << " xmax: " << p->xmax << "|";
        std::cout << "ymin: " << p->ymin << " ymax: " << p->ymax << "|";
        std::cout << "zmin: " << p->zmin << " zmax: " << p->zmax << "|";
        std::cout << std::endl;

        for (int i = 0; i < 8; i++) {
            preOrder(p->children[i]);
        }
    }
}

int nodeNum(OctreeNode *root) {
    int num = 1;
    if (root != nullptr) {
        for (int i = 0; i < 8; i++) {
            num += nodeNum(root->children[i]);
        }
    } else {
        num = 0;
    }
    return num;
}

int depth(OctreeNode *root)
{
    if (root == nullptr)
    {
        return -1;
    }
    int maxDepth = 1;
    for (int i = 0; i < 8; i++)
    {
        maxDepth = max(depth(root->children[i]), maxDepth);
    }
    return maxDepth;
}