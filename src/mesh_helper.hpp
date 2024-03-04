#pragma once

#include <iostream>
#include "al/graphics/al_Mesh.hpp"

using namespace al;

void addAABB(Mesh& mesh, Vec3f min, Vec3f max) {
    mesh.primitive(Mesh::LINE_STRIP);
    Mesh::Vertex points[8] = {min, Vec3f(max.x, min.y, min.z), 
                       Vec3f(min.x, max.y, min.z), Vec3f(max.x, max.y, min.z),
                       Vec3f(min.x, min.y, max.z), Vec3f(max.x, min.y, max.z),
                       Vec3f(min.x, max.y, max.z), max};

    for (int i = 0; i < 4; i++) {
        mesh.vertex(points[i * 2]);
        mesh.vertex(points[i * 2 + 1]);
        mesh.vertex(points[i]);
        mesh.vertex(points[i + 4]);
        if (i < 2) {
            mesh.vertex(points[i]);
            mesh.vertex(points[i + 2]);
        } else {
            mesh.vertex(points[2 + i]);
            mesh.vertex(points[2 + i + 2]);
        }
    }
} 