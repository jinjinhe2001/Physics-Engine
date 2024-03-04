#pragma once
#include "al/math/al_Mat.hpp"
#include "al/math/al_Vec.hpp"

using namespace al;

inline float max(float a, float b)
{
    return a > b ? a : b;
}

Mat4f CrossMatrix(Vec3f v)
{
    Mat4f mat(0, -v[2], v[1], 0,
              v[2], 0, -v[0], 0,
              -v[1], v[0], 0, 0,
              0, 0, 0, 1);
    return mat;
}

Mat4f ScaleMatrix(Vec3f s)
{
    Mat4f mat(s[0], 0, 0, 0,
              0, s[1], 0, 0,
              0, 0, s[2], 0,
              0, 0, 0, 1);
    return mat;
}

bool inBox(Vec3f v, Vec3f min, Vec3f max)
{
    if (v.x > min.x && v.x < max.x &&
        v.y > min.y && v.y < max.y &&
        v.z > min.z && v.z < max.z)
    {
        return true;
    }
    return false;
}

int QuickSortPartitionDoublet(std::vector<int> &a, int l, int r)
{
    int pivot_0, pivot_1, i, j;
    pivot_0 = a[l * 2 + 0];
    pivot_1 = a[l * 2 + 1];
    i = l;
    j = r + 1;
    while (true)
    {
        do ++i; while (i <= r && (a[i * 2] < pivot_0 || a[i * 2] == pivot_0 && a[i * 2 + 1] <= pivot_1));
        do --j; while (a[j * 2] > pivot_0 || a[j * 2] == pivot_0 && a[j * 2 + 1] > pivot_1);
        if (i >= j) break;
        std::swap(a[i * 2], a[j * 2]);
        std::swap(a[i * 2 + 1], a[j * 2 + 1]);
    }
    std::swap(a[l * 2], a[j * 2 + 0]);
    std::swap(a[l * 2 + 1], a[j * 2 + 1]);
    return j;
}

void QuickSort(std::vector<int> &a, int l, int r)
{
    int j;
    if (l < r)
    {
        j = QuickSortPartitionDoublet(a, l, r);
        QuickSort(a, l, j - 1);
        QuickSort(a, j + 1, r);
    }
}