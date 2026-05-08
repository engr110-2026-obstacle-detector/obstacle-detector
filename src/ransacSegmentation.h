#ifndef RANSAC_SEGMENTATION_H
#define RANSAC_SEGMENTATION_H

#include <stdint.h>
#include <stdbool.h>
#include <cmath>
#include <cstdlib>

#include "point.h"


// Plane equation: a*x + b*y + c*z = d
// Normal vector is (a, b, c)
struct Plane {
    float a, b, c, d;
    
    Plane() : a(0), b(0), c(0), d(0) {}
    Plane(float a_, float b_, float c_, float d_) : a(a_), b(b_), c(c_), d(d_) {}
};

// Fit a plane through three 3D points using cross product
// Returns plane equation coefficients
Plane fitPlaneFromPoints(Point p1, Point p2, Point p3) {
    // Two vectors in the plane
    float v1_x = p2.x - p1.x;
    float v1_y = p2.y - p1.y;
    float v1_z = p2.z - p1.z;
    
    float v2_x = p3.x - p1.x;
    float v2_y = p3.y - p1.y;
    float v2_z = p3.z - p1.z;
    
    // Normal vector = v1 cross v2
    float nx = v1_y * v2_z - v1_z * v2_y;
    float ny = v1_z * v2_x - v1_x * v2_z;
    float nz = v1_x * v2_y - v1_y * v2_x;
    
    // Normalize the normal vector
    float norm = sqrt(nx*nx + ny*ny + nz*nz);
    if (norm < 0.001f) {
        // Collinear points - return invalid plane
        return Plane(0, 0, 1, 0);
    }
    
    nx /= norm;
    ny /= norm;
    nz /= norm;
    
    // Plane equation: n·(r - p1) = 0
    // nx*(x - p1.x) + ny*(y - p1.y) + nz*(z - p1.z) = 0
    // nx*x + ny*y + nz*z = nx*p1.x + ny*p1.y + nz*p1.z
    float d = nx * p1.x + ny * p1.y + nz * p1.z;
    
    return Plane(nx, ny, nz, d);
}

// Calculate signed distance from a point to a plane
// Distance = |a*x + b*y + c*z - d| / sqrt(a^2 + b^2 + c^2)
// Since plane is normalized, denominator = 1
float pointToPlaneDistance(Point p, Plane plane) {
    float distance = plane.a * p.x + plane.b * p.y + plane.c * p.z - plane.d;
    return distance;
}

// RANSAC algorithm: find best-fit plane for a set of points
// Returns the plane that has the most inliers within tolerance
// Parameters:
//   points: array of Point objects
//   count: number of points in array
//   tolerance: distance tolerance in mm for inliers (typically 50)
//   maxIterations: maximum RANSAC iterations (typically 50-150)
// Returns:
//   Plane with the most inliers
//   Also fills inlierCount with the number of inliers found
Plane bestFitPlane(Point* points, int count, int tolerance, int maxIterations, int& inlierCount) {
    Plane bestPlane(0, 0, 1, 0);  // Default to z=0 plane
    int bestInlierCount = 0;
    
    // RANSAC main loop
    for (int iter = 0; iter < maxIterations; iter++) {
        // Randomly select 3 distinct points
        int idx1, idx2, idx3;
        
        // Pick first point
        idx1 = rand() % count;
        while (!points[idx1].valid) {
            idx1 = rand() % count;
        }
        
        // Pick second point (different from first)
        do {
            idx2 = rand() % count;
        } while (idx2 == idx1 || !points[idx2].valid);
        
        // Pick third point (different from first two)
        do {
            idx3 = rand() % count;
        } while ((idx3 == idx1 || idx3 == idx2) || !points[idx3].valid);
        
        // Fit plane through these 3 points
        Plane candidatePlane = fitPlaneFromPoints(points[idx1], points[idx2], points[idx3]);
        
        // Check if normal vector is valid (not degenerate)
        if (abs(candidatePlane.a) < 0.001f && abs(candidatePlane.b) < 0.001f && abs(candidatePlane.c) < 0.001f) {
            continue;  // Skip degenerate plane
        }
        
        // Count inliers: points within tolerance of this plane
        int currentInlierCount = 0;
        for (int i = 0; i < count; i++) {
            if (!points[i].valid) continue;
            
            float dist = pointToPlaneDistance(points[i], candidatePlane);
            if (abs(dist) <= tolerance) {
                currentInlierCount++;
            }
        }
        
        // Update best plane if this one has more inliers
        if (currentInlierCount > bestInlierCount) {
            bestInlierCount = currentInlierCount;
            bestPlane = candidatePlane;
        }
    }
    
    inlierCount = bestInlierCount;
    return bestPlane;
}

// Flood-fill to mark connected components
// Marks all adjacent valid points that aren't already assigned a segment
// Uses 4-connectivity (up, down, left, right)
void floodFillSegment(Point pointcloud[8][24], int segmentId, int row, int col, bool visited[8][24]) {
    // Boundary check
    if (row < 0 || row >= 8 || col < 0 || col >= 24) {
        return;
    }
    
    // Already visited or invalid point
    if (visited[row][col] || !pointcloud[row][col].valid) {
        return;
    }
    
    // Mark as visited and assign segment
    visited[row][col] = true;
    pointcloud[row][col].segment = segmentId;
    
    // Recursively fill 4-neighbors
    floodFillSegment(pointcloud, segmentId, row - 1, col, visited);  // up
    floodFillSegment(pointcloud, segmentId, row + 1, col, visited);  // down
    floodFillSegment(pointcloud, segmentId, row, col - 1, visited);  // left
    floodFillSegment(pointcloud, segmentId, row, col + 1, visited);  // right
}

#endif // RANSAC_SEGMENTATION_H
