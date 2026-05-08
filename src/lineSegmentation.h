#ifndef LINE_SEGMENTATION_H
#define LINE_SEGMENTATION_H

#include <stdint.h>
#include <stdbool.h>
#include <cmath>
#include <cstdlib>

#include "point.h"

// 3D Line representation: parametric form P(t) = P0 + t*D
// P0: point on the line
// D: direction vector (normalized)
struct Line {
    float p0_x, p0_y, p0_z;  // Point on line
    float d_x, d_y, d_z;      // Direction vector (normalized)
    
    Line() : p0_x(0), p0_y(0), p0_z(0), d_x(1), d_y(0), d_z(0) {}
    Line(float px, float py, float pz, float dx, float dy, float dz)
        : p0_x(px), p0_y(py), p0_z(pz), d_x(dx), d_y(dy), d_z(dz) {}
};

// Fit a line through two 3D points
// Returns line in parametric form: P(t) = p1 + t*(p2-p1)
Line fitLineFromPoints(Point p1, Point p2) {
    // Direction vector from p1 to p2
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    float dz = p2.z - p1.z;
    
    // Normalize direction vector
    float norm = sqrt(dx*dx + dy*dy + dz*dz);
    if (norm < 0.001f) {
        // Degenerate case: points are the same
        return Line(p1.x, p1.y, p1.z, 1, 0, 0);
    }
    
    dx /= norm;
    dy /= norm;
    dz /= norm;
    
    return Line(p1.x, p1.y, p1.z, dx, dy, dz);
}

// Calculate distance from a point to a 3D line
// Line is defined as: P(t) = P0 + t*D
// Distance = ||(P - P0) × D|| / ||D||
// Since D is normalized, ||D|| = 1
float pointToLineDistance(Point p, Line line) {
    // Vector from line point to point P
    float px = p.x - line.p0_x;
    float py = p.y - line.p0_y;
    float pz = p.z - line.p0_z;
    
    // Cross product: (P - P0) × D
    float cross_x = py * line.d_z - pz * line.d_y;
    float cross_y = pz * line.d_x - px * line.d_z;
    float cross_z = px * line.d_y - py * line.d_x;
    
    // Magnitude of cross product
    float distance = sqrt(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
    return distance;
}

// RANSAC algorithm: find best-fit line for a set of points
// Samples 2 points at a time to define candidate lines
// Returns line with most inliers within tolerance
// Parameters:
//   points: array of Point objects
//   count: number of points in array
//   tolerance: distance tolerance in mm for inliers (typically 50)
//   maxIterations: maximum RANSAC iterations (typically 100)
// Returns:
//   Line with the most inliers
//   Also fills inlierCount with the number of inliers found
Line bestFitLine3D(Point* points, int count, int tolerance, int maxIterations, int& inlierCount) {
    Line bestLine(0, 0, 0, 1, 0, 0);  // Default line along x-axis
    int bestInlierCount = 0;
    
    // RANSAC main loop
    for (int iter = 0; iter < maxIterations; iter++) {
        // Randomly select 2 distinct valid points
        int idx1, idx2;
        
        // Pick first point
        idx1 = rand() % count;
        while (!points[idx1].valid) {
            idx1 = rand() % count;
        }
        
        // Pick second point (different from first)
        do {
            idx2 = rand() % count;
        } while ((idx2 == idx1) || !points[idx2].valid);
        
        // Fit line through these 2 points
        Line candidateLine = fitLineFromPoints(points[idx1], points[idx2]);
        
        // Check if direction vector is valid (not degenerate)
        if (abs(candidateLine.d_x) < 0.001f && abs(candidateLine.d_y) < 0.001f && abs(candidateLine.d_z) < 0.001f) {
            continue;  // Skip degenerate line
        }
        
        // Count inliers: points within tolerance of this line
        int currentInlierCount = 0;
        for (int i = 0; i < count; i++) {
            if (!points[i].valid) continue;
            
            float dist = pointToLineDistance(points[i], candidateLine);
            if (dist <= tolerance) {
                currentInlierCount++;
            }
        }
        
        // Update best line if this one has more inliers
        if (currentInlierCount > bestInlierCount) {
            bestInlierCount = currentInlierCount;
            bestLine = candidateLine;
        }
    }
    
    inlierCount = bestInlierCount;
    return bestLine;
}

// Segment a single column (8 points) into groups based on discontinuities
// Returns a list of point indices for each segment
// A discontinuity is detected when:
//   - Distance between consecutive valid points exceeds discontinuityThreshold
// Invalid points are skipped but do NOT cause discontinuities
// Minimum 3 valid points required per segment
// Returns number of segments found (stored in segmentInfo)
struct SegmentInfo {
    int startIdx;  // Index of first point in segment (0-7)
    int endIdx;    // Index of last point in segment (0-7, inclusive)
    int pointCount; // Number of valid points in segment
};

int detectColumnSegments(Point* column, int columnSize, int discontinuityThreshold, 
                         SegmentInfo* segments, int maxSegments) {
    int segmentCount = 0;
    int currentSegmentStart = -1;
    int currentSegmentEnd = -1;
    int validPointCount = 0;  // Count of valid points in current segment
    Point* lastValidPoint = NULL;
    
    for (int i = 0; i < columnSize; i++) {
        if (!column[i].valid) {
            // Skip invalid points - they don't affect segmentation
            continue;
        }
        
        // Valid point processing
        if (currentSegmentStart == -1) {
            // Start new segment
            currentSegmentStart = i;
            currentSegmentEnd = i;
            validPointCount = 1;
            lastValidPoint = &column[i];
        } else {
            // Check for discontinuity with previous valid point
            float dx = column[i].x - lastValidPoint->x;
            float dy = column[i].y - lastValidPoint->y;
            float dz = column[i].z - lastValidPoint->z;
            float distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance > discontinuityThreshold) {
                // Discontinuity detected - end current segment, start new one
                if (validPointCount >= 3 && segmentCount < maxSegments) {
                    segments[segmentCount].startIdx = currentSegmentStart;
                    segments[segmentCount].endIdx = currentSegmentEnd;
                    segments[segmentCount].pointCount = validPointCount;
                    segmentCount++;
                }
                currentSegmentStart = i;
                currentSegmentEnd = i;
                validPointCount = 1;
            } else {
                // Continue segment
                currentSegmentEnd = i;
                validPointCount++;
            }
            
            lastValidPoint = &column[i];
        }
    }
    
    // Handle last segment
    if (currentSegmentStart != -1 && validPointCount >= 3 && segmentCount < maxSegments) {
        segments[segmentCount].startIdx = currentSegmentStart;
        segments[segmentCount].endIdx = currentSegmentEnd;
        segments[segmentCount].pointCount = validPointCount;
        segmentCount++;
    }
    
    return segmentCount;
}

#endif // LINE_SEGMENTATION_H
