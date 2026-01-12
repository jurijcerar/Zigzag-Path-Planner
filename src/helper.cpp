#include "helper.h"
#include <algorithm>
#include <cmath>

// Helper function to compute centroid of picked points (used for ordering)
Eigen::Vector3f computeCentroid(const std::vector<pcl::PointXYZ>& points) {
    Eigen::Vector3f centroid(0,0,0);
    for (const auto& p : points)
        centroid += Eigen::Vector3f(p.x, p.y, p.z);
    centroid /= points.size();
    return centroid;
}

void orderRectanglePoints(const std::vector<pcl::PointXYZ>& picked, 
                          pcl::PointXYZ& a, pcl::PointXYZ& b, 
                          pcl::PointXYZ& c, pcl::PointXYZ& d) {

    Eigen::Vector3f centroid = computeCentroid(picked);

    Eigen::Vector3f p0(picked[0].x,picked[0].y,picked[0].z);
    Eigen::Vector3f p1(picked[1].x,picked[1].y,picked[1].z);
    Eigen::Vector3f p2(picked[2].x,picked[2].y,picked[2].z);

    //get normal of the plane defined by first three points (normalized to avoid scaling issues (unit vec))
    Eigen::Vector3f normal = (p1-p0).cross(p2-p0).normalized();

    //unit vec orthogonal to normal
    Eigen::Vector3f u = normal.unitOrthogonal();

    //another unit vec perpendicular to both normal and u
    Eigen::Vector3f v = normal.cross(u);

    struct P2{ pcl::PointXYZ p; float angle; };
    std::vector<P2> pts;


    //https://stackoverflow.com/questions/6880899/sort-a-set-of-3-d-points-in-clockwise-counter-clockwise-order
    //and ChatGPT
    for (auto& p : picked) {
        // Step 1: Compute the vector from the centroid to the current point
        // This vector points from the center of the rectangle to the point
        Eigen::Vector3f dvec;
        dvec.x() = p.x - centroid.x();
        dvec.y() = p.y - centroid.y();
        dvec.z() = p.z - centroid.z();

        // Step 2: Project the vector onto the local plane axes
        // u -> local x-axis in the rectangle's plane
        // v -> local y-axis in the rectangle's plane
        float x_coord = dvec.dot(u);  // projection onto u (x in plane)
        float y_coord = dvec.dot(v);  // projection onto v (y in plane)

        // Step 3: Compute the angle of the point relative to the centroid in the plane
        // atan2(y, x) gives the counterclockwise angle from the u-axis
        float angle = std::atan2(y_coord, x_coord);

        // Step 4: Store the point together with its angle
        // This allows us to sort points by angle later
        pts.push_back({p, angle});
    }


    //sort counterclockwise by angle
    std::sort(pts.begin(), pts.end(), [](auto& a, auto& b){ return a.angle < b.angle; });

    a = pts[0].p;
    b = pts[1].p;
    c = pts[2].p;
    d = pts[3].p;
}

//Find nearest point on surface using KdTree
pcl::PointXYZ projectToSurface(const pcl::PointXYZ& p,
                               pcl::search::KdTree<pcl::PointXYZ>::Ptr tree) {
    std::vector<int> idx(1);
    std::vector<float> dist(1);
    if (tree->nearestKSearch(p, 1, idx, dist) > 0)
        return tree->getInputCloud()->points[idx[0]];
    return p;
}

void saveTrajectoryFile(const std::vector<PathPoint>& trajectory) {
    std::ofstream out("trajectory.txt", std::ios::out | std::ios::trunc);

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& pt = trajectory[i];
        out << i << " "
            << pt.position.x << " "
            << pt.position.y << " "
            << pt.position.z << " "
            << pt.normal.x() << " "
            << pt.normal.y() << " "
            << pt.normal.z() << "\n";
    }

    std::cout << "Trajectory saved to trajectory.txt\n";
}