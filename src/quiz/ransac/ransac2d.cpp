/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>
#include "../../processPointClouds.h"
#include "../../render/render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++) {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--) {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd(
        "../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function
    auto cloud_size = cloud->points.size();
    // For max iterations
    for (auto iter = 0; iter < maxIterations; ++iter) {
        auto num1 = cloud->points[rand() % cloud_size];
        auto num2 = cloud->points[rand() % cloud_size];
        auto num3 = cloud->points[rand() % cloud_size];
        // Randomly sample subset and fit line
        auto A = ((num2.y - num1.y) * (num3.z - num1.z) -
                  (num2.z - num1.z) *
                      (num3.y - num1.y));  //(y2-y1)(z3-z1) - (z2-z1)(y3-y1)

        auto B = (num2.z - num1.z) * (num3.x - num1.x) -
                 (num2.x - num1.x) * (num3.z - num1.z);  // x2 - x1
        auto C = (num2.x * num1.x) * (num3.y - num1.y) -
                 (num2.y - num1.y) * (num3.x - num1.x);  // x1y2 - x2y1

        auto D = -(A * num1.x + B * num1.y + C * num1.z);
        // Measure distance between every point and fitted line
        std::unordered_set<int> inliers;
        for (int index = 0; index < cloud->points.size(); ++index) {
            auto point = cloud->points[index];
            auto distance =
                std::abs(A * point.x + B * point.y + C * point.z + D) /
                std::sqrt(A * A + B * B + C * C);
            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol) {
                inliers.insert(index);
            }
        }
        inliersResult =
            inliers.size() > inliersResult.size() ? inliers : inliersResult;
    }
    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;
}

int main() {
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    // TODO: Change the max iteration and distance tolerance arguments for
    // Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 20, 0.2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
        new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size()) {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    } else {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}
