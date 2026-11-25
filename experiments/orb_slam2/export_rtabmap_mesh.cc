#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_surface.h>

int main(int argc, char** argv)
{
    if (argc != 3 || std::filesystem::exists(argv[2])) {
        std::cerr << "Usage: export_rtabmap_mesh MESH_TEXT NEW_DATABASE\n";
        return 2;
    }
    std::ifstream input(argv[1]);
    size_t count_vertices, count_faces;
    if (!(input >> count_vertices >> count_faces) || count_vertices < 3 || !count_faces)
        return 3;

    pcl::PointCloud<pcl::PointXYZRGB> vertices;
    vertices.reserve(count_vertices);
    std::string line;
    std::getline(input, line);
    for (size_t i = 0; i < count_vertices; ++i) {
        pcl::PointXYZRGB p;
        if (!std::getline(input, line))
            return 4;
        std::istringstream fields(line);
        if (!(fields >> p.x >> p.y >> p.z) ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            return 4;
        int red=239, green=239, blue=237;
        fields >> std::ws;
        if (!fields.eof() && (!(fields >> red >> green >> blue) ||
            std::min({red,green,blue}) < 0 || std::max({red,green,blue}) > 255))
            return 4;
        p.r = red;
        p.g = green;
        p.b = blue;
        vertices.push_back(p);
    }
    std::vector<std::vector<std::vector<RTABMAP_PCL_INDEX>>> polygons(1);
    for (size_t i = 0; i < count_faces; ++i) {
        RTABMAP_PCL_INDEX a, b, c;
        if (!(input >> a >> b >> c) || a < 0 || b < 0 || c < 0 ||
            static_cast<size_t>(std::max({a,b,c})) >= count_vertices || a == b || a == c || b == c)
            return 5;
        polygons[0].push_back({a, b, c});
    }
    auto db = std::unique_ptr<rtabmap::DBDriver>(rtabmap::DBDriver::create());
    if (!db || !db->openConnection(argv[2]))
        return 6;
    const auto scan = rtabmap::util3d::laserScanFromPointCloud(vertices);
    db->saveOptimizedMesh(scan.data(), polygons);
    db->closeConnection();
    if (!db->openConnection(argv[2], false, true))
        return 7;
    decltype(polygons) loaded;
    const auto cloud = db->loadOptimizedMesh(&loaded);
    if (cloud.total() != count_vertices || loaded != polygons)
        return 8;
    const auto colors = rtabmap::util3d::laserScanToPointCloudRGB(
        rtabmap::LaserScan::backwardCompatibility(cloud));
    if (colors->size() != vertices.size())
        return 9;
    for (size_t i = 0; i < colors->size(); ++i)
        if (colors->at(i).rgba != vertices.at(i).rgba)
            return 9;
    const auto rebuilt = rtabmap::util3d::assemblePolygonMesh(cloud, loaded.front());
    if (!rebuilt || rebuilt->polygons.size() != count_faces)
        return 9;
    std::cout << "RTAB-Map read-back: " << cloud.total() << " vertices, "
              << rebuilt->polygons.size() << " triangles.\n";
    db->closeConnection();
}
