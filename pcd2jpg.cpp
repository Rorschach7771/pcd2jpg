#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/filters/voxel_grid.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <filesystem>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace fs = std::filesystem;

int main() {
// 读取PCD点云地图
fs::path currentPath = fs::current_path();

std::string pcd_file = currentPath;
PointCloud::Ptr cloud(new PointCloud);
if (pcl::io::loadPCDFile<PointT>(pcd_file+"/GlobalMap.pcd", *cloud) == -1) {
    std::cerr << "Unable to read PCD file." << std::endl;
    return -1;
}

std::ofstream file("transData.txt",std::ios::trunc);


pcl::VoxelGrid<PointT> voxel_grid;
voxel_grid.setInputCloud(cloud);
voxel_grid.setLeafSize(0.1f,0.1f,0.1f);

PointCloud::Ptr filtered_cloud (new PointCloud);
voxel_grid.filter(*filtered_cloud);


// 计算点云的投影尺寸
float max_x = -std::numeric_limits<float>::max();
float min_x = std::numeric_limits<float>::max();
float max_y = -std::numeric_limits<float>::max();
float min_y = std::numeric_limits<float>::max();
for (const auto& point : cloud->points) {
    max_x = std::max(max_x, point.x);
    min_x = std::min(min_x, point.x);
    max_y = std::max(max_y, point.y);
    min_y = std::min(min_y, point.y);
}
int width = std::ceil(max_x - min_x) + 1;
int height = std::ceil(max_y - min_y) + 1;
file<<min_x<<"  "<<min_y<<"  "<<"10"<<std::endl;
file.close();
// 创建高分辨率的图像
cv::Mat image(height * 10, width * 10, CV_8UC3, cv::Scalar(255, 255, 255));
cv::Vec3b color = cv::Vec3b(0, 0, 255); 
// 投影点云到二维平面，并根据深度信息设置点的颜色
for (const auto& point : cloud->points) {
    // pointIndex ++;
    float x = (point.x - min_x) * 10;
    float y = (point.y - min_y) * 10;
    float depth = point.z;

    // 根据深度信息设置点的颜色
    if (depth < 15.0) {
        color = cv::Vec3b(0, 0, 255); 
    } else if (depth < 20.0) {
        color = cv::Vec3b(0, 255, 0); // 绿色
    } else {
        color = cv::Vec3b(255, 0, 0); 
    }

    // 在图像上绘制点
    cv::rectangle(image, cv::Rect(x, height*10-y, 1, 1), color, cv::FILLED);

}

// 保存结果为.jpg文件
std::string output_file = "output_image.jpg";
cv::imwrite(output_file, image);
std::cout << "Saved image: " << output_file << std::endl;

return 0;
}