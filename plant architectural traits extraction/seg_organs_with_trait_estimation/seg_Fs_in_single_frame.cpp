#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <thread>
#include <chrono>
#include <vector>

int main(int argc, char** argv) {
    // 读取点云
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("E:\\Tomato_data\\seg_organs\\22_fruit_pc.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file 22_fruit_pc.pcd\n");
        return -1;
    }

    // 创建欧氏聚类对象
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.07); // 设置聚类的容差
    ec.setMinClusterSize(50);   // 设置最小聚类的点数
    ec.setMaxClusterSize(25000); // 设置最大聚类的点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 输出聚类数
    std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;

    // 创建第一个可视化窗口（原始点云）
    pcl::visualization::PCLVisualizer viewer_original("Original Cloud Viewer");
    viewer_original.setBackgroundColor(0, 0, 0);
    viewer_original.addPointCloud(cloud, "original_cloud");

    // 创建第二个可视化窗口（分割后的点云）
    pcl::visualization::PCLVisualizer viewer_segmented("Segmented Cloud Viewer");
    viewer_segmented.setBackgroundColor(0, 0, 0);

    for (int i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (const auto& index : cluster_indices[i].indices) {
            cluster->points.push_back(cloud->points[index]);
        }

        // 在可视化窗口中给每个聚类赋予不同的颜色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(cluster, rand() % 256, rand() % 256, rand() % 256);
        viewer_segmented.addPointCloud(cluster, single_color, "cluster_" + std::to_string(i + 1));

        // 保存聚类点云（保留原色）
        pcl::io::savePCDFileBinary("E:\\Tomato_data\\seg_organs\\22_fruit_" + std::to_string(i + 1) + "_pc.pcd", *cluster);
    }

    // 启动第一个可视化窗口（原始点云）
    while (!viewer_original.wasStopped()) {
        viewer_original.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 启动第二个可视化窗口（分割后的点云）
    while (!viewer_segmented.wasStopped()) {
        viewer_segmented.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
