#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
    // 读取点云文件
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("E:\\Tomato_data\\seg_organs\\22_stem_pc.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file 22_stem_pc.pcd\n");
        return -1;
    }

    // 将点云投影至xy平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0.0;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    // 欧式聚类
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud_projected);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.02); // 设置聚类的欧氏距离阈值
    ec.setMinClusterSize(100);   // 设置每个聚类的最小点数
    ec.setMaxClusterSize(25000); // 设置每个聚类的最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_projected);
    ec.extract(cluster_indices);

    std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;

    // 创建 PCL 可视化窗口1（原始茎秆点云）
    pcl::visualization::PCLVisualizer::Ptr viewer_original(new pcl::visualization::PCLVisualizer("Original Point Cloud Viewer"));
    viewer_original->addPointCloud(cloud, "original_cloud");

    // 创建 PCL 可视化窗口2（分割后的茎秆点云）
    pcl::visualization::PCLVisualizer::Ptr viewer_segmented(new pcl::visualization::PCLVisualizer("Segmented Point Cloud Viewer"));

    // 分割并保存结果，并可视化分割后的茎秆点云（每个聚类使用不同颜色）
    for (int i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr stem_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

        for (int j = 0; j < cluster_indices[i].indices.size(); ++j) {
            int index = cluster_indices[i].indices[j];
            stem_cloud->points.push_back(cloud->points[index]);
        }

        stem_cloud->width = stem_cloud->size();
        stem_cloud->height = 1;
        stem_cloud->is_dense = true;

        // 可视化分割后的茎秆点云（每个聚类使用不同颜色）
        std::string stem_cloud_name = "stem_cloud_" + std::to_string(i + 1);
        viewer_segmented->addPointCloud(stem_cloud, stem_cloud_name);
        viewer_segmented->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rand() % 256 / 255.0, rand() % 256 / 255.0, rand() % 256 / 255.0, stem_cloud_name);

        // 保存分割后的茎秆点云
        std::string stem_file = "E:\\Tomato_data\\seg_organs\\22_stem_" + std::to_string(i + 1) + "_pc.pcd";
        pcl::io::savePCDFile(stem_file, *stem_cloud);
    }

    // 启动可视化窗口1（原始茎秆点云）
    while (!viewer_original->wasStopped()) {
        viewer_original->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 启动可视化窗口2（分割后的茎秆点云）
    while (!viewer_segmented->wasStopped()) {
        viewer_segmented->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
