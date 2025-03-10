#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <thread>
#include <chrono>
#include <vector>

int main(int argc, char** argv) {
    // ��ȡ����
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("E:\\Tomato_data\\seg_organs\\22_fruit_pc.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file 22_fruit_pc.pcd\n");
        return -1;
    }

    // ����ŷ�Ͼ������
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    ec.setClusterTolerance(0.07); // ���þ�����ݲ�
    ec.setMinClusterSize(50);   // ������С����ĵ���
    ec.setMaxClusterSize(25000); // ����������ĵ���
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // ���������
    std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;

    // ������һ�����ӻ����ڣ�ԭʼ���ƣ�
    pcl::visualization::PCLVisualizer viewer_original("Original Cloud Viewer");
    viewer_original.setBackgroundColor(0, 0, 0);
    viewer_original.addPointCloud(cloud, "original_cloud");

    // �����ڶ������ӻ����ڣ��ָ��ĵ��ƣ�
    pcl::visualization::PCLVisualizer viewer_segmented("Segmented Cloud Viewer");
    viewer_segmented.setBackgroundColor(0, 0, 0);

    for (int i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
        for (const auto& index : cluster_indices[i].indices) {
            cluster->points.push_back(cloud->points[index]);
        }

        // �ڿ��ӻ������и�ÿ�����ำ�費ͬ����ɫ
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(cluster, rand() % 256, rand() % 256, rand() % 256);
        viewer_segmented.addPointCloud(cluster, single_color, "cluster_" + std::to_string(i + 1));

        // ���������ƣ�����ԭɫ��
        pcl::io::savePCDFileBinary("E:\\Tomato_data\\seg_organs\\22_fruit_" + std::to_string(i + 1) + "_pc.pcd", *cluster);
    }

    // ������һ�����ӻ����ڣ�ԭʼ���ƣ�
    while (!viewer_original.wasStopped()) {
        viewer_original.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // �����ڶ������ӻ����ڣ��ָ��ĵ��ƣ�
    while (!viewer_segmented.wasStopped()) {
        viewer_segmented.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
