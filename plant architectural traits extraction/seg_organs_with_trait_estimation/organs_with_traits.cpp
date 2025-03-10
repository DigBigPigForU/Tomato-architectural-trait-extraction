#pragma once
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/colors.h>
#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <random>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>   // ������ڻ�ȡ���ư�Χ�е�ͷ�ļ�
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <vector>
#include <pcl/common/centroid.h>

using PointCloudT = pcl::PointCloud<pcl::PointXYZRGBA>;
using PointT = pcl::PointXYZRGBA;

// ��������֮��ľ���
float calculateDistance(const PointT& p1, const PointT& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

float calculateDistance_2d_xy(const PointT& p1, const PointT& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// ������Ƶ�����
pcl::PointXYZRGBA computeCentroid(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud) {
    pcl::PointXYZRGBA centroid;
    centroid.x = centroid.y = centroid.z = 0;

    for (const auto& point : cloud.points) {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }

    centroid.x /= cloud.size();
    centroid.y /= cloud.size();
    centroid.z /= cloud.size();

    return centroid;
}

// ����ҶƬ���Ʋ��������ڵ���
void sampleLeavesPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, int numSamples, float sampleSize, std::ofstream& csvFile, int x) {
    // ��ȡҶƬ���Ƶİ�Χ��
    pcl::PointXYZRGBA minPoint, maxPoint;
    pcl::getMinMax3D(cloud, minPoint, maxPoint);

    // ��ȡ��Χ�е�����
    pcl::PointXYZRGBA boxCenter((minPoint.x + maxPoint.x) / 2,
        (minPoint.y + maxPoint.y) / 2,
        (minPoint.z + maxPoint.z) / 2);

    // ��ȡ��Χ�еĳ����
    float boxWidth = maxPoint.x - minPoint.x;
    float boxLength = maxPoint.y - minPoint.y;
    float boxHeight = maxPoint.z - minPoint.z;

    // �������������
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> disX(-boxWidth / 2, boxWidth / 2);
    std::uniform_real_distribution<float> disY(-boxLength / 2, boxLength / 2);

    for (int sampleIndex = 0; sampleIndex < numSamples; ++sampleIndex) {
        csvFile << "1" << "," << x << ",";

        // ���ѡȡ��Χ���ڵĵ���Ϊ��������
        float sampleCenterX = boxCenter.x + disX(gen);
        float sampleCenterY = boxCenter.y + disY(gen);

        // �����������
        pcl::PointCloud<pcl::PointXYZRGBA> sampledRegion;

        // �������ƣ����ڲ��������ڵĵ���ӵ� sampledRegion ��
        for (const auto& point : cloud.points) {
            if (point.x >= sampleCenterX - sampleSize / 2 &&
                point.x <= sampleCenterX + sampleSize / 2 &&
                point.y >= sampleCenterY - sampleSize / 2 &&
                point.y <= sampleCenterY + sampleSize / 2 &&
                point.z >= minPoint.z &&
                point.z <= maxPoint.z) {
                sampledRegion.push_back(point);
            }
        }

        // �������ڴ洢�������ĵ�����
        std::vector<pcl::PointXYZRGBA> clusterCentroids;

        // ����ŷʽ����
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud(sampledRegion.makeShared());

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
        ec.setClusterTolerance(0.05); // ���þ�����ݲ�
        ec.setMinClusterSize(5);    // ������С�����С
        ec.setMaxClusterSize(25000);  // �����������С
        ec.setSearchMethod(tree);
        ec.setInputCloud(sampledRegion.makeShared());
        ec.extract(clusterIndices);

        // ����ÿ����������
        for (const auto& clusterIndex : clusterIndices) {
            pcl::PointCloud<pcl::PointXYZRGBA> cluster;

            for (const auto& idx : clusterIndex.indices) {
                cluster.push_back(sampledRegion.points[idx]);
            }

            // �����������
            pcl::PointXYZRGBA clusterCentroid = computeCentroid(cluster);

            // ��ӵ�����
            clusterCentroids.push_back(clusterCentroid);
        }

        if (clusterIndices.size() > 1) {

            // �Ծ������İ���ֱ�߶�����
            std::sort(clusterCentroids.begin(), clusterCentroids.end(),
                [](const auto& a, const auto& b) {
                    return a.z < b.z;
                });

            // �������ڵ���
            std::cout << "����������" << sampleIndex + 1 << "\n";
            std::cout << "���ڵ��ࣺ\n";
            for (int i = 0; i + 1 < clusterCentroids.size(); ++i) {
                float distance = clusterCentroids[i + 1].z - clusterCentroids[i].z;

                std::cout << "�� " << i + 1 << " �͵� " << i + 2 << " �ľ���Ϊ��" << distance << " �ס�\n";

                // �����д�� CSV �ļ�
                csvFile << distance;

                // ����������һ�У�����붺��
                if (i + 2 < clusterCentroids.size()) {
                    csvFile << ",";
                }
            }

        }
        std::cout << "\n";

        // ����
        csvFile << "\n";
    }
}

int main() {

    for (int frame_id = 1; frame_id <= 144; ++frame_id) {
        // ���������ļ���
        std::string base_dir_name = "E:/Tomato_data/seg_organs/p6";

        std::string flower_file_name = base_dir_name + "/flowers/" + std::to_string(frame_id) + "_flower_pc.pcd";
        std::string fruit_file_name = base_dir_name +"/fruits/" + std::to_string(frame_id) + "_fruit_pc.pcd";
        std::string leaf_file_name = base_dir_name +"/leaves/" + std::to_string(frame_id) + "_leaf_pc.pcd";
        std::string stem_file_name = base_dir_name+"/stems/" + std::to_string(frame_id) + "_stem_pc.pcd";


        // ��ȡ�����ļ�
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr flowers(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr fruits(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leaves(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr stems(new pcl::PointCloud<pcl::PointXYZRGBA>);


        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(stem_file_name, *stems) == -1) {
            continue;
        }
        else {
            // seg stem
            // ������ͶӰ��xyƽ��
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            coefficients->values.resize(4);
            coefficients->values[0] = coefficients->values[1] = 0;
            coefficients->values[2] = 1.0;
            coefficients->values[3] = 0.0;

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(stems);
            proj.setModelCoefficients(coefficients);
            proj.filter(*cloud_projected);

            // ŷʽ����
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
            tree->setInputCloud(cloud_projected);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
            ec.setClusterTolerance(0.02); // ���þ����ŷ�Ͼ�����ֵ
            ec.setMinClusterSize(100);   // ����ÿ���������С����
            ec.setMaxClusterSize(25000); // ����ÿ�������������
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud_projected);
            ec.extract(cluster_indices);

            std::cout << "Number of stem clusters: " << cluster_indices.size() << std::endl;

            if (cluster_indices.size() < 1) {
                continue;
            }

            // �ָ�������������ӻ��ָ��ľ��ѵ��ƣ�ÿ������ʹ�ò�ͬ��ɫ��
            for (int i = 0; i < cluster_indices.size(); ++i) {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr stem_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

                for (int j = 0; j < cluster_indices[i].indices.size(); ++j) {
                    int index = cluster_indices[i].indices[j];
                    stem_cloud->points.push_back(stems->points[index]);
                }

                stem_cloud->width = stem_cloud->size();
                stem_cloud->height = 1;
                stem_cloud->is_dense = true;

                // ����ָ��ľ��ѵ���
                std::string stem_file = base_dir_name+"/"+ std::to_string(frame_id) +"_stem_" + std::to_string(i + 1) + "_pc.pcd";
                pcl::io::savePCDFile(stem_file, *stem_cloud);
            }
        }
        
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fruit_file_name, *fruits) == -1) {

        }
        else {
            // ����ŷ�Ͼ������
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
            tree->setInputCloud(fruits);

            pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
            ec.setClusterTolerance(0.07); // ���þ�����ݲ�
            ec.setMinClusterSize(50);   // ������С����ĵ���
            ec.setMaxClusterSize(25000); // ����������ĵ���
            ec.setSearchMethod(tree);
            ec.setInputCloud(fruits);

            std::vector<pcl::PointIndices> cluster_indices;
            ec.extract(cluster_indices);

            // ���������
            std::cout << "Number of fruit clusters: " << cluster_indices.size() << std::endl;

            for (int i = 0; i < cluster_indices.size(); ++i) {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
                for (const auto& index : cluster_indices[i].indices) {
                    cluster->points.push_back(fruits->points[index]);
                }

                // ���������ƣ�����ԭɫ��
                pcl::io::savePCDFileBinary(base_dir_name + "/" + std::to_string(frame_id)+"_fruit_" + std::to_string(i + 1) + "_pc.pcd", *cluster);
            }
        }

        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(flower_file_name, *flowers) == -1) {

        }
        else {
            // ����ŷ�Ͼ������
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
            tree->setInputCloud(flowers);

            pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
            ec.setClusterTolerance(0.02); // ���þ�����ݲ�
            ec.setMinClusterSize(1);   // ������С����ĵ���
            ec.setMaxClusterSize(25000); // ����������ĵ���
            ec.setSearchMethod(tree);
            ec.setInputCloud(flowers);

            std::vector<pcl::PointIndices> cluster_indices;
            ec.extract(cluster_indices);

            // ���������
            std::cout << "Number of flower clusters: " << cluster_indices.size() << std::endl;

            for (int i = 0; i < cluster_indices.size(); ++i) {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
                for (const auto& index : cluster_indices[i].indices) {
                    cluster->points.push_back(flowers->points[index]);
                }

                // ���������ƣ�����ԭɫ��
                pcl::io::savePCDFileBinary(base_dir_name + "/" + std::to_string(frame_id) + "_flower_" + std::to_string(i + 1) + "_pc.pcd", *cluster);
            }
        }

        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(leaf_file_name, *leaves) == -1) {

        }
        else {

        }


        ////////////for plants////////////////
        // �洢ֲ��ĵ���
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> plants_vec;

        // �洢ֲ��ĵ���
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> stems_vec;

        std::vector<int> fruit_idx_for_each_plant;

        std::vector<int> flower_idx_for_each_plant;

        // ѭ����ȡ���ѵ�������
        for (int N = 1; ; N++) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            std::string single_stem_filename = base_dir_name + "/"+std::to_string(frame_id)+"_stem_" + std::to_string(N) + "_pc.pcd";
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(single_stem_filename, *cloud) == -1) {
                // �޷����ص����ļ��������Ѿ���ȡ���
                break;
            }

            // �洢ֲ��ĵ���
            plants_vec.push_back(cloud);

            stems_vec.push_back(cloud);

            fruit_idx_for_each_plant.push_back(0);

            flower_idx_for_each_plant.push_back(0);

            // �����ǰ���ص�ֲ������ļ���
            std::cout << "Loaded plant: " << single_stem_filename << std::endl;
        }

        // ѭ����ȡ��ʵ��������
        for (int M = 1; ; M++) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            std::string single_fruit_filename = base_dir_name + "/" + std::to_string(frame_id) + "_fruit_" + std::to_string(M) + "_pc.pcd";
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(single_fruit_filename, *cloud) == -1) {
                // �޷����ص����ļ��������Ѿ���ȡ���
                break;
            }

            // �����ǰ���صĹ�ʵ�����ļ���
            std::cout << "Loaded fruit: " << single_fruit_filename << std::endl;

            // Ѱ���뵱ǰ��ʵ�����ֲ��
            pcl::PointXYZRGBA fruit_centroid;
            pcl::computeCentroid(*cloud, fruit_centroid);

            double min_distance = std::numeric_limits<double>::max();
            int closest_plant_index = -1;

            for (size_t i = 0; i < stems_vec.size(); ++i) {
                pcl::PointXYZRGBA stem_centroid;
                pcl::computeCentroid(*(stems_vec[i]), stem_centroid);

                double distance = std::sqrt((fruit_centroid.x - stem_centroid.x) * (fruit_centroid.x - stem_centroid.x) +
                    (fruit_centroid.y - stem_centroid.y) * (fruit_centroid.y - stem_centroid.y));

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_plant_index = i;
                }
            }

            // �����ʵ�����ĸ�ֲ��
            std::cout << "Fruit belongs to plant: " << closest_plant_index + 1 << std::endl;

            fruit_idx_for_each_plant[closest_plant_index]++;
            std::string fruit_in_plant_filename = base_dir_name + "/" + std::to_string(frame_id) + "_plant_" + std::to_string(closest_plant_index + 1) + "_fruit_" + std::to_string(fruit_idx_for_each_plant[closest_plant_index]) + "_pc.pcd";
            pcl::io::savePCDFileBinary(fruit_in_plant_filename, *cloud);
            std::cout << "Saved plant: " << fruit_in_plant_filename << std::endl;

            // Concatenate the plant and fruit into a single point cloud
            *plants_vec[closest_plant_index] += *cloud;
        }

        // ѭ����ȡ����������
        for (int M = 1; ; M++) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            std::string single_flower_filename = base_dir_name + "/" + std::to_string(frame_id) + "_flower_" + std::to_string(M) + "_pc.pcd";
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(single_flower_filename, *cloud) == -1) {
                // �޷����ص����ļ��������Ѿ���ȡ���
                break;
            }

            // �����ǰ���صĹ�ʵ�����ļ���
            std::cout << "Loaded flower: " << single_flower_filename << std::endl;

            // Ѱ���뵱ǰ��ʵ�����ֲ��
            pcl::PointXYZRGBA flower_centroid;
            pcl::computeCentroid(*cloud, flower_centroid);

            double min_distance = std::numeric_limits<double>::max();
            int closest_plant_index = -1;

            for (size_t i = 0; i < stems_vec.size(); ++i) {
                pcl::PointXYZRGBA stem_centroid;
                pcl::computeCentroid(*(stems_vec[i]), stem_centroid);

                double distance = std::sqrt((flower_centroid.x - stem_centroid.x) * (flower_centroid.x - stem_centroid.x) +
                    (flower_centroid.y - stem_centroid.y) * (flower_centroid.y - stem_centroid.y));

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_plant_index = i;
                }
            }

            // �����ʵ�����ĸ�ֲ��
            std::cout << "Flower belongs to plant: " << closest_plant_index + 1 << std::endl;

            flower_idx_for_each_plant[closest_plant_index]++;
            std::string flower_in_plant_filename = base_dir_name + "/" + std::to_string(frame_id) + "_plant_" + std::to_string(closest_plant_index + 1) + "_flower_" + std::to_string(flower_idx_for_each_plant[closest_plant_index]) + "_pc.pcd";
            pcl::io::savePCDFileBinary(flower_in_plant_filename, *cloud);
            std::cout << "Saved plant: " << flower_in_plant_filename << std::endl;

            // Concatenate the plant and fruit into a single point cloud
            *plants_vec[closest_plant_index] += *cloud;
        }

        // leaves seg
        // ��ȡҶƬ��������
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leaf_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        std::string leaf_filename = base_dir_name + "/leaves/" + std::to_string(frame_id) + "_leaf_pc.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(leaf_filename, *leaf_cloud) != -1) {
            // �����ǰ���ص�ҶƬ�����ļ���
            std::cout << "Loaded leaf: " << leaf_filename << std::endl;

            // �²���ҶƬ����
            pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
            sor.setInputCloud(leaf_cloud);
            //sor.setLeafSize(0.05f, 0.05f, 0.05f); // �������ش�С
            sor.setLeafSize(0.025f, 0.025f, 0.025f); // �������ش�С
            //sor.setLeafSize(0.01f, 0.01f, 0.01f); // �������ش�С
            sor.filter(*leaf_cloud);

            // Map to store leaf indices and their corresponding plant indices
            std::map<int, int> leafPlantMap;

            // ѭ������ҶƬ�е�ÿ����
            for (size_t point_index = 0; point_index < leaf_cloud->size(); ++point_index) {
                pcl::PointXYZRGBA leaf_point = leaf_cloud->points[point_index];

                // Ѱ���뵱ǰҶƬ�������ֲ��
                double min_distance = std::numeric_limits<double>::max();
                int closest_plant_index = -1;

                for (size_t i = 0; i < stems_vec.size(); ++i) {
                    pcl::PointXYZRGBA stem_centroid;
                    pcl::computeCentroid(*(stems_vec[i]), stem_centroid);

                    double distance = std::sqrt((leaf_point.x - stem_centroid.x) * (leaf_point.x - stem_centroid.x) +
                        (leaf_point.y - stem_centroid.y) * (leaf_point.y - stem_centroid.y));

                    if (distance < min_distance) {
                        min_distance = distance;
                        closest_plant_index = i;
                    }
                }

                // ��ҶƬ����������Ӧ��ֲ���������浽ӳ����
                leafPlantMap[point_index] = closest_plant_index;

                //// �����������Ӷ�������
                //std::cout << "Leaf point " << point_index << " belongs to plant: " << closest_plant_index + 1 << std::endl;
            }

            // ���ݹ�����Ϣ�ϲ�ͬһֲ���ҶƬ����
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> mergedLeafClouds(plants_vec.size());
            for (auto const& entry : leafPlantMap) {
                int point_index = entry.first;
                int plant_index = entry.second;

                if (mergedLeafClouds[plant_index] == nullptr) {
                    mergedLeafClouds[plant_index] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
                }

                mergedLeafClouds[plant_index]->push_back(leaf_cloud->points[point_index]);
            }

            // ����ϲ����ҶƬ����
            for (size_t i = 0; i < mergedLeafClouds.size(); ++i) {
                if (mergedLeafClouds[i] != nullptr) {
                    std::string merged_leaf_filename = base_dir_name + "/" + std::to_string(frame_id) + "_plant_" + std::to_string(i + 1) + "_leaves_pc.pcd";
                    pcl::io::savePCDFileBinary(merged_leaf_filename, *mergedLeafClouds[i]);
                    std::cout << "Saved merged leaves: " << merged_leaf_filename << std::endl;
                    *plants_vec[i] += *mergedLeafClouds[i];
                }
            }
        }

        // Save the concatenated point clouds with original colors
        for (size_t i = 0; i < plants_vec.size(); ++i) {
            std::string output_filename = base_dir_name + "/" + std::to_string(frame_id) + "_plant_" + std::to_string(i + 1) + "_pc.pcd";
            pcl::io::savePCDFileBinary(output_filename, *plants_vec[i]);
            std::cout << "Saved plant: " << output_filename << std::endl;
        }



        ////////////traits estimation
        // 
        // 
            // ����PCD�ļ��Ļ���Ŀ¼
        const std::string baseDir = base_dir_name;

        // �������ֲ����
        const int maxX = 10;  // �������ֲ������10

        // ���vector���ڴ洢��������
        std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>>> pointClouds;

        // ����ֲ��
        for (int x = 0; x <= maxX; ++x) {
            std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>> plantPointClouds;

            // �������٣�Ҷ�� �� ��ʵ��
            for (int y = 0; y <= 2; ++y) {
                std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> organPointClouds;

                // ������ʵ
                for (int z = 0; z <= maxX; ++z) {
                    // �����ļ���������ֲ�ꡢ���ٺ͹�ʵ���
                    std::string filename = baseDir + "/" + std::to_string(frame_id) + "_plant_" + std::to_string(x + 1);

                    if (y == 0 && z == 0) {
                        filename += "_leaves_pc.pcd";
                    }
                    else if (y == 1) {
                        filename += "_fruit_" + std::to_string(z + 1) + "_pc.pcd";
                    }
                    else if (y == 2) {
                        filename += "_flower_" + std::to_string(z + 1) + "_pc.pcd";
                    }
                    else {
                        // ���������ڵ��ļ���
                        continue;
                    }

                    // ��ȡPCD�ļ��еĵ���
                    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
                    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, cloud) == -1) {
                        // ����ļ������ڣ�����
                        continue;
                    }

                    // �����ƴ洢����Ӧ��λ��
                    organPointClouds.push_back(cloud);

                    // �����Ϣ��ʾ���ƵĴ洢λ��
                    std::cout << "ֲ�� " << x + 1 << "������ " << std::to_string(y) <<( ", ��ţ�" + std::to_string(z + 1))
                        << " �ĵ����Ѵ洢�� pointClouds[" << x << "][" << y << "][" << z << "].\n";
                }

                // �����ٵ��ƴ洢����ǰֲ��
                plantPointClouds.push_back(organPointClouds);
            }

            // ��ֲ����ƴ洢
            pointClouds.push_back(plantPointClouds);
        }

        // ���ˣ�pointClouds ���������е�������
        // �ɸ�����Ҫ���н�һ������
        std::cout << " pointClouds.size:" << pointClouds.size() << endl;


        // ��CSV�ļ�
        std::ofstream csvFile(baseDir + "/" + std::to_string(frame_id) + "_traits.csv");

        // ����ֲ������ʵ���
        for (int x = 0; x < pointClouds.size(); ++x) {
            // ��ȡ��ǰֲ��Ĺ�ʵ����
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>& fruitPointClouds = pointClouds[x][1];

            // �Թ�ʵ���ư�����ֱ��������
            std::sort(fruitPointClouds.begin(), fruitPointClouds.end(), [](const auto& a, const auto& b) {
                return computeCentroid(a).z < computeCentroid(b).z;
                });

            csvFile << "0" << "," << x + 1 << ",";

            // �����ǰֲ��Ĵ�ֱ���뵽CSV�ļ�
            for (int z = 0; z + 1 < fruitPointClouds.size(); ++z) {

                // ��ȡ��ǰ��ʵ����һ����ʵ������
                pcl::PointXYZRGBA centroidCurrent = computeCentroid(fruitPointClouds[z]);
                pcl::PointXYZRGBA centroidNext = computeCentroid(fruitPointClouds[z + 1]);

                // ���㴹ֱ����
                float verticalDistance = centroidNext.z - centroidCurrent.z;

                // �����ֱ���뵽����̨
                std::cout << "ֲ�� " << x + 1 << "����ʵ " << z + 1 << " �͹�ʵ " << z + 2
                    << " �Ĵ�ֱ����Ϊ��" << verticalDistance << " �ס�\n";

                // �����ֱ���뵽CSV�ļ�

                csvFile << verticalDistance;

                // ����������һ�У�����붺��
                if (z + 2 < fruitPointClouds.size()) {
                    csvFile << ",";
                }
            }

            // ����
            csvFile << "\n";
        }

        // ����ֲ����㻨���͵�һ����߶�
        for (int x = 0; x < pointClouds.size(); ++x) {
            // ��ȡ��ǰֲ��Ļ��ص���
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>& flowerPointClouds = pointClouds[x][2];

            // �Ի��ص��ư�����ֱ��������
            std::sort(flowerPointClouds.begin(), flowerPointClouds.end(), [](const auto& a, const auto& b) {
                return computeCentroid(a).z < computeCentroid(b).z;
                });

            if (flowerPointClouds.size() > 0) {
                std::cout << " ��һ����߶�:" << computeCentroid(flowerPointClouds[0]).z << endl;
                csvFile << "4" << "," << x + 1 << "," << computeCentroid(flowerPointClouds[0]).z << "\n";
            }

            csvFile << "2" << "," << x + 1 << ",";

            // �����ǰֲ��Ĵ�ֱ���뵽CSV�ļ�
            for (int z = 0; z + 1 < flowerPointClouds.size(); ++z) {

                // ��ȡ��ǰ���غ���һ�����ص�����
                pcl::PointXYZRGBA centroidCurrent = computeCentroid(flowerPointClouds[z]);
                pcl::PointXYZRGBA centroidNext = computeCentroid(flowerPointClouds[z + 1]);

                // ���㴹ֱ����
                float verticalDistance = centroidNext.z - centroidCurrent.z;

                // �����ֱ���뵽����̨
                std::cout << "ֲ�� " << x + 1 << "������ " << z + 1 << " �ͻ��� " << z + 2
                    << " �Ĵ�ֱ����Ϊ��" << verticalDistance << " �ס�\n";

                // �����ֱ���뵽CSV�ļ�

                csvFile << verticalDistance;

                // ����������һ�У�����붺��
                if (z + 2 < flowerPointClouds.size()) {
                    csvFile << ",";
                }
            }

            // ����
            csvFile << "\n";
        }

        //����ֲ�����������ߵ�Ҷ���
        // ����ֲ��
        for (int x = 0; x < pointClouds.size(); ++x) {
            // ��ȡ��ǰֲ���ҶƬ����
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>& leavesPointClouds = pointClouds[x][0];

            // ����ҶƬ����
            for (int i = 0; i < leavesPointClouds.size(); ++i) {
                std::cout << "ֲ�� " << x + 1 << "��ҶƬ " << i + 1 << "��\n";
                // ��ҶƬ���ƽ���50�β�������������ڵ������
                sampleLeavesPointCloud(leavesPointClouds[i], 2000, 0.06f, csvFile, x + 1);
            }
        }

        // �ر�CSV�ļ�
        csvFile.close();

        /////////////stem thickness
        // ѭ����������ļ�
        for (int file_index = 1; ; ++file_index) {
            // ���������ļ���
            std::string file_name = base_dir_name+"/"+ std::to_string(frame_id) +"_stem_" + std::to_string(file_index) + "_pc.pcd";

            // ��ȡ��������
            PointCloudT::Ptr original_cloud(new PointCloudT);
            if (pcl::io::loadPCDFile<PointT>(file_name, *original_cloud) == -1) {
                // ����ļ������ڣ�����ֹѭ��
                std::cout << "Could not read file: " << file_name << ". Exiting loop." << std::endl;
                break;
            }

            // ... ��֮��Ĵ����ֲ��䣩
            // ִ�� PCA
            pcl::PCA<PointT> pca;
            pca.setInputCloud(original_cloud);
            Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
            Eigen::Vector4f centroid = pca.getMean();

            // ��ȡ�ڶ�������������Ϊƽ�淨����
            Eigen::Vector3f plane_normal = eigen_vectors.col(1).head(3);

            // ͶӰ���Ƶ�ƽ��
            PointCloudT::Ptr projected_cloud(new PointCloudT);
            projected_cloud->resize(original_cloud->size());

            for (size_t i = 0; i < original_cloud->size(); ++i) {
                const auto& point = original_cloud->points[i];
                Eigen::Vector3f point_vector(point.x - centroid(0), point.y - centroid(1), point.z - centroid(2));
                float distance_along_normal = point_vector.dot(plane_normal);
                Eigen::Vector3f projected_point = point_vector - distance_along_normal * plane_normal;

                // ����ͶӰ��ĵ���
                (*projected_cloud)[i].x = projected_point(0);
                (*projected_cloud)[i].y = projected_point(1);
                (*projected_cloud)[i].z = projected_point(2);
                (*projected_cloud)[i].rgba = point.rgba;  // ������ɫ��Ϣ
            }

            //// ����ͶӰ�����ļ���
            //std::string projected_file_name = "E:/Tomato_data/seg_organs/22_stem_" + std::to_string(file_index) + "_projected.pcd";

            //// ����ͶӰ���Ƶ��ļ�
            //pcl::io::savePCDFileASCII(projected_file_name, *projected_cloud);

            // ��ȡ Z �����ϵ���Сֵ�����ֵ
            float min_z = std::numeric_limits<float>::max();
            float max_z = std::numeric_limits<float>::min();

            for (const auto& point : projected_cloud->points) {
                if (point.z < min_z) {
                    min_z = point.z;
                }
                if (point.z > max_z) {
                    max_z = point.z;
                }
            }

            // �� Z ����ȷ�Ϊ N=50 ������
            int N = 50;
            float interval = (max_z - min_z) / N;
            std::cout << "interval: " << interval << std::endl;

            // �����洢50������Ƭ��
            std::vector<PointCloudT::Ptr> cloud_segments(N);

            // �����洢�������1��ʾ���գ�0Ϊ��
            std::vector<int> detection_results(N, 0);

            // �ָ���Ʋ�����Ƿ�Ϊ��
            for (int i = 0; i < N; ++i) {
                PointCloudT::Ptr segment(new PointCloudT);
                for (const auto& point : projected_cloud->points) {
                    if (point.z >= min_z + i * interval && point.z < min_z + (i + 1) * interval) {
                        segment->points.push_back(point);
                    }
                }
                cloud_segments[i] = segment;
                detection_results[i] = (segment->size() > 0) ? 1 : 0;
            }

            // ��������
            for (int i = 0; i < N; ++i) {
                std::cout << "Segment " << i << ": " << (detection_results[i] ? "Not empty" : "Empty") << std::endl;
            }

            // �����д�� CSV �ļ�
            std::ofstream csvFile(baseDir + "/" + std::to_string(frame_id) + "_traits.csv", std::ios::app);  // ���ļ���׷�ӷ�ʽд��

            csvFile << "3" << "," << file_index << ",";

            // ... ��֮���д�������ֲ��䣩
                // ��ͷ��ʼ��������Ƭ��
            for (int i = 1; i < N - 1; ++i) {
                if (detection_results[i] && detection_results[i - 1] && detection_results[i + 1]) {
                    // ����ǰ�����ε������ĵĺ������������֮��ľ���ֵ
                    Eigen::Vector4f centroid_prev, centroid_next, centroid_now;
                    pcl::compute3DCentroid(*cloud_segments[i - 1], centroid_prev);
                    pcl::compute3DCentroid(*cloud_segments[i + 1], centroid_next);
                    float diff_x = std::abs(centroid_prev(0) - centroid_next(0));
                    float diff_y = std::abs(centroid_prev(1) - centroid_next(1));

                    std::cout << "Segment " << i << ": "
                        << "Diff X: " << diff_x << ", Diff Y: " << diff_y << std::endl;

                    // ����õ���Ƭ���е��������
                    float max_distance_x = 0.0;
                    float max_distance_y = 0.0;
                    float max_distance_xy = 0.0;
                    for (size_t j = 0; j < cloud_segments[i]->size(); ++j) {
                        for (size_t k = 0; k < cloud_segments[i]->size(); ++k) {
                            float distance_xy = calculateDistance_2d_xy(cloud_segments[i]->points[j], cloud_segments[i]->points[k]);
                            if (distance_xy > max_distance_xy) {
                                max_distance_xy = distance_xy;
                            }
                        }
                    }

                    std::cout << "Max Distance xy in Segment: " << max_distance_xy << std::endl;

                    //���㾥��
                    float stem_thickness = (max_distance_xy - 0.5 * sqrt(pow(diff_x, 2) + pow(diff_y, 2))) * 2 * interval / sqrt(pow(diff_x, 2) + pow(diff_y, 2) + 4 * pow(interval, 2));
                    std::cout << "Stem Thickness: " << stem_thickness << std::endl;

                    if (stem_thickness < 0)
                    {
                        continue;
                    }

                    // �����д�� CSV �ļ�
                    csvFile << stem_thickness;
                    if (i < N - 2)
                    {
                        csvFile << ",";
                    }
                }
            }

            // ����
            csvFile << "\n";

            // �ر��ļ�
            csvFile.close();
        }

        //delete the empty line
        const std::string inputFileName = baseDir + "/" + std::to_string(frame_id) + "_traits.csv";
        const std::string outputFileName = baseDir + "/" + std::to_string(frame_id) + "_filtered_traits_no_empty.csv";

        std::ifstream inputFile(inputFileName);
        std::ofstream outputFile(outputFileName);

        if (!inputFile.is_open() || !outputFile.is_open()) {
            std::cerr << "Error opening files." << std::endl;
            return 1;
        }

        std::string line;
        while (std::getline(inputFile, line)) {
            // ʹ���ַ������ָ�ÿ�е�Ԫ��
            std::istringstream ss(line);
            std::vector<std::string> elements;
            std::string element;

            while (std::getline(ss, element, ',')) {
                elements.push_back(element);
            }

            // ���Ԫ���������� 2��д�뵽����ļ�
            if (elements.size() > 2) {
                outputFile << line << "\n";
            }
        }

        inputFile.close();
        outputFile.close();

        std::cout << "Filtered CSV file created successfully." << std::endl;
    }
    return 0;
}
