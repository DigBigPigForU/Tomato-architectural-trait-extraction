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
#include <pcl/common/common.h>   // 添加用于获取点云包围盒的头文件
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <vector>
#include <pcl/common/centroid.h>

using PointCloudT = pcl::PointCloud<pcl::PointXYZRGBA>;
using PointT = pcl::PointXYZRGBA;

// 计算两点之间的距离
float calculateDistance(const PointT& p1, const PointT& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

float calculateDistance_2d_xy(const PointT& p1, const PointT& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// 计算点云的质心
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

// 采样叶片点云并计算相邻点间距
void sampleLeavesPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, int numSamples, float sampleSize, std::ofstream& csvFile, int x) {
    // 获取叶片点云的包围盒
    pcl::PointXYZRGBA minPoint, maxPoint;
    pcl::getMinMax3D(cloud, minPoint, maxPoint);

    // 获取包围盒的中心
    pcl::PointXYZRGBA boxCenter((minPoint.x + maxPoint.x) / 2,
        (minPoint.y + maxPoint.y) / 2,
        (minPoint.z + maxPoint.z) / 2);

    // 获取包围盒的长宽高
    float boxWidth = maxPoint.x - minPoint.x;
    float boxLength = maxPoint.y - minPoint.y;
    float boxHeight = maxPoint.z - minPoint.z;

    // 生成随机数引擎
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> disX(-boxWidth / 2, boxWidth / 2);
    std::uniform_real_distribution<float> disY(-boxLength / 2, boxLength / 2);

    for (int sampleIndex = 0; sampleIndex < numSamples; ++sampleIndex) {
        csvFile << "1" << "," << x << ",";

        // 随机选取包围盒内的点作为采样中心
        float sampleCenterX = boxCenter.x + disX(gen);
        float sampleCenterY = boxCenter.y + disY(gen);

        // 定义采样区域
        pcl::PointCloud<pcl::PointXYZRGBA> sampledRegion;

        // 遍历点云，将在采样区域内的点添加到 sampledRegion 中
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

        // 定义用于存储聚类中心的容器
        std::vector<pcl::PointXYZRGBA> clusterCentroids;

        // 进行欧式聚类
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
        tree->setInputCloud(sampledRegion.makeShared());

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
        ec.setClusterTolerance(0.05); // 设置聚类的容差
        ec.setMinClusterSize(5);    // 设置最小聚类大小
        ec.setMaxClusterSize(25000);  // 设置最大聚类大小
        ec.setSearchMethod(tree);
        ec.setInputCloud(sampledRegion.makeShared());
        ec.extract(clusterIndices);

        // 处理每个聚类中心
        for (const auto& clusterIndex : clusterIndices) {
            pcl::PointCloud<pcl::PointXYZRGBA> cluster;

            for (const auto& idx : clusterIndex.indices) {
                cluster.push_back(sampledRegion.points[idx]);
            }

            // 计算聚类中心
            pcl::PointXYZRGBA clusterCentroid = computeCentroid(cluster);

            // 添加到容器
            clusterCentroids.push_back(clusterCentroid);
        }

        if (clusterIndices.size() > 1) {

            // 对聚类中心按竖直高度排序
            std::sort(clusterCentroids.begin(), clusterCentroids.end(),
                [](const auto& a, const auto& b) {
                    return a.z < b.z;
                });

            // 计算相邻点间距
            std::cout << "采样次数：" << sampleIndex + 1 << "\n";
            std::cout << "相邻点间距：\n";
            for (int i = 0; i + 1 < clusterCentroids.size(); ++i) {
                float distance = clusterCentroids[i + 1].z - clusterCentroids[i].z;

                std::cout << "点 " << i + 1 << " 和点 " << i + 2 << " 的距离为：" << distance << " 米。\n";

                // 将结果写入 CSV 文件
                csvFile << distance;

                // 如果不是最后一列，则加入逗号
                if (i + 2 < clusterCentroids.size()) {
                    csvFile << ",";
                }
            }

        }
        std::cout << "\n";

        // 换行
        csvFile << "\n";
    }
}

int main() {

    for (int frame_id = 1; frame_id <= 144; ++frame_id) {
        // 构建点云文件名
        std::string base_dir_name = "E:/Tomato_data/seg_organs/p6";

        std::string flower_file_name = base_dir_name + "/flowers/" + std::to_string(frame_id) + "_flower_pc.pcd";
        std::string fruit_file_name = base_dir_name +"/fruits/" + std::to_string(frame_id) + "_fruit_pc.pcd";
        std::string leaf_file_name = base_dir_name +"/leaves/" + std::to_string(frame_id) + "_leaf_pc.pcd";
        std::string stem_file_name = base_dir_name+"/stems/" + std::to_string(frame_id) + "_stem_pc.pcd";


        // 读取点云文件
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr flowers(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr fruits(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leaves(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr stems(new pcl::PointCloud<pcl::PointXYZRGBA>);


        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(stem_file_name, *stems) == -1) {
            continue;
        }
        else {
            // seg stem
            // 将点云投影至xy平面
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

            std::cout << "Number of stem clusters: " << cluster_indices.size() << std::endl;

            if (cluster_indices.size() < 1) {
                continue;
            }

            // 分割并保存结果，并可视化分割后的茎秆点云（每个聚类使用不同颜色）
            for (int i = 0; i < cluster_indices.size(); ++i) {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr stem_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

                for (int j = 0; j < cluster_indices[i].indices.size(); ++j) {
                    int index = cluster_indices[i].indices[j];
                    stem_cloud->points.push_back(stems->points[index]);
                }

                stem_cloud->width = stem_cloud->size();
                stem_cloud->height = 1;
                stem_cloud->is_dense = true;

                // 保存分割后的茎秆点云
                std::string stem_file = base_dir_name+"/"+ std::to_string(frame_id) +"_stem_" + std::to_string(i + 1) + "_pc.pcd";
                pcl::io::savePCDFile(stem_file, *stem_cloud);
            }
        }
        
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fruit_file_name, *fruits) == -1) {

        }
        else {
            // 创建欧氏聚类对象
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
            tree->setInputCloud(fruits);

            pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
            ec.setClusterTolerance(0.07); // 设置聚类的容差
            ec.setMinClusterSize(50);   // 设置最小聚类的点数
            ec.setMaxClusterSize(25000); // 设置最大聚类的点数
            ec.setSearchMethod(tree);
            ec.setInputCloud(fruits);

            std::vector<pcl::PointIndices> cluster_indices;
            ec.extract(cluster_indices);

            // 输出聚类数
            std::cout << "Number of fruit clusters: " << cluster_indices.size() << std::endl;

            for (int i = 0; i < cluster_indices.size(); ++i) {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
                for (const auto& index : cluster_indices[i].indices) {
                    cluster->points.push_back(fruits->points[index]);
                }

                // 保存聚类点云（保留原色）
                pcl::io::savePCDFileBinary(base_dir_name + "/" + std::to_string(frame_id)+"_fruit_" + std::to_string(i + 1) + "_pc.pcd", *cluster);
            }
        }

        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(flower_file_name, *flowers) == -1) {

        }
        else {
            // 创建欧氏聚类对象
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
            tree->setInputCloud(flowers);

            pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
            ec.setClusterTolerance(0.02); // 设置聚类的容差
            ec.setMinClusterSize(1);   // 设置最小聚类的点数
            ec.setMaxClusterSize(25000); // 设置最大聚类的点数
            ec.setSearchMethod(tree);
            ec.setInputCloud(flowers);

            std::vector<pcl::PointIndices> cluster_indices;
            ec.extract(cluster_indices);

            // 输出聚类数
            std::cout << "Number of flower clusters: " << cluster_indices.size() << std::endl;

            for (int i = 0; i < cluster_indices.size(); ++i) {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
                for (const auto& index : cluster_indices[i].indices) {
                    cluster->points.push_back(flowers->points[index]);
                }

                // 保存聚类点云（保留原色）
                pcl::io::savePCDFileBinary(base_dir_name + "/" + std::to_string(frame_id) + "_flower_" + std::to_string(i + 1) + "_pc.pcd", *cluster);
            }
        }

        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(leaf_file_name, *leaves) == -1) {

        }
        else {

        }


        ////////////for plants////////////////
        // 存储植株的点云
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> plants_vec;

        // 存储植株的点云
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> stems_vec;

        std::vector<int> fruit_idx_for_each_plant;

        std::vector<int> flower_idx_for_each_plant;

        // 循环读取茎秆点云数据
        for (int N = 1; ; N++) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            std::string single_stem_filename = base_dir_name + "/"+std::to_string(frame_id)+"_stem_" + std::to_string(N) + "_pc.pcd";
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(single_stem_filename, *cloud) == -1) {
                // 无法加载点云文件，可能已经读取完毕
                break;
            }

            // 存储植株的点云
            plants_vec.push_back(cloud);

            stems_vec.push_back(cloud);

            fruit_idx_for_each_plant.push_back(0);

            flower_idx_for_each_plant.push_back(0);

            // 输出当前加载的植株点云文件名
            std::cout << "Loaded plant: " << single_stem_filename << std::endl;
        }

        // 循环读取果实点云数据
        for (int M = 1; ; M++) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            std::string single_fruit_filename = base_dir_name + "/" + std::to_string(frame_id) + "_fruit_" + std::to_string(M) + "_pc.pcd";
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(single_fruit_filename, *cloud) == -1) {
                // 无法加载点云文件，可能已经读取完毕
                break;
            }

            // 输出当前加载的果实点云文件名
            std::cout << "Loaded fruit: " << single_fruit_filename << std::endl;

            // 寻找与当前果实最近的植株
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

            // 输出果实属于哪个植株
            std::cout << "Fruit belongs to plant: " << closest_plant_index + 1 << std::endl;

            fruit_idx_for_each_plant[closest_plant_index]++;
            std::string fruit_in_plant_filename = base_dir_name + "/" + std::to_string(frame_id) + "_plant_" + std::to_string(closest_plant_index + 1) + "_fruit_" + std::to_string(fruit_idx_for_each_plant[closest_plant_index]) + "_pc.pcd";
            pcl::io::savePCDFileBinary(fruit_in_plant_filename, *cloud);
            std::cout << "Saved plant: " << fruit_in_plant_filename << std::endl;

            // Concatenate the plant and fruit into a single point cloud
            *plants_vec[closest_plant_index] += *cloud;
        }

        // 循环读取花点云数据
        for (int M = 1; ; M++) {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            std::string single_flower_filename = base_dir_name + "/" + std::to_string(frame_id) + "_flower_" + std::to_string(M) + "_pc.pcd";
            if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(single_flower_filename, *cloud) == -1) {
                // 无法加载点云文件，可能已经读取完毕
                break;
            }

            // 输出当前加载的果实点云文件名
            std::cout << "Loaded flower: " << single_flower_filename << std::endl;

            // 寻找与当前果实最近的植株
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

            // 输出果实属于哪个植株
            std::cout << "Flower belongs to plant: " << closest_plant_index + 1 << std::endl;

            flower_idx_for_each_plant[closest_plant_index]++;
            std::string flower_in_plant_filename = base_dir_name + "/" + std::to_string(frame_id) + "_plant_" + std::to_string(closest_plant_index + 1) + "_flower_" + std::to_string(flower_idx_for_each_plant[closest_plant_index]) + "_pc.pcd";
            pcl::io::savePCDFileBinary(flower_in_plant_filename, *cloud);
            std::cout << "Saved plant: " << flower_in_plant_filename << std::endl;

            // Concatenate the plant and fruit into a single point cloud
            *plants_vec[closest_plant_index] += *cloud;
        }

        // leaves seg
        // 读取叶片点云数据
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leaf_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        std::string leaf_filename = base_dir_name + "/leaves/" + std::to_string(frame_id) + "_leaf_pc.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(leaf_filename, *leaf_cloud) != -1) {
            // 输出当前加载的叶片点云文件名
            std::cout << "Loaded leaf: " << leaf_filename << std::endl;

            // 下采样叶片点云
            pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
            sor.setInputCloud(leaf_cloud);
            //sor.setLeafSize(0.05f, 0.05f, 0.05f); // 设置体素大小
            sor.setLeafSize(0.025f, 0.025f, 0.025f); // 设置体素大小
            //sor.setLeafSize(0.01f, 0.01f, 0.01f); // 设置体素大小
            sor.filter(*leaf_cloud);

            // Map to store leaf indices and their corresponding plant indices
            std::map<int, int> leafPlantMap;

            // 循环处理叶片中的每个点
            for (size_t point_index = 0; point_index < leaf_cloud->size(); ++point_index) {
                pcl::PointXYZRGBA leaf_point = leaf_cloud->points[point_index];

                // 寻找与当前叶片点最近的植株
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

                // 将叶片点的索引与对应的植株索引保存到映射中
                leafPlantMap[point_index] = closest_plant_index;

                //// 在这里可以添加额外的输出
                //std::cout << "Leaf point " << point_index << " belongs to plant: " << closest_plant_index + 1 << std::endl;
            }

            // 根据归属信息合并同一植株的叶片点云
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> mergedLeafClouds(plants_vec.size());
            for (auto const& entry : leafPlantMap) {
                int point_index = entry.first;
                int plant_index = entry.second;

                if (mergedLeafClouds[plant_index] == nullptr) {
                    mergedLeafClouds[plant_index] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
                }

                mergedLeafClouds[plant_index]->push_back(leaf_cloud->points[point_index]);
            }

            // 保存合并后的叶片点云
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
            // 定义PCD文件的基本目录
        const std::string baseDir = base_dir_name;

        // 定义最大植株编号
        const int maxX = 10;  // 假设最大植株编号是10

        // 多层vector用于存储点云数据
        std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>>> pointClouds;

        // 遍历植株
        for (int x = 0; x <= maxX; ++x) {
            std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>> plantPointClouds;

            // 遍历器官（叶子 花 果实）
            for (int y = 0; y <= 2; ++y) {
                std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> organPointClouds;

                // 遍历果实
                for (int z = 0; z <= maxX; ++z) {
                    // 构建文件名，基于植株、器官和果实编号
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
                        // 跳过不存在的文件名
                        continue;
                    }

                    // 读取PCD文件中的点云
                    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
                    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename, cloud) == -1) {
                        // 如果文件不存在，跳过
                        continue;
                    }

                    // 将点云存储到对应的位置
                    organPointClouds.push_back(cloud);

                    // 输出信息显示点云的存储位置
                    std::cout << "植株 " << x + 1 << "，器官 " << std::to_string(y) <<( ", 编号，" + std::to_string(z + 1))
                        << " 的点云已存储在 pointClouds[" << x << "][" << y << "][" << z << "].\n";
                }

                // 将器官点云存储到当前植株
                plantPointClouds.push_back(organPointClouds);
            }

            // 将植株点云存储
            pointClouds.push_back(plantPointClouds);
        }

        // 至此，pointClouds 包含了所有点云数据
        // 可根据需要进行进一步处理
        std::cout << " pointClouds.size:" << pointClouds.size() << endl;


        // 打开CSV文件
        std::ofstream csvFile(baseDir + "/" + std::to_string(frame_id) + "_traits.csv");

        // 遍历植株计算果实间距
        for (int x = 0; x < pointClouds.size(); ++x) {
            // 获取当前植株的果实点云
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>& fruitPointClouds = pointClouds[x][1];

            // 对果实点云按照竖直坐标排序
            std::sort(fruitPointClouds.begin(), fruitPointClouds.end(), [](const auto& a, const auto& b) {
                return computeCentroid(a).z < computeCentroid(b).z;
                });

            csvFile << "0" << "," << x + 1 << ",";

            // 输出当前植株的垂直距离到CSV文件
            for (int z = 0; z + 1 < fruitPointClouds.size(); ++z) {

                // 获取当前果实和下一个果实的质心
                pcl::PointXYZRGBA centroidCurrent = computeCentroid(fruitPointClouds[z]);
                pcl::PointXYZRGBA centroidNext = computeCentroid(fruitPointClouds[z + 1]);

                // 计算垂直距离
                float verticalDistance = centroidNext.z - centroidCurrent.z;

                // 输出垂直距离到控制台
                std::cout << "植株 " << x + 1 << "，果实 " << z + 1 << " 和果实 " << z + 2
                    << " 的垂直距离为：" << verticalDistance << " 米。\n";

                // 输出垂直距离到CSV文件

                csvFile << verticalDistance;

                // 如果不是最后一列，则加入逗号
                if (z + 2 < fruitPointClouds.size()) {
                    csvFile << ",";
                }
            }

            // 换行
            csvFile << "\n";
        }

        // 遍历植株计算花间距和第一花序高度
        for (int x = 0; x < pointClouds.size(); ++x) {
            // 获取当前植株的花簇点云
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>& flowerPointClouds = pointClouds[x][2];

            // 对花簇点云按照竖直坐标排序
            std::sort(flowerPointClouds.begin(), flowerPointClouds.end(), [](const auto& a, const auto& b) {
                return computeCentroid(a).z < computeCentroid(b).z;
                });

            if (flowerPointClouds.size() > 0) {
                std::cout << " 第一花序高度:" << computeCentroid(flowerPointClouds[0]).z << endl;
                csvFile << "4" << "," << x + 1 << "," << computeCentroid(flowerPointClouds[0]).z << "\n";
            }

            csvFile << "2" << "," << x + 1 << ",";

            // 输出当前植株的垂直距离到CSV文件
            for (int z = 0; z + 1 < flowerPointClouds.size(); ++z) {

                // 获取当前花簇和下一个花簇的质心
                pcl::PointXYZRGBA centroidCurrent = computeCentroid(flowerPointClouds[z]);
                pcl::PointXYZRGBA centroidNext = computeCentroid(flowerPointClouds[z + 1]);

                // 计算垂直距离
                float verticalDistance = centroidNext.z - centroidCurrent.z;

                // 输出垂直距离到控制台
                std::cout << "植株 " << x + 1 << "，花簇 " << z + 1 << " 和花簇 " << z + 2
                    << " 的垂直距离为：" << verticalDistance << " 米。\n";

                // 输出垂直距离到CSV文件

                csvFile << verticalDistance;

                // 如果不是最后一列，则加入逗号
                if (z + 2 < flowerPointClouds.size()) {
                    csvFile << ",";
                }
            }

            // 换行
            csvFile << "\n";
        }

        //遍历植株计算多条射线的叶间距
        // 遍历植株
        for (int x = 0; x < pointClouds.size(); ++x) {
            // 获取当前植株的叶片点云
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>& leavesPointClouds = pointClouds[x][0];

            // 遍历叶片点云
            for (int i = 0; i < leavesPointClouds.size(); ++i) {
                std::cout << "植株 " << x + 1 << "，叶片 " << i + 1 << "：\n";
                // 对叶片点云进行50次采样、排序和相邻点间距计算
                sampleLeavesPointCloud(leavesPointClouds[i], 2000, 0.06f, csvFile, x + 1);
            }
        }

        // 关闭CSV文件
        csvFile.close();

        /////////////stem thickness
        // 循环处理点云文件
        for (int file_index = 1; ; ++file_index) {
            // 构建点云文件名
            std::string file_name = base_dir_name+"/"+ std::to_string(frame_id) +"_stem_" + std::to_string(file_index) + "_pc.pcd";

            // 读取点云数据
            PointCloudT::Ptr original_cloud(new PointCloudT);
            if (pcl::io::loadPCDFile<PointT>(file_name, *original_cloud) == -1) {
                // 如果文件不存在，则终止循环
                std::cout << "Could not read file: " << file_name << ". Exiting loop." << std::endl;
                break;
            }

            // ... （之后的处理保持不变）
            // 执行 PCA
            pcl::PCA<PointT> pca;
            pca.setInputCloud(original_cloud);
            Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
            Eigen::Vector4f centroid = pca.getMean();

            // 获取第二个特征向量作为平面法向量
            Eigen::Vector3f plane_normal = eigen_vectors.col(1).head(3);

            // 投影点云到平面
            PointCloudT::Ptr projected_cloud(new PointCloudT);
            projected_cloud->resize(original_cloud->size());

            for (size_t i = 0; i < original_cloud->size(); ++i) {
                const auto& point = original_cloud->points[i];
                Eigen::Vector3f point_vector(point.x - centroid(0), point.y - centroid(1), point.z - centroid(2));
                float distance_along_normal = point_vector.dot(plane_normal);
                Eigen::Vector3f projected_point = point_vector - distance_along_normal * plane_normal;

                // 保存投影后的点云
                (*projected_cloud)[i].x = projected_point(0);
                (*projected_cloud)[i].y = projected_point(1);
                (*projected_cloud)[i].z = projected_point(2);
                (*projected_cloud)[i].rgba = point.rgba;  // 复制颜色信息
            }

            //// 构建投影点云文件名
            //std::string projected_file_name = "E:/Tomato_data/seg_organs/22_stem_" + std::to_string(file_index) + "_projected.pcd";

            //// 保存投影点云到文件
            //pcl::io::savePCDFileASCII(projected_file_name, *projected_cloud);

            // 获取 Z 方向上的最小值和最大值
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

            // 将 Z 方向等分为 N=50 个区间
            int N = 50;
            float interval = (max_z - min_z) / N;
            std::cout << "interval: " << interval << std::endl;

            // 容器存储50个点云片段
            std::vector<PointCloudT::Ptr> cloud_segments(N);

            // 容器存储检测结果，1表示不空，0为空
            std::vector<int> detection_results(N, 0);

            // 分割点云并检测是否为空
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

            // 输出检测结果
            for (int i = 0; i < N; ++i) {
                std::cout << "Segment " << i << ": " << (detection_results[i] ? "Not empty" : "Empty") << std::endl;
            }

            // 将结果写入 CSV 文件
            std::ofstream csvFile(baseDir + "/" + std::to_string(frame_id) + "_traits.csv", std::ios::app);  // 打开文件以追加方式写入

            csvFile << "3" << "," << file_index << ",";

            // ... （之后的写入结果保持不变）
                // 从头开始遍历点云片段
            for (int i = 1; i < N - 1; ++i) {
                if (detection_results[i] && detection_results[i - 1] && detection_results[i + 1]) {
                    // 计算前后两段点云质心的横坐标和纵坐标之差的绝对值
                    Eigen::Vector4f centroid_prev, centroid_next, centroid_now;
                    pcl::compute3DCentroid(*cloud_segments[i - 1], centroid_prev);
                    pcl::compute3DCentroid(*cloud_segments[i + 1], centroid_next);
                    float diff_x = std::abs(centroid_prev(0) - centroid_next(0));
                    float diff_y = std::abs(centroid_prev(1) - centroid_next(1));

                    std::cout << "Segment " << i << ": "
                        << "Diff X: " << diff_x << ", Diff Y: " << diff_y << std::endl;

                    // 计算该点云片段中点的最大距离
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

                    //计算茎粗
                    float stem_thickness = (max_distance_xy - 0.5 * sqrt(pow(diff_x, 2) + pow(diff_y, 2))) * 2 * interval / sqrt(pow(diff_x, 2) + pow(diff_y, 2) + 4 * pow(interval, 2));
                    std::cout << "Stem Thickness: " << stem_thickness << std::endl;

                    if (stem_thickness < 0)
                    {
                        continue;
                    }

                    // 将结果写入 CSV 文件
                    csvFile << stem_thickness;
                    if (i < N - 2)
                    {
                        csvFile << ",";
                    }
                }
            }

            // 换行
            csvFile << "\n";

            // 关闭文件
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
            // 使用字符串流分割每行的元素
            std::istringstream ss(line);
            std::vector<std::string> elements;
            std::string element;

            while (std::getline(ss, element, ',')) {
                elements.push_back(element);
            }

            // 如果元素数量大于 2，写入到输出文件
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
