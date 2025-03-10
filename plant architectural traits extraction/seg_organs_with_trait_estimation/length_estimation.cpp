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
void sampleLeavesPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, int numSamples, float sampleSize, std::ofstream& csvFile,int x) {
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

        //// 按竖直高度进行排序
        //std::sort(sampledRegion.points.begin(), sampledRegion.points.end(),
        //    [](const auto& a, const auto& b) {
        //        return a.z < b.z;
        //    });

        //// 计算相邻点间距
        //std::cout << "采样次数：" << sampleIndex + 1 << "\n";
        //std::cout << "相邻点间距：\n";
        //for (int i = 0; i + 1 < sampledRegion.size(); ++i) {
        //    float distance =sampledRegion[i + 1].z - sampledRegion[i].z;

        //    if (distance < 0.06)
        //        continue;

        //    std::cout << "点 " << i + 1 << " 和点 " << i + 2 << " 的距离为：" << distance << " 米。\n";

        //    // 将结果写入 CSV 文件
        //    csvFile << distance;

        //    // 如果不是最后一列，则加入逗号
        //    if (i + 2 < sampledRegion.size()) {
        //        csvFile << ",";
        //    }
        //}
        //std::cout << "\n";

        //// 换行
        //csvFile << "\n";

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
    // 定义PCD文件的基本目录
    const std::string baseDir = "E:/Tomato_data/seg_organs/";

    // 定义最大植株编号
    const int maxX = 10;  // 假设最大植株编号是10

    // 多层vector用于存储点云数据
    std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>>> pointClouds;

    // 遍历植株
    for (int x = 0; x <= maxX; ++x) {
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>> plantPointClouds;

        // 遍历器官（叶子和果实）
        for (int y = 0; y <= 1; ++y) {
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> organPointClouds;

            // 遍历果实
            for (int z = 0; z <= maxX; ++z) {
                // 构建文件名，基于植株、器官和果实编号
                std::string filename = baseDir + "22_plant_" + std::to_string(x + 1);

                if (y == 0 && z == 0) {
                    filename += "_leaves_pc.pcd";
                }
                else if (y == 1) {
                    filename += "_fruit_" + std::to_string(z + 1) + "_pc.pcd";
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
                std::cout << "植株 " << x + 1 << "，器官 " << (y == 0 ? "叶子" : "果实 " + std::to_string(z + 1))
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

    // 打开CSV文件
    std::ofstream csvFile(baseDir + "vertical_distances.csv");

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

    //遍历植株计算多条射线的叶间距
    // 遍历植株
    for (int x = 0; x < pointClouds.size(); ++x) {
        // 获取当前植株的叶片点云
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>& leavesPointClouds = pointClouds[x][0];

        // 遍历叶片点云
        for (int i = 0; i < leavesPointClouds.size(); ++i) {
            std::cout << "植株 " << x + 1 << "，叶片 " << i + 1 << "：\n";
            // 对叶片点云进行50次采样、排序和相邻点间距计算
            sampleLeavesPointCloud(leavesPointClouds[i], 2000, 0.06f, csvFile,x+1);
        }
    }

    // 关闭CSV文件
    csvFile.close();

    //delete the empty line
    const std::string inputFileName = "E:/Tomato_data/seg_organs/vertical_distances.csv";
    const std::string outputFileName = "E:/Tomato_data/seg_organs/filtered_vertical_distances_no_empty.csv";

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

    return 0;
}
