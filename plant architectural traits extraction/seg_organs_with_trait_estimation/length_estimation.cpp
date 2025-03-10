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
void sampleLeavesPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, int numSamples, float sampleSize, std::ofstream& csvFile,int x) {
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

        //// ����ֱ�߶Ƚ�������
        //std::sort(sampledRegion.points.begin(), sampledRegion.points.end(),
        //    [](const auto& a, const auto& b) {
        //        return a.z < b.z;
        //    });

        //// �������ڵ���
        //std::cout << "����������" << sampleIndex + 1 << "\n";
        //std::cout << "���ڵ��ࣺ\n";
        //for (int i = 0; i + 1 < sampledRegion.size(); ++i) {
        //    float distance =sampledRegion[i + 1].z - sampledRegion[i].z;

        //    if (distance < 0.06)
        //        continue;

        //    std::cout << "�� " << i + 1 << " �͵� " << i + 2 << " �ľ���Ϊ��" << distance << " �ס�\n";

        //    // �����д�� CSV �ļ�
        //    csvFile << distance;

        //    // ����������һ�У�����붺��
        //    if (i + 2 < sampledRegion.size()) {
        //        csvFile << ",";
        //    }
        //}
        //std::cout << "\n";

        //// ����
        //csvFile << "\n";

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
    // ����PCD�ļ��Ļ���Ŀ¼
    const std::string baseDir = "E:/Tomato_data/seg_organs/";

    // �������ֲ����
    const int maxX = 10;  // �������ֲ������10

    // ���vector���ڴ洢��������
    std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>>> pointClouds;

    // ����ֲ��
    for (int x = 0; x <= maxX; ++x) {
        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>> plantPointClouds;

        // �������٣�Ҷ�Ӻ͹�ʵ��
        for (int y = 0; y <= 1; ++y) {
            std::vector<pcl::PointCloud<pcl::PointXYZRGBA>> organPointClouds;

            // ������ʵ
            for (int z = 0; z <= maxX; ++z) {
                // �����ļ���������ֲ�ꡢ���ٺ͹�ʵ���
                std::string filename = baseDir + "22_plant_" + std::to_string(x + 1);

                if (y == 0 && z == 0) {
                    filename += "_leaves_pc.pcd";
                }
                else if (y == 1) {
                    filename += "_fruit_" + std::to_string(z + 1) + "_pc.pcd";
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
                std::cout << "ֲ�� " << x + 1 << "������ " << (y == 0 ? "Ҷ��" : "��ʵ " + std::to_string(z + 1))
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

    // ��CSV�ļ�
    std::ofstream csvFile(baseDir + "vertical_distances.csv");

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

    //����ֲ�����������ߵ�Ҷ���
    // ����ֲ��
    for (int x = 0; x < pointClouds.size(); ++x) {
        // ��ȡ��ǰֲ���ҶƬ����
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>>& leavesPointClouds = pointClouds[x][0];

        // ����ҶƬ����
        for (int i = 0; i < leavesPointClouds.size(); ++i) {
            std::cout << "ֲ�� " << x + 1 << "��ҶƬ " << i + 1 << "��\n";
            // ��ҶƬ���ƽ���50�β�������������ڵ������
            sampleLeavesPointCloud(leavesPointClouds[i], 2000, 0.06f, csvFile,x+1);
        }
    }

    // �ر�CSV�ļ�
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

    return 0;
}
