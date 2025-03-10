#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/colors.h>
#include <vector>
#include <map>

int main()
{
    // �ļ���·��
    std::string folder_path = "E:\\Tomato_data\\seg_organs";

    // �洢ֲ��ĵ���
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> plants;

    // �洢ֲ��ĵ���
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> stems;

    // Map to store plant indices and their corresponding colors
    std::map<int, pcl::RGB> plantColorMap;

    // Visualization object
    pcl::visualization::PCLVisualizer viewer("PointCloud Viewer");

    std::vector<int> fruit_idx_for_each_plant;

    // ѭ����ȡ���ѵ�������
    for (int N = 1; ; N++) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        std::string stem_filename = folder_path + "\\22_stem_" + std::to_string(N) + "_pc.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(stem_filename, *cloud) == -1) {
            // �޷����ص����ļ��������Ѿ���ȡ���
            break;
        }

        // �洢ֲ��ĵ���
        plants.push_back(cloud);

        stems.push_back(cloud);

        fruit_idx_for_each_plant.push_back(0);

        // �����ǰ���ص�ֲ������ļ���
        std::cout << "Loaded plant: " << stem_filename << std::endl;

        // Assign random colors to each plant and add them to the viewer
        pcl::RGB color(rand() % 256, rand() % 256, rand() % 256);
        plantColorMap[N - 1] = color;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color_handler(cloud, color.r, color.g, color.b);
        viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, color_handler, "plant_" + std::to_string(N - 1));
    }

    // ѭ����ȡ��ʵ��������
    for (int M = 1; ; M++) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        std::string fruit_filename = folder_path + "\\22_fruit_" + std::to_string(M) + "_pc.pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fruit_filename, *cloud) == -1) {
            // �޷����ص����ļ��������Ѿ���ȡ���
            break;
        }

        // �����ǰ���صĹ�ʵ�����ļ���
        std::cout << "Loaded fruit: " << fruit_filename << std::endl;

        // Ѱ���뵱ǰ��ʵ�����ֲ��
        pcl::PointXYZRGBA fruit_centroid;
        pcl::computeCentroid(*cloud, fruit_centroid);

        double min_distance = std::numeric_limits<double>::max();
        int closest_plant_index = -1;

        for (size_t i = 0; i < stems.size(); ++i) {
            pcl::PointXYZRGBA stem_centroid;
            pcl::computeCentroid(*(stems[i]), stem_centroid);

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
        std::string fruit_in_plant_filename = folder_path + "\\22_plant_" + std::to_string(closest_plant_index + 1) + "_fruit_" + std::to_string(fruit_idx_for_each_plant[closest_plant_index]) + "_pc.pcd";
        pcl::io::savePCDFileBinary(fruit_in_plant_filename, *cloud);
        std::cout << "Saved plant: " << fruit_in_plant_filename << std::endl;

        // Concatenate the plant and fruit into a single point cloud
        *plants[closest_plant_index] += *cloud;

        // Assign the color of the corresponding plant to the fruit for visualization
        pcl::RGB plantColor = plantColorMap[closest_plant_index];
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> fruit_color_handler(cloud, plantColor.r, plantColor.g, plantColor.b);
        viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, fruit_color_handler, "fruit_" + std::to_string(M));
    }

    // leaves seg
    // ��ȡҶƬ��������
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr leaf_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::string leaf_filename = folder_path + "\\22_leaf_pc.pcd";
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

            for (size_t i = 0; i < stems.size(); ++i) {
                pcl::PointXYZRGBA stem_centroid;
                pcl::computeCentroid(*(stems[i]), stem_centroid);

                double distance = std::sqrt((leaf_point.x - stem_centroid.x) * (leaf_point.x - stem_centroid.x) +
                    (leaf_point.y - stem_centroid.y) * (leaf_point.y - stem_centroid.y));

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_plant_index = i;
                }
            }

            // ��ҶƬ����������Ӧ��ֲ���������浽ӳ����
            leafPlantMap[point_index] = closest_plant_index;

            // �����������Ӷ�������
            std::cout << "Leaf point " << point_index << " belongs to plant: " << closest_plant_index + 1 << std::endl;
        }

        // ���ݹ�����Ϣ�ϲ�ͬһֲ���ҶƬ����
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> mergedLeafClouds(plants.size());
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
                std::string merged_leaf_filename = folder_path + "\\22_plant_" + std::to_string(i + 1) + "_leaves_pc.pcd";
                pcl::io::savePCDFileBinary(merged_leaf_filename, *mergedLeafClouds[i]);
                std::cout << "Saved merged leaves: " << merged_leaf_filename << std::endl;
                *plants[i] += *mergedLeafClouds[i];

                // Assign the color of the corresponding plant to the leaves for visualization
                pcl::RGB plantColor = plantColorMap[i];
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> leaves_color_handler(mergedLeafClouds[i], plantColor.r, plantColor.g, plantColor.b);
                viewer.addPointCloud<pcl::PointXYZRGBA>(mergedLeafClouds[i], leaves_color_handler, "leaf_" + std::to_string(i));
            }
        }
    }

    // Save the concatenated point clouds with original colors
    for (size_t i = 0; i < plants.size(); ++i) {
        std::string output_filename = folder_path + "\\22_plant_" + std::to_string(i + 1) + "_pc.pcd";
        pcl::io::savePCDFileBinary(output_filename, *plants[i]);
        std::cout << "Saved plant: " << output_filename << std::endl;
    }

    // Spin to keep the viewer open
    viewer.spin();

    return 0;
}
