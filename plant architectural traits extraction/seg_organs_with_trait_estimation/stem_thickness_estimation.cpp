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

int main() {
    // ѭ����������ļ�
    for (int file_index = 1; ; ++file_index) {
        // ���������ļ���
        std::string file_name = "E:/Tomato_data/seg_organs/22_stem_" + std::to_string(file_index) + "_pc.pcd";

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
        std::ofstream csvFile("E:\\Tomato_data\\seg_organs\\stem_thickness.csv", std::ios::app);  // ���ļ���׷�ӷ�ʽд��

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

    return 0;
}
