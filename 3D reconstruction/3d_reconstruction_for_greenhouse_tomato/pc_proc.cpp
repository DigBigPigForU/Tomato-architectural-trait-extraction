#pragma once
#include <vtkAutoInit.h>
#pragma comment(lib, "User32.lib")
#pragma comment(lib, "gdi32.lib")
#include <conio.h>
#include <string>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <k4a/k4a.hpp>

#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <ratio>
#include <thread>
#include <filesystem> // C++17

#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

//pass through
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>


using namespace std;
using namespace cv;
using namespace boost;
using namespace pcl;
using namespace std::chrono;
using namespace Eigen;

namespace fs = std::filesystem;

typedef pcl::PointXYZRGBA PointType;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA(k4a::image&, k4a::image&, k4a::transformation&);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_transformed(k4a::image&, k4a::image&, k4a::transformation&);
inline bool endsWith(const string&, const string&);
void frames2pc(const string&, const string&, const string&, const bool&, const bool&, k4a::transformation&);//rgbd pcd
void save_pose_txt_to_each_folder(const string&, const Eigen::Matrix4d& , const Eigen::Matrix4d& , const Eigen::Matrix4d& );
pcl::PointCloud<PointType>::Ptr passThroughPc(pcl::PointCloud<PointType>::Ptr);
float pointDistance(PointType, PointType);
float str2int(string);
void vec3fNorm(Eigen::Vector3f&);
void vec3dNorm(Eigen::Vector3d&);
Eigen::Vector3f vec3fCross(Eigen::Vector3f, Eigen::Vector3f);
double vectorLength(double rx, double ry, double rz);
Eigen::Vector3d vec3dCross(Eigen::Vector3d, Eigen::Vector3d);
vector<Matrix4d> processCSVFile(const string& csv_file);
void saveCloudsToTXT(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& clouds, const std::string& save_file_path);
void saveCloudsToTXT_no_0_and_255(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& clouds, const std::string& save_file_path);
void frames2pc_v2(const string&, const string&, const string&, const bool&, const bool&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);//rgbd pcd
void frames2pc_v3(const string&, const string&, const string&, const string&, const string&, const string&, const string&, const bool&, const bool&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);//rgbd plant organ pcd
void frames2pc_v4(const string&, const string&, const std::vector<std::string>&, const string&, const string&, const bool&, const bool&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);//msd txt all and no 0 and 255 dn
void frames2pc_v5(const string&, const string&, const std::vector<std::string>&, const string&, const string&, const string&, const string&, const string&, const string&, const string&, const string&, const string&, const bool&, const bool&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);//msd txt organ and no 0 and 255 dn
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_v2(k4a::image&, k4a::image&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_transformed_v2(k4a::image&, k4a::image&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_v3(k4a::image&, k4a::image&,cv::Mat&,int&,k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_transformed_v3(k4a::image&, k4a::image&,cv::Mat&,int&,k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> convertMSDepthToPointXYZMS_v4(k4a::image&, k4a::image&, std::vector<k4a::image>&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> convertMSDepthToPointXYZMS_transformed_v4(k4a::image&, k4a::image&, std::vector<k4a::image>&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> convertMSDepthToPointXYZMS_v5(k4a::image&, k4a::image&, std::vector<k4a::image>&, cv::Mat&, int&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);
std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> convertMSDepthToPointXYZMS_transformed_v5(k4a::image&, k4a::image&, std::vector<k4a::image>&, cv::Mat&, int&, k4a::transformation&, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c);

int main(int argc, char* argv[])
{
    std::string baseFolder = "E:\\Tomato_data";
    std::string targetFolder = "F:\\FRC-REF";

    vector<string> csv_files = { "E:\\Tomato_data\\map_and_pose_data\\UR_poses_data.csv", "E:\\Tomato_data\\map_and_pose_data\\tomato_data_proc\\T_230315_v3.csv", "E:\\Tomato_data\\map_and_pose_data\\tomato_data_proc\\T_230705_v3.csv" };
    Eigen::Matrix4d T_a2b(4, 4);

    T_a2b << 0, -1, 0, 0,
        1, 0, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    vector<vector<Matrix4d>> all_matrices; // 存储所有矩阵的向量

    for (const string& csv_file : csv_files) {
        cout << "Processing " << csv_file << ":\n";

        // 调用处理CSV文件的函数
        vector<Matrix4d> matrices = processCSVFile(csv_file);

        // 输出矩阵的数量
        cout << "Number of matrices in " << csv_file << ": " << matrices.size() << endl;

        // 将矩阵向量添加到all_matrices中
        all_matrices.push_back(matrices);
    }

    bool is_transformed = true;
    bool is_mkv_frames = false;

    // Iterate over folders in the base folder
    for (const auto& entry : fs::directory_iterator(baseFolder))
    {
        // Check if the folder name starts with "2023" and is a directory
        if (entry.is_directory() && entry.path().filename().string().find("2023") == 0)
        {
            std::string tempFolder = entry.path().string();
            std::cout << "Processing folder: " << tempFolder << std::endl;

            int year_number = 0;
            size_t year_start = tempFolder.find("20230");
            if (year_start != std::string::npos && year_start + 5 < tempFolder.length()) {
                std::string year_number_str = tempFolder.substr(year_start + 5);
                year_number = std::stoi(year_number_str);
                std::cout << "年份数字：" << year_number << std::endl;
            }

            // Iterate over subfolders in the current folder
            for (const auto& subentry : fs::directory_iterator(tempFolder))
            {
                // Check if the subfolder starts with "p"
                if (subentry.is_directory() && subentry.path().filename().string().find("p") == 0)
                {
                    std::string subfolderPath = subentry.path().string();
                    std::cout << "Processing subfolder: " << subfolderPath << std::endl;

                    int p_number = 0;
                    size_t p_start = subfolderPath.find("p");
                    if (p_start != std::string::npos && p_start + 1 < subfolderPath.length()) {
                        std::string p_number_str = subfolderPath.substr(p_start + 1);
                        p_number = std::stoi(p_number_str);
                        std::cout << "p数字：" << p_number << std::endl;
                    }

                    // 根据数字选择第一个矩阵
                    Matrix4d selected_matrix1;
                    int selected_matrix1_row = -1;
                    int selected_matrix1_col = -1;

                    selected_matrix1_row = (year_number == 315 || year_number == 403) ? 1 : 2;

                    // 根据数字选择第二个矩阵
                    Matrix4d selected_matrix2;
                    int selected_matrix2_row = 0;  // 默认选择第一页
                    int selected_matrix2_col = p_number - 1;

                    if (p_number >= 1 && p_number <= all_matrices[0].size()) {
                        // 如果 "p" 后面的数字在 1 到 all_matrices[0] 的大小之间
                        selected_matrix2 = all_matrices[0][p_number - 1];
                        std::cout << "选择的 T_base2cam 在 all_matrices 中的索引：第 " << selected_matrix2_row + 1 << " 页，第 " << selected_matrix2_col + 1 << " 个矩阵" << std::endl;
                    }

                    // Process frames in this subfolder
                    for (int R = 0; R <= 5; R++)
                        for (int N = 1; N <= 24; N++)
                        {
                            int i = 24 * R + N;

                            if (p_number == 1 || p_number == 4 || p_number == 5 || p_number == 7 || p_number == 9) {
                                if (N == 1) {
                                    // 如果 N 等于 1，根据年份选择矩阵
                                    selected_matrix1 = (year_number == 315 || year_number == 403) ? all_matrices[1][23 * R + N - 1] : all_matrices[2][23 * R + N - 1];
                                    selected_matrix1_col = 23 * R + N - 1;
                                }
                                else {
                                    // 如果 N 不等于 1，根据 N 的不同值选择矩阵
                                    selected_matrix1 = (year_number == 315 || year_number == 403) ? all_matrices[1][23 * R + N - 2] : all_matrices[2][23 * R + N - 2];
                                    selected_matrix1_col = 23 * R + N - 2;
                                }
                                std::cout << "选择的 T_greenhouse2base 在 all_matrices 中的索引：第 " << selected_matrix1_row + 1 << " 页，第 " << selected_matrix1_col + 1 << " 个矩阵" << std::endl;
                            }
                            else {
                                if (N == 24) {
                                    // 如果 N 等于 1，根据年份选择矩阵
                                    selected_matrix1 = (year_number == 315 || year_number == 403) ? all_matrices[1][23 * R + N - 2] : all_matrices[2][23 * R + N - 2];
                                    selected_matrix1_col = 23 * R + N - 2;
                                }
                                else {
                                    // 如果 N 不等于 1，根据 N 的不同值选择矩阵
                                    selected_matrix1 = (year_number == 315 || year_number == 403) ? all_matrices[1][23 * R + N - 1] : all_matrices[2][23 * R + N - 1];
                                    selected_matrix1_col = 23 * R + N - 1;
                                }
                                std::cout << "选择的 T_greenhouse2base 在 all_matrices 中的索引：第 " << selected_matrix1_row + 1 << " 页，第 " << selected_matrix1_col + 1 << " 个矩阵" << std::endl;
                            }

                            // Construct paths for color, depth, and point cloud files
                            //std::string color_file_path = subfolderPath + "\\" + std::to_string(i) + "\\" + std::to_string(i) + "_color_uint8.png";
                            std::string color_file_path = subfolderPath + "\\" + std::to_string(i) + "\\" + std::to_string(i) + "_color_uint8_segmented.png";
                            //std::string color_file_path = subfolderPath + "\\" + std::to_string(i) + "\\" + std::to_string(i) + "_color_uint8_segmented_corrected.png";
                            std::string depth_file_path = subfolderPath + "\\" + std::to_string(i) + "\\" + std::to_string(i) + "_depth_uint16.png";
                            std::string calibration_file_path = subfolderPath + "\\" + std::to_string(i) + "\\" + std::to_string(i) + "_calibration.json";
                            std::string save_file_path = subfolderPath + "\\" + std::to_string(i) + "_pc.pcd";
                            std::string save_pose_txt_path = subfolderPath + "\\" + std::to_string(i) + "\\" + std::to_string(i) + "_pose.txt";

                            std::string label_file_path = subfolderPath + "\\" + std::to_string(i) + "\\" + "raw_label.png";
                            std::string save_stem_pc_path = subfolderPath + "\\" + "stems" + "\\" + std::to_string(i) + "_stem_pc.pcd";
                            std::string save_fruit_pc_path = subfolderPath + "\\" + "fruits" + "\\" + std::to_string(i) + "_fruit_pc.pcd";
                            std::string save_leaf_pc_path = subfolderPath + "\\" + "leaves" + "\\" + std::to_string(i) + "_leaf_pc.pcd";
                            std::string save_flower_pc_path = subfolderPath + "\\" + "flowers" + "\\" + std::to_string(i) + "_flower_pc.pcd";

                            // ok ms dn file path and save path
                            std::vector<std::string> ok_ms_dn_file_paths;
                            for (int b = 1; b <= 25; ++b) {
                                std::string ok_ms_dn_band_b_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + std::to_string(i) + "\\" + "msi_reflectance_moved" + "\\" + "part" + std::to_string(b) + ".png";
                                ok_ms_dn_file_paths.push_back(ok_ms_dn_band_b_file_path);
                            }
                            std::string save_dn_pc_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_pc_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc.txt";
                            std::string save_dn_no_0_and_255_pc_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_no_0_and_255_pc_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc.txt";
                            std::string save_dn_pc_stem_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_pc_stem_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc_stem.txt";
                            std::string save_dn_pc_fruit_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_pc_fruit_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc_fruit.txt";
                            std::string save_dn_pc_leaf_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_pc_leaf_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc_leaf.txt";
                            std::string save_dn_pc_flower_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_pc_flower_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc_flower.txt";
                            std::string save_dn_no_0_and_255_pc_stem_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_no_0_and_255_pc_stem_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc_stem.txt";
                            std::string save_dn_no_0_and_255_pc_fruit_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_no_0_and_255_pc_fruit_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc_fruit.txt";
                            std::string save_dn_no_0_and_255_pc_leaf_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_no_0_and_255_pc_leaf_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc_leaf.txt";
                            std::string save_dn_no_0_and_255_pc_flower_txt_file_path = targetFolder + subfolderPath.substr(baseFolder.length()) + "\\" + "ms_dn_no_0_and_255_pc_flower_txt_all" + "\\" + std::to_string(i) + "_ms_dn_pc_flower.txt";

                            // Load calibration data (you may want to modify this logic)
                            std::ifstream fin(calibration_file_path, std::ios::binary);
                            std::vector<uint8_t> buffer(std::istreambuf_iterator<char>(fin), {});
                            buffer.push_back('\0');

                            k4a::calibration calibration = k4a::calibration::get_from_raw(
                                buffer.data(),
                                buffer.size(),
                                K4A_DEPTH_MODE_NFOV_UNBINNED,
                                K4A_COLOR_RESOLUTION_720P);

                            k4a::transformation transformation(calibration);

                            // Process the frame
                            //frames2pc(color_file_path, depth_file_path, save_file_path, is_transformed, is_mkv_frames, transformation);
                            //frames2pc_v2(color_file_path, depth_file_path, save_file_path, is_transformed, is_mkv_frames, transformation, selected_matrix1,T_a2b, selected_matrix2);   // use this line please
                            //frames2pc_v3(color_file_path, depth_file_path, label_file_path, save_stem_pc_path, save_fruit_pc_path, save_leaf_pc_path, save_flower_pc_path, is_transformed, is_mkv_frames, transformation, selected_matrix1, T_a2b, selected_matrix2);
                            frames2pc_v4(color_file_path, depth_file_path, ok_ms_dn_file_paths, save_dn_pc_txt_file_path, save_dn_no_0_and_255_pc_txt_file_path, is_transformed, is_mkv_frames, transformation, selected_matrix1, T_a2b, selected_matrix2);
                            frames2pc_v5(color_file_path, depth_file_path, ok_ms_dn_file_paths, label_file_path, save_dn_pc_stem_txt_file_path, save_dn_no_0_and_255_pc_stem_txt_file_path, save_dn_pc_fruit_txt_file_path, save_dn_no_0_and_255_pc_fruit_txt_file_path, save_dn_pc_leaf_txt_file_path, save_dn_no_0_and_255_pc_leaf_txt_file_path, save_dn_pc_flower_txt_file_path, save_dn_no_0_and_255_pc_flower_txt_file_path,is_transformed, is_mkv_frames, transformation, selected_matrix1, T_a2b, selected_matrix2);
                            //save_pose_txt_to_each_folder(save_pose_txt_path, selected_matrix1, T_a2b, selected_matrix2);
                            std::cout << "--------" << i << "--------" << "Saving file done!" << std::endl;
                        }
                }
            }
        }
    }

    std::cout << "--------" << "All" << "--------" << "Saving files done!" << std::endl;
    return 0;
}

// 函数用于处理单个CSV文件并返回矩阵向量
vector<Matrix4d> processCSVFile(const string& csv_file) {
    ifstream file(csv_file);
    string line;
    vector<Matrix4d> matrices; // 存储矩阵的向量

    // 逐行读取CSV文件
    while (getline(file, line)) {
        stringstream ss(line);
        vector<double> row_data;
        double value;

        // 逐行读取CSV数据并将其存储在vector中
        while (ss >> value) {
            row_data.push_back(value);
            if (ss.peek() == ',') {
                ss.ignore();
            }
        }

        // 检查每行是否包含16个元素，然后将其转换为4x4矩阵
        if (row_data.size() != 16) {
            cerr << "Error: CSV row in " << csv_file << " does not contain 16 elements." << endl;
            continue;
        }

        // 创建4x4矩阵并填充数据
        Matrix4d matrix;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix(i, j) = row_data[i * 4 + j];
            }
        }

        matrices.push_back(matrix);
    }

    return matrices;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA(k4a::image& colorImage, k4a::image& depthImage, k4a::transformation& transformation)
{
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();

    for (int i = 0; i < width * height; ++i)
    {
        PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        //        if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
        //        {
        //            continue;
        //        }

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }
    PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>());
    //    cloud_filtered = radiusFilteringPc( cloud);
    //    cloud_filtered = cloud;
    cloud_filtered = passThroughPc(cloud);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_v2(k4a::image& colorImage, k4a::image& depthImage, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();

    for (int i = 0; i < width * height; ++i)
    {
        PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        //        if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
        //        {
        //            continue;
        //        }

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }
    PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>());
    //    cloud_filtered = radiusFilteringPc( cloud);
    //    cloud_filtered = cloud;
    cloud_filtered = passThroughPc(cloud);

    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);

    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T_b2c);

    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_a2b);

    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_o2a);

    return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_v3(k4a::image& colorImage, k4a::image& depthImage,cv::Mat& label_img,int& category, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();
    uint8_t* label_image_data = label_img.data;

    for (int i = 0; i < width * height; ++i)
    {
        PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        if (label_image_data[i] != category)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        //        if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
        //        {
        //            continue;
        //        }

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }
    PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>());
    //    cloud_filtered = radiusFilteringPc( cloud);
    //    cloud_filtered = cloud;
    cloud_filtered = passThroughPc(cloud);

    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);

    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T_b2c);

    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_a2b);

    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_o2a);

    return transformed_cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> convertMSDepthToPointXYZMS_v4(k4a::image& colorImage, k4a::image& depthImage, std::vector<k4a::image>& ok_ms_dn_images, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> ms_clouds;

    // Process colorImage
    for (int i = 0; i < width * height; ++i)
    {
        pcl::PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud_filtered = passThroughPc(cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T_b2c);
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_a2b);
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_o2a);

    ms_clouds.push_back(transformed_cloud);

    // Process each ok_ms_dn_image using a traditional for loop
    for (size_t idx = 0; idx < ok_ms_dn_images.size(); ++idx)
    {
        k4a::image& ms_image = ok_ms_dn_images[idx];

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ms_cloud->width = width;
        ms_cloud->height = height;
        ms_cloud->is_dense = false;
        ms_cloud->points.resize(ms_cloud->height * ms_cloud->width);

        uint8_t* ms_image_data = ms_image.get_buffer();

        for (int i = 0; i < width * height; ++i)
        {
            pcl::PointXYZRGBA point;

            point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
            point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
            point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

            if (point.z == 0)
            {
                continue;
            }

            point.b = ms_image_data[4 * i + 0];
            point.g = ms_image_data[4 * i + 1];
            point.r = ms_image_data[4 * i + 2];
            point.a = ms_image_data[4 * i + 3];

            if (color_image_data[4 * i + 0] == 0 && color_image_data[4 * i + 1] == 0 && color_image_data[4 * i + 2] == 0)
            {
                continue;
            }

            ms_cloud->points[i] = point;
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ms_cloud_filtered = passThroughPc(ms_cloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::transformPointCloud(*ms_cloud_filtered, *ms_transformed_cloud, T_b2c);
        pcl::transformPointCloud(*ms_transformed_cloud, *ms_transformed_cloud, T_a2b);
        pcl::transformPointCloud(*ms_transformed_cloud, *ms_transformed_cloud, T_o2a);

        ms_clouds.push_back(ms_transformed_cloud);
    }

    return ms_clouds;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> convertMSDepthToPointXYZMS_v5(k4a::image& colorImage, k4a::image& depthImage, std::vector<k4a::image>& ok_ms_dn_images, cv::Mat& label_img, int& category, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();
    uint8_t* label_image_data = label_img.data;

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> ms_clouds;

    // Process colorImage
    for (int i = 0; i < width * height; ++i)
    {
        pcl::PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        if (label_image_data[i] != category)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud_filtered = passThroughPc(cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T_b2c);
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_a2b);
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_o2a);

    ms_clouds.push_back(transformed_cloud);

    // Process each ok_ms_dn_image using a traditional for loop
    for (size_t idx = 0; idx < ok_ms_dn_images.size(); ++idx)
    {
        k4a::image& ms_image = ok_ms_dn_images[idx];

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ms_cloud->width = width;
        ms_cloud->height = height;
        ms_cloud->is_dense = false;
        ms_cloud->points.resize(ms_cloud->height * ms_cloud->width);

        uint8_t* ms_image_data = ms_image.get_buffer();

        for (int i = 0; i < width * height; ++i)
        {
            pcl::PointXYZRGBA point;

            point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
            point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
            point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

            if (point.z == 0)
            {
                continue;
            }

            if (label_image_data[i] != category)
            {
                continue;
            }

            point.b = ms_image_data[4 * i + 0];
            point.g = ms_image_data[4 * i + 1];
            point.r = ms_image_data[4 * i + 2];
            point.a = ms_image_data[4 * i + 3];

            if (color_image_data[4 * i + 0] == 0 && color_image_data[4 * i + 1] == 0 && color_image_data[4 * i + 2] == 0)
            {
                continue;
            }

            ms_cloud->points[i] = point;
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ms_cloud_filtered = passThroughPc(ms_cloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::transformPointCloud(*ms_cloud_filtered, *ms_transformed_cloud, T_b2c);
        pcl::transformPointCloud(*ms_transformed_cloud, *ms_transformed_cloud, T_a2b);
        pcl::transformPointCloud(*ms_transformed_cloud, *ms_transformed_cloud, T_o2a);

        ms_clouds.push_back(ms_transformed_cloud);
    }

    return ms_clouds;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_transformed(k4a::image& colorImage, k4a::image& depthImage, k4a::transformation& transformation)
{
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    //    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformed_depth_image = depthImage;
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();

    for (int i = 0; i < width * height; ++i)
    {
        PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        //        if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
        //        {
        //            continue;
        //        }

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }
    PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>());
    //    cloud_filtered = radiusFilteringPc( cloud);
    //    cloud_filtered = cloud;
    cloud_filtered = passThroughPc(cloud);
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_transformed_v2(k4a::image& colorImage, k4a::image& depthImage, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    //    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformed_depth_image = depthImage;
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();

    for (int i = 0; i < width * height; ++i)
    {
        PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        //        if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
        //        {
        //            continue;
        //        }

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }
    PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>());
    //    cloud_filtered = radiusFilteringPc( cloud);
    //    cloud_filtered = cloud;
    cloud_filtered = passThroughPc(cloud);
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);

    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T_b2c);

    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_a2b);

    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_o2a);

    return transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA_transformed_v3(k4a::image& colorImage, k4a::image& depthImage,cv::Mat& label_img,int& category, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = NULL;
    transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = NULL;
    point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    //    transformation.depth_image_to_color_camera(depthImage, &transformed_depth_image);
    transformed_depth_image = depthImage;
    transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();
    uint8_t* label_image_data = label_img.data;

    for (int i = 0; i < width * height; ++i)
    {
        PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        if (label_image_data[i] != category)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        //        if (point.b == 0 && point.g == 0 && point.r == 0 && point.a == 0)
        //        {
        //            continue;
        //        }

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }
    PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>());
    //    cloud_filtered = radiusFilteringPc( cloud);
    //    cloud_filtered = cloud;
    cloud_filtered = passThroughPc(cloud);
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);

    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T_b2c);

    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_a2b);

    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_o2a);

    return transformed_cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> convertMSDepthToPointXYZMS_transformed_v4(k4a::image& colorImage, k4a::image& depthImage, std::vector<k4a::image>& ok_ms_dn_images, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> ms_clouds;

    // Process colorImage
    for (int i = 0; i < width * height; ++i)
    {
        pcl::PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud_filtered = passThroughPc(cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T_b2c);
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_a2b);
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_o2a);

    ms_clouds.push_back(transformed_cloud);

    // Process each ok_ms_dn_image using a traditional for loop
    for (size_t idx = 0; idx < ok_ms_dn_images.size(); ++idx)
    {
        k4a::image& ms_image = ok_ms_dn_images[idx];

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ms_cloud->width = width;
        ms_cloud->height = height;
        ms_cloud->is_dense = false;
        ms_cloud->points.resize(ms_cloud->height * ms_cloud->width);

        uint8_t* ms_image_data = ms_image.get_buffer();

        for (int i = 0; i < width * height; ++i)
        {
            pcl::PointXYZRGBA point;

            point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
            point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
            point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

            if (point.z == 0)
            {
                continue;
            }

            point.b = ms_image_data[4 * i + 0];
            point.g = ms_image_data[4 * i + 1];
            point.r = ms_image_data[4 * i + 2];
            point.a = ms_image_data[4 * i + 3];

            if (color_image_data[4 * i + 0] == 0 && color_image_data[4 * i + 1] == 0 && color_image_data[4 * i + 2] == 0)
            {
                continue;
            }

            ms_cloud->points[i] = point;
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ms_cloud_filtered = passThroughPc(ms_cloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::transformPointCloud(*ms_cloud_filtered, *ms_transformed_cloud, T_b2c);
        pcl::transformPointCloud(*ms_transformed_cloud, *ms_transformed_cloud, T_a2b);
        pcl::transformPointCloud(*ms_transformed_cloud, *ms_transformed_cloud, T_o2a);

        ms_clouds.push_back(ms_transformed_cloud);
    }

    return ms_clouds;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> convertMSDepthToPointXYZMS_transformed_v5(k4a::image& colorImage, k4a::image& depthImage, std::vector<k4a::image>& ok_ms_dn_images, cv::Mat& label_img, int& category, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    int color_image_width_pixels = colorImage.get_width_pixels();
    int color_image_height_pixels = colorImage.get_height_pixels();

    k4a::image transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t));

    k4a::image point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t));

    transformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int width = colorImage.get_width_pixels();
    int height = colorImage.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    int16_t* point_cloud_image_data = (int16_t*)(void*)point_cloud_image.get_buffer();
    uint8_t* color_image_data = colorImage.get_buffer();
    uint8_t* label_image_data = label_img.data;

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> ms_clouds;

    // Process colorImage
    for (int i = 0; i < width * height; ++i)
    {
        pcl::PointXYZRGBA point;

        point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if (point.z == 0)
        {
            continue;
        }

        point.b = color_image_data[4 * i + 0];
        point.g = color_image_data[4 * i + 1];
        point.r = color_image_data[4 * i + 2];
        point.a = color_image_data[4 * i + 3];

        if (label_image_data[i] != category)
        {
            continue;
        }

        if (point.b == 0 && point.g == 0 && point.r == 0)
        {
            continue;
        }

        cloud->points[i] = point;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud_filtered = passThroughPc(cloud);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, T_b2c);
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_a2b);
    pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, T_o2a);

    ms_clouds.push_back(transformed_cloud);

    // Process each ok_ms_dn_image using a traditional for loop
    for (size_t idx = 0; idx < ok_ms_dn_images.size(); ++idx)
    {
        k4a::image& ms_image = ok_ms_dn_images[idx];

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ms_cloud->width = width;
        ms_cloud->height = height;
        ms_cloud->is_dense = false;
        ms_cloud->points.resize(ms_cloud->height * ms_cloud->width);

        uint8_t* ms_image_data = ms_image.get_buffer();

        for (int i = 0; i < width * height; ++i)
        {
            pcl::PointXYZRGBA point;

            point.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
            point.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
            point.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

            if (point.z == 0)
            {
                continue;
            }

            point.b = ms_image_data[4 * i + 0];
            point.g = ms_image_data[4 * i + 1];
            point.r = ms_image_data[4 * i + 2];
            point.a = ms_image_data[4 * i + 3];

            if (label_image_data[i] != category)
            {
                continue;
            }

            if (color_image_data[4 * i + 0] == 0 && color_image_data[4 * i + 1] == 0 && color_image_data[4 * i + 2] == 0)
            {
                continue;
            }

            ms_cloud->points[i] = point;
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>());
        ms_cloud_filtered = passThroughPc(ms_cloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ms_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::transformPointCloud(*ms_cloud_filtered, *ms_transformed_cloud, T_b2c);
        pcl::transformPointCloud(*ms_transformed_cloud, *ms_transformed_cloud, T_a2b);
        pcl::transformPointCloud(*ms_transformed_cloud, *ms_transformed_cloud, T_o2a);

        ms_clouds.push_back(ms_transformed_cloud);
    }

    return ms_clouds;
}

inline bool endsWith(const string& str, const string& suffix)
{

    if (str.size() < suffix.size()) {
        return false;
    }

    auto tstr = str.substr(str.size() - suffix.size());

    return tstr.compare(suffix) == 0;
}

void frames2pc(const string& color_file_path, const string& depth_file_path, const string& save_file_path, const bool& is_transformed, const bool& is_mkv_frames, k4a::transformation& transformation)
{
    cv::Mat color_img = cv::imread(color_file_path, cv::IMREAD_UNCHANGED);
    if (color_img.empty() || color_img.depth() != CV_8U)
    {
        std::cerr
            << "WARNING: cannot read color image. No such a file, or the image format is not CV_8U"
            << std::endl;
    }

    //    //if mkv frames, the channels is 3 rather than 4
    //    if(is_mkv_frames)
    //    {
    //        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    //    }
    //    //

        //if the num of channnels is 3, then  set it to 4
    if (color_img.channels() == 3)
    {
        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    }
    //

    cv::Mat depth_img = cv::imread(depth_file_path, cv::IMREAD_ANYDEPTH);
    if (depth_img.empty() || depth_img.depth() != CV_16U)
    {
        std::cerr
            << "WARNING: cannot read depth image. No such a file, or the image format is not CV_16U"
            << std::endl;
    }

    k4a::image colorImage = NULL;
    k4a::image depthImage = NULL;

    colorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_img.size().width,
        color_img.size().height,
        color_img.size().width * 4 * (int)sizeof(uint8_t));

    depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        depth_img.size().width,
        depth_img.size().height,
        depth_img.size().width * (int)sizeof(uint16_t));

    memcpy(colorImage.get_buffer(),
        color_img.data,
        color_img.size().height * color_img.size().width * 4 * (int)sizeof(uint8_t));

    memcpy(depthImage.get_buffer(),
        depth_img.data,
        depth_img.size().height * depth_img.size().width * (int)sizeof(uint16_t));

    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    if (is_transformed)
    {
        cloud = convertRGBADepthToPointXYZRGBA_transformed(colorImage, depthImage, transformation);
    }
    else
    {
        cloud = convertRGBADepthToPointXYZRGBA(colorImage, depthImage, transformation);
    }

    pcl::io::savePCDFileASCII(save_file_path, *cloud);
}

void frames2pc_v2(const string& color_file_path, const string& depth_file_path, const string& save_file_path, const bool& is_transformed, const bool& is_mkv_frames, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    cv::Mat color_img = cv::imread(color_file_path, cv::IMREAD_UNCHANGED);
    if (color_img.empty() || color_img.depth() != CV_8U)
    {
        std::cerr
            << "WARNING: cannot read color image. No such a file, or the image format is not CV_8U"
            << std::endl;
    }

    //    //if mkv frames, the channels is 3 rather than 4
    //    if(is_mkv_frames)
    //    {
    //        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    //    }
    //    //

        //if the num of channnels is 3, then  set it to 4
    if (color_img.channels() == 3)
    {
        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    }
    //

    cv::Mat depth_img = cv::imread(depth_file_path, cv::IMREAD_ANYDEPTH);
    if (depth_img.empty() || depth_img.depth() != CV_16U)
    {
        std::cerr
            << "WARNING: cannot read depth image. No such a file, or the image format is not CV_16U"
            << std::endl;
    }

    k4a::image colorImage = NULL;
    k4a::image depthImage = NULL;

    colorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_img.size().width,
        color_img.size().height,
        color_img.size().width * 4 * (int)sizeof(uint8_t));

    depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        depth_img.size().width,
        depth_img.size().height,
        depth_img.size().width * (int)sizeof(uint16_t));

    memcpy(colorImage.get_buffer(),
        color_img.data,
        color_img.size().height * color_img.size().width * 4 * (int)sizeof(uint8_t));

    memcpy(depthImage.get_buffer(),
        depth_img.data,
        depth_img.size().height * depth_img.size().width * (int)sizeof(uint16_t));

    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>());
    if (is_transformed)
    {
        cloud = convertRGBADepthToPointXYZRGBA_transformed_v2(colorImage, depthImage, transformation, T_o2a,T_a2b,T_b2c);
    }
    else
    {
        cloud = convertRGBADepthToPointXYZRGBA_v2(colorImage, depthImage, transformation, T_o2a,T_a2b,T_b2c);
    }

    pcl::io::savePCDFileASCII(save_file_path, *cloud);
    //may change the saving file format, such .txt
}

void frames2pc_v3(const string& color_file_path, const string& depth_file_path, const string& label_file_path, const string& save_stem_pc_path, const string& save_fruit_pc_path, const string& save_leaf_pc_path, const string& save_flower_pc_path, const bool& is_transformed, const bool& is_mkv_frames, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    cv::Mat color_img = cv::imread(color_file_path, cv::IMREAD_UNCHANGED);
    if (color_img.empty() || color_img.depth() != CV_8U)
    {
        std::cerr
            << "WARNING: cannot read color image. No such a file, or the image format is not CV_8U"
            << std::endl;
    }

    //    //if mkv frames, the channels is 3 rather than 4
    //    if(is_mkv_frames)
    //    {
    //        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    //    }
    //    //

        //if the num of channnels is 3, then  set it to 4
    if (color_img.channels() == 3)
    {
        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    }
    //

    cv::Mat depth_img = cv::imread(depth_file_path, cv::IMREAD_ANYDEPTH);
    if (depth_img.empty() || depth_img.depth() != CV_16U)
    {
        std::cerr
            << "WARNING: cannot read depth image. No such a file, or the image format is not CV_16U"
            << std::endl;
    }

    cv::Mat label_img = cv::imread(label_file_path, cv::IMREAD_GRAYSCALE);
    if (label_img.empty()) {
        std::cerr 
            << "Error: Could not read the label image." 
            << std::endl;
    }

    k4a::image colorImage = NULL;
    k4a::image depthImage = NULL;

    colorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_img.size().width,
        color_img.size().height,
        color_img.size().width * 4 * (int)sizeof(uint8_t));

    depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        depth_img.size().width,
        depth_img.size().height,
        depth_img.size().width * (int)sizeof(uint16_t));

    memcpy(colorImage.get_buffer(),
        color_img.data,
        color_img.size().height * color_img.size().width * 4 * (int)sizeof(uint8_t));

    memcpy(depthImage.get_buffer(),
        depth_img.data,
        depth_img.size().height * depth_img.size().width * (int)sizeof(uint16_t));

    PointCloud<PointXYZRGBA>::Ptr stem_cloud(new PointCloud<PointXYZRGBA>());
    PointCloud<PointXYZRGBA>::Ptr fruit_cloud(new PointCloud<PointXYZRGBA>());
    PointCloud<PointXYZRGBA>::Ptr leaf_cloud(new PointCloud<PointXYZRGBA>());
    PointCloud<PointXYZRGBA>::Ptr flower_cloud(new PointCloud<PointXYZRGBA>());

    int category_1 = 1;
    int category_2 = 2;
    int category_3 = 3;
    int category_4 = 4;

    if (is_transformed)
    {
        stem_cloud = convertRGBADepthToPointXYZRGBA_transformed_v3(colorImage, depthImage, label_img, category_1, transformation, T_o2a, T_a2b, T_b2c);
        fruit_cloud = convertRGBADepthToPointXYZRGBA_transformed_v3(colorImage, depthImage, label_img, category_2, transformation, T_o2a, T_a2b, T_b2c);
        leaf_cloud = convertRGBADepthToPointXYZRGBA_transformed_v3(colorImage, depthImage, label_img, category_3, transformation, T_o2a, T_a2b, T_b2c);
        flower_cloud = convertRGBADepthToPointXYZRGBA_transformed_v3(colorImage, depthImage, label_img, category_4, transformation, T_o2a, T_a2b, T_b2c);
    }
    else
    {
        stem_cloud = convertRGBADepthToPointXYZRGBA_v3(colorImage, depthImage, label_img, category_1, transformation, T_o2a, T_a2b, T_b2c);
        fruit_cloud = convertRGBADepthToPointXYZRGBA_v3(colorImage, depthImage, label_img, category_2, transformation, T_o2a, T_a2b, T_b2c);
        leaf_cloud = convertRGBADepthToPointXYZRGBA_v3(colorImage, depthImage, label_img, category_3, transformation, T_o2a, T_a2b, T_b2c);
        flower_cloud = convertRGBADepthToPointXYZRGBA_v3(colorImage, depthImage, label_img, category_4, transformation, T_o2a, T_a2b, T_b2c);
    }

    if (!stem_cloud->empty())
    {
        pcl::io::savePCDFileASCII(save_stem_pc_path, *stem_cloud);
    }
    if (!fruit_cloud->empty())
    {
        pcl::io::savePCDFileASCII(save_fruit_pc_path, *fruit_cloud);
    }
    if (!leaf_cloud->empty())
    {
        pcl::io::savePCDFileASCII(save_leaf_pc_path, *leaf_cloud);
    }
    if (!flower_cloud->empty())
    {
        pcl::io::savePCDFileASCII(save_flower_pc_path, *flower_cloud);
    }
    //may change the saving file format, such .txt
}

void frames2pc_v4(const string& color_file_path, const string& depth_file_path, const std::vector<std::string>& ok_ms_dn_file_paths, const string& save_file_path, const string& save_no_0_and_255_file_path, const bool& is_transformed, const bool& is_mkv_frames, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    cv::Mat color_img = cv::imread(color_file_path, cv::IMREAD_UNCHANGED);
    if (color_img.empty() || color_img.depth() != CV_8U)
    {
        std::cerr
            << "WARNING: cannot read color image. No such a file, or the image format is not CV_8U"
            << std::endl;
    }

    //    //if mkv frames, the channels is 3 rather than 4
    //    if(is_mkv_frames)
    //    {
    //        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    //    }
    //    //

        //if the num of channnels is 3, then  set it to 4
    if (color_img.channels() == 3)
    {
        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    }
    //

    cv::Mat depth_img = cv::imread(depth_file_path, cv::IMREAD_ANYDEPTH);
    if (depth_img.empty() || depth_img.depth() != CV_16U)
    {
        std::cerr
            << "WARNING: cannot read depth image. No such a file, or the image format is not CV_16U"
            << std::endl;
    }

    k4a::image colorImage = NULL;
    k4a::image depthImage = NULL;

    colorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_img.size().width,
        color_img.size().height,
        color_img.size().width * 4 * (int)sizeof(uint8_t));

    depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        depth_img.size().width,
        depth_img.size().height,
        depth_img.size().width * (int)sizeof(uint16_t));

    memcpy(colorImage.get_buffer(),
        color_img.data,
        color_img.size().height * color_img.size().width * 4 * (int)sizeof(uint8_t));

    memcpy(depthImage.get_buffer(),
        depth_img.data,
        depth_img.size().height * depth_img.size().width * (int)sizeof(uint16_t));

    // Process ok_ms_dn_file_paths images
    std::vector<k4a::image> ok_ms_dn_images;
    for (const auto& ok_ms_dn_file_path : ok_ms_dn_file_paths)
    {
        cv::Mat ok_ms_dn_img = cv::imread(ok_ms_dn_file_path, cv::IMREAD_UNCHANGED);
        if (ok_ms_dn_img.empty() || ok_ms_dn_img.depth() != CV_8U)
        {
            std::cerr << "WARNING: cannot read image: " << ok_ms_dn_file_path << std::endl;
        }
        else
        {
            // If the number of channels is 3, then set it to 4
            if (ok_ms_dn_img.channels() == 3)
            {
                cv::cvtColor(ok_ms_dn_img, ok_ms_dn_img, cv::COLOR_BGR2BGRA);
            }

            k4a::image msImage = NULL;
            msImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                ok_ms_dn_img.size().width,
                ok_ms_dn_img.size().height,
                ok_ms_dn_img.size().width * 4 * (int)sizeof(uint8_t));

            memcpy(msImage.get_buffer(),
                ok_ms_dn_img.data,
                ok_ms_dn_img.size().height * ok_ms_dn_img.size().width * 4 * (int)sizeof(uint8_t));

            ok_ms_dn_images.push_back(msImage);
        }
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clouds;
    if (is_transformed)
    {
        clouds = convertMSDepthToPointXYZMS_transformed_v4(colorImage, depthImage, ok_ms_dn_images, transformation, T_o2a, T_a2b, T_b2c);
    }
    else
    {
        clouds = convertMSDepthToPointXYZMS_v4(colorImage, depthImage, ok_ms_dn_images, transformation, T_o2a, T_a2b, T_b2c);
    }

    //pcl::io::savePCDFileASCII(save_file_path, *cloud);
    //may change the saving file format, such .txt
    saveCloudsToTXT(clouds, save_file_path);
    saveCloudsToTXT_no_0_and_255(clouds, save_no_0_and_255_file_path);
}

void frames2pc_v5(const string& color_file_path, const string& depth_file_path, const std::vector<std::string>& ok_ms_dn_file_paths, const string& label_file_path, const string& save_dn_pc_stem_txt_file_path, const string& save_dn_no_0_and_255_pc_stem_txt_file_path, const string& save_dn_pc_fruit_txt_file_path, const string& save_dn_no_0_and_255_pc_fruit_txt_file_path, const string& save_dn_pc_leaf_txt_file_path, const string& save_dn_no_0_and_255_pc_leaf_txt_file_path, const string& save_dn_pc_flower_txt_file_path, const string& save_dn_no_0_and_255_pc_flower_txt_file_path, const bool& is_transformed, const bool& is_mkv_frames, k4a::transformation& transformation, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    cv::Mat color_img = cv::imread(color_file_path, cv::IMREAD_UNCHANGED);
    if (color_img.empty() || color_img.depth() != CV_8U)
    {
        std::cerr
            << "WARNING: cannot read color image. No such a file, or the image format is not CV_8U"
            << std::endl;
    }

    //    //if mkv frames, the channels is 3 rather than 4
    //    if(is_mkv_frames)
    //    {
    //        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    //    }
    //    //

        //if the num of channnels is 3, then  set it to 4
    if (color_img.channels() == 3)
    {
        cv::cvtColor(color_img, color_img, COLOR_BGR2BGRA);
    }
    //

    cv::Mat depth_img = cv::imread(depth_file_path, cv::IMREAD_ANYDEPTH);
    if (depth_img.empty() || depth_img.depth() != CV_16U)
    {
        std::cerr
            << "WARNING: cannot read depth image. No such a file, or the image format is not CV_16U"
            << std::endl;
    }

    cv::Mat label_img = cv::imread(label_file_path, cv::IMREAD_GRAYSCALE);
    if (label_img.empty()) {
        std::cerr
            << "Error: Could not read the label image."
            << std::endl;
    }

    k4a::image colorImage = NULL;
    k4a::image depthImage = NULL;

    colorImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_img.size().width,
        color_img.size().height,
        color_img.size().width * 4 * (int)sizeof(uint8_t));

    depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        depth_img.size().width,
        depth_img.size().height,
        depth_img.size().width * (int)sizeof(uint16_t));

    memcpy(colorImage.get_buffer(),
        color_img.data,
        color_img.size().height * color_img.size().width * 4 * (int)sizeof(uint8_t));

    memcpy(depthImage.get_buffer(),
        depth_img.data,
        depth_img.size().height * depth_img.size().width * (int)sizeof(uint16_t));

    // Process ok_ms_dn_file_paths images
    std::vector<k4a::image> ok_ms_dn_images;
    for (const auto& ok_ms_dn_file_path : ok_ms_dn_file_paths)
    {
        cv::Mat ok_ms_dn_img = cv::imread(ok_ms_dn_file_path, cv::IMREAD_UNCHANGED);
        if (ok_ms_dn_img.empty() || ok_ms_dn_img.depth() != CV_8U)
        {
            std::cerr << "WARNING: cannot read image: " << ok_ms_dn_file_path << std::endl;
        }
        else
        {
            // If the number of channels is 3, then set it to 4
            if (ok_ms_dn_img.channels() == 3)
            {
                cv::cvtColor(ok_ms_dn_img, ok_ms_dn_img, cv::COLOR_BGR2BGRA);
            }

            k4a::image msImage = NULL;
            msImage = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                ok_ms_dn_img.size().width,
                ok_ms_dn_img.size().height,
                ok_ms_dn_img.size().width * 4 * (int)sizeof(uint8_t));

            memcpy(msImage.get_buffer(),
                ok_ms_dn_img.data,
                ok_ms_dn_img.size().height * ok_ms_dn_img.size().width * 4 * (int)sizeof(uint8_t));

            ok_ms_dn_images.push_back(msImage);
        }
    }

    int category_1 = 1;
    int category_2 = 2;
    int category_3 = 3;
    int category_4 = 4;

    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> stem_clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> fruit_clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> leaf_clouds;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> flower_clouds;

    if (is_transformed)
    {
        stem_clouds = convertMSDepthToPointXYZMS_transformed_v5(colorImage, depthImage, ok_ms_dn_images,label_img, category_1, transformation, T_o2a, T_a2b, T_b2c);
        fruit_clouds = convertMSDepthToPointXYZMS_transformed_v5(colorImage, depthImage, ok_ms_dn_images, label_img, category_2, transformation, T_o2a, T_a2b, T_b2c);
        leaf_clouds = convertMSDepthToPointXYZMS_transformed_v5(colorImage, depthImage, ok_ms_dn_images, label_img, category_3, transformation, T_o2a, T_a2b, T_b2c);
        flower_clouds = convertMSDepthToPointXYZMS_transformed_v5(colorImage, depthImage, ok_ms_dn_images, label_img, category_4, transformation, T_o2a, T_a2b, T_b2c);
    }
    else
    {
        stem_clouds = convertMSDepthToPointXYZMS_v5(colorImage, depthImage, ok_ms_dn_images, label_img, category_1, transformation, T_o2a, T_a2b, T_b2c);
        fruit_clouds = convertMSDepthToPointXYZMS_v5(colorImage, depthImage, ok_ms_dn_images, label_img, category_2, transformation, T_o2a, T_a2b, T_b2c);
        leaf_clouds = convertMSDepthToPointXYZMS_v5(colorImage, depthImage, ok_ms_dn_images, label_img, category_3, transformation, T_o2a, T_a2b, T_b2c);
        flower_clouds = convertMSDepthToPointXYZMS_v5(colorImage, depthImage, ok_ms_dn_images, label_img, category_4, transformation, T_o2a, T_a2b, T_b2c);
    }

    //pcl::io::savePCDFileASCII(save_file_path, *cloud);
    //may change the saving file format, such .txt
    saveCloudsToTXT(stem_clouds, save_dn_pc_stem_txt_file_path);
    saveCloudsToTXT_no_0_and_255(stem_clouds, save_dn_no_0_and_255_pc_stem_txt_file_path);
    saveCloudsToTXT(fruit_clouds, save_dn_pc_fruit_txt_file_path);
    saveCloudsToTXT_no_0_and_255(fruit_clouds, save_dn_no_0_and_255_pc_fruit_txt_file_path);
    saveCloudsToTXT(leaf_clouds, save_dn_pc_leaf_txt_file_path);
    saveCloudsToTXT_no_0_and_255(leaf_clouds, save_dn_no_0_and_255_pc_leaf_txt_file_path);
    saveCloudsToTXT(flower_clouds, save_dn_pc_flower_txt_file_path);
    saveCloudsToTXT_no_0_and_255(flower_clouds, save_dn_no_0_and_255_pc_flower_txt_file_path);
}

void save_pose_txt_to_each_folder(const string& save_pose_txt_path, const Eigen::Matrix4d& T_o2a, const Eigen::Matrix4d& T_a2b, const Eigen::Matrix4d& T_b2c)
{
    Eigen::Matrix4d T_o2c = (T_o2a * T_a2b) * T_b2c;
    //cout << "Transformation Matrix T_o2c:\n" << T_o2c << endl;
    ofstream file(save_pose_txt_path);
    if (file.is_open())
    {
        file << T_o2c << endl;
        file.close();
    }
    else
    {
        cerr << "无法打开文件：" << save_pose_txt_path << endl;
    }
}

pcl::PointCloud<PointType>::Ptr passThroughPc(pcl::PointCloud<PointType>::Ptr cloud)
{
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
    std::cout << "loading point cloud: " << cloud->points.size() << std::endl;
    pcl::PassThrough<PointType> pass;

    //z  0-60
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.01, 0.01);
    // pass.setKeepOrganized(true);
    pass.setNegative(true);
    pass.filter(*cloud_filtered);

    std::cout << "Cloud after filtering: " << cloud_filtered->points.size() << std::endl;
    return(cloud_filtered);
}

float pointDistance(PointType p1, PointType p2)
{
    float d;
    d = float(sqrt(double((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z))));
    return d;
}

float str2int(string num) {
    float res;
    stringstream stream(num);
    stream >> res;
    return res;
}

void vec3fNorm(Eigen::Vector3f& vec)
{
    float d = sqrt(vec(0) * vec(0) + vec(1) * vec(1) + vec(2) * vec(2));
    vec(0) = vec(0) / d;
    vec(1) = vec(1) / d;
    vec(2) = vec(2) / d;
}

Eigen::Vector3f vec3fCross(Eigen::Vector3f m, Eigen::Vector3f n)
{
    Eigen::Vector3f res_vec;
    res_vec(0) = m(1) * n(2) - m(2) * n(1);
    res_vec(1) = (-1) * (m(0) * n(2) - m(2) * n(0));
    res_vec(2) = m(0) * n(1) - m(1) * n(0);
    return res_vec;
}

void vec3dNorm(Eigen::Vector3d& vec)
{
    double d = sqrt(vec(0) * vec(0) + vec(1) * vec(1) + vec(2) * vec(2));
    vec(0) = vec(0) / d;
    vec(1) = vec(1) / d;
    vec(2) = vec(2) / d;
}

double vectorLength(double rx, double ry, double rz)
{
    return (double)sqrt(rx * rx + ry * ry + rz * rz);
}

Eigen::Vector3d vec3dCross(Eigen::Vector3d m, Eigen::Vector3d n)
{
    Eigen::Vector3d res_vec;
    res_vec(0) = m(1) * n(2) - m(2) * n(1);
    res_vec(1) = (-1) * (m(0) * n(2) - m(2) * n(0));
    res_vec(2) = m(0) * n(1) - m(1) * n(0);
    return res_vec;
}

void saveCloudsToTXT(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& clouds, const std::string& save_file_path)
{
    namespace fs = std::filesystem;

    if (clouds.empty())
    {
        std::cerr << "No clouds provided for saving." << std::endl;
        return;
    }

    // Create directory if it doesn't exist
    fs::path path(save_file_path);
    if (!fs::exists(path.parent_path()))
    {
        if (!fs::create_directories(path.parent_path()))
        {
            std::cerr << "Failed to create directory: " << path.parent_path() << std::endl;
            return;
        }
    }

    // Open the file for writing
    std::ofstream file(save_file_path);
    if (!file.is_open())
    {
        std::cerr << "Could not open file for writing: " << save_file_path << std::endl;
        return;
    }

    // Get the first point cloud (assumed to contain xyz and base rgb values)
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& base_cloud = clouds[0];
    const size_t num_points = base_cloud->size();
    const size_t num_clouds = clouds.size();

    for (size_t i = 0; i < num_points; ++i)
    {
        const pcl::PointXYZRGBA& point = base_cloud->points[i];
        file << point.x << " " << point.y << " " << point.z << " "
            << static_cast<int>(point.r) << " " << static_cast<int>(point.g) << " "
            << static_cast<int>(point.b);

        // Append DN values from other point clouds at the same index
        for (size_t j = 1; j < num_clouds; ++j)
        {
            const pcl::PointXYZRGBA& dn_point = clouds[j]->points[i];
            file << " " << static_cast<int>(dn_point.r); // Using the r channel for DN value
        }
        file << "\n";
    }

    file.close();
    std::cout << "Point cloud saved to: " << save_file_path << std::endl;
}

void saveCloudsToTXT_no_0_and_255(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>& clouds, const std::string& save_file_path)
{
    namespace fs = std::filesystem;

    if (clouds.empty())
    {
        std::cerr << "No clouds provided for saving." << std::endl;
        return;
    }

    // Create directory if it doesn't exist
    fs::path path(save_file_path);
    if (!fs::exists(path.parent_path()))
    {
        if (!fs::create_directories(path.parent_path()))
        {
            std::cerr << "Failed to create directory: " << path.parent_path() << std::endl;
            return;
        }
    }

    // Open the file for writing
    std::ofstream file(save_file_path);
    if (!file.is_open())
    {
        std::cerr << "Could not open file for writing: " << save_file_path << std::endl;
        return;
    }

    // Get the first point cloud (assumed to contain xyz and base rgb values)
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& base_cloud = clouds[0];
    const size_t num_points = base_cloud->size();
    const size_t num_clouds = clouds.size();

    for (size_t i = 0; i < num_points; ++i)
    {
        bool skip_line = false; // Flag to skip writing this line

        // Check DN values from all point clouds at the same index
        for (size_t j = 1; j < num_clouds; ++j)
        {
            const pcl::PointXYZRGBA& dn_point = clouds[j]->points[i];
            if (dn_point.r == 0 || dn_point.r == 255)
            {
                skip_line = true; // Skip this line if any DN value is 0 or 255
                break;
            }
        }

        if (!skip_line)
        {
            const pcl::PointXYZRGBA& point = base_cloud->points[i];
            file << point.x << " " << point.y << " " << point.z << " "
                << static_cast<int>(point.r) << " " << static_cast<int>(point.g) << " "
                << static_cast<int>(point.b);

            // Append DN values from other point clouds at the same index
            for (size_t j = 1; j < num_clouds; ++j)
            {
                const pcl::PointXYZRGBA& dn_point = clouds[j]->points[i];
                file << " " << static_cast<int>(dn_point.r); // Using the r channel for DN value
            }
            file << "\n";
        }
    }

    file.close();
    std::cout << "Point cloud saved to: " << save_file_path << std::endl;
}