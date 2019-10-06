#ifndef READ_H
#define READ_H

std::vector<std::vector<double>> GetCorners(const std::string &file);
int GetNumberofImages(const std::string &path, const std::string &type);
void ReadCornersinWCS(const std::string &file, Eigen::MatrixXd &points);
void ReadCameraMatrix(const std::string &file, Eigen::Matrix3d &k);
#endif