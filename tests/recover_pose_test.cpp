#include <vector>
#include <random>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Eigenvalues> 
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp>
struct Camera {
    double cx;
    double cy;

    double fx;
    double fy;

    double k1;
    double k2;
    double k3;

    double p1;
    double p2;

    cv::Mat rot;
    cv::Point3d t;

    cv::Point2d project(cv::Point3d p_world) {
        cv::Point3d p_temp = p_world - t;

        cv::Mat ptMat = (cv::Mat_<double>(3,1) << p_temp.x, p_temp.y, p_temp.z);

        cv::Mat res = rot.t() * ptMat;

        cv::Point3d p_camera;
        p_camera.x = res.at<double>(0, 0);
        p_camera.y = res.at<double>(1, 0);
        p_camera.z = res.at<double>(2, 0);

        cv::Point2d p_cameraPlane;
        p_cameraPlane.x = p_camera.x / p_camera.z;
        p_cameraPlane.y = p_camera.y / p_camera.z;

        cv::Point2d pixel;
        pixel.x = p_cameraPlane.x * fx + cx;
        pixel.y = p_cameraPlane.y * fy + cy;

        return pixel;
    }
};

void hat(Eigen::Matrix<double, 3, 1> vector, Eigen::Matrix<double, 3, 3> &mat) {
  double x = vector.x();
  double y = vector.y();
  double z = vector.z();
  mat << 0, -z, y, 
         z, 0, -x, 
         -y, x, 0;
}

Eigen::Vector3d NormalizePoint(const Eigen::Vector2d& point, const Eigen::Matrix3d& K) {
    return K.inverse() * Eigen::Vector3d(point(0), point(1), 1.0);
}

void linear_triangulation(
  Eigen::Vector2d &keypoint1,
  Eigen::Vector2d &keypoint2,
  Eigen::Matrix3d &R1,
  Eigen::Vector3d &T1,
  Eigen::Matrix3d &R2,
  Eigen::Vector3d &T2,
  Eigen::Vector4d &point_loc,
  Eigen::Matrix3d &K1_matrix,
  Eigen::Matrix3d &K2_matrix
) {
  Eigen::Vector3d p1, p2;
  p1 << keypoint1.x(), keypoint1.y(), 1;
  p2 << keypoint2.x(), keypoint2.y(), 1;

  // Eigen::Matrix3f K_inverse_matrix = K_matrix.inverse();
  Eigen::Matrix3d skew_point1, skew_point2;
  hat(p1, skew_point1);
  hat(p2, skew_point2);
  
  Eigen::Matrix<double, 3, 4> P1, P2;
  Eigen::Matrix<double, 4, 4> P;
  Eigen::Matrix3d R1_T = R1.transpose();
  Eigen::Vector3d T1_new = R1_T * T1;
  P1 << R1_T(0, 0), R1_T(0, 1), R1_T(0, 2), -T1_new.x(),
       R1_T(1, 0), R1_T(1, 1), R1_T(1, 2), -T1_new.y(),
       R1_T(2, 0), R1_T(2, 1), R1_T(2, 2), -T1_new.z();
  P1 = K1_matrix * P1;
  P1 = skew_point1 * P1;
  Eigen::Matrix3d R2_T = R2.transpose();
  Eigen::Vector3d T2_new = R2_T * T2;
  P2 << R2_T(0, 0), R2_T(0, 1), R2_T(0, 2), -T2_new.x(),
       R2_T(1, 0), R2_T(1, 1), R2_T(1, 2), -T2_new.y(),
       R2_T(2, 0), R2_T(2, 1), R2_T(2, 2), -T2_new.z();
  P2 = K2_matrix * P2;
  P2 = skew_point2 * P2;

  P << P1.row(0), P1.row(1), P2.row(0), P2.row(1);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_P(P, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::MatrixXd V = svd_P.matrixV();

  point_loc = V.col(3);

  point_loc /= point_loc(3, 0);
}

int main() {
  std::random_device rd; 
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-20.0, 20.0);

  std::vector<cv::Point3d> points;

  // for (int i = 0; i < 100; ++i) {
  //     cv::Point3d point(i / 8.0f, -i / 3.0f, i / 3.0f);
  //     points.push_back(point);
  // }
  // for (int i = 0; i < 100; ++i) {
  //     cv::Point3d point(i / 3.0f, i / 5.0f, i / 3.0f);
  //     points.push_back(point);
  // }
  // for (int i = 0; i < 100; ++i) {
  //     cv::Point3d point(-i / 3.0f, -i / 30.0f, i / 3.0f);
  //     points.push_back(point);
  // }
  // for (int i = 0; i < 100; ++i) {
  //     cv::Point3d point(-i / 4.0f, -i / 3.0f, i / 3.0f);
  //     points.push_back(point);
  // }
    for (int i = 0; i < 100; ++i) {
        cv::Point3d point(dis(gen), dis(gen), dis(gen));
        points.push_back(point);
    }


  int width = 320;
  int height = 240;
  Camera c1, c2;

  c1.fx = 201;
  c1.fy = 199;
  c1.cx = width / 2 - 1;
  c1.cy = height / 2 + 2;

  c2.fx = 200;
  c2.fy = 198;
  c2.cx = width / 2 - 3;
  c2.cy = height / 2 - 2;

  c1.rot = cv::Mat::eye(3, 3, CV_64F);
  c1.t.z = -40;
  
  c2.rot = cv::Mat::eye(3, 3, CV_64F);
  c2.rot.at<double>(0, 0) = 0.9848077;
  c2.rot.at<double>(0, 1) = 0.1736482;
  c2.rot.at<double>(0, 2) = 0.0000000;
  c2.rot.at<double>(1, 0) = -0.1736482;
  c2.rot.at<double>(1, 1) = 0.9848077;
  c2.rot.at<double>(1, 2) = 0;
  c2.rot.at<double>(2, 0) = 0;
  c2.rot.at<double>(2, 1) = 0;
  c2.rot.at<double>(2, 2) = 1;
  
  c2.t.x = -1;
  c2.t.y = 0;
  c2.t.z = -40;

  std::vector<std::vector<cv::Point2d>> pixels(2);
  for (cv::Point3d point : points) {
    pixels[0].push_back(c1.project(point));
    pixels[1].push_back(c2.project(point));
  }

  std::cout << "Finish generating pixels." << std::endl;

  // prepare the Kmatrix
  cv::Mat camera1KMat(3, 3, CV_64F, cv::Scalar(0.0f)), camera2KMat(3, 3, CV_64F, cv::Scalar(0.0f));
  
  camera1KMat.at<double>(0, 0) = c1.fx;
  camera1KMat.at<double>(1, 1) = c1.fy;
  camera1KMat.at<double>(0, 2) = c1.cx;
  camera1KMat.at<double>(1, 2) = c1.cy;
  camera1KMat.at<double>(2, 2) = 1;

  camera2KMat.at<double>(0, 0) = c2.fx;
  camera2KMat.at<double>(1, 1) = c2.fy;
  camera2KMat.at<double>(0, 2) = c2.cx;
  camera2KMat.at<double>(1, 2) = c2.cy;
  camera2KMat.at<double>(2, 2) = 1;

  std::vector<double> distCoeffs1(4, 0.0), distCoeffs2(4, 0.0);

  cv::Mat E, R, t;
  cv::recoverPose(pixels[0], pixels[1], camera1KMat, distCoeffs1, camera2KMat, distCoeffs2, E, R, t);


  t = -R.t() * t;
  R = R.t();

  std::cout << R << std::endl;
  std::cout << t << std::endl;

  // triangulate points
  std::vector<Eigen::Vector4d> tripoints(pixels[0].size());

  Eigen::Matrix3d eigenKMatrix1, eigenKMatrix2, eigenR1, eigenR2;
  Eigen::Vector3d eigenT1, eigenT2;

  cv::cv2eigen(camera1KMat, eigenKMatrix1);
  cv::cv2eigen(camera2KMat, eigenKMatrix2);
  
  cv::cv2eigen(c1.rot, eigenR1);
  cv::cv2eigen(R, eigenR2);

  eigenT1 << 0, 0, 0;
  eigenT2 << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0);

  for (int i=0; i<pixels[0].size(); i++) {
    Eigen::Vector2d pixel0, pixel1;
    pixel0 << pixels[0][i].x, pixels[0][i].y;
    pixel1 << pixels[1][i].x, pixels[1][i].y;

    linear_triangulation(
      pixel0, pixel1,
      eigenR1, eigenT1,
      eigenR2, eigenT2,
      tripoints[i],
      eigenKMatrix1, eigenKMatrix2);
  }
  
  for (int i=0; i<pixels[0].size(); i++) {
    Eigen::Vector4d loc = tripoints[i];

    std::cout << loc(0, 0) << ", " << loc(1, 0) << ", " << loc(2, 0)  << std::endl;
    std::cout << points[i] - c1.t << std::endl;
    std::cout << "------------------" << std::endl;
  }
}