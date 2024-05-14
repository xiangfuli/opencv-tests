#include <vector>
#include <random>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm/triangulation.hpp>

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

        cv::Mat res = rot * ptMat;

        

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

int main() {
  std::random_device rd; 
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-20.0, 20.0);

  std::vector<cv::Point3d> points;

  for (int i = 0; i < 400; ++i) {
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

  std::cout << R << std::endl;
  std::cout << -R.t() * t << std::endl;

  // triangulate points
  std::vector<cv::Point2d> cv_un_ps0, cv_un_ps1;
  cv::undistortPoints(pixels[0], cv_un_ps0, camera1KMat, distCoeffs1);
  cv::undistortPoints(pixels[1], cv_un_ps1, camera2KMat, distCoeffs2);
  cv::Mat proj0, proj1;

  cv::Mat c1TMat = (cv::Mat_<double>(3,1) << c1.t.x, c1.t.y, c1.t.z);
  cv::hconcat(c1.rot, c1TMat, proj0);
  cv::hconcat(R, t, proj1);

  std::vector<cv::Mat> projs;
  projs.push_back(proj0);
  projs.push_back(proj1);

  std::vector<cv::Mat> unpoints;
  cv::Mat undisMat1(2, pixels[0].size(), CV_64F), undisMat2(2, pixels[0].size(), CV_64F);
  for (int i=0; i<pixels[0].size(); i++) {
    undisMat1.at<double>(0, i) = pixels[0][i].x;
    undisMat1.at<double>(1, i) = pixels[0][i].y;

    undisMat2.at<double>(0, i) = pixels[1][i].x;
    undisMat2.at<double>(1, i) = pixels[1][i].y;
  }
  unpoints.push_back(undisMat1);
  unpoints.push_back(undisMat2);

  cv::Mat pnts3D(3, pixels[0].size(), CV_64F);
  cv::sfm::triangulatePoints(unpoints, projs, pnts3D);

  std::vector<cv::Point3d> tripoints;
  
  for (int i=0; i<pixels[0].size(); i++) {
    cv::Mat loc = pnts3D.col(i);
    loc = loc / loc.at<double>(3, 0);
    cv::Point3d p(loc.at<double>(0, 0), loc.at<double>(1, 0), loc.at<double>(2, 0));
    tripoints.push_back(p);

    std::cout << p << std::endl;
    std::cout << points[i] << std::endl;
    std::cout << "------------------" << std::endl;
  }

}