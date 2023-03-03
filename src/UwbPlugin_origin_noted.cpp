/*
MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Collision.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include <gazebo/common/common.hh>
#include <gazebo/sensors/Noise.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <gtec_msgs/Ranging.h>
#include <gtec_msgs/Ranging_Vehicle.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <gazebo/rendering/DynamicLines.hh>
#include <tf/transform_datatypes.h>

#include <iostream>

using namespace std;

namespace gazebo
{
    class UwbPlugin : public ModelPlugin
    {

        double rangingStd[141][3] =        //距离数据？
        {
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.513, -12.192,  22.182},
            {31.501, -12.192,  22.182},
            {30.698, -12.191,  22.182},
            {27.075, -12.096,  22.17},
            {29.929, -2.799,  21.951},
            {28.478, 33.958,  23.155},
            {25.672, 56.248,  23.111},
            {23.919, 50.137,  24.211},
            {25.466, 35.927,  22.698},
            {23.742, 41.73,  22.97},
            {22.981, 64.983,  21.461},
            {23.656, 98.312,  23.793},
            {27.169, 110.68,  22.425},
            {27.498, 105.53,  21.579},
            {27.299, 85.8,  22.028},
            {25.972, 93.239,  22.068},
            {23.662, 102.09,  24.255},
            {24.762, 105.34,  23.838},
            {24.073, 115.39,  23.856},
            {22.651, 107.15,  23.96},
            {21.012, 133.86, 23.443},
            {25.325, 102.38,  22.679},
            {25.919, 98.105, 22.753},
            {25.403, 124.86,  23.646},
            {25.696, 175.07,  23.149},
            {23.453, 200.68, 23.195},
            {22.291, 205.53,  23.395},
            {22.217, 187.33,  23.42},
            {22.472, 164.73,  23.167},
            {21.454, 142.57,  23.232},
            {21.642, 130.29,  23.18},
            {22.182, 137.13,  22.787},
            {22.852, 119.3,  22.424},
            {23.71, 132.55, 22.891},
            {23.369, 151.35,  23.018},
            {22.471, 149.82,  20.06},
            {21.729, 147.44,  22.383},
            {21.394, 167.7,  23.749},
            {22.086, 172.37, 24.022},
            {20.258, 173.26,  24.311},
            {20.121, 170.93,  24.36},
            {22.828, 177.02,  24.671},
            {22.247, 193.45,  24.648},
            {22.092, 207.94,  24.564},
            {22.288, 196.53,  23.557},
            {22.287, 157.33,  20.073},
            {22.114, 151.18,  24.099},
            {23.635, 149.36,  24.485},
            {22.904, 141.41, 22.277},
            {21.413, 142.79, 19.144},
            {21.388, 162.89,  21.555},
            {20.659, 171.44, 20.923},
            {20.97, 179.38,  22.282},
            {20.82, 189.58,  21.634},
            {19.158, 197.86,  16.7},
            {18.858, 192.62,  16.12},
            {19.446, 169.91,  16.12},
            {19.774, 160.03,  16.12},
            {19.871, 156.27,  16.12},
            {21.165, 155.08,  16.12},
            {20.888, 152.8,  16.12},
            {20.997, 155.59,  16.12},
            {21.589, 160.88,  16.12},
            {22.173, 164.23,  16.12},
            {20.549, 168.01,  16.12},
            {21.734, 168.27,  16.12},
            {23.783, 160.79,  16.12},
            {24.29, 168.02,  16.12},
            {25.308, 183.71,  16.12},
            {24.18, 194.86,  16.12},
            {22.495, 201.82,  16.12},
            {20.357, 206.62,  16.12},
            {18.665, 218.39, 16.12},
            {18.114, 223.67,  16.12},
            {18.784, 222.92,  16.12},
            {20.542, 222.9,  16.12},
            {17.736, 228.38, 16.12},
            {16.072, 226.38,  16.12},
            {15.945, 231.64,  16.12},
            {16.179, 228.45,  16.12},
            {19.054, 225.98, 16.12},
            {19.604, 225.35,  16.12},
            {19.569, 225.8,  16.12},
            {19.414, 226.88, 16.12},
            {19.44, 225.8,  16.12},
            {20.681, 212.01,  16.12},
            {20.904, 205.07,  16.12},
            {20.855, 204.65,  16.12},
            {19.894, 208.66,  16.12},
            {20.67, 216.24,  16.12},
            {21.353, 219.08,  16.12},
            {21.954, 225.47,  16.12},
            {22.372, 230.28,  16.12},
            {21.988, 231.1,  16.12},
            {21.155, 231.92,  16.12},
            {23.487, 236.02,  16.12},
            {28.752, 239.14,  16.12},
            {34.356, 240.34, 16.12},
            {31.388, 241.54,  16.12},
            {25.201, 241.3, 16.12},
            {20.001, 239.64,  16.12},
            {17.451, 238.92,  16.12},
            {17.839, 244.44,  16.12},
            {17.782, 246.21,  16.12},
            {18.332, 246.51,  16.12},
            {21.499, 243.48,  16.12},
            {22.921, 241.79,  16.12},
            {31.445, 241,  16.12},
            {36.097, 239.04,  16.12},
            {33.022, 235.4,  16.12},
            {19.251, 234.77,  16.12},
            {16.86, 235.51,  16.12},
            {16.385, 237.4, 16.127}
        };


        double rssMean[141][3] =        //接收信号强度的平均值？
        {
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.87, -80.479, -88.205},
            {-78.88, -80.479, -88.205},
            {-79.506, -80.479, -88.205},
            {-82.328, -80.484, -88.205},
            {-80.104, -81.027, -88.207},
            {-81.234, -83.172, -88.209},
            {-83.418, -84.473, -88.209},
            {-84.783, -84.118, -88.21},
            {-83.582, -83.29, -88.211},
            {-84.924, -83.628, -88.212},
            {-85.516, -84.984, -88.211},
            {-84.992, -86.928, -88.21},
            {-82.254, -87.65, -88.21},
            {-81.997, -87.35, -88.211},
            {-82.154, -86.202, -88.211},
            {-83.19, -86.638, -88.212},
            {-84.989, -87.155, -88.212},
            {-84.141, -87.344, -88.213},
            {-84.674, -87.937, -88.213},
            {-85.777, -87.453, -88.213},
            {-87.053, -89.015, -88.215},
            {-83.692, -87.179, -88.218},
            {-83.228, -86.935, -88.223},
            {-83.63, -88.499, -88.238},
            {-83.403, -91.422, -88.278},
            {-85.147, -92.916, -88.288},
            {-86.052, -93.197, -88.376},
            {-86.111, -92.14, -88.385},
            {-85.914, -90.822, -88.407},
            {-86.704, -89.53, -88.48},
            {-86.559, -88.817, -88.537},
            {-86.14, -89.211, -88.536},
            {-85.622, -88.167, -88.578},
            {-84.957, -88.936, -88.717},
            {-85.223, -90.033, -89.85},
            {-85.919, -89.94, -92.143},
            {-86.493, -89.8, -91.798},
            {-86.751, -90.982, -89.445},
            {-86.212, -91.251, -89.327},
            {-87.634, -91.302, -89.909},
            {-87.741, -91.167, -89.869},
            {-85.638, -91.523, -90.272},
            {-86.092, -92.484, -90.161},
            {-86.212, -93.332, -90.502},
            {-86.061, -92.669, -93.029},
            {-86.065, -90.385, -93.384},
            {-86.211, -90.028, -91.776},
            {-85.035, -89.922, -90.725},
            {-85.606, -89.462, -90.974},
            {-86.763, -89.549, -92.572},
            {-86.782, -90.725, -92.237},
            {-87.356, -91.241, -92.515},
            {-87.129, -91.699, -92.342},
            {-87.265, -92.289, -92.127},
            {-88.563, -92.766, -93.68},
            {-88.806, -92.453, -91.294},
            {-88.337, -91.124, -89.336},
            {-88.071, -90.549, -89.039},
            {-87.99, -90.324, -89.615},
            {-87.001, -90.262, -92.496},
            {-87.184, -90.132, -94.345},
            {-87.092, -90.289, -92.867},
            {-86.638, -90.592, -92.249},
            {-86.202, -90.785, -92.691},
            {-87.46, -91.007, -93.252},
            {-86.553, -91.026, -94.637},
            {-84.991, -90.587, -93.508},
            {-84.645, -91.012, -93.939},
            {-83.922, -91.933, -91.488},
            {-84.876, -92.586, -91.667},
            {-86.233, -92.996, -94.21},
            {-87.94, -93.285, -95.191},
            {-89.02, -93.994, -97.303},
            {-89.428, -94.328, -96.513},
            {-88.962, -94.278, -98.077},
            {-87.743, -94.27, -97.932},
            {-89.904, -94.627, -95.275},
            {-91.299, -94.532, -96.202},
            {-91.45, -94.877, -99.615},
            {-91.288, -94.683, -100.46},
            {-89.362, -94.536, -98.965},
            {-88.952, -94.506, -97.864},
            {-89.241, -94.537, -98.488},
            {-89.77, -94.59, -98.489},
            {-90.054, -94.547, -99.341},
            {-90.534, -93.698, -100.08},
            {-89.67, -93.28, -101.08},
            {-89.052, -93.243, -102.03},
            {-89.73, -93.471, -102.45},
            {-91.068, -93.958, -103.27},
            {-90.93, -94.179, -103.62},
            {-90.164, -94.583, -103.59},
            {-88.864, -94.875, -103.87},
            {-88.026, -94.923, -104.27},
            {-90.477, -94.978, -104.42},
            {-92.332, -95.251, -104.45},
            {-93.692, -95.463, -104.55},
            {-94.745, -95.582, -104.56},
            {-94.266, -95.66, -104.63},
            {-93.197, -95.654, -104.6},
            {-92.248, -95.547, -104.62},
            {-91.703, -95.476, -104.61},
            {-91.789, -95.834, -104.62},
            {-91.774, -95.963, -104.72},
            {-91.943, -95.998, -104.74},
            {-92.598, -95.763, -104.74},
            {-92.857, -95.634, -104.74},
            {-94.314, -95.57, -104.74},
            {-95.107, -95.408, -104.74},
            {-94.594, -95.116, -104.74},
            {-92.277, -95.087, -104.74},
            {-91.883, -95.129, -104.74},
            {-91.802, -95.257, -104.74}
        };

        double rssStd[141][3] =             //接受信号强度数据？
        {
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28265, 0.52405, 0.42595},
            {0.28273, 0.52405, 0.42595},
            {0.28802, 0.52405, 0.42595},
            {0.31193, 0.52392, 0.42593},
            {0.29309, 0.51064, 0.42581},
            {0.30265, 0.45871, 0.42568},
            {0.32115, 0.42762, 0.42565},
            {0.33271, 0.43817, 0.42562},
            {0.32256, 0.45904, 0.4256},
            {0.33391, 0.4501, 0.42562},
            {0.33892, 0.41617, 0.4256},
            {0.33449, 0.36839, 0.42557},
            {0.3113, 0.35052, 0.42555},
            {0.30912, 0.35914, 0.42558},
            {0.31046, 0.39002, 0.42557},
            {0.31924, 0.38186, 0.4256},
            {0.33449, 0.36936, 0.42561},
            {0.32735, 0.36462, 0.42561},
            {0.33184, 0.35698, 0.42562},
            {0.34116, 0.36488, 0.42563},
            {0.35196, 0.33029, 0.42565},
            {0.32349, 0.37679, 0.42564},
            {0.31955, 0.38847, 0.42567},
            {0.32295, 0.35443, 0.42579},
            {0.32104, 0.27502, 0.42622},
            {0.33579, 0.23774, 0.42635},
            {0.34345, 0.22965, 0.42749},
            {0.34396, 0.26009, 0.4276},
            {0.3423, 0.29305, 0.42789},
            {0.34898, 0.32595, 0.42884},
            {0.34775, 0.34724, 0.42957},
            {0.34422, 0.33149, 0.42957},
            {0.33985, 0.35423, 0.43011},
            {0.33424, 0.3302, 0.43196},
            {0.3365, 0.30287, 0.44692},
            {0.34237, 0.30164, 0.47716},
            {0.34721, 0.30319, 0.47259},
            {0.34937, 0.27464, 0.44155},
            {0.34481, 0.26435, 0.44},
            {0.35685, 0.26219, 0.4477},
            {0.35775, 0.26625, 0.44715},
            {0.33998, 0.25858, 0.45245},
            {0.34383, 0.23685, 0.451},
            {0.34485, 0.21927, 0.4555},
            {0.34357, 0.23866, 0.48884},
            {0.34362, 0.29779, 0.49354},
            {0.34493, 0.30859, 0.47231},
            {0.33503, 0.31208, 0.45842},
            {0.33986, 0.32803, 0.46171},
            {0.34964, 0.33222, 0.48286},
            {0.34979, 0.30749, 0.47844},
            {0.35469, 0.31324, 0.48211},
            {0.35286, 0.29601, 0.47985},
            {0.35413, 0.27666, 0.47704},
            {0.36514, 0.2577, 0.49755},
            {0.36726, 0.25683, 0.46604},
            {0.36322, 0.28514, 0.4402},
            {0.3609, 0.30088, 0.43625},
            {0.36018, 0.30122, 0.44384},
            {0.35193, 0.30984, 0.48189},
            {0.35328, 0.31716, 0.50633},
            {0.35245, 0.30678, 0.48683},
            {0.34865, 0.29261, 0.4787},
            {0.34507, 0.28513, 0.48469},
            {0.35568, 0.28172, 0.49213},
            {0.34809, 0.28503, 0.51043},
            {0.33508, 0.29321, 0.49557},
            {0.33243, 0.28578, 0.50118},
            {0.32673, 0.26951, 0.46866},
            {0.33526, 0.25688, 0.47096},
            {0.34701, 0.25141, 0.50455},
            {0.36172, 0.25416, 0.51752},
            {0.36944, 0.26108, 0.54557},
            {0.37277, 0.28157, 0.53503},
            {0.36915, 0.27648, 0.55569},
            {0.35973, 0.26925, 0.55374},
            {0.37788, 0.30123, 0.51851},
            {0.39029, 0.32821, 0.53077},
            {0.39187, 0.36244, 0.57602},
            {0.39062, 0.35787, 0.58723},
            {0.37618, 0.35901, 0.56732},
            {0.37281, 0.36604, 0.55273},
            {0.37682, 0.37148, 0.56097},
            {0.38374, 0.35877, 0.56095},
            {0.38796, 0.38246, 0.57229},
            {0.40064, 0.35333, 0.58209},
            {0.38921, 0.34942, 0.59541},
            {0.38007, 0.33631, 0.60813},
            {0.38539, 0.32452, 0.61413},
            {0.40828, 0.36174, 0.62553},
            {0.40946, 0.41779, 0.63079},
            {0.40121, 0.44286, 0.63038},
            {0.3844, 0.44835, 0.63433},
            {0.37052, 0.44675, 0.64188},
            {0.40202, 0.45388, 0.64658},
            {0.43959, 0.48541, 0.6486},
            {0.48364, 0.51261, 0.65211},
            {0.52483, 0.565, 0.65087},
            {0.50416, 0.57133, 0.65671},
            {0.46002, 0.5807, 0.6518},
            {0.42221, 0.57222, 0.65058},
            {0.40252, 0.54167, 0.64913},
            {0.40556, 0.57271, 0.64843},
            {0.40507, 0.59777, 0.65558},
            {0.41007, 0.61706, 0.65859},
            {0.43421, 0.55792, 0.6586},
            {0.44454, 0.52705, 0.6586},
            {0.5051, 0.50872, 0.6586},
            {0.53813, 0.45956, 0.6586},
            {0.51647, 0.37797, 0.6586},
            {0.41915, 0.38748, 0.6586},
            {0.40237, 0.38546, 0.6586},
            {0.39899, 0.40199, 0.6586}
        };

        double minPower[3] = {-94.5, -94.5, -94.5};         //LOS、NLOS_H和NLOS_S三种情况下的最小接收信号强度
        double rangingOffset[141][2] =                      //距离补偿参数
        {
            {-87.505, 228.73},
            {-86.505, 229.73},
            {-85.505, 230.73},
            {-84.505, 231.73},
            {-83.505, 232.73},
            {-82.505, 233.73},
            {-81.505, 234.73},
            {-80.505, 235.73},
            {-79.505, 236.73},
            {-78.505, 237.73},
            {-77.505, 238.73},
            {-76.505, 239.73},
            {-75.505, 240.73},
            {-74.505, 241.73},
            {-73.505, 242.73},
            {-72.505, 243.73},
            {-71.505, 244.73},
            {-70.505, 245.73},
            {-69.505, 246.73},
            {-68.505, 247.73},
            {-67.505, 248.73},
            {-66.505, 249.73},
            {-65.505, 250.73},
            {-64.505, 251.73},
            {-63.505, 252.73},
            {-62.505, 253.73},
            {-61.505, 254.73},
            {-60.505, 255.73},
            {-59.505, 256.73},
            {-58.505, 257.73},
            {-57.505, 258.73},
            {-56.505, 259.73},
            {-171.73, 159.43},
            {19.236, 37.667},
            {-59.397, -69.571},
            {-88.819, 55.978},
            {-87.637, 126.88},
            {-63.632, 218.34},
            {-166.5, 243.98},
            {2.9297, -133.46},
            {24.966, -7.3586},
            {-75.633, 179.6},
            {-275.63, 177.37},
            {-6.0524, 35.31},
            {-42.925, 2.8818},
            {-91.347, 64.352},
            {-45.604, -11.544},
            {-80.352, -163.65},
            {-70.501, -122.94},
            {-44.816, 47.885},
            {-62.616, 23.613},
            {-120.1, 96.477},
            {-24.175, -54.87},
            {-31.297, 52.603},
            {-68.891, 139.32},
            {-134.92, -112.3},
            {-82.555, 65.72},
            {-99.757, -115.76},
            {-91.724, -272.29},
            {-98.396, 104.96},
            {-135.68, 22.346},
            {-69.928, -142.3},
            {-54.738, -169.96},
            {-65.137, 108.48},
            {-159.24, -85.515},
            {-59.853, 127.5},
            {-37.471, -123.05},
            {-25.802, 25.88},
            {-26.242, -68.14},
            {-169.91, -187.94},
            {-146.91, -169},
            {-194.92, 200.43},
            {-117.44, 160.1},
            {48.046, 62.7},
            {80.894, -50.68},
            {-190.6, -75.238},
            {-360.33, -64.185},
            {-196.96, 0.028312},
            {-67.649, -12.525},
            {40.599, -64.741},
            {3.6309, -128.47},
            {-68.189, -63.786},
            {56.098, -296.78},
            {-44.121, -68.179},
            {-180.8, -82.837},
            {-253.97, 16.43},
            {-76.724, -76.724},
            {63.682, 63.682},
            {208.32, 208.32},
            {147.39, 147.39},
            {-148.67, -148.67},
            {-24.853, -24.853},
            {-15.417, -15.417},
            {-11.058, -11.058},
            {-58.758, -58.758},
            {-275.52, -275.52},
            {114.79, 114.79},
            {177.85, 177.85},
            {137.17, 137.17},
            {76.573, 76.573},
            {39.708, 39.708},
            {-39.521, -39.521},
            {-58.715, -58.715},
            {17.466, 17.466},
            {91.571, 91.571},
            {-14.972, -14.972},
            {-74.248, -74.248},
            {-104.1, -104.1},
            {-188.11, -188.11},
            {-258.89, -258.89},
            {-306.62, -306.62},
            {-257.13, -257.13},
            {-85.931, -85.931},
            {3.3227, 3.3227},
            {59.19, 59.19},
            {3.4495, 3.4495},
            {-29.723, -29.723},
            {9.6787, 9.6787},
            {23.18, 23.18},
            {90.738, 90.738},
            {89.798, 89.798},
            {-14.135, -14.135},
            {-87.788, -87.788},
            {-136.37, -136.37},
            {-174.65, -174.65},
            {-214.81, -214.81},
            {-208.3, -208.3},
            {-218.2, -218.2},
            {-110.18, -110.18},
            {10.234, 10.234},
            {16.536, 16.536},
            {-156.6, -156.6},
            {-58.566, -58.566},
            {69.13, 69.13},
            {136.01, 136.01},
            {-72.069, -72.069},
            {-114.48, -114.48},
            {-89.969, -89.969},
            {-75.265, -75.265},
            {-107.16, -107.16},
            {-224.83, -224.83}
        };


        enum LOSType        //枚举类型LOSType
        {
            LOS,            //0,信号没有阻挡
            NLOS,           //1,无信号，距离太远或者完全阻挡
            NLOS_S,         //2,有信号，有较薄的障碍物，收到信号强度减弱
            NLOS_H          //3，有微弱信号，障碍物太多，信号无法直接传递，但能收到反弹后的信号，比实际的距离更远
        };

    public:
        UwbPlugin() :
            ModelPlugin(),
            sequence(0)
        {
            this->updatePeriod = common::Time(0.0);     //更新updatePeriod时间
        }

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            if (!ros::isInitialized())              //检查ros初始化
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                 << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            if (!_sdf->HasElement("update_rate"))   //检查是否设置update_rate参数
            {
                ROS_FATAL_STREAM("GTEC UWB Plugin needs the parameter: update_rate");
            }

            this->model = _parent;                                  //模型
            this->world = _parent->GetWorld();                      //世界
            this->SetUpdateRate(_sdf->Get<double>("update_rate"));  //上传频率
            this->nlosSoftWallWidth = 0.25;                         //nlos-s情况的墙壁临界阈值
            this->tagZOffset = 0;                                   //标签z方向上的偏置

            //2023.2.21
            this->tagXOffset = 0;                
            this->tagYOffset = 0;                

            this->tagId = 0;                                        //标签的ID
            this->maxDBDistance = 14;                               //DB(基于数据？)的最大距离值
            this->stepDBDistance = 0.1;                             //DB(基于数据？)的步长距离值
            this->allBeaconsAreLOS = false;                         //是否所有信标都是LOS状态
            this->useParentAsReference = false;                     //是否把parnet作为pose的reference

            this->sequence = 0;

            //2023.2.17
            this->wallWidth  = 0;
            this->MaxPosNoise = 300;
            this->MinPosNoise = 100;
            this->MaxRSS = -80.0;
            this->MinRSS  = -95.0;

            this->MaxDisDecay = 3.0;

            //2023.2.21
            this->NoObstacleRadius =0.236;                                        //忽略障碍物的半径

            if (_sdf->HasElement("all_los"))    //检查是否设置all_los参数
            {
                this->allBeaconsAreLOS = _sdf->Get<bool>("all_los");
            }

            if (_sdf->HasElement("tag_id"))     //检查是否设置tag_id参数
            {
                this->tagId = _sdf->Get<double>("tag_id");
            }

            if (_sdf->HasElement("tag_z_offset"))   //检查是否设置tag_z_offset参数
            {
                this->tagZOffset = _sdf->Get<double>("tag_z_offset");
            }

            //2023.2.21
            if (_sdf->HasElement("tag_x_offset"))   //检查是否设置tag_z_offset参数
            {
                this->tagXOffset = _sdf->Get<double>("tag_x_offset");
            }
            if (_sdf->HasElement("tag_y_offset"))   //检查是否设置tag_z_offset参数
            {
                this->tagYOffset = _sdf->Get<double>("tag_y_offset");
            }

            if (_sdf->HasElement("nlosSoftWallWidth"))  //检查是否设置nlosSoftWallwidth参数
            {
                this->nlosSoftWallWidth = _sdf->Get<double>("nlosSoftWallWidth");
            }

            if (_sdf->HasElement("tag_link"))       //检查是否设置tag_link参数
            {
                std::string tag_link = _sdf->Get<std::string>("tag_link");      //tag_link赋值为参数tag_link的值
                this->tagLink = _parent->GetLink(tag_link);                     //tagLink赋值为parent中的tag_link

                ROS_INFO("Parent name: %s ChildCount: %d", _parent->GetName().c_str(), _parent->GetChildCount());
                if (this->tagLink == NULL)          //若tagLink为空，就把Parent作为reference
                {
                    std::vector<physics::LinkPtr> links = _parent->GetLinks();
                    for (int i = 0; i < links.size(); ++i)
                    {
                        ROS_INFO("Link[%d]: %s", i, links[i]->GetName().c_str());
                    }
                    ROS_INFO("UWB Plugin Tag link Is NULL We use The Parent As Reference");
                    this->useParentAsReference = true;
                }
            }

            if (_sdf->HasElement("anchor_prefix"))  //检查是否设置anchor_prefix参数,没有设置的话就用默认的"uwb_anchor"
            {
                this->anchorPrefix = _sdf->Get<std::string>("anchor_prefix");
            }
            else
            {
                this->anchorPrefix = "uwb_anchor";
            }

            //2023.2.18
            if (_sdf->HasElement("vehicle_prefix"))  //检查是否设置vehicle_prefix参数,没有设置的话就用默认的"iris"
            {
                this->VehiclePrefix = _sdf->Get<std::string>("vehicle_prefix");
            }
            else
            {
                this->VehiclePrefix = "iris";
                 ROS_INFO("No VehiclePrefix Data,  default: iris");
            }

            //2023.2.15   新添最远距离参数maxDBDistance
            if (_sdf->HasElement("maxDBDistance"))     //检查是否设置maxDBDistance参数
            {
                this->maxDBDistance = _sdf->Get<double>("maxDBDistance");
            }

            //2023.2.17
            if (_sdf->HasElement("MaxPosNoise"))     //检查是否设置MaxPosNoise参数
            {
                this->MaxPosNoise = _sdf->Get<int>("MaxPosNoise");
            }
            if (_sdf->HasElement("MinPosNoise"))     //检查是否设置MinPosNoise参数
            {
                this->MinPosNoise = _sdf->Get<int>("MinPosNoise");
            }
            if (_sdf->HasElement("MaxRSS"))     //检查是否设置MaxRSS参数
            {
                this->MaxRSS = _sdf->Get<float>("MaxRSS");
            }
            if (_sdf->HasElement("MinRSS"))     //检查是否设置MinRSS参数
            {
                this->MinRSS = _sdf->Get<float>("MinRSS");
            }
            if (_sdf->HasElement("MaxDisDecay"))     //检查是否设置MaxDisDecay参数
            {
                this->MaxDisDecay = _sdf->Get<float>("MaxDisDecay");
            }
            


            if (_sdf->HasElement("vehicle_prefix"))  //检查是否设置vehicle_prefix参数,没有设置的话就用默认的"iris"
            {
                this->VehiclePrefix = _sdf->Get<std::string>("vehicle_prefix");
            }
            else
            {
                this->VehiclePrefix = "iris";
                 ROS_INFO("No VehiclePrefix Data,  default: iris");
            }

            ROS_INFO("GTEC UWB Plugin is running. Tag %d", this->tagId);        //打印说明，输出tagID

            //2023.2.20


            std::string topicRanging = "/gtec/toa/ranging";         //设置测距ranging发布的话题名称
            std::string topicRangingVehicle = "/gtec/toa/ranging_vehicle";         //设置测距ranging_vehicle发布的话题名称

            //2023.2.21
            if (_sdf->HasElement("NoObstacleRadius"))  //检查是否设置PubRangeTopic参数
            {
                this->NoObstacleRadius = _sdf->Get<double>("NoObstacleRadius");
                ROS_INFO("UWB  Ranging NoObstacleRadius: %f", NoObstacleRadius);
            }
            else
            {
                this->NoObstacleRadius = 0.236;
                 ROS_INFO("No UWB  Ranging NoObstacleRadius \n  Default:  0.236");
            }


            if (_sdf->HasElement("PubRangeTopic"))  //检查是否设置PubRangeTopic参数
            {
                this->PubRangeTopic = _sdf->Get<bool>("PubRangeTopic");
                ROS_INFO("GTEC UWB Plugin Ranging Publishing in %s", topicRanging.c_str());
            }
            else
            {
                this->PubRangeTopic =  false;
                 ROS_INFO("PubRangeTopic:   False\n  Topic of UWB distance between anchor and tag won't be published");
            }

            if (_sdf->HasElement("PubRangeVehicleTopic"))  //检查是否设置PubRangeVehicleTopic参数
            {
                this->PubRangeVehicleTopic = _sdf->Get<bool>("PubRangeVehicleTopic");
                ROS_INFO("GTEC UWB Plugin Ranging_Vehicle Publishing in %s", topicRangingVehicle.c_str());
            }
            else
            {
                this->PubRangeVehicleTopic =  false;
                 ROS_INFO("PubRangeVehicleTopic:   False\n  Topic of UWB distance between vehicles won't be published");
            }




            ROS_INFO("GTEC UWB Plugin All parameters loaded");                  //所有参数加载完毕

            this->lastUpdateTime = common::Time(0.0);               //设置lastUpdateTime时间

            /*std::string topicRanging = "/gtec/toa/ranging";         //设置测距ranging发布的话题名称

            ROS_INFO("GTEC UWB Plugin Ranging Publishing in %s", topicRanging.c_str());

            std::string topicRangingVehicle = "/gtec/toa/ranging_vehicle";         //设置测距ranging_vehicle发布的话题名称

            ROS_INFO("GTEC UWB Plugin Ranging_Vehicle Publishing in %s", topicRangingVehicle.c_str());*/

            /* stringStream.str("");
            stringStream.clear();
            stringStream << "/gtec/toa/anchors" << this->tagId;*/
            //std::string topicAnchors = "/gtec/toa/anchors";     //设置锚点anchor发布的话题名称

            //ROS_INFO("GTEC UWB Plugin Anchors Position Publishing in %s", topicAnchors.c_str());
            if( PubRangeTopic || PubRangeVehicleTopic)
            {

           
                    ros::NodeHandle n;      //创建ROS节点句柄
                    if( PubRangeTopic) this->gtecUwbPub = n.advertise<gtec_msgs::Ranging>(topicRanging, 1000);                 //创建话题topicRanging的发布者，话题类型为gtec_msgs:Ranging
                    //2023.2.18
                    if( PubRangeVehicleTopic) this->gtecUwbPubVehicle = n.advertise<gtec_msgs::Ranging_Vehicle>(topicRangingVehicle, 1000);                       //创建话题topicRangingVehicle的发布者，话题类型为gtec_msgs:Ranging_Vehicle


                //this->gtecAnchors = n.advertise<visualization_msgs::MarkerArray>(topicAnchors, 1000);   //创建话题topicAnchors的发布者，话题类型为visualization_msgs::MarkerArray

                    this->firstRay = boost::dynamic_pointer_cast<physics::RayShape>(
                                        this->world->Physics()->CreateShape("ray", physics::CollisionPtr()));      //指定firstRay为世界物理中的ray类型

                    this->secondRay = boost::dynamic_pointer_cast<physics::RayShape>(
                                        this->world->Physics()->CreateShape("ray", physics::CollisionPtr()));     //指定secondRay为世界物理中的ray类型

                    this->updateConnection =
                        event::Events::ConnectWorldUpdateBegin(boost::bind(&UwbPlugin::OnUpdate, this, _1));        //开始更新，运行OnUpdate


             }
        }

    public:
        void OnUpdate(const common::UpdateInfo &_info)
        {
            common::Time simTime = _info.simTime;                       //更新系统时间simTime
            common::Time elapsed = simTime - this->lastUpdateTime;      //系统时间simTime与上次更新的时间lastUpdateTime做差，得到距离上次更新的时间elapsed
            if (elapsed >= this->updatePeriod)                          //距离上次更新的时间elapsed大于阈值updatePeriod时，才进行数据更新
            {
                this->lastUpdateTime = _info.simTime;


                ignition::math::Pose3d tagPose;             //定义标签的3维位姿tagPose

                if (!this->useParentAsReference)            //判断useParentAsReference，给tagPose进行赋值
                {
                    tagPose = this->tagLink->WorldPose();   //是，用tagLink作为pose的reference
                }
                else
                {
                    tagPose = this->model->WorldPose();     //否，用model作为pose的reference
                }

                ignition::math::Vector3d posCorrected(tagPose.Pos().X()+ this->tagXOffset, tagPose.Pos().Y()+ this->tagYOffset, tagPose.Pos().Z() + this->tagZOffset);     //定义3维向量posCorrected,其值为tagPose的xyz,同时加上标签的方向偏置值tagOffset(XYZ)
                tagPose.Set(posCorrected, tagPose.Rot());      //利用修正z后的位置posCorrectedZ和原有的姿态tagPose.Rot，对tagPose的位姿进行重设
                ignition::math::Vector3d currentTagPose(tagPose.Pos());     //把tagPose的位置tagPose.Pos()赋值给新的3维向量currentTagPose

                tf::Quaternion q(tagPose.Rot().X(),         //tagPose姿态的四元数转为欧拉角roll,pitch,currentYaw
                                 tagPose.Rot().Y(),
                                 tagPose.Rot().Z(),
                                 tagPose.Rot().W());

                tf::Matrix3x3 m(q);
                double roll, pitch, currentYaw;
                m.getRPY(roll, pitch, currentYaw);

                // if (currentYaw < 0)
                // {
                //     currentYaw = 2 * M_PI + currentYaw;
                // }

                double startAngle = currentYaw;     //开始的角度startAngle设为currentYaw
                double currentAngle = 0;            //现在的角度    0
                double arc = 3 * M_PI / 2;          //1 arc 对应 270°
                int numAnglesToTestBySide = 30;     //一侧侧需要测试角度的总数    30
                double incrementAngle = arc / numAnglesToTestBySide;            //角度的增量  这里为270°/30 = 9°
                int totalNumberAnglesToTest = 1 + 2 * numAnglesToTestBySide;    //总共需要测试的角度的总数(当前+两侧) 1 + 2 * 30 = 61
                double anglesToTest[totalNumberAnglesToTest];                   //定义需要测试角度anglesToTest数组

                anglesToTest[0] = startAngle;                           //数组angleToTest第一个存放开始的角度startAngle(currentYaw)
                for (int i = 1; i < totalNumberAnglesToTest; ++i)       //遍历每一个需要测试的角度
                {
                    double angleToTest;                                                 //对测试的角度进行编码
                    if (i % 2 == 0)                                                     //   …   5    3    1      0       2   4   6  …          正方向奇数，负方向偶数，原点为0
                    {                                                                   //+ <--------------  startAngle  ---------------- -
                        angleToTest = startAngle - (i / 2) * incrementAngle;
                        // if (angleToTest < 0)
                        // {
                        //     angleToTest = 2 * M_PI + angleToTest;
                        // }
                    }
                    else
                    {
                        angleToTest = startAngle + (i - (i - 1) / 2) * incrementAngle;
                        // if (angleToTest > 2 * M_PI)
                        // {
                        //     angleToTest = angleToTest - 2 * M_PI;
                        // }
                    }
                    anglesToTest[i] = angleToTest;
                }

                visualization_msgs::MarkerArray markerArray;                //可视化Marker阵列  MarkerArray
                visualization_msgs::MarkerArray interferencesArray;         //可视化Marker阵列  interferencesArray(干扰阵列?)

                physics::Model_V models = this->world->Models();            //获取模型们models
                for (physics::Model_V::iterator iter = models.begin(); iter != models.end(); ++iter)  //遍历world中的每一个模型
                {
                       // cout<<((*iter)->GetName())<<endl;
                    if (   (  (*iter)->GetName().find(this->anchorPrefix) == 0  ) && PubRangeTopic ) //找到带有anchorPrefix前缀命名的模型同时PubRangeTopic为true
                    {
                        physics::ModelPtr anchor = *iter;           //得到锚点anchor
                        //std::string aidStr = anchor->GetName().substr(anchor->GetName().length()-1);
                        std::string aidStr = anchor->GetName().substr(this->anchorPrefix.length());

                        //cout<<((*iter)->GetName())<<endl;
                        //cout<<aidStr<<endl;
                        //ROS_INFO("%s\n", &anchor);
                        //ROS_INFO("%c\n", &aidStr);
                        int aid = std::stoi(aidStr);
                        ignition::math::Pose3d anchorPose = anchor->WorldPose();    //锚点的3维位姿anchoPose

                        LOSType losType = LOS;     //设定lostType为LOS(0)
                        double distance = tagPose.Pos().Distance(anchorPose.Pos());     //得到标签与锚点之间的距离distance
                        //double distanceAfterRebounds = 0;               //定义反射后的距离 0

                        if (!allBeaconsAreLOS)              //所有的信标不全是LOS的情况
                        {
                            //We check if a ray can reach the anchor:
                            double distanceToObstacleFromTag;   //从标签到障碍物的距离
                            std::string obstacleName;           //障碍物名称

                            ignition::math::Vector3d directionToAnchor = (anchorPose.Pos() - tagPose.Pos()).Normalize();    //归一化后的标签到锚点的方向向量

                            


                            ignition::math::Vector3d tagPosOffset(tagPose.Pos().X() + directionToAnchor.X()* this->NoObstacleRadius, 
                                                                                                            tagPose.Pos().Y() +  directionToAnchor.Y()* this->NoObstacleRadius ,
                                                                                                            tagPose.Pos().Z() + directionToAnchor.Z() * this->NoObstacleRadius);     
                            tagPose.Set(tagPosOffset, tagPose.Rot());      //修正用于检测障碍物的tag位置

                            ignition::math::Vector3d anchorPosOffset(anchorPose.Pos().X() - directionToAnchor.X()* this->NoObstacleRadius, 
                                                                                                            anchorPose.Pos().Y() -  directionToAnchor.Y()* this->NoObstacleRadius ,
                                                                                                            anchorPose.Pos().Z() - directionToAnchor.Z() * this->NoObstacleRadius);     
                            anchorPose.Set(anchorPosOffset, anchorPose.Rot());      //修正用于检测障碍物的anchor位置


                            this->firstRay->Reset();                                        //重置firstRay
                            this->firstRay->SetPoints(tagPose.Pos(), anchorPose.Pos());     //设firstRay为tagPose到anchorPose
                            this->firstRay->GetIntersection(distanceToObstacleFromTag, obstacleName);   //从firstRay，得到标签到锚点途中，障碍物的信息，包括标签到障碍物的距离，障碍物的名称

                            if (obstacleName.compare("") == 0)      //障碍物名称为空，即无障碍物，为LOS情况
                            {
                                //There is no obstacle between anchor and tag, we use the LOS model
                                losType = LOS;                      //losType为LOS
                                 this->RSSI = this->MaxRSS;
                                //distanceAfterRebounds = distance;   //反射距离为直接距离
                            }
                            else                                    //存在障碍物,分情况讨论
                            {

                                //We use a second ray to measure the distance from anchor to tag, so we can
                                //know what is the width of the walls
                                double distanceToObstacleFromAnchor;    //锚点到障碍物的距离
                                std::string otherObstacleName;          //其他障碍物的名称

                                this->secondRay->Reset();               //重置secondRay
                                this->secondRay->SetPoints(anchorPose.Pos(), tagPose.Pos());    //设secondRay为anchorPose到tagPose
                                this->secondRay->GetIntersection(distanceToObstacleFromAnchor, otherObstacleName);      //从secondRay，得到锚点到标签途中，障碍物的信息，包括锚点到障碍物的距离，障碍物的名称

                                this->wallWidth = distance - distanceToObstacleFromTag - distanceToObstacleFromAnchor;     //得到障碍物的厚度wallWidth
                                if (this->wallWidth <= this->nlosSoftWallWidth )   //障碍物厚度小于阈值，为NLOS_S
                                {
                                    //We use NLOS - SOFT model
                                    losType = NLOS_S;                   //losType为NLOS_S
                                    this->RSSI = this->MaxRSS - (this->MaxRSS - this->MinRSS) * this->wallWidth / this->nlosSoftWallWidth;  //根据障碍物厚度得到当前RSSI值
                                }
                                else        //障碍物厚度大于阈值
                                {
                                    losType =NLOS ;
                                
                                
                                    
                                    /*
                                    //We try to find a rebound to reach the anchor from the tag
                                    bool end = false;           //尝试找一个从标签到锚点的反射路径

                                    double maxDistance = 30;            //最远距离
                                    double distanceToRebound = 0;       //到反射面的距离
                                    double distanceToFinalObstacle = 0; //到最终障碍物的距离
                                    double distanceNlosHard = 0;        //NLOS-H的距离

                                    double stepFloor = 1;                   //层数步长？
                                    double startFloorDistanceCheck = 2;     //最初层数的检测距离？
                                    int numStepsFloor = 6;                  //总层数？


                                    std::string finalObstacleName;          //最终障碍物的名称
                                    int indexRay = 0;                       //Ray的编号，用来索引之前的anglesToTest数组
                                    bool foundNlosH = false;                //flag,是否找到NLOS

                                    int currentFloorDistance = 0;           //当前层数的距离

                                    while (!end)
                                    {

                                        currentAngle = anglesToTest[indexRay];


                                        double x = currentTagPose.X() + maxDistance * cos(currentAngle);
                                        double y = currentTagPose.Y() + maxDistance * sin(currentAngle);
                                        double z = currentTagPose.Z();

                                        if (currentFloorDistance>0){
                                          double tanAngleFloor = (startFloorDistanceCheck + stepFloor*(currentFloorDistance-1))/currentTagPose.Z();
                                          double angleFloor = atan(tanAngleFloor);

                                          double h = sin(angleFloor)*maxDistance;

                                          double horizontalDistance = sqrt(maxDistance*maxDistance - h*h);

                                          x = currentTagPose.X() + horizontalDistance * cos(currentAngle);
                                          y = currentTagPose.Y() + horizontalDistance * sin(currentAngle);

                                          z = -1*(h - currentTagPose.Z());

                                        }

                                        ignition::math::Vector3d rayPoint(x, y, z);

                                        this->firstRay->Reset();
                                        this->firstRay->SetPoints(currentTagPose, rayPoint);
                                        this->firstRay->GetIntersection(distanceToRebound, obstacleName);

                                        if (obstacleName.compare("") != 0)
                                        {
                                            ignition::math::Vector3d collisionPoint(currentTagPose.X() + distanceToRebound * cos(currentAngle), currentTagPose.Y() + distanceToRebound * sin(currentAngle), currentTagPose.Z());

                                            if (currentFloorDistance>0){
                                                //if (obstacleName.compare("FloorStatic)") == 0){
                                                  // ROS_INFO("TOUCHED GROUND %s - Z: %f", obstacleName.c_str(), z);   
                                               //}
                                                
                                                collisionPoint.Set(currentTagPose.X() + distanceToRebound * cos(currentAngle), currentTagPose.Y() + distanceToRebound * sin(currentAngle), 0.0);
                                            }

                                            //We try to reach the anchor from here
                                            this->secondRay->Reset();
                                            this->secondRay->SetPoints(collisionPoint, anchorPose.Pos());
                                            this->secondRay->GetIntersection(distanceToFinalObstacle, finalObstacleName);

                                            if (finalObstacleName.compare("") == 0)
                                            {



                                                //We reach the anchor after one rebound
                                                distanceToFinalObstacle = anchorPose.Pos().Distance(collisionPoint);

                                                if (currentFloorDistance>0 ){
                                                      //ROS_INFO("Rebound in GROUND %s - Distance: %f", obstacleName.c_str(), distanceToFinalObstacle);   
                                                }


                                                if (distanceToRebound + distanceToFinalObstacle <= maxDBDistance)
                                                {
                                                    foundNlosH = true;
                                                    //We try to find the shortest rebound
                                                    if (distanceNlosHard < 0.1)
                                                    {
                                                        distanceNlosHard = distanceToRebound + distanceToFinalObstacle;
                                                    }
                                                    else if (distanceNlosHard > distanceToRebound + distanceToFinalObstacle)
                                                    {
                                                        distanceNlosHard = distanceToRebound + distanceToFinalObstacle;
                                                    }
                                                }
                                            }
                                        }

                                        if (indexRay < totalNumberAnglesToTest - 1)
                                        {
                                            indexRay += 1;
                                        }
                                        else
                                        {
                                          if (currentFloorDistance<numStepsFloor){

                                            currentFloorDistance+=1;
                                            indexRay= 0;

                                          } else {
                                              end = true;  
                                          }

                                        }
                                    }

                                    if (foundNlosH)
                                    {
                                        //We use the NLOS Hard Model with distance = distanceNlosHard
                                        losType = NLOS_H;
                                        distanceAfterRebounds = distanceNlosHard;
                                    }
                                    else
                                    {
                                        //We can not reach the anchor, no ranging.
                                        losType = NLOS;
                                    }*/
                                }
                            }

                        }
                        else        //所有信标都是LOS状态，则losType为LOS，反射距离等于直接距离
                        {
                            //All beacons are LOS
                            losType = LOS;
                            this->RSSI = this->MaxRSS;
                        }

                        if ((losType == LOS || losType == NLOS_S) && distance > maxDBDistance)  //对于LOS和NLOS_S情况，当距离大于maxDBDistance时，则变为NLOS情况
                        {
                            losType = NLOS;
                        }

                        /*if (losType == NLOS_H && distance > maxDBDistance)     //NLOS_H情况同上
                        {
                            losType = NLOS;
                        }*/

                        if (losType != NLOS)        //对于非NLOS情况，即LOS,NLOS_H,NLOS_S
                        {

                            /*int indexScenario = 0;      //方案序号，LOS为0，NLOS_H为1，NLOS_S为2
                            if (losType == NLOS_S)
                            {
                                indexScenario = 2;
                            }
                            else if (losType == NLOS_H)
                            {
                                indexScenario = 1;
                            }
                                                            //round()函数：返回四舍五入后的值
                            int indexRangingOffset = (int) round(distanceAfterRebounds / stepDBDistance);       //在RangingOffset中的序号，按相应比值进行四舍五入获得

                            double distanceAfterReboundsWithOffset = distanceAfterRebounds;                 //根据LOS与NLOS_S情况，(基于测试得到的数据rangingOffset?)，对测距的偏置进行补偿
                            if (losType == LOS)
                            {
                                distanceAfterReboundsWithOffset = distanceAfterRebounds + rangingOffset[indexRangingOffset][0] / 1000.0;
                            }
                            else if (losType == NLOS_S)
                            {
                                distanceAfterReboundsWithOffset = distanceAfterRebounds + rangingOffset[indexRangingOffset][1] / 1000.0;
                            }

                            int indexRanging = (int) round(distanceAfterReboundsWithOffset / stepDBDistance);   //补偿后的反射距离于步长基于数据(?)长度的比值，得到测距的序号*/

                            //normal_distribution:正态分布


                            double DisDeacayRss = this->MaxDisDecay * distance / maxDBDistance;
                            double powerValue = this->RSSI - DisDeacayRss;

                            if (powerValue < MinRSS)   losType = NLOS;

                            double PosNoise ;
                            if (losType == LOS) PosNoise = this->MinPosNoise;
                            else
                            {
                                PosNoise= this->MinPosNoise + (this->MaxPosNoise - this->MinPosNoise) * (this->MaxRSS-powerValue) / (this->MaxRSS - this->MinRSS);
                            }
                            

                            

                            std::normal_distribution<double> distributionRanging(distance * 1000, PosNoise);
                           // std::normal_distribution<double> distributionRss(rssMean[indexRanging][indexScenario], rssStd[indexRanging][indexScenario]);
                                                                            //对补偿后的测距，根据方案序号(losType)，（以及测试数据rangingStd?），对测距数据进行正态分布处理，得到最终测距数据rangingValue
                                                                            //根据方案序号(losType)，（以及测试数据rssMean和rssStd?），得到正态分布处理后的rss(接收信号强度)数据powerValue

                            double rangingValue = distributionRanging(this->random_generator);
                            //double powerValue = distributionRss(this->random_generator);
                            /*double DisDeacayRss = this->MaxDisDecay * distance / maxDBDistance;
                            double powerValue = this->RSSI - DisDeacayRss;*/

                            /*if (powerValue < minPower[indexScenario])       //三种情况下，小于(实验测得的最小接收信号强度minPower?)是，losTpye变为NLOS
                            {
                                losType = NLOS;
                            }*/
                            

                            if (losType!=NLOS )//   && aid != tagId)              //非NLOS情况下，发布测距话题,对应ranging_msg消息
                            {
                                gtec_msgs::Ranging ranging_msg;
                                ranging_msg.anchorId = aid;
                                ranging_msg.tagId = this->tagId;
                                ranging_msg.range = rangingValue;
                                ranging_msg.seq = this->sequence;
                                ranging_msg.rss = powerValue;
                                ranging_msg.errorEstimation = PosNoise; 
                                this->gtecUwbPub.publish(ranging_msg);
                            }
                        }

                        /*visualization_msgs::Marker marker;                 //更具losType情况发布MarkerArray
                        marker.header.frame_id = "world";
                        marker.header.stamp = ros::Time::now();     //之前为ros::Time()，没消息
                        marker.id = aid;
                        marker.type = visualization_msgs::Marker::CYLINDER;     //圆柱体
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.pose.position.x = anchorPose.Pos().X();
                        marker.pose.position.y = anchorPose.Pos().Y();
                        marker.pose.position.z = anchorPose.Pos().Z();
                        marker.pose.orientation.x = anchorPose.Rot().X();
                        marker.pose.orientation.y = anchorPose.Rot().Y();
                        marker.pose.orientation.z = anchorPose.Rot().Z();
                        marker.pose.orientation.w = anchorPose.Rot().W();
                        marker.scale.x = 0.2;
                        marker.scale.y = 0.2;
                        marker.scale.z = 0.5;
                        marker.color.a = 1.0;

                        if (losType == LOS)         //LOS为绿色
                        {
                            marker.color.r = 0.0;
                            marker.color.g = 0.6;
                            marker.color.b = 0.0;
                        }
                        else if (losType == NLOS_S) //NLOS_S为黄色
                        {
                            marker.color.r = 0.6;
                            marker.color.g = 0.6;
                            marker.color.b = 0.0;
                        }
                        else if (losType == NLOS_H) //NLOS_H为蓝色
                        {
                            marker.color.r = 0.0;
                            marker.color.g = 0.0;
                            marker.color.b = 0.6;
                        }
                        else if (losType == NLOS) //NLOS为红色
                        {
                            marker.color.r = 0.6;
                            marker.color.g = 0.0;
                            marker.color.b = 0.0;
                        }

                        markerArray.markers.push_back(marker);*/
                    }

                    //2023.2.18
                    else if ((  (*iter)->GetName().find(this->VehiclePrefix) == 0  )  &&  PubRangeVehicleTopic) //找到带有VehiclePrefix前缀命名的模型同时PubRangeVehicleTopic为true
                    {
                        physics::ModelPtr vehicle = *iter;           //得到载具vehicle
                        //std::string aidStr = anchor->GetName().substr(anchor->GetName().length()-1);
                        std::string vidStr = vehicle->GetName().substr(this->VehiclePrefix.length()+1);

                        /*cout<<tagId<<endl;
                        cout<<((*iter)->GetName())<<endl;
                        cout<<endl;*/
                        //cout<<vidStr<<endl;
                        
                        int vid = std::stoi(vidStr);
                        ignition::math::Pose3d vehiclePose = vehicle->WorldPose();    //锚点的3维位姿anchoPose

                        LOSType losType = LOS;     //设定lostType为LOS(0)
                        double distance = tagPose.Pos().Distance(vehiclePose.Pos());     //得到标签与载具之间的距离distance
                        //double distanceAfterRebounds = 0;               //定义反射后的距离 0

                        if (!allBeaconsAreLOS)              //所有的信标不全是LOS的情况
                        {
                            
                            double distanceToObstacleFromTag;   //从标签到障碍物的距离
                            std::string obstacleName;           //障碍物名称

                            ignition::math::Vector3d directionToVehicle = (vehiclePose.Pos() - tagPose.Pos()).Normalize();    //归一化后的标签到载具的方向向量



                            ignition::math::Vector3d tagPosOffset(tagPose.Pos().X() + directionToVehicle.X()* this->NoObstacleRadius, 
                                                                                                            tagPose.Pos().Y() +  directionToVehicle.Y()* this->NoObstacleRadius ,
                                                                                                            tagPose.Pos().Z() + directionToVehicle.Z() * this->NoObstacleRadius);     
                            tagPose.Set(tagPosOffset, tagPose.Rot());      //修正用于检测障碍物的tag位置

                            ignition::math::Vector3d vehiclePosOffset(vehiclePose.Pos().X() - directionToVehicle.X()* this->NoObstacleRadius, 
                                                                                                            vehiclePose.Pos().Y() -  directionToVehicle.Y()* this->NoObstacleRadius ,
                                                                                                            vehiclePose.Pos().Z() - directionToVehicle.Z() * this->NoObstacleRadius);     
                            vehiclePose.Set(vehiclePosOffset, vehiclePose.Rot());      //修正用于检测障碍物的vehicle位置

                            this->firstRay->Reset();                                        //重置firstRay
                            this->firstRay->SetPoints(tagPose.Pos(), vehiclePose.Pos());     //设firstRay为tagPose到vehiclePose
                            this->firstRay->GetIntersection(distanceToObstacleFromTag, obstacleName);   //从firstRay，得到标签到载具途中，障碍物的信息，包括标签到障碍物的距离，障碍物的名称

                            /*cout<<"Vehicle  "<<vid<<endl;
                            cout<<"distanceToObstacleFromTag  "<<tagId<<" :"<<distanceToObstacleFromTag<<endl;
                            cout<<"obstacleName:"<<obstacleName<<endl;*/


                            if (obstacleName.compare("") == 0)      //障碍物名称为空，即无障碍物，为LOS情况
                            {
                                //There is no obstacle between anchor and tag, we use the LOS model
                                losType = LOS;                      //losType为LOS
                                 this->RSSI = this->MaxRSS;
                                //distanceAfterRebounds = distance;   //反射距离为直接距离
                                //cout<<"No Obstacle ,  LOS"<<endl;
                            }
                            else                                    //存在障碍物,分情况讨论
                            {

                                //We use a second ray to measure the distance from anchor to tag, so we can
                                //know what is the width of the walls
                                double distanceToObstacleFromVehicle;    //载具到障碍物的距离
                                std::string otherObstacleName;          //其他障碍物的名称

                                this->secondRay->Reset();               //重置secondRay
                                this->secondRay->SetPoints(vehiclePose.Pos(), tagPose.Pos());    //设secondRay为anchorPose到tagPose
                                this->secondRay->GetIntersection(distanceToObstacleFromVehicle, otherObstacleName);      //从secondRay，得到锚点到标签途中，障碍物的信息，包括锚点到障碍物的距离，障碍物的名称

                                //cout<<"distanceToObstacleFromVehicle:"<<distanceToObstacleFromVehicle<<endl;

                                this->wallWidth = distance - distanceToObstacleFromTag - distanceToObstacleFromVehicle;     //得到障碍物的厚度wallWidth

                                
                                
                                //ROS_INFO("wallWidth of vehicle:  %f\n",this->wallWidth);
                                if (this->wallWidth <= this->nlosSoftWallWidth )   //障碍物厚度小于阈值，为NLOS_S
                                {
                                    //We use NLOS - SOFT model
                                    losType = NLOS_S;                   //losType为NLOS_S
                                    this->RSSI = this->MaxRSS - (this->MaxRSS - this->MinRSS) * this->wallWidth / this->nlosSoftWallWidth;  //根据障碍物厚度得到当前RSSI值
                                }
                                else        //障碍物厚度大于阈值
                                {
                                    losType =NLOS ;
                                
                                }
                            }

                        }
                        else        //所有信标都是LOS状态，则losType为LOS，反射距离等于直接距离
                        {
                            //All beacons are LOS
                            losType = LOS;
                            this->RSSI = this->MaxRSS;
                        }

                        if ((losType == LOS || losType == NLOS_S) && distance > maxDBDistance)  //对于LOS和NLOS_S情况，当距离大于maxDBDistance时，则变为NLOS情况
                        {
                            losType = NLOS;
                        }

                        /*if (losType == NLOS_H && distance > maxDBDistance)     //NLOS_H情况同上
                        {
                            losType = NLOS;
                        }*/

                        if (losType != NLOS)        //对于非NLOS情况，即LOS,NLOS_H,NLOS_S
                        {

              

                            //normal_distribution:正态分布


                            double DisDeacayRss = this->MaxDisDecay * distance / maxDBDistance;
                            double powerValue = this->RSSI - DisDeacayRss;

                            if (powerValue < MinRSS)   losType = NLOS;

                            double PosNoise ;
                            if (losType == LOS)     PosNoise = this->MinPosNoise;
                           
                            else
                            {
                                PosNoise= this->MinPosNoise + (this->MaxPosNoise - this->MinPosNoise) * (this->MaxRSS-powerValue) / (this->MaxRSS - this->MinRSS);
                            }
                            

                            

                            std::normal_distribution<double> distributionRanging(distance * 1000, PosNoise);
                           // std::normal_distribution<double> distributionRss(rssMean[indexRanging][indexScenario], rssStd[indexRanging][indexScenario]);
                                                                            //对补偿后的测距，根据方案序号(losType)，（以及测试数据rangingStd?），对测距数据进行正态分布处理，得到最终测距数据rangingValue
                                                                            //根据方案序号(losType)，（以及测试数据rssMean和rssStd?），得到正态分布处理后的rss(接收信号强度)数据powerValue

                            double rangingValue = distributionRanging(this->random_generator);
                         
                           /* cout<<endl;
                            cout<<vid<<endl;
                            cout<<tagId<<endl;
                            cout<<endl;*/
                            if ( vid != tagId)              //非同ID情况下，发布测距话题,对应ranging_msg消息
                            {
                                gtec_msgs::Ranging_Vehicle ranging_msg;
                                ranging_msg.vehicleId = vid;
                                ranging_msg.tagId = this->tagId;
                                ranging_msg.range = rangingValue;
                                ranging_msg.seq = this->sequence;
                                ranging_msg.rss = powerValue;
                                ranging_msg.errorEstimation = PosNoise; 
                                this->gtecUwbPubVehicle.publish(ranging_msg);
                            }
                        }

                      
                    }
                }

                //this->gtecAnchors.publish(markerArray);
                this->sequence++;
            }
        }

    public:
        void SetUpdateRate(double _rate)
        {
            if (_rate > 0.0)
            {
                this->updatePeriod = 1.0 / _rate;
            }
            else
            {
                this->updatePeriod = 0.0;
            }
        }

    public:
        void Reset() override
        {
            ROS_INFO("GTEC UWB Plugin RESET");
            this->lastUpdateTime = common::Time(0.0);
        }

    private:
        physics::ModelPtr model;
    private:
        physics::WorldPtr world;
    private:
        physics::RayShapePtr firstRay;
    private:
        physics::RayShapePtr secondRay;
    private:
        event::ConnectionPtr updateConnection;
    private:
        common::Time updatePeriod;
    private:
        common::Time lastUpdateTime;
    private:
        double tagZOffset;

    //2023.2.21
    private:
        double tagXOffset;
     private:
        double tagYOffset;

    private:
        std::string anchorPrefix;
    private:
        physics::LinkPtr tagLink;
    private:
        ros::Publisher gtecUwbPub;

    //2023.2.18
    private:
        ros::Publisher gtecUwbPubVehicle;

    //private:
    //    ros::Publisher gtecAnchors;
    private:
        unsigned int sequence;      //之前是unsined char
    private:
        double nlosSoftWallWidth;
    private:
        double maxDBDistance;
    private:
        double stepDBDistance;
    private:
        bool allBeaconsAreLOS;
    private:
        int tagId;
    private:
        bool useParentAsReference;
    private:
        std::default_random_engine random_generator;

    //2023.2.17
    private:
        double wallWidth;
    private:
        int MaxPosNoise;
    private:
        int MinPosNoise;
    private :
        float MaxRSS;
    private :
        float MinRSS;
    private:
        double RSSI;
    private:
        float MaxDisDecay;

    //2023.2.18
    private:
        std::string VehiclePrefix;

    //2023.2.20
    private:
        bool PubRangeTopic;
    private:
        bool PubRangeVehicleTopic;

    //2023.2.21
    private:
        double NoObstacleRadius;

    };

    GZ_REGISTER_MODEL_PLUGIN(UwbPlugin)
}

