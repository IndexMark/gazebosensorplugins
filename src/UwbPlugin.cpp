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

        enum LOSType        //枚举类型LOSType
        {
            LOS,            //0,信号没有阻挡
            NLOS,           //1,无信号，距离太远或者完全阻挡
            NLOS_S,         //2,有信号，有较薄的障碍物，收到信号强度减弱
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

                

                physics::Model_V models = this->world->Models();            //获取模型们models
                for (physics::Model_V::iterator iter = models.begin(); iter != models.end(); ++iter)  //遍历world中的每一个模型
                {
                       // cout<<((*iter)->GetName())<<endl;
                    if (   (  (*iter)->GetName().find(this->anchorPrefix) == 0  ) && PubRangeTopic ) //找到带有anchorPrefix前缀命名的模型同时PubRangeTopic为true
                    {
                        physics::ModelPtr anchor = *iter;           //得到锚点anchor
                        std::string aidStr = anchor->GetName().substr(this->anchorPrefix.length());

                        //cout<<((*iter)->GetName())<<endl;
                        //cout<<aidStr<<endl;
                        //ROS_INFO("%s\n", &anchor);
                        //ROS_INFO("%c\n", &aidStr);
                        int aid = std::stoi(aidStr);
                        ignition::math::Pose3d anchorPose = anchor->WorldPose();    //锚点的3维位姿anchoPose

                        LOSType losType = LOS;     //设定lostType为LOS(0)
                        double distance = tagPose.Pos().Distance(anchorPose.Pos());     //得到标签与锚点之间的距离distance

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

                        if (losType != NLOS)        //对于非NLOS情况，即LOS,NLOS_H,NLOS_S
                        {

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
                            double rangingValue = distributionRanging(this->random_generator);

                            

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

                    }

                    //2023.2.18
                    else if ((  (*iter)->GetName().find(this->VehiclePrefix) == 0  )  &&  PubRangeVehicleTopic) //找到带有VehiclePrefix前缀命名的模型同时PubRangeVehicleTopic为true
                    {
                        physics::ModelPtr vehicle = *iter;           //得到载具vehicl
                        std::string vidStr = vehicle->GetName().substr(this->VehiclePrefix.length()+1);

                        /*cout<<tagId<<endl;
                        cout<<((*iter)->GetName())<<endl;
                        cout<<endl;*/
                        //cout<<vidStr<<endl;
                        
                        int vid = std::stoi(vidStr);
                        ignition::math::Pose3d vehiclePose = vehicle->WorldPose();    //锚点的3维位姿anchoPose

                        LOSType losType = LOS;     //设定lostType为LOS(0)
                        double distance = tagPose.Pos().Distance(vehiclePose.Pos());     //得到标签与载具之间的距离distance

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

