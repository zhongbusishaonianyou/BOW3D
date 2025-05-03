#include "BoW3D.h"
#include <fstream>

using namespace std;


namespace BoW3D
{
    BoW3D::BoW3D(LinK3D_Extractor* pLinK3D_Extractor,ros::NodeHandle* nh): 
            mpLinK3D_Extractor(pLinK3D_Extractor), 
            nh(*nh)   {
            nh->param("/bow3d/thr", thr, 3.0f);
            nh->param("/bow3d/thf", thf, 5);
            nh->param("/bow3d/revisit_threshold", revisit_thres, 4.0);
            nh->param("/bow3d/num_add_retrieve_features", num_add_retrieve_features, 5);
            nh->param("/bow3d/num_nodes_exclude", num_nodes_exclude, 300);
            N_nw_ofRatio = std::make_pair(0, 0); 
            //cout<<"The number of added or retrieved features for each frame is "<<num_add_retrieve_features<<endl;
    }
    BoW3D::~BoW3D(){}
    void BoW3D::update(Frame* pCurrentFrame)
    {
        mvFrames.emplace_back(pCurrentFrame);

        cv::Mat descriptors = pCurrentFrame->mDescriptors;
        long unsigned int frameId = pCurrentFrame->mnId;
        size_t numFeature = descriptors.rows;
    
        if(numFeature < (size_t)num_add_retrieve_features) 
        {
            for(size_t i = 0; i < numFeature; i++)
            {
                float *p = descriptors.ptr<float>(i);
                for(size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    if(p[j] != 0)
                    {
                        unordered_map<pair<float, int>, unordered_set<pair<int, int>, pair_hash>, pair_hash>::iterator it; 

                        pair<float, int> word= make_pair(p[j], j);
                        it = this->find(word);
                        
                        //没有找到,添加单词
                        if(it == this->end())
                        {
                            unordered_set<pair<int,int>, pair_hash> place;
                            //PlaceS et.insert((FrameID, DesID))，i表示描述子的行数
                            place.insert(make_pair(frameId, i));
                            (*this)[word] = place;
                            //the number of words in vocabulary
                            N_nw_ofRatio.first++;
                            N_nw_ofRatio.second++;
                        }
                        else
                        {   //将当前观测位置添加到该单词的位置集合中
                            (*it).second.insert(make_pair(frameId, i));
                           // the total number of places seenso far
                            N_nw_ofRatio.second++;
                        }

                    }
                }
            }
        }
        else
        {
            for(size_t i = 0; i < (size_t)num_add_retrieve_features; i++)
            {
                float *p = descriptors.ptr<float>(i);
                for(size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    if(p[j] != 0)
                    {
                        unordered_map<pair<float, int>, unordered_set<pair<int, int>, pair_hash>, pair_hash>::iterator it; 

                        pair<float, int> word= make_pair(p[j], j);
                        it = this->find(word);

                        if(it == this->end())
                        {
                            unordered_set<pair<int,int>, pair_hash> place;
                            //对于每个位置使用帧号和描述子的行号
                            place.insert(make_pair(frameId, i));
                            (*this)[word] = place;

                            N_nw_ofRatio.first++;
                            N_nw_ofRatio.second++;
                        }
                        else
                        {
                            (*it).second.insert(make_pair(frameId, i));

                            N_nw_ofRatio.second++;
                        }
                    }
                }
            }
        }
    }
       

    void BoW3D::retrieve(Frame* pCurrentFrame, int &loopFrameId, Eigen::Matrix3d &loopRelR, Eigen::Vector3d &loopRelt,double & matched_count)
    {        
        int frameId = pCurrentFrame->mnId;

        cv::Mat descriptors = pCurrentFrame->mDescriptors;      
        size_t rowSize = descriptors.rows;    

        vector<pair<int, int>> candidateFrame; 
        map<int, int> mFrameScore;  // save all frame with score
        map<int, int>mScoreFrameID;
       /*这里分类，检索较小的行数*/
        if(rowSize < (size_t)num_add_retrieve_features) 
        {  
            for(size_t i = 0; i < rowSize; i++)
            {   //frameID,DESID-->frequency
                unordered_map<pair<int, int>, int, pair_hash> mPlaceScore;                
           //opencv ptr用法 ushort d = d1.ptr<unsignedshort> (row)[column];就是指向d1的第row行的第column个数据。数据类型为无符号的短整型。
          //其中使用.ptr函数访问Mat类对象d1的第row行首地址，[column]表示本行的第column个对象，整体来看就是获取了d1内第row行第column列的元素的值，存储为uchar类型                 
                float *p = descriptors.ptr<float>(i);

                int countValue = 0;

                for(size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    countValue++;

                    if(p[j] != 0)
                    {   //构建单词           
                        pair<float, int> word = make_pair(p[j], j); 
                        //查找单词 
                        auto wordPlacesIter = this->find(word);

                        if(wordPlacesIter == this->end())
                        {
                            continue;
                        }
                        else
                        {  //在后面更新时对相应参数进行统计
                            double averNumInPlaceSet = N_nw_ofRatio.second / N_nw_ofRatio.first;
                            int curNumOfPlaces = (wordPlacesIter->second).size();
                            
                            double ratio = curNumOfPlaces / averNumInPlaceSet;
                            //过滤频率过高单词
                            if(ratio > thr)
                            {
                                continue;
                            }

                            for(auto placesIter = (wordPlacesIter->second).begin(); placesIter != (wordPlacesIter->second).end(); placesIter++)
                            {
                                //The interval between the loop and the current frame should be at least 300.
                                if(frameId - (*placesIter).first < num_nodes_exclude) 
                                {
                                    continue;
                                }

                                auto placeNumIt = mPlaceScore.find(*placesIter);                    
                                //寻找该位置是否已经在哈系表中，不存在，置1
                                if(placeNumIt == mPlaceScore.end())
                                {                                
                                    mPlaceScore[*placesIter] = 1;
                                }
                                //存在，频率加一
                                else
                                {
                                    mPlaceScore[*placesIter]++;                                    
                                }                                                              
                            }                       
                        }                            
                    }                    
                }

                for(auto placeScoreIter = mPlaceScore.begin(); placeScoreIter != mPlaceScore.end(); placeScoreIter++)
                {  
                    if((*placeScoreIter).second > thf) 
                    {
                        if(mFrameScore.find(((*placeScoreIter).first).first) == mFrameScore.end())
                        mFrameScore[((*placeScoreIter).first).first] = (*placeScoreIter).second;
                        else
                        {
                        if(mFrameScore[((*placeScoreIter).first).first] < (*placeScoreIter).second)
                          mFrameScore[((*placeScoreIter).first).first] = (*placeScoreIter).second;
                        }


                       mScoreFrameID[(*placeScoreIter).second] = ((*placeScoreIter).first).first;
                      // std::cout<<"FrameID:"<< ((*placeScoreIter).first).first<<"of times:"<<(*placeScoreIter).second<<std::endl;
                    }
                }                                   
            }                  
        }
        else
        {
            for(size_t i = 0; i < (size_t)num_add_retrieve_features; i++) 
            {
                unordered_map<pair<int, int>, int, pair_hash> mPlaceScore;
                
                float *p = descriptors.ptr<float>(i);

                int countValue = 0;

                for(size_t j = 0; j < (size_t)descriptors.cols; j++)
                {
                    countValue++;

                    if(p[j] != 0)
                    {                   
                        pair<float, int> word = make_pair(p[j], j);    

                        auto wordPlacesIter = this->find(word);

                        if(wordPlacesIter == this->end())
                        {
                            continue;
                        }
                        else
                        {
                            double averNumInPlaceSet = (double) N_nw_ofRatio.second / N_nw_ofRatio.first;
                            int curNumOfPlaces = (wordPlacesIter->second).size();

                            double ratio = curNumOfPlaces / averNumInPlaceSet;

                            if(ratio > thr)
                            {
                                continue;
                            }
                            
                            for(auto placesIter = (wordPlacesIter->second).begin(); placesIter != (wordPlacesIter->second).end(); placesIter++)
                            {
                                //The interval between the loop and the current frame should be at least 300.
                                if(frameId - (*placesIter).first < num_nodes_exclude) 
                                {
                                    continue;
                                }

                                auto placeNumIt = mPlaceScore.find(*placesIter);                    
                                
                                if(placeNumIt == mPlaceScore.end())
                                {                                
                                    mPlaceScore[*placesIter] = 1;
                                }
                                else
                                {
                                    mPlaceScore[*placesIter]++;                                    
                                }                                                              
                            }                       
                        }                            
                    }
                }
                //frameid.desid-->fre
                for(auto placeScoreIter = mPlaceScore.begin(); placeScoreIter != mPlaceScore.end(); placeScoreIter++)
                {
                    if((*placeScoreIter).second > thf) 
                    {  
                       
                        if(mFrameScore.find(((*placeScoreIter).first).first) == mFrameScore.end())
                        mFrameScore[((*placeScoreIter).first).first] = (*placeScoreIter).second;
                        else
                        {
                        if(mFrameScore[((*placeScoreIter).first).first] < (*placeScoreIter).second)
                          mFrameScore[((*placeScoreIter).first).first] = (*placeScoreIter).second;
                        }
                       
                        //desid-->FrameId
                       mScoreFrameID[(*placeScoreIter).second] = ((*placeScoreIter).first).first;
                      // std::cout<<"FrameID:"<< ((*placeScoreIter).first).first<<"of times:"<<(*placeScoreIter).second<<std::endl;
                    }
                   
                }                                   
            }                           
        }     
        //cout<<"FrameID:"<<frameId<<"has "<<mFrameScore.size()<<" candidates."<<endl;
        //没有找到满足要求的候选帧
        if(mFrameScore.size() == 0)
        {
            return;
        }
        for(auto it:mFrameScore)
        {
            candidateFrame.emplace_back(make_pair(it.second, it.first));
        }
        sort(candidateFrame.begin(), candidateFrame.end(), [](const pair<int, int> &a, const pair<int, int> &b){return a.first > b.first;});
        matched_count=1;
        double inlier_ratio = 0;  
        for(auto it = candidateFrame.begin(); it !=candidateFrame.end(); it++)
        {
                
            int loopId = (*it).second;

            Frame* pLoopFrame = mvFrames[loopId];
            vector<pair<int, int>> vMatchedIndex;  
            
            mpLinK3D_Extractor->match(pCurrentFrame->mvAggregationKeypoints, pLoopFrame->mvAggregationKeypoints, pCurrentFrame->mDescriptors, pLoopFrame->mDescriptors, vMatchedIndex);               
           // if((int)vMatchedIndex.size()>matched_count)
           // {
            //    matched_count = 1;
            //    loopFrameId = -1;
           // }
           
            int returnValue = 0;
            Eigen::Matrix3d loopRelativeR;
            Eigen::Vector3d loopRelativet;
                            
            returnValue = loopCorrection(pCurrentFrame, pLoopFrame, vMatchedIndex,inlier_ratio,loopRelativeR, loopRelativet);
           
            //The distance between the loop and the current should less than 3m.                  
            if(returnValue != -1 && loopRelativet.norm() < revisit_thres && loopRelativet.norm() > 0) 
            {
    
                loopFrameId = (*it).second;
                loopRelR = loopRelativeR;
                loopRelt = loopRelativet;                         
                matched_count = 1-inlier_ratio;
                return;
            }
           

        //   if(returnValue != -1  && loopRelativet.norm() > 0)
        //    {     
                 
        //          loopFrameId = (*it).second;
        //          loopRelR = loopRelativeR;
        //          if (count!=1)
        //          {
        //          if (loopRelativet.norm()<loopRelt.norm())
        //          {
        //          loopRelt = loopRelativet;  
        //          }
                                        
        //         }
        //         else 
        //         loopRelt = loopRelativet;
        //         count++;
        //          //return;
        //      }

        } 
    }


    int BoW3D::loopCorrection(Frame* currentFrame, Frame* matchedFrame, vector<pair<int, int>> &vMatchedIndex, double & inlier_ratio,Eigen::Matrix3d &R, Eigen::Vector3d &t)
    {   
        
        if(vMatchedIndex.size() <=30)
        {
            return -1;
        }

        ScanEdgePoints currentFiltered;
        ScanEdgePoints matchedFiltered;
        mpLinK3D_Extractor->filterLowCurv(currentFrame->mClusterEdgeKeypoints, currentFiltered);
        mpLinK3D_Extractor->filterLowCurv(matchedFrame->mClusterEdgeKeypoints, matchedFiltered);

        vector<std::pair<PointXYZSCA, PointXYZSCA>> matchedEdgePt;
        mpLinK3D_Extractor->findEdgeKeypointMatch(currentFiltered, matchedFiltered, vMatchedIndex, matchedEdgePt);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::CorrespondencesPtr corrsPtr (new pcl::Correspondences()); 

        for(int i = 0; i < (int)matchedEdgePt.size(); i++)
        {
            std::pair<PointXYZSCA, PointXYZSCA> matchPoint = matchedEdgePt[i];

            pcl::PointXYZ sourcePt(matchPoint.first.x, matchPoint.first.y, matchPoint.first.z);            
            pcl::PointXYZ targetPt(matchPoint.second.x, matchPoint.second.y, matchPoint.second.z);
            
            source->push_back(sourcePt);
            target->push_back(targetPt);

            pcl::Correspondence correspondence(i, i, 0);
            corrsPtr->push_back(correspondence);
        }

        pcl::Correspondences corrs;
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> Ransac_based_Rejection;
        Ransac_based_Rejection.setInputSource(source);
        Ransac_based_Rejection.setInputTarget(target);
        double sac_threshold = 0.4;
        Ransac_based_Rejection.setInlierThreshold(sac_threshold);
        Ransac_based_Rejection.getRemainingCorrespondences(*corrsPtr, corrs);
        
        if(corrs.size() <= 100)
        {
            return -1;
        } 
        //add this line to get the inlier ratio for pr curve      
        inlier_ratio = static_cast<double>(corrs.size()) / corrsPtr->size();
        //std::cout << "Inlier Ratio: " << inlier_ratio << std::endl;   

        Eigen::Vector3d p1 = Eigen::Vector3d::Zero();
        Eigen::Vector3d p2 = p1;
        int corrSize = (int)corrs.size();
        for(int i = 0; i < corrSize; i++)
        {  
            pcl::Correspondence corr = corrs[i];         
            p1(0) += source->points[corr.index_query].x;
            p1(1) += source->points[corr.index_query].y;
            p1(2) += source->points[corr.index_query].z; 

            p2(0) += target->points[corr.index_match].x;
            p2(1) += target->points[corr.index_match].y;
            p2(2) += target->points[corr.index_match].z;
        }

        Eigen::Vector3d center1 = Eigen::Vector3d(p1(0)/corrSize, p1(1)/corrSize, p1(2)/corrSize);
        Eigen::Vector3d center2 = Eigen::Vector3d(p2(0)/corrSize, p2(1)/corrSize, p2(2)/corrSize);
       
        vector<Eigen::Vector3d> vRemoveCenterPt1, vRemoveCenterPt2; 
        for(int i = 0; i < corrSize; i++)
        {
            pcl::Correspondence corr = corrs[i];
            pcl::PointXYZ sourcePt = source->points[corr.index_query];
            pcl::PointXYZ targetPt = target->points[corr.index_match];

            Eigen::Vector3d removeCenterPt1 = Eigen::Vector3d(sourcePt.x - center1(0), sourcePt.y - center1(1), sourcePt.z - center1(2));
            Eigen::Vector3d removeCenterPt2 = Eigen::Vector3d(targetPt.x - center2(0), targetPt.y - center2(1), targetPt.z - center2(2));
        
            vRemoveCenterPt1.emplace_back(removeCenterPt1);
            vRemoveCenterPt2.emplace_back(removeCenterPt2);
        }

        Eigen::Matrix3d w = Eigen::Matrix3d::Zero();

        for(int i = 0; i < corrSize; i++)
        {
            w += vRemoveCenterPt1[i] * vRemoveCenterPt2[i].transpose();
        }      

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeFullU|Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();
        
        R = V * U.transpose();
        t = center2 - R * center1;

        return 1;
    }

}
