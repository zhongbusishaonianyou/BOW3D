# BOW3D(RAL-2024)
- source code :https://github.com/YungeCui/BoW3D
# revised description
- we add code for visualizing edge points based on Link3D .
- We modified the method for extracting candidate frames, suggested from [http://q](https://github.com/YungeCui/BoW3D/pulls).
- We draw the PR curve by calculating the proportion of the inner points after the RANSAC point cloud registration. Maybe this approach is not very appropriate.We attempted to use most of the parameters in the algorithm as adjustable thresholds to plot the PR curve, but none of them could correctly reflect the performance of the method. The most reasonable parameter in the algorithm should be the number of ‘vMatchedIndex’.
  
  ![2025-05-03_20-27](https://github.com/user-attachments/assets/4e8f3dbc-e517-4c0d-9e9e-f810fff4307f)

# results
 |                                                    KITTI 00  |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![Figure_5](https://github.com/user-attachments/assets/ce2d4153-ddc3-4c02-8cd7-06d642f5026d) |![Figure_6](https://github.com/user-attachments/assets/2cb5dbc6-01a4-43f7-b950-f524938109c0)|

   |                                                    KITTI 02  |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![Figure_7](https://github.com/user-attachments/assets/32e9a9c6-8a87-481f-9b28-02e0bcc83721)|![Figure_8](https://github.com/user-attachments/assets/272f89ca-e57c-4373-9fe3-af96bc694555)|

   |                                                    KITTI 05  |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   | ![Figure_3](https://github.com/user-attachments/assets/63808d0f-70fa-4788-91d8-9a365f0d6d12)| ![Figure_4](https://github.com/user-attachments/assets/8eb4c511-0b1a-471b-ae1c-5f3ba2571317)|

   |                                                     KITTI 08 |                                                              |
   | ------------------------------------------------------------ | ------------------------------------------------------------ |
   |  ![Figure_1](https://github.com/user-attachments/assets/cddd01fe-71a7-40d9-a9ca-e11e519b7ef2)|![Figure_2](https://github.com/user-attachments/assets/7e9d6752-dc38-4399-9ea0-4166870a0998)| 
 # cite
```
@ARTICLE{9944848,
  author={Cui, Yunge and Chen, Xieyuanli and Zhang, Yinlong and Dong, Jiahua and Wu, Qingxiao and Zhu, Feng},
  journal={IEEE Robotics and Automation Letters}, 
  title={BoW3D: Bag of Words for Real-Time Loop Closing in 3D LiDAR SLAM}, 
  year={2023},
  volume={8},
  number={5},
  pages={2828-2835},
  doi={10.1109/LRA.2022.3221336}}
  ```
