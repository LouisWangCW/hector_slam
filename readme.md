# urg_test
ROSでURGを使う際のサンプルコード.  
1秒ごとにURGの取得データのうち中央の距離/強度データを出力します.  
実際に使うURGに合わせて `launch/urg_test.launch` を変更してください.  

## 利用
コンパイル.
```
sudo apt-get install ros-${ROS_DISTRO}-urg-node
cd ~/catkin_ws/
git@github.com:BrownieAlice/urg_test.git
catkin_make
```

実際に使う場合.
```
roslaunch urg_test urg_test.launch
```
