#To build the code : 
mkdir build
cd ./build
cmake ..
make

#How to run the code :

cd ./build
./g2o_customBundle -input ../data/problem-16-22106-pre.txt

#see more detail settings by :
./g2o_customBundle -help



problem-16-22106-pre.txt数据集说明：
第1行表示相机数量为16，路标点数量为22106，观测数量为83718，这不奇怪，因为不同相机不一定能看到所有路标点，观测数量并非16*22106
第2行到第83719行表示相机m看到路标n在相机中的像素坐标，实际上以图像中心为原点，没有进行平移，以第2行为例，表示第0个相机看到第0个路标的像素坐标为(-3.859900e+02, 3.871200e+02)
第83720行到83863行一共144行表示16*9，即16个相机各自的9个参数，分别为旋转向量(012)，平移向量(345)，焦距fx(6)，畸变参数k1,k2(78)
第83864行到150181行一共66318行表示22106*3，即22106个路标点各自的3个世界坐标


