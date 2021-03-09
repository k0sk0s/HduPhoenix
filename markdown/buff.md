# Buff.h
定义detectMode 选择我方颜色

定义颜色模式 预测模式

定义装甲数据格式 初始化

计算距离、中心、角速度、3点拟合buff椭圆

# Buff.cpp
## countRotationAngle
根据当前中心和上次中心算出距离，得出旋转角

## circleLeastFit
根据点集用最小二乘法拟合圆，得出圆心坐标、半径

## setBinary
BGR转换原图Mode为灰度二值图

HSV根据buff颜色分割阈值，和灰度图与操作获得轮廓

## armorCornerSort
返回锤子的实际角度与装甲顶点排序

## getArmorCenter_new
### 锤子（只有1个子轮廓）
1. 原图二值化 膨胀
2. 寻找轮廓 统计轮廓个数
3. 根据轮廓数量筛选合适的轮廓
4. 根据轮廓4个顶点画出装甲板矩形
5. 根据两个中心的距离和运行时间计算角速度，预测新的中心
6. 将点存入点集 用于拟合椭圆
### 六边形（子轮廓>2)
1. 根据面积筛选，画出全部六边形
2. 找到所有子轮廓并存入下标
3. 计算所有子轮廓的长宽比
4. 利用子轮廓的长宽比来区分装甲板和其他两个扁长的矩形
### circleLeastFit
根据不同的半径，计算平均半径，拟合最小圆

## preArmorCenter
### SMALL_BUFF
获得转动的角度，nextCoordinate预测下一个中心
### BIG_BUFF
似乎和SMALL_BUFF相同，感觉有点问题

## isClockwise
判断目标中心的象限位置,得出旋转方向

## nextCoordinate
利用现在的中心点坐标和旋转角度的三角函数，根据旋转方向，计算瞄准中心点坐标（提前量）

## RansacCircle
根据点云拟合圆，排除噪声干扰，具体算法还不太理解

