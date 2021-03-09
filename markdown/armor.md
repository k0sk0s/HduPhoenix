# ArmorDector.h (class)
## ArmorParam

设置Filter阈值、预处理参数:

Pre-treatment; Filter lights; Filter pairs; Filter armor; other params
## LightDescriptor
确定灯条的width length center angle area 画出轮廓

## ArmorDescriptor
参数初始化，计算置信度

## ArmorDetector
定义Armor检测状态，设置敌方颜色，加载图片

# ArmorDector.cpp (function)
## adjustRec
调整角度 构造矩形

## tesseract(没看懂)

## detectArmorNumber(没看懂)
```c++
gammaCorrect(armor.frontImg, roi, 1.0/1.5);
threshold(roi, roi, 50, 255, THRESH_BINARY_INV);
tess.SetImage((uchar*)roi.data, roi.cols, roi.rows, 1, roi.cols);
```

## detect
### lightinfos:
pre-treatment
1. cvtColor(COLOR_BGR2GRAY)
2. threshold
3. dilate*2
   
find and filter light bars

1. findContours()
2. 遍历轮廓 画出灯条轮廓 
   
   contour.size()<=5, lightContourArea < min_area 忽略
   
   fitEllipse()拟合椭圆;  adjustRec()调整;

   长宽比、solidity判读是否忽略

   长宽比扩大, boundingRect()获得最小矩形

   构建Mat类 lightMask 掩模用于meanVal的计算 获取敌方颜色
3. 检测是否检测到装甲

find and filter light bar pairs

1. 2-3m的情况  根据灯条角度和长度筛选
2. 长度相近的情况下 根据长宽比筛选(适应于大、小装甲)
3. 储存符合条件的装甲

### delete the fake armors
calculate the final score
1. 遍历_armors detectArmorNumber(armor)
   finalScore=sizeScore+distScore+rotationScore
2. 找到得分最高的armor 储存到targetArmor
3. 返回 _flag=ARMOR_LOCAL

## getArmorVertex
获得目标实际位置（原roi位置）

## loadImg
显示roiImg(还有一些细节没看懂)

## ArmorDescriptor
根据两个灯条的四个顶点确定装甲位置 计算获得 rotationScore sizeScore distScore

## getFrontImg
通过getPerspectiveTransform和warpPerspective得到装甲正面图像

## isArmorPattern
装甲图片格式转换成Mat 传入SVM 进行预测

## getCenterPoint
根据装甲矩形的四个顶点 计算aimPoint(瞄准中心点)