# coding=utf-8
from osgeo import gdal
from gdalconst import *
import numpy as np


def readImage(img_path):
    band_data = []
    band_name = []

    # 以只读方式打开遥感影像
    dataset = gdal.Open(img_path, GA_ReadOnly)
    if dataset is None:
        print("Unable to open image file.")
        return band_data
    else:
        print("Open image file success.\n")

        # 读取地理变换参数
        param_geoTransform = dataset.GetGeoTransform()
        print "GeoTransform info:\n", param_geoTransform, "\n"

        # 读取投影信息
        param_proj = dataset.GetProjection()
        print "Projection info:\n", param_proj, "\n"

        # 读取波段数及影像大小
        bands_num = dataset.RasterCount
        print("Image height:" + dataset.RasterYSize.__str__() + " Image width:" + dataset.RasterXSize.__str__())
        print(bands_num.__str__() + " bands in total.")

        # 依次读取波段数据
        for i in range(bands_num):
            # 获取影像的第i+1个波段
            band_i = dataset.GetRasterBand(i + 1)

            # 获取影像第i+1个波段的描述(名称)
            name = band_i.GetDescription()
            band_name.append(name)

            # 读取第i+1个波段数据
            data = band_i.ReadAsArray(0, 0, band_i.XSize, band_i.YSize)
            band_data.append(data)

            print("band " + (i + 1).__str__() + " read success.")
            if name != "":
                print "Name:", name
        return band_data, param_geoTransform, param_proj, band_name


def writeImage(save_path, bands, geotrans=None, proj=None, names=None):
    projection = [
        # WGS84坐标系(EPSG:4326)
        """GEOGCS["WGS 84", DATUM["WGS_1984", SPHEROID["WGS 84", 6378137, 298.257223563, AUTHORITY["EPSG", "7030"]], AUTHORITY["EPSG", "6326"]], PRIMEM["Greenwich", 0, AUTHORITY["EPSG", "8901"]], UNIT["degree", 0.01745329251994328, AUTHORITY["EPSG", "9122"]], AUTHORITY["EPSG", "4326"]]""",
        # Pseudo-Mercator、球形墨卡托或Web墨卡托(EPSG:3857)
        """PROJCS["WGS 84 / Pseudo-Mercator",GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563,AUTHORITY["EPSG","7030"]],AUTHORITY["EPSG","6326"]],PRIMEM["Greenwich",0,AUTHORITY["EPSG","8901"]],UNIT["degree",0.0174532925199433,AUTHORITY["EPSG","9122"]],AUTHORITY["EPSG","4326"]],PROJECTION["Mercator_1SP"],PARAMETER["central_meridian",0],PARAMETER["scale_factor",1],PARAMETER["false_easting",0],PARAMETER["false_northing",0],UNIT["metre",1,AUTHORITY["EPSG","9001"]],AXIS["X",EAST],AXIS["Y",NORTH],EXTENSION["PROJ4","+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext  +no_defs"],AUTHORITY["EPSG","3857"]]"""
    ]

    if bands is None or bands.__len__() == 0:
        return False
    else:
        # 认为各波段大小相等，所以以第一波段信息作为保存
        band1 = bands[0]
        # 设置影像保存大小、波段数
        img_width = band1.shape[1]
        img_height = band1.shape[0]
        num_bands = bands.__len__()

        # 设置保存影像的数据类型
        if 'int8' in band1.dtype.name:
            datatype = gdal.GDT_Byte
        elif 'int16' in band1.dtype.name:
            datatype = gdal.GDT_UInt16
        else:
            datatype = gdal.GDT_Float32

        # 创建文件
        driver = gdal.GetDriverByName("GTiff")
        dataset = driver.Create(save_path, img_width, img_height, num_bands, datatype)

        if dataset is not None:
            # 写入仿射变换参数
            if geotrans is not None:
                dataset.SetGeoTransform(geotrans)

            # 写入投影参数
            if proj is not None:
                if proj is 'WGS84' or \
                        proj is 'wgs84' or \
                        proj is 'EPSG:4326' or \
                        proj is 'EPSG-4326' or \
                        proj is '4326':
                    dataset.SetProjection(projection[0])  # 写入投影
                elif proj is 'EPSG:3857' or \
                        proj is 'EPSG-3857' or \
                        proj is '3857':
                    dataset.SetProjection(projection[1])  # 写入投影
                else:
                    dataset.SetProjection(proj)  # 写入投影

            # 逐波段写入数据
            for i in range(bands.__len__()):
                raster_band = dataset.GetRasterBand(i + 1)

                # 设置没有数据的像素值为0
                raster_band.SetNoDataValue(0)

                if names is not None:
                    # 设置波段的描述(名称)
                    raster_band.SetDescription(names[i])

                # 写入数据
                raster_band.WriteArray(bands[i])
            print("save image success.")
            return True


if __name__ == '__main__':
    input_img = "F:\\blue\\Blue_201810.tif"
    output_img = "output.tif"

    # 读取影像
    band_data, param_geoTransform, param_proj, band_name = readImage(input_img)
    # 获得第一波段数据
    band_1 = band_data[0]

    # 归一化的范围
    target_max_v = 1.0
    target_min_v = 0.0
    # 原始影像灰度范围(不含无意义值)
    old_max_v = np.max(band_1)
    old_min_v = 0.0

    print 'original max value', np.max(band_1)
    print 'original min value', np.min(band_1)

    # 按照最大最小值法进行归一化，计算缩放比率
    scale_ratio = (target_max_v - target_min_v) / (old_max_v - old_min_v)

    # 对影像进行归一化，无意义值赋为0
    band_1_n = np.where(band_1 >= old_min_v, (band_1 - old_min_v) * scale_ratio, target_min_v)
    print 'normalized max value', np.max(band_1_n)
    print 'normalized min value', np.min(band_1_n)

    # 输出影像
    writeImage(output_img, [band_1_n], param_geoTransform, param_proj, band_name)
